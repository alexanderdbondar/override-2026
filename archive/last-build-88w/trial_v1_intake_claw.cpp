/*----------------------------------------------------------------------------*/
/* Trial V1 — Intake + Banshee Claw driver (paste-ready for codev5.vex.com)  */
/* Bot: intake rear low (3x flywheel, 12in) + front claw on lift              */
/*----------------------------------------------------------------------------*/
/* PORTS (do not change without checking wattage):                            */
/*   Intake  PORT 10  11W  (flywheels, low at back)                           */
/*   Tilt    PORT 11  5.5W 12:60 external (450 motor deg = 90 claw deg)        */
/*   Grip    PORT 18  5.5W (spin fwd = seat/hold, rev = release)               */
/* RESERVED — handled by other agent, DO NOT TOUCH in this file:              */
/*   Drive L PORT 14/15, Drive R PORT 8/9  on Axis3/Axis1                     */
/*   Lift    PORT 12/20                on R1/R2                               */
/* Wattage: 11 + 5.5 + 5.5 = 22W for these 3. Rest of budget for drive/lift.  */
/* Cycle: tilt vertical(0) -> intake feeds bottom -> grip seats -> tilt       */
/*        horizontal(450) -> lift up (other agent) -> grip reverse to score.  */
/*----------------------------------------------------------------------------*/
/* CONTROLLER MAP (single controller, avoids Axis3/Axis1 + R1/R2):            */
/*   L1 hold ........ Intake IN (feed to claw)                                */
/*   L2 hold ........ Intake OUT / reject (clear jam, avoid 2nd object)       */
/*   X press ........ Tilt preset VERTICAL 0 (intake-aligned, receive)        */
/*   B press ........ Tilt preset HORIZONTAL 450 (stack, score-ready)         */
/*   Up / Down hold . Manual tilt jog (fine align, cancels preset)            */
/*   Y press ........ Toggle grip latch (retain at low power, no hold needed) */
/*   Right press .... Timed grip RELEASE (reverse pulse, then stop+unlatch)   */
/*   A press ........ Auto-grab HANDOFF macro (interruptible, see below)      */
/*   Left press ..... E-STOP manipulators (cancel macro, stop intake/grip)    */
/* Handoff macro (A): vertical -> intake+grip seat -> tilt horizontal ->      */
/*   latch. ABORT by pressing L1/L2/X/B/Y/Right/Left/A again.                 */
/* Possession safety (1 pin + 1 cup max): macro auto-stops intake after seat; */
/*   intake over-run rumble after 4 s; L2 to spit extra object; Left E-stop.  */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

// ---- Devices (only ours — drive/lift defined in other agent's file) ----
brain         Brain;
controller    Controller1 = controller(primary);
competition   Competition;

motor Intake = motor(PORT10, gearSetting::ratio6_1,  false); // 11W flywheel, blue
motor Tilt   = motor(PORT11, gearSetting::ratio36_1, false); // 5.5W, red for torque+hold
motor Grip   = motor(PORT18, gearSetting::ratio18_1, false); // 5.5W grip wheels

// If a mechanism runs backwards, flip ONLY its third arg above to true.
// Do not swap ports.

// ---- Tunables ----
const int INTAKE_IN_PCT      = 100;  // L1
const int INTAKE_OUT_PCT     = 80;   // L2 (reject)
const int INTAKE_SEAT_PCT    = 100;  // macro seating power
const int JAM_REV_PCT        = 80;   // anti-jam pulse power
const int JAM_STALL_MSEC     = 250;  // velocity<10% this long => jam
const int JAM_REV_MSEC       = 200;  // pulse duration
const double JAM_TORQUE_NM   = 0.40; // + torque gate so light load isn't "jam"

const int TILT_PRESET_VEL    = 50;   // % — keep low to protect 5.5W
const int TILT_JOG_VEL       = 30;   // Up/Down manual
const double TILT_VERTICAL   = 0.0;  // motor deg = claw vertical (intake-aligned)
const double TILT_HORIZONTAL = 450.0;// motor deg = claw horizontal (stack)
const int TILT_TIMEOUT_MSEC  = 1500; // preset timeout protects 5.5W

const int GRIP_IN_PCT        = 100;  // seating power
const int GRIP_LATCH_PCT     = 30;   // retain power (holds pin/cup, low heat)
const int GRIP_RELEASE_PCT   = 80;   // reverse release power
const int GRIP_RELEASE_MSEC  = 450;  // timed release duration

const int INTAKE_WARN_MSEC   = 4000; // rumble reminder: possible 2nd object

// ---- State ----
static bool gripLatched = false;
static bool tiltBusy = false;
static double tiltTarget = TILT_VERTICAL;
static uint32_t tiltStartMs = 0;
static bool macroRunning = false;
static uint32_t releaseEndMs = 0;   // non-zero = timed release active
static uint32_t intakeRunMs = 0;    // accumulated continuous run (warn timer)
static uint32_t jamStallStart = 0;
static bool jamPulsing = false;
static bool intakeWarned = false;

// Edge-detect helpers for press (not hold) buttons
static bool prevX=false, prevB=false, prevY=false, prevA=false,
            prevRight=false, prevLeft=false;

static uint32_t nowMs() { return Brain.Timer.time(timeUnits::msec); }

// Any manipulator button => aborts handoff macro
static bool abortRequested() {
  return Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing() ||
         Controller1.ButtonX.pressing()  || Controller1.ButtonB.pressing() ||
         Controller1.ButtonY.pressing()  || Controller1.ButtonRight.pressing() ||
         Controller1.ButtonLeft.pressing() || Controller1.ButtonUp.pressing() ||
         Controller1.ButtonDown.pressing();
}
static bool abortEdge() { // A pressed again also aborts
  return abortRequested() || Controller1.ButtonA.pressing();
}

// Interruptible sleep: 20 ms chunks, returns false if aborted
static bool sleepBreakable(int msec) {
  int elapsed = 0;
  while (elapsed < msec) {
    if (abortRequested()) return false;
    task::sleep(20);
    elapsed += 20;
  }
  return true;
}

static void tiltGoto(double targetDeg) {
  tiltTarget = targetDeg;
  tiltStartMs = nowMs();
  tiltBusy = true;
  Tilt.setTimeout(TILT_TIMEOUT_MSEC, timeUnits::msec);
  Tilt.spinToPosition(targetDeg, rotationUnits::deg, TILT_PRESET_VEL,
                      velocityUnits::pct, false); // non-blocking: intake stays live
}

static void tiltUpdate() {
  // Manual jog cancels preset (driver takes over, protects 5.5W from fighting)
  if (Controller1.ButtonUp.pressing() || Controller1.ButtonDown.pressing()) {
    tiltBusy = false;
    Tilt.setTimeout(0, timeUnits::msec); // cancel preset timeout
    Tilt.stop(brakeType::hold);
    if (Controller1.ButtonUp.pressing())
      Tilt.spin(directionType::fwd, TILT_JOG_VEL, velocityUnits::pct);
    else
      Tilt.spin(directionType::rev, TILT_JOG_VEL, velocityUnits::pct);
    return;
  }
  if (tiltBusy) {
    bool done = !Tilt.isSpinning();
    bool timedOut = (nowMs() - tiltStartMs) > (uint32_t)(TILT_TIMEOUT_MSEC + 200);
    // Hard clamp: never drive past ends (protects 12:60 + 5.5W)
    double p = Tilt.position(rotationUnits::deg);
    if (p < -30 || p > 480) { Tilt.stop(brakeType::hold); tiltBusy = false; return; }
    if (done || timedOut) {
      Tilt.stop(brakeType::hold); // hold position against gravity/back-drive
      tiltBusy = false;
      Controller1.rumble(".");
    }
  } else {
    // Not jogging, not preset: ensure hold (claw must not droop mid-cycle)
    if (!Tilt.isSpinning() && !Controller1.ButtonUp.pressing() &&
        !Controller1.ButtonDown.pressing()) {
      // Only enforce once — stop(hold) is sticky, no need to call every loop.
      // Checked via velocity to avoid re-issuing: cheap guard:
      static bool holding = false;
      if (!holding) { Tilt.stop(brakeType::hold); holding = true; }
    }
  }
}

// Returns intake command: +1 fwd, -1 rev, 0 off (before anti-jam)
static int intakeCommand() {
  if (macroRunning) return 0; // macro drives intake directly
  if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing())
    return 0; // both = safety stop (avoids L1+L2 fight)
  if (Controller1.ButtonL1.pressing()) return +1;
  if (Controller1.ButtonL2.pressing()) return -1;
  return 0;
}

static void intakeUpdate() {
  if (macroRunning || jamPulsing) return; // macro / jam pulse owns intake

  // Timed-release owns grip, not intake — intake still driver-controlled here.
  int cmd = intakeCommand();
  static int lastCmd = 0;

  if (cmd > 0) {
    Intake.spin(directionType::fwd, INTAKE_IN_PCT, velocityUnits::pct);
    // Anti-jam: commanded ON but stalled (vel<10%) + torque high
    double v = Intake.velocity(velocityUnits::pct); // -100..100
    double t = Intake.torque(torqueUnits::Nm);
    if (fabs(v) < 10.0 && t > JAM_TORQUE_NM) {
      if (jamStallStart == 0) jamStallStart = nowMs();
      if (nowMs() - jamStallStart > (uint32_t)JAM_STALL_MSEC) {
        jamPulsing = true;
        Intake.spin(directionType::rev, JAM_REV_PCT, velocityUnits::pct);
        Controller1.rumble("--");
        task::sleep(JAM_REV_MSEC); // brief; tilt/grip hold through it
        Intake.spin(directionType::fwd, INTAKE_IN_PCT, velocityUnits::pct);
        jamStallStart = 0;
        jamPulsing = false;
      }
    } else {
      jamStallStart = 0;
    }
    // Possession-limit feel: rumble if intake held too long (may have 2nd obj)
    if (lastCmd <= 0) { intakeRunMs = nowMs(); intakeWarned = false; }
    if (!intakeWarned && nowMs() - intakeRunMs > (uint32_t)INTAKE_WARN_MSEC) {
      Controller1.rumble(".-");
      intakeWarned = true; // once per continuous hold
    }
  } else if (cmd < 0) {
    Intake.spin(directionType::rev, INTAKE_OUT_PCT, velocityUnits::pct);
    jamStallStart = 0;
    intakeRunMs = nowMs(); intakeWarned = false;
  } else {
    Intake.stop(brakeType::coast); // flywheels coast: less heat, less drag
    jamStallStart = 0;
    intakeWarned = false;
  }
  lastCmd = cmd;
}

static void gripUpdate() {
  uint32_t now = nowMs();
  // Timed release in progress?
  if (releaseEndMs != 0) {
    if (now >= releaseEndMs) {
      releaseEndMs = 0;
      gripLatched = false;
      Grip.stop(brakeType::hold);
      Controller1.rumble(".");
    }
    return; // release owns grip until done
  }
  if (macroRunning) return; // macro owns grip

  // Latch holds at low power so driver doesn't have to hold a button.
  // Manual momentary seat: hold Right? No — Right is release. Latch covers it.
  if (gripLatched) {
    Grip.spin(directionType::fwd, GRIP_LATCH_PCT, velocityUnits::pct);
  } else {
    // Not latched: full stop with hold so seated object doesn't slip
    static bool stopped = false;
    if (!stopped) { Grip.stop(brakeType::hold); stopped = true; }
    // reset guard when latched again
    if (gripLatched) stopped = false;
  }
}

// Y rising-edge: toggle latch. Right rising-edge: timed reverse release.
static void gripButtonsUpdate() {
  bool y = Controller1.ButtonY.pressing();
  bool r = Controller1.ButtonRight.pressing();
  if (y && !prevY && !macroRunning && releaseEndMs == 0) {
    gripLatched = !gripLatched;
    if (gripLatched) Grip.spin(directionType::fwd, GRIP_LATCH_PCT, velocityUnits::pct);
    else Grip.stop(brakeType::hold);
    Controller1.rumble(gripLatched ? "-" : ".");
  }
  if (r && !prevRight && !macroRunning) {
    gripLatched = false;
    Grip.spin(directionType::rev, GRIP_RELEASE_PCT, velocityUnits::pct);
    releaseEndMs = nowMs() + GRIP_RELEASE_MSEC;
    Controller1.rumble("..");
  }
  prevY = y; prevRight = r;
}

static void estopAll(const char *why) {
  (void)why;
  macroRunning = false;
  jamPulsing = false;
  releaseEndMs = 0;
  tiltBusy = false;
  gripLatched = false;
  Tilt.setTimeout(0, timeUnits::msec);
  Tilt.stop(brakeType::hold);
  Grip.stop(brakeType::hold);
  Intake.stop(brakeType::coast);
  Controller1.rumble("---");
}

// Handoff macro (A): assumes object is at claw mouth / intake throat.
// Steps: vertical -> seat (intake+grip) -> horizontal -> latch. Interruptible.
static void runHandoffMacro() {
  macroRunning = true;
  gripLatched = false;
  releaseEndMs = 0;

  // 1) Tilt vertical (receive). Non-blocking start, then breakable wait.
  tiltGoto(TILT_VERTICAL);
  {
    uint32_t t0 = nowMs();
    while (tiltBusy && nowMs() - t0 < (uint32_t)(TILT_TIMEOUT_MSEC + 200)) {
      if (abortEdge()) { estopAll("abort"); return; }
      tiltUpdate();
      task::sleep(20);
    }
    Tilt.stop(brakeType::hold); tiltBusy = false;
    if (abortEdge()) { estopAll("abort"); return; }
  }

  // 2) Seat: intake feeds bottom + grip spins to seat. Auto-stops intake after
  //    to respect 1 pin + 1 cup possession limit (no dragging a 2nd object).
  Grip.spin(directionType::fwd, GRIP_IN_PCT, velocityUnits::pct);
  Intake.spin(directionType::fwd, INTAKE_SEAT_PCT, velocityUnits::pct);
  if (!sleepBreakable(800)) { estopAll("abort"); return; } // seating window
  Intake.stop(brakeType::coast);

  // 3) Tilt horizontal (stack/score-ready)
  tiltGoto(TILT_HORIZONTAL);
  {
    uint32_t t0 = nowMs();
    Grip.spin(directionType::fwd, GRIP_LATCH_PCT, velocityUnits::pct); // retain
    while (tiltBusy && nowMs() - t0 < (uint32_t)(TILT_TIMEOUT_MSEC + 200)) {
      if (abortEdge()) { estopAll("abort"); return; }
      tiltUpdate();
      task::sleep(20);
    }
    Tilt.stop(brakeType::hold); tiltBusy = false;
    if (abortEdge()) { estopAll("abort"); return; }
  }

  // 4) Latch retain, done. Lift up + reverse-grip score handled by driver
  //    (lift on R1/R2 by other agent; release on Right).
  gripLatched = true;
  Grip.spin(directionType::fwd, GRIP_LATCH_PCT, velocityUnits::pct);
  macroRunning = false;
  Controller1.rumble("--");
}

static void buttonsUpdate() {
  bool x = Controller1.ButtonX.pressing();
  bool b = Controller1.ButtonB.pressing();
  bool a = Controller1.ButtonA.pressing();
  bool l = Controller1.ButtonLeft.pressing();

  if (x && !prevX && !macroRunning) tiltGoto(TILT_VERTICAL);
  if (b && !prevB && !macroRunning) tiltGoto(TILT_HORIZONTAL);
  if (l && !prevLeft) { estopAll("estop"); } // Left = E-stop, also cancels macro
  if (a && !prevA && !macroRunning) {
    // Enter macro; it polls abort internally. Rest of loop skipped while in it.
    runHandoffMacro();
  } else if (macroRunning && abortEdge()) {
    estopAll("abort");
  }

  prevX = x; prevB = b; prevA = a; prevLeft = l;
}

void pre_auton() {
  // Zero tilt at vertical BEFORE match: physically set claw vertical
  // (intake-aligned), then power on / run this. Zero = software vertical.
  Tilt.setPosition(TILT_VERTICAL, rotationUnits::deg);
  Tilt.setStopping(brakeType::hold);
  Tilt.setMaxTorque(100, percentUnits::pct);
  Tilt.setTimeout(0, timeUnits::msec);

  Grip.setPosition(0, rotationUnits::deg);
  Grip.setStopping(brakeType::hold);

  Intake.setStopping(brakeType::coast);

  Brain.Screen.clearScreen();
  Brain.Screen.print("V1 Intake+Claw ready. Zero=VERTICAL.");
}

void autonomous() {
  // Trial V1: driver-only. Keep manipulators safe during auto.
  Tilt.stop(brakeType::hold);
  Grip.stop(brakeType::hold);
  Intake.stop(brakeType::coast);
}

void usercontrol() {
  // NOTE: drive (Axis3/Axis1) + lift (R1/R2) owned by other agent's code.
  // This loop NEVER reads Axis3/Axis1/R1/R2 and never drives those motors.
  while (true) {
    buttonsUpdate();      // X/B presets, A macro, Left e-stop (may block in macro)
    if (!macroRunning) {
      gripButtonsUpdate();// Y latch, Right timed release
      intakeUpdate();     // L1/L2 hold-to-run + anti-jam + over-run rumble
      gripUpdate();       // latch hold / release timing
    }
    tiltUpdate();         // presets w/ timeout+hold, Up/Down jog
    task::sleep(20);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) { task::sleep(100); }
}
