/*----------------------------------------------------------------------------*/
/* VEX Override — Competition Ready Driver Program                           */
/* Platform: codev5.vex.com V5 C++ (Competition Template)                    */
/* Paste: replace entire main.cpp in a Competition Template project          */
/*----------------------------------------------------------------------------*/
/* BOT MAP — 88W legal                                                       */
/*  Drive 44W: Left 14,15 | Right 8,9 reversed | blue 600rpm                 */
/*           36T motor -> 48T wheel = ~450rpm on 2.75in wheels               */
/*  Lift 22W: 12,20 blue 600rpm winch (string on axle), 3-stage cascade      */
/*           Front banche claw rides lift. Encoder-only, no sensors.         */
/*  Intake 11W: port 10, 3 flywheels 12in long, low at rear chassis          */
/*  Tilt 5.5W: port 11, 12:60 gearing. 450 motor deg = 90 claw deg           */
/*           Vertical 0 = intake-aligned (receive). Horizontal 450 = stack   */
/*  Grip 5.5W: port 18, claw rollers. Forward grips, reverse releases.       */
/*----------------------------------------------------------------------------*/
/* SINGLE-CONTROLLER MAP                                                     */
/*  Axis3 + Axis1 : split arcade drive (both REVERSED per driver feel)      */
/*  Tank mode    : Axis3 left + Axis2 right (both reversed)                 */
/*  X + B together (tap) : toggle TANK / ARCADE                             */
/*  R1 / R2       : lift up 65% / down 50% (manual, wins over presets)       */
/*  Up / Right / Down : lift presets HIGH 1650 / MID 900 / REST 0            */
/*  Down + A (hold) : re-zero lift at bottom only                            */
/*  L1 / L2       : intake in 100% / out 80% (hold)                          */
/*  Y             : grip latch toggle 35% (REVERSED motor dir, see Grip)     */
/*  X / B         : tilt vertical 0 / horizontal 450 (tap one, not both)     */
/*  A (tap)       : auto handoff macro (vertical -> feed -> horizontal)       */
/*  Left (tap)    : timed grip release (reverse 80% 450ms) + clear lift fault */
/*  Left (hold 1s): e-stop manipulators + clear lift desync fault             */
/*----------------------------------------------------------------------------*/
/* TUNE FIRST (in order)                                                     */
/*  1. Lift R1 dies after ~1s = desync fault. Fix: flip LiftB reverse below, */
/*     then tap Left to clear. Tolerance now 250deg + R2-down recovery.      */
/*  2. If tilt B goes wrong way, flip Tilt reverse flag below                */
/*  3. Grip was reversed per driver: Grip motor flag is now true. If still   */
/*     backwards, flip it back to false.                                     */
/*  4. Start fully lowered. Raise LIFT_MAX_DEG in 100deg steps.              */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <algorithm>

using namespace vex;

// ================= Devices =================
brain Brain;
controller Controller1 = controller(controllerType::primary);
competition Competition;

// Drive — 4x 11W blue. Right side mirrored.
motor LeftFront  = motor(PORT14, gearSetting::ratio6_1, false);
motor LeftRear   = motor(PORT15, gearSetting::ratio6_1, false);
motor RightFront = motor(PORT8,  gearSetting::ratio6_1, true);
motor RightRear  = motor(PORT9,  gearSetting::ratio6_1, true);

// Lift — 2x 11W blue winch. Mirrored on shared spool.
// If motors fight (grind / instant desync), flip LiftB to false.
motor LiftA = motor(PORT12, gearSetting::ratio6_1, false);
motor LiftB = motor(PORT20, gearSetting::ratio6_1, true);

// Intake — 1x 11W. 5.5W claw motors use same API, wattage set in ports UI.
motor Intake = motor(PORT10, gearSetting::ratio18_1, false);
motor Tilt   = motor(PORT11, gearSetting::ratio36_1, false);
// Grip REVERSED per driver test (was false). Flip back if grip runs backwards.
motor Grip   = motor(PORT18, gearSetting::ratio18_1, true);

// ================= Tunables =================
// --- Drive (fast 450rpm needs restraint) ---
const int    DRIVE_DEADBAND   = 7;
const double DRIVE_EXPO       = 1.6;
const double DRIVE_TURN_SCALE = 0.85;
const int    DRIVE_MAX_PCT    = 85;
const int    DRIVE_SLEW       = 10; // % per 20ms

// --- Lift (600rpm winch: never full speed) ---
const int    LIFT_UP_PCT     = 65;
const int    LIFT_DOWN_PCT   = 50;
const int    LIFT_PRESET_PCT = 60;
const int    LIFT_SLEW       = 15;
const double LIFT_MIN_DEG    = 0.0;
const double LIFT_MAX_DEG    = 1800.0; // tune upward in 100deg steps
const double LIFT_SLOW_ZONE  = 300.0;
const int    LIFT_SLOW_PCT   = 25;
const double LIFT_DESYNC_DEG = 250.0;
const double LIFT_REST       = 0.0;
const double LIFT_MID        = 900.0;
const double LIFT_HIGH       = 1650.0;

// --- Intake / Tilt / Grip ---
const int    INTAKE_IN_PCT   = 100;
const int    INTAKE_OUT_PCT  = -80;
const double TILT_VERT_DEG   = 0.0;
const double TILT_HORIZ_DEG  = 450.0; // 12:60 = 5:1, 450 motor = 90 claw
const int    TILT_SPEED_PCT  = 50;
const int    GRIP_HOLD_PCT   = 35;
const int    GRIP_IN_PCT     = 100;
const int    GRIP_OUT_PCT    = -80;
const int    RELEASE_MS      = 450;

// ================= State =================
int  driveLeftCmd  = 0;
int  driveRightCmd = 0;
bool driveTankMode = false; // false = arcade Axis3+Axis1, true = tank Axis3+Axis2
bool prevTankChord = false;
int  liftCmd       = 0;
bool liftPresetOn  = false;
double liftPresetTarget = 0.0;
bool liftFault     = false;

bool gripLatched   = false;
bool releasing     = false;
double releaseEndMs = 0.0;

bool macroOn       = false;
int  macroStep     = 0;
double macroStepEndMs = 0.0;

int  jamTimerMs    = 0;
int  intakeRunMs   = 0;

bool prevUp = false, prevRight = false, prevDown = false;
bool prevA = false, prevLeft = false, prevY = false;
bool prevX = false, prevB = false;
double leftHoldMs = 0.0;

// ================= Helpers =================
int applyDeadband(int v, int db) {
  if (std::abs(v) < db) return 0;
  int sign = (v > 0) ? 1 : -1;
  return sign * (std::abs(v) - db) * 100 / (100 - db);
}

double applyExpo(double pct100, double exp) {
  double s = (pct100 >= 0) ? 1.0 : -1.0;
  return s * std::pow(std::fabs(pct100) / 100.0, exp) * 100.0;
}

int slewToward(int cur, int target, int step) {
  int d = target - cur;
  if (d > step) return cur + step;
  if (d < -step) return cur - step;
  return target;
}

double nowMs() { return Brain.timer(timeUnits::msec); }

// ================= Drive =================
void driveInit() {
  LeftFront.setStopping(brakeType::brake);
  LeftRear.setStopping(brakeType::brake);
  RightFront.setStopping(brakeType::brake);
  RightRear.setStopping(brakeType::brake);
  LeftFront.setMaxTorque(85, percentUnits::pct);
  LeftRear.setMaxTorque(85, percentUnits::pct);
  RightFront.setMaxTorque(85, percentUnits::pct);
  RightRear.setMaxTorque(85, percentUnits::pct);
}

void driveUpdate(double liftHeightDeg, bool liftBusy) {
  // Tank/arcade toggle: X + B together (tap). Handled here so it works
  // even if tiltUpdate also sees the buttons. Suppresses tilt that loop.
  bool chordNow = Controller1.ButtonX.pressing() && Controller1.ButtonB.pressing();
  if (chordNow && !prevTankChord) {
    driveTankMode = !driveTankMode;
    Controller1.rumble(driveTankMode ? "--" : ".");
    Brain.Screen.print(driveTankMode ? "TANK mode" : "ARCADE mode");
    Brain.Screen.newLine();
  }
  prevTankChord = chordNow;
  if (chordNow) return; // skip drive + tilt this loop, toggle only

  int leftTarget = 0, rightTarget = 0;
  if (driveTankMode) {
    // Tank: left stick Y + right stick Y, both reversed per driver feel.
    int leftRaw  = -Controller1.Axis3.position();
    int rightRaw = -Controller1.Axis2.position();
    int left  = applyDeadband(leftRaw, DRIVE_DEADBAND);
    int right = applyDeadband(rightRaw, DRIVE_DEADBAND);
    leftTarget  = (int)std::round(applyExpo(left, DRIVE_EXPO));
    rightTarget = (int)std::round(applyExpo(right, DRIVE_EXPO));
  } else {
    // Arcade: Axis3 fwd + Axis1 turn, both reversed per driver feel.
    int fwd  = applyDeadband(-Controller1.Axis3.position(), DRIVE_DEADBAND);
    int turn = applyDeadband(-Controller1.Axis1.position(), DRIVE_DEADBAND);

    double fwdE  = applyExpo(fwd, DRIVE_EXPO);
    double turnE = applyExpo(turn, DRIVE_EXPO) * DRIVE_TURN_SCALE;

    leftTarget  = (int)std::round(fwdE + turnE);
    rightTarget = (int)std::round(fwdE - turnE);
  }

  // Brownout + tip guards: cap drive when lift is busy or high.
  int cap = DRIVE_MAX_PCT;
  if (liftHeightDeg > LIFT_MID) cap = 50;       // high cascade = slow drive
  else if (liftBusy) cap = 70;                  // shared battery budget

  leftTarget  = std::max(-cap, std::min(cap, leftTarget));
  rightTarget = std::max(-cap, std::min(cap, rightTarget));

  driveLeftCmd  = slewToward(driveLeftCmd, leftTarget, DRIVE_SLEW);
  driveRightCmd = slewToward(driveRightCmd, rightTarget, DRIVE_SLEW);

  if (driveLeftCmd == 0 && driveRightCmd == 0) {
    LeftFront.stop(brakeType::brake);
    LeftRear.stop(brakeType::brake);
    RightFront.stop(brakeType::brake);
    RightRear.stop(brakeType::brake);
  } else {
    LeftFront.spin(directionType::fwd, driveLeftCmd, velocityUnits::pct);
    LeftRear.spin(directionType::fwd, driveLeftCmd, velocityUnits::pct);
    RightFront.spin(directionType::fwd, driveRightCmd, velocityUnits::pct);
    RightRear.spin(directionType::fwd, driveRightCmd, velocityUnits::pct);
  }
}

// ================= Lift =================
double liftHeight() {
  return (LiftA.position(rotationUnits::deg) +
          LiftB.position(rotationUnits::deg)) * 0.5;
}

bool liftDesynced() {
  return std::fabs(LiftA.position(rotationUnits::deg) -
                   LiftB.position(rotationUnits::deg)) > LIFT_DESYNC_DEG;
}

void liftHold() {
  LiftA.stop(brakeType::hold);
  LiftB.stop(brakeType::hold);
}

void liftTare() {
  LiftA.setPosition(0.0, rotationUnits::deg);
  LiftB.setPosition(0.0, rotationUnits::deg);
}

void liftInit() {
  LiftA.setStopping(brakeType::hold);
  LiftB.setStopping(brakeType::hold);
  LiftA.setMaxTorque(90, percentUnits::pct);
  LiftB.setMaxTorque(90, percentUnits::pct);
  liftTare();
}

void liftUpdate() {
  double h = liftHeight();

  // Fault: motors disagree (twisted cascade / snapped string risk).
  // #1 cause of "R1 works 1s then lift dead" is LiftB reversed wrong
  // (motors fight -> positions diverge -> fault latches). Tap Left to clear,
  // flip LiftB flag, test again. R2-down still allowed to unwind/recover.
  if (liftDesynced()) {
    if (!liftFault) {
      liftFault = true;
      liftPresetOn = false;
      liftCmd = 0;
      Controller1.rumble("---");
      Brain.Screen.print("LIFT DESYNC A=%.0f B=%.0f flip LiftB?",
        LiftA.position(rotationUnits::deg), LiftB.position(rotationUnits::deg));
      Brain.Screen.newLine();
    }
    // Recovery: allow slow unwind with R2 even while faulted.
    if (Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()) {
      liftCmd = slewToward(liftCmd, -LIFT_SLOW_PCT, LIFT_SLEW);
      LiftA.spin(directionType::fwd, liftCmd, velocityUnits::pct);
      LiftB.spin(directionType::fwd, liftCmd, velocityUnits::pct);
    } else {
      liftHold();
    }
    return; // cleared by Left tap/hold in buttonsUpdate
  }

  bool r1 = Controller1.ButtonR1.pressing();
  bool r2 = Controller1.ButtonR2.pressing();
  bool manual = r1 || r2;
  if (manual) liftPresetOn = false;

  // Preset taps.
  bool upNow = Controller1.ButtonUp.pressing();
  bool rightNow = Controller1.ButtonRight.pressing();
  bool downNow = Controller1.ButtonDown.pressing();
  if (!manual && !liftFault) {
    if (upNow && !prevUp)         { liftPresetOn = true; liftPresetTarget = LIFT_HIGH; }
    else if (rightNow && !prevRight) { liftPresetOn = true; liftPresetTarget = LIFT_MID; }
    else if (downNow && !prevDown && !Controller1.ButtonA.pressing()) {
      liftPresetOn = true; liftPresetTarget = LIFT_REST;
    }
  }
  prevUp = upNow; prevRight = rightNow; prevDown = downNow;
  liftPresetTarget = std::max(LIFT_MIN_DEG, std::min(LIFT_MAX_DEG, liftPresetTarget));

  // Re-zero: Down + A together, only at bottom.
  bool aNow = Controller1.ButtonA.pressing();
  if (downNow && aNow && !prevA) {
    liftTare();
    liftPresetOn = false;
    Controller1.rumble(".");
  }
  prevA = aNow;

  // Desired command before safety.
  int want = 0;
  if (manual) {
    want = r1 ? LIFT_UP_PCT : -LIFT_DOWN_PCT;
  } else if (liftPresetOn) {
    double err = liftPresetTarget - h;
    if (std::fabs(err) < 25.0) liftPresetOn = false;
    else want = (err > 0) ? LIFT_PRESET_PCT : -LIFT_PRESET_PCT;
  }

  // Hard clamp past limits.
  if ((h >= LIFT_MAX_DEG - 2.0 && want > 0) ||
      (h <= LIFT_MIN_DEG + 2.0 && want < 0)) {
    want = 0;
    liftPresetOn = false;
  }

  // Slow zone: no full-speed slam at 600rpm.
  if (want > 0 && h > LIFT_MAX_DEG - LIFT_SLOW_ZONE) want = std::min(want, LIFT_SLOW_PCT);
  if (want < 0 && h < LIFT_MIN_DEG + LIFT_SLOW_ZONE) want = std::max(want, -LIFT_SLOW_PCT);

  liftCmd = slewToward(liftCmd, want, LIFT_SLEW);

  if (liftCmd == 0) liftHold(); // anti-unwind: winch backdrives without hold
  else {
    LiftA.spin(directionType::fwd, liftCmd, velocityUnits::pct);
    LiftB.spin(directionType::fwd, liftCmd, velocityUnits::pct);
  }
}

bool liftIsBusy() {
  return Controller1.ButtonR1.pressing() ||
         Controller1.ButtonR2.pressing() || liftPresetOn;
}

void liftClearFault() {
  liftFault = false;
  liftCmd = 0;
  liftHold();
  Brain.Screen.print("Lift fault cleared. Re-zero with Down+A.");
  Brain.Screen.newLine();
}

// ================= Intake =================
void intakeInit() {
  Intake.setStopping(brakeType::coast);
  Intake.setMaxTorque(100, percentUnits::pct);
}

void intakeStop() {
  Intake.stop(brakeType::coast);
  jamTimerMs = 0;
}

void intakeUpdate(bool macroDriving, int macroIntakePct) {
  // Macro owns intake while handoff runs.
  if (macroDriving) {
    Intake.spin(directionType::fwd, macroIntakePct, velocityUnits::pct);
    return;
  }
  // Release owns the pause: intake off while grip reverses.
  if (releasing) {
    intakeStop();
    return;
  }

  bool in  = Controller1.ButtonL1.pressing();
  bool out = Controller1.ButtonL2.pressing();
  int cmd = 0;
  if (in && out) cmd = 0; // both = safety stop
  else if (in) cmd = INTAKE_IN_PCT;
  else if (out) cmd = INTAKE_OUT_PCT;

  if (cmd == 0) {
    intakeStop();
    intakeRunMs = 0;
    return;
  }

  Intake.spin(directionType::fwd, cmd, velocityUnits::pct);
  intakeRunMs += 20;

  // Anti-jam: commanded forward but stalled (11W jams on pins).
  if (cmd > 0 &&
      std::fabs(Intake.velocity(velocityUnits::pct)) < 10.0 &&
      Intake.torque(torqueUnits::Nm) > 0.4) {
    jamTimerMs += 20;
    if (jamTimerMs > 250) {
      Intake.spin(directionType::rev, 80, velocityUnits::pct);
      wait(200, timeUnits::msec);
      jamTimerMs = 0;
      Controller1.rumble("-");
    }
  } else {
    jamTimerMs = 0;
  }

  // Possession guard: running intake >4s straight risks 1+1 violation.
  if (intakeRunMs > 4000 && intakeRunMs < 4020) Controller1.rumble("--");
}

// ================= Tilt =================
void tiltInit() {
  Tilt.setStopping(brakeType::hold);
  Tilt.setTimeout(1500, timeUnits::msec);
  Tilt.setMaxTorque(100, percentUnits::pct);
  Tilt.setPosition(0.0, rotationUnits::deg); // zero = vertical at start
}

void tiltTo(double deg) {
  deg = std::max(-30.0, std::min(480.0, deg)); // hard clamp protects 5.5W
  Tilt.setVelocity(TILT_SPEED_PCT, velocityUnits::pct);
  Tilt.spinToPosition(deg, rotationUnits::deg, false);
}

void tiltUpdate() {
  // X+B together = drive-mode toggle (handled in driveUpdate). Ignore tilt.
  if (Controller1.ButtonX.pressing() && Controller1.ButtonB.pressing()) {
    prevX = true; prevB = true;
    if (!Tilt.isSpinning()) Tilt.stop(brakeType::hold);
    return;
  }
  bool xNow = Controller1.ButtonX.pressing();
  bool bNow = Controller1.ButtonB.pressing();

  // Manual jog on Up/Down d-pad already used by lift, so use X/B presets.
  // Any preset cancels macro tilt step.
  if (xNow && !prevX) { tiltTo(TILT_VERT_DEG);  macroOn = false; }
  if (bNow && !prevB) { tiltTo(TILT_HORIZ_DEG); macroOn = false; }
  prevX = xNow; prevB = bNow;

  if (!Tilt.isSpinning()) Tilt.stop(brakeType::hold);
}

// ================= Grip + Release =================
void gripInit() {
  Grip.setStopping(brakeType::hold);
  Grip.setMaxTorque(100, percentUnits::pct);
}

void gripUpdate() {
  // Timed release in progress.
  if (releasing) {
    if (nowMs() >= releaseEndMs) {
      releasing = false;
      gripLatched = false;
      Grip.stop(brakeType::hold);
    } else {
      return; // keep reversing, ignore latch during release
    }
  }

  bool yNow = Controller1.ButtonY.pressing();
  if (yNow && !prevY) {
    gripLatched = !gripLatched; // toggle hold
    if (!gripLatched) Grip.stop(brakeType::hold);
  }
  prevY = yNow;

  if (gripLatched) Grip.spin(directionType::fwd, GRIP_HOLD_PCT, velocityUnits::pct);
  else if (!Grip.isSpinning()) Grip.stop(brakeType::hold);
}

void startRelease() {
  releasing = true;
  gripLatched = false;
  macroOn = false;
  releaseEndMs = nowMs() + RELEASE_MS;
  Grip.spin(directionType::rev, 80, velocityUnits::pct);
}

// ================= Handoff Macro =================
// A tap: vertical -> feed 800ms -> horizontal + latch. Any L1/L2/X/B/Y aborts.
void macroUpdate() {
  bool aTap = Controller1.ButtonA.pressing() && !prevA;
  // Note: prevA is updated in liftUpdate; re-read edge here for macro start.
  // To keep it simple, start macro on A press when not holding Down (re-zero).
  if (aTap && !Controller1.ButtonDown.pressing() && !macroOn && !releasing) {
    macroOn = true;
    macroStep = 0;
    macroStepEndMs = nowMs();
  }

  // Abort on any manual manipulator input.
  if (macroOn &&
      (Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing() ||
       Controller1.ButtonX.pressing() || Controller1.ButtonB.pressing() ||
       Controller1.ButtonY.pressing())) {
    macroOn = false;
    intakeStop();
  }

  if (!macroOn) return;

  if (macroStep == 0) {
    tiltTo(TILT_VERT_DEG);
    gripLatched = false;
    macroStep = 1;
    macroStepEndMs = nowMs() + 300;
  } else if (macroStep == 1) {
    Intake.spin(directionType::fwd, INTAKE_IN_PCT, velocityUnits::pct);
    Grip.spin(directionType::fwd, GRIP_IN_PCT, velocityUnits::pct);
    if (nowMs() >= macroStepEndMs) {
      macroStep = 2;
      macroStepEndMs = nowMs() + 800; // seat time, no sensors
    }
  } else if (macroStep == 2) {
    Intake.spin(directionType::fwd, INTAKE_IN_PCT, velocityUnits::pct);
    Grip.spin(directionType::fwd, GRIP_IN_PCT, velocityUnits::pct);
    if (nowMs() >= macroStepEndMs) {
      intakeStop();
      tiltTo(TILT_HORIZ_DEG);
      gripLatched = true; // hold through lift
      macroOn = false;
      Controller1.rumble(".");
    }
  }
}

// ================= E-stop / Release buttons =================
void buttonsUpdate() {
  // Left tap = clear lift fault if faulted, else timed release.
  // Left hold 1s = e-stop + clear lift fault.
  bool leftNow = Controller1.ButtonLeft.pressing();
  if (leftNow && !prevLeft) leftHoldMs = nowMs(); // press start

  if (!leftNow && prevLeft) {
    // released: tap (<800ms)
    if (nowMs() - leftHoldMs < 800) {
      if (liftFault) liftClearFault();
      else startRelease();
    }
    leftHoldMs = 0;
  }
  if (leftNow && leftHoldMs != 0 && nowMs() - leftHoldMs > 1000) {
    // hold 1s = e-stop
    macroOn = false;
    releasing = false;
    gripLatched = false;
    Intake.stop(brakeType::coast);
    Grip.stop(brakeType::hold);
    Tilt.stop(brakeType::hold);
    if (liftFault) liftClearFault();
    Controller1.rumble("---");
    leftHoldMs = 0; // fire once per hold
  }
  prevLeft = leftNow;
}

// ================= Telemetry =================
void telemetryUpdate(int tick) {
  if (tick % 5 != 0) return; // ~100ms
  Controller1.Screen.setCursor(1, 1);
  // D = drive mode, F = lift fault flag
  Controller1.Screen.print("%s Lf:%4.0f T:%3.0f G:%d%s",
    driveTankMode ? "TNK" : "ARC",
    liftHeight(), Tilt.position(rotationUnits::deg),
    gripLatched ? 1 : 0, liftFault ? " FLT" : "");
  if (Brain.Battery.capacity() < 20) Controller1.rumble("-");
}

// ================= Competition Template =================
// pre_auton runs once at power-on. Zero lift at bottom, set brake modes.
void pre_auton(void) {
  driveInit();
  liftInit();
  intakeInit();
  tiltInit();
  gripInit();
  Brain.Screen.print("Ready. Start fully lowered.");
  Brain.Screen.newLine();
}

// autonomous: hold lift/tilt so winch does not unwind. No sensors.
void autonomous(void) {
  liftHold();
  Tilt.stop(brakeType::hold);
  Grip.stop(brakeType::hold);
}

// usercontrol: driver only. Drive updated every 20ms, never blocked.
void usercontrol(void) {
  driveLeftCmd = driveRightCmd = liftCmd = 0;
  driveTankMode = false;
  prevTankChord = false;
  liftPresetOn = false;
  liftFault = false;
  gripLatched = false;
  releasing = false;
  macroOn = false;
  int tick = 0;

  while (true) {
    macroUpdate();                   // A handoff first (edge before liftUpdate)
    liftUpdate();                    // R1/R2 + presets + limits + desync
    intakeUpdate(macroOn && macroStep >= 1, INTAKE_IN_PCT);
    gripUpdate();                    // Y latch + timed release
    tiltUpdate();                    // X/B presets + hold (X+B = drive toggle)
    buttonsUpdate();                 // Left tap release / clear fault, hold e-stop
    driveUpdate(liftHeight(), liftIsBusy());
    telemetryUpdate(++tick);
    wait(20, timeUnits::msec);
  }
}

// Leave this main() in place for field control.
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) wait(100, timeUnits::msec);
}
