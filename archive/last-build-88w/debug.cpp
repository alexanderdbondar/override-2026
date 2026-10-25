/*----------------------------------------------------------------------------*/
/* VEX Override — DEBUG build (paste over main.cpp in codev5 to diagnose)    */
/* Tracks: sticks/buttons, commanded %, motor deg/vel/torque/temp/current,   */
/* battery, desync, fault flags. Plus raw motor-test mode.                   */
/*----------------------------------------------------------------------------*/
/* DEBUG CONTROLS                                                             */
/*  Left / Right : change debug page (0=INPUTS, 1=MOTORS, 2=POWER, 3=TEST)    */
/*  TEST page: Up/Down = select motor, L1 = -30%, R1 = +30%, A = zero motor,  */
/*             B = zero lift+tilt, Y = toggle drive test (spin drive 30%)     */
/*  Pages 0-2: normal raw drive still live so you can see cmd vs actual:     */
/*    Axis3/1 arcade (reversed), R1/R2 lift 40%, L1/L2 intake, X/B tilt,      */
/*    Y grip latch, A release pulse. E-stop = hold Left 1s in any page? No —  */
/*    Left/Right reserved for paging, so E-stop = X+B together in debug.      */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace vex;

brain Brain;
controller Controller1 = controller(controllerType::primary);
competition Competition;

motor LeftFront  = motor(PORT14, gearSetting::ratio6_1, false);
motor LeftRear   = motor(PORT15, gearSetting::ratio6_1, false);
motor RightFront = motor(PORT8,  gearSetting::ratio6_1, true);
motor RightRear  = motor(PORT9,  gearSetting::ratio6_1, true);
motor LiftA = motor(PORT12, gearSetting::ratio6_1, false);
motor LiftB = motor(PORT20, gearSetting::ratio6_1, true);
motor Intake = motor(PORT10, gearSetting::ratio18_1, false);
motor Tilt   = motor(PORT11, gearSetting::ratio36_1, false);
motor Grip   = motor(PORT18, gearSetting::ratio18_1, true); // reversed per test

int page = 0; // 0..3
bool prevLeftDbg = false, prevRightDbg = false;
bool gripLatchedDbg = false, prevYDbg = false;
int testIdx = 0; // 0..8 motor under test
bool prevUpDbg = false, prevDownDbg = false, prevADbg = false, prevBDbg = false;

const char* motorNames[9] = {"LF14","LR15","RF8","RR9","LftA12","LftB20","Int10","Tlt11","Grp18"};
motor* motors[9] = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};

void dbgInitMotors() {
  motors[0]=&LeftFront; motors[1]=&LeftRear; motors[2]=&RightFront; motors[3]=&RightRear;
  motors[4]=&LiftA; motors[5]=&LiftB; motors[6]=&Intake; motors[7]=&Tilt; motors[8]=&Grip;
}

// ---- raw helpers (no expo/slew, so cmd vs actual is obvious) ----
int rawAxis(int v) { return (std::abs(v) < 7) ? 0 : -v; } // reversed + deadband

void stopAll() {
  LeftFront.stop(brakeType::brake); LeftRear.stop(brakeType::brake);
  RightFront.stop(brakeType::brake); RightRear.stop(brakeType::brake);
  LiftA.stop(brakeType::hold); LiftB.stop(brakeType::hold);
  Intake.stop(brakeType::coast); Tilt.stop(brakeType::hold); Grip.stop(brakeType::hold);
}

void pageInputs() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("P0 INPUTS L/R=page X+B=estop");
  Brain.Screen.newLine();
  Brain.Screen.print("A3=%d A1=%d A2=%d", (int)Controller1.Axis3.position(), (int)Controller1.Axis1.position(), (int)Controller1.Axis2.position());
  Brain.Screen.newLine();
  Brain.Screen.print("R1=%d R2=%d L1=%d L2=%d", Controller1.ButtonR1.pressing(), Controller1.ButtonR2.pressing(), Controller1.ButtonL1.pressing(), Controller1.ButtonL2.pressing());
  Brain.Screen.newLine();
  Brain.Screen.print("X=%d B=%d Y=%d A=%d", Controller1.ButtonX.pressing(), Controller1.ButtonB.pressing(), Controller1.ButtonY.pressing(), Controller1.ButtonA.pressing());
  Brain.Screen.newLine();
  Brain.Screen.print("Up=%d Dn=%d Lf=%d Rt=%d", Controller1.ButtonUp.pressing(), Controller1.ButtonDown.pressing(), Controller1.ButtonLeft.pressing(), Controller1.ButtonRight.pressing());
  Brain.Screen.newLine();
  // commanded outputs
  int fwd = rawAxis(Controller1.Axis3.position());
  int turn = rawAxis(Controller1.Axis1.position());
  int liftCmd = (Controller1.ButtonR1.pressing()?40:0) + (Controller1.ButtonR2.pressing()?-40:0);
  int inCmd = (Controller1.ButtonL1.pressing()?100:0) + (Controller1.ButtonL2.pressing()?-80:0);
  Brain.Screen.print("cmd drv %d/%d lift %d in %d", fwd+turn, fwd-turn, liftCmd, inCmd);
  Brain.Screen.newLine();
  Brain.Screen.print("gripLat=%d Tilt=%d", gripLatchedDbg?1:0, (int)Tilt.position(rotationUnits::deg));
  Brain.Screen.newLine();
}

void pageMotors() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("P1 MOTORS deg/vel");
  Brain.Screen.newLine();
  // Lift pair first — the suspect
  double a = LiftA.position(rotationUnits::deg), b = LiftB.position(rotationUnits::deg);
  Brain.Screen.print("LiftA %.0f %.0f%% B %.0f %.0f%% d=%.0f", a, LiftA.velocity(velocityUnits::pct), b, LiftB.velocity(velocityUnits::pct), a-b);
  Brain.Screen.newLine();
  Brain.Screen.print("LF %.0f RF %.0f", LeftFront.position(rotationUnits::deg), RightFront.position(rotationUnits::deg));
  Brain.Screen.newLine();
  Brain.Screen.print("Int %.0f %d%% Tlt %.0f Grp %.0f", Intake.position(rotationUnits::deg), (int)Intake.velocity(velocityUnits::pct), Tilt.position(rotationUnits::deg), Grip.position(rotationUnits::deg));
  Brain.Screen.newLine();
  Brain.Screen.print("GripVel %d%% TiltVel %d%%", (int)Grip.velocity(velocityUnits::pct), (int)Tilt.velocity(velocityUnits::pct));
  Brain.Screen.newLine();
  Brain.Screen.print("R1/R2 should move both Lift +/-");
  Brain.Screen.newLine();
  Brain.Screen.print("X/B should move Tilt 0..450");
  Brain.Screen.newLine();
}

void pagePower() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("P2 POWER batt=%.0f%% %.1fV", (double)Brain.Battery.capacity(), (double)Brain.Battery.voltage(voltageUnits::volt));
  Brain.Screen.newLine();
  Brain.Screen.print("LiftA %.1fNm %.1fA %dC", LiftA.torque(torqueUnits::Nm), LiftA.current(currentUnits::amp), (int)LiftA.temperature(temperatureUnits::celsius));
  Brain.Screen.newLine();
  Brain.Screen.print("LiftB %.1fNm %.1fA %dC", LiftB.torque(torqueUnits::Nm), LiftB.current(currentUnits::amp), (int)LiftB.temperature(temperatureUnits::celsius));
  Brain.Screen.newLine();
  Brain.Screen.print("Int %.1fNm %.1fA Tilt %.1fNm", Intake.torque(torqueUnits::Nm), Intake.current(currentUnits::amp), Tilt.torque(torqueUnits::Nm));
  Brain.Screen.newLine();
  Brain.Screen.print("Grp %.1fNm %.1fA LF %.1fA", Grip.torque(torqueUnits::Nm), Grip.current(currentUnits::amp), LeftFront.current(currentUnits::amp));
  Brain.Screen.newLine();
  Brain.Screen.print("Stall=vel~0+torq high. Hot>50C");
  Brain.Screen.newLine();
  Brain.Screen.print("Brownout if batt<20 or all stall");
  Brain.Screen.newLine();
}

void pageTest() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("P3 TEST %s Up/Dn sel", motorNames[testIdx]);
  Brain.Screen.newLine();
  motor* m = motors[testIdx];
  Brain.Screen.print("pos %.0f vel %d%%", m->position(rotationUnits::deg), (int)m->velocity(velocityUnits::pct));
  Brain.Screen.newLine();
  Brain.Screen.print("tq %.2fNm cur %.2fA %dC", m->torque(torqueUnits::Nm), m->current(currentUnits::amp), (int)m->temperature(temperatureUnits::celsius));
  Brain.Screen.newLine();
  Brain.Screen.print("L1=-30%% R1=+30%% hold to spin");
  Brain.Screen.newLine();
  Brain.Screen.print("A=zero this B=zero lift+tilt");
  Brain.Screen.newLine();
  Brain.Screen.print("If vel=0 but cmd>0: unplugged/");
  Brain.Screen.newLine();
  Brain.Screen.print("wrong port/reversed/burnt");
  Brain.Screen.newLine();
}

void pre_auton(void) {
  dbgInitMotors();
  LeftFront.setStopping(brakeType::brake); LeftRear.setStopping(brakeType::brake);
  RightFront.setStopping(brakeType::brake); RightRear.setStopping(brakeType::brake);
  LiftA.setStopping(brakeType::hold); LiftB.setStopping(brakeType::hold);
  Intake.setStopping(brakeType::coast); Tilt.setStopping(brakeType::hold); Grip.setStopping(brakeType::hold);
  Brain.Screen.print("DEBUG ready. L/R=page.");
  Brain.Screen.newLine();
}

void autonomous(void) { stopAll(); }

void usercontrol(void) {
  dbgInitMotors();
  int tick = 0;
  while (true) {
    // paging (tap)
    bool l = Controller1.ButtonLeft.pressing(), r = Controller1.ButtonRight.pressing();
    if (l && !prevLeftDbg) { page = (page + 3) % 4; }
    if (r && !prevRightDbg) { page = (page + 1) % 4; }
    prevLeftDbg = l; prevRightDbg = r;

    // e-stop: X+B together
    if (Controller1.ButtonX.pressing() && Controller1.ButtonB.pressing()) {
      stopAll(); gripLatchedDbg = false;
    } else if (page == 3) {
      // ---- TEST mode: direct single-motor control ----
      bool up = Controller1.ButtonUp.pressing(), dn = Controller1.ButtonDown.pressing();
      if (up && !prevUpDbg) testIdx = (testIdx + 1) % 9;
      if (dn && !prevDownDbg) testIdx = (testIdx + 8) % 9;
      prevUpDbg = up; prevDownDbg = dn;
      bool a = Controller1.ButtonA.pressing(), b = Controller1.ButtonB.pressing();
      if (a && !prevADbg) motors[testIdx]->setPosition(0, rotationUnits::deg);
      if (b && !prevBDbg) { LiftA.setPosition(0, rotationUnits::deg); LiftB.setPosition(0, rotationUnits::deg); Tilt.setPosition(0, rotationUnits::deg); }
      prevADbg = a; prevBDbg = b;
      // spin selected (stop everything else first so readings are clean)
      stopAll();
      if (Controller1.ButtonR1.pressing()) motors[testIdx]->spin(directionType::fwd, 30, velocityUnits::pct);
      else if (Controller1.ButtonL1.pressing()) motors[testIdx]->spin(directionType::fwd, -30, velocityUnits::pct);
      pageTest();
    } else {
      // ---- LIVE mode: raw passthrough + observe ----
      int fwd = rawAxis(Controller1.Axis3.position());
      int turn = rawAxis(Controller1.Axis1.position());
      LeftFront.spin(directionType::fwd, fwd+turn, velocityUnits::pct); LeftRear.spin(directionType::fwd, fwd+turn, velocityUnits::pct);
      RightFront.spin(directionType::fwd, fwd-turn, velocityUnits::pct); RightRear.spin(directionType::fwd, fwd-turn, velocityUnits::pct);

      if (Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()) {
        LiftA.spin(directionType::fwd, 40, velocityUnits::pct); LiftB.spin(directionType::fwd, 40, velocityUnits::pct);
      } else if (Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()) {
        LiftA.spin(directionType::fwd, -40, velocityUnits::pct); LiftB.spin(directionType::fwd, -40, velocityUnits::pct);
      } else { LiftA.stop(brakeType::hold); LiftB.stop(brakeType::hold); }

      if (Controller1.ButtonL1.pressing()) Intake.spin(directionType::fwd, 100, velocityUnits::pct);
      else if (Controller1.ButtonL2.pressing()) Intake.spin(directionType::fwd, -80, velocityUnits::pct);
      else Intake.stop(brakeType::coast);

      bool y = Controller1.ButtonY.pressing();
      if (y && !prevYDbg) { gripLatchedDbg = !gripLatchedDbg; if(!gripLatchedDbg) Grip.stop(brakeType::hold); }
      prevYDbg = y;
      if (gripLatchedDbg) Grip.spin(directionType::fwd, 35, velocityUnits::pct);
      else if (!Controller1.ButtonA.pressing()) Grip.stop(brakeType::hold);
      if (Controller1.ButtonA.pressing()) Grip.spin(directionType::rev, 80, velocityUnits::pct); // hold A = release

      if (Controller1.ButtonX.pressing()) { Tilt.spinToPosition(0, rotationUnits::deg, 50, velocityUnits::pct, false); }
      else if (Controller1.ButtonB.pressing()) { Tilt.spinToPosition(450, rotationUnits::deg, 50, velocityUnits::pct, false); }
      else if (!Tilt.isSpinning()) Tilt.stop(brakeType::hold);

      if (page == 0) pageInputs();
      else if (page == 1) pageMotors();
      else pagePower();
    }

    // controller mini-status + serial log every 200ms
    if (++tick % 10 == 0) {
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("P%d A%.0f/B%.0f d%.0f", page, LiftA.position(rotationUnits::deg), LiftB.position(rotationUnits::deg), LiftA.position(rotationUnits::deg)-LiftB.position(rotationUnits::deg));
      printf("P%d A3=%d A1=%d R1=%d R2=%d LftA=%.1f,%.1f,%.2f LftB=%.1f,%.1f,%.2f Int=%.1f Tilt=%.1f Grp=%.1f Batt=%.0f\n",
        page, (int)Controller1.Axis3.position(), (int)Controller1.Axis1.position(),
        (int)Controller1.ButtonR1.pressing(), (int)Controller1.ButtonR2.pressing(),
        LiftA.position(rotationUnits::deg), LiftA.velocity(velocityUnits::pct), LiftA.torque(torqueUnits::Nm),
        LiftB.position(rotationUnits::deg), LiftB.velocity(velocityUnits::pct), LiftB.torque(torqueUnits::Nm),
        Intake.position(rotationUnits::deg), Tilt.position(rotationUnits::deg), Grip.position(rotationUnits::deg),
        (double)Brain.Battery.capacity());
    }
    wait(20, timeUnits::msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) wait(100, timeUnits::msec);
}
