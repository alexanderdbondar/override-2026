# ADR 0001. Season toolchain

Status: decided in principle, not in practice yet. Move to PROS on VS Code this season. Preferred chassis library is LemLib, but we pick that after the drive is rebuilt and we know what sensors are going in.
Date: 2026-10-25

The old build's code was single files pasted into codev5.vex.com. Two files ended up arguing about port ownership in comments, and nothing got compiled before robot day. Changing toolchains costs nothing right now because there is no code to move.

## What we found about what other teams use

Checked Oct 2026, mostly from VEX Forum, team write-ups, and public season repos.

- PROS on VS Code is the common runtime at Worlds-level teams. Open source C++ runtime for the V5, CLI build and upload, normal project folder, works with git. Made by Purdue SIGBots.
- Chassis libraries on top of PROS: LemLib (odometry, pure pursuit, move to pose) and EZ-Template (PID, odometry, auton selector, live PID tuning) do similar jobs with different APIs. Teams use one, not both. LemLib's v0.6 is a rewrite in progress, so if we use it we pin the last stable release.
- robodash for the auton selector and console, or the selector built into EZ-Template.
- Some very fast teams write their own odometry on raw PROS instead of using a library.
- If we stayed on VEXcode, JAR-Template is the library of that path, and PATH.JERRYIO-style path export feeds pure pursuit in either case.

Sources: VEX forum programming poll May 2025 (vexforum.com/t/programming-software-poll/136928), Spartan Design onboarding (spartandesignrobotics.org/toolchain-onboarding), the 315P season write-up (aadishv.dev/robotics-2), public season repos 38535A, 2530A, 1624A, r/vex thread on what top teams use (reddit.com/r/vex/comments/1hjio3k), LemLib and EZ-Template readmes.

## Things people raised

- The G4 question about open-source drivetrain libraries comes up on the forum every so often. Using common libraries is normal practice, but people on the team should be able to explain how the movement code works.
- A PROS project is a plain folder, so setting up CI to compile pull requests later is possible.
- First toolchain install takes a while (the VS Code PROS extension downloads a compiler toolchain).

## Next steps

1. Team reads the sources and agrees on PROS + VS Code.
2. After the drive is rebuilt: pick IMU and/or tracking wheels, then the chassis library (LemLib as the current preference).
