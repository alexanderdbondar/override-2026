# Tuning steps

From the build's "TUNE FIRST" notes (archive/last-build-88w/main.cpp). Work down the list after wiring, before trying auton.

1. Check every motor spins the way its config says. Direction flags get changed in the shared config, not in subsystem files.
2. Check ports match docs/port-map.md.

## Zeroing and limits

1. Put every actuator in its rest position, then re-zero in software from there.
2. Raise limits in small steps. On the lift we raised LIFT_MAX_DEG by 100 deg at a time, after checking the low positions were right.

## Paired motors and faults

1. On the last build the lift cut out after about a second because one motor in the two-motor group was reversed wrong. The fix was to flip the flag, then clear the fault with the Left tap button.
2. Check that fault clearing works from the controller, not just from a re-download.
3. Left held for 1s is the e-stop: stops the manipulator motors and clears faults. Keep this reachable from any state, including mid-macro.

## Drivers

1. Driver-preferred direction reversals get locked into the shared config.
2. Tune subsystems before auton. One change at a time.
