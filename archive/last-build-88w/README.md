# Last build (bot taken apart)

This code does not run on anything anymore. It describes the build we just disassembled, and that hardware no longer exists.

Kept because the header comments are the build's working notes: the bot map, the controller bindings, and the "tune first" checklist.

Useful for the new build:

- Controller bindings -> docs/controller-map.md
- Tuning notes, including the lift problem (one motor in the two-motor group reversed wrong, cut out after a second, fixed by flipping the flag and clearing the fault with Left tap) -> docs/tuning-protocol.md
- trial_v1_intake_claw.cpp has the handoff macro: tilt vertical, intake and grip seat the game piece, tilt horizontal, lift. Worth remapping to new ports and reusing.
- debug.cpp has screen readout patterns for tuning.

Reference build: drive 44W (ports 14/15, 8/9), lift 22W (12/20), intake 11W (10), tilt 5.5W (11), grip 5.5W (18). Docs on this: docs/port-map.md.

One problem to avoid repeating: two driver files traded ownership notes in comments ("handled by another agent, do not touch"). This build, port numbers live in one place instead.
