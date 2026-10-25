# Port list

From the last build (header comments in archive/last-build-88w/main.cpp). 8 motors x 11W = 88W is the limit, and the last build used exactly 88W.

| Subsystem | Ports | W | Notes |
|---|---|---|---|
| Drive left | 14, 15 | 22 | blue 600rpm. 36T motor to 48T wheel, about 450rpm on 2.75in wheels |
| Drive right | 8, 9 | 22 | reversed, blue 600rpm |
| Lift | 12, 20 | 22 | blue 600rpm winch, 3 stage cascade, string on axle, encoder only, no sensors |
| Intake | 10 | 11 | 3 flywheels 12in long, low at the rear |
| Tilt | 11 | 5.5 | 12:60 gearing. 450 motor deg = 90 claw deg |
| Grip | 18 | 5.5 | claw rollers |
| Total | | 88 | at the limit |

Update this file when the wiring of the new build is decided, in the same commit as the code.

New assignments:

| What | Ports | W | Running total |
|---|---|---|---|
| | | | |
