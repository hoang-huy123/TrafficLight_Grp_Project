# Demonstration / Test Plan

| ID | Scenario | Trigger | Expected result |
|---|---|---|---|
| T1 | Normal traffic cycle | No external event | V green -> V amber -> all red -> H green -> H amber -> all red |
| T2 | Vertical demand | Menu 1 | Demand is recorded and served by the vertical green phase |
| T3 | Horizontal demand in sensor mode | Menu 2 while V is green | At a sensor checkpoint, V phase can finish and safely transition to H |
| T4 | Pedestrian request | Menu 3 | Request becomes pending; after safe road transition, both roads red and PED_CROSS is served |
| T5 | Train approaching | Menu 4 | Intersection enters railway safety sequence and ultimately RAIL_SAFE |
| T6 | Gate down | Menu 5 | Roads remain held safe while railway state is active |
| T7 | Train/gate clear | Menu 6 | Controller leaves RAIL_SAFE through all-red and resumes road operation |
| T8 | Railway fault | Menu 7 | Roads are held safe; Central reports fault; Display shows FAULT |
| T9 | Central failure | Kill Central while locals run | Local traffic cycles continue autonomously |
| T10 | Central recovery | Restart Central after T9 | A later local status reconnects and Central/Display receive updates again |
| T11 | Congestion mode | Menu 8 | Central broadcasts mode and active locals use fixed configured green durations |
| T12 | Sensor mode | Menu 9 | Central broadcasts mode and locals respond to demand checkpoints |
| T13 | Multiple locals | Run I1 and I2 | Each local runs independently and Central monitors both |

For each test, capture terminal output/screenshots and record Actual Result + Pass/Fail in the final Implementation Note.
