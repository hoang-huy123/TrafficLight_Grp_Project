# Traffic Light Project - Improved Review Version

This folder intentionally preserves the submitted team source in `original_source/` and marks changes in the improved source with comments such as `[ORIGINAL]`, `[IMPROVEMENT]`, `[NEW]`, and `[FIX]`.

## What was already in the original project
- QNX named IPC with `name_attach`, `name_open`, `MsgSend`, `MsgReceive`, `MsgReply`.
- Central controller, local controllers, display process, and manual test process.
- Vertical/horizontal traffic-light state machine with amber and all-red clearance.
- Congestion/fixed-like mode and sensor-driven mode.
- Car sensor events.
- Initial railway events and `RAIL_SAFE` state.
- Local controller continuing its state machine when Central cannot be reached.
- Mutex protection for local shared state.

## Added / improved
1. **New railway module**: `railway_controller.c/.h`.
   - Explicit `RAIL_CLEAR`, `RAIL_APPROACHING`, `RAIL_GATE_DOWN_STATE`, `RAIL_FAULT_STATE`.
   - Separates a railway fault from a normal train event.
2. **Railway fault reporting**.
   - `StatusMsg` now carries `rail_state`.
   - Central prints a visible fault alert and Display shows the railway state.
3. **Pedestrian implementation**.
   - `PEDESTRIAN_PRESSED` now records pending demand.
   - Pedestrian crossing is only served from an all-red road condition.
   - Railway safety has priority over pedestrian service.
4. **Expanded event simulator (`test.c`)**.
   - Car, pedestrian, train approach, gate down, rail clear, railway fault, and mode switching.
   - Fixed original `send_mode` signature/call mismatch.
5. **Safer IPC/state handling**.
   - Local controller snapshots state before blocking IPC; it does not hold the road-state mutex across `MsgSend`.
   - Input validation for intersection ID and operating mode.
6. **Improved observability**.
   - Display includes Railway and Pedestrian columns.
   - Local logs autonomous operation when Central is unavailable.

## Important design assumptions to justify before final submission
- Timing constants are still proof-of-concept values and need evidence/justification in the Implementation Note.
- After a railway event clears, the controller restarts with the vertical road. The team should justify this based on the chosen physical intersection/railway layout.
- The railway module models railway state; it does not yet implement a separate train-line controller/process that drives a physical train signal.
- `MsgSend` is synchronous. Central broadcasts commands sequentially, so a more advanced implementation could use worker threads/pulses if stronger timing isolation is required.
- Sensor demand is represented by booleans, not a vehicle queue/count. This is a deliberate proof-of-concept simplification.

## Recommended QNX build
Example commands (adapt to your QNX environment):

```sh
qcc -Wall -Wextra -o central_controller central_controller.c
qcc -Wall -Wextra -o display display.c
qcc -Wall -Wextra -pthread -o local_controller local_controller.c railway_controller.c
qcc -Wall -Wextra -o test test.c
```

This code was prepared outside a QNX runtime, so the team must compile and execute it on the actual QNX environment before submission.

## Attribution labels used in the improved source

- `ORIGINAL BASELINE - DAM HOANG HUY`: code/functionality already present in the preserved `original_source/` baseline.
- `PROPOSED CONTRIBUTION - TRAN VO VUONG`: code/functionality added or changed in this improved working version.
- `MIXED`: an original section that has been extended by the proposed contribution.

These labels are based on source comparison. They should be updated to match the team's actual reviewed, tested, and agreed contribution record before submission.
