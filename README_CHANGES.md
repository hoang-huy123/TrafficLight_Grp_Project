# Traffic Light Project - Change Round 2 (QNX Tested)

This continues from the previous `README_CHANGES.md`. Everything in this round
has been **compiled with qcc and run on a real QNX 7.1 VM target**, which the
previous round had not been.

Changed files: `ipc.h`, `local_controller.c`, `central_controller.c`,
`display.c`, `test.c`.

## IMPORTANT before you build

- `ipc.h` exists as a **separate copy in each of the four Momentics projects**.
  All four copies must be replaced, or you get errors like
  `OVERRIDE_NONE undeclared`.
- `StatusMsg` and `CommandMsg` changed size, so **all four binaries must be
  rebuilt and redeployed together**. Mixing an old binary with a new one
  compiles fine but exchanges garbage at runtime.
- `/tmp` on the QNX VM is cleared on reboot, so the binaries must be copied
  across again after every restart.

## Verified on QNX 

- The custom `msg_header_t` / `_mysigval` structs in `ipc.h` compile and work
  correctly on QNX 7.1. They are not a hack - they come from the course's own
  Week 8 `NativeMsgPass-Server-1.c` example.
- `-pthread` is not needed on QNX. pthread is part of libc and qcc ignores the
  flag. The old build commands still work, the flag is just redundant.
- Test plan T1-T13 was run on the target. All passed, except for the deadlock
  below which was found during T11 and then fixed.

## Fixed

1. **Deadlock between Central and a local controller** 
   - Symptom: choosing "Set mode: CONGESTION" froze `test`, `central_controller`
     and `display`. Only the local light sequencing kept running.
   - Cause: Central blocked in `MsgSend()` waiting for I1's reply, while I1's
     `server_thread` called `send_status()` (another blocking `MsgSend()` back
     to Central) *before* replying. Neither could proceed - a circular wait.
   - Fix: `send_status()` moved out of `handle_message()` and is now called in
     `server_thread()` **after** `MsgReply()` completes.
   - For the report: this is one of the four necessary deadlock conditions from
     Lecture 8, removed by reordering. The existing code was already following
     the lecture's other advice (not holding a mutex across `MsgSend()`).

2. **Override countdown not visible on the Display** 
   - `send_status()` only runs on a phase change, and the phase does not change
     during an override hold, so the Display showed a frozen value.
   - `override_hold()` now sends a status update once per second, the same 1 Hz
     pattern `handle_railway_event()` already used.

## Added

3. **Control-room priority override**
   - `CommandMsg.hold_green` existed in the original `ipc.h` but was never read
     by any process - it was dead code. The brief requires this feature:
     "The central control room may also initiate override commands ... to deal
     with exceptional situations (e.g. to provide a clear path for a visiting
     dignitary)."
   - An operator can now hold a chosen road green at one intersection, or
     across all of them, for a bounded time.
   - `hold_green` carries the direction (`OVERRIDE_NONE` / `OVERRIDE_VERTICAL` /
     `OVERRIDE_HORIZONTAL`), `hold_seconds` the duration, and
     `target_intersection` the target (0 = all, 1-6 = one).
   - `StatusMsg` gained `override_dir` and `override_remaining` so the override
     is visible at Central and on the Display.

   Safety rules built in (expect questions on these):
   - Railway safety still wins - a train event ends an override hold at once.
   - No green-to-green. If the override targets the road that is currently red,
     the active green finishes through amber and all-red first. Confirmed on
     the target: `H_GREEN -> H_AMBER -> ALL_RED -> V_GREEN`.
   - An override always expires. The countdown is kept **locally**, not by
     Central, and is clamped (default 20 s, max 120 s), so a lost cancel
     command or an offline Central cannot hold a green forever.
   - Pedestrian requests are deferred, not dropped - the request stays pending
     and is served at the first all-red after the override ends.

4. **Display override column** 
   - Seventh column showing direction and seconds left (e.g. `V 24s`), or `-`.

5. **Test menu override options** 
   - `10)` hold VERTICAL green, `11)` hold HORIZONTAL green, `12)` cancel.
   - Each asks for a target intersection and, for 10 and 11, a duration.

## Refactor note

`hold_green()` now handles override, sensor-driven demand, and fixed timing in
one place. The separate `interruptible_sleep(green_duration(...))` call for
congestion mode was folded into it.

**Behaviour with no override active is unchanged.** Re-tested: sensor mode
still ends a green at the checkpoint (measured 10 s) and congestion mode still
runs the full fixed green (measured 30 s).

## New test cases

| ID | Scenario | Status |
| --- | --- | --- |
| T14 | Override vertical green on I1 for 30 s | Passed on target |
| T15 | Cancel an active override (option 12) | Passed on target |
| T16 | Train event during an active override | Not yet run |
| T17 | Override with target 0 (all intersections) | Not yet run |
| T18 | Pedestrian request during an active override | Not yet run |
| T19 | Regression: T2/T3 and T11 after the refactor | Passed on target |

