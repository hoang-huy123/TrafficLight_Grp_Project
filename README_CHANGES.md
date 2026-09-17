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
- The deploy sequence is always **scp -> chmod +x -> run**. `chmod +x` must be
  redone after *every* scp, because the newly copied files do not keep the
  executable bit. `chmod` and `pidin` run in the QNX session (`#` prompt), not
  in PowerShell.

## Verified on QNX (no code change)

- The custom `msg_header_t` / `_mysigval` structs in `ipc.h` compile and work
  correctly on QNX 7.1. They are not a hack - they come from the course's own
  Week 8 `NativeMsgPass-Server-1.c` example.
- `-pthread` is not needed on QNX. pthread is part of libc and qcc ignores the
  flag. The old build commands still work, the flag is just redundant.
- Test plan T1-T13 was run on the target. All passed, except for the deadlock
  below which was found during T11 and then fixed.

## Fixed

1. **Deadlock between Central and a local controller** (VU LUONG MINH TRIET)
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

2. **Override countdown not visible on the Display** (VU LUONG MINH TRIET)
   - `send_status()` only runs on a phase change, and the phase does not change
     during an override hold, so the Display showed a frozen value.
   - `override_hold()` now sends a status update once per second, the same 1 Hz
     pattern `handle_railway_event()` already used.

## Added

3. **Control-room priority override** (VU LUONG MINH TRIET)
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

4. **Display override column** (VU LUONG MINH TRIET)
   - Seventh column showing direction and seconds left (e.g. `V 24s`), or `-`.

5. **Test menu override options** (VU LUONG MINH TRIET)
   - `10)` hold VERTICAL green, `11)` hold HORIZONTAL green, `12)` cancel.
   - Each asks for a target intersection and, for 10 and 11, a duration.

6. **Real-time thread priorities** (VU LUONG MINH TRIET)
   - Nothing in the system set a scheduling priority or policy - every thread
     ran at the QNX default of 10. Lecture 6 (Gomaa Task Priority Criteria)
     covers this explicitly, so it was worth doing properly.
   - Added `rt_set_self_priority()` and the priority constants to `ipc.h`, so
     all four projects pick them up without adding new files.
   - Policy is `SCHED_RR`. Our threads are not CPU-bound (they block on timed
     waits and `MsgReceive`), so round-robin costs nothing and stops a
     same-priority thread monopolising the CPU. `SCHED_FIFO` is the
     alternative if strict run-to-block behaviour is wanted.

   | Thread | Priority | Reason |
   | --- | --- | --- |
   | Local event intake (server thread) | 21 | Railway/sensor events must preempt everything else. A train event stuck behind other work is a safety failure. |
   | Local state machine (main thread) | 19 | Enforces amber and clearance timing. Late transitions are a safety failure. |
   | Central controller | 14 | Supervisory only - never drives a light directly, and the locals run without it, so it must not compete with intersection control. |
   | Display | 10 | Output only, no deadline. Redrawing must never delay a controller. |
   | Test simulator | 10 | Stands in for external hardware, not part of the deployed system. |

   - Failure to set a priority is a warning, not a fatal error - the system
     still runs correctly, it just loses the timing guarantee.
   - Verified on the target with `pidin -p local_controller`:

     ```
     pid     tid name              prio STATE     Blocked
     1343529  1  p/local_controller 19r NANOSLEEP
     1343529  2  p/local_controller 21r RECEIVE   1
     ```

     The `r` suffix confirms `SCHED_RR` is actually active, and the STATE
     column shows each thread doing its intended job. Use `pidin -p <name>`
     (without the word `thread`) to see the prio column.
   - Regression tested afterwards: T3 (sensor demand), T5/T7 (railway) and T14
     (override) all still behave as before. Priorities change timing
     guarantees, not behaviour.

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

## For the team to decide

- **Pedestrian deferral during an override.** Serving pedestrians first is
  equally arguable. We should agree the reasoning before the demo rather than
  improvise it.
- **Attribution.** New code is marked `[NEW]`, `[FIX]`, `[CHANGED]` and
  `[WAS DEAD CODE - NOW IMPLEMENTED]`. Please review and confirm before
  submission.

## Still outstanding

1. Multi-node QNX (Qnet) - still one VM. Needs a second VM and attach points
   changed from `local_1` to `/net/<hostname>/local_1`.
2. Central's broadcast is sequential and blocking, with no timeout.
3. No coordinated railway event across I1 and I2 - one crossing, but a train
   currently has to be injected per intersection.
4. Trains from both directions not distinguished (double-track corridor).
5. Timing constants still unjustified - the brief asks for R1-R5 timing and
   coordination assumptions to be stated and justified.
6. `sleep()` used rather than QNX timers (`timer_create` + `SIGEV_PULSE`, Week 6).
7. Pulses ignored in `server_thread` (`rcvid == 0`); Week 8 handles
   `_PULSE_CODE_DISCONNECT`.
8. Separate train-line controller process (the brief mentions it for a full
   implementation).
