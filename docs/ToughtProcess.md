# Thought process

This document explains the design of the host-based SIL in `src/sil.cpp` and
the test strategy in `tests/sil_tests.cpp`. The supplied ECU logic
(`given/smu.cpp`, `given/ebs.cpp`) is untouched.

## Overview

`SilSystem` owns two POSIX threads (via `std::thread`): one runs
`smu::run_cycle()` and one runs `ebs::run_cycle()`, each every 10 ms. The ECUs
reach the simulated world only through the four `ecu_io` functions. They are
forwarded to the active `SilSystem`, which keeps all simulated state in one
`State` struct: sensor values, actuator outputs, the EBS input queue and the
emitted-frame monitor queue.

## Synchronization strategy

- **One mutex, short critical sections.** A single `std::mutex` protects the
  whole `State` (sensors, outputs, both queues). Every access is a few
  instructions with no I/O and no blocking call inside, so contention is
  negligible and there is no lock ordering to get wrong.
- **The ECU step never runs under the lock.** The ECU code calls back into
  `read_sensor`, `write`, etc., which lock the mutex. Holding it around
  `run_cycle()` would deadlock with a non-recursive mutex.
- **Timed waits only.** `receive_frame` uses `condition_variable::wait_for`
  with a predicate (queue not empty): no infinite wait, no busy-wait, immune to
  spurious wakeups, and it returns `false` on timeout. The condition variable
  is notified only when a frame is added to the monitor queue (`write`).
- **Fixed-rate ECU loops.** Each thread advances an absolute deadline on
  `steady_clock` by one period and calls `sleep_until`, so the period does not
  drift and is not affected by wall-clock changes.
- **Atomics only for control flags.** `running` and `stop_requested` are
  atomics because they are read outside the mutex; they carry no other data.
- **The global `active_system` pointer** is written in `start()` before the
  threads are created and cleared in `stop()` after they are joined. Thread
  creation and join give the required happens-before, so no lock is needed.
  This is also why only one running `SilSystem` is supported at a time (the
  ECU modules have process-global state anyway).

## Communication routing

| Producer | Monitor queue (`receive_frame`) | EBS input queue |
| --- | --- | --- |
| SMU `write` | yes | yes |
| EBS `write` | yes | no |
| `send_frame` (test) | no | yes |

`try_read` only serves the EBS (the SMU has no input); it is a non-blocking
pop. Both queues are bounded (64 frames). When full, the **oldest** frame is
discarded before the new one is added (`push_bounded`): under overload the
writers must never block, and for control data the newest value is the useful
one.

## Thread lifecycle

- `start()` is idempotent, resets both ECUs' internal state, then launches the
  threads. `stop()` is idempotent: it raises `stop_requested`, joins both
  threads and clears `active_system`. The destructor calls `stop()`, so a
  running SIL that goes out of scope is always joined. A stopped SIL can be
  started again.
- Sensors, outputs and queued frames survive `stop()`/`start()` on purpose, so
  a test can preload sensors and frames *before* starting. ECU-internal state
  is reset on every `start()`.
- `start()`/`stop()` are meant to be called from one controlling thread.

## Test approach

- **Black box.** Tests use only `SilSystem` and the public `ecu_io`
  functions. Where the bus routing has to be checked, the test plays the role
  of an ECU by calling `ecu_io::write(Ecu::Smu/Ebs, ...)`.
- **Deadlines, not sleeps.** Tests wait for a condition under a timeout
  (`wait_for_matching_frame`, `drive_until`) and re-publish the EBS inputs every
  10 ms so they never go stale (the limit is 250 ms). A fresh `SilSystem` is
  used per case because the ECU modules are process-global.
- **Boundaries.** Values sit exactly on every threshold (49/50 counts,
  10/11 points, 7 and 10 bar, 15 and 99 bar, the 16/17 bar coherency edge,
  5/6 dm/s) so a flipped comparison is detected. The two APP faults are told
  apart by their fault code. Pneumatic/hydraulic pairing (front-left,
  rear-right) is checked with asymmetric values.
- **Complete output check.** "Safe" means ASNode open **and** both valves
  engaged **and** the expected fault code; "nominal" means the opposite plus
  fault 0. All outputs start at zero, so checking one output could pass before
  the EBS has even run.
- **Live transitions.** Sensors and frames are changed while the SIL runs
  (ASMS, watchdog, valve power, pressure, mission finished, speed, APP
  faults) and the fault and the recovery are both checked.
- **Bus and lifecycle.** Periodic frames from both ECUs, `receive_frame`
  timeout, SMU frames delivered to the EBS, EBS frames *not* looped back, drop
  oldest on a full queue, double start/stop, restart, destruction while
  running.
- **Checked against faulty ECUs.** I also ran the suite against hand-made
  faulty copies of the ECUs (flipped comparisons, changed thresholds, ignored
  inputs, swapped pressure pairs). 13 of 15 were detected; the other two are
  behaviourally equivalent to the original and cannot be detected.

## CI

`.github/workflows/ci.yml` runs on every push and pull request on
`ubuntu-24.04`: a clean `cmake --workflow --preset sil` job, and a second job
that builds the same tests with ThreadSanitizer and runs them through CTest.
Both fail the workflow on a non-zero test result.

## Trade-offs and limitations

- A single mutex is simple and easy to review; a lock per queue would not
  help at this scale.
- ECU threads use `sleep_until`, so shutdown can take up to one period (10 ms).
  A condition-variable wait woken by `stop()` would be immediate but is not
  needed here.
- `receive_frame` does not wake up early when the SIL stops; it waits for its
  timeout, which is always bounded.
- If a cycle overruns, the loop catches up without skipping; there is no
  overrun handling on a host simulator.
- A few tests use time margins (for example "still fresh after 120 ms" against
  a 250 ms limit). They are wide on purpose to avoid flaky results on a loaded
  CI runner, at the cost of not testing the exact 250 ms edge.
