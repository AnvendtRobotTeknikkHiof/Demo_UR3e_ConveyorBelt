"""
Stage 5: the robot picks the cylinder from the moving belt.

The laptop runs the belt, times the cylinder from S4 to S3 and to S1 (the software
encoder counts at that speed) and, as soon as S1 has seen it, sends the robot a pick job:
the tick at which the cylinder's centre is at the pick spot (178 cm). The robot waits
above the spot, starts following the belt at that tick, goes down, grips, lifts, stops
following and puts the cylinder on the table. The laptop stops the belt when S2 no longer
sees the cylinder (lifted away, or gone past if the pick missed).

    python pick_demo\\stage5_pick.py --static          belt stopped: check the taught pick once
    python pick_demo\\stage5_pick.py                   5 runs
    python pick_demo\\stage5_pick.py --offset-mm -25   start following 5 mm earlier than default

After each run S2 tells roughly where the cylinder was when the robot started following
(S2 sees its front edge when the centre is 3 cm short of S2). Tune --offset-mm by eye in
5 mm steps: more negative if the gripper comes down after the cylinder has passed,
less negative if it comes down in front of it.

For each run put the cylinder upright in the middle of the belt, before S4 (0-15 cm),
and stand at the belt end. Ctrl+C stops the belt.
"""
import argparse
import time

import stage2_belt_test as s2
from iolink_sensors import SENSORS, SensorReader, measure_baseline

PICK_CM = 178.0       # cylinder centre when the robot starts following (the taught 'pick')
DEFAULT_OFFSET_MM = -50   # tuned 2026-10-21 (fingers across the belt, 1 V): grip in the middle
RADIUS_CM = 3.0       # cylinder radius: S2 sees the front edge when the centre is this short of S2
TRIGGER = "S1"        # the job is sent when this sensor has seen the whole cylinder
MIN_LEAD_S = 1.0      # the job must reach the robot at least this long before the start
STOP_PAST_S2_CM = 12  # if the pick misses, the belt stops with the cylinder ~12 cm past S2
DONE_TIMEOUT_S = 30   # pick and place must be finished within this


def run_once(belt, reader, base, order, pos, volts, offset_mm, reads):
    first = order[0]
    centre, info = {}, {}

    def on_leave(name, t_enter, t_leave):
        centre[name] = (t_enter + t_leave) / 2
        if name == order[-1]:
            info["last_enter"] = t_enter      # S2 saw the front edge
        if name == first or first not in centre or name not in (order[1], TRIGGER):
            return
        v = (pos[name] - pos[first]) / (centre[name] - centre[first])
        belt.set_belt_speed(v)
        info["speed"] = v
        if name != TRIGGER:
            return
        belt.update()
        tick_trigger = s2.tick_at(belt.samples, centre[TRIGGER])
        start = tick_trigger + (PICK_CM / 100 - pos[TRIGGER] + offset_mm / 1000) * s2.TICKS_PER_M
        lead = (start - s2.tick_of(belt.samples[-1])) / (v * s2.TICKS_PER_M)
        if lead < MIN_LEAD_S:
            info["late"] = lead
            return
        info["job"] = belt.send_job(start)
        info["lead"] = lead
        info["start"] = start

    belt.set_belt_speed(s2.belt_speed_guess(volts))
    s2.one_pass(belt, reader, base, order, pos, volts, reads, on_leave,
                stop_past_cm=STOP_PAST_S2_CM)

    if "speed" in info:
        print(f"  speed {first} -> {TRIGGER}: {info['speed'] * 100:.2f} cm/s")
    if "late" in info:
        print(f"  Too late to send the job ({info['late']:.1f} s before the pick spot); no pick.")
        return False
    if "job" not in info:
        print(f"  {TRIGGER} did not see the whole cylinder; no pick.")
        return False
    print(f"  job {info['job']} sent {info['lead']:.1f} s before the cylinder reached {PICK_CM:.0f} cm")
    if "last_enter" in info:
        # where was the cylinder centre at the start tick, measured by S2?
        last = order[-1]
        tick_enter = s2.tick_at(belt.samples, info["last_enter"])
        centre_at_start = (pos[last] * 100 - RADIUS_CM
                           - (tick_enter - info["start"]) / s2.TICKS_PER_M * 100)
        ahead_mm = (centre_at_start - PICK_CM) * 10
        print(f"  when the robot started following, the cylinder centre was at about"
              f" {centre_at_start:.1f} cm by S2 ({abs(ahead_mm):.0f} mm"
              f" {'past' if ahead_mm >= 0 else 'before'} {PICK_CM:.0f} cm)")
    return wait_for_job(belt, info["job"])


def wait_for_job(belt, job):
    """Wait until the robot reports the job done. True when it did."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < DONE_TIMEOUT_S:
        s = belt.update()
        if s["output_int_register_27"] == job:
            print(f"  robot finished job {job} after {time.monotonic() - t0:.1f} s")
            return True
        if s["runtime_state"] != s2.PLAYING:
            print("  The robot program STOPPED during the pick. Look at the message on the"
                  " pendant (or the Log tab) and take a photo.")
            return False
        time.sleep(0.05)
    print(f"  The robot did not finish job {job} within {DONE_TIMEOUT_S} s."
          " Is pick_stage5 running?")
    return False


def static_pick(belt):
    """Belt stopped: the robot picks the cylinder standing on the pick mark, using the
    same program as on the moving belt. Checks the taught pick before the belt runs."""
    print(f"Belt stopped. Put the cylinder upright on the {PICK_CM:.0f} cm mark, in the middle"
          " of the belt.")
    input("Press Enter and the robot picks it ... ")
    belt.update()
    job = belt.send_job(s2.tick_of(belt.samples[-1]))     # start following now
    return wait_for_job(belt, job)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--volts", type=float, default=1.0, help="belt voltage (default 1.0)")
    ap.add_argument("--runs", type=int, default=5, help="number of runs (default 5)")
    ap.add_argument("--offset-mm", type=float, default=DEFAULT_OFFSET_MM,
                    help=f"+ = the robot starts following later, - = earlier"
                         f" (default {DEFAULT_OFFSET_MM})")
    ap.add_argument("--static", action="store_true",
                    help=f"belt stopped: pick the cylinder from the {PICK_CM:.0f} cm mark once")
    args = ap.parse_args()

    order = [n for n, _, _ in sorted(SENSORS, key=lambda s: s[2])]     # along the belt
    pos = {n: p for n, _, p in SENSORS}
    belt = s2.Belt()
    reader = SensorReader()
    reads, done = [], 0
    try:
        if not belt.program_running():
            raise SystemExit("The robot program is not running. Start it: python pick_demo\\"
                             "stage5_robot.py start (or Play pick_stage5 on the pendant).")
        belt.stop()
        belt.wait_until_still()
        if args.static:
            done += static_pick(belt)
            return
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        base = measure_baseline(reader)
        missing = [n for n, b in base.items() if b is None]
        if missing:
            raise SystemExit(f"No valid reading from {', '.join(missing)}.")

        for run in range(1, args.runs + 1):
            print(f"\nRun {run} of {args.runs}: put the cylinder upright in the middle of the"
                  " belt, before S4 (0-15 cm).")
            input("Stand at the belt end. Press Enter to start the belt ... ")
            done += run_once(belt, reader, base, order, pos, args.volts, args.offset_mm, reads)
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    except (ConnectionError, RuntimeError) as e:
        print(f"\n{e}")
    finally:
        if belt.moved:
            try:
                belt.stop()
            except Exception as e:
                print(f"!! Could not stop the belt ({e}). Set conveyor_stop High in the I/O tab.")
        reader.close()
        if belt.samples:
            print(f"Log: {belt.save('stage5_pick')}")
        if reads:
            print(f"Sensor log: {s2.save_reads(reads, order, 'stage5_pick_sensors')}")
        belt.close()
    print(f"\nRobot finished {done} pick(s). Did each one grip the cylinder in the middle?"
          " If the fingers met it too early or too late, run again with --offset-mm.")


if __name__ == "__main__":
    main()
