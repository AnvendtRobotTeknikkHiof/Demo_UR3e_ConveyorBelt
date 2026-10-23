"""
Stage 3: software encoder and prediction. The arm does not move.

The encoder wheel slips, so the robot program counts ticks itself
(robot/pick_vencoder.script) at the belt speed the laptop measures for each cylinder:
the time it takes from S4 to S3, and then from S4 to S1. This test checks the whole
timing chain the pick will use: when the cylinder has passed S1, the laptop predicts
the tick at which it reaches S2, and compares with the tick when S2 really sees it.

    python pick_demo\\stage3_predict_test.py                     5 runs at 1.0 V
    python pick_demo\\stage3_predict_test.py --volts 0.5 --runs 10

For every run, put the cylinder upright in the middle of the belt before S4 (0-15 cm);
later robot #1 does this. Ctrl+C stops the belt. Logs go to logs/.
"""
import argparse

import stage2_belt_test as s2
from iolink_sensors import SENSORS, SensorReader, measure_baseline

TRIGGER, CHECK = "S1", "S2"    # the pick is planned from S1; S2 checks the prediction
PASS_MM = 10                   # largest allowed prediction error
PASS_RATIO = 0.02              # software encoder must count within 2 % of the speed sent


def run_once(belt, reader, base, order, pos, volts, reads):
    """One cylinder past all sensors. Returns a result dict or None."""
    first = order[0]
    centre, speeds = {}, {}

    def on_leave(name, t_enter, t_leave):
        # as soon as a sensor has seen the whole cylinder, update the speed the robot counts with
        centre[name] = (t_enter + t_leave) / 2
        if name != first and first in centre and name in (order[1], TRIGGER):
            v = (pos[name] - pos[first]) / (centre[name] - centre[first])
            belt.set_belt_speed(v)
            speeds[name] = v

    belt.set_belt_speed(s2.belt_speed_guess(volts))
    crossings = s2.one_pass(belt, reader, base, order, pos, volts, reads, on_leave)
    if not crossings or TRIGGER not in crossings or CHECK not in crossings \
            or TRIGGER not in speeds:
        print(f"  {first}, {TRIGGER} and {CHECK} must all see the whole cylinder; run ignored.")
        return None

    t, c = crossings[TRIGGER], crossings[CHECK]
    v = speeds[TRIGGER]
    predicted = t["tick"] + (pos[CHECK] - pos[TRIGGER]) * s2.TICKS_PER_M
    error_mm = (c["tick"] - predicted) / s2.TICKS_PER_M * 1000
    ratio = (c["tick"] - t["tick"]) / (c["t"] - t["t"]) / (v * s2.TICKS_PER_M)
    r = {"speed": v, "error_mm": error_mm, "ratio": ratio,
         "nearest": {n: crossings[n]["nearest"] for n in order if n in crossings}}
    print(f"  speed {first} -> {TRIGGER}: {v * 100:.2f} cm/s (sent to the robot)")
    print(f"  {CHECK} predicted from {TRIGGER}: error {error_mm:+.1f} mm")
    print(f"  software encoder counted {ratio * 100:.1f} % of the speed sent")
    print("  nearest face (cylinder centred): "
          + ", ".join(f"{n} {d} cm" for n, d in r["nearest"].items()))
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--volts", type=float, default=1.0, help="belt voltage (default 1.0)")
    ap.add_argument("--runs", type=int, default=5, help="number of runs (default 5)")
    args = ap.parse_args()

    order = [n for n, _, _ in sorted(SENSORS, key=lambda s: s[2])]     # along the belt
    pos = {n: p for n, _, p in SENSORS}
    belt = s2.Belt()
    reader = SensorReader()
    reads, results = [], []
    try:
        if not belt.program_running():
            raise SystemExit("The robot program is not running. Load pick_stage3 and press Play.")
        belt.stop()
        belt.wait_until_still()
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        base = measure_baseline(reader)
        missing = [n for n, b in base.items() if b is None]
        if missing:
            raise SystemExit(f"No valid reading from {', '.join(missing)}.")
        print("Empty belt: " + ", ".join(f"{n} {base[n]} cm" for n in order))

        for run in range(1, args.runs + 1):
            print(f"\nRun {run} of {args.runs}: put the cylinder upright in the middle of the"
                  " belt, before S4 (0-15 cm).")
            input("Stand at the belt end, ready to catch it. Press Enter to start the belt ... ")
            r = run_once(belt, reader, base, order, pos, args.volts, reads)
            if r:
                results.append(r)
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    except ConnectionError as e:
        print(f"\n{e}")
    finally:
        if belt.moved:
            try:
                belt.stop()
            except Exception as e:
                print(f"!! Could not stop the belt ({e}). Set conveyor_stop High in the I/O tab.")
        reader.close()
        if belt.samples:
            print(f"Log: {belt.save('stage3_predict')}")
        if reads:
            print(f"Sensor log: {s2.save_reads(reads, order, 'stage3_predict_sensors')}")
        belt.close()

    if results:
        errors = [r["error_mm"] for r in results]
        ratios = [r["ratio"] for r in results]
        speeds = [r["speed"] * 100 for r in results]
        worst = max(abs(e) for e in errors)
        print(f"\n{len(results)} runs at {args.volts:.2f} V")
        print(f"belt speed: {min(speeds):.2f} - {max(speeds):.2f} cm/s")
        print(f"prediction error at {CHECK}: " + ", ".join(f"{e:+.1f}" for e in errors)
              + f" mm  (worst {worst:.1f} mm)")
        print(f"software encoder: {min(ratios) * 100:.1f} - {max(ratios) * 100:.1f} % of the speed sent")
        if max(abs(x - 1) for x in ratios) > PASS_RATIO:
            print("-> The robot is not counting at the speed we sent. Is pick_vencoder.script"
                  " in BeforeStart, and does the program run pick_stage3?")
        elif worst <= PASS_MM:
            print(f"PASS: every prediction within {PASS_MM} mm.")
        else:
            print(f"FAIL: a prediction was more than {PASS_MM} mm off.")


if __name__ == "__main__":
    main()
