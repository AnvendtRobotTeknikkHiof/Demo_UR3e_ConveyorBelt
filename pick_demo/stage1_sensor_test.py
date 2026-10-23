"""
Stage 1: check the laser distance sensors. Laptop only, the robot is not used.

    python stage1_sensor_test.py live                    values twice a second; compare
                                                         with the number on each sensor's display
    python stage1_sensor_test.py mapping                 cover one sensor at a time; it says which
    python stage1_sensor_test.py baseline --seconds 30   empty belt: noise and poll timing

Ctrl+C stops live and mapping.
"""
import argparse
import statistics
import time

from iolink_sensors import SENSORS, SensorReader, measure_baseline

BLOCKED_BELOW_CM = 5   # mapping: blocked when this much closer than the empty-belt value
CLEAR_WITHIN_CM = 2    # mapping: clear again when back within this of the empty-belt value


def run_until(seconds):
    end = None if seconds is None else time.monotonic() + seconds
    return lambda: end is None or time.monotonic() < end


def live(reader, seconds):
    keep_going = run_until(seconds)
    while keep_going():
        t0, t1, values = reader.read()
        cells = [f"{v.name}:{v.distance_cm if v.distance_cm is not None else '--':>4} cm"
                 f" OUT1={int(v.out1)} [{v.raw}]" for v in values.values()]
        print(f"{(t1 - t0) * 1000:4.0f} ms  " + "  |  ".join(cells))
        time.sleep(0.5)


def mapping(reader, seconds):
    print("Keep the belt empty for 2 s...")
    base = measure_baseline(reader)
    print("Empty belt:", ", ".join(f"{n}={d} cm" for n, d in base.items()))
    print("Now cover one sensor at a time, starting at the belt start (S4, S3, S1, S2).")
    blocked = {name: False for name in base}
    keep_going = run_until(seconds)
    while keep_going():
        _, _, values = reader.read()
        for v in values.values():
            b = base[v.name]
            if b is None:
                continue
            d = v.distance_cm if v.distance_cm is not None else 0
            if not blocked[v.name] and d < b - BLOCKED_BELOW_CM:
                blocked[v.name] = True
                print(f"  {v.name} (port {v.port}, {v.position_m * 100:.1f} cm from belt start)"
                      f"  BLOCKED  {v.distance_cm} cm")
            elif blocked[v.name] and d >= b - CLEAR_WITHIN_CM:
                blocked[v.name] = False
                print(f"  {v.name} clear")


def baseline(reader, seconds):
    samples = {name: [] for name, _, _ in SENSORS}
    invalid = {name: 0 for name in samples}
    poll_ms = []
    keep_going = run_until(seconds)
    while keep_going():
        t0, t1, values = reader.read()
        poll_ms.append((t1 - t0) * 1000)
        for v in values.values():
            if v.distance_cm is None:
                invalid[v.name] += 1
            else:
                samples[v.name].append(v.distance_cm)

    print(f"{len(poll_ms)} reads in {seconds:.0f} s = {len(poll_ms) / seconds:.1f} reads/s")
    later = sorted(poll_ms[1:]) or poll_ms   # the first read also opens the connection
    print(f"read time: first {poll_ms[0]:.0f} ms, then mean {statistics.mean(later):.0f} ms,"
          f" 95% under {later[int(0.95 * (len(later) - 1))]:.0f} ms, max {later[-1]:.0f} ms")
    for name, _, pos in SENSORS:
        s = samples[name]
        if not s:
            print(f"  {name}: no valid readings ({invalid[name]} invalid)")
            continue
        spread = max(s) - min(s)
        counts = ", ".join(f"{d} cm x{s.count(d)}" for d in sorted(set(s)))
        print(f"  {name} ({pos * 100:5.1f} cm): mean {statistics.mean(s):5.1f}  min {min(s)}"
              f"  max {max(s)}  spread {spread}  invalid {invalid[name]}  [{counts}]"
              f"  {'OK' if spread <= 2 and not invalid[name] else 'CHECK'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["live", "mapping", "baseline"])
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    reader = SensorReader()
    try:
        if args.mode == "live":
            live(reader, args.seconds)
        elif args.mode == "mapping":
            mapping(reader, args.seconds)
        else:
            baseline(reader, args.seconds or 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


if __name__ == "__main__":
    main()
