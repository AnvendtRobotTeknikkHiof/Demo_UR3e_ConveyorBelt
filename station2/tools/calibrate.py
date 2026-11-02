"""
Measure encoder ticks per metre: an item runs from S4 to S2 (1.62 m, tape-measured
sensor positions), ticks per metre = ticks between them / distance.
Needs the robot program running (python station2/station.py start). The arm does not move.

    python station2/tools/calibrate.py            3 runs
    python station2/tools/calibrate.py --runs 5

Pass: the runs agree within 1 %. Then put the mean in config.TICKS_PER_M (and in
Installation -> Conveyor Tracking).
"""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                       # noqa: E402
from iolink import Edge                             # noqa: E402
from station import FORWARD, STOP, Station2         # noqa: E402

FIRST, LAST = "S4", "S2"


def one_run(station):
    edges = {n: Edge(station.empty[n]) for n in (FIRST, LAST)}
    passed = {}
    station.belt(FORWARD, wait=True)
    t_start = time.monotonic()
    try:
        while LAST not in passed:
            t, values = station.sensors.read()
            station.check_running()
            for n, edge in edges.items():
                p = edge.update(t, values[n])
                if p and n not in passed:
                    passed[n] = (p[0], station.link.tick_at(p[0]))
            if time.monotonic() - t_start > 30:
                print("  The item did not pass both sensors within 30 s; run ignored.")
                return None
    finally:
        station.belt(STOP, wait=True)
    if FIRST not in passed:
        print(f"  {FIRST} did not see the item; run ignored.")
        return None
    (t0, k0), (t1, k1) = passed[FIRST], passed[LAST]
    dist = config.SENSORS[LAST][1] - config.SENSORS[FIRST][1]
    print(f"  {(k1 - k0) / dist:.0f} ticks/m, belt {dist / (t1 - t0) * 100:.1f} cm/s")
    return (k1 - k0) / dist


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=3)
    args = ap.parse_args()
    station = Station2()
    results = []
    try:
        station.belt(STOP, wait=True)
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        station.empty = station.sensors.baseline()
        for run in range(1, args.runs + 1):
            input(f"\nRun {run}: put the item upright in the middle of the belt before {FIRST},"
                  " stand at the belt end, press Enter ... ")
            r = one_run(station)
            if r:
                results.append(r)
    finally:
        station.close()
    if results:
        mean = statistics.mean(results)
        spread = (max(results) - min(results)) / mean * 100
        print(f"\nticks/m: {', '.join(f'{r:.0f}' for r in results)} -> mean {mean:.0f},"
              f" spread {spread:.1f} %  ({'PASS' if spread <= 1 else 'not within 1 %'})."
              f" config.TICKS_PER_M = {config.TICKS_PER_M:.0f}")


if __name__ == "__main__":
    main()
