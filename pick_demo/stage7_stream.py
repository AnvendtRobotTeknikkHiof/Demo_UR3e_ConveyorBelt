"""
Stage 7: a stream of cylinders on a belt that runs all the time.

Put cylinders on at the start of the belt while it runs, one after the other, in the
middle of the belt and at least about 60 cm apart. The robot picks each one, puts it on
the table and goes back above the belt, ready for the next one.

How it works (each pick is the stage 6 pick, with the robot program from stage5_robot.py,
which waits above the pick spot between jobs):
- Every cylinder is followed from S3 to S1 to S2 (S4 is left out: hands are near it while
  you put cylinders on). Between sensors its position is counted in encoder ticks.
- When S1 has seen a cylinder, the laptop works out its start tick and queues it. The job
  goes to the robot as soon as the robot is free and waiting above the belt, if there is
  at least 0.5 s left; otherwise the cylinder is skipped (too close to the one before).
- A cylinder is skipped too when S1 saw it only briefly or far from the middle of the belt
  (2026-10-22: 4-5 cm off the middle, S1 saw it in one reading, the robot came 5 cm late).
- The grip check (output int 28/29) tells when the robot holds a cylinder.
- The belt stops when a cylinder that the robot doesn't hold would come 12 cm past S2
  (counted from the last sensor that saw it), when the robot program stops, when the
  encoder does not count, or when the IO-Link master doesn't answer. Then take every cylinder off the belt, press Enter.

    python pick_demo\\stage7_stream.py                15 minutes
    python pick_demo\\stage7_stream.py --minutes 3

Ctrl+C stops the belt. Results per cylinder: logs/stage7_cylinders_*.csv.
"""
import argparse
import csv
import json
import os
import time
from collections import Counter

import stage2_belt_test as s2
import stage5_pick as s5
import stage6_loop as s6
from iolink_sensors import SENSORS, SensorReader, measure_baseline

TRACKED = ["S3", "S1", "S2"]   # S4 is left out: hands are near it
MIN_LEAD_S = 0.5          # robot waiting above the belt: the job must come this long before
MIN_LEAD_AWAY_S = 2.5     # robot somewhere else (e.g. a PolyScope program): time to get there
MIN_READINGS = 3          # S1 must see a cylinder in at least this many readings ...
MAX_NEAREST_CM = 25       # ... no further away than this (middle of the belt: 21-22 cm)
MATCH_M = 0.25            # a sensor reading belongs to the cylinder predicted within this
SKIPPED, NOT_PICKED = "SKIPPED", "NOT PICKED"


class Cylinder:
    """One cylinder on the belt, from the first sensor that saw it until it is gone."""

    def __init__(self, number):
        self.number = number
        self.seen = {}            # sensor -> tick (centre passing, or front edge for S2)
        self.ref = None           # (centre position in m, tick) of the last sighting
        self.gap_cm = None        # distance to the cylinder before, at S3
        self.speed = None         # belt speed when S1 saw it, m/s (for the log)
        self.start = None         # tick at which the robot starts following
        self.job = None
        self.held = False         # the robot's grip check says it holds it
        self.checked = False
        self.obj = None
        self.skip = None          # why it can't be picked
        self.tick_s2 = None       # tick when S2 saw its front edge
        self.reported = False

    def where(self, tick):
        return self.ref[0] + (tick - self.ref[1]) / s2.TICKS_PER_M


class Stream:
    """Runs the belt and picks cylinders until the time is up or something goes wrong."""

    def __init__(self, belt, reader, base, args, out):
        self.belt, self.reader, self.base, self.args, self.out = belt, reader, base, args, out
        self.pos = {n: p for n, _, p in SENSORS}
        self.stop_at = self.pos["S2"] + (s5.STOP_PAST_S2_CM - s5.RADIUS_CM) / 100
        self.count = 0            # cylinders seen
        self.ready_q = load_pick_above()
        self.reads = []

    # ---- one run of the belt ----

    def run(self, end):
        """Returns None when the time is up and the belt is empty, else why it stopped."""
        self.on_belt, self.queue, self.active = [], [], None
        self.blocked = {n: False for n in TRACKED}
        self.t_in, self.readings, self.nearest = {}, {}, {}
        self.belt.run(s2.FORWARD, self.args.volts)
        self.t_run = time.monotonic()
        why = None
        while why is None:
            why = self.step(end)
        if why == "done":
            self.belt.stop()
            return None
        self.belt.stop()
        self.belt.wait_until_still()
        if self.active is not None and why != s6.ROBOT_STOPPED:
            state, s = s6.wait_for_robot(self.belt, self.active.job)
            self.robot_events(s)
        for c in self.on_belt:
            self.report(c, SKIPPED if c.skip else NOT_PICKED, c.skip or why)
        return why

    def step(self, end):
        try:
            t0, t1, values = self.reader.read()
        except Exception:
            return s6.SENSORS_LOST
        s = self.belt.update()
        now = time.monotonic()
        if s["runtime_state"] != s2.PLAYING:
            return s6.ROBOT_STOPPED
        tm = (t0 + t1) / 2
        self.reads.append((tm, {n: values[n].distance_cm for n in self.pos}))
        for n in TRACKED:
            d = values[n].distance_cm
            if not self.blocked[n] and s6.is_blocked(d, self.base[n]):
                self.blocked[n] = True
                self.t_in[n], self.readings[n], self.nearest[n] = tm, 1, d
                if n == "S2":
                    self.at_s2(tm)
            elif self.blocked[n]:
                if d is not None and d >= self.base[n] - s2.CLEAR_WITHIN_CM:
                    self.blocked[n] = False
                    if n != "S2":
                        self.passed(n, (self.t_in[n] + tm) / 2)
                else:
                    self.readings[n] += 1
                    if d is not None:
                        self.nearest[n] = min(self.nearest[n] or d, d)
        self.robot_events(s)
        self.send_next(s)
        if now - self.t_run > s6.ENCODER_GRACE_S and s2.tick_rate(self.belt.samples) < s6.MIN_TICK_RATE:
            return s6.ENCODER_DEAD
        for c in self.on_belt:
            if c.ref and c.where(s2.tick_of(s)) >= self.stop_at:
                return f"cylinder {c.number} is near the belt end ({c.skip or 'not picked'})"
        if (now > end or (self.args.jobs and self.count >= self.args.jobs)) \
                and not self.on_belt and self.active is None:
            return "done"
        return None

    # ---- sensors ----

    def tick(self, t):
        """Tick count at laptop time t (a sensor reading from the last couple of seconds)."""
        return s2.tick_at(self.belt.samples[-300:], t)

    def match(self, n, tick, at_m):
        """The cylinder on the belt, not yet seen by n, predicted nearest at_m at this tick."""
        best = [c for c in self.on_belt
                if n not in c.seen and c.ref and abs(c.where(tick) - at_m) < MATCH_M]
        return min(best, key=lambda c: abs(c.where(tick) - at_m)) if best else None

    def new_cylinder(self):
        self.count += 1
        c = Cylinder(self.count)
        self.on_belt.append(c)
        return c

    def passed(self, n, t):
        """The centre of a cylinder passed S3 or S1 at time t."""
        tick = self.tick(t)
        c = self.match(n, tick, self.pos[n])
        if c is None:
            if n == "S3" and self.args.jobs and self.count >= self.args.jobs:
                print(f"  (a cylinder after the last of {self.args.jobs} passed S3: take it off)")
            c = self.new_cylinder()
            if n == "S3" and len(self.on_belt) > 1:
                c.gap_cm = (self.on_belt[-2].where(tick) - self.pos["S3"]) * 100
            gap = f", {c.gap_cm:.0f} cm after the one before" if c.gap_cm else ""
            rate = s2.tick_rate(self.belt.samples)
            if c.gap_cm and rate > 0:
                gap += f" ({c.gap_cm / 100 * s2.TICKS_PER_M / rate:.1f} s)"
            print(f"\n  Cylinder {c.number} seen by {n}{gap}")
        c.seen[n] = tick
        c.ref = (self.pos[n], tick)
        if n == "S3" and self.nearest[n] and self.nearest[n] > MAX_NEAREST_CM:
            print(f"  Cylinder {c.number}: {self.nearest[n]} cm at S3, off the middle of the belt")
        if n != "S1":
            return
        c.speed = s2.tick_rate(self.belt.samples) / s2.TICKS_PER_M
        if self.readings["S1"] < MIN_READINGS or (self.nearest["S1"] or 99) > MAX_NEAREST_CM:
            c.skip = (f"S1 saw it badly ({self.readings['S1']} reading(s), nearest"
                      f" {self.nearest['S1']} cm): off the middle of the belt?")
            print(f"  Cylinder {c.number} will not be picked: {c.skip}")
            return
        c.start = tick + (s5.PICK_CM / 100 + self.args.offset_mm / 1000 - self.pos["S1"]) \
            * s2.TICKS_PER_M
        self.queue.append(c)

    def at_s2(self, t):
        """S2 sees the front edge of a cylinder (its centre is 3 cm before S2)."""
        front = self.pos["S2"] - s5.RADIUS_CM / 100
        tick = self.tick(t)
        c = self.match("S2", tick, front)
        if c is None:
            if self.active is not None:
                return        # the gripper or the cylinder it is lifting, not a new object
            c = self.new_cylinder()
            c.skip = "unknown object at S2"
        c.seen["S2"] = tick
        c.ref = (front, tick)
        c.tick_s2 = tick

    # ---- robot ----

    def robot_events(self, s):
        c = self.active
        if c is None:
            return
        if not c.checked and s["output_int_register_29"] == c.job:
            c.checked, c.obj = True, s["output_int_register_28"]
            if c.obj == s6.OBJ_HELD:
                c.held = True
                self.on_belt.remove(c)
        if s["output_int_register_27"] == c.job:
            # not held: it stays on the belt, and the belt stops before it reaches the end
            self.report(c, s6.PICKED if c.held else s6.MISSED if c.checked else s6.NO_CHECK)
            self.active = None

    def send_next(self, s):
        # the robot is free when it finished the last job, or waits for a parked one
        free = self.belt.job in (s["output_int_register_27"], self.belt.parked)
        if self.active is not None or not self.queue or not free:
            return
        c = self.queue.pop(0)
        ready = self.ready_q and max(abs(a - b) for a, b in zip(s["actual_q"], self.ready_q)) < 0.02
        rate = s2.tick_rate(self.belt.samples)
        lead = (c.start - s2.tick_of(s)) / rate if rate > 0 else 0.0
        if lead < (MIN_LEAD_S if ready else MIN_LEAD_AWAY_S):
            c.skip = f"the robot was free {-lead:.1f} s too late: too close to the one before" \
                if lead < 0 else "the robot was free too late: too close to the one before"
            print(f"  Cylinder {c.number} will not be picked: {c.skip}")
            return
        c.job = self.belt.send_job(c.start)
        self.active = c
        print(f"  Cylinder {c.number}: job {c.job} sent {lead:.1f} s before the pick spot")

    # ---- results ----

    def report(self, c, result, why=""):
        if c.reported:
            return
        c.reported = True
        centre = None
        if c.start is not None and c.tick_s2 is not None:
            centre = (self.pos["S2"] * 100 - s5.RADIUS_CM
                      - (c.tick_s2 - c.start) / s2.TICKS_PER_M * 100)
        self.out(c, result, why, centre, c.speed)


def load_pick_above():
    """Joint angles of the pose where stage5_robot.py waits between jobs, if recorded."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stage5_poses.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)["pick_above"]["q"]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=15, help="how long (default 15)")
    ap.add_argument("--jobs", type=int, default=0, help="stop after this many cylinders")
    ap.add_argument("--volts", type=float, default=1.0, help="belt voltage (default 1.0)")
    ap.add_argument("--offset-mm", type=float, default=s5.DEFAULT_OFFSET_MM,
                    help=f"as stage 5 (default {s5.DEFAULT_OFFSET_MM})")
    args = ap.parse_args()

    belt = s2.Belt()
    reader = SensorReader()
    order = [n for n, _, _ in sorted(SENSORS, key=lambda x: x[2])]
    os.makedirs(s2.LOG_DIR, exist_ok=True)
    path = os.path.join(s2.LOG_DIR, f"stage7_cylinders_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    log_file = open(path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["cylinder", "clock", "result", "obj", "robot_job", "gap_cm", "speed_cm_s",
                  "centre_at_start_cm", "why"])
    counts = Counter()

    def out(c, result, why, centre, v):
        counts[result] += 1
        what = f" ({s6.OBJ_TEXT.get(c.obj, 'OBJ ' + str(c.obj))})" if c.obj is not None else ""
        print(f"  -> Cylinder {c.number}: {result}{what}" + (f", {why}" if why else "")
              + (f", centre at {centre:.1f} cm when the robot started following" if centre else "")
              + f".  So far {counts[s6.PICKED]} picked of {sum(counts.values())}.")
        log.writerow([c.number, time.strftime("%H:%M:%S"), result, c.obj, c.job,
                      f"{c.gap_cm:.0f}" if c.gap_cm else "", f"{v * 100:.2f}" if v else "",
                      f"{centre:.1f}" if centre else "", why])
        log_file.flush()

    t_begin = time.monotonic()
    end = t_begin + args.minutes * 60
    stream = Stream(belt, reader, None, args, out)
    finished = False
    try:
        if not belt.program_running():
            raise SystemExit("The robot program is not running. Start it: python pick_demo\\"
                             "stage5_robot.py start")
        if stream.ready_q is None:
            print("No stage5_poses.json: the robot gets every job 2.5 s early (fewer cylinders).")
        belt.park_job()
        belt.stop()
        belt.wait_until_still()
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        stream.base = measure_baseline(reader)
        missing = [m for m, b in stream.base.items() if b is None]
        if missing:
            raise SystemExit(f"No valid reading from {', '.join(missing)}.")
        while True:
            if not belt.program_running():
                s6.wait_for_play(belt)
            _, _, values = reader.read()
            busy = s6.busy_sensors(values, stream.base, order)
            if busy:
                print(f"Something is in front of {', '.join(busy)}: take it off the belt.")
                input("Then press Enter to start the belt ... ")
                continue
            print(f"\nBelt running for {max(0, end - time.monotonic()) / 60:.1f} more minutes."
                  " Put cylinders on at the start of the belt, in the middle, about 60 cm apart.")
            why = stream.run(end)
            if why is None:
                break
            belt.park_job()
            print(f"\nBelt stopped: {why}.")
            print(s6.WHAT_TO_DO.get(why, "Take every cylinder off the belt."))
            input("Then press Enter to start the belt again (Ctrl+C ends) ... ")
        finished = True
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    except (OSError, RuntimeError) as e:
        print(f"\n{e}")
    finally:
        if belt.moved:
            try:
                belt.stop()
                belt.park_job()
            except Exception as e:
                print(f"!! Could not stop the belt ({e}). Set conveyor_stop High in the I/O tab.")
        reader.close()
        log_file.close()
        if belt.samples:
            print(f"Log: {belt.save('stage7_stream')}")
        if stream.reads:
            print(f"Sensor log: {s2.save_reads(stream.reads, order, 'stage7_stream_sensors')}")
        belt.close()

    minutes = (time.monotonic() - t_begin) / 60
    total = sum(counts.values())
    print(f"\n{total} cylinder(s) in {minutes:.1f} minutes: "
          + (", ".join(f"{c} {res}" for res, c in counts.most_common()) or "none"))
    print(f"Results per cylinder: {path}")
    if finished and not args.jobs and minutes >= args.minutes:
        print(f"PASS: {args.minutes:g} minutes, every cylinder picked." if total and
              counts[s6.PICKED] == total else
              f"Not a pass yet: {total - counts[s6.PICKED]} of {total} not picked (see above).")


if __name__ == "__main__":
    main()
