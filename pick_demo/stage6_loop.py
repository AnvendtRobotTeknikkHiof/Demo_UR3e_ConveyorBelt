"""
Stage 6: pick loop. The robot picks one cylinder after the other for 15 minutes; the
laptop reports every cylinder and keeps a count.

Each pick works like stage 5 (same pick spot and --offset-mm). New in stage 6:
- Grip check: after lifting, the robot asks the gripper whether it holds the cylinder
  (robot/pick_grip_check.script): output int 28 = OBJ (2 = held), int 29 = job number.
  On a miss the robot still goes to the table and opens the empty gripper there.
- One result line per cylinder (PICKED, MISSED, ...) with a running count, also written
  to logs/stage6_jobs_*.csv.
- The belt stops when S2 no longer sees the cylinder, when the robot reports its grip
  check, or at the latest when the cylinder is 12 cm past S2 (predicted from the last
  sensor that saw it, so also when S2 never sees it). If the robot program stops, the
  laptop sets conveyor_stop directly.
- After anything but PICKED the belt stays stopped and the laptop says what to do.
- --auto: no Enter per cylinder. The belt starts when a cylinder has stood still in front
  of S4, in the middle of the belt, for 2 s (later robot #1 puts it there).
- A belt that runs all the time with several cylinders on it: stage7_stream.py.

    python pick_demo\\stage6_loop.py --grip                     read-only: what does the gripper hold?
    python pick_demo\\stage6_loop.py --jobs 1                   one cylinder, Enter to start
    python pick_demo\\stage6_loop.py --jobs 1 --offset-mm 30    a deliberate miss
    python pick_demo\\stage6_loop.py --auto                     15 minutes, starts by itself

Use one cylinder: after each pick take it from the table and put it upright in the middle
of the belt in front of S4 (19.5 cm). Ctrl+C stops the belt.
"""
import argparse
import csv
import os
import socket
import time
from collections import Counter

import stage2_belt_test as s2
import stage5_pick as s5
from iolink_sensors import SENSORS, SensorReader, measure_baseline
from rtde_link import ROBOT_IP

GRIPPER_PORT = 63352      # Robotiq URCap gripper server (text commands, GET OBJ)
OBJ_HELD = 2
OBJ_TEXT = {-1: "the robot could not reach the gripper server", 0: "no answer from the gripper",
            1: "stopped while opening", 2: "cylinder held", 3: "closed on nothing"}
SETTLE_S = 2.0            # --auto: the cylinder must stand still in front of S4 this long
CENTRED_CM = (19, 22)     # --auto: S4 reading with the cylinder in the middle (stage 3: 20-21)
SEEN_WITHIN_S = 15        # stop the belt if no sensor has seen the cylinder by then

PICKED, MISSED, NO_PICK, NO_CHECK = "PICKED", "MISSED", "NO PICK", "NO GRIP CHECK"
ROBOT_STOPPED, NO_ANSWER, SENSORS_LOST = "ROBOT STOPPED", "NO ANSWER", "SENSORS LOST"
WHAT_TO_DO = {
    MISSED: "The gripper came up empty. Take the cylinder off the belt.",
    NO_PICK: "The robot did not try. Take the cylinder off the belt.",
    NO_CHECK: "The robot finished without a grip check: is pick_grip_check.script in the"
              " program (pick_stage6)? Take the cylinder off the belt or the table.",
    ROBOT_STOPPED: "The robot program stopped: take a photo of the pendant message. Unlock"
                   " it, move the gripper up and away, take the cylinder off, start it again.",
    NO_ANSWER: f"The robot did not finish within {s5.DONE_TIMEOUT_S} s: look at the pendant."
               " Take the cylinder off the belt.",
    SENSORS_LOST: "The IO-Link master did not answer, so the belt stopped at once. Check the"
                  " network cable. Take the cylinder off the belt.",
}
JOB_COLUMNS = ["cylinder", "clock", "result", "obj", "robot_job", "speed_cm_s", "lead_s",
               "centre_at_start_cm", "belt_stopped_because", "seconds"]


def is_blocked(distance_cm, base_cm):
    d = 0 if distance_cm is None else distance_cm      # no valid distance: very close
    return d < base_cm - s2.BLOCKED_BELOW_CM


def run_belt(belt, reader, base, order, pos, volts, offset_mm, reads):
    """Run the belt with one cylinder on it, send the pick job when S1 has seen it, and
    stop the belt in time. Returns a dict for the report; the belt is stopped."""
    last = order[-1]
    stop_at = pos[last] + (s5.STOP_PAST_S2_CM - s5.RADIUS_CM) / 100   # centre, m
    guess = s2.belt_speed_guess(volts)
    info = {"job": None, "speed": None, "lead": None, "start": None, "why": None}
    try:
        _, _, values = reader.read()
    except Exception:
        info["why"] = SENSORS_LOST
        return info
    # A cylinder already standing in front of S4 (--auto) can't be timed there; the
    # speed is then measured from S3.
    blocked = {n: is_blocked(values[n].distance_cm, base[n]) for n in order}
    enter, leave, centre = {}, {}, {}

    belt.set_belt_speed(guess)
    belt.update()
    i0 = len(belt.samples)
    belt.run(s2.FORWARD, volts)
    t_start = time.monotonic()
    # (centre position in m, laptop time) when a sensor last placed the cylinder
    stood = blocked[order[0]]
    ref = (pos[order[0]], t_start) if stood else None
    v = None                  # belt speed measured by the sensors, m/s
    rough = None              # speed from the belt start, if it stood in front of S4
    while True:
        try:
            t0, t1, values = reader.read()
        except Exception:
            info["why"] = SENSORS_LOST     # without the sensors: stop at once
            break
        s = belt.update()
        now = time.monotonic()
        if s["runtime_state"] != s2.PLAYING:
            info["why"] = ROBOT_STOPPED
            break
        tm = (t0 + t1) / 2        # the sample was taken somewhere in the request
        reads.append((tm, {n: values[n].distance_cm for n in order}))
        for n in order:
            if n in leave:
                continue
            d = values[n].distance_cm
            if not blocked[n] and is_blocked(d, base[n]):
                blocked[n] = True
                enter[n] = tm
                if n == last:
                    ref = (pos[n] - s5.RADIUS_CM / 100, tm)    # S2 sees the front edge
            elif blocked[n] and d is not None and d >= base[n] - s2.CLEAR_WITHIN_CM:
                blocked[n] = False
                leave[n] = tm
                if n not in enter:
                    continue      # it stood there at the start: no timing
                centre[n] = (enter[n] + tm) / 2
                ref = (pos[n], centre[n])
                timed = [m for m in order if m in centre and m != n]
                if timed and pos[n] <= pos[s5.TRIGGER]:
                    v = (pos[n] - pos[timed[0]]) / (centre[n] - centre[timed[0]])
                    belt.set_belt_speed(v)
                    info["speed"] = v
                elif not timed and stood:
                    # only for the end check until S1 times it (a few % low: start-up)
                    rough = (pos[n] - pos[order[0]]) / (centre[n] - t_start)
                    belt.set_belt_speed(rough)
                if n == s5.TRIGGER and v:
                    send_job(belt, info, pos[n], centre[n], v, offset_mm)

        job = info["job"]
        if last in leave:
            info["why"] = f"{last} no longer sees the cylinder"
            break
        if job is not None and s["output_int_register_29"] == job:
            info["why"] = "the robot has checked its grip"
            break
        if job is not None and s["output_int_register_27"] == job:
            info["why"] = "the robot has finished"
            break
        if ref is not None:
            # until a speed is measured, assume a faster belt: rough + 25 %, or twice the guess
            v_end = v or (1.25 * rough if rough else 2 * guess)
            if ref[0] + v_end * (now - ref[1]) >= stop_at:
                info["why"] = "the cylinder is near the belt end"
                break
        elif now - t_start > SEEN_WITHIN_S:
            info["why"] = "no sensor saw the cylinder"
            break
        if now - t_start > 120:
            info["why"] = "took longer than 2 minutes"
            break
    belt.stop()
    belt.wait_until_still()

    if info["job"] is not None and last in enter:
        # where was the cylinder centre at the start tick, measured by S2?
        tick_enter = s2.tick_at(belt.samples[i0:], enter[last])
        info["centre_at_start"] = (pos[last] * 100 - s5.RADIUS_CM
                                   - (tick_enter - info["start"]) / s2.TICKS_PER_M * 100)
    if info["job"] is None and "late" not in info and info["why"] not in (ROBOT_STOPPED,
                                                                         SENSORS_LOST):
        info["no_job"] = ("the robot was still busy with the last job" if "busy" in info
                          else f"{s5.TRIGGER} did not see the whole cylinder"
                          if s5.TRIGGER not in centre else "no belt speed measured before "
                          + s5.TRIGGER)
    return info


def send_job(belt, info, sensor_m, t_centre, v, offset_mm):
    """The cylinder centre passed sensor_m (m) at t_centre: ask the robot to start
    following when it is at the pick spot (+ offset). Counted from the newest tick, so it
    doesn't matter how fast the software encoder counted before; from now on it counts
    at v (stage 5 counted from the tick at t_centre: the same when the speed was right)."""
    belt.update()
    s = belt.samples[-1]
    if s["output_int_register_27"] != belt.job and belt.parked != belt.job:
        # still busy with the last job: it would be late (a parked job is different: the
        # robot waits above the belt and this job releases it)
        info["busy"] = True
        return
    here = sensor_m + v * (s["_t"] - t_centre)          # cylinder centre now, m
    tick_now = s2.tick_of(s)
    lead = (s5.PICK_CM / 100 + offset_mm / 1000 - here) / v
    if lead < s5.MIN_LEAD_S:
        info["late"] = lead
        return
    info["start"] = tick_now + lead * v * s2.TICKS_PER_M
    info["job"] = belt.send_job(info["start"])
    info["lead"] = lead


def wait_for_robot(belt, job):
    """Wait until the robot reports the job done. Returns (state, newest sample)."""
    end = time.monotonic() + s5.DONE_TIMEOUT_S
    while True:
        s = belt.update()
        if s["output_int_register_27"] == job:
            return "done", s
        if s["runtime_state"] != s2.PLAYING:
            return "stopped", s
        if time.monotonic() > end:
            return "timeout", s
        time.sleep(0.05)


def one_job(belt, reader, base, order, pos, args, reads):
    """One cylinder from the belt start to a result. Returns a dict for the report."""
    t0 = time.monotonic()
    info = run_belt(belt, reader, base, order, pos, args.volts, args.offset_mm, reads)
    if info["speed"]:
        print(f"  belt speed {info['speed'] * 100:.2f} cm/s")
    if info["job"] is not None:
        print(f"  job {info['job']} sent {info['lead']:.1f} s before the cylinder reached"
              f" {s5.PICK_CM:.0f} cm")
    if "centre_at_start" in info:
        print(f"  when the robot started following, the cylinder centre was at about"
              f" {info['centre_at_start']:.1f} cm by S2")
    print(f"  belt stopped: {info['why']}")

    obj = None
    if info["why"] in (ROBOT_STOPPED, SENSORS_LOST):
        result = info["why"]
    elif info["job"] is None:
        result = NO_PICK
        if "late" in info:
            print(f"  too late to send the job ({info['late']:.1f} s before the pick spot)")
        else:
            print(f"  {info.get('no_job', 'no job sent')}")
    else:
        state, s = wait_for_robot(belt, info["job"])
        if state == "stopped":
            result = ROBOT_STOPPED
        elif state == "timeout":
            result = NO_ANSWER
        elif s["output_int_register_29"] != info["job"]:
            result = NO_CHECK
        else:
            obj = s["output_int_register_28"]
            result = PICKED if obj == OBJ_HELD else MISSED
    return {"result": result, "obj": obj, "robot_job": info["job"], "speed": info["speed"],
            "lead": info["lead"], "centre_at_start": info.get("centre_at_start"),
            "why": info["why"], "seconds": time.monotonic() - t0}


def wait_for_play(belt):
    """Wait until the robot program runs again (Play on the pendant)."""
    print("  The robot program is not running. Start it again: python pick_demo\\"
          "stage5_robot.py start (or Play pick_stage6 on the pendant) ...")
    while not belt.program_running():
        belt.update()
        time.sleep(0.4)
    print("  Robot program running.")


def busy_sensors(values, base, names):
    return [n for n in names if is_blocked(values[n].distance_cm, base[n])]


def wait_for_cylinder(belt, reader, base, order, end):
    """--auto: wait until a cylinder has stood still in front of the first sensor (S4),
    in the middle of the belt, for SETTLE_S, with the other sensors clear. False when
    the time is up first."""
    first = order[0]
    said, since, last_d = None, None, None
    while time.monotonic() < end:
        s = belt.update()
        if s["runtime_state"] != s2.PLAYING:
            wait_for_play(belt)
            said = None
        try:
            _, _, values = reader.read()
        except Exception as e:
            msg, since = f"The IO-Link master does not answer ({e}). Retrying ...", None
            time.sleep(1.0)
        else:
            d = values[first].distance_cm
            busy = busy_sensors(values, base, order[1:])
            if busy:
                msg, since = f"Something is in front of {', '.join(busy)}: take it off the belt.", None
            elif not is_blocked(d, base[first]):
                msg, since = f"Waiting for a cylinder in front of {first} ...", None
            elif d is None or not CENTRED_CM[0] <= d <= CENTRED_CM[1]:
                msg, since = (f"{first} reads {d} cm: put the cylinder in the middle of the belt"
                              f" ({CENTRED_CM[0]}-{CENTRED_CM[1]} cm)."), None
            else:
                msg = f"Cylinder in front of {first}. Starting when it has stood still {SETTLE_S:g} s ..."
                if d != last_d or since is None:
                    since = time.monotonic()
                elif time.monotonic() - since >= SETTLE_S:
                    return True
            last_d = d
        if msg != said:
            print(f"  {msg}")
            said = msg
    return False


def grip_status():
    """Read-only: ask the Robotiq URCap's gripper server on the robot what the gripper does."""
    try:
        with socket.create_connection((ROBOT_IP, GRIPPER_PORT), timeout=2.0) as sock:
            for name in ("STA", "OBJ", "POS"):
                sock.sendall(f"GET {name}\n".encode())
                print(f"  {sock.recv(64).decode(errors='replace').strip()}")
    except OSError as e:
        print(f"No answer from the gripper server on {ROBOT_IP}:{GRIPPER_PORT} ({e}).")
        return
    print("STA 3 = activated. OBJ 2 = holds an object, 3 = closed on nothing, 0 = moving."
          " POS 0 = open, 255 = closed.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--auto", action="store_true",
                    help="start the belt when a cylinder stands in front of S4 (no Enter)")
    ap.add_argument("--minutes", type=float, default=15, help="how long (default 15)")
    ap.add_argument("--jobs", type=int, default=0, help="stop after this many cylinders")
    ap.add_argument("--volts", type=float, default=1.0, help="belt voltage (default 1.0)")
    ap.add_argument("--offset-mm", type=float, default=s5.DEFAULT_OFFSET_MM,
                    help=f"as stage 5 (default {s5.DEFAULT_OFFSET_MM})")
    ap.add_argument("--grip", action="store_true",
                    help="read-only: ask the gripper what it holds, then stop")
    args = ap.parse_args()
    if args.grip:
        grip_status()
        return

    order = [n for n, _, _ in sorted(SENSORS, key=lambda s: s[2])]     # along the belt
    pos = {n: p for n, _, p in SENSORS}
    belt = s2.Belt()
    reader = SensorReader()
    os.makedirs(s2.LOG_DIR, exist_ok=True)
    jobs_path = os.path.join(s2.LOG_DIR, f"stage6_jobs_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    jobs_file = open(jobs_path, "w", newline="")
    jobs_log = csv.writer(jobs_file)
    jobs_log.writerow(JOB_COLUMNS)
    reads, counts, n = [], Counter(), 0
    t_begin = time.monotonic()
    end = t_begin + args.minutes * 60
    finished = False
    try:
        if not belt.program_running():
            raise SystemExit("The robot program is not running. Start it: python pick_demo\\"
                             "stage5_robot.py start (or Play pick_stage6 on the pendant).")
        belt.park_job()
        belt.stop()
        belt.wait_until_still()
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        base = measure_baseline(reader)
        missing = [m for m, b in base.items() if b is None]
        if missing:
            raise SystemExit(f"No valid reading from {', '.join(missing)}.")
        print(f"Running for {args.minutes:g} minutes" + (f" or {args.jobs} cylinders" if args.jobs
                                                         else "") + ". Ctrl+C stops.")

        while time.monotonic() < end and not (args.jobs and n >= args.jobs):
            print(f"\nCylinder {n + 1}: put it upright in the middle of the belt in front of"
                  " S4 (19.5 cm).")
            if args.auto:
                if not wait_for_cylinder(belt, reader, base, order, end):
                    break
            else:
                input("Press Enter to start the belt ... ")
                if not belt.program_running():
                    wait_for_play(belt)
                try:
                    _, _, values = reader.read()
                except Exception as e:
                    print(f"  The IO-Link master does not answer ({e}). Check the network cable.")
                    continue
                busy = busy_sensors(values, base, order[1:])
                if busy:
                    print(f"  Something is in front of {', '.join(busy)}: take it off the belt.")
                    continue
            n += 1
            r = one_job(belt, reader, base, order, pos, args, reads)
            counts[r["result"]] += 1
            what = f" ({OBJ_TEXT.get(r['obj'], 'OBJ ' + str(r['obj']))})" if r["obj"] is not None else ""
            others = n - counts[PICKED] - counts[MISSED]
            print(f"  -> {r['result']}{what}.  So far {counts[PICKED]} picked, {counts[MISSED]}"
                  f" missed, {others} other, of {n}.")
            jobs_log.writerow([n, time.strftime("%H:%M:%S"), r["result"], r["obj"], r["robot_job"],
                               f"{r['speed'] * 100:.2f}" if r["speed"] else "",
                               f"{r['lead']:.1f}" if r["lead"] else "",
                               f"{r['centre_at_start']:.1f}" if r["centre_at_start"] else "",
                               r["why"], f"{r['seconds']:.1f}"])
            jobs_file.flush()
            if r["result"] != PICKED:
                belt.park_job()
                print(f"  {WHAT_TO_DO[r['result']]}")
                if args.auto:
                    input("  Then press Enter to go on (Ctrl+C ends the loop) ... ")
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
        jobs_file.close()
        if belt.samples:
            print(f"Log: {belt.save('stage6_loop')}")
        if reads:
            print(f"Sensor log: {s2.save_reads(reads, order, 'stage6_loop_sensors')}")
        belt.close()

    minutes = (time.monotonic() - t_begin) / 60
    print(f"\n{n} cylinder(s) in {minutes:.1f} minutes: "
          + (", ".join(f"{c} {res}" for res, c in counts.most_common()) or "none"))
    print(f"Results per cylinder: {jobs_path}")
    if finished and not args.jobs and minutes >= args.minutes:
        if n and counts[PICKED] == n:
            print(f"PASS: {args.minutes:g} minutes, every cylinder picked.")
        else:
            print(f"Not a pass yet: {n - counts[PICKED]} of {n} cylinder(s) not picked (see above).")


if __name__ == "__main__":
    main()
