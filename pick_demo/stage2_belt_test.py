"""
Stage 2: measure the belt. The arm does not move. The robot program
(robot/pick_belt_server.script) switches the belt outputs when the laptop asks.

    python stage2_belt_test.py check          read-only: program, heartbeat, belt, sensors
    python stage2_belt_test.py speed          belt EMPTY: speed at several voltages, start
                                              delay, stop distance, hiccups
    python stage2_belt_test.py start-pause    belt EMPTY: does the belt start with no pause
                                              between conveyor_stop Low and the start pulse?
    python stage2_belt_test.py calibrate      ticks per metre: the cylinder passes the four
                                              sensors (tape-measured positions)
    python stage2_belt_test.py watch          belt EMPTY: run the belt (30 s) and show the
                                              encoder speed live, to watch the wheel
    python stage2_belt_test.py stop           stop the belt

Ctrl+C stops the belt. Each run is logged to logs/ as CSV (125 samples per second).
"""
import argparse
import bisect
import csv
import os
import statistics
import time

from iolink_sensors import SENSORS, SensorReader, measure_baseline
from rtde_link import PROGRAM_STATES, ROBOT_IP, RTDELink

TICKS_PER_M = 1825.0      # Installation -> Conveyor Tracking; "calibrate" checks it
FORWARD, BACKWARD, STOP = 1, 2, 3
PLAYING = 2               # runtime_state
FWD_BIT, BWD_BIT, STOP_BIT = 5, 6, 7

EMPTY_MIN_CM = 45         # an empty belt reads 52-54 cm (stage 1)
BLOCKED_BELOW_CM = 5      # object present: this much closer than the empty belt
CLEAR_WITHIN_CM = 2       # object gone: back within this of the empty belt
STOP_PAST_S2_CM = 8       # calibrate: stop at the latest when the object is this far past S2
RUN_S = 6.0               # speed: seconds per run
PARKED_TICK = 1e9         # a start tick the software encoder never reaches (~85 days at 1 V)

OUTPUTS = ["timestamp", "runtime_state", "actual_digital_output_bits", "standard_analog_output1",
           "output_int_register_24", "output_int_register_26", "output_double_register_24",
           "output_int_register_27",       # last pick job the robot finished (stage 5)
           "output_int_register_28",       # gripper OBJ at the grip check (stage 6, 2 = held)
           "output_int_register_29",       # job number of that grip check (stage 6)
           "actual_q",                     # joints: is the robot waiting above the belt? (stage 7)
           "actual_TCP_pose"]              # where the gripper is (logged)
INPUTS = ["input_int_register_25", "input_int_register_26",
          "input_double_register_24", "input_double_register_25",
          "input_double_register_26",      # belt speed (m/s) for the software encoder
          "input_int_register_27",         # pick job number (stage 5)
          "input_double_register_27"]      # tick at which the robot starts following the belt
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def belt_speed_guess(volts):
    """Belt speed in m/s before a cylinder has been timed. At 1.0 V the sensors measured
    11.9 cm/s on 2026-10-16 but 7.3 cm/s on 2026-10-21, so every cylinder is timed and
    this is only a starting value (the latest measurement). The shape over voltage comes
    from the encoder while it rolled (31 + 105 x V ticks/s)."""
    return 0.0733 * (31 + 105 * volts) / 136


def tick_of(s):
    return s["output_double_register_24"]


def bit(s, n):
    return (s["actual_digital_output_bits"] >> n) & 1


def fmt(value, spec, unit=""):
    return "-" if value is None else f"{value:{spec}}{unit}"


def tick_at(samples, t):
    """Tick count at laptop time t (time.monotonic()), interpolated between samples."""
    lt = [s["_t"] for s in samples]
    i = bisect.bisect_left(lt, t)
    if i <= 0:
        return tick_of(samples[0])
    if i >= len(samples):
        return tick_of(samples[-1])
    s0, s1 = samples[i - 1], samples[i]
    f = (t - s0["_t"]) / (s1["_t"] - s0["_t"])
    return tick_of(s0) + f * (tick_of(s1) - tick_of(s0))


class Belt:
    """Belt commands through the robot program, and a log of every RTDE sample."""

    def __init__(self):
        self.link = RTDELink(ROBOT_IP, outputs=OUTPUTS, inputs=INPUTS, keep_history=True)
        self.samples = []
        s = self.link.latest()
        self.seq = s["output_int_register_26"]
        self.job = s["output_int_register_27"]     # last pick job the robot finished
        self.moved = False
        self.parked = None        # last job given up with park_job()
        self.belt_speed = None    # m/s for the software encoder; None = guess from volts
        self._sent = False

    def _send(self, **values):
        # Every package carries all inputs. In the first one keep the job number at the
        # robot's last finished job, or a stage 5 program would see "new job 0", and the
        # start tick out of reach, or a robot still waiting for an earlier job's start
        # tick would go down at once (2026-10-22: an empty pick when a new run connected).
        if not self._sent:
            values.setdefault("input_int_register_27", self.job)
            values.setdefault("input_double_register_27", PARKED_TICK)
            self._sent = True
        self.link.send(**values)

    def update(self):
        """Collect new samples and return the newest."""
        self.samples += self.link.drain()
        return self.samples[-1] if self.samples else self.link.latest()

    def program_running(self):
        """True when the program plays and its heartbeat moves."""
        before = self.link.latest()["output_int_register_24"]
        time.sleep(0.1)
        s = self.link.latest()
        return s["runtime_state"] == PLAYING and s["output_int_register_24"] != before

    def set_belt_speed(self, m_per_s):
        """Tell the software encoder how fast the belt runs (no belt command)."""
        self.belt_speed = m_per_s
        self._send(input_int_register_26=self.seq, input_double_register_26=float(m_per_s))

    def send_job(self, start_tick):
        """Stage 5: ask the robot to pick, following the belt from start_tick.
        Returns the job number; the robot copies it to output int 27 when done."""
        self.job += 1
        self._send(input_int_register_26=self.seq, input_double_register_27=float(start_tick),
                   input_int_register_27=self.job)
        return self.job

    def park_job(self):
        """Stage 6: set the start tick out of reach. A robot that still waits for the start
        of a job the laptop gave up on keeps waiting (instead of starting at once, e.g.
        when a new laptop session sends 0); the next job's start tick releases it."""
        self._send(input_int_register_26=self.seq, input_double_register_27=PARKED_TICK)
        self.parked = self.job

    def command(self, cmd, volts=0.0, pause=0.0, timeout=3.0):
        self.seq += 1
        speed = self.belt_speed
        if speed is None:
            speed = belt_speed_guess(volts) if cmd in (FORWARD, BACKWARD) else 0.0
        self._send(input_int_register_25=cmd, input_int_register_26=self.seq,
                   input_double_register_24=float(volts),
                   input_double_register_25=float(pause),
                   input_double_register_26=float(speed))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.update()
            if s["output_int_register_26"] == self.seq:
                return
            if s["runtime_state"] != PLAYING:
                raise RuntimeError("the robot program is not running")
            time.sleep(0.01)
        raise TimeoutError(f"the robot program did not confirm the belt command within {timeout} s"
                           " (is pick_belt_server.script the main program node?)")

    def run(self, direction, volts, pause=0.2):
        self.moved = True
        self.command(direction, volts, pause)

    def stop(self):
        try:
            self.command(STOP)
        except RuntimeError:
            # The robot program has stopped, so its belt thread can't help.
            if not stop_belt_directly():
                raise
            print("Robot program not running: set conveyor_stop High directly.")

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.update()
            time.sleep(0.02)

    def wait_until_still(self, timeout=5.0, window=0.3):
        """Wait until the tick count moves less than 0.5 within `window` seconds."""
        start = len(self.samples)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.wait(0.05)
            if len(self.samples) <= start:
                continue
            newest = self.samples[-1]["timestamp"]
            ticks, covered = [], False
            for s in reversed(self.samples[start:]):
                if newest - s["timestamp"] > window:
                    covered = True
                    break
                ticks.append(tick_of(s))
            if covered and max(ticks) - min(ticks) < 0.5:
                return True
        return False

    def save(self, name):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"{name}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["controller_s", "laptop_s", "tick", "fwd", "bwd", "stop", "speed_V",
                        "program", "tcp_x", "tcp_y", "tcp_z"])
            for s in self.samples:
                tcp = s.get("actual_TCP_pose") or [0.0, 0.0, 0.0]
                w.writerow([f"{s['timestamp']:.3f}", f"{s['_t']:.4f}", f"{tick_of(s):.2f}",
                            bit(s, FWD_BIT), bit(s, BWD_BIT), bit(s, STOP_BIT),
                            f"{s['standard_analog_output1']:.3f}",
                            PROGRAM_STATES.get(s["runtime_state"], s["runtime_state"]),
                            f"{tcp[0]:.4f}", f"{tcp[1]:.4f}", f"{tcp[2]:.4f}"])
        return path

    def close(self):
        self.link.close()


def stop_belt_directly():
    """Set conveyor_stop High (fwd/bwd Low) over RTDE without the robot program.
    Uses its own short connection. Returns True if the output really went High."""
    try:
        with RTDELink(ROBOT_IP, outputs=["actual_digital_output_bits"],
                      inputs=["standard_digital_output_mask", "standard_digital_output"]) as l:
            mask = (1 << FWD_BIT) | (1 << BWD_BIT) | (1 << STOP_BIT)
            l.send(standard_digital_output_mask=mask, standard_digital_output=1 << STOP_BIT)
            time.sleep(0.3)
            return bit(l.latest(), STOP_BIT) == 1
    except Exception as e:
        print(f"Direct stop over RTDE failed: {e}")
        return False


def save_reads(reads, order, name):
    """Write the sensor readings collected by one_pass() to logs/ and return the path."""
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"{name}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["laptop_s"] + order)
        for tm, d in reads:
            w.writerow([f"{tm:.4f}"] + [d[n] for n in order])
    return path


def require_empty_belt():
    reader = SensorReader()
    try:
        _, _, values = reader.read()
    finally:
        reader.close()
    busy = [v.name for v in values.values()
            if v.distance_cm is not None and v.distance_cm < EMPTY_MIN_CM]
    if busy:
        raise SystemExit(f"Something is in front of {', '.join(busy)}. The belt must be empty.")


# ---------------------------------------------------------------- check

def check(belt):
    s = belt.link.latest()
    running = belt.program_running()
    print(f"program:   {PROGRAM_STATES.get(s['runtime_state'], s['runtime_state'])}"
          f", heartbeat {'moving' if running else 'NOT moving'}")
    print(f"belt:      fwd={bit(s, FWD_BIT)} bwd={bit(s, BWD_BIT)} stop={bit(s, STOP_BIT)}"
          f"  conveyor_speed={s['standard_analog_output1']:.2f} V  tick={tick_of(s):.2f}")
    print(f"last belt command done: {s['output_int_register_26']}")
    reader = SensorReader()
    try:
        _, _, values = reader.read()
    finally:
        reader.close()
    print("sensors:   " + "  ".join(f"{v.name}={v.distance_cm} cm" for v in values.values()))
    if not running:
        print("-> Load the program (pick_stage2 or later) on the pendant and press Play.")


# ---------------------------------------------------------------- speed

def analyse_run(seg, direction):
    """seg: samples from before the start pulse until the belt stood still after the stop."""
    out_bit = FWD_BIT if direction == FORWARD else BWD_BIT
    sign = 1 if direction == FORWARD else -1
    n = len(seg)
    ts = [s["timestamp"] for s in seg]
    tk = [tick_of(s) for s in seg]

    def first(start, pred):
        for i in range(start, n):
            if pred(i):
                return i
        return None

    a = first(0, lambda i: bit(seg[i], out_bit))
    if a is None:
        return {"error": "start pulse not seen"}
    b = first(a, lambda i: bit(seg[i], STOP_BIT))
    if b is None:
        return {"error": "stop not seen"}
    r = {"volts": seg[(a + b) // 2]["standard_analog_output1"], "started": False}
    m = first(a, lambda i: sign * (tk[i] - tk[a]) > 2)
    if m is None or m >= b:
        return r
    r["started"] = True
    r["start_delay"] = ts[m] - ts[a]

    # steady speed: from 2 s after the start pulse to the last sample before the stop
    w = first(a, lambda i: ts[i] >= ts[a] + 2.0)
    if w is None or w >= b - 1:
        return r
    v_ref = sign * (tk[b - 1] - tk[w]) / (ts[b - 1] - ts[w])
    r["ticks_per_s"] = v_ref
    if v_ref <= 0:
        return r

    # 0.1 s windows from the start of motion to the stop. A stall is a window
    # below 30 % of the steady speed after the belt had reached 50 %.
    k = max(1, round(0.1 * (n - 1) / (ts[-1] - ts[0])))
    reached, above_half, in_stall, stalls = None, False, False, []
    i = m
    while i + k < b:
        v = sign * (tk[i + k] - tk[i]) / (ts[i + k] - ts[i])
        if reached is None and v >= 0.9 * v_ref:
            reached = ts[i + k] - ts[a]
        if above_half and v < 0.3 * v_ref:
            if not in_stall:
                stalls.append([ts[i] - ts[a], 0.0])
                in_stall = True
            stalls[-1][1] += ts[i + k] - ts[i]
        else:
            in_stall = False
        if v >= 0.5 * v_ref:
            above_half = True
        i += k
    r["time_to_speed"] = reached
    r["stalls"] = stalls

    final = tk[-1]
    r["stop_ticks"] = abs(final - tk[b])
    last_moving = b
    for i in range(n - 1, b - 1, -1):
        if abs(tk[i] - final) >= 0.5:
            last_moving = i
            break
    r["stop_time"] = ts[min(last_moving + 1, n - 1)] - ts[b]
    return r


def one_run(belt, direction, volts, run_s=RUN_S, pause=0.2):
    belt.update()
    i0 = len(belt.samples)
    belt.run(direction, volts, pause)
    belt.wait(run_s)
    belt.stop()
    belt.wait_until_still()
    belt.wait(0.3)
    return analyse_run(belt.samples[i0:], direction)


def speed_test(belt, args):
    require_empty_belt()
    volts_list = args.volts or [0.2, 0.3, 0.5, 1.0, 1.5, 2.0]
    print(f"Belt must be EMPTY. Each voltage: {RUN_S:.0f} s forward, then stop. "
          "Last: 1.0 V backward.")
    results = []
    for v in volts_list:
        print(f"  forward at {v:.2f} V ...")
        results.append(("fwd", one_run(belt, FORWARD, v)))
    print("  backward at 1.00 V ...")
    results.append(("back", one_run(belt, BACKWARD, 1.0)))

    print("\n dir   volts  started  start delay  to 90 %  ticks/s   cm/s*  stop ticks"
          "  stop cm*  stop time  stalls")
    for name, r in results:
        if "error" in r:
            print(f" {name:4}  {r['error']}")
            continue
        if not r["started"]:
            print(f" {name:4}  {r['volts']:5.2f}  no")
            continue
        v = r.get("ticks_per_s")
        stop = r.get("stop_ticks")
        stalls = r.get("stalls", [])
        detail = "; ".join(f"at {t:.1f} s for {d:.1f} s" for t, d in stalls)
        print(f" {name:4}  {r['volts']:5.2f}  yes      {fmt(r['start_delay'], '6.2f', ' s')}"
              f"     {fmt(r.get('time_to_speed'), '5.2f', ' s')}  {fmt(v, '7.1f')}"
              f"  {fmt(v and v / TICKS_PER_M * 100, '6.2f')}  {fmt(stop, '10.1f')}"
              f"  {fmt(stop and stop / TICKS_PER_M * 100, '8.2f')}"
              f"  {fmt(r.get('stop_time'), '7.2f', ' s')}  {len(stalls)}"
              + (f" ({detail})" if detail else ""))
    print(f"* converted with {TICKS_PER_M:.0f} ticks/m, which 'calibrate' checks."
          " Times are from the start pulse.")


# ---------------------------------------------------------------- start-pause

def start_pause_test(belt, args):
    require_empty_belt()
    volts = args.volts[0] if args.volts else 1.0
    tally = {0.0: [0, 0], 0.2: [0, 0]}     # pause -> [started, tries]
    print("Belt must be EMPTY. 10 starts from a blocked belt (conveyor_stop High),"
          " alternating no pause and 0.2 s pause.")
    for i in range(10):
        pause = 0.0 if i % 2 == 0 else 0.2
        belt.stop()
        belt.wait_until_still()
        belt.wait(0.5)
        belt.update()
        i0 = len(belt.samples)
        belt.run(FORWARD, volts, pause)
        belt.wait(1.5)
        seg = belt.samples[i0:]
        a = next((j for j, s in enumerate(seg) if bit(s, FWD_BIT)), None)
        started = a is not None and tick_of(seg[-1]) - tick_of(seg[a]) > 20
        tally[pause][0] += started
        tally[pause][1] += 1
        print(f"  try {i + 1:2}: pause {pause:.1f} s -> {'started' if started else 'DID NOT START'}")
    belt.stop()
    for pause, (ok, tries) in tally.items():
        print(f"pause {pause:.1f} s: started {ok} of {tries}")


# ---------------------------------------------------------------- calibrate

def one_pass(belt, reader, base, order, pos, volts, reads, on_leave=None,
             stop_past_cm=STOP_PAST_S2_CM):
    """Run the cylinder past all sensors. Returns {name: crossing} or None.

    on_leave(name, t_enter, t_leave) is called as soon as a sensor sees the cylinder go
    (laptop time.monotonic() values), while the belt is still running. The belt stops
    when the last sensor no longer sees the cylinder, or at the latest when it is
    stop_past_cm past that sensor."""
    _, _, values = reader.read()
    if any(values[n].distance_cm is not None and values[n].distance_cm < base[n] - BLOCKED_BELOW_CM
           for n in order):
        print("  A sensor already sees something. Put the cylinder before S4 and try again.")
        return None

    blocked = {n: False for n in order}
    enter, leave, nearest = {}, {}, {}
    belt.update()
    i0 = len(belt.samples)
    belt.run(FORWARD, volts)
    t_start = time.monotonic()
    deadline = None
    last, before_last = order[-1], order[-2]
    while True:
        try:
            t0, t1, values = reader.read()
        except Exception as e:
            # without the sensors we can't know where the cylinder is: stop at once
            belt.stop()
            raise ConnectionError(f"The IO-Link master did not answer ({e}). Belt stopped;"
                                  " check the network (a cable is steadier than Wi-Fi).") from e
        if belt.update()["runtime_state"] != PLAYING:
            # protective stop or pause: the belt would run on (2026-10-22, stage 5 run 1)
            print("  The robot program is not running (protective stop?). Stopping the belt.")
            break
        tm = (t0 + t1) / 2      # the sample was taken somewhere in the request
        reads.append((tm, {n: values[n].distance_cm for n in order}))
        for n in order:
            if n in leave:
                continue
            d = values[n].distance_cm
            d = 0 if d is None else d     # no valid distance: object very close to the sensor
            if not blocked[n] and d < base[n] - BLOCKED_BELOW_CM:
                blocked[n] = True
                enter[n] = tm
                nearest[n] = d
            elif blocked[n]:
                if d >= base[n] - CLEAR_WITHIN_CM:
                    blocked[n] = False
                    leave[n] = tm
                    if on_leave:
                        on_leave(n, enter[n], tm)
                else:
                    nearest[n] = min(nearest[n], d)
        now = time.monotonic()
        if last in leave:
            break
        if last in enter and deadline is None:
            # speed from the last two sensors, so the stop doesn't depend on ticks/m
            speed = 10.0
            if before_last in enter:
                speed = (pos[last] - pos[before_last]) * 100 / (enter[last] - enter[before_last])
            deadline = enter[last] + stop_past_cm / speed
        if deadline is not None and now > deadline:
            print(f"  {last} did not clear in time; stopped early.")
            break
        if order[0] not in enter and now - t_start > 15:
            print(f"  {order[0]} never saw the cylinder; stopped.")
            break
        if now - t_start > 120:
            print("  Took longer than 2 minutes; stopped.")
            break
    belt.stop()
    belt.wait_until_still()

    seg = belt.samples[i0:]
    crossings = {}
    for n in order:
        if n in enter and n in leave:
            t_in, t_out = tick_at(seg, enter[n]), tick_at(seg, leave[n])
            crossings[n] = {"t": (enter[n] + leave[n]) / 2, "tick": (t_in + t_out) / 2,
                            "length_ticks": t_out - t_in, "nearest": nearest[n]}
    return crossings


def calibrate(belt, args):
    volts = args.volts[0] if args.volts else 1.0
    order = [n for n, _, _ in sorted(SENSORS, key=lambda s: s[2])]     # along the belt
    pos = {n: p for n, _, p in SENSORS}
    reader = SensorReader()
    reads, results = [], []
    try:
        belt.stop()
        belt.wait_until_still()
        print("Measuring the empty belt for 2 s (keep it empty) ...")
        base = measure_baseline(reader)
        missing = [n for n, b in base.items() if b is None]
        if missing:
            raise SystemExit(f"No valid reading from {', '.join(missing)}.")
        print("Empty belt: " + ", ".join(f"{n} {base[n]} cm" for n in order))

        for run in range(1, args.runs + 1):
            print(f"\nRun {run} of {args.runs}: put the cylinder upright on the belt between the"
                  " start and S4 (0-15 cm), roughly in the middle.")
            input("Stand at the belt end, ready to catch it. Press Enter to start the belt ... ")
            crossings = one_pass(belt, reader, base, order, pos, volts, reads)
            if not crossings:
                continue
            seen = [n for n in order if n in crossings]
            if len(seen) < 2:
                print("  Fewer than two sensors saw the whole cylinder; run ignored.")
                continue
            a, b = crossings[seen[0]], crossings[seen[-1]]
            dist_m = pos[seen[-1]] - pos[seen[0]]
            tpm = (b["tick"] - a["tick"]) / dist_m
            results.append({"tpm": tpm, "cm_s": dist_m * 100 / (b["t"] - a["t"]),
                            "ticks_s": (b["tick"] - a["tick"]) / (b["t"] - a["t"]),
                            "span": f"{seen[0]} -> {seen[-1]}"})

            print("  sensor  position  object length  nearest face")
            for n in seen:
                c = crossings[n]
                print(f"  {n:6}  {pos[n] * 100:6.1f} cm  {c['length_ticks'] / tpm * 100:8.1f} cm"
                      f"  {c['nearest']:9} cm")
            print("  from -> to   distance   ticks/m   speed (laptop clock)")
            for n0, n1 in zip(seen, seen[1:]):
                d = pos[n1] - pos[n0]
                c0, c1 = crossings[n0], crossings[n1]
                print(f"  {n0} -> {n1}   {d * 100:6.1f} cm  {(c1['tick'] - c0['tick']) / d:8.0f}"
                      f"   {d * 100 / (c1['t'] - c0['t']):6.2f} cm/s")
            print(f"  {seen[0]} -> {seen[-1]}   {dist_m * 100:6.1f} cm  {tpm:8.0f}"
                  f"   {results[-1]['cm_s']:6.2f} cm/s   <- result")
    finally:
        reader.close()
        if reads:
            print(f"Sensor log: {save_reads(reads, order, 'stage2_calibrate_sensors')}")

    if results:
        tpms = [r["tpm"] for r in results]
        mean = sum(tpms) / len(tpms)
        spread = (max(tpms) - min(tpms)) / mean * 100
        speed = sum(r["cm_s"] for r in results) / len(results)
        rate = sum(r["ticks_s"] for r in results) / len(results)
        print(f"\nticks/m over {len(tpms)} runs: " + ", ".join(f"{t:.0f}" for t in tpms)
              + f"  -> mean {mean:.0f}, spread {spread:.1f} %")
        print(f"Installation value: {TICKS_PER_M:.0f} ticks/m ({(mean / TICKS_PER_M - 1) * 100:+.1f} %)")
        print(f"At {volts:.2f} V: {speed:.2f} cm/s, {rate:.0f} ticks/s")


# ---------------------------------------------------------------- watch

def watch(belt, args):
    """Run the belt and print the encoder speed four times a second."""
    require_empty_belt()
    volts = args.volts[0] if args.volts else 1.0
    direction, sign = (BACKWARD, -1) if args.backward else (FORWARD, 1)
    print(f"Belt {'backward' if args.backward else 'forward'} at {volts:.2f} V for"
          f" {args.seconds:.0f} s. Watch the red wheel: does it stand still when a line says STOP?")
    belt.run(direction, volts)
    t0 = time.monotonic()
    last = belt.update()
    rates = []
    while time.monotonic() - t0 < args.seconds:
        belt.wait(0.25)
        s = belt.samples[-1]
        rate = sign * (tick_of(s) - tick_of(last)) / (s["timestamp"] - last["timestamp"])
        last = s
        rates.append(rate)
        ref = statistics.median(rates) if len(rates) >= 8 else None
        flag = "  <-- STOP" if ref and rate < 0.3 * ref else ""
        bar = "#" * max(0, min(60, round(rate / 5)))
        print(f"{time.monotonic() - t0:5.1f} s {rate:6.0f} ticks/s  {bar}{flag}")
    belt.stop()
    belt.wait_until_still()

    running = rates[4:]                       # skip the first second (starting)
    if running:
        ref = statistics.median(running)
        stalled = sum(1 for r in running if r < 0.3 * ref)
        print(f"\nRunning speed {ref:.0f} ticks/s. Encoder stood still {stalled * 0.25:.1f} s"
              f" of {len(running) * 0.25:.1f} s ({stalled / len(running) * 100:.0f} %).")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["check", "speed", "start-pause", "calibrate", "watch", "stop"])
    ap.add_argument("--volts", type=float, nargs="+",
                    help="speed: voltages to test (default 0.2 0.3 0.5 1.0 1.5 2.0);"
                         " other modes: one voltage (default 1.0). The robot limits to 2 V.")
    ap.add_argument("--runs", type=int, default=3, help="calibrate: number of runs (default 3)")
    ap.add_argument("--seconds", type=float, default=30, help="watch: how long (default 30 s)")
    ap.add_argument("--backward", action="store_true", help="watch: run the belt backward")
    args = ap.parse_args()

    belt = Belt()
    if args.mode == "check":
        try:
            check(belt)
        finally:
            belt.close()
        return

    try:
        if not belt.program_running():
            raise SystemExit("The robot program is not running. Load pick_stage2 and press Play.")
        if args.mode == "stop":
            belt.stop()
            print("Belt stopped (conveyor_stop High).")
        elif args.mode == "speed":
            speed_test(belt, args)
        elif args.mode == "start-pause":
            start_pause_test(belt, args)
        elif args.mode == "watch":
            watch(belt, args)
        else:
            calibrate(belt, args)
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
        if belt.samples and args.mode != "stop":
            print(f"Log: {belt.save('stage2_' + args.mode)}")
        belt.close()


if __name__ == "__main__":
    main()
