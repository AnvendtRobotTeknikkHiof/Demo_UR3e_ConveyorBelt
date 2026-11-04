"""
Station 2: UR3e#2 picks items from the moving belt onto the temp board, and from the
board onto the Omron.

    python station2/station2.py start      send robot.script (robot in Remote mode)
    python station2/station2.py run 3      fill the board with 3 items, then (Enter when
                                          the Omron is there) move them to the Omron
    python station2/station2.py stop       stop the program and the belt
    python station2/station2.py show       print the robot program, send nothing

main.py uses the same calls: Station2().fill_board(x), then board_to_omron(x).
"""
import json
import os
import socket
import sys
import threading
import time
from collections import deque

import rtde.rtde as rtde

import config
from iolink import Edge, SensorReader

HERE = os.path.dirname(os.path.abspath(__file__))
POSES_FILE = os.path.join(HERE, "poses.json")
PLAYING, STOPPED = 2, 1       # runtime_state
ROBOT_ON, SAFETY_NORMAL = 7, 1
FORWARD, STOP = 1, 3          # belt commands
BELT_TO_BOARD, BOARD_TO_OMRON, HOME = 1, 2, 3   # job types
PARKED = 1e9                  # a start tick the encoder never reaches
STOP_BITS = 0b11100000        # standard digital outputs 5 (fwd), 6 (bwd), 7 (stop)

OUTPUTS = ["timestamp", "runtime_state", "robot_mode", "safety_mode",
           "output_double_register_24"] + [f"output_int_register_{n}" for n in (24, 26, 27, 28, 29)]
INPUTS = [f"input_int_register_{n}" for n in (25, 26, 27, 28, 29, 30, 31)] \
    + [f"input_double_register_{n}" for n in (24, 25, 27)] \
    + ["standard_digital_output_mask", "standard_digital_output"]
INPUT_TYPES = ["INT32"] * 7 + ["DOUBLE"] * 3 + ["UINT8", "UINT8"]


class StationError(RuntimeError):
    pass


class Link:
    """RTDE connection: a thread keeps the newest robot state and the last 2 s of ticks."""

    def __init__(self, host=config.ROBOT_IP, outputs=OUTPUTS, inputs=INPUTS, types=INPUT_TYPES,
                 output_types=()):
        self.con = rtde.RTDE(host, 30004)
        self.con.connect()
        if not self.con.send_output_setup(outputs, list(output_types), frequency=125):
            raise StationError("RTDE output setup failed (unknown field?)")
        self.inp = self.con.send_input_setup(inputs, types) if inputs else None
        if inputs and self.inp is None:
            raise StationError("RTDE input registers are in use by another program")
        self.con.send_start()
        self.state = self.con.receive()
        self.ticks = deque(maxlen=250)
        self.alive = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self):
        while self.alive:
            try:
                s = self.con.receive()
            except rtde.RTDEException:
                self.alive = False        # check_running() reports it
                return
            if s is None:
                continue
            self.state = s
            if hasattr(s, "output_double_register_24"):
                self.ticks.append((time.monotonic(), s.output_double_register_24))

    def start_heartbeat(self):
        """After every input has a value: the robot stops the belt when this goes quiet
        for 1 s (the laptop crashed or hung)."""
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _heartbeat(self):
        n = 0
        while self.alive:
            n += 1
            self.send(input_int_register_31=n)
            time.sleep(0.1)

    def send(self, **values):
        with self.lock:
            if not self.alive:        # closed: a thread still stopping must not send
                return
            for name, value in values.items():
                setattr(self.inp, name, value)
            self.con.send(self.inp)

    def tick_at(self, t):
        """Tick at laptop time t, interpolated (t within the last 2 s)."""
        hist = list(self.ticks)
        for (t0, k0), (t1, k1) in zip(hist, hist[1:]):
            if t0 <= t <= t1:
                return k0 + (k1 - k0) * (t - t0) / (t1 - t0)
        return hist[-1][1] if t > hist[-1][0] else hist[0][1]

    def tick_rate(self, window=0.5):
        hist = list(self.ticks)
        old = next((x for x in reversed(hist) if hist[-1][0] - x[0] >= window), hist[0])
        return (hist[-1][1] - old[1]) / (hist[-1][0] - old[0]) if hist[-1][0] > old[0] else 0.0

    def close(self):
        self.alive = False
        self.thread.join(timeout=2)   # let the receive thread end before the socket closes
        self.con.disconnect()


class Station2:
    def __init__(self, ask=input):
        self.ask = ask            # how to ask the operator to fix something
        self.link = Link()
        s = self.link.state
        # Start from what the robot has done, so nothing old runs again
        self.seq, self.job = s.output_int_register_26, s.output_int_register_27
        self.link.send(input_int_register_25=0, input_int_register_26=self.seq,
                       input_int_register_27=self.job, input_int_register_28=0,
                       input_int_register_29=0, input_int_register_30=0, input_int_register_31=0,
                       input_double_register_24=config.BELT_VOLTS,
                       input_double_register_25=config.START_PAUSE_S,
                       input_double_register_27=PARKED,
                       standard_digital_output_mask=0, standard_digital_output=0)
        self.link.start_heartbeat()
        self.sensors = SensorReader()
        self.empty = None         # sensor readings with an empty belt

    # ---- robot program ----

    def start(self):
        s = self.link.state
        if s.robot_mode != ROBOT_ON:
            raise StationError("Robot not on: power on, release the brakes, Remote mode")
        if s.safety_mode != SAFETY_NORMAL:
            raise StationError("Protective stop: read the pendant, Unlock, try again")
        if s.runtime_state != STOPPED:
            dashboard("stop")
            time.sleep(1.0)
        with socket.create_connection((config.ROBOT_IP, 30002), timeout=3) as sock:
            sock.sendall(build_program().encode())
            time.sleep(0.5)       # closed at once, the robot may drop the program unread
        end, last = time.monotonic() + 15, None
        while time.monotonic() < end:
            time.sleep(0.2)
            s = self.link.state
            if s.runtime_state == PLAYING and last is not None and s.output_int_register_24 > last:
                print("Robot program runs.")
                return
            last = s.output_int_register_24 if s.runtime_state == PLAYING else None
        raise StationError("The program did not start: Remote mode? Look at the Log tab")

    def stop(self):
        dashboard("stop")
        self.stop_belt_directly()

    def check_running(self):
        if not self.link.alive:
            raise StationError("Lost the RTDE connection to the robot. Stop the belt on the pendant.")
        s = self.link.state
        if s.runtime_state != PLAYING:
            self.stop_belt_directly()
            why = ("a protective stop: read the pendant, Unlock" if s.safety_mode != SAFETY_NORMAL
                   else "an error in the program: look at the Log tab on the pendant")
            raise StationError(f"The robot program stopped, {why}. Belt stopped.")

    # ---- belt ----

    def belt(self, cmd, wait=False):
        """Ask the robot to run (FORWARD) or STOP the belt. wait: until it has done it."""
        self.seq += 1
        self.link.send(input_int_register_25=cmd, input_int_register_26=self.seq)
        end = time.monotonic() + 3
        while wait and self.link.state.output_int_register_26 != self.seq:
            self.check_running()
            if time.monotonic() > end:
                raise StationError("The robot did not carry out the belt command")
            time.sleep(0.01)

    def stop_belt_directly(self):
        """conveyor_stop High without the robot program (it may have stopped)."""
        self.link.send(standard_digital_output_mask=STOP_BITS, standard_digital_output=1 << 7)
        time.sleep(0.1)
        self.link.send(standard_digital_output_mask=0)

    # ---- jobs ----

    def send_job(self, kind, start=PARKED, board=0, omron=0):
        self.job += 1
        self.link.send(input_int_register_27=self.job, input_int_register_28=kind,
                       input_int_register_29=board, input_int_register_30=omron,
                       input_double_register_27=float(start))
        return self.job

    def wait_job(self, job, timeout=30):
        end = time.monotonic() + timeout
        while self.link.state.output_int_register_27 != job:
            self.check_running()
            if time.monotonic() > end:
                raise StationError(f"The robot did not finish job {job} within {timeout} s")
            time.sleep(0.05)
        return self.link.state

    def fill_board(self, x):
        """Pick x items from the belt onto the board. Several may be on the belt at once;
        when the robot is busy and the next item gets close, the belt pauses (the encoder
        stops too, so every item's position stays right)."""
        if self.empty is None:
            self.empty = self.sensors.baseline()    # S1 must see an empty belt now
        s1_m = config.SENSORS[config.TRIGGER][1]
        edge = Edge(self.empty[config.TRIGGER])
        on_belt, queue, active = [], deque(), None  # items: {"s1": tick, "start": tick}
        placed, running = 0, False
        try:
            while placed < x:
                t, values = self.sensors.read()     # IO-Link lost -> error, belt stops
                self.check_running()
                s = self.link.state
                tick = s.output_double_register_24
                passed = edge.update(t, values[config.TRIGGER])
                if passed:
                    t_centre, readings, nearest = passed
                    if readings < 3 or nearest > config.CENTRED_MAX_CM:
                        self.belt(STOP)
                        running = False
                        self.ask(f">>> Item off the middle of the belt ({nearest} cm). Take it off,"
                                 " press Enter ... ")
                    else:
                        k = self.link.tick_at(t_centre)
                        item = {"s1": k, "start": k + (config.PICK_M - s1_m + config.OFFSET_MM
                                                       / 1000) * config.TICKS_PER_M}
                        on_belt.append(item)
                        queue.append(item)
                        print(f"Item seen by {config.TRIGGER} ({len(on_belt)} on the belt)")

                if active and s.output_int_register_29 == active["job"] and "held" not in active:
                    active["held"] = s.output_int_register_28 == 2
                    if active["held"]:
                        if active in on_belt:
                            on_belt.remove(active)
                    else:
                        print("  missed: the item stays on the belt")
                if active and s.output_int_register_27 == active["job"]:
                    if s.output_int_register_28 == 2:     # checked again at the board slot
                        placed += 1
                        print(f"  on the board: {placed} of {x}")
                    elif active.get("held"):
                        print("  dropped on the way to the board: take it away")
                    active = None
                if active is None and queue:
                    active = queue.popleft()
                    active["job"] = self.send_job(BELT_TO_BOARD, active["start"], board=placed)

                # pause only while the robot works on an item: a robot still waiting for
                # its start tick needs the belt to move
                close = queue and active and tick >= active["start"] \
                    and queue[0]["start"] - tick < config.PAUSE_LEAD_M * config.TICKS_PER_M
                if running and close:
                    self.belt(STOP)
                    running = False
                    print("  belt paused: the robot is busy and the next item is close")
                elif not running and not close:
                    self.belt(FORWARD)
                    running, t_run = True, time.monotonic()
                    print("  belt running")
                if running and time.monotonic() - t_run > config.ENCODER_GRACE_S \
                        and self.link.tick_rate() < config.MIN_TICK_RATE:
                    raise StationError("The belt runs but the encoder does not count")
                for item in on_belt:
                    if item is active and "held" not in active:
                        continue          # the robot follows it down the belt; its grip check decides
                    if s1_m + (tick - item["s1"]) / config.TICKS_PER_M >= config.END_M:
                        self.belt(STOP)
                        running = False
                        self.ask(">>> An item reached the belt end without being picked. Take it"
                                 " off, press Enter ... ")
                        on_belt.remove(item)
                        if item in queue:
                            queue.remove(item)
                        break
        finally:
            self.belt(STOP)
        left = len(on_belt) + (edge.t_in is not None)       # + one in front of S1 now
        if left:
            print(f"  {x} on the board, done. {left} more item(s) still on the belt: take them off")
        return placed

    def board_to_omron(self, n):
        """Move board slots 0..n-1 to the Omron, then go back above the belt. Returns how
        many were moved (the grip is checked again at each Omron slot)."""
        moved = 0
        for i in range(n):
            s = self.wait_job(self.send_job(BOARD_TO_OMRON, board=i, omron=moved))
            if s.output_int_register_28 == 2:
                moved += 1
            else:
                print(f"  board slot {i}: nothing gripped")
        self.wait_job(self.send_job(HOME))
        return moved

    def close(self):
        self.sensors.close()
        self.link.close()


def build_program(poses=None):
    poses = poses or load_poses()
    q = lambda name: "[" + ", ".join(f"{a:.5f}" for a in poses[name]["q"]) + "]"
    p = lambda name: "p[" + ", ".join(f"{a:.5f}" for a in poses[name]["tcp"]) + "]"
    pay = poses.get("payload")
    with open(os.path.join(HERE, "robot.script")) as f:
        program = f.read().format(
            payload=f"set_payload({pay['mass']}, {pay['cog']})" if pay else "# payload as on the robot",
            tcp=f"set_tcp(p{poses['tcp_offset']})" if "tcp_offset" in poses else "# TCP as on the robot",
            pick_above=q("pick_above"), pick=q("pick"),
            board_origin=p("board_origin"), board_row_end=p("board_row_end"),
            board_col_end=p("board_col_end"), board_rows=config.BOARD_GRID[0],
            board_cols=config.BOARD_GRID[1], omron_origin=p("omron_origin"),
            omron_row_end=p("omron_row_end"), omron_col_end=p("omron_col_end"),
            omron_rows=config.OMRON_GRID[0], omron_cols=config.OMRON_GRID[1],
            above=config.ABOVE_M, omron_above=config.OMRON_ABOVE_M, belt_direction=config.BELT_DIRECTION,
            ticks_per_m=config.TICKS_PER_M, encoder_type=config.ENCODER_TYPE,
            encoder_a=config.ENCODER_A, encoder_b=config.ENCODER_B,
            pick_v=config.PICK_V, pick_a=config.PICK_A,
            move_v=config.MOVE_V, move_a=config.MOVE_A, joint_v=config.JOINT_V,
            joint_a=config.JOINT_A, grip_speed=config.GRIP_SPEED, grip_force=config.GRIP_FORCE)
    lines = program.rstrip().splitlines()
    if any(line.strip() and not line.startswith(" ") for line in lines[1:-1]):
        raise StationError("robot.script: every line but the first and last must be indented")
    return program


def load_poses():
    if not os.path.exists(POSES_FILE):
        raise SystemExit("No poses.json yet: run  python station2/tools/teach.py  first.")
    with open(POSES_FILE) as f:
        return json.load(f)


def dashboard(command):
    with socket.create_connection((config.ROBOT_IP, 29999), timeout=3) as sock:
        sock.recv(1024)                       # welcome message
        sock.sendall(command.encode() + b"\n")
        return sock.recv(1024).decode(errors="replace").strip()


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "show":
        print(build_program())
        return
    if cmd not in ("start", "run", "stop"):
        raise SystemExit(__doc__)
    station = Station2()
    try:
        if cmd == "start":
            station.start()
        elif cmd == "stop":
            station.stop()
        else:
            x = int(sys.argv[2]) if len(sys.argv) > 2 else 1
            print(f"Filling the board with {x} items. Put them on the belt.")
            placed = station.fill_board(x)
            input(f"{placed} items on the board. Press Enter when the Omron is at UR3e#2 ... ")
            moved = station.board_to_omron(placed)
            print(f"{moved} of {placed} moved to the Omron.")
    except KeyboardInterrupt:
        station.belt(STOP)
        print("\nStopped by Ctrl+C.")
    except StationError as e:
        print(f"\n{e}")
    finally:
        station.close()


if __name__ == "__main__":
    main()
