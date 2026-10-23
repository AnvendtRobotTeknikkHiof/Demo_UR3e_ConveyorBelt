"""
Stage 5 robot program from Python, instead of the PolyScope program pick_stage5.

The laptop builds one URScript program and sends it to the robot (port 30002). It does
what pick_stage5 did:
  start  stop the belt, activate and open the gripper, start the threads: tick and
         heartbeat to the laptop, software encoder (robot/pick_vencoder.script), belt
         commands (robot/pick_belt_thread.script)
  loop   wait for a pick job -> above the pick spot -> wait for the start tick -> follow
         the belt -> down, grip, up -> stop following -> on the table -> back above the
         pick spot -> job done
It also reports the grip (output int 28 = gripper OBJ, int 29 = job). Between jobs it
waits above the pick spot, so it is ready for the next cylinder (stage 7); it moves there
when the program starts.

    python pick_demo\\stage5_robot.py teach   Local mode: record the four poses (once)
    python pick_demo\\stage5_robot.py start   Remote mode: send the program, check it runs
    python pick_demo\\stage5_robot.py stop    stop the program and the belt
    python pick_demo\\stage5_robot.py show    print the program, send nothing

The poses are saved in stage5_poses.json. All moves go to joint positions, so the
program doesn't depend on which TCP is active.
"""
import json
import os
import socket
import sys
import time

import stage2_belt_test as s2
from rtde_link import ROBOT_IP, RTDEError, RTDELink
from stage4_where import describe

HERE = os.path.dirname(os.path.abspath(__file__))
POSES_FILE = os.path.join(HERE, "stage5_poses.json")
POSE_NAMES = ["pick_above", "pick", "place_above", "place"]
SCRIPT_PORT = 30002       # secondary interface: runs URScript programs (Remote mode only)
DASHBOARD_PORT = 29999
RUNNING = 7               # robot_mode: powered on, brakes released
NORMAL = 1                # safety_mode (3 = protective stop)
STOPPED = 1               # runtime_state (2 playing, 4 paused)

# Values from stage 5 (tuned 2026-10-21)
BELT_DIRECTION = "p[-0.0247, 0.9997, -0.0038, 0, 0, 0]"   # in base, towards the belt end
PICK_V, PICK_A = 0.4, 1.5      # MoveL down to pick and back up: 400 mm/s, 1500 mm/s2
MOVE_V, MOVE_A = 0.25, 1.2     # MoveL at the table (PolyScope default)
JOINT_V, JOINT_A = 1.05, 1.4   # MoveJ in rad/s, rad/s2 (PolyScope default 60, 80 deg/s)
GRIP_SPEED, GRIP_FORCE = 255, 100   # Robotiq 0-255; speed 255 = 100 %

PROGRAM = """def pick_stage5():
  {payload}
  # ---- gripper: Robotiq URCap server on the controller ----
  def grip_set(name, value):
    socket_set_var(name, value, "grip")     # sends "SET <name> <value>"
    socket_read_byte_list(3, "grip")        # the server answers "ack"
  end
  def grip_get(name):
    return socket_get_var(name, "grip")     # "GET <name>" -> number
  end
  def gripper(pos):
    # 0 = open, 255 = closed. Returns OBJ when the fingers stop:
    # 2 = they hold something, 3 = they reached pos (nothing between them)
    grip_set("POS", pos)
    grip_set("GTO", 1)
    while grip_get("PRE") != pos:           # until the gripper has the new request
      sleep(0.01)
    end
    obj = grip_get("OBJ")
    while obj == 0:                         # 0 = still moving
      sleep(0.01)
      obj = grip_get("OBJ")
    end
    return obj
  end

  # ---- start (was BeforeStart) ----
  set_standard_digital_out(5, False)        # conveyor_fwd
  set_standard_digital_out(6, False)        # conveyor_bcwd
  set_standard_digital_out(7, True)         # conveyor_stop: the belt stops
  sleep(0.5)
  set_standard_digital_out(7, False)
  if not socket_open("127.0.0.1", 63352, "grip"):
    popup("No answer from the gripper server (port 63352)", "pick_stage5", error=True)
    halt
  end
  if grip_get("STA") != 3:                  # activation is lost after power-off
    grip_set("ACT", 1)
    while grip_get("STA") != 3:
      sleep(0.1)
    end
  end
  grip_set("SPE", {grip_speed})
  grip_set("FOR", {grip_force})
  gripper(0)

{vencoder}
  # tick and heartbeat to the laptop (as pick_link.script)
  write_output_integer_register(24, 0)
  write_output_integer_register(26, read_input_integer_register(26))
  thread link_thread():
    hb = 0
    while True:
      write_output_float_register(24, encoder_get_tick_count(0))
      write_output_integer_register(24, hb)
      hb = hb + 1
      sync()
    end
  end
  pick_link = run link_thread()

{belt_thread}
  # wait above the pick spot, ready for the first cylinder
  movej({pick_above}, a={joint_a}, v={joint_v})

  # ---- the pick loop (was the main program) ----
  while True:
    # wait for a new pick job from the laptop
    while read_input_integer_register(27) == read_output_integer_register(27):
      sync()
    end
    movej({pick_above}, a={joint_a}, v={joint_v})
    # wait until the cylinder is at the pick spot, then follow the belt
    while encoder_get_tick_count(0) < read_input_float_register(27):
      sync()
    end
    track_conveyor_linear({belt_direction}, {ticks_per_m})
    movel({pick}, a={pick_a}, v={pick_v})
    gripper(255)
    movel({pick_above}, a={pick_a}, v={pick_v})
    stop_conveyor_tracking()
    # grip check for the laptop (stage 6): 2 = the cylinder is in the gripper
    write_output_integer_register(28, grip_get("OBJ"))
    write_output_integer_register(29, read_input_integer_register(27))
    # put it on the table
    movej({place_above}, a={joint_a}, v={joint_v})
    movel({place}, a={move_a}, v={move_v})
    gripper(0)
    movel({place_above}, a={move_a}, v={move_v})
    # back above the pick spot, ready for the next cylinder; then the job is done
    movej({pick_above}, a={joint_a}, v={joint_v})
    write_output_integer_register(27, read_input_integer_register(27))
  end
end
"""


def joints(q):
    return "[" + ", ".join(f"{a:.5f}" for a in q) + "]"


def build_program(poses):
    def script(name):
        """A robot/*.script file without its header comment (PolyScope instructions)."""
        with open(os.path.join(HERE, "robot", name)) as f:
            lines = f.read().splitlines()
        while lines and (not lines[0].strip() or lines[0].startswith("#")):
            lines.pop(0)
        # indented: the robot takes the first "end" at the start of a line as the end
        # of the program and would run the lines after it one by one
        return "\n".join([f"  # ---- robot/{name} ----"] + ["  " + line for line in lines])

    payload = "# payload: as set on the robot (not recorded)"
    if "payload" in poses:
        p = poses["payload"]
        payload = f"set_payload({p['mass']:.3f}, [{', '.join(f'{c:.4f}' for c in p['cog'])}])"
    program = PROGRAM.format(
        payload=payload, grip_speed=GRIP_SPEED, grip_force=GRIP_FORCE,
        vencoder=script("pick_vencoder.script"), belt_thread=script("pick_belt_thread.script"),
        belt_direction=BELT_DIRECTION, ticks_per_m=s2.TICKS_PER_M,
        joint_a=JOINT_A, joint_v=JOINT_V, pick_a=PICK_A, pick_v=PICK_V, move_a=MOVE_A,
        move_v=MOVE_V, **{name: joints(poses[name]["q"]) for name in POSE_NAMES})
    lines = program.splitlines()
    if any(line.strip() and not line.startswith(" ") for line in lines[1:-1]):
        raise RuntimeError("Only the first and last line of the program may start at column 0.")
    return program


def load_poses():
    if not os.path.exists(POSES_FILE):
        raise SystemExit("No poses yet: run  python pick_demo\\stage5_robot.py teach  first.")
    with open(POSES_FILE) as f:
        return json.load(f)


def teach():
    """Record the four poses from the PolyScope program pick_stage5 (robot in Local mode)."""
    fields = ["actual_q", "actual_TCP_pose", "runtime_state", "payload", "payload_cog"]
    try:
        link = RTDELink(ROBOT_IP, outputs=fields)
    except RTDEError:
        fields = fields[:3]       # older software: no payload outputs
        link = RTDELink(ROBOT_IP, outputs=fields)
    print("Robot in Local mode, pick_stage5 open on the pendant. For each pose: tap the"
          " waypoint, hold 'Move robot here' until it is there.")
    poses = {"recorded": time.strftime("%Y-%m-%d %H:%M")}
    with link:
        for name in POSE_NAMES:
            input(f"\nRobot at '{name}'? Press Enter ... ")
            s = link.latest()
            poses[name] = {"q": [round(a, 5) for a in s["actual_q"]],
                           "tcp": [round(a, 5) for a in s["actual_TCP_pose"]]}
            print(f"  {name}: {describe(s)}")
        if "payload" in fields:
            poses["payload"] = {"mass": round(s["payload"], 3),
                                "cog": [round(c, 4) for c in s["payload_cog"]]}
            print(f"\nPayload on the robot: {s['payload']:.3f} kg")
    above, pick = poses["pick_above"]["tcp"], poses["pick"]["tcp"]
    side_mm = ((above[0] - pick[0]) ** 2 + (above[1] - pick[1]) ** 2) ** 0.5 * 1000
    print(f"pick_above is {(above[2] - pick[2]) * 1000:.0f} mm above pick and {side_mm:.1f} mm"
          " to the side" + ("  <-- must be straight above (< 2 mm)" if side_mm > 2 else ""))
    with open(POSES_FILE, "w") as f:
        json.dump(poses, f, indent=2)
    print(f"Saved {POSES_FILE}")


def start():
    """Send the program to the robot and wait until its heartbeat runs."""
    program = build_program(load_poses())
    with RTDELink(ROBOT_IP, outputs=["robot_mode", "safety_mode", "runtime_state",
                                     "output_int_register_24"]) as link:
        if link.latest()["robot_mode"] != RUNNING:
            raise SystemExit("The robot is not on. On the pendant: power on and release the"
                             " brakes, then switch to Remote mode.")
        if link.latest()["safety_mode"] != NORMAL:
            raise SystemExit("The robot is stopped by its safety system (protective stop?)."
                             " Read the message on the pendant, take a photo, tap Unlock,"
                             " then run start again.")
        if link.latest()["runtime_state"] != STOPPED:
            print("A program is running or paused; stopping it first.")
            dashboard("stop")
            time.sleep(1.0)
        with socket.create_connection((ROBOT_IP, SCRIPT_PORT), timeout=3) as sock:
            sock.sendall(program.encode())
            time.sleep(0.2)
        print("Program sent. Waiting for it to run (the gripper opens, then the arm moves"
              " above the belt) ...")
        end = time.monotonic() + 15
        last = None
        while time.monotonic() < end:
            time.sleep(0.2)
            s = link.latest()
            hb = s["output_int_register_24"]
            if s["runtime_state"] == s2.PLAYING and last is not None and hb > last:
                print("pick_stage5 runs on the robot. Now e.g. python pick_demo\\"
                      "stage7_stream.py (or stage5_pick.py, stage6_loop.py)")
                return
            last = hb if s["runtime_state"] == s2.PLAYING else None
    print("The program did not start. Is the robot in Remote mode (top right on the"
          " pendant)? Otherwise look at the Log tab on the pendant and take a photo.")


def dashboard(command):
    """One Dashboard Server command; prints the robot's answer."""
    with socket.create_connection((ROBOT_IP, DASHBOARD_PORT), timeout=3) as sock:
        sock.recv(1024)                       # welcome message
        sock.sendall(command.encode() + b"\n")
        print(f"Robot: {sock.recv(1024).decode(errors='replace').strip()}")


def stop():
    """Stop the program and set conveyor_stop."""
    dashboard("stop")
    if s2.stop_belt_directly():
        print("Belt stopped (conveyor_stop High).")


def main():
    commands = {"teach": teach, "start": start, "stop": stop,
                "show": lambda: print(build_program(load_poses()))}
    if len(sys.argv) != 2 or sys.argv[1] not in commands:
        raise SystemExit(__doc__)
    commands[sys.argv[1]]()


if __name__ == "__main__":
    main()
