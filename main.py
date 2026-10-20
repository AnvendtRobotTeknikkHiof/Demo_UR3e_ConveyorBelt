"""
Runs the whole loop from one PC. The Omron starts loaded at UR3_1.

  1. Omron at UR3_1 -> UR3#1 picks items off the Omron onto the conveyor
     while the SICK PIM60 sees items -> FINISHED k   (k = 0 stops the loop)
  2. UR3#2 starts the conveyor and the Omron drives to UR3_2 at the same time
  3. Omron at UR3_2 -> UR3#2 picks k items off the belt onto the Omron,
     then stops the belt -> FINISHED
  4. Omron drives back to UR3_1, repeat

Connections:
  PC -> Omron   ARCL, port 7171                       (omron.py)
  UR3 -> PC     one TCP port per UR3 (50001, 50002)   (ur3.py + *.script)
  PC -> UR3     Dashboard Server 29999, optional      (ur3.py)
"""
from omron import ARCL, HOST, PORT, PASSWORD
from ur3 import UR3Station, Dashboard, CMD_ARRIVED, CMD_CONVEYOR_START

# goal:    goal name in MobilePlanner
# port:    port the UR3 program connects to on this PC
# ur_ip:   UR3 IP for the Dashboard Server, None = start the program on the pendant
# program: .urp file to load and play via the Dashboard Server
UR3_1 = {"name": "UR3_1", "goal": "UR3_1", "port": 50001, "ur_ip": None, "program": "pick.urp"}
UR3_2 = {"name": "UR3_2", "goal": "UR3_2", "port": 50002, "ur_ip": None, "program": "place.urp"}
GOTO_RETRIES = 3
TASK_TIMEOUT = None   # seconds to wait for FINISHED, None = wait forever


def drive_to(robot, goal):
    for attempt in range(1, GOTO_RETRIES + 1):
        print(f"Omron: Driving to {goal} (attempt {attempt})")
        if robot.goto(goal, timeout=120):
            print(f"Omron: Arrived at {goal}")
            return True
    return False


def run_loop(robot, picker, placer):
    cycle = 0
    while True:
        cycle += 1
        print(f"===== cycle {cycle} =====")

        if not drive_to(robot, UR3_1["goal"]):
            raise SystemExit(f"Could not reach {UR3_1['goal']}")
        picker.send_cmd(CMD_ARRIVED)
        k = picker.wait_finished(timeout=TASK_TIMEOUT)
        print(f"UR3_1: Finished picking up {k} items")
        if not k:
            print("No items left on the Omron, stopping")
            return

        placer.send_cmd(CMD_CONVEYOR_START)
        if not drive_to(robot, UR3_2["goal"]):
            raise SystemExit(f"Could not reach {UR3_2['goal']}")
        placer.send_cmd(CMD_ARRIVED, k)
        # Re-sending k after a reconnect could place items twice, so stop instead
        placed = placer.wait_finished(timeout=TASK_TIMEOUT, resend_on_reconnect=False)
        print(f"UR3_2: Finished placing {placed} items")
        if placed != k:
            print(f"WARNING: UR3_1 picked {k} items but UR3_2 placed {placed}")


def main():
    picker = UR3Station(UR3_1["name"], UR3_1["port"])
    placer = UR3Station(UR3_2["name"], UR3_2["port"])
    dashboards = []
    robot = None
    try:
        for s in (UR3_1, UR3_2):
            if s["ur_ip"]:
                dash = Dashboard(s["ur_ip"])
                dash.start_program(s["program"])
                dashboards.append(dash)
        picker.wait_connected()
        placer.wait_connected()
        robot = ARCL(HOST, PORT, PASSWORD)
        robot.status()
        run_loop(robot, picker, placer)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        if robot:
            robot.send("stop")
            robot.close()
        for dash in dashboards:
            dash.stop()
            dash.close()
        picker.close()
        placer.close()


if __name__ == "__main__":
    main()
