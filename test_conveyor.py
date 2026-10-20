"""
Conveyor-tracking test for UR3#2: track + pick + place, no Omron.

  1. Measure the trigger sensor (SENSOR_PORT) on the empty belt (baseline)
  2. UR3#2 starts the conveyor                          (2,0)
  3. Item passes the sensor -> PC tells UR3#2             (3,0)
     UR3#2 saves the encoder count, waits until the item reaches the pick
     spot, tracks + picks it, places it, replies FINISHED
  4. After the last item UR3#2 stops the conveyor       (4,0)

Usage:
  python3 test_conveyor.py --watch       print the sensor live (to choose --margin)
  python3 test_conveyor.py               run the test with 1 item
  python3 test_conveyor.py --items 3     run the test with 3 items
"""
import argparse
import time

from sensors import read_distance
from ur3 import UR3Station, CMD_CONVEYOR_START, CMD_ITEM_AT_SENSOR, CMD_CONVEYOR_STOP

SENSOR_PORT = 3
UR3_2_PORT = 50002
PICK_TIMEOUT = 60   # seconds from trigger to FINISHED before giving up


def watch():
    print(f"Sensor on port {SENSOR_PORT} live (Ctrl+C to stop). Put an item under it to see the change.")
    while True:
        # pad so a shorter number fully overwrites the previous one (9 after 10 showed "90")
        print(f"distance {str(read_distance(SENSOR_PORT)):>5}", end="\r", flush=True)
        time.sleep(0.05)


def baseline():
    """Average reading on the empty belt."""
    values = [read_distance(SENSOR_PORT) for _ in range(10)]
    if None in values:
        raise SystemExit(f"Sensor on port {SENSOR_PORT} gives no data, check the IO-Link master")
    print(f"Empty belt: readings {min(values)}..{max(values)}")
    return sum(values) / len(values)


def item_present(base, margin):
    d = read_distance(SENSOR_PORT)
    return d is not None and abs(d - base) > margin


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--watch", action="store_true", help="only print the trigger sensor live")
    parser.add_argument("--items", type=int, default=1, help="number of items to pick")
    parser.add_argument("--margin", type=float, default=3,
                        help="change from the empty-belt reading that counts as an item")
    args = parser.parse_args()
    if args.watch:
        return watch()

    base = baseline()
    print(f"Item = reading differs from {base:.1f} by more than {args.margin}")
    ur = UR3Station("UR3_2", UR3_2_PORT)
    try:
        ur.wait_connected()
        ur.send_cmd(CMD_CONVEYOR_START)
        for i in range(1, args.items + 1):
            while item_present(base, args.margin):   # previous item still in view
                pass
            print(f"Waiting for item {i} at the sensor (port {SENSOR_PORT})")
            while not item_present(base, args.margin):
                pass
            ur.send_cmd(CMD_ITEM_AT_SENSOR)
            ur.wait_finished(timeout=PICK_TIMEOUT, resend_on_reconnect=False)
            print(f"Item {i} picked and placed")
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        if ur.conn:
            ur.send_cmd(CMD_CONVEYOR_STOP)
            time.sleep(0.2)   # let the stop command go out before closing
        ur.close()


if __name__ == "__main__":
    main()
