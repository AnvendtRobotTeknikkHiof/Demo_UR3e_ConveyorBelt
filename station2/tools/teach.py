"""
Record the poses station 2 uses and save them in station2/poses.json (Local mode).

Move the robot to each pose on the pendant (Move tab or Freedrive), then press Enter.
    pick_above, pick   over the belt at the pick spot (178 cm), fingers across the belt;
                       pick_above straight above pick (fingertips ~2 cm above the item)
    board_*            the gripper holding an item at place height, 2-3 mm above the board:
                       origin = slot 0, row_end = last row / first column,
                       col_end = first row / last column
    omron_*            the same for the tray on the Omron

    python station2/tools/teach.py           all poses
    python station2/tools/teach.py pick      only some (the others are kept)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from station2 import POSES_FILE, Link, StationError   # noqa: E402

POSES = ["pick_above", "pick", "board_origin", "board_row_end", "board_col_end",
         "omron_origin", "omron_row_end", "omron_col_end"]
FIELDS = ["actual_q", "actual_TCP_pose", "payload", "payload_cog", "tcp_offset"]
TYPES = ["VECTOR6D", "VECTOR6D", "DOUBLE", "VECTOR3D", "VECTOR6D"]


def main():
    wanted = [p for p in POSES if any(p.startswith(a) for a in sys.argv[1:])] or POSES
    poses = {}
    if os.path.exists(POSES_FILE):
        with open(POSES_FILE) as f:
            poses = json.load(f)
    try:
        link, extras = Link(outputs=FIELDS, inputs=(), output_types=TYPES), True
    except StationError:
        link, extras = Link(outputs=FIELDS[:2], inputs=(), output_types=TYPES[:2]), False
    try:
        for name in wanted:
            input(f"Robot at '{name}'? Press Enter ... ")
            s = link.state
            poses[name] = {"q": [round(a, 5) for a in s.actual_q],
                           "tcp": [round(a, 5) for a in s.actual_TCP_pose]}
            x, y, z = s.actual_TCP_pose[:3]
            print(f"  {name}: x {x * 1000:.0f}  y {y * 1000:.0f}  z {z * 1000:.0f} mm")
        if extras:
            poses["payload"] = {"mass": round(s.payload, 3), "cog": [round(c, 4) for c in s.payload_cog]}
            poses["tcp_offset"] = [round(a, 5) for a in s.tcp_offset]
    finally:
        link.close()

    if "pick" in poses and "pick_above" in poses:
        above, pick = poses["pick_above"]["tcp"], poses["pick"]["tcp"]
        side_mm = ((above[0] - pick[0]) ** 2 + (above[1] - pick[1]) ** 2) ** 0.5 * 1000
        print(f"pick_above is {(above[2] - pick[2]) * 1000:.0f} mm above pick, {side_mm:.1f} mm"
              " to the side" + ("  <-- must be straight above (< 2 mm)" if side_mm > 2 else ""))
    missing = [p for p in POSES if p not in poses]
    with open(POSES_FILE, "w") as f:
        json.dump(poses, f, indent=2)
    print(f"Saved {POSES_FILE}" + (f". Still missing: {', '.join(missing)}" if missing else ""))


if __name__ == "__main__":
    main()
