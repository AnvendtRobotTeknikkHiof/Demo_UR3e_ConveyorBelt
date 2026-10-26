"""
Stage 4 helper: where is the gripper, measured along the belt? Read-only.

Shows the TCP (gr10_mid_griper) as belt position (cm from the start of the belt),
offset from the belt centreline, height above the taught line points, how far the tool
is tilted from pointing straight down, and the joint angles. Use it while you teach the
pick pose and when you jog along the belt to check how far the robot reaches.

    python pick_demo\\stage4_where.py           twice a second until Ctrl+C
    python pick_demo\\stage4_where.py --once

The belt geometry comes from the two points you taught for the Line feature (TCP on the
belt centreline at 165 and 195 cm, base frame, metres).
"""
import argparse
import math
import time

from rtde_link import PROGRAM_STATES, ROBOT_IP, RTDELink

P165 = (-0.46929, -0.23696, -0.41115)
P195 = (-0.47689, 0.07018, -0.41232)
S165, S195 = 165.0, 195.0          # cm from the start of the belt (tape)
BELT_END_CM, S2_CM = 200.0, 181.5


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


d = sub(P195, P165)
L = math.sqrt(dot(d, d))
u = [x / L for x in d]                               # along the belt, towards the robot end
h = math.hypot(u[0], u[1])
n = [-u[1] / h, u[0] / h, 0.0]                       # across the belt, horizontal
if dot(n, [-P165[0], -P165[1], 0.0]) < 0:            # make + point towards the robot base
    n = [-x for x in n]


def tool_tilt_deg(rx, ry, rz):
    """Angle between the tool z axis and straight down (rotation vector -> matrix element R33)."""
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    kz = rz / angle if angle > 1e-9 else 0.0
    zz = math.cos(angle) + kz * kz * (1 - math.cos(angle))   # z component of the tool z axis
    return math.degrees(math.acos(max(-1.0, min(1.0, -zz))))


def describe(s):
    p = s["actual_TCP_pose"]
    rel = sub(p[:3], P165)
    along = S165 + (S195 - S165) * dot(rel, u) / L
    across = dot(rel, n) * 100
    f = (along - S165) / (S195 - S165)
    z_line = P165[2] + f * (P195[2] - P165[2])
    height = (p[2] - z_line) * 100
    q = [math.degrees(a) for a in s["actual_q"]]
    return (f"belt {along:6.1f} cm   centreline {across:+5.1f} cm (+ = robot side)"
            f"   height {height:5.1f} cm   tilt {tool_tilt_deg(*p[3:]):4.1f} deg"
            f"   elbow {q[2]:6.1f} deg   joints [" + ", ".join(f"{a:.1f}" for a in q) + "]")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true", help="print one line and stop")
    args = ap.parse_args()
    with RTDELink(ROBOT_IP, outputs=["runtime_state", "actual_TCP_pose", "actual_q"]) as link:
        s = link.latest()
        print(f"program {PROGRAM_STATES.get(s['runtime_state'])}. S2 at {S2_CM} cm,"
              f" belt ends at {BELT_END_CM:.0f} cm. Height is above the taught line points.")
        try:
            while True:
                print(describe(link.latest()))
                if args.once:
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
