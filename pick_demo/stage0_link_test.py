"""
Stage 0: test the RTDE link between the laptop and the UR3e. The arm does not move.

Robot side: robot/pick_link.script in BeforeStart of the stage-0 program (see README.md).

    python stage0_link_test.py                 (Ctrl+C to stop)
    python stage0_link_test.py --seconds 20

Every 0.5 s the laptop writes a counter to input int register 24 and prints:
    readback  input int 24 as the controller sees it      -> laptop -> controller works
    hb        heartbeat from the robot program (out int 24) -> robot program publishes
    echo      program's copy of input int 24 (out int 25)  -> full round trip
    tick      conveyor tick count (out float 24) and belt speed
    DO        standard outputs 5/6/7 = conveyor_fwd / conveyor_bcwd / conveyor_stop
    AO1       conveyor_speed in volts
"""
import argparse
import time

from rtde_link import PROGRAM_STATES, ROBOT_IP, RTDELink

TICKS_PER_M = 1825.0   # Installation -> Conveyor Tracking (not yet checked with a tape measure)

OUTPUTS = ["timestamp", "runtime_state", "actual_digital_output_bits", "standard_analog_output1",
           "input_int_register_24", "output_int_register_24", "output_int_register_25",
           "output_double_register_24"]
INPUTS = ["input_int_register_24"]


def bit(bits, n):
    return (bits >> n) & 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=None, help="stop after this many seconds")
    args = ap.parse_args()

    link = RTDELink(ROBOT_IP, outputs=OUTPUTS, inputs=INPUTS)
    print(f"Connected to {ROBOT_IP}, controller {'.'.join(map(str, link.controller_version))}")
    seen = {"readback": False, "heartbeat": False, "echo": False,
            "tick forward": False, "tick backward": False}
    counter, prev = 0, None
    end = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        while end is None or time.monotonic() < end:
            counter += 1
            link.send(input_int_register_24=counter)
            time.sleep(0.5)
            s = link.latest()

            program = PROGRAM_STATES.get(s["runtime_state"], s["runtime_state"])
            hb, echo, tick = (s["output_int_register_24"], s["output_int_register_25"],
                              s["output_double_register_24"])
            seen["readback"] |= s["input_int_register_24"] == counter
            seen["echo"] |= echo == counter

            speed = ""
            if prev:
                seen["heartbeat"] |= hb != prev["output_int_register_24"]
            # The tick register only changes while the program runs, and jumps from its old
            # value when the program starts, so only compare two samples taken while playing.
            if prev and s["runtime_state"] == 2 and prev["runtime_state"] == 2:
                dt = s["timestamp"] - prev["timestamp"]
                dticks = tick - prev["output_double_register_24"]
                seen["tick forward"] |= dticks > 1
                seen["tick backward"] |= dticks < -1
                if dt > 0:
                    speed = f"({dticks:+7.2f}, {dticks / dt / TICKS_PER_M * 100:+5.2f} cm/s)"

            bits = s["actual_digital_output_bits"]
            print(f"prog={program:8} sent={counter:4} readback={s['input_int_register_24']:4}"
                  f"  hb={hb:8}  echo={echo:4}  tick={tick:11.2f} {speed:24}"
                  f"  fwd={bit(bits, 5)} bwd={bit(bits, 6)} stop={bit(bits, 7)}"
                  f"  AO1={s['standard_analog_output1']:.2f} V")
            prev = s
    except KeyboardInterrupt:
        pass
    finally:
        link.send(input_int_register_24=0)
        time.sleep(0.1)
        link.close()

    print("\nSummary (paste this to your notes / to Claude):")
    for name, ok in seen.items():
        print(f"  {name:14} {'SEEN' if ok else 'not seen'}")


if __name__ == "__main__":
    main()
