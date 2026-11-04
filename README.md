# UR3e + conveyor + Omron loop

Two UR3e robots, a conveyor belt and an Omron LD mobile robot pass the same N cylinders
round in a loop, all run from one PC by `main.py`.

```
         Route1 (station1 -> station2)
   ┌──────────────── Omron ────────────────┐
   │                                       ▼
station1 (UR3e#1)                     station2 (UR3e#2)
PIM60 #1 finds a cylinder             belt -> temp board (2x3)
on the Omron, puts it on the belt     board -> Omron tray (2x2)
   ▲                                       │
   └──────────────── Omron ────────────────┘
         Route2 (station2 -> station1)
```

One round (the Omron starts at station1, loaded with `ITEMS` cylinders):
1. station2 starts picking from the belt onto its temp board (in a background thread).
2. station1 picks every cylinder off the Omron onto the belt until its camera sees none.
   Before each one it waits until the belt has moved `PLACE_GAP_M` since the last, so it
   never places on top of the previous cylinder.
3. The Omron drives `Route1` to station2.
4. When all `ITEMS` are on the board, station2 moves them to the Omron.
5. The Omron drives `Route2` back to station1.

A count that doesn't match `ITEMS` stops everything (belt, station2's program, the Omron)
with a message: a cylinder is missing or was dropped, and needs a look.

## Files
| Path | What |
|---|---|
| `main.py` | the whole loop; settings at the top: `ITEMS`, route names, `PLACE_GAP_M` |
| `omron.py` | ARCL client: `run_route(name)`, `goto(goal)`, `status()` |
| `station1/station1.py` | UR3e#1: `get_cylinder()` (PIM60 #1), `pick_object(x, y, 1)` (ur_rtde) |
| `station1/robotiq_gripper.py` | Robotiq gripper over port 63352, used by station1 |
| `station2/station2.py` | UR3e#2: `Station2` with `start()`, `fill_board(x)`, `board_to_omron(n)`, `stop()` |
| `station2/robot.script` | the whole UR3e#2 program; `station2.py` fills in poses and settings |
| `station2/config.py` | every station2 setting: IPs, sensors, ticks/m, offset, grids, speeds |
| `station2/poses.json` | station2's taught poses (written by `tools/teach.py`) |
| `station2/iolink.py` | reads the laser sensors on the IO-Link master |
| `station2/tools/teach.py` | records station2's poses |
| `station2/tools/calibrate.py` | measures encoder ticks per metre |

## Network
| Device | IP | Used for |
|---|---|---|
| UR3e#1 (station1) | 158.39.162.177 | ur_rtde control, gripper 63352 |
| PIM60 #1 camera | 158.39.162.190 | trigger 2116, result 2114: `(x,y,decision)` |
| UR3e#2 (station2) | 158.39.162.151 | program on 30002, RTDE 30004, dashboard 29999 |
| IO-Link master (belt sensors) | 158.39.162.193 | HTTP |
| Omron LD | 158.39.162.122 | ARCL 7171 |

The PC must be on the lab network (not a phone hotspot).

## Setup (once)
1. `pip install -r requirements.txt`. On a Mac, or with Python 3.13+, `ur_rtde` has no
   ready-made wheel and compiles from source: `brew install boost`, then
   `pip install cmake pybind11 setuptools setuptools_scm` and
   `pip install --no-build-isolation ur_rtde`.
2. Both pendants: **Settings → System → Remote Control → Enable**.
3. MobilePlanner: the routes `Route1` and `Route2` exist and end at the two stations.
4. station2 poses, pendant in **Local** mode: `python station2/tools/teach.py` (or only some:
   `teach.py pick`, `teach.py board`, `teach.py omron`). Move to each pose, press Enter:
   - `pick_above` / `pick`: over the belt at 178 cm, fingers across the belt,
     `pick_above` straight above `pick`.
   - `*_origin`, `*_row_end`, `*_col_end`: three corners of the temp board and of the
     Omron tray, holding a cylinder 2-3 mm above the surface. origin = slot 0,
     row_end = other row / same column, col_end = same row / other column. The other
     slots are calculated from these; grid sizes are `BOARD_GRID` / `OMRON_GRID` in
     `station2/config.py`.

## Run
Both pendants in **Remote** mode.
```
python main.py                           the loop, until Ctrl+C
python main.py 1                         one round
```
Each part on its own:
```
python omron.py                          Route1, then Route2
python station1/station1.py              station1 unloads the Omron (max 10)
python station2/station2.py start        send station2's program, arm above the belt
python station2/station2.py run 4        4 items: belt -> board, Enter, board -> Omron
python station2/station2.py stop         station2's program and the belt stop
python station2/tools/calibrate.py       ticks per metre (pass: 3 runs within 1 %)
```
`main.py` sends station2's program itself. After changing `robot.script`, `config.py` or
`poses.json`, run `station2.py start` again before `run`.

## station2 in detail
When a cylinder passes sensor S1, the PC notes the encoder tick and tells the robot the
tick at which it reaches the pick spot (178 cm). The robot follows the belt (conveyor
tracking), grips, and places it in the next board slot. Several cylinders may be on the
belt at once; when the robot is busy and the next one gets close, the belt pauses (the
encoder stops too, so every position stays right). The gripper is checked after the lift
and again at each slot, so the counts are the robot's own; station2 has no camera.

```
PC                                         UR3e#2 (robot.script, sent on port 30002)
iolink.py   S1 over HTTP, ~14/s            thread: encoder tick, heartbeat -> registers
station2.py RTDE (port 30004) <-> jobs     thread: belt commands (DO5/6/7, AO1)
                                           main:   job 1 belt->board, 2 board->Omron, 3 home
```

Registers (24-47; 0-23 belong to fieldbus/PLC):
| PC → robot | | Robot → PC | |
|---|---|---|---|
| int 25 | belt command: 1 forward, 3 stop | float 24 | encoder tick |
| int 26 | belt command sequence number | int 24 | heartbeat |
| int 27 | job number (new job when it changes) | int 26 | last belt command done |
| int 28 | job type: 1 belt→board, 2 board→Omron, 3 home | int 27 | last job done |
| int 29 / 30 | board slot / Omron slot | int 28 | gripper OBJ (2 = held) |
| float 24 / 25 | belt volts / pause before start (s) | int 29 | job of that OBJ |
| float 27 | tick at which to start following the belt | | |
| int 31 | PC heartbeat, every 0.1 s; quiet for 1 s = the robot stops the belt | | |

Tuning: retune `OFFSET_MM` when the belt speed changes (more negative if the gripper comes
down behind the cylinder, less negative if in front of it).

## Facts from the lab
- Encoder: 1676 ticks/m (calibrated 2026-11-02, 0.6 % spread). The belt runs 17.6 cm/s at 1 V.
- The belt's run outputs latch: a pulse on `conveyor_fwd` starts it, only `conveyor_stop`
  stops it, and it needs 0.2 s after `conveyor_stop` goes Low before a start pulse.
  `conveyor_speed` (analog out 1) must be set to voltage in Installation → I/O Setup.
- station2's fingers close across the belt, so the cylinder must be in the middle of the
  belt (S1 reads 19-22 cm); one off the middle stops the belt and asks you to remove it.
- The Omron tray's far corner is at the edge of UR3e#2's reach (about 500 mm), so the
  robot comes from 6 cm above the tray (`OMRON_ABOVE_M`) instead of 10 cm.
- A program sent on port 30002 gets nothing from the installation: `robot.script` sets up
  the encoder itself. The robot silently drops a program that uses a reserved word (for
  example `pause`) or an unknown function, with no Log entry.
- Network programs end at the first `end` at the start of a line, so `robot.script` is
  indented and `station2.py` refuses to send it otherwise.
- The Robotiq gripper loses its activation at power-off; both stations activate it.
- ur_rtde built on macOS can't keep up at its default 500 Hz: UR3e#1 closes the
  connection after ~1 s and the reconnect crashes Python. station1 connects at 125 Hz.
- If the PC crashes, UR3e#2 stops the belt by itself (PC heartbeat, int 31); the arm
  finishes the move it is in. UR3e#1 has no such guard: check it on the pendant.
- The history of the experiments (`pick_demo/`, stages 0-7, software encoder) is in git.
