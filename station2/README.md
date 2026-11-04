# Station 2: UR3e#2, belt → temp board → Omron

Items come along the belt from UR3e#1. When an item passes sensor S1, the laptop notes the
encoder tick and tells the robot the tick at which the item reaches the pick spot
(178 cm). The robot follows the belt (conveyor tracking), grips the item and puts it in
the next slot on the temp board. When x items are on the board and the Omron is at
UR3e#2, the robot moves them to the Omron, and PIM60 #2 checks that the board is empty.

Several items may be on the belt at once. When the robot is busy and the next item gets
close, the belt pauses: the encoder stops too, so every item's position stays right.

```
laptop                                     UR3e#2 (robot.script, sent on port 30002)
iolink.py   S1 over HTTP, ~14/s            thread: encoder tick, heartbeat -> registers
station2.py RTDE (port 30004) <-> jobs     thread: belt commands (DO5/6/7, AO1)
                                           main:   job 1 belt->board, 2 board->Omron, 3 PIM60
```

## Files
| File | What |
|---|---|
| `config.py` | every setting: IPs, sensor positions, ticks/m, offset, grid sizes, PIM60 input |
| `station2.py` | `Station2`: `start()`, `fill_board(x)`, `board_to_omron(n)`, `stop()` |
| `robot.script` | the whole robot program; `station2.py` fills in poses and settings |
| `iolink.py` | reads the laser sensors |
| `tools/teach.py` | records the poses into `poses.json` |
| `tools/calibrate.py` | measures encoder ticks per metre |

## Setup (once)
1. `pip install -r requirements.txt` (from the repo root).
2. Pendant: **Settings → System → Remote Control → Enable**.
3. Local mode: `python station2/tools/teach.py`. Move the robot to each pose and press Enter:
   `pick_above` / `pick` over the belt at 178 cm, fingers across the belt, `pick_above`
   straight above `pick`; three corners each of the temp board and of the Omron tray
   (holding an item 2-3 mm above the surface). Set the grid sizes in `config.py`.
4. Set `PIM60_DI` in `config.py` to the robot input the camera is wired to (High = items seen).

## Run
Switch the pendant to **Remote**, then:
```
python station2/station2.py start        program on the robot, arm above the belt
python station2/station2.py run 3        3 items: belt -> board, Enter, board -> Omron
python station2/station2.py stop         program and belt stop
```
`python station2/tools/calibrate.py` checks ticks per metre (pass: 3 runs within 1 %).
Retune `OFFSET_MM` when the belt speed changes: more negative if the gripper comes down
behind the item, less negative if in front of it.

## Registers (24-47; 0-23 belong to fieldbus/PLC)
| Laptop → robot | | Robot → laptop | |
|---|---|---|---|
| int 25 | belt command: 1 forward, 3 stop | float 24 | encoder tick |
| int 26 | belt command sequence number | int 24 | heartbeat |
| int 27 | job number (new job when it changes) | int 26 | last belt command done |
| int 28 | job type: 1 belt→board, 2 board→Omron, 3 PIM60 | int 27 | last job done |
| int 29 / 30 | board slot / Omron slot | int 28 | gripper OBJ (2 = held) |
| float 24 / 25 | belt volts / pause before start (s) | int 29 | job of that OBJ |
| float 27 | tick at which to start following the belt | int 30 | PIM60: 1 items, 0 empty, -1 none |

## Facts from the lab
- Encoder: 1676 ticks/m (calibrated 2026-11-02, 0.6 % spread). The belt runs 17.6 cm/s at 1 V.
- The belt's run outputs latch: a pulse on `conveyor_fwd` starts it, only `conveyor_stop`
  stops it, and it needs 0.2 s after `conveyor_stop` goes Low before a start pulse.
- With the fingers closing across the belt, a timing error along the belt can't put a
  finger on the item. The item must be in the middle of the belt (S1 reads 19-22 cm).
- The Robotiq gripper loses its activation at power-off; the program activates it.
- Network programs end at the first `end` at the start of a line, so `robot.script` is
  indented and `station2.py` refuses to send it otherwise.
- The history of the experiments (stages 0-7, software encoder) is in git before this folder.
