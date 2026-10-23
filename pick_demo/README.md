# RobotLab pick demo: UR3e picks a cylinder from the conveyor

Robot #1 puts a white cylinder (Ø 6 cm, 7 cm tall) in the middle of the belt at its
start; until then a person does it. The belt carries it to our robot. The laptop sees it
with the laser sensors, works out when it reaches the pick spot, and tells the robot.
The robot follows the belt (conveyor tracking), picks the cylinder at a fixed spot in the
middle of the belt, and puts it on the table.

```
LAPTOP (Python, standard library only)          UR3e (PolyScope 5.21, installation line1)
IO-Link master --HTTP ~14/s--> iolink_sensors   thread: tick count -> output float reg 24
RTDE (port 30004) <--- tick, heartbeat, state           heartbeat  -> output int reg 24
RTDE (port 30004) ---> jobs (input registers)   main: wait for job, pick, place, report
```

The laptop opens every connection, so Wi-Fi and the Windows firewall don't matter.
The robot program is started from the pendant (Local mode). Only the robot sets the
belt outputs.

## Facts established so far

| Fact | How we know |
|---|---|
| PolyScope 5.21.3, UR3 (e-Series), robot in Local mode | Dashboard Server |
| The laptop can write RTDE input registers in Local mode | stage 0 readback, robot in Local |
| Belt outputs are *standard* digital outputs 5/6/7, `conveyor_speed` = analog out 1 (1.00 V) | RTDE `actual_digital_output_bits`, `standard_analog_output1` |
| Sensor process data: distance = bits 15-4 in cm (valid 5-200), OUT1 = bit 0 | ifm IODD for DeviceID 372 |
| Port 3 (S1) is an O5D150, the others O5D100; same IODD, same data | IO-Link master |
| Empty belt reads S4 52, S3 52, S1 54, S2 53 cm, no noise (1 cm resolution) | stage 1 baseline |
| One request reads all 4 sensors in ~73 ms (13.7 reads/s) | stage 1 baseline |
| Stage 0 link works: readback, heartbeat (500 Hz), echo, tick in both directions | stage 0 runs |
| **The belt run commands latch.** After `conveyor_fwd`/`conveyor_bcwd` go Low the belt keeps running at full speed. Only `conveyor_stop` High stops it (within ~0.5 s), and setting stop Low again does not restart it | stage 0 runs |
| Belt speed when running smoothly, ticks/s: 0.2 V 47, 0.3 V 60, 0.5 V 83, 1.0 V 136, 1.5 V 188, 2.0 V 240 (about 31 + 105 x V); backward 1.0 V 136. It still runs at 0.2 V. The earlier 73.7 ticks/s at 1 V probably included a stall | stage 2 speed |
| **Starting needs a pause.** With `conveyor_stop` going Low and the start pulse in the same control cycle the belt never started (0 of 5); with 0.2 s between them it always started (5 of 5). Always wait 0.2 s after stop Low | stage 2 start-pause |
| Start: motion begins 0.15-0.2 s after the start pulse, full speed after 0.3-0.7 s. Stop: 7 ticks (0.2 V) to 90 ticks (2.0 V), standing still after 0.3-1.0 s | stage 2 speed |
| **Forward stalls.** At 0.5 and 1.0 V forward the tick count stops completely for 0.6-1.35 s (stepping back 0.5-1.7 ticks), then ramps up like a fresh start. At 1.5/2.0 V only short dips; backward smooth | stage 0 + stage 2 speed |
| **The belt does not stop; the encoder does.** At 1.0 V the cylinder crossed the sensors at a steady 11.9 cm/s (9 segments, all 11.7-12.1 cm/s, laptop clock, tape-measured positions) | stage 2 calibrate |
| During `calibrate` the encoder went stop-go: ~1 s counting at ~220 ticks/s, ~1 s standing still (stepping back 3-7 ticks), over and over; stalled ~40 % of the time. 220 ticks/s / 11.9 cm/s = ~1850 ticks/m, so 1825 is plausible *while the wheel rolls*. Measured ticks/m per run: 854, 1028, 860 (useless) | stage 2 calibrate |
| In the `speed` test at the same 1.0 V the encoder ran a steady 136 ticks/s for seconds at a time. With the belt at 11.9 cm/s that would be 1143 ticks/m, so the encoder also under-counts without stalling. **The encoder is mechanically unreliable**; Conveyor Tracking cannot work until it is fixed (a stop-go encoder would also explain "Acceleration failed to pass sanity check") | stage 2 speed + calibrate |
| **Why the encoder fails:** its red 3D-printed wheel (about 70 mm, so 1825 ticks/m is right) presses on the slack **return run** under the belt, and stands still at intervals while the belt runs: the wheel slips on the belt | looked at in the lab, 2026-10-21 |
| **Decision (2026-10-21): software encoder.** Until the wheel is fixed, the robot program counts ticks itself at the belt speed the laptop measures for each cylinder (`robot/pick_vencoder.script`). The cylinder is always placed in the middle of the belt | decided with the group |
| The software encoder takes over encoder 0: `watch` at 1 V showed a steady 217 ticks/s for 10 s, 0 % standing still | stage 3 `watch`, 2026-10-21 |
| **Stage 3 passed (3 runs):** S2 predicted from S1 with errors -2.4, -6.8, -5.2 mm; the software encoder counted 99.9-100.1 % of the speed sent. The error leans about -5 mm (S2 sees the cylinder slightly early); a fixed offset like this is tuned out at the pick spot in stage 5 | stage 3, 2026-10-21 |
| **The belt speed at 1 V is not the same every day:** 7.33-7.35 cm/s on 2026-10-21 (very steady over 3 runs), 11.9 cm/s on 2026-10-16. That is why every cylinder is timed (S4 -> S1). Ask the lab engineer whether the belt controller has another speed setting | stage 2 calibrate + stage 3 |
| With the cylinder centred, the sensors' nearest-face reading is 19-22 cm (S4 20-21, S3 19-20, S1 21-22, S2 20-22), so the belt centreline is about 23.5 cm from the sensors | stage 3 |
| The IO-Link master once stopped answering for over 2 s on Wi-Fi. The tests now stop the belt at once when that happens. Use a network cable for the demo | stage 3 run 4 |
| The Robotiq gripper loses its activation when the robot is switched off (`STA 0`: Open/Close do nothing). Activate it from the UR+ panel or with the Gripper Activate node | 2026-10-21 |
| **Stage 4 passed:** `pick_stage4` picks the cylinder from the 178 cm mark and places it on the table (main program: Popup, MoveJ pick_above, MoveL pick, Gripper Close, MoveL pick_above, MoveJ place_above, MoveL place, Gripper Open, MoveL place_above) | stage 4, 2026-10-21 |
| **Stage 5 works: the robot picks the cylinder from the moving belt** and puts it on the table. Fingers close across the belt, `pick_above` straight above `pick` (fingertips ~2 cm above the cylinder), MoveL 400 mm/s and 1500 mm/s2, gripper 100 %, start 50 mm early (`--offset-mm -50`): grip in the middle | stage 5, 2026-10-21 |

## Status and next steps (2026-10-21)

Done: stage 0 (link), stage 2 `speed`, `start-pause`, `calibrate`. Stage 1's hands-on
checks (display, mapping, tape, centreline) are not done yet.

The belt keeps running while the robot picks, so the robot must know where the belt is
all the time. The encoder wheel slips, so we use a **software encoder** now (stage 3) and
swap back to the real one when it is fixed.

Stage 3 (software encoder + prediction) passed on 2026-10-21. Stage 4 passed the same day,
and stage 5 picks from the moving belt, tuned to grip in the middle (offset -50).
On 2026-10-22 the robot program moved to Python (`stage5_robot.py`, Remote mode) and picked
from the moving belt; stage 6 picked its first cylinder with it. Stage 7 (several
cylinders on a belt that runs all the time) is written and tested offline only.

Next session:
1. **Stage 5 repeatability:** `python pick_demo\stage5_pick.py --runs 5`; pass 5 of 5.
   Then the same with the robot program run from Python (`stage5_robot.py`, see Stage 5).
2. **Stage 6:** tests 3-4 (see Stage 6), then **Stage 7** with several cylinders.
3. **Encoder fix, for the lab engineer** (the wheel slips on the slack return run):
   move the wheel to a place where the belt is supported or tight (for example where it
   wraps round a roller), add a spring that presses it on, put rubber on the rim (O-ring
   or grip tape), or put the encoder on the axle of the undriven roller. After a fix:
   remove the `pick_vencoder.script` node, then `watch` (pass: 0 % standing still,
   forward and `--backward`) and `calibrate` (pass: 3 runs within 1 %).
4. Optional **stopwatch check** at 1 V: two marks on the frame 1.00 m apart; time a tape
   mark on the belt. Expected ~8.4 s (11.9 cm/s).

## Registers (only 24-47 are used; 0-23 belong to fieldbus/PLC)

| Register | Direction | Meaning |
|---|---|---|
| output float 24 | robot -> laptop | conveyor tick count |
| output int 24 | robot -> laptop | heartbeat (counts up while the program runs) |
| output int 25 | robot -> laptop | stage 0: copy of input int 24 |
| output int 26 | robot -> laptop | sequence number of the last belt command carried out |
| output int 27 | robot -> laptop | last pick job the robot finished (stage 5) |
| output int 28 | robot -> laptop | gripper OBJ at the grip check: 2 = holds the cylinder, 3 = closed on nothing (stage 6) |
| output int 29 | robot -> laptop | job number of that grip check (stage 6) |
| input int 24 | laptop -> robot | stage 0: test counter |
| input int 25 | laptop -> robot | belt command: 1 forward, 2 backward, 3 stop |
| input int 26 | laptop -> robot | belt command sequence number (a command runs when it changes) |
| input int 27 | laptop -> robot | pick job number; a new job starts when it differs from output int 27 (stage 5) |
| input float 24 | laptop -> robot | belt speed in volts (the robot limits it to 0-2 V) |
| input float 25 | laptop -> robot | pause between `conveyor_stop` Low and the start pulse (s) |
| input float 26 | laptop -> robot | belt speed in m/s for the software encoder (stage 3) |
| input float 27 | laptop -> robot | tick at which the robot starts following the belt (stage 5); 1e9 = "parked" (stage 6) |

---

## Stage 0: RTDE link (the arm does not move)

**Goal:** a robot program publishes the encoder tick, and a value makes the round trip
laptop -> robot program -> laptop.

### Build the program on the pendant

1. Copy `robot/pick_link.script` to a USB stick and plug it into the USB port on
   the teach pendant.
2. **Load your installation.** Top bar: **Open... -> Installation**, choose `line1`,
   **Open**. The header must now say *INSTALLATION line1*. If it asks whether to
   save the current installation, answer **No** (don't change `default`).
3. **New... -> Program.** The header shows the new (unnamed) program with
   *INSTALLATION line1*.
4. **Program** tab. Tap **Robot Program** at the top of the program tree. On the
   right, tick **Add Before Start Sequence** and **Program Loops Forever** (it may be
   unticked in a new program; without it the program ends after one pass, about 1 s).
   A **BeforeStart** node appears in the tree. It runs once, when you press Play.
5. **Stop the belt and leave it ready.** The run commands latch, so setting
   `conveyor_fwd` Low does *not* stop a running belt; only `conveyor_stop` does.
   Select **BeforeStart**, then on the left **Basic -> Set**. In the Set node, choose
   the action that sets a digital output, pick `conveyor_fwd` in the list and choose
   **Low**. Add more nodes the same way, each below the previous one:
   `conveyor_bcwd` **Low**, `conveyor_stop` **High**, **Basic -> Wait** 0.5 s,
   `conveyor_stop` **Low**.
   (Tip: select a node, use **Copy** and **Paste** under the tree, then change the output.)
6. **Add the link script.** With the last Set node selected, choose
   **Advanced -> Script**. In the Script node, change the dropdown from **Line** to
   **File** and tap **Edit**. In the editor, use **Open** to pick `pick_link.script` on the
   USB stick, then **Save As** to save a copy on the robot
   (e.g. `/programs/robotlab/pick_link.script`), so the program doesn't need the
   USB stick. Close the editor. The node must sit *inside* BeforeStart, below the Set nodes.
   *No Open button in your editor?* Type the lines from the file instead
   (a USB keyboard plugged into the pendant makes this easy).
7. **Main loop.** In the tree, select the empty line under **Robot Program** that is
   *not* inside BeforeStart, and add **Basic -> Wait** with **1.0 s**. The main loop
   does nothing in stage 0.
8. **Save... -> Save Program As...**, name it `pick_stage0` (e.g. in a folder `robotlab`).
   If it asks about saving the installation, save `line1` only.

The tree should look like this:

```
Robot Program
  BeforeStart
    Set conveyor_fwd = Low
    Set conveyor_bcwd = Low
    Set conveyor_stop = High
    Wait: 0.5
    Set conveyor_stop = Low
    Script: pick_link.script
  Wait: 1.0
```

### Run the test

1. Press **Play** (bottom right). The arm does not move. If PolyScope shows an
   error, take a photo of it.
2. On the laptop:

   ```
   cd pick_demo
   python stage0_link_test.py
   ```
3. While it runs, at the pendant: **I/O** tab -> tap `conveyor_fwd` High for about
   5 s (belt runs forward), then Low. Then `conveyor_bcwd` High for about 2 s, then Low.
   **The belt keeps running** after you set them Low: stop it with `conveyor_stop`
   High, then Low.
4. Stop the program (square button). Watch 2 more seconds, then press Ctrl+C on the laptop.

### Pass when

| Check | What you see | Proves |
|---|---|---|
| readback | equals `sent` | laptop -> controller (already passed) |
| hb | rises while the program runs, freezes after you stop it | the program publishes |
| echo | equals `sent` while the program runs | full round trip |
| tick | rises forward, falls backward (measured: ~135/s at 1 V) | encoder via register |
| fwd/bwd/stop | follow what you tap in the I/O tab | output mapping |

The summary printed at Ctrl+C should say `SEEN` for all five lines.
*If PolyScope won't let you change outputs in the I/O tab while the program runs,*
note it. Stage 2 lets the laptop request belt moves through the program instead.

---

## Stage 1: laser sensors (laptop only, the robot is not used)

```
cd pick_demo
python stage1_sensor_test.py live
python stage1_sensor_test.py mapping
python stage1_sensor_test.py baseline --seconds 30
```

| Test | Do | Pass when |
|---|---|---|
| Display | `live`: compare each value with the number on that sensor's display | equal, +-1 cm |
| Mapping | `mapping`: hold the cylinder in front of S4, S3, S1, S2 in turn (belt start first) | each prints its own name, nothing else |
| Tape | put the cylinder so its near face is 20.0 cm from S1's front face | `live` shows 20 +-2 cm |
| Baseline | `baseline` with an empty belt | every sensor `OK` (spread <= 2 cm, no invalid) |
| Centreline | put the cylinder centred on the belt centreline (the line you taught) in front of S1, then S2 | write down both readings; stage 3 needs them |

---

## Stage 2: belt and encoder (the arm does not move)

**Goal:** the real belt speed, ticks per metre, stop distance, whether the belt needs a
pause after `conveyor_stop` goes Low, and what the forward hiccups are. The laptop asks
the robot program to run and stop the belt; only the robot sets the outputs.

### Build the program on the pendant

1. Copy `robot/pick_link.script` (new version) and `robot/pick_belt_server.script` to
   the USB stick and plug it into the pendant.
2. **Open... -> Program** -> `pick_stage0`. Straight away **Save... -> Save Program As...**
   `pick_stage2`, so stage 0 stays as it is.
3. **Update the link script.** Tap the Script node inside BeforeStart -> **Edit** -> **Open**
   -> `pick_link.script` on the USB stick -> **Save As** over the copy on the robot
   (same name and folder as before) -> close. The new version also works for stage 0.
4. **BeforeStart** must read: `conveyor_fwd` Low, `conveyor_bcwd` Low, `conveyor_stop`
   High, Wait 0.5, `conveyor_stop` Low, Script (see stage 0, step 5).
5. **Main program.** Select the `Wait: 1.0` node. Add **Advanced -> Script**, set it to
   **File**, **Edit -> Open** -> `pick_belt_server.script` on the USB stick -> **Save As**
   on the robot (e.g. next to `pick_link.script`) -> close. Then select the `Wait: 1.0`
   node and delete it (trash-can button under the tree).
6. Tap **Robot Program**: **Program Loops Forever** must be ticked. **Save**.

```
Robot Program                     [x] Program Loops Forever
  BeforeStart
    Set conveyor_fwd = Low
    Set conveyor_bcwd = Low
    Set conveyor_stop = High
    Wait: 0.5
    Set conveyor_stop = Low
    Script: pick_link.script
  Script: pick_belt_server.script
```

### Run the tests

Press **Play** on the pendant first. Run one test at a time, from the project folder:

| Command | Belt | Takes | What it does |
|---|---|---|---|
| `python pick_demo\stage2_belt_test.py check` | any | 1 s | read-only: program, heartbeat, belt outputs, sensors |
| `python pick_demo\stage2_belt_test.py speed` | **empty** | ~1.5 min | 6 s forward at 0.2, 0.3, 0.5, 1.0, 1.5, 2.0 V, then 1.0 V backward |
| `python pick_demo\stage2_belt_test.py start-pause` | **empty** | ~40 s | 10 starts from a blocked belt, half with no pause, half with 0.2 s |
| `python pick_demo\stage2_belt_test.py calibrate` | cylinder | 3 x ~30 s | the cylinder passes all four sensors; ticks between them give ticks/m |

**Ctrl+C stops the belt.** If the laptop script crashes the belt may keep running: set
`conveyor_stop` High in the I/O tab. For `calibrate`, stand at the belt end: the script
stops the belt after S2, but catch the cylinder if it doesn't.
Every run is logged in `pick_demo\logs\` (not committed).

### Pass when

| Test | Pass when |
|---|---|
| check | `program: playing, heartbeat moving` |
| speed | a table row per voltage: ticks/s, start delay, stop distance, stalls. The lowest voltage that starts is the dead zone |
| start-pause | we know whether "no pause" always starts. If it ever fails, keep the 0.2 s pause |
| calibrate | the 3 runs agree within 1 %. That is the real ticks/m; compare with 1825 |

Send the printed tables. If `speed` shows stalls, send the CSV file from `logs\` too.

---

## Stage 3: software encoder and prediction (the arm does not move)

**Goal:** the robot counts ticks itself at the measured belt speed, and from the moment
the cylinder passes S1 the laptop can say at which tick it reaches a spot further on.
The pick (stage 5) uses exactly this, with the pick spot instead of S2.

How it works: `robot/pick_vencoder.script` takes over encoder 0. It counts at 1825
ticks/m, like the real wheel, at the speed in input float 26, and sees the belt start and
stop from the outputs (the same latch rule as the belt controller). The laptop times
each cylinder from S4 to S3 and S4 to S1 and sends that speed.

### Build the program on the pendant

1. Copy `robot/pick_vencoder.script` to the USB stick and plug it into the pendant.
2. **Open... -> Program** -> `pick_stage2`, then straight away **Save Program As...**
   `pick_stage3`.
3. Tap the `Script: pick_link.script` node inside BeforeStart, then **Advanced -> Script**
   (the new node goes below it), **File**, **Edit -> Open** -> `pick_vencoder.script` on the
   USB stick -> **Save As** on the robot next to the others -> close.
4. **Save**.

```
Robot Program                     [x] Program Loops Forever
  BeforeStart
    Set conveyor_fwd = Low
    Set conveyor_bcwd = Low
    Set conveyor_stop = High
    Wait: 0.5
    Set conveyor_stop = Low
    Script: pick_link.script
    Script: pick_vencoder.script
  Script: pick_belt_server.script
```

### Run the tests (Play first, one at a time)

| Command | Belt | Pass when |
|---|---|---|
| `python pick_demo\stage2_belt_test.py watch --seconds 10` | **empty** | a steady speed, **no** `STOP` lines (the software encoder has taken over). The number is the start value the laptop sends (about 134 ticks/s at 1 V), not a measurement |
| `python pick_demo\stage3_predict_test.py` | cylinder, 5 runs | `PASS`: every S2 prediction within 10 mm, encoder 98-102 % of the speed sent |

For each run put the cylinder upright **in the middle of the belt**, before S4 (0-15 cm),
and stand at the belt end. The test also prints the sensors' nearest-face reading with
the cylinder centred: write it down, it is the "centreline" value for later checks.

---

## Stage 4: gripper and place, belt stopped

**Goal:** teach the poses the pick will use, and pick the cylinder from a fixed mark on
the belt and put it on one spot on the table, 5 times in a row. Stage 5 keeps these
poses and adds conveyor tracking around the pick, so the cylinder can be moving.

**Decisions:** the pick mark is **178 cm** from the start of the belt, on the centreline
(cylinder centre; S2 is at 181.5 cm). In stage 5 the robot starts following the belt
there and has picked by about 191 cm. The fingers close **across the belt** (one pad on
the robot side of the cylinder, one on the far side). Then a timing error along the belt
can't make a finger land on the cylinder: the fingers always come down beside it. (We
first had them closing along the belt; with 60 mm cylinder and 85 mm opening that left
12 mm for timing errors, and a finger hit the cylinder in every moving pick.) The belt
speed stays at 1 V. Gripper: Robotiq 2F adaptive gripper
(URCap 1.8.7), driven with its own PolyScope nodes (`Gripper Activate`, `Gripper`,
`Grip Check` under **Program -> URCaps**).

**Helper:** `python pick_demo\stage4_where.py` (read-only, Ctrl+C to stop) shows the TCP as
belt position, offset from the centreline, height, tilt from straight down and the
elbow angle, twice a second. Keep it running on the laptop while you teach.

### Before you start
1. The belt is stopped (the program's BeforeStart stops it). Put a piece of tape on the
   belt **frame** at 178.0 cm (not on the belt, it moves).
2. Put the cylinder upright with its centre at the 178 cm tape, in the middle of the belt
   (19 cm from either edge).
3. Speed slider at about **30 %** while teaching and for the first test runs.

### Build the program
1. **Open... -> Program** -> `pick_stage3`, then **Save Program As...** `pick_stage4`.
2. In the main program (under BeforeStart), select `Script: pick_belt_server.script` and
   delete it. The belt is not used in this stage.
3. **BeforeStart:** select its last node and add **URCaps -> Gripper Activate**, then
   **URCaps -> Gripper** set to **Open**.
4. **Main program**, in this order (each new node goes below the selected one):
   - **Basic -> Popup**: message "Put the cylinder on the 178 cm mark, in the middle.
     Continue = pick." (type: Message).
   - **Basic -> Folder** named `Pick`, and inside it:
     **Basic -> Move** (MoveJ) with one waypoint `pick_above`;
     **Basic -> Move** set to **MoveL** with one waypoint `pick`;
     **URCaps -> Gripper** set to **Close**;
     **URCaps -> Grip Check**, set so the program stops (or shows a message) when no
     object is detected (photo its settings if unsure);
     **Basic -> Move** (MoveL) with one waypoint `pick_above` again (copy-paste the
     first one so it is the same pose).
   - **Basic -> Folder** named `Place`, and inside it: **MoveJ** `place_above`,
     **MoveL** `place`, **URCaps -> Gripper** **Open**, **MoveL** `place_above` (copy-paste).
   - On the MoveL nodes set **tool speed 100 mm/s** to begin with.

```
Robot Program                      [x] Program Loops Forever
  BeforeStart
    (as stage 3: belt outputs, pick_link.script, pick_vencoder.script)
    Gripper Activate
    Gripper Open
  Popup: put the cylinder on the 178 cm mark
  Folder Pick
    MoveJ  pick_above
    MoveL  pick
    Gripper Close
    Grip Check
    MoveL  pick_above
  Folder Place
    MoveJ  place_above
    MoveL  place
    Gripper Open
    MoveL  place_above
```

### Teach the waypoints
Tap a waypoint -> **Set Waypoint**: the Move tab opens; move the robot there, then **OK**.
Rename it by tapping its name. The **UR+** icon at the top right opens the Robotiq
toolbar, where you can open and close the gripper by hand while teaching.

1. **`pick`** (cylinder on the mark, gripper open):
   - Make the tool point straight down: in the Move tab choose feature **Base** and
     hold **Align** until it stops. `stage4_where` should show tilt below 1 deg.
   - Turn the tool (feature **Tool**, rotate about Z) until the fingers close across the
     belt: one pad on the robot side of the cylinder, one on the far side.
   - Lower the gripper around the cylinder: the pads around the middle of the cylinder
     height, fingertips at least 1 cm above the belt.
   - `stage4_where` should show **belt 178.0 +-0.5 cm** and **centreline 0.0 +-0.5 cm**.
     Move with the arrows until it does, then close the gripper with the toolbar to check
     that the cylinder is centred between the pads (it should not slide). Open again,
     then set the waypoint.
2. **`pick_above`**: from `pick`, move straight up 10 cm (feature **Base**, Z+). Set it.
3. **`place_above`** and **`place`**: close the gripper on the cylinder, go up to
   `pick_above`, then move to the table spot. Lower until the cylinder's bottom is 2-3 mm
   above the table: set `place`. Move straight up 10 cm: set `place_above`. Open the
   gripper and put the cylinder back on the mark.
4. **Save**.

### Reach check (cylinder taken off the belt)
In stage 5 the gripper follows the belt from 178 cm to about 191 cm at pick height.
Go to `pick` (in the waypoint, hold **Move robot here**), then move along the belt with
feature **Base**, **Y+**, watching `stage4_where`: write down the **elbow** angle at 178,
185 and 191 cm, and also go back to 175 cm. If PolyScope warns about a singularity or a
limit, stop there and note the belt position.

### Run the test
Press **Play** at 30 % speed, answer the popup, watch the whole pick and place. Then 5
runs in a row (slider up to 100 % when it looks safe).

| Check | Pass when |
|---|---|
| `pick` pose | `stage4_where`: belt 178.0 +-0.5 cm, centreline 0.0 +-0.5 cm, tilt < 1 deg |
| pick and place | 5 of 5: Grip Check sees the cylinder, it stands upright on the table spot, no protective stop |
| reach | the robot reaches 175-191 cm at pick height; elbow angles written down |

Send: the `stage4_where` line at `pick`, `pick_above` and `place`, and the elbow angles.

---

## Stage 5: pick while the belt runs

**How it works:** the counting starts at S1. When S1 has seen the cylinder, the laptop
knows its speed and adds the distance to the pick spot (178 cm, about 40 cm on): it tells
the robot the tick at which the cylinder is there. The robot waits above the spot, starts
following the cylinder at that tick, goes down with it, grips, lifts, stops following and
puts it on the table. The arm can't follow from S1 itself: S1 (138 cm) is out of reach.
The laptop stops the belt when S2 no longer sees the cylinder.

### Build the program (from `pick_stage4`)
1. Copy `robot/pick_belt_thread.script` to the USB stick. **Open** `pick_stage4`,
   **Save Program As** `pick_stage5`.
2. **BeforeStart:** below `pick_vencoder.script` add **Script -> File** `pick_belt_thread.script`.
3. **Main program**, four changes (new nodes go below the selected one):
   - Replace the **Popup** with **Basic -> Wait**, "wait until expression":
     `read_input_integer_register(27) != read_output_integer_register(27)`
   - Below the first `MoveJ pick_above`: **Wait** until expression
     `read_output_float_register(24) >= read_input_float_register(27)` (the tick count
     that `pick_link.script` publishes; PolyScope's expression editor rejects the old
     name `get_conveyor_tick_count`), then
     **Script** (Line) `track_conveyor_linear(p[-0.0247, 0.9997, -0.0038, 0, 0, 0], 1825)`
   - Below the second `MoveL pick_above`: **Script** (Line) `stop_conveyor_tracking()`
   - At the very end: **Script** (Line)
     `write_output_integer_register(27, read_input_integer_register(27))`
4. On `MoveL pick` and the second `MoveL pick_above` set tool speed **250 mm/s** and
   acceleration **1000 mm/s2**, so the pick is done before the cylinder is too far on.
5. **Save**.

```
Robot Program                      [x] Program Loops Forever
  BeforeStart
    (as stage 4) ... Script: pick_vencoder.script
    Script: pick_belt_thread.script
    Gripper Activate
    Gripper Open
  Wait until  read_input_integer_register(27) != read_output_integer_register(27)
  MoveJ  pick_above
  Wait until  read_output_float_register(24) >= read_input_float_register(27)
  Script track_conveyor_linear(p[-0.0247, 0.9997, -0.0038, 0, 0, 0], 1825)
  MoveL  pick
  Gripper Close
  MoveL  pick_above
  Script stop_conveyor_tracking()
  MoveJ  place_above
  MoveL  place
  Gripper Open
  MoveL  place_above
  Script write_output_integer_register(27, read_input_integer_register(27))
```

### Run
**Play** (speed slider at 100 %: the moves were tested in stage 4; keep a hand on the
stop button). The robot waits. Then:
```
python pick_demo\stage5_pick.py
```
For each run put the cylinder in the middle of the belt before S4 and press Enter.
After each run the script prints roughly where the cylinder centre was when the robot
started following (from when S2 first saw it). **Tuned: the default start is 50 mm early
(`--offset-mm -50`), which grips the cylinder in the middle** (2026-10-21, fingers across
the belt, belt at 1 V). Retune by eye in 5 mm steps if the belt speed, the pick pose or
the gripper changes.

**First pick from the moving belt (2026-10-21, job 7):** fingers across the belt,
offset -15, cylinder at about 177.2 cm (S2) when the robot started following. The robot
followed it, gripped it, lifted it and put it on the table; job done after 4.5 s. It
gripped just a little late, so the default is now -20.

**Before the belt runs, check the pick with the belt stopped:** put the cylinder on the
178 cm mark and run `python pick_demo\stage5_pick.py --static`. The robot picks it with
the stage 5 program (it "follows" a belt that stands still).

Runs 4 and 5 (offset -20 and -33, fingers along the belt): a finger hit the cylinder every
time, about 0.2 s after the gripper reached pick height, then protective stop; the
gripper closed on nothing (`OBJ 3`). S2 put the cylinder at 176.9 and 175.5 cm when the
robot started following, so it was early rather than late, and a different finger hit.
**Fix: re-teach `pick` and `pick_above` with the fingers closing across the belt.**
In the `Gripper Close` node set the speed to 100 %, so the grip is quick.

First run (2026-10-21, offset 0): the cylinder was 11-15 mm ahead of the gripper when the
robot started following, one finger hit its top and the robot made a protective stop.
With 60 mm cylinder and 85 mm opening there is only 12.5 mm to spare on each side.
After a protective stop: unlock it on the pendant, move the gripper up and away
(Freedrive), take the cylinder off, press Play again.

Second run (offset -13): timing right (cylinder centre at 177.8 cm when the robot started
following), the robot followed it to ~186 cm, but going down 10 cm and gripping took
about 1 s and ended in a protective stop. **Make the pick quicker:** teach both
`pick_above` so the fingertips are ~2 cm above the cylinder top (not 10 cm above
`pick`; the fingers must still clear the cylinder as it arrives), MoveL `pick` and the
second `pick_above` at 400 mm/s and 1500 mm/s2, `Gripper Close` speed 100 %. Put the
cylinder exactly in the middle of the belt: the fingers close along the belt, so a
cylinder 2 cm to the side lands under a finger (S2 read 23-24 cm, centred is 20-22 cm).

Third run (lower pick_above, faster moves): timing right again (177.7 cm), and the new
TCP log showed the gripper following the belt at exactly its speed. But the new
`pick_above` was taught about 2.5 cm upstream of `pick`, so on the way down the gripper
also slid 2.5 cm forward relative to the cylinder and the front finger clipped its top
edge (program stopped ~0.8 s after the start). **`pick_above` must be straight above
`pick`:** go to `pick`, then move only up (feature Base, Z+). The direct belt stop over
RTDE worked when the program stopped.

| Check | Pass when |
|---|---|
| pick | 5 of 5: the robot follows the cylinder, grips it in the middle, puts it on the table |
| no errors | no protective stop and no "Acceleration failed to pass sanity check" |

### Run the robot program from Python instead of PolyScope

`stage5_robot.py` builds the whole stage 5 robot program as one URScript program and
sends it to the robot (port 30002); no PolyScope program is needed. It does what
`pick_stage5` does, with the same poses, moves and gripper speed, uses the same
`pick_vencoder.script` and `pick_belt_thread.script`, and also reports the grip for
stage 6. It drives the gripper through the Robotiq server (port 63352) instead of the
Gripper nodes, and moves to joint positions, so the active TCP doesn't matter.
The robot only runs programs from the network in **Remote mode**.

1. **Once: allow Remote mode.** Pendant menu (top right) -> **Settings -> System ->
   Remote Control -> Enable**, then **Exit**.
2. **Record the poses** (Local mode; again whenever a pose is re-taught). Open
   `pick_stage5` and power on the robot. Run `python pick_demo\stage5_robot.py teach`.
   For `pick_above`, `pick`, `place_above` and `place` in turn: tap the waypoint in the
   program tree, hold **Move robot here**, press Enter on the laptop. It saves
   `pick_demo\stage5_poses.json` (commit it) with the payload, and checks that
   `pick_above` is straight above `pick`.
3. **Switch to Remote:** tap **Local** at the top right of the pendant -> **Remote Control**.
   In Remote mode the pendant can't Play or jog; switch back to Local for that.
4. `python pick_demo\stage5_robot.py start`: sends the program, waits until it runs
   (the belt stops, the gripper activates if needed and opens).
5. As before: `python pick_demo\stage5_pick.py --static`, then
   `python pick_demo\stage5_pick.py` (or `stage6_loop.py`).
6. `python pick_demo\stage5_robot.py stop` stops the program and the belt. The emergency
   stop on the pendant always works.

`python pick_demo\stage5_robot.py show` prints the program without sending it.

| Check | Pass when |
|---|---|
| start | `pick_stage5 runs on the robot`, the gripper opens |
| static | `--static` picks the cylinder from the 178 cm mark as the PolyScope program did |
| moving belt | 5 of 5, and the TCP in the log follows the belt while gripping |

If `start` says the program did not start: is the robot in Remote mode? Look at the Log
tab (photo). If the pendant has **Settings -> Security -> Services**, the Secondary Client
Interface (30002) and the Dashboard Server must be enabled.
**The robot ends a program sent over the network at the first `end` at the start of a
line**, and runs every line after it on its own (first try on 2026-10-22: the inserted
`.script` files were not indented; nothing moved, but `movel` lines were started).
`stage5_robot.py` now indents everything and refuses to send a program with any other
line at column 0. After that fix (2026-10-22) `teach` and `start` worked on the robot:
the program started as one program and the heartbeat ran after about 2.5 s.

---

## Stage 6: pick loop with grip check (15 minutes)

**Goal:** the robot picks one cylinder after the other for 15 minutes without anyone
touching the laptop or the pendant, and every cylinder gets a result line. A failed pick
never lets a cylinder reach the belt end.

**How it works:**
- Every pick is the stage 5 pick: same poses, same default `--offset-mm` (-50).
- **Grip check.** After the lift, one new Script node (`robot/pick_grip_check.script`) asks
  the Robotiq gripper server on the controller (`GET OBJ`, port 63352, the same server the
  Gripper nodes use) whether the gripper holds something. It writes the answer to output
  int 28 (2 = holds the cylinder, 3 = closed on nothing) and the job number to output int 29.
  On a miss the robot still goes to the table and opens the empty gripper there: that is
  the release, so the program needs no If.
- **One result line per cylinder** on the laptop, with a running count, also saved in
  `logs\stage6_jobs_*.csv`:

  | Result | Meaning |
  |---|---|
  | `PICKED` | the robot finished and the gripper held the cylinder (OBJ 2) |
  | `MISSED` | the robot finished but the gripper came up empty (OBJ 3) |
  | `NO PICK` | no job was sent (S1 did not see the whole cylinder, or it was too late) |
  | `NO GRIP CHECK` | the robot finished without running `pick_grip_check.script` (old program?) |
  | `ROBOT STOPPED` | the robot program stopped during the job (protective stop) |
  | `NO ANSWER` | the robot did not finish within 30 s |
  | `SENSORS LOST` | the IO-Link master did not answer; the belt stopped at once |

- **Belt end.** The belt stops as soon as one of these happens: S2 no longer sees the
  cylinder; the robot reports its grip check (on a miss the cylinder then stays at about
  183 cm, in front of S2); or the cylinder is 12 cm past S2, predicted from the last sensor
  that saw it, so the belt also stops when S2 never sees it (a cylinder that fell over).
  If the robot program stops, the laptop sets `conveyor_stop` directly over RTDE.
- **After anything but PICKED** the belt stays stopped and the laptop says what to do
  (usually: take the cylinder off the belt). With `--auto` it then waits for Enter.
- **`--auto`, ready for robot #1.** No Enter per cylinder: the belt starts when a cylinder
  has stood still for 2 s in front of S4, in the middle of the belt (S4 reads 19-22 cm),
  and S3, S1 and S2 see nothing. That is where robot #1 must put it (centre 17-22 cm from
  the belt start, in the middle). A hand in front of S4 or a cylinder off the middle does
  not start the belt. The speed is then timed from S3 to S1.
- A belt that runs all the time with several cylinders on it: Stage 7.
- A job is only sent when the robot has finished the last one (or waits for a parked one).
- The start tick is now counted from the newest tick when the job is sent (stage 5 counted
  from the tick when the cylinder passed S1: the same when the speed was right; `--auto`
  only knows the speed at S1).
- If the laptop gives up on a job (sensors lost, Ctrl+C) it "parks" it (start tick 1e9): a
  robot still waiting above the belt keeps waiting instead of moving off, and the next job
  releases it.

### Build the program (from `pick_stage5`)
1. Copy `robot/pick_grip_check.script` to the USB stick and plug it into the pendant.
2. **Open... -> Program** -> `pick_stage5`, then straight away **Save Program As...**
   `pick_stage6`, so stage 5 stays as it is.
3. In the main program tap `Script stop_conveyor_tracking()` (below the second
   `MoveL pick_above`). Add **Advanced -> Script** (it goes below), set it to **File**,
   **Edit -> Open** -> `pick_grip_check.script` on the USB stick -> **Save As** on the robot
   next to the others -> close.
4. The new node must sit between `stop_conveyor_tracking()` and `MoveJ place_above`. **Save**.

```
Robot Program                      [x] Program Loops Forever
  BeforeStart
    (as stage 5)
  Wait until  read_input_integer_register(27) != read_output_integer_register(27)
  MoveJ  pick_above
  Wait until  read_output_float_register(24) >= read_input_float_register(27)
  Script track_conveyor_linear(p[-0.0247, 0.9997, -0.0038, 0, 0, 0], 1825)
  MoveL  pick
  Gripper Close
  MoveL  pick_above
  Script stop_conveyor_tracking()
  Script: pick_grip_check.script                       <- new
  MoveJ  place_above
  MoveL  place
  Gripper Open
  MoveL  place_above
  Script write_output_integer_register(27, read_input_integer_register(27))
```

### Run
**Play** first (speed slider at 100 %, a hand near the stop button). Use **one** cylinder:
after each pick take it from the table and put it upright in the middle of the belt **in
front of S4** (19.5 cm). Ctrl+C stops the belt.

| # | Command | Do | Pass when |
|---|---|---|---|
| 1 | `python pick_demo\stage6_loop.py --grip` | close the gripper on the cylinder with the UR+ toolbar and run it; then close it on nothing and run it again | `OBJ 2`, then `OBJ 3`. If the laptop can't connect, go on: the robot asks the same server from inside |
| 2 | `python pick_demo\stage6_loop.py --jobs 1` | Enter, as in stage 5 | `-> PICKED (cylinder held)` |
| 3 | `python pick_demo\stage6_loop.py --jobs 1 --offset-mm 30` | a deliberate miss: the robot starts following 8 cm late and closes behind the cylinder | `-> MISSED (closed on nothing)`, the belt stops with the cylinder at about 185 cm, the robot goes to the table empty and back. (`NO ANSWER` instead: the belt stopped before the robot started; try `--offset-mm 20`) |
| 4 | `python pick_demo\stage6_loop.py --auto` | 15 minutes; only move the cylinder from the table to S4 | `PASS: 15 minutes, every cylinder picked` (about 25 cylinders), nothing past the belt end, no protective stop |

**First run on the robot (2026-10-22):** with the robot program from Python
(`stage5_robot.py start`), `--jobs 1` gave `PICKED`; the cylinder centre was at 173.6 cm
when the robot started following, and the gripper followed the belt at belt speed.

Send the summary lines and the `logs\stage6_jobs_*.csv` file. If a test shows
`NO GRIP CHECK`, the new Script node is missing or in the wrong place; if every job shows
`MISSED` with OBJ -1 or 0, the robot can't reach the gripper server (photo the Log tab).

**Offline test (2026-10-21):** a simulated robot program, belt and sensors (not in the
repository) ran `stage6_loop.py` through a hit, a forced miss, `--offset-mm 30` and -150,
a cylinder lifted off the belt, `--auto` (hand first, off-centre first), a program stop
in the middle of a pick, S2 blind with no grip check, the IO-Link master dropping out and
a belt at 11.9 cm/s: every result as expected; the cylinder's front edge never got past
195 cm (S2 blind, no grip check: stopped by the prediction from S1).

---

## Stage 7: a stream of cylinders on a belt that runs all the time

**Goal:** the belt runs without stopping, cylinders are put on at its start one after the
other, several can be on the belt at once, and the robot picks each one and is ready for
the next. Needs the robot program from Python (`stage5_robot.py start`).

**How it works** (`stage7_stream.py`):
- **The robot waits above the belt.** `stage5_robot.py` now moves to `pick_above` when the
  program starts and goes back there after every place, before it reports the job done.
  A new job can then follow 0.5 s before the pick spot (2.5 s if the robot is somewhere
  else, e.g. with a PolyScope program).
- **Every cylinder is followed** from S3 to S1 to S2; S4 is left out because hands are
  near it while cylinders are put on. Its time from S3 to S1 gives the belt speed, and
  each new sighting is matched to the cylinder predicted nearest that sensor.
- **One job at a time.** When S1 has seen a cylinder its start tick is planned (as in
  stage 6) and queued. The job goes to the robot as soon as the robot is free and back
  above the belt, if there is at least 0.5 s left; otherwise the cylinder is **skipped**.
  The robot needs about 7 s from starting to follow one cylinder until it is back above
  the belt, so **keep cylinders 8-9 s apart** (about 60 cm at 7.3 cm/s, more on a faster
  belt); the laptop prints the gap in cm and seconds.
- Until the first belt speed is measured (S3 -> S1), a sensor reading belongs to the oldest
  cylinder that sensor hasn't seen yet: a guessed speed can be far off (11.9 cm/s on
  2026-10-16 against a guess of 7.3; in the simulation that made S1 take the first cylinder
  for a new one, and it fell off the belt end before this was fixed).
- **A cylinder is skipped** when S1 saw it in fewer than 3 readings or further than 25 cm
  away: it is off the middle of the belt. (2026-10-22, stage 6 with the belt running: S4,
  S3 read 25 cm and S1 27 cm in a single reading, 4-5 cm off the middle; S1's timing was
  0.4 s off and the robot came 5 cm too late. Off the middle is also unsafe: with the
  fingers across the belt only about 1.2 cm to either side is free.)
- **The belt stops** when a cylinder the robot doesn't hold (missed, skipped, not seen
  by S1) would come 12 cm past S2, when the robot program stops, or when the IO-Link
  master doesn't answer. Every cylinder still on the belt is then reported `NOT PICKED`;
  take them all off and press Enter, and the belt starts again.
- One line per cylinder (`PICKED`, `MISSED`, `SKIPPED`, `NOT PICKED`) and
  `logs\stage7_cylinders_*.csv` with the gap to the cylinder before.

### Run
1. `python pick_demo\stage5_robot.py start` (the arm moves above the belt).
2. `python pick_demo\stage7_stream.py` (15 minutes; `--minutes 3` for a short test).
3. While the belt runs, put cylinders upright in the middle of the belt at its start,
   about 60 cm apart. Take each one from the table after the robot has put it there.

| Check | Pass when |
|---|---|
| stream | 15 minutes, every cylinder `PICKED`, the belt only stopped at the end |
| spacing | cylinders 8-9 s apart: never `SKIPPED`; much closer: `SKIPPED`, the belt stops before the end |

**Offline test (2026-10-22, simulated robot, belt and sensors):** 4 cylinders 67 cm
(9 s) apart, three on the belt at once: all picked, the belt stopped only at the end;
5 s apart: the second `SKIPPED`, the belt stopped before the end; a forced miss, an
off-centre cylinder seen once by S1 and a program stop: as expected; a belt at 11.9 cm/s
with 8 s gaps: all picked. The front edge never got past 197 cm.

---

## Next stages

| # | Build | Pass test |
|---|---|---|
| later | Robot #1 puts the cylinder in front of S4 (`stage6_loop.py --auto` waits for it); it must not do so while S3, S1 or S2 sees a cylinder | 15 min with robot #1, nobody at the belt |
| later | Real encoder fixed: remove the `pick_vencoder.script` node, rerun `watch` and `calibrate` | 0 % standing still; 3 runs within 1 % |

Gripper (stage 4): Robotiq 2F on the wrist connector (RS-485). The Robotiq URCap
1.8.7 on the robot accepts text commands on port 63352 (`SET POS 255 SPE 128 FOR 100 GTO 1`,
`GET OBJ`). The laptop can use that for tests. During a pick the robot program
sends the same commands to `127.0.0.1:63352`, so the grip is timed with the arm.
Only one of them should drive the gripper at a time.
