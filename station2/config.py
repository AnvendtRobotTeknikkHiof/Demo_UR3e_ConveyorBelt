"""Settings for station 2 (UR3e#2: belt -> temp board -> Omron). Change values here only."""

ROBOT_IP = "158.39.162.151"
IOLINK_HOST = "158.39.162.193"
IOLINK_TIMEOUT_S = 0.5    # per sensor request (one takes ~73 ms)
SENSOR_RETRY_S = 10       # sensors not answering: belt paused, retried this long, then stop

# Laser sensors on the IO-Link master: name -> (IO-Link port, distance from belt start in m).
# Only S1 is needed to pick; S4 and S2 are used by tools/calibrate.py.
SENSORS = {"S4": (4, 0.195), "S3": (2, 0.715), "S1": (3, 1.383), "S2": (1, 1.815)}
TRIGGER = "S1"
EMPTY_DROP_CM = 5         # an item is in front of a sensor: this much closer than the empty belt
CENTRED_MAX_CM = 25       # nearest face further than this = item off the middle of the belt

# Belt and encoder
TICKS_PER_M = 1676.0      # tools/calibrate.py 2026-11-02 (1683, 1672, 1674)
# Encoder as in Installation -> Conveyor Tracking (a program sent from the laptop must set
# it up itself): decoder type 1 = quadrature, 2 = rise and fall on A, 3 = rising, 4 = falling;
# inputs A and B are pins 8-11. Check these against the installation screen.
ENCODER_TYPE, ENCODER_A, ENCODER_B = 1, 8, 9
BELT_VOLTS = 1.0          # 17.6 cm/s on 2026-11-02
BELT_DIRECTION = "p[-0.0247, 0.9997, -0.0038, 0, 0, 0]"   # in base, towards the belt end
START_PAUSE_S = 0.2       # between conveyor_stop Low and the start pulse (else it may not start)
MIN_TICK_RATE = 20        # ticks/s; slower while the belt runs = encoder not counting
ENCODER_GRACE_S = 1.5     # the belt needs up to ~0.9 s to reach full speed

# Pick from the belt
PICK_M = 1.78             # item centre at the taught 'pick' pose, from belt start
OFFSET_MM = -45          # + = start following later. Tune at the belt speed in use
PAUSE_LEAD_M = 0.10       # robot busy and the next item this close to its start: pause the belt
END_M = 1.935             # an item not held gets this far: stop the belt (12 cm past S2)

# Temp board and Omron tray: grids from three taught corners (tools/teach.py)
BOARD_GRID = (2, 3)       # rows, columns
OMRON_GRID = (2, 2)
ABOVE_M = 0.10            # approach height above a board slot
OMRON_ABOVE_M = 0.06      # above an Omron slot: the far corner is out of reach at 0.10

# Robot moves
PICK_V, PICK_A = 0.4, 1.5     # down to the belt and up again, m/s, m/s2
MOVE_V, MOVE_A = 0.25, 1.2    # MoveL at the board and the Omron
JOINT_V, JOINT_A = 1.05, 1.4  # MoveJ, rad/s, rad/s2
GRIP_SPEED, GRIP_FORCE = 255, 100   # Robotiq 0-255
