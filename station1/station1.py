import math
import rtde_control
import socket

from robotiq_gripper import RobotiqGripper

ROBOT_IP = "158.39.162.177"
CAMERA_IP = "158.39.162.190"

def get_cylinder():
    # Åpne resultatforbindelsen før vi trigger et nytt bilde.
    with socket.create_connection((CAMERA_IP, 2114), timeout=5) as results:
        results.settimeout(5)

        with socket.create_connection((CAMERA_IP, 2116), timeout=5) as trigger:
            trigger.sendall(b"TRIG")

            with results.makefile("rb") as stream:
                line = stream.readline(4096)

    if not line:
        raise ConnectionError("Kameraet lukket forbindelsen uten resultat.")

    if not line.endswith(b"\n"):
        raise ValueError("Ufullstendig eller for langt kameraresultat.")

    response = line.decode("ascii").strip()
    print("Svar fra kamera:", response)

    if not (response.startswith("(") and response.endswith(")")):
        raise ValueError(f"Uventet kameraformat: {response}")

    x_text, y_text, decision_text = response[1:-1].split(",")

    x_camera = float(x_text)
    y_camera = float(y_text)
    decision = int(decision_text)

    if decision == 0:
        return None

    if decision != 1:
        raise RuntimeError(f"Kameraet rapporterte status {decision}.")

    if not all(math.isfinite(v) for v in (x_camera, y_camera)):
        raise ValueError("Ugyldige kamerakoordinater.")

    return x_camera, y_camera

def camera_to_robot(x_camera, y_camera):
    """Kamera-X/Y i mm → robot-X/Y i meter."""
    x_mm = (
        -0.0595043698446115 * x_camera
        -1.046699179110458 * y_camera
        -584.0207627223303
    )

    y_mm = (
        0.9054482347611532 * x_camera
        -0.01451470359260307 * y_camera
        -175.39126597137064
    )

    return x_mm / 1000, y_mm / 1000


OVER_OMRON_POSE = [
    -0.5183033478625491,
     0.04600417002394193,
     0.008722267552653429,
    -0.05470447921501878,
     0.026886813090100135,
    -0.6282089257936256,
]

OVER_OMRON_Q = [
    -2.9648402372943323,
    -2.488467355767721,
    -0.404511958360672,
    -1.7625190220274867,
     1.5555108785629272,
     0.09653719514608383,
]


ABOVE_BELT_Q = [
    -4.747911993657247,
    -2.55429806331777,
    -0.17803335189819336,
    -1.9885031185545863,
     1.5480715036392212,
     0.025402018800377846,
]

ABOVE_BELT_POSE = [
     0.15578439567215654,
     0.4726478239195613,
     0.022181445189910554,
    -0.04235559832350234,
    -0.008310910090086963,
    -2.340453529781162,
]

PLACE_BELT_POSE = [
     0.15579521719690453,
    0.47267070021945334,
    -0.07738163086626465,
    -0.04232684164967558,
    0.00820371887823487,
    2.340435163542522
]

def pick_object(x_camera, y_camera, decision):
    if decision != 1:
        raise ValueError("Kameraet fant ikke sylinderen.")

    if not all(math.isfinite(v) for v in (x_camera, y_camera)):
        raise ValueError("Ugyldige kamerakoordinater.")

    x_robot, y_robot = camera_to_robot(x_camera, y_camera)

    above = OVER_OMRON_POSE.copy()
    above[0] = x_robot
    above[1] = y_robot

    pick = above.copy()
    pick[2] = -0.047  # Plukkhøyde: −47 mm
    lift = pick.copy()
    lift[2] = 0.121

    if pick[2] >= above[2]:
        raise ValueError("Plukkhøyden må være under tilnærmingshøyden.")

    gripper = RobotiqGripper()
    controller = None
    gripper_connected = False

    try:
        gripper.connect(ROBOT_IP, 63352)
        gripper_connected = True

        # 125 Hz: at the default 500 Hz the robot closes the connection after ~1 s on macOS
        controller = rtde_control.RTDEControlInterface(ROBOT_IP, 125.0)

        # Far out on the Omron the arm can't reach 121 mm up with the gripper straight down:
        # lift as high as it can there, but never lower than the approach height.
        while lift[2] > above[2] and not controller.getInverseKinematicsHasSolution(lift, OVER_OMRON_Q):
            lift[2] = max(lift[2] - 0.01, above[2])
        print(f"Løftehøyde: {lift[2] * 1000:.0f} mm")

        targets = [
            ("Over sylinderen", above, OVER_OMRON_Q),
            ("Plukk", pick, OVER_OMRON_Q),
            ("Løft rett opp fra plukkstedet", lift, OVER_OMRON_Q),
            ("Over båndet", ABOVE_BELT_POSE, ABOVE_BELT_Q),
            ("Plassering", PLACE_BELT_POSE, ABOVE_BELT_Q),
        ]

        for name, pose, q_near in targets:
            if not controller.getInverseKinematicsHasSolution(pose, q_near):
                raise RuntimeError(f"{name}: Ingen kinematisk løsning.")

            if not controller.isPoseWithinSafetyLimits(pose):
                raise RuntimeError(f"{name}: Utenfor sikkerhetsgrensene.")

        if not controller.isProgramRunning():
            raise RuntimeError("RTDE-kontrollskriptet kjører ikke.")

        # Kontroller begge målposene før bevegelse.
        for name, pose in [("Over sylinderen", above), ("Plukk", pick)]:
            if not controller.getInverseKinematicsHasSolution(
                pose, OVER_OMRON_Q
            ):
                raise RuntimeError(f"{name}: Ingen kinematisk løsning.")

            if not controller.isPoseWithinSafetyLimits(pose):
                raise RuntimeError(f"{name}: Utenfor sikkerhetsgrensene.")

        print("1. Aktiverer og åpner griperen")
        gripper.activate(auto_calibrate=False)
        _, status = gripper.move_and_wait_for_pos(0, 40, 30)

        if status != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError("Griperen kunne ikke åpne helt.")

        print("2. Kjører til over_omron")
        if not controller.moveJ(OVER_OMRON_Q, 1.0, 0.20):
            raise RuntimeError("Bevegelsen til over_omron mislyktes.")

        print("3. Kjører over sylinderen")
        if not controller.moveL(above, 0.06, 0.05):
            raise RuntimeError("Bevegelsen over sylinderen mislyktes.")

        print("4. Senker til Z = −74 mm")
        if not controller.moveL(pick, 0.7, 0.03):
            raise RuntimeError("Senkingen mislyktes.")

        print("5. Lukker griperen")
        position, status = gripper.move_and_wait_for_pos(255, 40, 30)
        print("Griperposisjon:", position, "Status:", status)

        # Status 2: fingrene stoppet mot noe under lukking.
        if status != RobotiqGripper.ObjectStatus.STOPPED_INNER_OBJECT:
            raise RuntimeError(
                "Ingen forventet objektregistrering under lukking. "
                "Avbryter uten å løfte."
            )

        print("6. Løfter sylinderen rett opp ved plukkstedet")
        if not controller.moveL(lift, 0.05, 0.03):
            raise RuntimeError("Det vertikale løftet fra plukkstedet mislyktes.")

        print("7. Kjører over transportbåndet")
        # Denne leddbanen må være kontrollert for klaring.
        if not controller.moveJ(ABOVE_BELT_Q, 1.0, 0.20):
            raise RuntimeError("Bevegelsen over båndet mislyktes.")

        print("8. Senker sylinderen til båndet")
        if not controller.moveL(PLACE_BELT_POSE, 0.10, 0.03):
            raise RuntimeError("Senkingen til båndet mislyktes.")

        print("9. Åpner griperen")
        _, status = gripper.move_and_wait_for_pos(0, 40, 30)

        if status != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError("Griperen åpnet ikke som forventet.")

        print("10. Løfter griperen fra båndet")
        if not controller.moveL(ABOVE_BELT_POSE, 0.10, 0.03):
            raise RuntimeError("Tilbaketrekkingen fra båndet mislyktes.")

        print("Plukk og plassering fullført.")


    finally:
        # Ikke åpne automatisk ved avslutning eller feil.
        try:
            if controller is not None:
                try:
                    controller.stopScript()
                finally:
                    controller.disconnect()
        finally:
            if gripper_connected:
                gripper.disconnect()


def main():
    completed = 0
    max_cycles = 10  # Grense under første test.

    for _ in range(max_cycles):
        print("\nSjekker Omron etter en sylinder …")
        cylinder = get_cylinder()

        if cylinder is None:
            print(
                f"Kameraet finner ingen flere sylindere. "
                f"Fullførte sekvenser: {completed}."
            )
            return

        x_camera, y_camera = cylinder
        print(f"Sylinder funnet: X={x_camera}, Y={y_camera}")

        # Må inkludere både plukking og plassering på båndet.
        pick_object(x_camera, y_camera, decision=1)

        completed += 1
        print(f"Sylinder {completed} ferdig plassert.")

        # Neste runde tar et nytt bilde etter fullført plassering.

    print("Testgrensen er nådd. Programmet stopper.")


if __name__ == "__main__":
    main()