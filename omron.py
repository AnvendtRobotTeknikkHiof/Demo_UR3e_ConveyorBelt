"""
Minimal ARCL client for Omron LD-series AMRs (e.g. LD-90).
Uses only the Python standard library.
"""
import socket
import time

HOST = "158.39.162.122"   # <-- the robot's IP (same as in PuTTY)
PORT = 7171              # <-- ARCL port (same as in PuTTY)
PASSWORD = "12345"       # <-- the ARCL password configured in MobilePlanner


class ARCL:
    def __init__(self, host, port, password, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = ""
        self.read_until("Enter password", timeout=timeout)
        self.send(password)
        # After login ARCL prints the command list, ending with "End of commands"
        self.read_until("End of commands", timeout=timeout)
        print("Connected to ARCL")

    def send(self, cmd):
        print(f">> {cmd}")
        self.sock.sendall((cmd + "\r\n").encode())

    def read_line(self, timeout=1.0):
        """Return one line, or None if nothing arrives within timeout."""
        end = time.time() + timeout
        while "\n" not in self.buf:
            remaining = end - time.time()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                return None
            if not data:
                raise ConnectionError("ARCL connection closed")
            self.buf += data.decode(errors="ignore")
        line, self.buf = self.buf.split("\n", 1)
        line = line.strip()
        if line:
            print(f"<< {line}")
        return line

    def read_until(self, text, timeout=10.0):
        """Read lines until one contains text. Returns that line or None."""
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line(timeout=end - time.time())
            if line and text.lower() in line.lower():
                return line
        return None

    def drain(self):
        """Discard anything ARCL sent while we were not listening."""
        while self.read_line(timeout=0.1) is not None:
            pass

    def goto(self, goal, timeout=120.0):
        """Send the robot to a goal and wait. True = arrived, False = failed/timeout."""
        self.drain()
        self.send(f"goto {goal}")
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line(timeout=1.0)
            if not line:
                continue
            low = line.lower()
            if low.startswith("arrived at"):
                return True
            if "failed" in low or "interrupted" in low or low.startswith("error"):
                return False
        print(f"Timeout going to {goal}, stopping robot")
        self.send("stop")
        return False

    def status(self):
        self.send("oneLineStatus")
        return self.read_until("Status:", timeout=3.0)

    def close(self):
        self.sock.close()


if __name__ == "__main__":
    # Standalone Omron test without the UR3s. The full loop lives in main.py.
    robot = ARCL(HOST, PORT, PASSWORD)
    try:
        robot.status()
        for goal in ["UR3_1", "UR3_2"]:   # <-- your goal names from getgoals
            ok = robot.goto(goal, timeout=120)
            print(f"{goal}: {'arrived' if ok else 'FAILED'}")
            if not ok:
                break
    finally:
        robot.close()
