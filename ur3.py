"""
PC side of the UR3 stations. Uses only the Python standard library.

UR3Station: the PC listens on one port per UR3, and the UR3 program
connects with socket_open() (see ur3_1_pick.script / ur3_2_place.script).
    PC  -> UR3: (cmd,arg)        read with socket_read_ascii_float(2, ...)
                (1,n)  ARRIVED         Omron is parked here, start (n = items, UR3#2 only)
                (2,0)  CONVEYOR_START  Omron left UR3#1, start the belt (UR3#2 only)
                (3,0)  ITEM_AT_SENSOR  item passed the trigger sensor, track and pick it (UR3#2 only)
                (4,0)  CONVEYOR_STOP   stop the belt (UR3#2 only)
    UR3 -> PC : FINISHED <k>     done, k = items actually handled

Dashboard: optional client for the UR Dashboard Server (port 29999), used to
load and start the UR3 program from the PC. On e-Series the robot must be in
Remote Control mode for load/play to work.
"""
import socket
import time

DASHBOARD_PORT = 29999

CMD_ARRIVED = 1
CMD_CONVEYOR_START = 2
CMD_ITEM_AT_SENSOR = 3
CMD_CONVEYOR_STOP = 4


class UR3Station:
    def __init__(self, name, port, host="0.0.0.0"):
        self.name = name
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(1)
        self.conn = None
        self.buf = ""
        self.last_cmd = None   # re-sent if the UR3 reconnects mid-task

    def wait_connected(self, timeout=None):
        print(f"[{self.name}] waiting for UR3 to connect on port {self.port}")
        self.server.settimeout(timeout)
        try:
            self.conn, addr = self.server.accept()
        except socket.timeout:
            raise TimeoutError(f"{self.name} did not connect within {timeout} s")
        self.buf = ""
        print(f"[{self.name}] connected from {addr[0]}")

    def send(self, msg):
        if self.conn is None:
            self.wait_connected()
        print(f"[{self.name}] >> {msg}")
        self.conn.sendall(msg.encode())

    def read_line(self, timeout=1.0):
        """Return one line, or None if nothing arrives within timeout."""
        end = time.time() + timeout
        while "\n" not in self.buf:
            remaining = end - time.time()
            if remaining <= 0:
                return None
            self.conn.settimeout(remaining)
            try:
                data = self.conn.recv(1024)
            except socket.timeout:
                return None
            if not data:
                self.conn.close()
                self.conn = None
                raise ConnectionError(f"{self.name} disconnected")
            self.buf += data.decode(errors="ignore")
        line, self.buf = self.buf.split("\n", 1)
        line = line.strip()
        if line:
            print(f"[{self.name}] << {line}")
        return line

    def send_cmd(self, cmd, arg=0):
        self.last_cmd = (cmd, int(arg))
        self.send(f"({cmd},{int(arg)})")

    def wait_finished(self, timeout=None, resend_on_reconnect=True):
        """Block until the UR3 sends FINISHED. Returns the reported count or None.

        If the UR3 program restarts mid-task, the last command is sent again
        when resend_on_reconnect is True; otherwise a ConnectionError is raised.
        """
        end = None if timeout is None else time.time() + timeout
        while end is None or time.time() < end:
            try:
                line = self.read_line(timeout=1.0)
            except ConnectionError as e:
                if not resend_on_reconnect:
                    raise
                print(f"[{self.name}] {e}, waiting for it to come back")
                self.wait_connected()
                self.send_cmd(*self.last_cmd)
                continue
            if not line or not line.upper().startswith("FINISHED"):
                continue
            parts = line.split()
            return int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else None
        raise TimeoutError(f"{self.name} did not finish within {timeout} s")

    def close(self):
        if self.conn:
            self.conn.close()
        self.server.close()


class Dashboard:
    def __init__(self, ip, timeout=5.0):
        self.ip = ip
        self.sock = socket.create_connection((ip, DASHBOARD_PORT), timeout=timeout)
        self.file = self.sock.makefile("r")
        print(f"[dashboard {ip}] << {self.file.readline().strip()}")   # welcome line

    def cmd(self, text):
        print(f"[dashboard {self.ip}] >> {text}")
        self.sock.sendall((text + "\n").encode())
        reply = self.file.readline().strip()
        print(f"[dashboard {self.ip}] << {reply}")
        return reply

    def start_program(self, program):
        """Power on, release brakes, load and play a .urp program."""
        self.cmd("power on")
        self.cmd("brake release")
        time.sleep(5)   # brake release takes a few seconds
        if not self.cmd(f"load {program}").lower().startswith("loading program"):
            raise RuntimeError(f"Could not load {program} on {self.ip}")
        if "starting" not in self.cmd("play").lower():
            raise RuntimeError(f"Could not start {program} on {self.ip}")

    def stop(self):
        self.cmd("stop")

    def close(self):
        self.sock.close()
