"""
Minimal client for the Universal Robots RTDE interface (port 30004).
Uses only the Python standard library.

A background thread receives robot outputs (registers, I/O, program state)
at a fixed rate; send() writes robot input registers.

    link = RTDELink(ROBOT_IP,
                    outputs=["timestamp", "output_double_register_24"],
                    inputs=["input_int_register_24"])
    state = link.latest()          # newest values + "_t" (laptop time.monotonic())
    link.send(input_int_register_24=5)
    link.close()

With keep_history=True, drain() returns every package since the last call
(125 per second), for logging.

The laptop opens the connection, so this works over Wi-Fi and through the
Windows firewall. This project only uses registers 24-47 (0-23 belong to
fieldbus/PLC).

Run this file directly for a read-only check of the connection.
"""
import socket
import struct
import threading
import time
from collections import deque

ROBOT_IP = "158.39.162.151"
RTDE_PORT = 30004

# RTDE message types (protocol version 2)
_REQUEST_PROTOCOL_VERSION = 86   # 'V'
_GET_URCONTROL_VERSION = 118     # 'v'
_TEXT_MESSAGE = 77               # 'M'
_DATA_PACKAGE = 85               # 'U'
_SETUP_OUTPUTS = 79              # 'O'
_SETUP_INPUTS = 73               # 'I'
_START = 83                      # 'S'
_PAUSE = 80                      # 'P'

_FORMATS = {"BOOL": "?", "UINT8": "B", "UINT32": "I", "UINT64": "Q", "INT32": "i",
            "DOUBLE": "d", "VECTOR3D": "3d", "VECTOR6D": "6d",
            "VECTOR6INT32": "6i", "VECTOR6UINT32": "6I"}

# runtime_state values
PROGRAM_STATES = {0: "stopping", 1: "stopped", 2: "playing", 3: "pausing",
                  4: "paused", 5: "resuming"}


class RTDEError(RuntimeError):
    pass


class RTDELink:
    def __init__(self, host=ROBOT_IP, outputs=(), inputs=(), frequency=125.0, timeout=3.0,
                 keep_history=False):
        self.sock = socket.create_connection((host, RTDE_PORT), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = b""
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._latest = None
        self._history = deque(maxlen=200000) if keep_history else None
        self._error = None
        self._closing = False

        if not self._request(_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))[0]:
            raise RTDEError("robot refused RTDE protocol version 2")
        self.controller_version = struct.unpack(">IIII", self._request(_GET_URCONTROL_VERSION))

        self.output_names = list(outputs)
        reply = self._request(_SETUP_OUTPUTS, struct.pack(">d", frequency)
                              + ",".join(self.output_names).encode())
        self._out_id, self._out_types = self._parse_setup(reply, self.output_names)

        self.input_names = list(inputs)
        self._in_id = None
        if self.input_names:
            reply = self._request(_SETUP_INPUTS, ",".join(self.input_names).encode())
            self._in_id, self._in_types = self._parse_setup(reply, self.input_names)
            # A data package must carry every input in the recipe, so remember them all.
            # Inputs not yet sent are written as 0.
            self._in_values = {name: 0 for name in self.input_names}

        if not self._request(_START)[0]:
            raise RTDEError("robot refused to start sending RTDE data")
        self.sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

        end = time.monotonic() + 2.0
        while self._latest is None:
            if self._error or time.monotonic() > end:
                raise RTDEError(f"no RTDE data received ({self._error})")
            time.sleep(0.01)

    # ---- public ----

    def latest(self, max_age=1.0):
        """Newest output values as a dict. "_t" is the laptop time.monotonic()
        when the package arrived."""
        if self._error:
            raise ConnectionError(f"RTDE link lost: {self._error}")
        with self._lock:
            values = dict(self._latest)
        if time.monotonic() - values["_t"] > max_age:
            raise TimeoutError(f"no RTDE data for {max_age} s")
        return values

    def drain(self):
        """Every package received since the last drain(), oldest first."""
        if self._history is None:
            raise RTDEError("create the link with keep_history=True to use drain()")
        if self._error:
            raise ConnectionError(f"RTDE link lost: {self._error}")
        with self._lock:
            items = list(self._history)
            self._history.clear()
        return items

    def send(self, **values):
        """Write input registers, e.g. send(input_int_register_24=3)."""
        if self._in_id is None:
            raise RTDEError("no inputs were set up")
        for name, value in values.items():
            if name not in self._in_values:
                raise KeyError(f"{name} is not in the input recipe {self.input_names}")
            self._in_values[name] = value
        payload = struct.pack(">B", self._in_id)
        for name, typ in zip(self.input_names, self._in_types):
            value = self._in_values[name]
            items = value if isinstance(value, (list, tuple)) else (value,)
            payload += struct.pack(">" + _FORMATS[typ], *items)
        self._send(_DATA_PACKAGE, payload)

    def close(self):
        self._closing = True
        try:
            self._send(_PAUSE)
        except OSError:
            pass
        self._thread.join(timeout=1.0)
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- protocol ----

    def _send(self, mtype, payload=b""):
        with self._send_lock:
            self.sock.sendall(struct.pack(">HB", len(payload) + 3, mtype) + payload)

    def _recv_message(self):
        while True:
            if len(self._buf) >= 3:
                size, mtype = struct.unpack_from(">HB", self._buf)
                if len(self._buf) >= size:
                    payload = self._buf[3:size]
                    self._buf = self._buf[size:]
                    return mtype, payload
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("RTDE connection closed by the robot")
            self._buf += chunk

    def _request(self, mtype, payload=b""):
        """Send a setup message and return the payload of the matching reply."""
        self._send(mtype, payload)
        while True:
            rtype, reply = self._recv_message()
            if rtype == mtype:
                return reply
            if rtype == _TEXT_MESSAGE:
                print(f"[rtde] robot says: {self._text(reply)}")

    @staticmethod
    def _parse_setup(reply, names):
        recipe_id = reply[0]
        types = reply[1:].decode().split(",")
        bad = [f"{n} -> {t}" for n, t in zip(names, types) if t not in _FORMATS]
        if bad:
            # NOT_FOUND = unknown name, IN_USE = another RTDE client owns that input
            hint = ""
            if "IN_USE" in types:
                hint = (" -- another program is already connected and writing this register."
                        " Is an earlier test still running in another terminal? Stop it with Ctrl+C.")
            raise RTDEError("RTDE setup failed: " + ", ".join(bad) + hint)
        return recipe_id, types

    @staticmethod
    def _text(payload):
        n = payload[0]
        return payload[1:1 + n].decode(errors="replace")

    def _unpack(self, payload):
        if payload[0] != self._out_id:
            return None
        values, offset = {}, 1
        for name, typ in zip(self.output_names, self._out_types):
            fmt = ">" + _FORMATS[typ]
            items = struct.unpack_from(fmt, payload, offset)
            offset += struct.calcsize(fmt)
            values[name] = items[0] if len(items) == 1 else list(items)
        return values

    def _reader(self):
        try:
            while not self._closing:
                try:
                    mtype, payload = self._recv_message()
                except socket.timeout:
                    continue
                if mtype == _DATA_PACKAGE:
                    values = self._unpack(payload)
                    if values is not None:
                        values["_t"] = time.monotonic()
                        with self._lock:
                            self._latest = values
                            if self._history is not None:
                                self._history.append(values)
                elif mtype == _TEXT_MESSAGE:
                    print(f"[rtde] robot says: {self._text(payload)}")
        except (OSError, ConnectionError) as e:
            if not self._closing:
                self._error = e


if __name__ == "__main__":
    # Read-only check: nothing is written to the robot.
    fields = ["timestamp", "robot_mode", "runtime_state", "actual_TCP_pose",
              "actual_digital_output_bits", "standard_analog_output1",
              "output_double_register_24", "output_int_register_24"]
    with RTDELink(ROBOT_IP, outputs=fields) as link:
        print("controller version:", ".".join(map(str, link.controller_version)))
        for _ in range(3):
            s = link.latest()
            pose = ", ".join(f"{v:.4f}" for v in s["actual_TCP_pose"])
            print(f"t={s['timestamp']:.3f}  program={PROGRAM_STATES.get(s['runtime_state'])}"
                  f"  robot_mode={s['robot_mode']}  DO bits={s['actual_digital_output_bits']:016b}"
                  f"  AO1={s['standard_analog_output1']:.3f}")
            print(f"   TCP=[{pose}]  out_float24={s['output_double_register_24']}"
                  f"  out_int24={s['output_int_register_24']}")
            time.sleep(0.5)
