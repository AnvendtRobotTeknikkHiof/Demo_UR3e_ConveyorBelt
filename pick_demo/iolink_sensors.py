"""
ifm O5D100 / O5D150 laser distance sensors on the ifm AL1120 IO-Link master.
Uses only the Python standard library.

Process data (ifm IODD ifm-000174-20210526-IODD1.1, DeviceID 372, covers the
O5D10x and O5D15x), 16 bits:
    bits 15-4  Distance, 12-bit unsigned, cm, valid 5..200
    bit 0      Switch state OUT1
Example: "0351" -> 0x035 = 53 cm, OUT1 active.

The sensors sit on the robot side of the belt and look across it at the wall,
so the distance is the cross-belt position of the object's near face.
Position along the belt comes from the encoder.
"""
import http.client
import json
import time
from collections import namedtuple

IOLINK_HOST = "158.39.162.193"

# name, IO-Link port, distance from the start of the belt (m), measured with a tape.
# Port mapping not yet verified (stage 1 "mapping" test).
SENSORS = [
    ("S4", 4, 0.195),
    ("S3", 2, 0.715),
    ("S1", 3, 1.383),
    ("S2", 1, 1.815),
]
DIST_MIN_CM, DIST_MAX_CM = 5, 200

# distance_cm is None when the sensor reports no valid distance
SensorValue = namedtuple("SensorValue", "name port position_m distance_cm out1 raw")


def decode_pdin(hex_string):
    """Return (distance_cm or None, out1) for one O5D process-data string."""
    raw = int(hex_string, 16)
    distance = raw >> 4
    if not DIST_MIN_CM <= distance <= DIST_MAX_CM:
        distance = None
    return distance, bool(raw & 1)


class SensorReader:
    """Reads every sensor with one HTTP request and keeps the connection open."""

    def __init__(self, host=IOLINK_HOST, sensors=SENSORS, timeout=1.0):
        self.host = host
        self.timeout = timeout
        self._conn = None
        self._keys = {f"/iolinkmaster/port[{port}]/iolinkdevice/pdin": (name, port, pos)
                      for name, port, pos in sensors}
        self._body = json.dumps({"code": "request", "cid": 1, "adr": "/getdatamulti",
                                 "data": {"datatosend": list(self._keys)}}).encode()

    def _post(self):
        if self._conn is None:
            self._conn = http.client.HTTPConnection(self.host, timeout=self.timeout)
        self._conn.request("POST", "/", body=self._body,
                           headers={"Content-Type": "application/json"})
        return json.loads(self._conn.getresponse().read())

    def read(self):
        """Return (t_sent, t_received, {name: SensorValue}).

        Times are time.monotonic(); the sensor sample was taken somewhere between them.
        """
        t_sent = time.monotonic()
        try:
            reply = self._post()
        except (OSError, http.client.HTTPException):
            # The master may close an idle connection; reconnect once.
            self.close()
            t_sent = time.monotonic()
            reply = self._post()
        t_received = time.monotonic()

        values = {}
        for key, (name, port, pos) in self._keys.items():
            entry = reply.get("data", {}).get(key, {})
            raw = entry.get("data") if entry.get("code") == 200 else None
            distance, out1 = decode_pdin(raw) if raw else (None, False)
            values[name] = SensorValue(name, port, pos, distance, out1, raw)
        return t_sent, t_received, values

    def close(self):
        if self._conn:
            self._conn.close()
            self._conn = None


def measure_baseline(reader, seconds=2.0):
    """Median distance per sensor over `seconds` (belt empty). None = no valid reading."""
    samples = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        _, _, values = reader.read()
        for v in values.values():
            samples.setdefault(v.name, [])
            if v.distance_cm is not None:
                samples[v.name].append(v.distance_cm)
    return {name: sorted(s)[len(s) // 2] if s else None for name, s in samples.items()}


if __name__ == "__main__":
    reader = SensorReader()
    t0, t1, values = reader.read()
    print(f"request took {(t1 - t0) * 1000:.0f} ms")
    for v in values.values():
        print(f"  {v.name} port {v.port} at {v.position_m * 100:5.1f} cm: "
              f"{v.distance_cm} cm  OUT1={int(v.out1)}  raw={v.raw}")
    reader.close()
