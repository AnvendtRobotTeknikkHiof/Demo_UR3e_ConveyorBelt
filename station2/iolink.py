"""
ifm O5D laser distance sensors on the ifm AL1120 IO-Link master (standard library only).

Process data, 16 bits: bits 15-4 = distance in cm (valid 5-200), bit 0 = OUT1.
The sensors look across the belt, so the distance is the item's near face.
"""
import http.client
import json
import time

import config


class SensorReader:
    """Reads all sensors with one HTTP request (~73 ms) over a kept-open connection."""

    def __init__(self, host=config.IOLINK_HOST, timeout=1.0):
        self.host, self.timeout, self.conn = host, timeout, None
        self.keys = {f"/iolinkmaster/port[{port}]/iolinkdevice/pdin": name
                     for name, (port, _) in config.SENSORS.items()}
        self.body = json.dumps({"code": "request", "cid": 1, "adr": "/getdatamulti",
                                "data": {"datatosend": list(self.keys)}}).encode()

    def _post(self):
        if self.conn is None:
            self.conn = http.client.HTTPConnection(self.host, timeout=self.timeout)
        self.conn.request("POST", "/", body=self.body, headers={"Content-Type": "application/json"})
        return json.loads(self.conn.getresponse().read())

    def read(self):
        """Return (time, {name: distance_cm or None}). The time is the middle of the request."""
        t0 = time.monotonic()
        try:
            reply = self._post()
        except (OSError, http.client.HTTPException):
            self.close()              # the master closes idle connections: reconnect once
            t0 = time.monotonic()
            reply = self._post()
        t = (t0 + time.monotonic()) / 2
        values = {}
        for key, name in self.keys.items():
            entry = reply.get("data", {}).get(key, {})
            raw = int(entry["data"], 16) if entry.get("code") == 200 else None
            values[name] = raw >> 4 if raw is not None and 5 <= raw >> 4 <= 200 else None
        return t, values

    def baseline(self, seconds=2.0):
        """Median distance per sensor with the belt empty."""
        samples = {name: [] for name in config.SENSORS}
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for name, d in self.read()[1].items():
                if d is not None:
                    samples[name].append(d)
        missing = [name for name, s in samples.items() if not s]
        if missing:
            raise RuntimeError(f"No valid reading from {', '.join(missing)}")
        return {name: sorted(s)[len(s) // 2] for name, s in samples.items()}

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None


class Edge:
    """One sensor: sees an item come and go, and reports when its centre passed."""

    def __init__(self, empty_cm):
        self.empty = empty_cm
        self.t_in = None

    def update(self, t, distance_cm):
        """Return (centre time, readings, nearest cm) when an item has just left, else None."""
        d = 0 if distance_cm is None else distance_cm    # no valid distance: very close
        if self.t_in is None:
            if d < self.empty - config.EMPTY_DROP_CM:
                self.t_in, self.readings, self.nearest = t, 1, d
            return None
        if d >= self.empty - 2:
            passed = ((self.t_in + t) / 2, self.readings, self.nearest)
            self.t_in = None
            return passed
        self.readings += 1
        self.nearest = min(self.nearest, d)
        return None


if __name__ == "__main__":
    t, values = SensorReader().read()
    print("  ".join(f"{name} {d} cm" for name, d in values.items()))
