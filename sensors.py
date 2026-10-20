"""
ifm O5D distance sensors on the IO-Link master (same request as conveyor_belt.py).
Port 1 is closest to the door, 4 is furthest away from the door.

Process data is 2 bytes, e.g. "0361":
    bits 15..4 = distance (0x036 = 54, unit mm or cm - not verified yet)
    bit 0      = switching output OUT1
"""
import requests

IOLINK_HOST = "158.39.162.193"

# One kept-open connection: ~60 ms per reading instead of ~1 s with a new one each time
_session = requests.Session()


def read_raw(port, host=IOLINK_HOST, timeout=2.0):
    """Return the 16-bit process data value of a port, or None if unreadable."""
    key = f"/iolinkmaster/port[{port}]/iolinkdevice/pdin"
    r = _session.post(f"http://{host}/", timeout=timeout, json={
        "code": "request", "cid": 1, "adr": "/getdatamulti",
        "data": {"datatosend": [key]},
    })
    entry = r.json()["data"][key]
    if entry.get("code") != 200:
        return None
    return int(entry["data"], 16)


def read_distance(port, host=IOLINK_HOST):
    """Return the measured distance on a port, or None if unreadable."""
    raw = read_raw(port, host)
    return None if raw is None else raw >> 4


if __name__ == "__main__":
    for port in range(1, 5):
        raw = read_raw(port)
        if raw is None:
            print(f"port {port}: no data")
        else:
            print(f"port {port}: distance {raw >> 4}  OUT1 {raw & 1}")
