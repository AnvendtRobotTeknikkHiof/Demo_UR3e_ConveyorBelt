"""
The whole loop from one PC. The Omron starts at station1, loaded with ITEMS cylinders.

  1. station2 starts filling its temp board from the belt (in a thread)
  2. station1 (UR3e#1) picks every cylinder off the Omron onto the belt
  3. the Omron drives Route1 to station2
  4. when all ITEMS are on the board, station2 (UR3e#2) moves them to the Omron
  5. the Omron drives Route2 back to station1, repeat

    python main.py          run until Ctrl+C
    python main.py 2        run 2 rounds

Both UR3e in Remote mode. station2's program is sent at the start; station1 uploads its
own (ur_rtde) for every cylinder.
"""
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "station1"), os.path.join(HERE, "station2")]
import config as station2_config                     # noqa: E402
from omron import ARCL, HOST, PORT, PASSWORD         # noqa: E402
from station1 import get_cylinder, pick_object       # noqa: E402
from station2 import Station2, StationError          # noqa: E402

ITEMS = 4                 # cylinders in the loop, max 6 (the 2x3 board)
ROUTE_TO_2 = "Route1"     # MobilePlanner route station1 -> station2
ROUTE_TO_1 = "Route2"     # station2 -> station1
PLACE_GAP_M = 0.15        # belt travel since station1's last place before it places again


class Background(threading.Thread):
    """Runs fn in a thread; join() returns its result or raises its error here."""

    def __init__(self, fn, *args):
        super().__init__(daemon=True)
        self.fn, self.args, self.result, self.error = fn, args, None, None
        self.start()

    def run(self):
        try:
            self.result = self.fn(*self.args)
        except BaseException as e:
            self.error = e

    def join(self, timeout=None):
        super().join(timeout)
        if self.error:
            raise self.error
        return self.result


def unload_station1(station, filling):
    """station1 puts every cylinder on the Omron onto the belt. Returns how many.
    Before each one the belt must have moved PLACE_GAP_M since the last one: station2
    pauses the belt when it is busy, and station1 always places on the same spot."""
    gap = PLACE_GAP_M * station2_config.TICKS_PER_M
    placed, last_tick = 0, None
    while True:
        cylinder = get_cylinder()
        if cylinder is None:
            return placed
        while last_tick is not None and station.link.state.output_double_register_24 - last_tick < gap:
            if not filling.is_alive():
                filling.join()                       # station2 stopped: raise its error
                raise StationError("station2 stopped filling the board")
            time.sleep(0.05)
        pick_object(*cylinder, decision=1)
        last_tick = station.link.state.output_double_register_24
        placed += 1
        print(f"station1: {placed} on the belt")


def run_round(n, station, omron):
    print(f"\n===== round {n} =====")
    filling = Background(station.fill_board, ITEMS)
    k = unload_station1(station, filling)
    if k != ITEMS:
        raise StationError(f"station1 found {k} of {ITEMS} cylinders: check the Omron and the"
                           " belt, then restart")
    print(f"Omron: {ROUTE_TO_2} to station2")
    if not omron.run_route(ROUTE_TO_2):
        raise StationError(f"The Omron did not finish {ROUTE_TO_2}")
    placed = filling.join()
    if placed != ITEMS:
        raise StationError(f"station2 has {placed} of {ITEMS} on the board")
    moved = station.board_to_omron(placed)
    if moved != ITEMS:
        raise StationError(f"station2 moved {moved} of {ITEMS} to the Omron: check the board")
    print(f"Omron: {ROUTE_TO_1} to station1")
    if not omron.run_route(ROUTE_TO_1):
        raise StationError(f"The Omron did not finish {ROUTE_TO_1}")


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else None
    station = omron = None
    try:
        station = Station2()
        station.start()
        omron = ARCL(HOST, PORT, PASSWORD)
        omron.status()
        n = 0
        while rounds is None or n < rounds:
            n += 1
            run_round(n, station, omron)
        print(f"\n{n} round(s) done.")
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    except StationError as e:
        print(f"\nSTOP: {e}")
    finally:
        if station:
            station.stop()        # program and belt stop; a running fill_board thread ends too
            station.close()
        if omron:
            omron.send("stop")
            omron.close()


if __name__ == "__main__":
    main()
