import serial
import struct
import math
import threading
from typing import List, Dict

class VirtualLidar:
    def __init__(
        self
    ):
        pass

    def get_scan(self) -> tuple[List[float], bool]:
        """
        Returns one flattened list of all distances,
        in ascending order of the packet insertion.

        Also returns a boolean indicating if an error occured.
        """
        return [5 for _ in range(1080)], False

    def stop(self):
        """Signal the reader thread to exit and close serial port."""
        pass
