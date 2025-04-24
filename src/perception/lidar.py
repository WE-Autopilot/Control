import serial
import struct
import math
import threading
from typing import List, Dict

class Lidar:
    def __init__(
        self,
        port: str = "/dev/tty.usbserial-0001",
        baudrate: int = 230400,
        timeout: float = 0.01,
    ):
        # --- serial & parser state ---
        self.ser = serial.Serial(
            port,
            baudrate=baudrate,
            bytesize=8,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=timeout,
        )
        self._buffer = b""
        self._rad_factor = math.pi / 180.0

        # --- flat‐packet buffer + threading ---
        # each entry: {"start": float, "end": float, "distances": List[float]}
        self._flat_packets: List[Dict] = []
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # kick off the reader thread
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True
        )
        self._reader_thread.start()

    def _reader_loop(self):
        """Continuously read & parse raw packets, then insert into flat buffer."""
        while not self._stop_event.is_set():
            data = self.ser.read(1024)
            if data:
                self._buffer += data

            # extract as many 48-byte packets as possible
            while True:
                idx = self._buffer.find(b"\x54\x2C")
                if idx < 0 or idx + 48 > len(self._buffer):
                    break
                raw = self._buffer[idx : idx + 48]
                self._buffer = self._buffer[idx + 48 :]

                try:
                    pkt = self._parse_packet(raw)
                except ValueError:
                    continue

                # compute degrees and flat distances
                sa = pkt["start_angle"] / 100.0
                ea = pkt["end_angle"]   / 100.0
                dists = [
                    float(m["distance"] if not m["invalid"] else 0)
                    for m in pkt["measurements"]
                ]
                self._insert_flat_packet(sa, ea, dists)

    def _parse_packet(self, packet48: bytes) -> Dict:
        if len(packet48) != 48:
            raise ValueError("Packet must be exactly 48 bytes.")
        h,v,s,sa = struct.unpack("<BBHH", packet48[:6])
        data = packet48[6:42]
        ea, ts, crc = struct.unpack("<HHH", packet48[42:])
        measurements = []
        for i in range(0, 36, 3):
            d0,d1,stren = data[i:i+3]
            distance = ((d1 & 0x3F) << 8) | d0
            measurements.append({
                "distance": distance,
                "strength": stren,
                "invalid": bool(d1 & 0x80),
                "warning": bool(d1 & 0x40),
            })
        return {"start_angle": sa, "end_angle": ea, "measurements": measurements}

    def _insert_flat_packet(self, start: float, end: float, distances: List[float]):
        """Replace any existing packet that overlaps [start,end], else just append."""
        with self._lock:
            # keep only non-overlapping packets
            kept = [
                p for p in self._flat_packets
                if not (start <= p["end"] and end >= p["start"])
            ]
            # append newest packet last
            kept.append({"start": start, "end": end, "distances": distances})
            self._flat_packets = kept

    def get_scan(self) -> tuple[List[float], bool]:
        """
        Returns one flattened list of all distances,
        in ascending order of the packet insertion.
        """
        with self._lock:
            flat: List[float] = []
            for pkt in self._flat_packets:
                flat.extend(pkt["distances"])
            return flat, False

    def stop(self):
        """Signal the reader thread to exit and close serial port."""
        self._stop_event.set()
        self._reader_thread.join()
        if self.ser.is_open:
            self.ser.close()
