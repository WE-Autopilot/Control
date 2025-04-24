import serial
import struct
import math
import threading
from typing import List, Dict, Tuple

class Lidar:
    def __init__(
        self,
        port: str = "/dev/tty.usbserial-0001",
        baudrate: int = 230400,
        timeout: float = 0.01,
    ):
        try:
            # --- serial & parser state ---
            self.ser = serial.Serial(
                port,
                baudrate=baudrate,
                bytesize=8,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=timeout,
            )
        except serial.SerialException as e:
            raise RuntimeError(
                f"Failed to open lidar port({port}). Please make sure port is valid.\n"
                f"Error: \n\t {str(e)}"
                "Try: \n\t1. MAC: /dev/tty.usbserial-0001\n\t2. Linux: /dev/ttyUSB0\n\t3. Windows: COM3 or COM6"
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
        last_sa = None
        while not self._stop_event.is_set():
            ...
            pkt = self._parse_packet(raw)
            sa = pkt["start_angle"]/100.0

            with self._lock:
                if last_sa is not None and sa < last_sa:
                    # rotation rollover detected
                    self._full_scan = [d for p in self._raw_packets for d in p["distances"]]
                    self._raw_packets.clear()
                self._raw_packets.append({"start": sa, "distances": distances})
            last_sa = sa

    def get_scan(self):
        with self._lock:
            if hasattr(self, "_full_scan"):
                return self._full_scan, True
            else:
                # not yet a full revolution
                return [], False

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

    def stop(self):
        """Signal the reader thread to exit and close serial port."""
        self._stop_event.set()
        self._reader_thread.join()
        if self.ser.is_open:
            self.ser.close()

class LidarBlocking:
    def __init__(
        self,
        port: str = "/dev/tty.usbserial-0001",
        baudrate: int = 230400,
        timeout: float = 0.1,
    ):
        try:
            self.ser = serial.Serial(
                port,
                baudrate=baudrate,
                bytesize=8,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=timeout,
            )
        except serial.SerialException as e:
            raise RuntimeError(
                f"Failed to open lidar port({port}). Error: {e}"
            )
        self._rad_factor = math.pi / 180.0

    def _read_packet(self) -> Dict:
        """Read exactly 48 bytes and parse one packet."""
        raw = self.ser.read(48)
        if len(raw) != 48 or raw[0:2] != b"\x54\x2C":
            # skip until we see a good header
            return None

        h, v, s, sa = struct.unpack("<BBHH", raw[:6])
        data = raw[6:42]
        ea, ts, crc = struct.unpack("<HHH", raw[42:])
        measurements = []
        for i in range(0, 36, 3):
            d0, d1, stren = data[i : i + 3]
            distance = ((d1 & 0x3F) << 8) | d0
            measurements.append({
                "distance": distance if not (d1 & 0x80) else 0,
                "strength": stren,
            })
        return {
            "start_angle": sa / 100.0,
            "end_angle":   ea / 100.0,
            "distances":   [m["distance"] for m in measurements],
        }

    def get_scan(self) -> List[float]:
        """
        Block until one full 360° revolution is read,
        then return a single flat list of all distances.
        """
        scan_packets = []
        last_start = None

        while True:
            pkt = self._read_packet()
            if pkt is None:
                continue

            sa = pkt["start_angle"]
            # first packet
            if last_start is None:
                last_start = sa

            # detect wrap-around: new start < previous start → full revolution
            elif sa < last_start:
                # we’ve wrapped: stop and return everything
                break

            scan_packets.append(pkt)
            last_start = sa

        # flatten in ascending-angle order
        flat = []
        for pkt in sorted(scan_packets, key=lambda p: p["start_angle"]):
            flat.extend(pkt["distances"])
        return flat

    def close(self):
        if self.ser.is_open:
            self.ser.close()
