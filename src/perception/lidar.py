import serial
import struct
import math
import time
from collections import defaultdict

class Lidar:
    def __init__(self, port, baudrate=230400):
        """
        Initialize the Lidar class.
        
        Args:
            port (str): Serial port to connect to the LIDAR sensor.
            baudrate (int): Baud rate for serial communication.
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_buffer = b""
        self.angles_rad = []
        self.distances = []
        self.RAD_FACTOR = math.pi / 180.0  # Pre-calculate conversion factor
        
        # Initialize the serial connection
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0.01
        )
    
    def __del__(self):
        """Close serial connection when object is destroyed."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
    
    def parse_packet(self, packet48):
        """
        Parse one 48-byte packet:
          1 byte  - header       (0x54)
          1 byte  - verLen       (0x2C)
          2 bytes - speed
          2 bytes - start_angle
          36 bytes - measurement data (12 x 3 bytes)
          2 bytes - end_angle
          2 bytes - timestamp
          2 bytes - crc
        """
        if len(packet48) != 48:
            raise ValueError("Packet must be exactly 48 bytes.")

        header, ver_len, speed, start_angle = struct.unpack("<BBHH", packet48[:6])
        data_bytes = packet48[6:42]
        end_angle, timestamp, crc = struct.unpack("<HHH", packet48[42:48])

        measurements = []
        for i in range(0, 36, 3):
            d0, d1, strength = data_bytes[i : i + 3]
            # Decode distance: combine low byte and lower 6 bits of high byte.
            distance = ((d1 & 0x3F) << 8) | d0
            invalid_flag = (d1 & 0x80) >> 7
            warning_flag = (d1 & 0x40) >> 6

            measurements.append({
                "distance": distance,
                "strength": strength,
                "invalid": invalid_flag,
                "warning": warning_flag
            })

        return {
            "header": header,
            "ver_len": ver_len,
            "speed": speed,
            "start_angle": start_angle,   # in hundredths of a degree
            "measurements": measurements,
            "end_angle": end_angle,       # in hundredths of a degree
            "timestamp": timestamp,
            "crc": crc
        }
    
    def process_serial_data(self):
        """
        Read all available data from the serial port and process complete packets.
        Returns the number of packets processed.
        """
        packets_processed = 0
        # Read available data (up to 1024 bytes)
        data = self.ser.read(1024)
        if data:
            self.serial_buffer += data

        # Process all complete packets in the buffer.
        while True:
            # Look for the header bytes: 0x54, 0x2C.
            idx = self.serial_buffer.find(b"\x54\x2C")
            if idx < 0:
                break  # No header found.
            if idx + 48 > len(self.serial_buffer):
                break  # Not enough data for a complete packet.
            
            packet_bytes = self.serial_buffer[idx : idx + 48]
            self.serial_buffer = self.serial_buffer[idx + 48 :]
            try:
                packet = self.parse_packet(packet_bytes)
            except ValueError:
                continue

            # Convert start and end angles (in hundredths of a degree) to degrees.
            start_angle_deg = packet["start_angle"] / 100.0
            end_angle_deg = packet["end_angle"] / 100.0
            angle_diff = end_angle_deg - start_angle_deg
            if angle_diff < 0:
                angle_diff += 360.0

            num_meas = len(packet["measurements"])
            for i, meas in enumerate(packet["measurements"]):
                frac = i / (num_meas - 1) if num_meas > 1 else 0
                angle_deg = start_angle_deg + angle_diff * frac
                angle_rad = angle_deg * self.RAD_FACTOR  # convert to radians
                distance = meas["distance"] if not meas["invalid"] else 0

                self.angles_rad.append(angle_rad)
                self.distances.append(distance)

            packets_processed += 1

        return packets_processed
    
    def get_scan(self, num_packets=200, timeout=10):
        """
        Collect LIDAR scan data for a specified number of packets.
        
        Args:
            num_packets (int): Number of packets to collect for a complete scan.
            timeout (float): Maximum time in seconds to wait for the scan to complete.
            
        Returns:
            list: A list of distances starting from the forward-facing direction (0°)
                 and wrapping around clockwise. Returns None if timeout occurs.
        """
        # Reset data accumulators
        self.angles_rad = []
        self.distances = []
        
        # Process packets until we have enough data
        start_time = time.time()
        packets_processed = 0
        
        while packets_processed < num_packets and (time.time() - start_time) < timeout:
            new_packets = self.process_serial_data()
            packets_processed += new_packets
            if new_packets == 0:
                time.sleep(0.001)  # Small delay to prevent CPU hogging
        
        if packets_processed < num_packets:
            print(f"Warning: Only collected {packets_processed} of {num_packets} packets before timeout")
        
        # Organize data by angle bins (360 bins for each degree)
        angle_bins = defaultdict(list)
        for angle, distance in zip(self.angles_rad, self.distances):
            # Convert to degrees and find the bin (0-359)
            angle_deg = (angle * 180.0 / math.pi) % 360
            bin_idx = int(angle_deg)
            angle_bins[bin_idx].append(distance)
        
        # Create the ordered scan data starting from 0° (forward-facing)
        ordered_scan = []
        for angle in range(360):
            # Use the average of non-zero distances in each bin, or 0 if none
            distances = [d for d in angle_bins.get(angle, []) if d > 0]
            if distances:
                ordered_scan.append(sum(distances) / len(distances))
            else:
                ordered_scan.append(0)
        
        return ordered_scan
    
    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()