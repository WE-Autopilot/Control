import serial
import struct
import math
import time
import array
from collections import defaultdict
from bisect import bisect_left  # For faster binary search

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
        self.serial_buffer = bytearray(2048)  # Pre-allocated buffer for better performance
        self.buffer_size = 0
        self.angles_rad = array.array('f')  # Use array.array for better performance
        self.distances = array.array('H')   # Unsigned short (2 bytes) for distances
        
        # Constants
        self.RAD_FACTOR = math.pi / 180.0  # Pre-calculate conversion factor
        self.TWO_PI = 2.0 * math.pi

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
            "start_angle": start_angle,
            "measurements": measurements,
            "end_angle": end_angle,
            "timestamp": timestamp,
            "crc": crc
        }

    def process_serial_data(self):
        """
        Read available data and process complete packets.
        Returns the number of packets processed.
        """
        packets_processed = 0
        
        # Read available data into buffer
        bytes_read = self.ser.readinto(memoryview(self.serial_buffer)[self.buffer_size:])
        if bytes_read:
            self.buffer_size += bytes_read

        # Fast packet processing
        buffer_view = memoryview(self.serial_buffer)[:self.buffer_size]
        buffer_bytes = bytes(buffer_view)  # Convert to bytes for find operation
        
        while True:
            # Find packet header
            idx = buffer_bytes.find(b"\x54\x2C")
            if idx < 0 or idx + 48 > self.buffer_size:
                # Shift unprocessed data to beginning of buffer
                if idx >= 0:
                    # Keep data that might be part of a packet
                    remaining = self.buffer_size - idx
                    buffer_view[:remaining] = buffer_view[idx:self.buffer_size]
                    self.buffer_size = remaining
                elif self.buffer_size > 1024:
                    # If buffer is large but no header found, discard most of it
                    # but keep the last byte in case it's the start of a header
                    buffer_view[0] = buffer_view[self.buffer_size-1]
                    self.buffer_size = 1
                break

            packet_bytes = buffer_view[idx:idx+48]
            
            try:
                # Unpack directly for better performance
                header, ver_len, speed, start_angle = struct.unpack_from("<BBHH", packet_bytes, 0)
                end_angle, timestamp, crc = struct.unpack_from("<HHH", packet_bytes, 42)
                
                # Process measurements directly
                start_angle_deg = start_angle / 100.0
                end_angle_deg = end_angle / 100.0
                angle_diff = (end_angle_deg - start_angle_deg) % 360.0
                
                # Fast processing of 12 measurements
                for j in range(12):
                    offset = 6 + j * 3
                    d0, d1, strength = packet_bytes[offset], packet_bytes[offset+1], packet_bytes[offset+2]
                    
                    distance = ((d1 & 0x3F) << 8) | d0
                    invalid_flag = (d1 & 0x80) >> 7
                    
                    if not invalid_flag and distance > 0:
                        frac = j / 11  # 11 = num_meas - 1 (12 measurements)
                        angle_deg = start_angle_deg + angle_diff * frac
                        angle_rad = angle_deg * self.RAD_FACTOR
                        
                        self.angles_rad.append(angle_rad)
                        self.distances.append(distance)
                
                packets_processed += 1
            except (ValueError, IndexError, struct.error):
                # Skip invalid packets
                pass
            
            # Remove processed packet from buffer
            remaining = self.buffer_size - (idx + 48)
            if remaining > 0:
                buffer_view[:remaining] = buffer_view[idx+48:self.buffer_size]
            self.buffer_size = remaining
            
            # Update buffer_bytes for next iteration
            buffer_bytes = bytes(buffer_view[:self.buffer_size])

        return packets_processed

    def get_scan(self, num_points=720, timeout=10):
        """
        Collect one full 360° revolution of LIDAR data and interpolate to
        a specified number of evenly spaced points around the circle.

        Args:
            num_points (int): Number of points to interpolate around the circle
            timeout (float): Max seconds to wait for a full revolution

        Returns:
            list: Distance values evenly spaced around the circle
        """
        # Reset accumulators
        self.angles_rad.clear()
        self.distances.clear()
        start_time = time.time()
        last_deg = None

        # Pre-allocate arrays for better performance
        result = [0] * num_points
        two_pi = 2 * math.pi

        # Read until wrap-around detected or timeout
        while True:
            if time.time() - start_time > timeout:
                print("Warning: Timeout before full revolution")
                break

            new_pkts = self.process_serial_data()
            if new_pkts == 0:
                time.sleep(0.001)
                continue

            if self.angles_rad:
                deg = (self.angles_rad[-1] * 180.0 / math.pi) % 360.0
                if last_deg is not None and deg < last_deg:
                    # Wrapped past 0°
                    break
                last_deg = deg

        # Early return if no valid data
        if not self.angles_rad:
            return result

        # Filter and sort in one pass with a list comprehension and sort
        valid_scan_pairs = sorted(
            [(ang % two_pi, dist) for ang, dist in zip(self.angles_rad, self.distances) if dist > 0],
            key=lambda x: x[0]
        )
        
        if not valid_scan_pairs:
            return result
            
        # Extract sorted angles and distances more efficiently
        sorted_angles, sorted_distances = zip(*valid_scan_pairs)
        sorted_angles = list(sorted_angles)
        sorted_distances = list(sorted_distances)
        
        # Handle scan padding for incomplete revolutions
        angle_range = sorted_angles[-1] - sorted_angles[0]
        if angle_range < 1.9 * math.pi:  # Less than ~340°
            # Find all points in the first 180° and copy them wrapped to the end
            cutoff_index = next((i for i, a in enumerate(sorted_angles) if a > math.pi), len(sorted_angles))
            # Append in one batch instead of one-by-one
            sorted_angles.extend([a + two_pi for a in sorted_angles[:cutoff_index]])
            sorted_distances.extend(sorted_distances[:cutoff_index])
                
        # Pre-calculate target angles
        angle_step = two_pi / num_points
        
        # Fast interpolation
        self._fast_interpolate(sorted_angles, sorted_distances, result, angle_step, two_pi)

        quarter = num_points // 4
        result = result[quarter:] + result[:quarter]
        
        return result
    
    def _fast_interpolate(self, angles, distances, result, angle_step, two_pi):
        """
        Fast interpolation using binary search and direct array writing.
        
        Args:
            angles (list): List of measurement angles in radians
            distances (list): List of distance measurements
            result (list): Pre-allocated result array to fill
            angle_step (float): Angular step between target points
            two_pi (float): Cached 2*pi value
        """
        n_angles = len(angles)
        if n_angles <= 1:
            # Not enough data points to interpolate
            if n_angles == 1:
                result[:] = [distances[0]] * len(result)
            return
        
        # Cache array length for performance
        n_targets = len(result)
        
        # Fast path for common case - reuse index between target angles
        # Most of the time, consecutive target angles will be near each other
        last_right_idx = 0
        
        for i in range(n_targets):
            target = (i * angle_step) % two_pi
            
            # Binary search starting from last index (much faster for sorted targets)
            # This is more efficient than linear search for large datasets
            right_idx = last_right_idx
            
            # Small linear search first from previous position (works well for consecutive angles)
            while right_idx < n_angles and angles[right_idx] < target:
                right_idx += 1
                
            # If linear search didn't find it, fall back to binary search
            if right_idx == n_angles or angles[right_idx] >= target and right_idx > 0 and angles[right_idx-1] >= target:
                # Full binary search
                left, right = 0, n_angles - 1
                right_idx = n_angles
                
                while left <= right:
                    mid = (left + right) // 2
                    if angles[mid] < target:
                        left = mid + 1
                    else:
                        right_idx = mid
                        right = mid - 1
            
            # Handle edge cases
            if right_idx == 0:
                # Target is before first point
                left_idx = n_angles - 1
                left_angle = angles[left_idx]
                right_angle = angles[right_idx] + two_pi if angles[right_idx] < angles[left_idx] else angles[right_idx]
            elif right_idx == n_angles:
                # Target is after last point
                right_idx = 0
                left_idx = n_angles - 1
                left_angle = angles[left_idx]
                right_angle = angles[right_idx] + two_pi
            else:
                # Normal case: target between two points
                left_idx = right_idx - 1
                left_angle = angles[left_idx]
                right_angle = angles[right_idx]
                
                # Handle wrapping for interpolation
                if right_idx == 0 and left_idx == n_angles - 1:
                    # Wrap case
                    if angles[right_idx] < angles[left_idx]:
                        right_angle = angles[right_idx] + two_pi
                
            # Fast linear interpolation
            angle_diff = right_angle - left_angle
            if abs(angle_diff) < 1e-6:  # Avoid division by near-zero
                result[i] = distances[left_idx]
            else:
                t = (target - left_angle) / angle_diff
                result[i] = distances[left_idx] * (1 - t) + distances[right_idx] * t
                
            # Save index for next iteration
            last_right_idx = right_idx

    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()