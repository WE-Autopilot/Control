import matplotlib.pyplot as plt
import numpy as np
import time
import threading
from typing import List, Dict

# --- We'll re-use only the flat-buffer insertion logic here ---
class DummyLidarBuffer:
    def __init__(self):
        self._flat_packets: List[Dict] = []
        self._lock = threading.Lock()

    def _insert_flat_packet(self, start: float, end: float, distances: List[float]):
        with self._lock:
            # drop any packets that overlap this angular span
            kept = [
                p for p in self._flat_packets
                if not (start <= p["end"] and end >= p["start"])
            ]
            # append newest last
            kept.append({"start": start, "end": end, "distances": distances})
            self._flat_packets = kept

    def get_packets(self) -> List[Dict]:
        with self._lock:
            return list(self._flat_packets)


def plot_packets_radial(packets: List[Dict]):
    """Draw each packet as a radial bar from start→end at radius=avg(distances)."""
    ax.clear()
    for pkt in packets:
        theta = np.deg2rad(pkt["start"])
        width = np.deg2rad(pkt["end"] - pkt["start"])
        radius = float(sum(pkt["distances"]) / len(pkt["distances"]))
        ax.bar(theta, radius, width=width, bottom=0.0, alpha=0.6, edgecolor='k')
    ax.set_ylim(0, max((sum(p["distances"])/len(p["distances"]) for p in packets), default=1) * 1.2)
    ax.set_title(f"{len(packets)} packets in buffer")
    plt.draw()
    plt.pause(0.5)  # pause so you can see the update


if __name__ == "__main__":
    lidar_buf = DummyLidarBuffer()

    # turn on interactive plotting
    plt.ion()
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, polar=True)

    # define 5 synthetic packets: (start°, end°, distances)
    test_packets = [
        (  0.0,  30.0, [100, 110, 105]),  # A
        ( 25.0,  60.0, [200, 190, 195]),  # B overlaps A → replaces A
        ( 65.0, 100.0, [300, 310, 305]),  # C no overlap
        ( 95.0, 130.0, [400, 390, 395]),  # D overlaps C → replaces C
        (200.0, 240.0, [150, 155, 152]),  # E no overlap
    ]

    for start, end, dists in test_packets:
        lidar_buf._insert_flat_packet(start, end, dists)
        packets = lidar_buf.get_packets()
        plot_packets_radial(packets)
        time.sleep(0.5)

    # leave the final plot up
    plt.ioff()
    plt.show()
