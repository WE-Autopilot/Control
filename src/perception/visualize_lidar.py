import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import time
from lidar import Lidar

# Configure the port for your LIDAR sensor
PORT = "/dev/tty.usbserial-0001"  # Update this to match your system

# Create Lidar instance
lidar = Lidar(port=PORT)

# Create a polar plot for visualization
fig = plt.figure(figsize=(10, 8))
ax = plt.subplot(111, polar=True)
line, = ax.plot([], [], 'ro', markersize=1)
ax.set_rmax(2000)  # Max range in mm
ax.grid(True)
ax.set_title("Real-Time LIDAR Visualization")

# Set the polar plot to start at the top (0° is up/forward)
ax.set_theta_zero_location("N")  
# Make angles go clockwise
ax.set_theta_direction(-1)  

def init():
    """Initialize the plot with empty data."""
    line.set_data([], [])
    return (line,)

def update(frame):
    """Update the plot with new LIDAR scan data."""
    # Get a fresh scan from the LIDAR
    scan_data = lidar.get_scan(num_packets=20)  # Use fewer packets for faster updates
    
    if scan_data:
        # Convert to polar coordinates for plotting
        # Angles from 0 to 359 degrees in radians
        angles = np.linspace(0, 2*np.pi, len(scan_data), endpoint=False)
        
        # Update plot data
        line.set_data(angles, scan_data)
        ax.set_title(f"LIDAR Scan - {len(scan_data)} Points")
    
    return (line,)

try:
    # Create animation: update every 50ms
    ani = animation.FuncAnimation(
        fig, update, init_func=init, interval=50, blit=True
    )
    
    plt.show()
    
except KeyboardInterrupt:
    print("Visualization stopped by user")
    
finally:
    # Make sure to close the LIDAR connection
    lidar.close()
    print("LIDAR connection closed")