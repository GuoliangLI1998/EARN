#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

class plot_data:
    def __init__(self) -> None:
        # Set default font size for all text elements
        plt.rcParams.update({'font.size': 16})
        
        # Data storage for phi
        self.max_data_points = 100  # Maximum number of points to display
        self.phi_data = []
        self.phi_time = []
        
        # Current values
        self.ro1_value = 0.1
        self.ro2_value = 0.1
        self.phi_value = 0.0

        rospy.init_node('plot_data', anonymous=True)
        
        # Subscribe to topics
        rospy.Subscriber("/ro1", Float64, self.ro1_callback)
        rospy.Subscriber("/ro2", Float64, self.ro2_callback)
        rospy.Subscriber("/phi", Float64, self.phi_callback)

        # Initialize start time
        self.start_time = rospy.get_time()

        self.init_ani()
        
        # Set the window position and size
        self.set_window_position(x=90, y=100, width=750, height=720)

        ani = FuncAnimation(self.fig, self.update, interval=1000, frames=100, repeat=True)
        plt.show()

    def set_window_position(self, x=100, y=100, width=800, height=600):
        """Set the position and size of the figure window based on the backend."""
        backend = plt.get_backend()
        
        try:
            manager = plt.get_current_fig_manager()
            
            if backend == 'TkAgg':
                # Position the window
                manager.window.wm_geometry(f"+{x}+{y}")
                # Optionally resize the window (in pixels)
                manager.resize(width, height)
                
            elif backend == 'WXAgg':
                manager.frame.SetPosition((x, y))
                manager.frame.SetSize((width, height))
                
            elif 'Qt' in backend:  # Covers Qt4Agg, Qt5Agg, etc.
                manager.window.setGeometry(x, y, width, height)
                
            elif backend == 'GTK3Agg':
                manager.window.move(x, y)
                manager.window.resize(width, height)
                
            elif backend in ['MacOSX']:
                # MacOSX backend doesn't support window positioning
                pass
                
            else:
                rospy.logwarn(f"Window positioning not implemented for {backend} backend")
                
        except Exception as e:
            rospy.logwarn(f"Failed to set window position: {e}")

    def ro1_callback(self, data):
        self.ro1_value = data.data
        rospy.loginfo_throttle(5, f"Received ro1: {self.ro1_value}")
        
    def ro2_callback(self, data):
        self.ro2_value = data.data
        rospy.loginfo_throttle(5, f"Received ro2: {self.ro2_value}")
        
    def phi_callback(self, data):
        self.phi_value = data.data
        current_time = rospy.get_time() - self.start_time
        self.phi_data.append(self.phi_value)
        self.phi_time.append(current_time)
        # Limit data points
        if len(self.phi_data) > self.max_data_points:
            self.phi_data.pop(0)
            self.phi_time.pop(0)
        rospy.loginfo_throttle(5, f"Received phi: {self.phi_value}")

    def init_ani(self):
        # Create figure with multiple subplots
        # Adjust figure size here (width, height in inches)
        self.fig = plt.figure(figsize=(12, 10))  # Increased from (10, 6)
        
        gs = gridspec.GridSpec(2, 1, height_ratios=[1, 1])
        
        # First subplot for ro1 and ro2 horizontal bar chart
        self.ax_ro = plt.subplot(gs[0])
        
        # Create horizontal bars
        self.bars = self.ax_ro.barh([r'$\rho_2$', r'$\rho_1$'], [self.ro2_value, self.ro1_value], color=['#FF7F00', '#984EA3'], height=0.5)
        
        # Create text labels for values (will be updated later)
        self.bar_text = [self.ax_ro.text(0, 0, '', va='center'), 
                         self.ax_ro.text(0, 0, '', va='center')]

        # Update bar value text labels
        padding = 0.001  # Small padding to position text just after the bar end
        
        # Update ro1 text (second bar)
        self.bar_text[0].set_position((max(self.ro1_value + padding, 0.01), 1))  # y=1 corresponds to ro1 position
        self.bar_text[0].set_text(f'{self.ro1_value:.3f}')
        
        # Update ro2 text (first bar)
        self.bar_text[1].set_position((max(self.ro2_value + padding, 0.01), 0))  # y=0 corresponds to ro2 position
        self.bar_text[1].set_text(f'{self.ro2_value:.3f}')

        # Set labels and grid
        self.ax_ro.set_xlabel('Parameter Value')
        self.ax_ro.tick_params(axis='both', labelsize=14)
        self.ax_ro.grid(True, axis='x')
        
        # Second subplot for phi values
        self.ax_phi = plt.subplot(gs[1])
        self.phi_line, = self.ax_phi.plot([], [], color='#E41A1C', linewidth=3)
        self.ax_phi.set_xlabel('Time (s)')
        self.ax_phi.set_ylabel(r'$\phi$')
        self.ax_phi.tick_params(axis='both', labelsize=14)
        self.ax_phi.grid(True)
        
        # Adjust the padding between subplots
        self.fig.tight_layout(pad=3.0)  # Increased padding for more space

    def update(self, frame):
        # Update ro1 and ro2 bars
        self.bars[1].set_width(self.ro1_value)  # ro1 is the second bar (index 1)
        self.bars[0].set_width(self.ro2_value)  # ro2 is the first bar (index 0)
        
        # Update bar value text labels
        padding = 0.001  # Small padding to position text just after the bar end
        
        # Update ro1 text (second bar)
        self.bar_text[0].set_position((max(self.ro1_value + padding, 0.01), 1))  # y=1 corresponds to ro1 position
        self.bar_text[0].set_text(f'{self.ro1_value:.3f}')
        
        # Update ro2 text (first bar)
        self.bar_text[1].set_position((max(self.ro2_value + padding, 0.01), 0))  # y=0 corresponds to ro2 position
        self.bar_text[1].set_text(f'{self.ro2_value:.3f}')
        
        # Adjust x-axis limit for bars
        bar_max = max(self.ro1_value, self.ro2_value, 0.1)
        self.ax_ro.set_xlim(0, bar_max * 1.2)  # 20% headroom
            
        # Update phi plot
        if self.phi_time:
            self.phi_line.set_data(self.phi_time, self.phi_data)
            
            # Update phi plot limits
            self.ax_phi.set_xlim(min(self.phi_time), max(self.phi_time) + 0.1)
            if self.phi_data:
                phi_min = min(self.phi_data)
                phi_max = max(self.phi_data)
                padding = (phi_max - phi_min) * 0.1 if phi_max > phi_min else 0.1
                self.ax_phi.set_ylim(phi_min - padding, phi_max + padding)
        
        return [self.bars[0], self.bars[1], self.bar_text[0], self.bar_text[1], self.phi_line]

    def run(self):
        print('Starting to plot ro1, ro2, and phi')
        rospy.spin()

if __name__ == '__main__':
    try:
        pd = plot_data() 
        pd.run()
    except rospy.ROSInterruptException:
        pass