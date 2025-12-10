import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from collections import deque
import threading
import time
import numpy as np
import signal
import sys

class IMUVisualizer:
    def __init__(self, port='COM8', baudrate=115200, max_points=200):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.running = True
        self.shutdown_requested = False
        
        # Data buffers for yaw, pitch, roll
        self.timestamps = deque(maxlen=max_points)
        self.yaw_data = deque(maxlen=max_points)
        self.pitch_data = deque(maxlen=max_points)
        self.roll_data = deque(maxlen=max_points)
        
        # Current values
        self.current_yaw = 0
        self.current_pitch = 0
        self.current_roll = 0
        
        # Serial connection
        self.ser = None
        
        # Threads
        self.serial_thread = None
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Setup GUI
        self.setup_gui()
        
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C signal"""
        print("\nCtrl+C detected. Shutting down...")
        self.shutdown_requested = True
        self.quit()
        sys.exit(0)
    
    def setup_gui(self):
        """Setup the Tkinter GUI with matplotlib plots"""
        self.root = tk.Tk()
        self.root.title("IMU Data Visualizer - Yaw, Pitch, Roll")
        self.root.geometry("1200x800")
        
        # Handle window close button
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        
        # Create matplotlib figure with subplots
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(10, 8))
        self.fig.suptitle('IMU Orientation Data', fontsize=16)
        
        # Setup line plots
        self.line1, = self.ax1.plot([], [], 'r-', label='Yaw')
        self.line2, = self.ax2.plot([], [], 'g-', label='Pitch')
        self.line3, = self.ax3.plot([], [], 'b-', label='Roll')
        
        # Setup 3D visualization
        self.ax4.remove()
        self.ax4 = self.fig.add_subplot(224, projection='3d')
        
        # Configure subplots
        self.ax1.set_title('Yaw (Heading)')
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Degrees')
        self.ax1.grid(True)
        self.ax1.legend()
        self.ax1.set_ylim(-180, 180)
        
        self.ax2.set_title('Pitch')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Degrees')
        self.ax2.grid(True)
        self.ax2.legend()
        self.ax2.set_ylim(-90, 90)
        
        self.ax3.set_title('Roll')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Degrees')
        self.ax3.grid(True)
        self.ax3.legend()
        self.ax3.set_ylim(-180, 180)
        
        self.ax4.set_title('3D Orientation')
        self.ax4.set_xlim(-1, 1)
        self.ax4.set_ylim(-1, 1)
        self.ax4.set_zlim(-1, 1)
        
        # Embed matplotlib figure in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Add control frame
        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Add labels for current values
        self.yaw_label = tk.Label(control_frame, text="Yaw: 0.00°", font=("Arial", 12))
        self.yaw_label.pack(side=tk.LEFT, padx=20)
        
        self.pitch_label = tk.Label(control_frame, text="Pitch: 0.00°", font=("Arial", 12))
        self.pitch_label.pack(side=tk.LEFT, padx=20)
        
        self.roll_label = tk.Label(control_frame, text="Roll: 0.00°", font=("Arial", 12))
        self.roll_label.pack(side=tk.LEFT, padx=20)
        
        # Add quit button
        quit_button = tk.Button(control_frame, text="Quit", command=self.quit, font=("Arial", 12))
        quit_button.pack(side=tk.RIGHT, padx=20, pady=10)
        
        # Add status label
        self.status_label = tk.Label(control_frame, text="Status: Connecting...", font=("Arial", 10))
        self.status_label.pack(side=tk.LEFT, padx=20)
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.status_label.config(text=f"Status: Connected to {self.port}")
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            self.status_label.config(text=f"Status: Connection failed")
            return False
    
    def parse_imu_data(self, line):
        """Parse IMU data line and extract yaw, pitch, roll"""
        try:
            # Split by whitespace
            parts = line.strip().split()
            
            # Check if we have at least 5 values (first two + yaw, pitch, roll)
            if len(parts) >= 5:
                # Extract last three values: yaw, pitch, roll
                yaw = float(parts[-3])
                pitch = float(parts[-2])
                roll = float(parts[-1])
                
                return yaw, pitch, roll
            else:
                return None
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e}, Line: {line}")
            return None
    
    def read_serial_data(self):
        """Thread function to read data from serial port"""
        while self.running and not self.shutdown_requested:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                    # Read a line from serial
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        if line:  # Only process non-empty lines
                            # Parse IMU data
                            print(f"Raw: {line}")
                            data = self.parse_imu_data(line)
                            
                            if data:
                                yaw, pitch, roll = data
                                
                                # Update current values
                                self.current_yaw = yaw
                                self.current_pitch = pitch
                                self.current_roll = roll
                                
                                # Add to buffers
                                current_time = time.time()
                                self.timestamps.append(current_time)
                                self.yaw_data.append(yaw)
                                self.pitch_data.append(pitch)
                                self.roll_data.append(roll)
                                
                                # Update labels
                                if not self.shutdown_requested:
                                    self.root.after(0, self.update_labels, yaw, pitch, roll)
                
                except serial.SerialException as e:
                    print(f"Serial read error: {e}")
                    self.status_label.config(text="Status: Serial error")
                    break
                except Exception as e:
                    if not self.shutdown_requested:
                        print(f"Unexpected error: {e}")
            
            time.sleep(0.001)  # Small delay to prevent CPU overload
        
        print("Serial reading thread stopped")
    
    def update_labels(self, yaw, pitch, roll):
        """Update the displayed labels"""
        if not self.shutdown_requested:
            self.yaw_label.config(text=f"Yaw: {yaw:6.2f}°")
            self.pitch_label.config(text=f"Pitch: {pitch:6.2f}°")
            self.roll_label.config(text=f"Roll: {roll:6.2f}°")
    
    def update_plot(self, frame):
        """Update the matplotlib plots"""
        if self.shutdown_requested:
            return self.line1, self.line2, self.line3
            
        if self.timestamps:
            # Convert timestamps to relative time
            time_ref = self.timestamps[0]
            relative_times = [t - time_ref for t in self.timestamps]
            
            # Update line plots
            self.line1.set_data(relative_times, self.yaw_data)
            self.line2.set_data(relative_times, self.pitch_data)
            self.line3.set_data(relative_times, self.roll_data)
            
            # Update plot limits
            self.ax1.set_xlim(0, max(relative_times) if relative_times else 10)
            self.ax2.set_xlim(0, max(relative_times) if relative_times else 10)
            self.ax3.set_xlim(0, max(relative_times) if relative_times else 10)
            
            # Clear and redraw 3D visualization
            self.ax4.clear()
            self.draw_3d_orientation()
            
        return self.line1, self.line2, self.line3
    
    def draw_3d_orientation(self):
        """Draw 3D representation of orientation"""
        if self.shutdown_requested:
            return
            
        # Convert angles to radians
        yaw_rad = np.radians(self.current_yaw)
        pitch_rad = np.radians(self.current_pitch)
        roll_rad = np.radians(self.current_roll)
        
        # Create coordinate frame
        length = 0.8
        
        # Original axes (X=red, Y=green, Z=blue)
        x_axis = np.array([length, 0, 0])
        y_axis = np.array([0, length, 0])
        z_axis = np.array([0, 0, length])
        
        # Apply rotations in order: yaw (Z), pitch (Y), roll (X)
        # Yaw rotation (around Z)
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Pitch rotation (around Y)
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # Roll rotation (around X)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        # Combine rotations: R = Rz * Ry * Rx
        R = Rz @ Ry @ Rx
        
        # Rotate axes
        x_rotated = R @ x_axis
        y_rotated = R @ y_axis
        z_rotated = R @ z_axis
        
        # Plot rotated axes
        self.ax4.quiver(0, 0, 0, x_rotated[0], x_rotated[1], x_rotated[2], 
                       color='r', linewidth=2, arrow_length_ratio=0.1, label='X')
        self.ax4.quiver(0, 0, 0, y_rotated[0], y_rotated[1], y_rotated[2], 
                       color='g', linewidth=2, arrow_length_ratio=0.1, label='Y')
        self.ax4.quiver(0, 0, 0, z_rotated[0], z_rotated[1], z_rotated[2], 
                       color='b', linewidth=2, arrow_length_ratio=0.1, label='Z')
        
        # Draw a simple cube for reference
        self.draw_reference_cube()
        
        self.ax4.set_xlabel('X')
        self.ax4.set_ylabel('Y')
        self.ax4.set_zlabel('Z')
        self.ax4.set_xlim(-1, 1)
        self.ax4.set_ylim(-1, 1)
        self.ax4.set_zlim(-1, 1)
        self.ax4.legend()
        self.ax4.set_title(f'3D Orientation\nYaw: {self.current_yaw:.1f}°, '
                          f'Pitch: {self.current_pitch:.1f}°, '
                          f'Roll: {self.current_roll:.1f}°')
    
    def draw_reference_cube(self):
        """Draw a simple reference cube"""
        if self.shutdown_requested:
            return
            
        # Cube vertices
        vertices = np.array([
            [-0.3, -0.3, -0.3],
            [0.3, -0.3, -0.3],
            [0.3, 0.3, -0.3],
            [-0.3, 0.3, -0.3],
            [-0.3, -0.3, 0.3],
            [0.3, -0.3, 0.3],
            [0.3, 0.3, 0.3],
            [-0.3, 0.3, 0.3]
        ])
        
        # Cube edges
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top
            [0, 4], [1, 5], [2, 6], [3, 7]   # Sides
        ]
        
        # Plot edges
        for edge in edges:
            self.ax4.plot3D(*zip(vertices[edge[0]], vertices[edge[1]]), 'k-', alpha=0.3)
    
    def start(self):
        """Start the application"""
        # Connect to serial port
        if not self.connect_serial():
            print("Failed to connect to serial port. Exiting...")
            return
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, 
                                          interval=50, blit=False, cache_frame_data=False)
        
        print("Application started. Press Ctrl+C to exit.")
        
        # Start Tkinter main loop with exception handling
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt in main loop")
            self.quit()
        except Exception as e:
            print(f"Error in main loop: {e}")
            self.quit()
    
    def quit(self):
        """Clean up and quit"""
        print("\nShutting down application...")
        
        # Set shutdown flag
        self.shutdown_requested = True
        self.running = False
        
        # Stop serial reading thread
        if self.serial_thread and self.serial_thread.is_alive():
            print("Stopping serial thread...")
            # Give thread time to exit gracefully
            self.serial_thread.join(timeout=2.0)
            if self.serial_thread.is_alive():
                print("Serial thread did not exit gracefully")
        
        # Close serial port
        if self.ser and self.ser.is_open:
            print("Closing serial port...")
            try:
                self.ser.close()
            except:
                pass
        
        # Stop animation
        if hasattr(self, 'ani'):
            try:
                self.ani.event_source.stop()
            except:
                pass
        
        # Close Tkinter window
        if hasattr(self, 'root'):
            try:
                self.root.quit()
                self.root.destroy()
            except:
                pass
        
        # Close matplotlib figures
        if hasattr(self, 'fig'):
            try:
                plt.close(self.fig)
            except:
                pass
        
        print("Application closed.")
        sys.exit(0)

def main():
    """Main function"""
    print("IMU Data Visualizer")
    print("=" * 50)
    print("Make sure your ESP32 is connected to COM8")
    print("and sending data in the format:")
    print("4576 0 142.36 -5.24 -15.82")
    print("=" * 50)
    print("Press Ctrl+C to exit at any time")
    
    # Create and start visualizer
    visualizer = IMUVisualizer(port='COM8', baudrate=115200)
    
    try:
        visualizer.start()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt in main")
        visualizer.quit()
    except Exception as e:
        print(f"Error: {e}")
        visualizer.quit()

if __name__ == "__main__":
    main()