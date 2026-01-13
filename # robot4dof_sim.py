#      _ _  ___   ___  ___   ___ _  __  ___ ___ __  __ 
#     | | ||   \ / _ \| __| |_ _| |/ / / __|_ _|  \/  |
#     |_  _| |) | (_) | _|   | || ' <  \__ \| || |\/| |
#       |_||___/ \___/|_|   |___|_|\_\ |___/___|_|  |_|
#                                                      

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports

class RobotArm4DOF_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("4 DOF Robot Arm Simulator - Fixed IK")
        self.root.state('zoomed') 
        
        # Robot parameters dengan variabel tkinter untuk kontrol real-time
        self.U1 = tk.DoubleVar(value=125)  # Link 1 length
        self.U2 = tk.DoubleVar(value=125)  # Link 2 length  
        self.U3 = tk.DoubleVar(value=195)  # Link 3 length
        self.base_height = tk.DoubleVar(value=0)
        
        # Default target position
        self.target_x = tk.DoubleVar(value=200)
        self.target_y = tk.DoubleVar(value=150)
        self.target_z = tk.DoubleVar(value=100)
        
        # Default orientation (roll, pitch, yaw) - for end effector orientation
        self.target_roll = tk.DoubleVar(value=0)
        
        # Forward kinematics joint angles
        self.theta1_fk = tk.DoubleVar(value=0)
        self.theta2_fk = tk.DoubleVar(value=0)
        self.theta3_fk = tk.DoubleVar(value=0)
        self.theta4_fk = tk.DoubleVar(value=0)
        
        # Arduino control variables
        self.arduino_connected = False
        self.serial_connection = None
        self.selected_port = tk.StringVar()
        
        # Joint angles
        self.angles = [0, 0, 0, 0]
        
        self.setup_gui()
        
    def setup_gui(self):
        # Configure grid weights untuk responsive layout
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(2, weight=1)  # Plot area gets more space
        
        # Control panel - lebih compact
        control_frame = ttk.LabelFrame(main_frame, text="Robot Control", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        control_frame.grid_columnconfigure(1, weight=1)
        
        # Robot Parameters Section - lebih compact
        ttk.Label(control_frame, text="Robot Parameters:", font=('Arial', 10, 'bold')).grid(
            row=0, column=0, columnspan=2, pady=(0, 8), sticky=tk.W)
        
        ttk.Label(control_frame, text="L1:", font=('Arial', 9)).grid(row=1, column=0, sticky=tk.W, pady=2)
        entry_l1 = ttk.Entry(control_frame, textvariable=self.U1, width=8, font=('Arial', 9))
        entry_l1.grid(row=1, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="L2:", font=('Arial', 9)).grid(row=2, column=0, sticky=tk.W, pady=2)
        entry_l2 = ttk.Entry(control_frame, textvariable=self.U2, width=8, font=('Arial', 9))
        entry_l2.grid(row=2, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="L3:", font=('Arial', 9)).grid(row=3, column=0, sticky=tk.W, pady=2)
        entry_l3 = ttk.Entry(control_frame, textvariable=self.U3, width=8, font=('Arial', 9))
        entry_l3.grid(row=3, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="Base:", font=('Arial', 9)).grid(row=4, column=0, sticky=tk.W, pady=2)
        entry_base = ttk.Entry(control_frame, textvariable=self.base_height, width=8, font=('Arial', 9))
        entry_base.grid(row=4, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        # Separator
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=8)
        
        # Target position inputs - lebih compact
        ttk.Label(control_frame, text="Target Position:", font=('Arial', 10, 'bold')).grid(
            row=6, column=0, columnspan=2, pady=(0, 8), sticky=tk.W)
        
        ttk.Label(control_frame, text="X:", font=('Arial', 9)).grid(row=7, column=0, sticky=tk.W, pady=2)
        entry_x = ttk.Entry(control_frame, textvariable=self.target_x, width=8, font=('Arial', 9))
        entry_x.grid(row=7, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="Y:", font=('Arial', 9)).grid(row=8, column=0, sticky=tk.W, pady=2)
        entry_y = ttk.Entry(control_frame, textvariable=self.target_y, width=8, font=('Arial', 9))
        entry_y.grid(row=8, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="Z:", font=('Arial', 9)).grid(row=9, column=0, sticky=tk.W, pady=2)
        entry_z = ttk.Entry(control_frame, textvariable=self.target_z, width=8, font=('Arial', 9))
        entry_z.grid(row=9, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        # End effector orientation - DENGAN SLIDER
        ttk.Label(control_frame, text="End Effector Orientation:", font=('Arial', 10, 'bold')).grid(
            row=10, column=0, columnspan=2, pady=(8, 2), sticky=tk.W)
        
        ttk.Label(control_frame, text="Roll:", font=('Arial', 9)).grid(row=11, column=0, sticky=tk.W, pady=2)
        
        # Frame untuk slider dan entry
        roll_frame = ttk.Frame(control_frame)
        roll_frame.grid(row=11, column=1, columnspan=1, padx=(5,0), pady=2, sticky=(tk.W, tk.E))
        roll_frame.grid_columnconfigure(0, weight=1)
        
        # Slider untuk Roll
        roll_slider = ttk.Scale(roll_frame, from_=-180, to=180, variable=self.target_roll, 
                               orient=tk.HORIZONTAL, command=self.on_roll_slider)
        roll_slider.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        # Entry untuk Roll
        entry_roll = ttk.Entry(roll_frame, textvariable=self.target_roll, width=6, font=('Arial', 9))
        entry_roll.grid(row=0, column=1, padx=(5,0), sticky=tk.W)
        
        # Calculate button - lebih compact
        ttk.Button(control_frame, text="Calculate Inverse Kinematics", 
                   command=self.calculate_ik, style='Accent.TButton').grid(
                   row=12, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E))
        
        # Separator
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(
            row=13, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=8)
        
        # Forward Kinematics Section - lebih compact
        ttk.Label(control_frame, text="Forward Kinematics:", font=('Arial', 10, 'bold')).grid(
            row=14, column=0, columnspan=2, pady=(0, 8), sticky=tk.W)
        
        ttk.Label(control_frame, text="θ1:", font=('Arial', 9)).grid(row=15, column=0, sticky=tk.W, pady=2)
        entry_theta1 = ttk.Entry(control_frame, textvariable=self.theta1_fk, width=8, font=('Arial', 9))
        entry_theta1.grid(row=15, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="θ2:", font=('Arial', 9)).grid(row=16, column=0, sticky=tk.W, pady=2)
        entry_theta2 = ttk.Entry(control_frame, textvariable=self.theta2_fk, width=8, font=('Arial', 9))
        entry_theta2.grid(row=16, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="θ3:", font=('Arial', 9)).grid(row=17, column=0, sticky=tk.W, pady=2)
        entry_theta3 = ttk.Entry(control_frame, textvariable=self.theta3_fk, width=8, font=('Arial', 9))
        entry_theta3.grid(row=17, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        ttk.Label(control_frame, text="θ4:", font=('Arial', 9)).grid(row=18, column=0, sticky=tk.W, pady=2)
        entry_theta4 = ttk.Entry(control_frame, textvariable=self.theta4_fk, width=8, font=('Arial', 9))
        entry_theta4.grid(row=18, column=1, padx=(5,0), pady=2, sticky=tk.W)
        
        # Calculate Forward Kinematics button
        ttk.Button(control_frame, text="Calculate Forward Kinematics", 
                   command=self.calculate_fk, style='Accent.TButton').grid(
                   row=19, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E))
        
        # Update Parameters button
        ttk.Button(control_frame, text="Update Parameters", 
                   command=self.update_parameters).grid(
                   row=20, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Arduino Control Frame - Widget baru di sebelah kiri plot
        arduino_frame = ttk.LabelFrame(main_frame, text="Arduino Control", padding="10")
        arduino_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        arduino_frame.grid_columnconfigure(0, weight=1)
        
        # COM Port Selection
        ttk.Label(arduino_frame, text="Select COM Port:", font=('Arial', 10, 'bold')).grid(
            row=0, column=0, columnspan=2, pady=(0, 8), sticky=tk.W)
        
        # COM Port dropdown
        com_ports = self.get_available_ports()
        com_combobox = ttk.Combobox(arduino_frame, textvariable=self.selected_port, 
                                    values=com_ports, state="readonly", width=15)
        com_combobox.grid(row=1, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        if com_ports:
            com_combobox.set(com_ports[0])
        
        # Connect/Disconnect button
        self.connect_button = ttk.Button(arduino_frame, text="Connect", 
                                         command=self.toggle_arduino_connection)
        self.connect_button.grid(row=2, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Separator
        ttk.Separator(arduino_frame, orient=tk.HORIZONTAL).grid(
            row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Send Coordinate button
        ttk.Label(arduino_frame, text="Send to Arduino:", font=('Arial', 10, 'bold')).grid(
            row=4, column=0, columnspan=2, pady=(0, 8), sticky=tk.W)
        
        self.send_button = ttk.Button(arduino_frame, text="Send Coordinate", 
                                     command=self.send_to_arduino, state=tk.DISABLED)
        self.send_button.grid(row=5, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E))
        
        # Status label
        self.arduino_status = ttk.Label(arduino_frame, text="Status: Disconnected", 
                                        foreground="red", font=('Arial', 9))
        self.arduino_status.grid(row=6, column=0, columnspan=2, pady=5, sticky=tk.W)
        
        # Data to be sent display
        ttk.Label(arduino_frame, text="Data to be sent:", font=('Arial', 9, 'bold')).grid(
            row=7, column=0, columnspan=2, pady=(10, 5), sticky=tk.W)
        
        self.data_display = tk.Text(arduino_frame, height=6, width=25, font=('Consolas', 8))
        scrollbar_data = ttk.Scrollbar(arduino_frame, orient=tk.VERTICAL, command=self.data_display.yview)
        self.data_display.configure(yscrollcommand=scrollbar_data.set)
        self.data_display.grid(row=8, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar_data.grid(row=8, column=2, sticky=(tk.N, tk.S))
        
        # Results frame - lebih kecil agar tidak menutupi forward kinematics
        results_frame = ttk.LabelFrame(main_frame, text="RESULTS", padding="20")
        results_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        results_frame.grid_columnconfigure(0, weight=1)
        results_frame.grid_rowconfigure(0, weight=1)
        
        self.results_text = tk.Text(results_frame, height=23, width=45, font=('Consolas', 8))
        scrollbar = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.results_text.yview)
        self.results_text.configure(yscrollcommand=scrollbar.set)
        self.results_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # Plot frame - akan mengembang mengisi space
        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=2, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        
        # Create 3D plot dengan ukuran yang responsive
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Enable zoom dan pan untuk plot 3D
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_button_press)
        self.canvas.mpl_connect('button_release_event', self.on_button_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        
        # Variabel untuk mouse interaction
        self._drag_start = None
        
        # Style untuk button yang lebih menonjol
        style = ttk.Style()
        style.configure('Accent.TButton', font=('Arial', 9, 'bold'))
        
        # Bind Enter key untuk semua entry fields
        entries = [entry_l1, entry_l2, entry_l3, entry_base, entry_x, entry_y, entry_z, entry_roll,
                   entry_theta1, entry_theta2, entry_theta3, entry_theta4]
        for entry in entries:
            entry.bind('<Return>', self.on_entry_return)
        
        # Initial update
        self.calculate_ik()
    
    def get_available_ports(self):
        """Get list of available COM ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def toggle_arduino_connection(self):
        """Connect or disconnect from Arduino"""
        if not self.arduino_connected:
            self.connect_to_arduino()
        else:
            self.disconnect_from_arduino()
    
    def connect_to_arduino(self):
        """Connect to Arduino"""
        try:
            port = self.selected_port.get()
            if not port:
                self.arduino_status.config(text="Status: No port selected", foreground="red")
                return
                
            self.serial_connection = serial.Serial(port, 9600, timeout=1)
            self.arduino_connected = True
            self.connect_button.config(text="Disconnect")
            self.send_button.config(state=tk.NORMAL)
            self.arduino_status.config(text="Status: Connected", foreground="green")
            
            # Update data display
            self.update_data_display()
            
        except Exception as e:
            self.arduino_status.config(text=f"Status: Error - {str(e)}", foreground="red")
    
    def disconnect_from_arduino(self):
        """Disconnect from Arduino"""
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
            self.arduino_connected = False
            self.connect_button.config(text="Connect")
            self.send_button.config(state=tk.DISABLED)
            self.arduino_status.config(text="Status: Disconnected", foreground="red")
        except Exception as e:
            self.arduino_status.config(text=f"Status: Error - {str(e)}", foreground="red")
    
    # --- MODIFIKASI DIMULAI DI SINI ---
    def send_to_arduino(self):
        """Send current coordinates and orientation to Arduino"""
        if not self.arduino_connected or not self.serial_connection:
            self.arduino_status.config(text="Status: Not connected", foreground="red")
            return
        
        try:
            # Get current target values (X, Y, Z, Roll)
            x = self.target_x.get()
            y = self.target_y.get()
            z = self.target_z.get()
            roll = self.target_roll.get()
            
            # Format data for Arduino (X, Y, Z, Roll)
            # Format: X<X_coord>Y<Y_coord>Z<Z_coord>R<Roll_angle>\n
            data_string = f"X{x:.1f}Y{y:.1f}Z{z:.1f}R{roll:.1f}\n"
            
            # Send data
            self.serial_connection.write(data_string.encode())
            
            # Update status
            self.arduino_status.config(text="Status: Target Coordinate Sent", foreground="green")
            
            # Update data display with sent data
            self.data_display.delete(1.0, tk.END)
            self.data_display.insert(tk.END, f"Sent to Arduino:\n{data_string}")
            
        except Exception as e:
            self.arduino_status.config(text=f"Status: Send error - {str(e)}", foreground="red")
    
    def update_data_display(self):
        """Update the data display with current values (X, Y, Z, Roll)"""
        if hasattr(self, 'data_display'):
            x = self.target_x.get()
            y = self.target_y.get()
            z = self.target_z.get()
            roll = self.target_roll.get()
            
            # Display information about the data being sent (X, Y, Z, Roll)
            data_info = f"""Current Target Values:
X: {x:.1f}
Y: {y:.1f} 
Z: {z:.1f}
Roll: {roll:.1f}°

Data Format (Sent):
X{x:.1f}Y{y:.1f}Z{z:.1f}R{roll:.1f}"""
            
            self.data_display.delete(1.0, tk.END)
            self.data_display.insert(tk.END, data_info)
    # --- MODIFIKASI BERAKHIR DI SINI ---
    
    def on_roll_slider(self, event=None):
        """Handle slider movement for roll orientation"""
        self.calculate_ik()
        self.update_data_display()
    
    def on_entry_return(self, event):
        """Handle Enter key press in entry fields"""
        self.calculate_ik()
        self.update_data_display()
    
    def update_parameters(self, event=None):
        """Update simulation when robot parameters are changed"""
        self.calculate_ik()
    
    def calculate_fk(self):
        """Calculate forward kinematics from joint angles"""
        try:
            # Get joint angles from entries
            theta1 = math.radians(self.theta1_fk.get())
            theta2 = math.radians(self.theta2_fk.get())
            theta3 = math.radians(self.theta3_fk.get())
            theta4 = math.radians(self.theta4_fk.get())
            
            angles = [theta1, theta2, theta3, theta4]
            self.angles = angles
            
            # Calculate forward kinematics
            fk_pos, fk_orientation = self.forward_kinematics(angles)
            
            # Update display for FK
            self.update_fk_display(angles, fk_pos, fk_orientation)
            self.plot_robot()
            
        except Exception as e:
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, f"ERROR in Forward Kinematics: {str(e)}")
    
    def update_fk_display(self, angles, fk_pos, fk_orientation):
        """Update results display for forward kinematics"""
        theta1, theta2, theta3, theta4 = angles
        
        # Get current parameters
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        results = f"""=== FORWARD KINEMATICS RESULTS ===
Input Joint Angles (degrees):
  θ1 (Base):    {math.degrees(theta1):.2f}°
  θ2 (Shoulder): {math.degrees(theta2):.2f}°
  θ3 (Elbow):    {math.degrees(theta3):.2f}°
  θ4 (Wrist):    {math.degrees(theta4):.2f}°

Calculated End Effector:
  Position: X={fk_pos[0]:.2f}, Y={fk_pos[1]:.2f}, Z={fk_pos[2]:.2f}
  Orientation: Roll={math.degrees(fk_orientation):.2f}°

Robot Parameters:
  L1={U1:.1f}, L2={U2:.1f}, L3={U3:.1f}, Base={base_height:.1f}"""
        
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, results)
    
    def inverse_kinematics(self, x, y, z, roll=0):
        """Calculate joint angles for given end effector position and orientation"""
        try:
            # Get current parameter values
            U1 = self.U1.get()
            U2 = self.U2.get()
            U3 = self.U3.get()
            base_height = self.base_height.get()
            
            # Convert to cylindrical coordinates
            r = math.sqrt(x**2 + y**2)
            h = z - base_height
            
            # Calculate theta1 (base rotation)
            theta1 = math.atan2(y, x)
            
            # Calculate wrist position (point where U1 and U2 meet)
            desired_angle = math.radians(roll)
            
            # Wrist position calculation considering end effector orientation
            wrist_x = r - U3 * math.cos(desired_angle)
            wrist_z = h - U3 * math.sin(desired_angle)
            
            d = math.sqrt(wrist_x**2 + wrist_z**2)
            
            # Check if point is reachable
            if d > (U1 + U2) or d < abs(U1 - U2):
                return None
            
            # Calculate theta3 (elbow joint) - using law of cosines
            cos_theta3 = (d**2 - U1**2 - U2**2) / (2 * U1 * U2)
            cos_theta3 = max(min(cos_theta3, 1), -1)
            theta3 = math.acos(cos_theta3)
            
            # Calculate theta2 (shoulder joint)
            alpha = math.atan2(wrist_z, wrist_x)
            beta = math.atan2(U2 * math.sin(theta3), U1 + U2 * math.cos(theta3))
            theta2 = alpha - beta
            
            # Calculate theta4 (wrist joint) - now based on desired orientation
            theta4 = desired_angle - (theta2 + theta3)
            
            return [theta1, theta2, theta3, theta4]
            
        except Exception as e:
            print(f"IK Error: {e}")
            return None
    
    def calculate_ik(self):
        """Calculate inverse kinematics and update display"""
        x = self.target_x.get()
        y = self.target_y.get()
        z = self.target_z.get()
        roll = self.target_roll.get()
        
        angles = self.inverse_kinematics(x, y, z, roll)
        
        if angles is not None:
            self.angles = angles
            self.update_display(x, y, z, roll, angles)
            self.plot_robot()
            self.update_data_display()
        else:
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, "ERROR: Target position unreachable!\nTry different coordinates.")
    
    def update_simulation(self, event=None):
        """Update simulation when sliders are moved"""
        self.calculate_ik()
    
    def update_display(self, x, y, z, roll, angles):
        """Update results display"""
        theta1, theta2, theta3, theta4 = angles
        
        # Calculate forward kinematics for verification
        fk_pos, fk_orientation = self.forward_kinematics(angles)
        pos_error = math.sqrt((fk_pos[0]-x)**2 + (fk_pos[1]-y)**2 + (fk_pos[2]-z)**2)
        orient_error = abs(math.degrees(fk_orientation) - roll)
        
        # Get current parameters
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        results = f"""=== INVERSE KINEMATICS RESULTS ===
Target Position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}
Target Orientation: Roll={roll:.1f}°

Joint Angles (degrees):
  θ1 (Base):    {math.degrees(theta1):.2f}°
  θ2 (Shoulder): {math.degrees(theta2):.2f}°
  θ3 (Elbow):    {math.degrees(theta3):.2f}°
  θ4 (Wrist):    {math.degrees(theta4):.2f}°

Forward Kinematics Verification:
  Position: X={fk_pos[0]:.2f}, Y={fk_pos[1]:.2f}, Z={fk_pos[2]:.2f}
  Orientation: Roll={math.degrees(fk_orientation):.2f}°
  
Accuracy:
  Position Error: {pos_error:.4f} cm
  Orientation Error: {orient_error:.4f}°

Robot Parameters:
  L1={U1:.1f}, L2={U2:.1f}, L3={U3:.1f}, Base={base_height:.1f}"""
        
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, results)
    
    def forward_kinematics(self, angles):
        """Calculate end effector position and orientation from joint angles"""
        theta1, theta2, theta3, theta4 = angles
        
        # Get current parameters
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        # Position calculation
        x = (U1 * math.cos(theta2) + 
             U2 * math.cos(theta2 + theta3) + 
             U3 * math.cos(theta2 + theta3 + theta4)) * math.cos(theta1)
        
        y = (U1 * math.cos(theta2) + 
             U2 * math.cos(theta2 + theta3) + 
             U3 * math.cos(theta2 + theta3 + theta4)) * math.sin(theta1)
        
        z = (U1 * math.sin(theta2) + 
             U2 * math.sin(theta2 + theta3) + 
             U3 * math.sin(theta2 + theta3 + theta4)) + base_height
        
        # Orientation calculation (end effector roll)
        orientation = theta2 + theta3 + theta4
        
        return [x, y, z], orientation
    
    def calculate_joint_positions(self, angles):
        """Calculate positions of all joints for visualization"""
        theta1, theta2, theta3, theta4 = angles
        
        # Get current parameters
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        positions = []
        
        # Base position
        positions.append([0, 0, 0])
        positions.append([0, 0, base_height])
        
        # Joint 2 position (shoulder)
        x2 = U1 * math.cos(theta1) * math.cos(theta2)
        y2 = U1 * math.sin(theta1) * math.cos(theta2)
        z2 = base_height + U1 * math.sin(theta2)
        positions.append([x2, y2, z2])
        
        # Joint 3 position (elbow)
        x3 = x2 + U2 * math.cos(theta1) * math.cos(theta2 + theta3)
        y3 = y2 + U2 * math.sin(theta1) * math.cos(theta2 + theta3)
        z3 = z2 + U2 * math.sin(theta2 + theta3)
        positions.append([x3, y3, z3])
        
        # End effector position
        x4 = x3 + U3 * math.cos(theta1) * math.cos(theta2 + theta3 + theta4)
        y4 = y3 + U3 * math.sin(theta1) * math.cos(theta2 + theta3 + theta4)
        z4 = z3 + U3 * math.sin(theta2 + theta3 + theta4)
        positions.append([x4, y4, z4])
        
        return np.array(positions)
    
    def plot_robot(self):
        """Plot the robot arm in 3D"""
        self.ax.clear()
        
        positions = self.calculate_joint_positions(self.angles)
        
        # Plot robot arm
        self.ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                     'o-', linewidth=4, markersize=8, color='blue', label='Robot Arm')
        
        # Plot target position (only for IK)
        if hasattr(self, 'target_x'):
            target_pos = [self.target_x.get(), self.target_y.get(), self.target_z.get()]
            self.ax.scatter(*target_pos, color='red', s=100, label='Target Position')
        
        # Plot end effector orientation
        self.plot_end_effector_orientation(positions[-1], self.angles[-1])
        
        # Plot workspace boundary
        self.plot_workspace()
        
        # Settings
        self.ax.set_xlabel('X (cm)', fontsize=10)
        self.ax.set_ylabel('Y (cm)', fontsize=10)
        self.ax.set_zlabel('Z (cm)', fontsize=10)
        self.ax.set_title('4 DOF Robot Arm Simulation', fontsize=12, fontweight='bold')
        
        # Get current parameters for workspace calculation
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        # Set equal aspect ratio dengan workspace yang dinamis
        max_range = max(U1 + U2 + U3, 60)
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([0, max_range])
        
        self.ax.legend(fontsize=9)
        self.canvas.draw()
    
    def plot_end_effector_orientation(self, end_effector_pos, theta4):
        """Plot end effector orientation arrow"""
        # Calculate orientation vector
        arrow_length = 8
        dx = arrow_length * math.cos(theta4) * math.cos(self.angles[0])
        dy = arrow_length * math.cos(theta4) * math.sin(self.angles[0])
        dz = arrow_length * math.sin(theta4)
        
        self.ax.quiver(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2],
                       dx, dy, dz, color='green', linewidth=2, arrow_length_ratio=0.2,
                       label='End Effector Orientation')
    
    def plot_workspace(self):
        """Plot approximate workspace boundary"""
        U1 = self.U1.get()
        U2 = self.U2.get()
        U3 = self.U3.get()
        base_height = self.base_height.get()
        
        radius = U1 + U2 + U3
        height = base_height + U1 + U2 + U3
        
        # Draw workspace boundary
        theta = np.linspace(0, 2*np.pi, 36)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z_bottom = np.full_like(x, 0)
        z_top = np.full_like(x, height)
        
        self.ax.plot(x, y, z_bottom, 'g--', alpha=0.3, label='Workspace')
        self.ax.plot(x, y, z_top, 'g--', alpha=0.3)
    
    # Mouse interaction functions for zoom and pan
    def on_scroll(self, event):
        """Handle mouse scroll for zooming"""
        scale_factor = 1.1 if event.button == 'up' else 0.9
        
        # Get current limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()
        
        # Calculate new limits
        new_xlim = [(x - np.mean(xlim)) * scale_factor + np.mean(xlim) for x in xlim]
        new_ylim = [(y - np.mean(ylim)) * scale_factor + np.mean(ylim) for y in ylim]
        new_zlim = [(z - np.mean(zlim)) * scale_factor + np.mean(zlim) for z in zlim]
        
        # Set new limits
        self.ax.set_xlim(new_xlim)
        self.ax.set_ylim(new_ylim)
        self.ax.set_zlim(new_zlim)
        
        self.canvas.draw()
    
    def on_button_press(self, event):
        """Handle mouse button press for panning"""
        if event.button == 1:  # Left mouse button
            self._drag_start = (event.x, event.y)
    
    def on_button_release(self, event):
        """Handle mouse button release"""
        self._drag_start = None
    
    def on_motion(self, event):
        """Handle mouse motion for panning"""
        if self._drag_start is None or event.button != 1:
            return
        
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        
        # Adjust view angle based on drag
        elev = self.ax.elev - dy * 0.5
        azim = self.ax.azim - dx * 0.5
        
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw()
        
        self._drag_start = (event.x, event.y)

# Run the GUI application
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArm4DOF_GUI(root)
    root.mainloop()