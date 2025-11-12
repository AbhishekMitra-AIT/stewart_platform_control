"""
6-DOF Motion Platform Control GUI - Demo Version

A demonstration GUI for controlling a Stewart Platform (6 degrees of freedom).
This is a refactored version suitable for portfolio/GitHub demonstration.

Features:
- Manual 6-DOF control (surge, sway, heave, roll, pitch, yaw)
- Real-time 3D visualization
- CSV motion profile execution
- System monitoring and status logging
- Emergency controls
- Maintenance and settings panels

Author: [Abhishek Mitra]
License: GNU GENERAL PUBLIC LICENSE Version 3
Repository: [https://github.com/AbhishekMitra-AIT/Demo_Desktop_App]
"""

import customtkinter as ctk
from tkinter import filedialog, messagebox
from PIL import Image
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import threading, numpy as np, pandas as pd
import socket, json, time, tkinter as tk
from hashlib import sha256
import configparser, os, webbrowser, platform, subprocess


# ============================================================================
# CONFIGURATION AND CONSTANTS
# ============================================================================

class PlatformConfig:
    """Configuration constants for the Stewart Platform"""
    
    # Motion limits
    ROLL_LIMIT = 21  # degrees
    PITCH_LIMIT = 20  # degrees
    YAW_LIMIT = 24  # degrees
    SURGE_LIMIT = 350  # mm
    SWAY_LIMIT = 300  # mm
    HEAVE_LIMIT = 240  # mm
    
    # Velocity limits
    MAX_LINEAR_VELOCITY = 700  # mm/s
    MAX_ANGULAR_VELOCITY = 35  # deg/s
    
    # Acceleration limits
    MAX_LINEAR_ACCELERATION = 6  # m/s²
    MAX_ANGULAR_ACCELERATION = 250  # deg/s²
    
    # Platform geometry
    BASE_RADIUS = 600  # mm
    PLATFORM_RADIUS = 400  # mm
    DEFAULT_HEIGHT = 1365.97  # mm
    
    # Network settings
    DEFAULT_IP = "192.168.1.1"
    DEFAULT_PORT = 502
    
    # Security
    ADMIN_PASSWORD_HASH = "5e884898da28047151d0e56f8dc6292773603d0d6aabbdd62a11ef721d1542d8"  # "password"


# ============================================================================
# MOCK HARDWARE INTERFACE
# ============================================================================

class MockHardwareInterface:
    """
    Mock hardware interface for demonstration purposes.
    In production, this would interface with actual hardware controllers.
    """
    
    def __init__(self):
        self.connected = False
        self.enabled = False
        self.emergency_active = False
        self.actuator_positions = [0.0] * 6
        
    def is_connected(self):
        """Simulate connection check"""
        try:
            # In production: socket.create_connection((ip, port), timeout=2)
            self.connected = True
            return True
        except:
            self.connected = False
            return False
    
    def enable_actuators(self):
        """Enable the actuators"""
        self.enabled = True
        return True
    
    def disable_actuators(self):
        """Disable the actuators"""
        self.enabled = False
        return True
    
    def move_actuators(self, positions, velocities, accelerations):
        """
        Simulate actuator movement
        
        Args:
            positions: List of 6 target positions (mm)
            velocities: List of 6 velocities (mm/s)
            accelerations: List of 6 accelerations (mm/s²)
        """
        if not self.enabled:
            raise Exception("Actuators not enabled")
        
        # Simulate movement
        self.actuator_positions = positions.copy()
        time.sleep(0.1)  # Simulate movement time
        
        return True
    
    def get_actuator_positions(self):
        """Get current actuator positions"""
        return self.actuator_positions.copy()
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_active = True
        self.enabled = False
        return True
    
    def reset_emergency(self):
        """Reset emergency state"""
        self.emergency_active = False
        return True


# ============================================================================
# KINEMATICS ENGINE
# ============================================================================

class StewartPlatformKinematics:
    """
    Simplified kinematics calculations for Stewart Platform.
    In production, this would contain full forward/inverse kinematics.
    """
    
    def __init__(self, config: PlatformConfig):
        self.config = config
        self.base_points = self._generate_hexagon_points(config.BASE_RADIUS)
        self.platform_points = self._generate_hexagon_points(config.PLATFORM_RADIUS)
    
    def _generate_hexagon_points(self, radius):
        """Generate hexagon attachment points"""
        angles = np.linspace(0, 2*np.pi, 7)[:-1]  # 6 points
        points = np.column_stack([
            radius * np.cos(angles),
            radius * np.sin(angles),
            np.zeros(6)
        ])
        return points
    
    def rotation_matrix(self, roll, pitch, yaw):
        """
        Calculate 3D rotation matrix from Euler angles
        
        Args:
            roll, pitch, yaw: Rotation angles in radians
        
        Returns:
            3x3 rotation matrix
        """
        # Roll (X-axis rotation)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Pitch (Y-axis rotation)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Yaw (Z-axis rotation)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        return Rz @ Ry @ Rx
    
    def inverse_kinematics(self, position, orientation):
        """
        Calculate actuator lengths for given position and orientation
        
        Args:
            position: [x, y, z] in mm
            orientation: [roll, pitch, yaw] in degrees
        
        Returns:
            List of 6 actuator lengths in mm
        """
        # Convert orientation to radians
        roll, pitch, yaw = np.radians(orientation)
        
        # Calculate rotation matrix
        R = self.rotation_matrix(roll, pitch, yaw)
        
        # Transform platform points
        transformed_points = position + (R @ self.platform_points.T).T
        
        # Calculate leg vectors and lengths
        leg_vectors = transformed_points - self.base_points
        leg_lengths = np.linalg.norm(leg_vectors, axis=1)
        
        return leg_lengths
    
    def forward_kinematics(self, leg_lengths):
        """
        Calculate platform pose from actuator lengths (simplified)
        
        Args:
            leg_lengths: List of 6 actuator lengths in mm
        
        Returns:
            Tuple of (position, orientation)
        """
        # Simplified - in production would use numerical methods
        # For demo, return approximate values
        position = np.array([0, 0, self.config.DEFAULT_HEIGHT])
        orientation = np.array([0, 0, 0])
        
        return position, orientation


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def log_action(message):
    """
    Log system actions to file and console
    
    Args:
        message: Message to log
    """
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    
    # Console output
    print(log_entry)
    
    # File logging
    log_dir = os.path.join(os.path.dirname(__file__), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    log_file = os.path.join(log_dir, f'system_log_{time.strftime("%Y%m%d")}.txt')
    
    try:
        with open(log_file, 'a') as f:
            f.write(log_entry + '\n')
    except Exception as e:
        print(f"Failed to write log: {e}")


def validate_motion_parameters(position, orientation, velocity, acceleration):
    """
    Validate motion parameters against safety limits
    
    Returns:
        Tuple of (is_valid, error_message)
    """
    config = PlatformConfig()
    
    # Check position limits
    if abs(position[0]) > config.SURGE_LIMIT:
        return False, f"Surge exceeds limit of ±{config.SURGE_LIMIT}mm"
    if abs(position[1]) > config.SWAY_LIMIT:
        return False, f"Sway exceeds limit of ±{config.SWAY_LIMIT}mm"
    if abs(position[2]) > config.HEAVE_LIMIT:
        return False, f"Heave exceeds limit of ±{config.HEAVE_LIMIT}mm"
    
    # Check orientation limits
    if abs(orientation[0]) > config.ROLL_LIMIT:
        return False, f"Roll exceeds limit of ±{config.ROLL_LIMIT}°"
    if abs(orientation[1]) > config.PITCH_LIMIT:
        return False, f"Pitch exceeds limit of ±{config.PITCH_LIMIT}°"
    if abs(orientation[2]) > config.YAW_LIMIT:
        return False, f"Yaw exceeds limit of ±{config.YAW_LIMIT}°"
    
    # Validate velocity-acceleration consistency
    for i, axis in enumerate(['x', 'y', 'z']):
        if velocity[i] > 0 and acceleration[i] <= 0:
            return False, f"Invalid: velocity_{axis} > 0 but acceleration_{axis} <= 0"
        if velocity[i] <= 0 and acceleration[i] > 0:
            return False, f"Invalid: velocity_{axis} <= 0 but acceleration_{axis} > 0"
    
    return True, ""


# ============================================================================
# MAIN APPLICATION
# ============================================================================

class MotionPlatformGUI:
    """Main GUI application for Stewart Platform control"""
    
    def __init__(self, root):
        self.root = root
        self.config = PlatformConfig()
        self.hardware = MockHardwareInterface()
        self.kinematics = StewartPlatformKinematics(self.config)
        
        # State variables
        self.current_position = [0, 0, 0]
        self.current_orientation = [0, 0, 0]
        self.emergency_active = False
        
        # Setup UI
        self.setup_window()
        self.create_menu()
        self.create_widgets()
        
        # Start monitoring thread
        self.monitoring_active = True
        self.start_monitoring()
    
    def setup_window(self):
        """Configure main window"""
        self.root.title("Stewart Platform Control - Demo Version")
        self.root.geometry("1440x1024")
        self.root.grid_columnconfigure((0, 1, 2), weight=1)
        self.root.rowconfigure((0, 1, 2, 3), weight=1)
        
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
    
    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        
        # System menu
        system_menu = tk.Menu(menubar, tearoff=0)
        system_menu.add_command(label="Settings", command=self.open_settings)
        system_menu.add_command(label="Maintenance", command=self.open_maintenance)
        menubar.add_cascade(label="System", menu=system_menu)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Documentation", command=self.open_docs)
        menubar.add_cascade(label="Help", menu=help_menu)
        
        self.root.config(menu=menubar)
    
    def create_widgets(self):
        """Create all GUI widgets"""
        self.create_header()
        self.create_manual_control_panel()
        self.create_preset_panel()
        self.create_csv_upload_panel()
        self.create_position_display_panel()
        self.create_3d_visualization_panel()
        self.create_status_panel()
        self.create_control_buttons()
    
    def create_header(self):
        """Create header with logo and title"""
        header_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        header_frame.grid(row=0, column=0, columnspan=3, pady=10, sticky="ew")
        
        title_label = ctk.CTkLabel(
            header_frame,
            text="6-DOF Stewart Platform Control",
            font=ctk.CTkFont(size=28, weight="bold")
        )
        title_label.pack(side="left", expand=True)
    
    def create_manual_control_panel(self):
        """Create manual control sliders"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        
        ctk.CTkLabel(
            frame,
            text="Manual Control",
            font=ctk.CTkFont(size=20, weight="bold")
        ).grid(row=0, column=0, columnspan=4, pady=10)
        
        # Create sliders for each DOF
        self.sliders = {}
        self.entries = {}
        
        controls = [
            ("Roll", "roll", self.config.ROLL_LIMIT),
            ("Pitch", "pitch", self.config.PITCH_LIMIT),
            ("Yaw", "yaw", self.config.YAW_LIMIT),
            ("Surge (X)", "surge", self.config.SURGE_LIMIT),
            ("Sway (Y)", "sway", self.config.SWAY_LIMIT),
            ("Heave (Z)", "heave", self.config.HEAVE_LIMIT),
        ]
        
        for idx, (label, key, limit) in enumerate(controls, start=1):
            self.create_slider_control(frame, idx, label, key, limit)
        
        # Update button
        ctk.CTkButton(
            frame,
            text="Update Position",
            command=self.update_position,
            fg_color="#e8b931",
            text_color="#2c2c2c",
            width=150,
            height=40
        ).grid(row=len(controls)+1, column=0, columnspan=2, pady=10)
        
        # Reset button
        ctk.CTkButton(
            frame,
            text="Reset",
            command=self.reset_controls,
            fg_color="#fff1c2",
            text_color="#2c2c2c",
            width=150,
            height=40
        ).grid(row=len(controls)+1, column=2, columnspan=2, pady=10)
    
    def create_slider_control(self, parent, row, label, key, limit):
        """Create a slider with entry box"""
        # Label
        ctk.CTkLabel(
            parent,
            text=f"{label} (±{limit})",
            font=ctk.CTkFont(size=14, weight="bold"),
            anchor="w"
        ).grid(row=row, column=0, padx=10, pady=5, sticky="w")
        
        # Entry
        entry = ctk.CTkEntry(parent, width=70)
        entry.grid(row=row, column=1, padx=10, pady=5)
        entry.insert(0, "0.00")
        self.entries[key] = entry
        
        # Slider
        slider = ctk.CTkSlider(parent, from_=-limit, to=limit, width=200)
        slider.grid(row=row, column=2, padx=10, pady=5)
        slider.set(0)
        self.sliders[key] = slider
        
        # Bind events
        slider.configure(command=lambda v: self.sync_slider_to_entry(key, v))
        entry.bind("<Return>", lambda e: self.sync_entry_to_slider(key))
    
    def sync_slider_to_entry(self, key, value):
        """Sync slider value to entry box"""
        self.entries[key].delete(0, "end")
        self.entries[key].insert(0, f"{float(value):.2f}")
    
    def sync_entry_to_slider(self, key):
        """Sync entry box to slider"""
        try:
            value = float(self.entries[key].get())
            self.sliders[key].set(value)
        except ValueError:
            self.entries[key].delete(0, "end")
            self.entries[key].insert(0, f"{self.sliders[key].get():.2f}")
    
    def create_preset_panel(self):
        """Create preset buttons panel"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")
        
        ctk.CTkLabel(
            frame,
            text="Presets",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=10)
        
        presets = [
            ("Home", self.move_to_home),
            ("Base", self.move_to_base),
            ("Reset Emergency", self.reset_emergency),
        ]
        
        for text, command in presets:
            ctk.CTkButton(
                frame,
                text=text,
                command=command,
                font=ctk.CTkFont(size=16, weight="bold"),
                width=200,
                height=50
            ).pack(pady=10)
    
    def create_csv_upload_panel(self):
        """Create CSV file upload panel"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")
        
        ctk.CTkLabel(
            frame,
            text="CSV Motion Profile",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=10)
        
        self.file_path_entry = ctk.CTkEntry(frame, width=300)
        self.file_path_entry.pack(pady=5)
        
        ctk.CTkButton(
            frame,
            text="Browse",
            command=self.browse_csv_file,
            width=150
        ).pack(pady=5)
        
        ctk.CTkButton(
            frame,
            text="Execute CSV",
            command=self.execute_csv,
            fg_color="#27ae60",
            text_color="#2c2c2c",
            width=150
        ).pack(pady=5)
    
    def create_position_display_panel(self):
        """Create current position display"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        
        ctk.CTkLabel(
            frame,
            text="Current Position",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=10)
        
        # Actuator positions
        actuator_frame = ctk.CTkFrame(frame)
        actuator_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(
            actuator_frame,
            text="Actuator Lengths (mm)",
            font=ctk.CTkFont(size=14, weight="bold")
        ).pack(pady=5)
        
        self.actuator_labels = []
        actuator_grid = ctk.CTkFrame(actuator_frame, fg_color="transparent")
        actuator_grid.pack()
        
        for i in range(6):
            ctk.CTkLabel(
                actuator_grid,
                text=f"A{i+1}:",
                width=40
            ).grid(row=0, column=i, padx=5)
            
            label = ctk.CTkLabel(actuator_grid, text="0.000", width=60)
            label.grid(row=1, column=i, padx=5)
            self.actuator_labels.append(label)
        
        # Position values
        position_frame = ctk.CTkFrame(frame)
        position_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(
            position_frame,
            text="Platform Pose",
            font=ctk.CTkFont(size=14, weight="bold")
        ).pack(pady=5)
        
        self.position_labels = []
        position_grid = ctk.CTkFrame(position_frame, fg_color="transparent")
        position_grid.pack()
        
        pose_names = ["Roll", "Pitch", "Yaw", "Surge", "Sway", "Heave"]
        for i, name in enumerate(pose_names):
            ctk.CTkLabel(
                position_grid,
                text=f"{name}:",
                width=60
            ).grid(row=0, column=i, padx=5)
            
            label = ctk.CTkLabel(position_grid, text="0.000", width=60)
            label.grid(row=1, column=i, padx=5)
            self.position_labels.append(label)
    
    def create_3d_visualization_panel(self):
        """Create 3D platform visualization"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=1, column=2, rowspan=2, padx=10, pady=10, sticky="nsew")
        
        ctk.CTkLabel(
            frame,
            text="3D Visualization",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=10)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Initial plot
        self.update_3d_plot()
    
    def create_status_panel(self):
        """Create status message panel"""
        frame = ctk.CTkFrame(self.root, corner_radius=10)
        frame.grid(row=3, column=0, columnspan=3, padx=10, pady=5, sticky="ew")
        
        ctk.CTkLabel(
            frame,
            text="System Status",
            font=ctk.CTkFont(size=16, weight="bold")
        ).pack(side="left", padx=10)
        
        self.status_text = ctk.CTkTextbox(frame, height=60)
        self.status_text.pack(side="left", fill="x", expand=True, padx=10, pady=5)
        self.update_status("System initialized. Ready for operation.")
    
    def create_control_buttons(self):
        """Create main control buttons"""
        frame = ctk.CTkFrame(self.root, fg_color="transparent")
        frame.grid(row=4, column=0, columnspan=3, pady=10)
        
        ctk.CTkButton(
            frame,
            text="ENABLE",
            command=self.enable_system,
            fg_color="#27ae60",
            text_color="#2c2c2c",
            font=ctk.CTkFont(size=18, weight="bold"),
            width=150,
            height=50
        ).pack(side="left", padx=10)
        
        ctk.CTkButton(
            frame,
            text="STOP",
            command=self.emergency_stop,
            fg_color="#e74c3c",
            text_color="#2c2c2c",
            font=ctk.CTkFont(size=18, weight="bold"),
            width=150,
            height=50
        ).pack(side="left", padx=10)
        
        ctk.CTkButton(
            frame,
            text="EXIT",
            command=self.exit_application,
            fg_color="#95a5a6",
            text_color="#2c2c2c",
            font=ctk.CTkFont(size=18, weight="bold"),
            width=150,
            height=50
        ).pack(side="right", padx=10)
    
    # ========================================================================
    # CONTROL METHODS
    # ========================================================================
    
    def update_position(self):
        """Update platform position based on slider values"""
        def run_update():
            try:
                # Get values from sliders
                orientation = [
                    float(self.sliders['roll'].get()),
                    float(self.sliders['pitch'].get()),
                    float(self.sliders['yaw'].get())
                ]
                
                position = [
                    float(self.sliders['surge'].get()),
                    float(self.sliders['sway'].get()),
                    float(self.sliders['heave'].get())
                ]
                
                # Validate parameters
                is_valid, error = validate_motion_parameters(
                    position, orientation, [0]*3, [0]*3
                )
                
                if not is_valid:
                    self.update_status(f"Error: {error}")
                    return
                
                # Calculate kinematics
                leg_lengths = self.kinematics.inverse_kinematics(
                    np.array(position),
                    np.array(orientation)
                )
                
                # Simulate hardware movement
                if self.hardware.is_connected():
                    self.hardware.move_actuators(
                        leg_lengths,
                        [50]*6,  # Default velocities
                        [10]*6   # Default accelerations
                    )
                    
                    self.current_position = position
                    self.current_orientation = orientation
                    
                    self.update_status("Position updated successfully")
                    self.update_displays()
                    self.update_3d_plot()
                else:
                    self.update_status("Warning: Hardware not connected (demo mode)")
                    # Still update visualization in demo mode
                    self.current_position = position
                    self.current_orientation = orientation
                    self.update_displays()
                    self.update_3d_plot()
                
                log_action(f"Position updated: {position}, {orientation}")
                
            except Exception as e:
                self.update_status(f"Error: {str(e)}")
                log_action(f"Error in update_position: {str(e)}")
        
        threading.Thread(target=run_update, daemon=True).start()
    
    def reset_controls(self):
        """Reset all controls to zero"""
        for slider in self.sliders.values():
            slider.set(0)
        for entry in self.entries.values():
            entry.delete(0, "end")
            entry.insert(0, "0.00")
        
        self.update_status("Controls reset to zero")
        log_action("Controls reset")
    
    def move_to_home(self):
        """Move platform to home position"""
        def run_home():
            try:
                self.update_status("Moving to home position...")
                
                # Home is at default height with zero orientation
                home_position = [0, 0, 0]
                home_orientation = [0, 0, 0]
                
                # Calculate kinematics
                leg_lengths = self.kinematics.inverse_kinematics(
                    np.array(home_position),
                    np.array(home_orientation)
                )
                
                if self.hardware.is_connected():
                    self.hardware.move_actuators(leg_lengths, [50]*6, [10]*6)
                
                self.current_position = home_position
                self.current_orientation = home_orientation
                
                # Update UI
                for key in self.sliders:
                    self.sliders[key].set(0)
                    self.entries[key].delete(0, "end")
                    self.entries[key].insert(0, "0.00")
                
                self.update_displays()
                self.update_3d_plot()
                self.update_status("Home position reached")
                log_action("Moved to home position")
                
            except Exception as e:
                self.update_status(f"Error: {str(e)}")
                log_action(f"Error in move_to_home: {str(e)}")
        
        threading.Thread(target=run_home, daemon=True).start()
    
    def move_to_base(self):
        """Move platform to base position (fully retracted)"""
        def run_base():
            try:
                self.update_status("Moving to base position...")
                
                if self.hardware.is_connected():
                    self.hardware.move_actuators([0]*6, [50]*6, [10]*6)
                
                self.current_position = [0, 0, -self.config.DEFAULT_HEIGHT]
                self.current_orientation = [0, 0, 0]
                
                self.update_displays()
                self.update_status("Base position reached")
                log_action("Moved to base position")
                
            except Exception as e:
                self.update_status(f"Error: {str(e)}")
                log_action(f"Error in move_to_base: {str(e)}")
        
        threading.Thread(target=run_base, daemon=True).start()
    
    def browse_csv_file(self):
        """Open file dialog to select CSV file"""
        file_path = filedialog.askopenfilename(
            title="Select Motion Profile CSV",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        
        if file_path:
            self.file_path_entry.delete(0, "end")
            self.file_path_entry.insert(0, file_path)
            log_action(f"CSV file selected: {file_path}")
    
    def execute_csv(self):
        """Execute motion profile from CSV file"""
        def run_csv():
            try:
                csv_path = self.file_path_entry.get()
                
                if not csv_path:
                    self.update_status("Error: No file selected")
                    return
                
                # Load and validate CSV
                df = pd.read_csv(csv_path)
                
                required_columns = [
                    "pitch", "yaw", "roll", "heave", "sway", "surge",
                    "velocity_x", "velocity_y", "velocity_z"
                ]
                
                missing = [col for col in required_columns if col not in df.columns]
                if missing:
                    self.update_status(f"Error: Missing columns: {missing}")
                    return
                
                self.update_status(f"Executing CSV with {len(df)} waypoints...")
                log_action(f"Starting CSV execution: {csv_path}")
                
                # Execute each waypoint
                for idx, row in df.iterrows():
                    if self.emergency_active:
                        self.update_status("CSV execution aborted: Emergency active")
                        break
                    
                    position = [row['surge'], row['sway'], row['heave']]
                    orientation = [row['roll'], row['pitch'], row['yaw']]
                    
                    # Calculate and move
                    leg_lengths = self.kinematics.inverse_kinematics(
                        np.array(position),
                        np.array(orientation)
                    )
                    
                    if self.hardware.is_connected():
                        self.hardware.move_actuators(leg_lengths, [50]*6, [10]*6)
                    
                    self.current_position = position
                    self.current_orientation = orientation
                    
                    self.update_displays()
                    self.update_status(f"Executing waypoint {idx+1}/{len(df)}")
                    
                    time.sleep(0.1)  # Brief pause between waypoints
                
                self.update_status("CSV execution completed")
                log_action("CSV execution completed")
                
            except Exception as e:
                self.update_status(f"Error: {str(e)}")
                log_action(f"Error in execute_csv: {str(e)}")
        
        threading.Thread(target=run_csv, daemon=True).start()
    
    def enable_system(self):
        """Enable the motion platform"""
        try:
            if self.hardware.enable_actuators():
                self.update_status("System enabled - Ready for operation")
                log_action("System enabled")
            else:
                self.update_status("Error: Failed to enable system")
        except Exception as e:
            self.update_status(f"Error: {str(e)}")
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        try:
            self.emergency_active = True
            self.hardware.emergency_stop()
            self.update_status("⚠️ EMERGENCY STOP ACTIVATED")
            log_action("EMERGENCY STOP")
        except Exception as e:
            self.update_status(f"Error: {str(e)}")
    
    def reset_emergency(self):
        """Reset emergency state"""
        try:
            self.hardware.reset_emergency()
            self.emergency_active = False
            self.update_status("Emergency reset - System ready")
            log_action("Emergency reset")
        except Exception as e:
            self.update_status(f"Error: {str(e)}")
    
    def exit_application(self):
        """Safely exit the application"""
        try:
            # Return to base position
            if self.hardware.is_connected() and self.hardware.enabled:
                self.update_status("Returning to base position before exit...")
                self.hardware.move_actuators([0]*6, [50]*6, [10]*6)
                time.sleep(1)
            
            # Disable hardware
            self.hardware.disable_actuators()
            
            log_action("Application exiting")
            self.monitoring_active = False
            self.root.quit()
            
        except Exception as e:
            log_action(f"Error during exit: {str(e)}")
            self.root.quit()
    
    # ========================================================================
    # UI UPDATE METHODS
    # ========================================================================
    
    def update_status(self, message):
        """Update status text box"""
        timestamp = time.strftime("%H:%M:%S")
        status_message = f"[{timestamp}] {message}"
        
        current_text = self.status_text.get("1.0", "end").strip()
        new_text = f"{status_message}\n{current_text}" if current_text else status_message
        
        # Keep only last 10 messages
        lines = new_text.split('\n')[:10]
        
        self.status_text.delete("1.0", "end")
        self.status_text.insert("1.0", '\n'.join(lines))
    
    def update_displays(self):
        """Update all position displays"""
        # Update actuator position labels
        leg_lengths = self.kinematics.inverse_kinematics(
            np.array(self.current_position),
            np.array(self.current_orientation)
        )
        
        for i, label in enumerate(self.actuator_labels):
            label.configure(text=f"{leg_lengths[i]:.2f}")
        
        # Update pose labels
        pose_values = self.current_orientation + self.current_position
        for i, label in enumerate(self.position_labels):
            label.configure(text=f"{pose_values[i]:.2f}")
    
    def update_3d_plot(self):
        """Update 3D visualization of platform"""
        try:
            self.ax.clear()
            
            # Get current pose
            position = np.array(self.current_position)
            orientation_rad = np.radians(self.current_orientation)
            
            # Calculate rotation matrix
            R = self.kinematics.rotation_matrix(*orientation_rad)
            
            # Transform platform points
            transformed_points = position + (R @ self.kinematics.platform_points.T).T
            
            # Plot base platform (hexagon)
            base_points = np.vstack([self.kinematics.base_points, 
                                    self.kinematics.base_points[0]])
            self.ax.plot(base_points[:, 0], base_points[:, 1], 
                        base_points[:, 2], 'r-', linewidth=2, label='Base')
            self.ax.scatter(self.kinematics.base_points[:, 0], 
                           self.kinematics.base_points[:, 1],
                           self.kinematics.base_points[:, 2], 
                           c='red', s=50, marker='o')
            
            # Plot top platform (hexagon)
            top_points = np.vstack([transformed_points, transformed_points[0]])
            self.ax.plot(top_points[:, 0], top_points[:, 1], 
                        top_points[:, 2], 'b-', linewidth=2, label='Platform')
            self.ax.scatter(transformed_points[:, 0], 
                           transformed_points[:, 1],
                           transformed_points[:, 2], 
                           c='blue', s=50, marker='o')
            
            # Draw actuator legs
            for i in range(6):
                self.ax.plot(
                    [self.kinematics.base_points[i, 0], transformed_points[i, 0]],
                    [self.kinematics.base_points[i, 1], transformed_points[i, 1]],
                    [self.kinematics.base_points[i, 2], transformed_points[i, 2]],
                    'gray', linewidth=1.5
                )
            
            # Set axis properties
            limit = max(self.config.BASE_RADIUS, self.config.PLATFORM_RADIUS) * 1.5
            self.ax.set_xlim([-limit, limit])
            self.ax.set_ylim([-limit, limit])
            self.ax.set_zlim([0, limit * 2])
            
            self.ax.set_xlabel("X (mm)")
            self.ax.set_ylabel("Y (mm)")
            self.ax.set_zlabel("Z (mm)")
            self.ax.set_title("Stewart Platform")
            self.ax.legend()
            
            self.canvas.draw()
            
        except Exception as e:
            log_action(f"Error updating 3D plot: {str(e)}")
    
    # ========================================================================
    # MONITORING AND BACKGROUND TASKS
    # ========================================================================
    
    def start_monitoring(self):
        """Start background monitoring thread"""
        def monitor():
            while self.monitoring_active:
                try:
                    # Check hardware status
                    if not self.hardware.is_connected():
                        # Connection lost
                        pass
                    
                    # Check for emergency conditions
                    if self.hardware.emergency_active and not self.emergency_active:
                        self.emergency_active = True
                        self.update_status("⚠️ Emergency condition detected!")
                    
                    time.sleep(0.5)
                    
                except Exception as e:
                    log_action(f"Monitoring error: {str(e)}")
                    time.sleep(1)
        
        threading.Thread(target=monitor, daemon=True).start()
    
    # ========================================================================
    # DIALOG WINDOWS
    # ========================================================================
    
    def open_settings(self):
        """Open settings dialog with authentication"""
        # Authenticate first
        if not self.authenticate_user("Settings Access"):
            return
        
        settings_window = SettingsDialog(self.root)
        log_action("Settings window opened")
    
    def open_maintenance(self):
        """Open maintenance dialog with authentication"""
        if not self.authenticate_user("Maintenance Access"):
            return
        
        maintenance_window = MaintenanceDialog(self.root)
        log_action("Maintenance window opened")
    
    def authenticate_user(self, title):
        """Show authentication dialog"""
        dialog = ctk.CTkInputDialog(
            text=f"{title}\nEnter password:",
            title="Authentication Required"
        )
        
        password = dialog.get_input()
        
        if password is None:
            return False
        
        # Hash and verify
        hashed = sha256(password.encode()).hexdigest()
        
        if hashed == self.config.ADMIN_PASSWORD_HASH:
            log_action(f"Successful authentication for {title}")
            return True
        else:
            messagebox.showerror("Error", "Invalid password")
            log_action(f"Failed authentication attempt for {title}")
            return False
    
    def show_about(self):
        """Show about dialog"""
        about_text = """
Stewart Platform Control System
Demo Version 1.0

A 6-DOF motion platform control system demonstrating:
• Real-time kinematics calculations
• 3D visualization
• CSV motion profile execution
• Safety monitoring

This is a demonstration version for portfolio purposes.

Author: [Your Name]
GitHub: [Your Repository URL]
License: MIT
"""
        messagebox.showinfo("About", about_text)
    
    def open_docs(self):
        """Open documentation in browser"""
        docs_url = "https://github.com/[your-username]/stewart-platform"
        webbrowser.open(docs_url)
        log_action("Documentation opened")


# ============================================================================
# DIALOG CLASSES
# ============================================================================

class SettingsDialog(ctk.CTkToplevel):
    """Settings configuration dialog"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.title("System Settings")
        self.geometry("600x400")
        
        # Load settings
        self.config = self.load_config()
        
        # Create UI
        self.create_widgets()
    
    def load_config(self):
        """Load configuration from file"""
        config = configparser.ConfigParser()
        config_path = os.path.join(os.path.dirname(__file__), 'config.ini')
        
        # Create default config if doesn't exist
        if not os.path.exists(config_path):
            config['Platform'] = {
                'base_radius': '600',
                'platform_radius': '400',
                'default_height': '1365.97'
            }
            config['Network'] = {
                'ip_address': '192.168.1.1',
                'port': '502'
            }
            config['Safety'] = {
                'max_velocity': '700',
                'max_acceleration': '6'
            }
            
            with open(config_path, 'w') as f:
                config.write(f)
        else:
            config.read(config_path)
        
        return config
    
    def create_widgets(self):
        """Create settings UI"""
        ctk.CTkLabel(
            self,
            text="System Settings",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=20)
        
        # Settings notebook
        notebook = ctk.CTkTabview(self)
        notebook.pack(fill="both", expand=True, padx=20, pady=10)
        
        # Platform tab
        platform_tab = notebook.add("Platform")
        self.create_platform_settings(platform_tab)
        
        # Network tab
        network_tab = notebook.add("Network")
        self.create_network_settings(network_tab)
        
        # Safety tab
        safety_tab = notebook.add("Safety")
        self.create_safety_settings(safety_tab)
        
        # Save button
        ctk.CTkButton(
            self,
            text="Save Settings",
            command=self.save_settings,
            fg_color="#27ae60"
        ).pack(pady=10)
    
    def create_platform_settings(self, parent):
        """Create platform configuration settings"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        settings = [
            ("Base Radius (mm)", "Platform", "base_radius"),
            ("Platform Radius (mm)", "Platform", "platform_radius"),
            ("Default Height (mm)", "Platform", "default_height")
        ]
        
        self.platform_entries = {}
        
        for label, section, key in settings:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", pady=5)
            
            ctk.CTkLabel(row, text=label, width=200, anchor="w").pack(side="left", padx=5)
            
            entry = ctk.CTkEntry(row, width=150)
            entry.pack(side="left", padx=5)
            entry.insert(0, self.config.get(section, key))
            
            self.platform_entries[key] = (section, entry)
    
    def create_network_settings(self, parent):
        """Create network configuration settings"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        settings = [
            ("IP Address", "Network", "ip_address"),
            ("Port", "Network", "port")
        ]
        
        self.network_entries = {}
        
        for label, section, key in settings:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", pady=5)
            
            ctk.CTkLabel(row, text=label, width=200, anchor="w").pack(side="left", padx=5)
            
            entry = ctk.CTkEntry(row, width=150)
            entry.pack(side="left", padx=5)
            entry.insert(0, self.config.get(section, key))
            
            self.network_entries[key] = (section, entry)
    
    def create_safety_settings(self, parent):
        """Create safety limit settings"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        settings = [
            ("Max Velocity (mm/s)", "Safety", "max_velocity"),
            ("Max Acceleration (m/s²)", "Safety", "max_acceleration")
        ]
        
        self.safety_entries = {}
        
        for label, section, key in settings:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", pady=5)
            
            ctk.CTkLabel(row, text=label, width=200, anchor="w").pack(side="left", padx=5)
            
            entry = ctk.CTkEntry(row, width=150)
            entry.pack(side="left", padx=5)
            entry.insert(0, self.config.get(section, key))
            
            self.safety_entries[key] = (section, entry)
    
    def save_settings(self):
        """Save settings to config file"""
        try:
            # Update config with all entries
            for entries_dict in [self.platform_entries, self.network_entries, self.safety_entries]:
                for key, (section, entry) in entries_dict.items():
                    self.config.set(section, key, entry.get())
            
            # Save to file
            config_path = os.path.join(os.path.dirname(__file__), 'config.ini')
            with open(config_path, 'w') as f:
                self.config.write(f)
            
            messagebox.showinfo("Success", "Settings saved successfully")
            log_action("Settings saved")
            self.destroy()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save settings: {str(e)}")
            log_action(f"Error saving settings: {str(e)}")


class MaintenanceDialog(ctk.CTkToplevel):
    """System maintenance and diagnostics dialog"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.title("System Maintenance")
        self.geometry("800x600")
        
        self.create_widgets()
    
    def create_widgets(self):
        """Create maintenance UI"""
        ctk.CTkLabel(
            self,
            text="System Maintenance",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=20)
        
        # Maintenance notebook
        notebook = ctk.CTkTabview(self)
        notebook.pack(fill="both", expand=True, padx=20, pady=10)
        
        # Diagnostics tab
        diag_tab = notebook.add("Diagnostics")
        self.create_diagnostics_tab(diag_tab)
        
        # Logs tab
        logs_tab = notebook.add("Logs")
        self.create_logs_tab(logs_tab)
        
        # Calibration tab
        calib_tab = notebook.add("Calibration")
        self.create_calibration_tab(calib_tab)
    
    def create_diagnostics_tab(self, parent):
        """Create system diagnostics display"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(
            frame,
            text="System Diagnostics",
            font=ctk.CTkFont(size=16, weight="bold")
        ).pack(pady=10)
        
        # Status indicators
        status_items = [
            "Hardware Connection",
            "Actuator Status",
            "Emergency System",
            "Communication Link"
        ]
        
        for item in status_items:
            row = ctk.CTkFrame(frame, fg_color="transparent")
            row.pack(fill="x", pady=5)
            
            ctk.CTkLabel(row, text=item, width=200, anchor="w").pack(side="left", padx=10)
            
            status = ctk.CTkLabel(row, text="✓ OK", text_color="green")
            status.pack(side="left", padx=10)
        
        # Run diagnostics button
        ctk.CTkButton(
            frame,
            text="Run Full Diagnostics",
            command=self.run_diagnostics
        ).pack(pady=20)
    
    def create_logs_tab(self, parent):
        """Create logs viewer"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(
            frame,
            text="System Logs",
            font=ctk.CTkFont(size=16, weight="bold")
        ).pack(pady=10)
        
        # Log text box
        log_text = ctk.CTkTextbox(frame, height=400)
        log_text.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Load recent logs
        try:
            log_file = os.path.join(
                os.path.dirname(__file__),
                'logs',
                f'system_log_{time.strftime("%Y%m%d")}.txt'
            )
            
            if os.path.exists(log_file):
                with open(log_file, 'r') as f:
                    logs = f.readlines()[-50:]  # Last 50 lines
                    log_text.insert("1.0", ''.join(logs))
        except Exception as e:
            log_text.insert("1.0", f"Error loading logs: {str(e)}")
        
        # Open logs folder button
        ctk.CTkButton(
            frame,
            text="Open Logs Folder",
            command=self.open_logs_folder
        ).pack(pady=10)
    
    def create_calibration_tab(self, parent):
        """Create calibration tools"""
        frame = ctk.CTkFrame(parent)
        frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(
            frame,
            text="System Calibration",
            font=ctk.CTkFont(size=16, weight="bold")
        ).pack(pady=10)
        
        ctk.CTkLabel(
            frame,
            text="Calibration procedures should only be performed by trained personnel.",
            wraplength=500
        ).pack(pady=10)
        
        calibration_buttons = [
            ("Calibrate Home Position", self.calibrate_home),
            ("Calibrate Actuator Lengths", self.calibrate_actuators),
            ("Reset to Factory Defaults", self.factory_reset)
        ]
        
        for text, command in calibration_buttons:
            ctk.CTkButton(
                frame,
                text=text,
                command=command,
                width=300
            ).pack(pady=10)
    
    def run_diagnostics(self):
        """Run system diagnostics"""
        messagebox.showinfo("Diagnostics", "System diagnostics completed successfully")
        log_action("System diagnostics run")
    
    def open_logs_folder(self):
        """Open logs folder in file explorer"""
        logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        
        system = platform.system()
        try:
            if system == 'Windows':
                os.startfile(logs_dir)
            elif system == 'Darwin':  # macOS
                subprocess.run(['open', logs_dir])
            else:  # Linux
                subprocess.run(['xdg-open', logs_dir])
        except Exception as e:
            messagebox.showerror("Error", f"Could not open folder: {str(e)}")
    
    def calibrate_home(self):
        """Calibrate home position"""
        if messagebox.askyesno("Confirm", "Start home position calibration?"):
            messagebox.showinfo("Calibration", "Home position calibration completed")
            log_action("Home position calibrated")
    
    def calibrate_actuators(self):
        """Calibrate actuator lengths"""
        if messagebox.askyesno("Confirm", "Start actuator calibration?"):
            messagebox.showinfo("Calibration", "Actuator calibration completed")
            log_action("Actuators calibrated")
    
    def factory_reset(self):
        """Reset to factory defaults"""
        result = messagebox.askyesnocancel(
            "Warning",
            "This will reset all settings to factory defaults. Continue?"
        )
        if result:
            messagebox.showinfo("Reset", "System reset to factory defaults")
            log_action("Factory reset performed")


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main application entry point"""
    # Create logs directory
    os.makedirs(os.path.join(os.path.dirname(__file__), 'logs'), exist_ok=True)
    
    # Initialize application
    root = ctk.CTk()
    app = MotionPlatformGUI(root)
    
    log_action("Application started")
    
    # Handle window close
    def on_closing():
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            app.exit_application()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start main loop
    root.mainloop()


if __name__ == "__main__":
    main()

# ============================================================================