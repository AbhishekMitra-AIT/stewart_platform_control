# Stewart Platform Control System

A professional 6-DOF (Degrees of Freedom) motion platform control system with real-time kinematics, 3D visualization, and advanced motion profile execution.

![Python Version](https://img.shields.io/badge/python-3.8%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey)

## 🎯 Project Overview

This project implements a comprehensive GUI application for controlling a Stewart Platform (Hexapod), featuring inverse kinematics calculations, real-time 3D visualization, CSV-based motion profile execution, and safety monitoring systems. Originally developed for industrial motion simulation applications.

**Note:** This is a demonstration version with mock hardware interfaces suitable for portfolio and educational purposes.

## ✨ Features

### Core Functionality
- **6-DOF Manual Control**
  - Surge, Sway, Heave (Linear motion: X, Y, Z)
  - Roll, Pitch, Yaw (Rotational motion)
  - Real-time slider and numeric input
  - Position, velocity, and acceleration control

- **Inverse Kinematics Engine**
  - Real-time calculation of actuator lengths
  - Rotation matrix transformations
  - Configurable platform geometry
  - Safety limit validation

- **3D Real-Time Visualization**
  - Interactive Matplotlib 3D plot
  - Platform and base geometry display
  - Actuator leg visualization
  - Dynamic updates during motion

### Advanced Features
- **CSV Motion Profiles**
  - Load and execute complex motion sequences
  - Multi-axis coordinated motion
  - Waypoint-based trajectory planning
  - Progress monitoring

- **Safety Systems**
  - Emergency stop functionality
  - Motion limit validation
  - Hardware status monitoring
  - Error logging and recovery

- **System Management**
  - Configuration management (INI files)
  - Comprehensive logging system
  - Password-protected maintenance mode
  - System diagnostics and calibration tools

### User Interface
- Modern dark-themed GUI (CustomTkinter)
- Intuitive layout with multiple control panels
- Real-time status updates
- Position and actuator length displays

## 📁 Project Structure

```
stewart-platform-control/
│
├── src/
│   ├── __init__.py
│   ├── main.py                      # Application entry point
│   ├── gui/
│   │   ├── __init__.py
│   │   ├── main_window.py           # Main GUI application
│   │   ├── dialogs.py               # Settings & maintenance dialogs
│   │   └── widgets.py               # Custom UI components
│   │
│   ├── core/
│   │   ├── __init__.py
│   │   ├── kinematics.py            # Stewart platform kinematics
│   │   ├── hardware_interface.py   # Hardware abstraction layer
│   │   └── config.py                # Configuration management
│   │
│   └── utils/
│       ├── __init__.py
│       ├── logger.py                # Logging utilities
│       └── validators.py            # Input validation
│
├── config/
│   ├── config.ini                   # System configuration
│   └── platform_params.json         # Platform geometry parameters
│
├── data/
│   └── sample_motion_profile.csv    # Example CSV motion profile
│
├── logs/                            # Log files (auto-generated)
│
├── docs/
│   ├── API.md                       # API documentation
│   ├── USER_GUIDE.md                # User manual
│   └── images/                      # Screenshots and diagrams
│
├── tests/
│   ├── test_kinematics.py
│   ├── test_validators.py
│   └── test_hardware.py
│
├── requirements.txt                 # Python dependencies
├── setup.py                         # Package setup
├── .gitignore
├── LICENSE
└── README.md
```

## 🚀 Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager
- Git (for cloning)

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/stewart-platform-control.git
cd stewart-platform-control
```

### Step 2: Create Virtual Environment (Recommended)
```bash
# Windows
python -m venv venv
venv\Scripts\activate

# Linux/macOS
python3 -m venv venv
source venv/bin/activate
```

### Step 3: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 4: Run the Application
```bash
python src/main.py
```

## 📦 Dependencies

```
customtkinter>=5.2.0        # Modern GUI framework
matplotlib>=3.7.0           # 3D visualization
numpy>=1.24.0               # Numerical computations
pandas>=2.0.0               # CSV data handling
Pillow>=10.0.0              # Image processing
```

## 🎮 Usage

### Basic Operation

1. **Enable System**
   - Click the "ENABLE" button to activate the platform
   - Verify system status in the status panel

2. **Manual Control**
   - Use sliders or numeric inputs for each DOF
   - Click "Update Position" to execute motion
   - Monitor real-time 3D visualization

3. **Preset Positions**
   - "Home" - Return to neutral position
   - "Base" - Fully retract actuators
   - "Reset Emergency" - Clear emergency state

4. **CSV Motion Profiles**
   - Click "Browse" to select CSV file
   - Click "Execute CSV" to run motion sequence
   - Monitor progress in status panel

### CSV File Format

```csv
surge,sway,heave,roll,pitch,yaw,velocity_x,velocity_y,velocity_z
0,0,0,0,0,0,0,0,0
100,0,50,5,0,0,50,0,25
100,100,50,5,10,0,50,50,0
0,100,0,0,10,5,50,0,25
0,0,0,0,0,0,0,0,0
```

### Safety Limits

| DOF | Range | Unit |
|-----|-------|------|
| Roll | ±21 | degrees |
| Pitch | ±20 | degrees |
| Yaw | ±24 | degrees |
| Surge (X) | ±350 | mm |
| Sway (Y) | ±300 | mm |
| Heave (Z) | ±240 | mm |
| Linear Velocity | ±700 | mm/s |
| Angular Velocity | ±35 | deg/s |

## 🔧 Configuration

Edit `config/config.ini` to customize:

```ini
[Platform]
base_radius = 600
platform_radius = 400
default_height = 1365.97

[Network]
ip_address = 192.168.1.1
port = 502

[Safety]
max_velocity = 700
max_acceleration = 6
```

**Note:** Password-protected settings dialog available in GUI (default password: "password")

## 📸 Screenshots

### Main Control Interface
![Main Interface](docs/images/main_interface.png)
*Main control panel with 6-DOF sliders and 3D visualization*

### 3D Platform Visualization
![3D Visualization](docs/images/3d_visualization.png)
*Real-time Stewart platform geometry rendering*

### CSV Motion Profile Execution
![CSV Execution](docs/images/csv_execution.png)
*Automated motion sequence from CSV file*

### System Maintenance Panel
![Maintenance](docs/images/maintenance_panel.png)
*Advanced diagnostics and calibration tools*

## 🛠️ Technical Stack

### Frontend
- **CustomTkinter** - Modern, customizable tkinter-based GUI
- **Matplotlib** - 3D plotting and visualization
- **Pillow** - Image handling for UI assets

### Backend
- **NumPy** - Matrix operations and kinematics calculations
- **Pandas** - CSV data processing
- **Threading** - Asynchronous operations and monitoring

### Architecture
- **MVC Pattern** - Separation of GUI, logic, and data
- **Hardware Abstraction** - Mock interface for demo/testing
- **Event-Driven** - Responsive UI with background processing

### Key Algorithms
- **Inverse Kinematics** - Position/orientation to actuator lengths
- **Rotation Matrices** - 3D spatial transformations (Euler angles)
- **Trajectory Planning** - Waypoint interpolation and velocity profiling

## 🧪 Testing

Run unit tests:
```bash
python -m pytest tests/
```

Run specific test file:
```bash
python -m pytest tests/test_kinematics.py -v
```

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please ensure:
- Code follows PEP 8 style guidelines
- All tests pass
- Documentation is updated
- Commit messages are clear and descriptive

## 📄 License

This project is licensed under the GNU GENERAL PUBLIC LICENSE - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**Your Name**
- GitHub: [@AbhishekMitra-AIT](https://github.com/AbhishekMitra-AIT)
- LinkedIn: [Abhishek Mitra](https://www.linkedin.com/in/abhishekmitra03/)
- Email: abhishekmitra91@gmail.com

## 🙏 Acknowledgments

- Inspired by industrial motion simulation systems
- Stewart Platform kinematics based on parallel robot research
- GUI framework courtesy of CustomTkinter project

## 📚 Additional Resources -- to be added

- [User Guide](docs/USER_GUIDE.md) - Comprehensive usage instructions
- [API Documentation](docs/API.md) - Developer reference
- [Stewart Platform Theory](https://en.wikipedia.org/wiki/Stewart_platform) - Mathematical background

## 🐛 Known Issues

- 3D visualization may lag on systems with integrated graphics
- CSV execution requires specific column format (see sample file)
- Mock hardware interface for demonstration only

## 🔮 Future Enhancements

- [ ] Forward kinematics implementation
- [ ] Motion recording and playback
- [ ] Network control API (REST/WebSocket)
- [ ] VR/AR visualization integration
- [ ] Real-time force feedback simulation
- [ ] Multi-platform support optimization

## 📞 Support

For questions, issues, or suggestions:
- Open an issue on GitHub
- Email: abhishekmitra91@gmail.com
- Discussion forum: [Link to discussions]

---

**⭐ If you find this project useful, please consider giving it a star!**

Made with ❤️ and Python