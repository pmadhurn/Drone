# ESP32 Drone Flight Controller

A custom quadcopter flight controller built with ESP32, featuring WiFi control, altitude hold, and stabilization.
## Table of Contents
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Pin Connections](#-pin-connections)
- [Software Setup](#-software-setup)
- [Operation](#-operation)
- [Configuration](#-configuration)
- [Safety Warnings](#-safety-warnings)
- [Troubleshooting](#-troubleshooting)
- [Telemetry Data](#-telemetry-data)
- [Debug Mode](#-debug-mode)
- [License](#-license)
- [Contributing](#-contributing)
- [Performance Specs](#-performance-specs)

## 🚁 Features
- **6-axis stabilization** using MPU6050 IMU
- **Altitude hold** with 4x HC-SR04 ultrasonic sensors (45° mounted)
- **WiFi control** via Android app
- **PID-based flight control** for stable hovering
- **Battery monitoring** with low-voltage protection
- **Safety features** including emergency stop and failsafe
- **Real-time telemetry** over UDP

## 📋 Hardware Requirements
### Core Components
- **ESP32 DevKit** (30-pin version)
- **MPU6050** 6-axis IMU module
- **4x HC-SR04** ultrasonic sensors
- **4x ESCs** (Electronic Speed Controllers) - 30A recommended
- **4x Brushless Motors** (matched to your props and battery)
- **3S LiPo Battery** (11.1V nominal)
- **Power Distribution Board**

### Additional Components
- **470µF 25V** electrolytic capacitor
- **0.1µF** ceramic capacitor (marked 104)
- **10kΩ + 3.3kΩ** resistors for battery voltage divider
- Connecting wires and headers
- Frame (X-configuration)

## 🔌 Pin Connections
### I2C - MPU6050
| MPU6050 | ESP32 |
|---------|-------|
| VCC     | 3.3V  |
| GND     | GND   |
| SCL     | GPIO 22 |
| SDA     | GPIO 21 |
| AD0     | GND (I2C address 0x68) |

### Motors/ESCs
| Motor Position | GPIO Pin |
|---------------|----------|
| Front Right   | GPIO 25  |
| Front Left    | GPIO 26  |
| Rear Left     | GPIO 27  |
| Rear Right    | GPIO 14  |
### Ultrasonic Sensors (HC-SR04)
| Sensor Position | Trigger | Echo |
|----------------|---------|------|
| Front Right    | GPIO 32 | GPIO 33 |
| Front Left     | GPIO 18 | GPIO 19 |
| Rear Left      | GPIO 16 | GPIO 17 |
| Rear Right     | GPIO 4  | GPIO 5  |

### Battery Monitoring
Battery + → 10kΩ → GPIO 34 → 3.3kΩ → GND

### Power Filtering
- 470µF capacitor: Battery + to Battery -
- 0.1µF capacitor: Battery + to Battery - (near ESP32)

## 🛠️ Software Setup
### Prerequisites
- [PlatformIO](https://platformio.org/) with VSCode
- ESP-IDF framework (auto-installed by PlatformIO)
- Android Studio (for app compilation)
### Building and Flashing

1. **Clone the repository**
	```bash
	git clone <your-repo-url>
	cd drone-controller
	```
2. **Configure WiFi credentials (optional)**
	Edit `main/config.h` if you want to change default settings:
	```c
	#define WIFI_SSID "DroneAP"
	#define WIFI_PASS "drone1234"
	```
3. **Build the project**
	```bash
	pio run
	```
4. **Flash to ESP32**
	```bash
	pio run -t upload
	```
5. **Monitor output**
	```bash
	pio device monitor
	```

## 🎮 Operation

### First Time Setup
- Power on the drone (battery connected, props removed!)
- ESP32 creates WiFi AP: `DroneAP` (password: `drone1234`)
- Keep drone still for 5 seconds for gyro calibration
- Connect Android app to drone's WiFi
- Wait for "Connected" status in app

### Flight Sequence
#### Pre-flight checks
- Battery voltage > 11V
- All sensors responding
- Props secured properly
- Clear flight area

#### Arming
- Ensure throttle is at minimum
- Press ARM button in app
- Motors will spin at idle speed

#### Takeoff
- Gradually increase throttle
- Use altitude hold for stable hovering

#### Landing
- Reduce throttle slowly
- Disarm after landing

#### Emergency Procedures
- **EMERGENCY STOP:** Press red STOP button - all motors stop immediately
- **Connection Lost:** Drone will hold position, then slowly descend
- **Low Battery:** Auto-landing initiated < 10.5V
- **Extreme Angle:** Auto-disarm if pitch/roll > 45°

## ⚙️ Configuration

### PID Tuning
Edit values in `main/flight_controller.c`:
```c
static pid_controller_t pitch_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
static pid_controller_t roll_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
```

### Flight Parameters
In `main/config.h`:
```c
#define MAX_ANGLE 30.0f       // Maximum tilt angle
#define FLIGHT_LOOP_FREQ 250  // Hz
#define MIN_THROTTLE 1000     // ESC minimum pulse
#define MAX_THROTTLE 2000     // ESC maximum pulse
```

## 🚨 Safety Warnings

> ⚠️ **ALWAYS REMOVE PROPS DURING TESTING**
> ⚠️ **NEVER FLY NEAR PEOPLE OR PROPERTY**
> ⚠️ **CHECK BATTERY VOLTAGE BEFORE FLIGHT**
> ⚠️ **TEST IN OPEN AREAS ONLY**

## 🔧 Troubleshooting

### Motors not spinning
- Check ESC connections
- Verify battery voltage
- Ensure drone is armed
- Check ESC calibration

### Unstable flight
- Recalibrate gyroscope (power cycle while level)
- Check for loose components
- Reduce P gain in PID
- Check prop balance

### WiFi connection issues
- Ensure ESP32 is powered
- Check WiFi credentials
- Restart Android app
- Check distance from drone

### Sensor errors
- Check I2C connections (MPU6050)
- Verify ultrasonic sensor wiring
- Look for loose connections
- Monitor serial output for errors

## 📊 Telemetry Data

The drone sends telemetry at 10Hz including:
- Current altitude (cm)
- Pitch angle (degrees)
- Roll angle (degrees)
- Yaw angle (degrees)
- Battery voltage (V)

## 🔍 Debug Mode

Enable verbose logging in `platformio.ini`:
```ini
build_flags = -DCORE_DEBUG_LEVEL=5
```

## 📝 License

This project is open source. Use at your own risk. The authors are not responsible for any damages or injuries.

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch
3. Test thoroughly with props removed
4. Submit pull request

## ⚡ Performance Specs

- Control loop: 250Hz
- Telemetry rate: 10Hz
- WiFi latency: <20ms typical
- Max tilt angle: 30°
- Altitude hold accuracy: ±5cm

---

_Remember: Building and flying drones can be dangerous. Always prioritize safety!_
ESP32 Drone Controller - README.md
markdown
# ESP32 Drone Flight Controller

A custom quadcopter flight controller built with ESP32, featuring WiFi control, altitude hold, and stabilization.

## 🚁 Features

- **6-axis stabilization** using MPU6050 IMU
- **Altitude hold** with 4x HC-SR04 ultrasonic sensors (45° mounted)
- **WiFi control** via Android app
- **PID-based flight control** for stable hovering
- **Battery monitoring** with low-voltage protection
- **Safety features** including emergency stop and failsafe
- **Real-time telemetry** over UDP

## 📋 Hardware Requirements

### Core Components
- **ESP32 DevKit** (30-pin version)
- **MPU6050** 6-axis IMU module
- **4x HC-SR04** ultrasonic sensors
- **4x ESCs** (Electronic Speed Controllers) - 30A recommended
- **4x Brushless Motors** (matched to your props and battery)
- **3S LiPo Battery** (11.1V nominal)
- **Power Distribution Board**

### Additional Components
- **470µF 25V** electrolytic capacitor
- **0.1µF** ceramic capacitor (marked 104)
- **10kΩ + 3.3kΩ** resistors for battery voltage divider
- Connecting wires and headers
- Frame (X-configuration)

## 🔌 Pin Connections

### I2C - MPU6050
MPU6050 ESP32
VCC → 3.3V
GND → GND
SCL → GPIO 22
SDA → GPIO 21
AD0 → GND (I2C address 0x68)



### Motors/ESCs
Motor Position GPIO Pin
Front Right → GPIO 25
Front Left → GPIO 26
Rear Left → GPIO 27
Rear Right → GPIO 14


### Ultrasonic Sensors (HC-SR04)
Sensor Position Trigger Echo
Front Right → GPIO 32 GPIO 33
Front Left → GPIO 18 GPIO 19
Rear Left → GPIO 16 GPIO 17
Rear Right → GPIO 4 GPIO 5


### Battery Monitoring
Battery + → 10kΩ → GPIO 34 → 3.3kΩ → GND


### Power Filtering
470µF capacitor: Battery + to Battery -
0.1µF capacitor: Battery + to Battery - (near ESP32)

text

## 🛠️ Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) with VSCode
- ESP-IDF framework (auto-installed by PlatformIO)
- Android Studio (for app compilation)

### Building and Flashing

1. **Clone the repository**
```bash
git clone <your-repo-url>
cd drone-controller
Configure WiFi credentials (optional)
Edit main/config.h if you want to change default settings:
c
#define WIFI_SSID "DroneAP"
#define WIFI_PASS "drone1234"
Build the project
bash
pio run
Flash to ESP32
bash
pio run -t upload
Monitor output
bash
pio device monitor
🎮 Operation
First Time Setup
Power on the drone (battery connected, props removed!)
ESP32 creates WiFi AP: "DroneAP" (password: "drone1234")
Keep drone still for 5 seconds for gyro calibration
Connect Android app to drone's WiFi
Wait for "Connected" status in app
Flight Sequence
Pre-flight checks

Battery voltage > 11V
All sensors responding
Props secured properly
Clear flight area
Arming

Ensure throttle is at minimum
Press ARM button in app
Motors will spin at idle speed
Takeoff

Gradually increase throttle
Use altitude hold for stable hovering
Landing

Reduce throttle slowly
Disarm after landing
Emergency Procedures
EMERGENCY STOP: Press red STOP button - all motors stop immediately
Connection Lost: Drone will hold position, then slowly descend
Low Battery: Auto-landing initiated < 10.5V
Extreme Angle: Auto-disarm if pitch/roll > 45°
⚙️ Configuration
PID Tuning
Edit values in main/flight_controller.c:

c
static pid_controller_t pitch_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
static pid_controller_t roll_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
Flight Parameters
In main/config.h:

c
#define MAX_ANGLE 30.0f       // Maximum tilt angle
#define FLIGHT_LOOP_FREQ 250  // Hz
#define MIN_THROTTLE 1000     // ESC minimum pulse
#define MAX_THROTTLE 2000     // ESC maximum pulse
🚨 Safety Warnings
⚠️ ALWAYS REMOVE PROPS DURING TESTING
⚠️ NEVER FLY NEAR PEOPLE OR PROPERTY
⚠️ CHECK BATTERY VOLTAGE BEFORE FLIGHT
⚠️ TEST IN OPEN AREAS ONLY

🔧 Troubleshooting
Motors not spinning
Check ESC connections
Verify battery voltage
Ensure drone is armed
Check ESC calibration
Unstable flight
Recalibrate gyroscope (power cycle while level)
Check for loose components
Reduce P gain in PID
Check prop balance
WiFi connection issues
Ensure ESP32 is powered
Check WiFi credentials
Restart Android app
Check distance from drone
Sensor errors
Check I2C connections (MPU6050)
Verify ultrasonic sensor wiring
Look for loose connections
Monitor serial output for errors
📊 Telemetry Data
The drone sends telemetry at 10Hz including:

Current altitude (cm)
Pitch angle (degrees)
Roll angle (degrees)
Yaw angle (degrees)
Battery voltage (V)
🔍 Debug Mode
Enable verbose logging in platformio.ini:

ini
build_flags = -DCORE_DEBUG_LEVEL=5
📝 License
This project is open source. Use at your own risk. The authors are not responsible for any damages or injuries.

🤝 Contributing
Fork the repository
Create your feature branch
Test thoroughly with props removed
Submit pull request
⚡ Performance Specs
Control loop: 250Hz
Telemetry rate: 10Hz
WiFi latency: <20ms typical
Max tilt angle: 30°
Altitude hold accuracy: ±5cm
Remember: Building and flying drones can be dangerous. Always prioritize safety!