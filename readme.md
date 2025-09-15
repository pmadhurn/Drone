MPU6050 (Gyro/Accelerometer):
- VCC → 3.3V
- GND → GND  
- SCL → GPIO 22
- SDA → GPIO 21
- AD0 → GND

Motors/ESCs:
- Motor 1 (Front Right) → GPIO 25
- Motor 2 (Front Left) → GPIO 26  
- Motor 3 (Rear Left) → GPIO 27
- Motor 4 (Rear Right) → GPIO 14

HC-SR04 Ultrasonic Sensors (4x):
Bottom Sensors (45° angle mount):
- Sensor 1 (Front Right):
  - Trigger → GPIO 32
  - Echo → GPIO 33
  
- Sensor 2 (Front Left):
  - Trigger → GPIO 18
  - Echo → GPIO 19
  
- Sensor 3 (Rear Left):
  - Trigger → GPIO 16
  - Echo → GPIO 17
  
- Sensor 4 (Rear Right):
  - Trigger → GPIO 4
  - Echo → GPIO 5

Power for all HC-SR04:
- VCC → 5V (from ESC BEC or separate regulator)
- GND → Common Ground

Optional (Recommended):
- Battery Voltage Monitor → GPIO 34 (through voltage divider)
- Status LED → GPIO 2 (built-in LED)