#ifndef CONFIG_H
#define CONFIG_H

// Motor Pins
#define MOTOR_1_PIN 25  // Front Right
#define MOTOR_2_PIN 26  // Front Left
#define MOTOR_3_PIN 27  // Rear Left
#define MOTOR_4_PIN 14  // Rear Right

// I2C Pins for MPU6050
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

// Ultrasonic Sensor Pins
#define US1_TRIG_PIN 32  // Front Right
#define US1_ECHO_PIN 33
#define US2_TRIG_PIN 18  // Front Left
#define US2_ECHO_PIN 19
#define US3_TRIG_PIN 16  // Rear Left
#define US3_ECHO_PIN 17
#define US4_TRIG_PIN 4   // Rear Right
#define US4_ECHO_PIN 5

// Battery Monitor
#define BATTERY_ADC_PIN 34

// WiFi Settings
#define WIFI_SSID "DroneAP"
#define WIFI_PASS "drone1234"
#define CONTROL_PORT 8080
#define TELEMETRY_PORT 8081

// Flight Parameters
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 1500
#define ARM_THROTTLE 1100
#define FLIGHT_LOOP_FREQ 250  // Hz
#define MAX_ANGLE 30.0f       // degrees

// Safety
#define BATTERY_LOW_VOLTAGE 10.5f  // 3S battery
#define BATTERY_CRITICAL_VOLTAGE 9.9f

#endif