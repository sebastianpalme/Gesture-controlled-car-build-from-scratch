# Gesture-controlled-car-build-from-scratch
This is a Bluetooth-controlled car with servo motor steering, operated by a gesture-based remote. The entire car is designed from scratch and is 3D printable.

<img width="4032" height="2268" alt="Bild_2025-07-16_132212688-min" src="https://github.com/user-attachments/assets/2f62facd-08a4-47d0-871f-6058e5390c70" />


## Components and supplies

1x ESP 32

1x HC-05 Bluetooth Module

1x Servo motor

2x Battery 7.4V lithium

1x mpu-6050 gyro and accelerometer

1x ARDUINO UNO R3

1x L289N Motor Driver

4x DC MOTOR WHEELS

40x Jumper Wires

2x Geared DC Motor


## Tools and machines

1x Friend with a 3D printer

1x Soldering kit


## Apps and platforms

Arduino IDE or PlatformIO


# Project description

### Overview

This project demonstrates how to build a gesture-controlled robotic car using an ESP32 as a wireless remote control and an Arduino UNO to drive motors and steering. The remote reads tilt gestures using an MPU6050 accelerometer and sends control signals via Bluetooth (HC-05) to the car. The car then interprets these commands to control its direction and speed. 

### Fritzing sheet

<img width="4041" height="2523" alt="image" src="https://github.com/user-attachments/assets/044625fa-5bc7-4010-b925-2033607fd2c4" />

### HC-05 Configuration (AT Mode Tutorial)
To enable automatic Bluetooth connection between the HC-05 (on the Arduino UNO) and the ESP32, we must configure the HC-05 using AT commands. 
 

#### Step 1: Entering AT Mode
Method A – With a “KEY” pin: 
Connect KEY pin to 3.3V. Hold the KEY connection Power the HC-05 while KEY is HIGH. The LED will blink slowly (2 seconds interval) — this indicates AT mode. 
 
Method B – No KEY pin, If your module has a button: 
Hold the button while powering on the HC-05. 
The LED should blink slowly — you're now in AT mode. 
 
 
#### AT Mode Wiring Example 
VCC  to 5V

GND to GND 

TXD	Pin to 1 (TX) 

RXD via Voltage Divider to Pin 0 (RX) 

KEY	to 3.3V (or leave floating if not used) 

 
⚠️ The HC-05 RX pin must not receive 5V — always use a voltage divider (e.g., 1kΩ + 2kΩ)
 

#### Step 2: Open Serial Monitor

Open the Arduino IDE Serial Monitor 

Set Baud rate: 38400 

Line ending: Both NL & CR 

Type AT and press Enter 

✅ You should get a response: OK 

 
#### Step 3: Enter Configuration Commands
⚠️ Use exact spacing and format, or you’ll get ERROR: 

```bash
AT+ROLE=1     
```
//Set to Master mode 
 
```bash
AT+CMODE=0  
```    
//Only connect to a specific address (not any device). 
 
 ```bash
AT+BIND=3818,2BEA6A3E    
```  
//Bind to your ESP32's Bluetooth MAC address 

//Use commas instead of colons (:) 

//Example: ESP32 Bluetooth MAC: 38:18:2B:EA:6A:3E 

 
#### You can find the ESP32 MAC address by uploading this code to it: 


 ```cpp
#include <BluetoothSerial.h>

void setup() {      
	 Serial.begin(115200);      
	 Serial.println(BluetoothSerial::btStart() ? "Bluetooth Started" : "Failed");      
	 Serial.println("ESP32 MAC: " + BluetoothSerial().getBtAddress().toString());      
}     

void loop() {}      

``` 

Last but not least:
```bash
AT+INIT     
``` 
//Initializes the SPP profile (safe to run even if already initialized). 
 
#### Exit AT Mode 
Simply remove power and reconnect the HC-05 normally (without KEY pin HIGH). 
The module will now auto-connect to the bound ESP32 address on startup. 


## Code

### Arduino car

```cpp
#include <Arduino.h>
#include <Servo.h>

// Motor A control pins
#define ENA 5       // PWM speed control for motor A
#define IN1 8       // Direction control pin 1 for motor A
#define IN2 9       // Direction control pin 2 for motor A

// Motor B control pins
#define ENB 6       // PWM speed control for motor B
#define IN3 10      // Direction control pin 1 for motor B
#define IN4 11      // Direction control pin 2 for motor B

// LED pin (optional visual indicator)
#define LED 12

int MAX_SPEED = 255;                // Maximum motor speed (PWM value)

Servo servus;                       // Servo motor for steering
int lastSteeringAngle = 90;         // Last angle sent to the servo
int tolerance = 2;                  // Minimum difference required to update steering
float pitch;                        // Variable to store pitch value from remote

#define BT Serial                   // Alias for Serial communication over Bluetooth (HC-05)

void setup() {
  servus.attach(3);                 // Attach servo to digital pin 3
  servus.write(90);                 // Initialize steering to center position

  pinMode(LED, OUTPUT);             // Set LED pin as output
  digitalWrite(LED, LOW);          // Turn LED off initially

  // Set motor pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  BT.begin(115200);                 // Begin Bluetooth communication at 115200 baud

  BT.println("Car Ready");          // Print startup message via Bluetooth
}

// Sets motor speed and direction based on signed speed value
void setMotors(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);  // Limit speed to valid range

  if (speed > 0) {
    // Move forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    // Move backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Set PWM speed
  analogWrite(ENA, abs(speed));
  analogWrite(ENB, abs(speed));
}

void loop() {
  // Check if Bluetooth data is available
  while (BT.available()) {
    // Read one full message line (ending with \n)
    String line = BT.readStringUntil('\n');

    // If button press detected (e.g., "BUTTON:1")
    if (line.indexOf("BUTTON:1") >= 0) {
      digitalWrite(LED, HIGH);     // Turn LED on
    } else {
      digitalWrite(LED, LOW);      // Turn LED off
    }

    // Extract pitch value from message
    int pitchIndex = line.indexOf("PITCH:");
    if (pitchIndex >= 0) {
      int commaIndex1 = line.indexOf(',', pitchIndex);
      if (commaIndex1 == -1) commaIndex1 = line.length();
      String pitchString = line.substring(pitchIndex + 6, commaIndex1);
      pitch = pitchString.toFloat(); // Convert pitch string to float

      // Map pitch angle to steering angle (servo range)
      int steeringAngle = map(pitch, -75, 75, 60, 120);
      steeringAngle = constrain(steeringAngle, 0, 180);

      // Update servo only if the angle changed significantly
      if (abs(steeringAngle - lastSteeringAngle) > tolerance) {
        servus.write(steeringAngle);
        lastSteeringAngle = steeringAngle;
      }
    }

    // Extract roll value from message
    int rollIndex = line.indexOf("ROLL:");
    if (rollIndex >= 0) {
      int commaIndex2 = line.indexOf(',', rollIndex);
      String rollString = line.substring(rollIndex + 5, commaIndex2);
      float roll = rollString.toFloat(); // Convert roll string to float

      // Map roll angle to motor speed
      int speed = map(roll, -75, 75, -MAX_SPEED, MAX_SPEED);
      speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

      setMotors(speed); // Set motors with calculated speed
    }

    delay(10); // Small delay to avoid overloading the loop
  }
}
```

### ESP32 Remote

```cpp
#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#define MPU 0x68                      // I2C address of the MPU6050 sensor
BluetoothSerial BT;                  // Create Bluetooth serial object

float initialRoll = 0;               // Initial roll reference (for calibration)
float initialPitch = 0;              // Initial pitch reference (for calibration)
int16_t accX, accY, accZ;            // Raw accelerometer values

// Function to read raw accelerometer data from MPU6050
void readAccelerometer() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);    // Request 6 bytes: X, Y, Z (2 bytes each)

  accX = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for X
  accY = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for Y
  accZ = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for Z
}

// Calculates roll angle (tilt sideways) using accelerometer values
float calcRoll(int16_t ay, int16_t az) {
  float f_ay = ay / 16384.0;         // Convert to 'g' units
  float f_az = az / 16384.0;
  return atan2(f_ay, f_az) * 180.0 / PI;  // Convert radians to degrees
}

// Calculates pitch angle (tilt forward/backward) using accelerometer values
float calcPitch(int16_t ax, int16_t ay, int16_t az) {
  float f_ax = ax / 16384.0;
  float f_ay = ay / 16384.0;
  float f_az = az / 16384.0;
  return atan2(-f_ax, sqrt(f_ay * f_ay + f_az * f_az)) * 180.0 / PI;
}

void setup() {
  Serial.begin(115200);              // Start serial monitor for debugging

  Wire.begin(21, 22);                // Initialize I2C with custom SDA (21), SCL (22)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);                  // Power management register
  Wire.write(0);                     // Wake up MPU6050
  Wire.endTransmission();

  pinMode(16, INPUT);                // Input pin for recalibration button

  readAccelerometer();              // First sensor reading
  initialRoll = calcRoll(accY, accZ);      // Save initial roll for reference
  initialPitch = calcPitch(accX, accY, accZ);  // Save initial pitch for reference

  BT.setPin("password");             // Optional: set Bluetooth pairing PIN
  BT.begin("Name_of_Device");        // Start Bluetooth with custom device name

  Serial.println("Bluetooth started");
}

void loop() {
  readAccelerometer();               // Read accelerometer data

  float roll = calcRoll(accY, accZ) - initialRoll;     // Calculate roll offset from reference
  float pitch = calcPitch(accX, accY, accZ) - initialPitch; // Calculate pitch offset from reference

  int buttonState = digitalRead(16); // Check if recalibration button is pressed

  if (buttonState == HIGH) {
    // Recalibrate reference angles
    initialRoll = calcRoll(accY, accZ); 
    initialPitch = calcPitch(accX, accY, accZ);
    Serial.println("------Calibration successful------");
  }

  // Create formatted message to send via Bluetooth
  // Format: ROLL:<value>,PITCH:<value>,BUTTON:<state>
  String message = "ROLL:" + String(roll, 2) + ",PITCH:" + String(pitch, 2) + ",BUTTON:" + String(buttonState);

  BT.println(message);               // Send message via Bluetooth
  Serial.println(message);           // Print message to serial monitor

  delay(10);                         // Short delay between messages
}
```




## Driving construct
Here you can see the construct I designed for this project. 

You can download it below. 

<img width="860" height="632" alt="image" src="https://github.com/user-attachments/assets/40250b65-8820-473d-ac95-b90ff3bfaa0c" />

<img width="872" height="566" alt="image" src="https://github.com/user-attachments/assets/8065cfce-f37d-4c66-811d-80355e31346c" />

<img width="863" height="431" alt="image" src="https://github.com/user-attachments/assets/cc5f398a-3cc4-4784-b481-041708a6b888" />

<img width="876" height="317" alt="image" src="https://github.com/user-attachments/assets/8f7fae08-1c2c-4504-89d7-1b04c4a606de" />


## Result:
ESP-32 Remote

<img width="2268" height="4032" alt="image" src="https://github.com/user-attachments/assets/234b41a1-8b1d-4bfe-8ee7-fac9b1d02d8e" />

Arduino Car

<img width="4032" height="2268" alt="Bild_2025-07-16_132212688-min" src="https://github.com/user-attachments/assets/74d3ad04-8cc3-4039-a01f-47c7f941b2a3" />


<img width="4032" height="2268" alt="Bild_2025-07-16_134725712-min" src="https://github.com/user-attachments/assets/45566384-3cef-4964-b2fc-0a28e5dd6733" />


<img width="4032" height="2268" alt="Bild_2025-07-16_134818376-min" src="https://github.com/user-attachments/assets/e499270d-9c1a-47dc-9929-21b2eb46cd23" />


#### If you have any questions, ask me and I'll try to help you.






