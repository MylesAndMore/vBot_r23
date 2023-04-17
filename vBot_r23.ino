#include <Servo.h>
#include <Wire.h>
#include "FastIMU.h"

// Define IO pins, naming is mostly self-explanatory
// Drivetrain
#define PIN_SPEED_R 3
#define PIN_DIR_R1  12
#define PIN_DIR_R2  11
#define PIN_SPEED_L 6
#define PIN_DIR_L1  7
#define PIN_DIR_L2  8
// Ultrasonic sensor / servo
#define PIN_ECHO 2
#define PIN_TRIG 10
#define PIN_SERVO 9
#define SERVO_OFFSET -9
// IMU
#define IMU_ADDR 0x68
#define IMU_CALIBRATE // comment this line out to skip IMU calibration on startup

// Define servo object
Servo servo;
// Define MPU6050-related objects and variables
MPU6050 imu;
calData calib = { 0 };
AccelData accelData;
GyroData gyrData;

/**
 * Sets the direction of the drivetrain, parameters for different directions specified below.
 * @param dir 0 for forwards, 1 for backwards
 * @param turn 0 for no turn, 1 for left turn, 2 for right turn
*/
void drivetrain_setDir(uint8_t dir, uint8_t turn) {
  // Logic to figure out what we need to set motors to, then, well...set the motors
  if (dir == 0) {
    if (turn == 0) {
      // Forward
      digitalWrite(PIN_DIR_R1, HIGH);
      digitalWrite(PIN_DIR_R2, LOW);
      digitalWrite(PIN_DIR_L1, HIGH);
      digitalWrite(PIN_DIR_L2, LOW);
    } else if (turn == 1) {
      // Forward left turn
      // TODO: experiment with changing speeds to try and speed up turns?
      digitalWrite(PIN_DIR_R1, HIGH);
      digitalWrite(PIN_DIR_R2, LOW);
      digitalWrite(PIN_DIR_L1, LOW);
      digitalWrite(PIN_DIR_L2, HIGH);
    } else if (turn == 2) {
      // Forward right turn
      digitalWrite(PIN_DIR_R1, LOW);
      digitalWrite(PIN_DIR_R2, HIGH);
      digitalWrite(PIN_DIR_L1, HIGH);
      digitalWrite(PIN_DIR_L2, LOW);
    }
  } else if (dir == 1) {
    if (turn == 0) {
      // Backward
      digitalWrite(PIN_DIR_R1, LOW);
      digitalWrite(PIN_DIR_R2, HIGH);
      digitalWrite(PIN_DIR_L1, LOW);
      digitalWrite(PIN_DIR_L2, HIGH);
    } else if (turn == 1) {
      // Backward left turn
      digitalWrite(PIN_DIR_R1, LOW);
      digitalWrite(PIN_DIR_R2, HIGH);
      digitalWrite(PIN_DIR_L1, HIGH);
      digitalWrite(PIN_DIR_L2, LOW);
    } else if (turn == 2) {
      // Backward right turn
      digitalWrite(PIN_DIR_R1, HIGH);
      digitalWrite(PIN_DIR_R2, LOW);
      digitalWrite(PIN_DIR_L1, LOW);
      digitalWrite(PIN_DIR_L2, HIGH);
    }
  }
}

/**
 * Sets the speed of the drivetrain (all motors).
 * ~150ish is the minimum for all motors to turn
 * @param speedL the speed value (0 to 255) to use on the left
 * @param speedR the speed value (0 to 255) to use on the right
*/
void drivetrain_setSpeed(uint8_t speedL, uint8_t speedR) {
  analogWrite(PIN_SPEED_L, speedL); 
  analogWrite(PIN_SPEED_R, speedR); 
}

/**
 * Measures the distance from the object currently in front of the ultrasonic sensor.
 * @return the distance in cm
*/
long ultrasonic_measure() {
  long echo_distance;
  digitalWrite(PIN_TRIG,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(PIN_TRIG,HIGH);
  delayMicroseconds(15);
  digitalWrite(PIN_TRIG,LOW);
  echo_distance=pulseIn(PIN_ECHO,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}

void setup() {
  // Initialize serial port for debugging
  // Don't keep this in the final build!
  Serial.begin(115200);

  // Set up pin modes
  // Drivetrain
  pinMode(PIN_DIR_R1, OUTPUT); 
  pinMode(PIN_DIR_R2, OUTPUT); 
  pinMode(PIN_SPEED_L, OUTPUT);  
  pinMode(PIN_DIR_L1, OUTPUT);
  pinMode(PIN_DIR_L2, OUTPUT); 
  pinMode(PIN_SPEED_R, OUTPUT); 
  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT); 
  pinMode(PIN_ECHO, INPUT);

  // Tell servo object what pin the servo is on
  servo.attach(PIN_SERVO);
  // Set the servo to its centerpoint
  servo.write(90 + SERVO_OFFSET);

  // Set up 400kHZ clock for MPU6050
  Wire.begin();
  Wire.setClock(400000);
  imu.init(calib, IMU_ADDR);
}

void loop() {
  imu.update();
  imu.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  imu.getGyro(&gyrData);
  Serial.print(gyrData.gyroX);
  Serial.print("\t");
  Serial.print(gyrData.gyroY);
  Serial.print("\t");
  Serial.print(gyrData.gyroZ);
}
