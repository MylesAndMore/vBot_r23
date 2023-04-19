/**
 * Contributed by MylesAndMore and other open source softwares, 04-2023
 * Licensed under the GNU GPL V3 license (https://www.gnu.org/licenses/gpl-3.0.en.html).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <Servo.h>
#include <Wire.h>
#include <FastIMU.h>

// Define direction macros
#define FORWARD 0
#define BACKWARD 1
#define LEFT 1
#define RIGHT 2
// List of sequential turns for the robot to make in order!
// This is what you'll need to update on competition day :)
uint8_t turns[] = { LEFT, RIGHT, LEFT };
uint8_t turnIndex = 0;

// Configuration values that we can change to change robot's behavior
// The distance threshold the robot must reach to execute a turn from the list (mesaured in cm)
#define TURN_THRESHOLD 25
// The amount of times the ultrasonic sensor is measured per cycle and averaged (due to interference)
#define ULTRASONIC_AVG 10
// THe degree value that the code should attempt to turn to during a turn
// This should be a little smaller than the actual wanted value to compensate for overshoot because I'm too pressed for time to implement actual PID
#define TURN_DEG 86
#define IMU // comment to disable IMU functionality

// Define IO pins, naming is mostly self-explanatory
// Drivetrain
#define PIN_SPEED_R A3
#define PIN_DIR_R1  12
#define PIN_DIR_R2  11
#define PIN_SPEED_L A6
#define PIN_DIR_L1  7
#define PIN_DIR_L2  8
// Ultrasonic sensor / servo
#define PIN_ECHO 2
#define PIN_TRIG 10
#define PIN_SERVO 9
#define SERVO_OFFSET -9 // The value at which to offset the servo's position (telling the servo 90 degrees is usually not perfectly centered)

// IMU stuff; pins, objects, etc.
#ifdef IMU
  #define PIN_SDA A4 // alternate pin D18
  #define PIN_SCL A5 // alternate pin D19
  #define IMU_ADDR 0x68
  #define IMU_CALIBRATE // comment to disable IMU calibration on startup

  MPU6050 imu;
  // Create data storage for IMU
  calData imu_calData = { 0 };  //Calibration data
  GyroData imu_gyroData;
  float turnSetpoint;
#endif

// Define servo obect
Servo servo;


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
float ultrasonic_measure() {
  float total_dist = 0;
  // We use a for loop to measure the distance multiple times and take an average, because the sound waves are subject to interference
  for (uint8_t i = 0; i < ULTRASONIC_AVG; i++) {
    float echo_dist;
    // Sequence to send out a sound pulse
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(5);                                                                              
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(15);
    digitalWrite(PIN_TRIG, LOW);
    // Get the length of the sound pulse and convert it to cm
    echo_dist = pulseIn(PIN_ECHO,HIGH);
    echo_dist *= 0.01657;
    // Add the recorded distance to the tottal that we will later average
    total_dist += round(echo_dist);
  }
  // Find the average using the amount of times we ran a measurement and return the final output
  float dist = (total_dist / ULTRASONIC_AVG);
  Serial.print("Measured distance: ");
  Serial.println(dist);
  return dist;
}

// ---

void setup() {
  // Initialize serial port for debugging
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
  // IMU
  #ifdef IMU
    pinMode(PIN_SDA, INPUT);
    pinMode(PIN_SCL, INPUT);
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
  
    // Initialize and optionally calibrate IMU
    Wire.begin();
    Wire.setClock(400000); // 400kHZ is transmission frequency
    int imuError = imu.init(calib, IMU_ADDR);
    if (imuError != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(imuError);
    }
    #ifdef IMU_CALIBRATE
      imu.calibrateAccelGyro(&calib);
      imu.init(calib, IMU_ADDR);
    #endif // IMU_CALIBRATE
  #endif // IMU

  // Tell servo object what pin the servo is on
  servo.attach(PIN_SERVO);
  // Set the servo to its centerpoint so we're looking ahead to detect obstacles
  servo.write(90 + SERVO_OFFSET);
}

void loop() {
  #ifdef IMU
    // Refresh IMU data
    imu.getGyro(&imu_gyroData);
  #endif
  if (ultrasonic_measure() < TURN_THRESHOLD) {
    Serial.println("Wall detected!");
    // If the measured distance is less than the turn threshold, it's time to execute a turn, get the current turn from the list and...turn
    drivetrain_setDir(FORWARD, turns[turnIndex]);
    if (turns[turnIndex] == LEFT) {
      // Turn the servo in the direction we're turning, mostly for visual confirmation of the turn and doesn't really do anything
      servo.write(135 + SERVO_OFFSET);
      Serial.println("Turning left...");
      // Set our turn setpoint (for later) to the current Z value plus whichever way we are turning
      turnSetpoint = imu_gyroData.gyroZ - TURN_DEG;
    } else if (turns[turnIndex] == RIGHT) {
      servo.write(45 + SERVO_OFFSET);
      Serial.println("Turning right...");
      turnSetpoint = imu_gyroData.gyroZ + TURN_DEG;
    } else {
      // Otherwise, we must have hit the end of our turning list, so stop
      Serial.println("Stopping...");
      drivetrain_setSpeed(0, 0);
      // Infinite loop to stop execution of the main program loop
      while (true) { }
    }
    #ifdef IMU
      // // Just refresh IMU data until we meet the turn setpoint, then we can proceed (and stop the turn)
      // while (imu_gyroData.gyroZ < turnSetpoint) {
      //   imu.getGyro(&imu_gyroData);
      // }
    #else
      // No IMU enabled, just use a random delay that should maybe probably possibly get us to 90
      delay(800);
    #endif
    // Increment the index so we execute the next turn next time
    turnIndex++;
    // Turn the servo to face forwards again so we can detect the next obstacle
    servo.write(90 + SERVO_OFFSET);
    // Small delay for the servo to turn back and prevent possible false detections
    delay(50);
  }
  // Otherwise, continue straight at full speed
  drivetrain_setDir(FORWARD, FORWARD);
  drivetrain_setSpeed(255, 255);
}
