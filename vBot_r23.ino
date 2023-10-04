/**
 * Contributed by MylesAndMore and other open source software authors, 10.2023
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
#include "MPU6050_light.h"

typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
} Direction;

// List of sequential turns for the robot to make, in order
uint8_t turns[] = { LEFT, RIGHT, RIGHT, LEFT, RIGHT };
// Distance to go on the straight portion between each turn, in cm
#define STRAIGHT_DIST 100

#define NUM_TURNS sizeof(turns) / sizeof(turns[0])
uint8_t turnIndex = 0;

// The amount of time (in ms) to wait after a turn has been performed, Tte robot will not move or execute any code during this wait cycle.
#define WAIT_AFTER_TURN_MS 10
// The speed to complete turns at (value between 150-255)
// Be careful! This value correlates to the TURN_DEG found below
#define TURN_SPEED 180
// The degree value that the code should attempt to turn to during a turn
// This should be a little smaller than the actual wanted value to compensate for overshoot because I'm too pressed for time to implement actual PID
// TODO: pid lol
#define TURN_DEG 67
#define IMU // comment to disable IMU functionality

// IMU stuff; pins, objects, etc.
#ifdef IMU
  // The value within which to discard IMU values--this is in place due to small fluctuations in the protocol
  #define IMU_DEADBAND 1
  #define IMU_CALIBRATE // comment this line to disable IMU calibration on startup

  #define PIN_SDA 4 // alternate pins A4/D18
  #define PIN_SCL 5 // alternate pins A5/D19
  #define IMU_ADDR 0x68
  #define IMU_FREQ_KHZ 400

  MPU6050 mpu(Wire);
  float imu_zeroCalib = 0;
#endif

// Pin defs
// Drivetrain
#define PIN_SPEED_R 3 // A3
#define PIN_DIR_R1  12
#define PIN_DIR_R2  11
#define PIN_SPEED_L 6 // A6
#define PIN_DIR_L1  7
#define PIN_DIR_L2  8


/**
 * Sets the direction of the drivetrain.
 * @param dir direction to follow
*/
void drivetrain_setDir(Direction dir) {
  switch (dir) {
    case FORWARD:
      digitalWrite(PIN_DIR_R1, HIGH);
      digitalWrite(PIN_DIR_R2, LOW);
      digitalWrite(PIN_DIR_L1, HIGH);
      digitalWrite(PIN_DIR_L2, LOW);
      break;
    case BACKWARD:
      digitalWrite(PIN_DIR_R1, LOW);
      digitalWrite(PIN_DIR_R2, HIGH);
      digitalWrite(PIN_DIR_L1, LOW);
      digitalWrite(PIN_DIR_L2, HIGH);
      break;
    case LEFT:
      digitalWrite(PIN_DIR_R1, HIGH);
      digitalWrite(PIN_DIR_R2, LOW);
      digitalWrite(PIN_DIR_L1, LOW);
      digitalWrite(PIN_DIR_L2, HIGH);
      break;
    case RIGHT:
      digitalWrite(PIN_DIR_R1, LOW);
      digitalWrite(PIN_DIR_R2, HIGH);
      digitalWrite(PIN_DIR_L1, HIGH);
      digitalWrite(PIN_DIR_L2, LOW);
      break;
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
 * @return the current approximated angle of the Z axis (percieved) based on fusion math. 
*/ 
float imu_getZ() {
  // Fetch the latest computed angles from IMU/fusion algorithm and compensate it for our zero calibration
  mpu.update();
  return (mpu.getAngleZ() + imu_zeroCalib);
}

/**
 * Zeroes/resets the percieved IMU data.
*/ 
void imu_zero() {
  mpu.update();
  // Zeroing simply involves getting the inverse of the Z value at the time of calling the function and applying it to all future calculations
  imu_zeroCalib = -mpu.getAngleZ();
}

/* --- */

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
  // IMU
  #ifdef IMU
    pinMode(PIN_SDA, INPUT);
    pinMode(PIN_SCL, INPUT);
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
  
    // Initialize and optionally calibrate IMU
    Wire.begin();
    Wire.setClock(IMU_FREQ_KHZ * 1000); // 400kHZ is transmission frequency
    byte imuError = mpu.begin();
    if (imuError != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(imuError);
    }
    #ifdef IMU_CALIBRATE
      mpu.calcOffsets();
    #endif // IMU_CALIBRATE
  #endif // IMU
}

void loop() {
  // TODO: turn condition based on distance traveled
  if (false) {
    Serial.println("Wall detected!");
    // If the measured distance is less than the turn threshold, it's time to execute a turn, get the current turn from the list and...turn
    drivetrain_setSpeed(TURN_SPEED, TURN_SPEED);
    drivetrain_setDir(turns[turnIndex]);
    float turnSetpoint;
    if (turns[turnIndex] == LEFT) {
      Serial.println("Turning left...");
      // Set our turn setpoint (for later) to the current Z value plus whichever way we are turning
      #ifdef IMU
        turnSetpoint = imu_getZ() + TURN_DEG;
      #endif
    } else if (turns[turnIndex] == RIGHT) {
      Serial.println("Turning right...");
      #ifdef IMU
        turnSetpoint = imu_getZ() - TURN_DEG;
      #endif
    }
    #ifdef IMU
      // Wait (aka continue turning) until we meet the turn setpoint, only until after we can proceed (and stop the turn)
      // We use absolute value here so it works for both left and right turns
      while (abs(imu_getZ()) < abs(turnSetpoint));
    #else
      // No IMU enabled, just use a random delay that should maybe probably possibly get us to 90, this is the absolute failsafe
      delay(700);
    #endif
    // Increment the index so we execute the next turn next time
    turnIndex++;
    drivetrain_setSpeed(0, 0);
    Serial.println("Turn complete.");
    // Small delay for the servo to turn back and prevent possible false detections
    delay(WAIT_AFTER_TURN_MS);
    // Zero the IMU to prepare for the next turn
    imu_zero();
    // Check to see if that was our last turn, if so, stop after the preprogrammed delay
    if (turnIndex >= NUM_TURNS) {
      drivetrain_setDir(FORWARD);
      drivetrain_setSpeed(255, 255);
      // FIXME: once encoders are working, implement the encoder logic here as well
      Serial.println("Stopping...");
      drivetrain_setSpeed(0, 0);
      // Infinite loop to stop execution of the main program loop and hold the robot
      while (true);
    }
  }
  // Otherwise, continue straight at full speed
  drivetrain_setDir(FORWARD);
  drivetrain_setSpeed(255, 255);
}
