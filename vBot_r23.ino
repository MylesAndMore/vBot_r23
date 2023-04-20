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
#include "MPU6050_light.h"

// Define direction macros
#define FORWARD 0
#define BACKWARD 1
#define LEFT 1
#define RIGHT 2
// List of sequential turns for the robot to make in order!
// This is what you'll need to update on competition day :)
uint8_t turns[] = { LEFT, RIGHT, LEFT };
uint8_t turnsAmount = turns.size()--;
uint8_t turnIndex = 0;

// Configuration values that we can change to change robot's behavior
// The distance threshold the robot must reach to execute a turn from the list (mesaured in cm)
#define TURN_THRESHOLD 25
// The amount of times the ultrasonic sensor is measured per cycle and averaged (due to interference)
#define ULTRASONIC_AVG 10
// The amount of time (in ms) to wait after a turn has been performed. This is mainly so that the servo can return back to a zero position and we don't get any false positives.
// The robot will not move or execute any code during this wait cycle.
#define WAIT_AFTER_TURN_MS 50
// The amount of time (in ms) to drive after the final turn has been made
#define FINAL_DRIVE_MS 1000
// THe degree value that the code should attempt to turn to during a turn
// This should be a little smaller than the actual wanted value to compensate for overshoot because I'm too pressed for time to implement actual PID
#define TURN_DEG 85
#define IMU // comment to disable IMU functionality
#define DATA_LOGGING // uncomment to enable data logging functionality (for practice log), should be commented for competition

// Define IO pins, naming is mostly self-explanatory
// Drivetrain
#define PIN_SPEED_R 3 // A3
#define PIN_DIR_R1  12
#define PIN_DIR_R2  11
#define PIN_SPEED_L 6 // A6
#define PIN_DIR_L1  7
#define PIN_DIR_L2  8
// Ultrasonic sensor / servo
#define PIN_ECHO 2
#define PIN_TRIG 10
float dist;

#define PIN_SERVO 9
#define SERVO_OFFSET -9 // The value at which to offset the servo's position (telling the servo 90 degrees is usually not perfectly centered)

// IMU stuff; pins, objects, etc.
#ifdef IMU
  // The value within which to discard IMU values--this is in place due to small fluctuations in the protocol
  #define IMU_DEADBAND 1
  #define IMU_CALIBRATE // comment this line to disable IMU calibration on startup

  #define PIN_SDA 4 // alternate pins A4/D18
  #define PIN_SCL 5 // alternate pins A5/D19
  #define IMU_ADDR 0x68

  MPU6050 mpu(Wire);
  float imu_zeroCalib = 0;
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
    echo_dist = pulseIn(PIN_ECHO, HIGH);
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

#ifdef DATA_LOGGING
  /**
   * Prepares for data logging. This is when the stopwatch is started.
  */ 
  void logInit() {
    float log_currentDist[];
    float log_overTime[];
    float log_startTime = millis();
  }

  /**
   * Logs reccuring data for the robot's practice log.
   * @param log_dist the current distance value from the ultrasonic sensor to log.
  */ 
  void logData(float log_dist) {
    log_currentDist.push_back(log_dist);
    log_overTime.push_back((millis() - log_startTime) / 1000);
  }

  /**
   * Outputs the log to serial.
  */ 
  void logOutput() {
    // Iterate through all values of distance and time and print them to serial
    for (uint i = 0; i <= overTime.size(); i++) {
      Serial.print(TURN_THRESHOLD);
      Serial.print("\t");
      Serial.print(currentDist[i]);
      Serial.print("\t");
      Serial.println(overTime[i]);
    }
  }
#endif

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
    byte imuError = mpu.begin();
    if (imuError != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(imuError);
    }
    #ifdef IMU_CALIBRATE
      mpu.calcOffsets();
    #endif // IMU_CALIBRATE
  #endif // IMU

  // Tell servo object what pin the servo is on
  servo.attach(PIN_SERVO);
  // Set the servo to its centerpoint so we're looking ahead to detect obstacles
  servo.write(90 + SERVO_OFFSET);
  
  #ifdef DATA_LOGGING
    // If data logging is enabled, initialize it now
    logInit();
  #endif
}

void loop() {
  dist = ultrasonic_measure();
  #ifdef DATA_LOGGING
    logData(dist);
  #endif
  if (dist < TURN_THRESHOLD) {
    Serial.println("Wall detected!");
    // If the measured distance is less than the turn threshold, it's time to execute a turn, get the current turn from the list and...turn
    drivetrain_setDir(FORWARD, turns[turnIndex]);
    float turnSetpoint;
    if (turns[turnIndex] == LEFT) {
      // Turn the servo in the direction we're turning, mostly for visual confirmation of the turn and doesn't really do anything
      servo.write(135 + SERVO_OFFSET);
      Serial.println("Turning left...");
      // Set our turn setpoint (for later) to the current Z value plus whichever way we are turning
      #ifdef IMU
        turnSetpoint = imu_getZ() + TURN_DEG;
      #endif
    } else if (turns[turnIndex] == RIGHT) {
      servo.write(45 + SERVO_OFFSET);
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
    // Turn the servo to face forwards again so we can detect the next obstacle
    servo.write(90 + SERVO_OFFSET);
    drivetrain_setSpeed(0, 0);
    Serial.println("Turn complete.");
    // Small delay for the servo to turn back and prevent possible false detections
    delay(WAIT_AFTER_TURN_MS);
    // Zero the IMU to prepare for the next turn
    imu_zero();
    // Check to see if that was our last turn, if so, stop after the preprogrammed delay
    if (turnIndex >= turnsAmount) {
      delay(FINAL_DRIVE_MS);
      Serial.println("Stopping...");
      drivetrain_setSpeed(0, 0);
      #ifdef DATA_LOGGING
        // If data logging is enabled, stop logging and wait a bit before printing data to serial
        delay(30000);
        logoutput();
      #endif
      // Infinite loop to stop execution of the main program loop and hold the robot
      while (true);
    }
  }
  // Otherwise, continue straight at full speed
  drivetrain_setDir(FORWARD, FORWARD);
  drivetrain_setSpeed(255, 255);
}
