/**
 * Contributed by MylesAndMore and other open source software authors, 11.2023
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

#include <Wire.h>
#include "MPU6050_light.h"

typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
} Direction;

typedef enum {
  TIME, // Uses a set delay between turns. Most robust solution but not always the most reliable.
  ENCODER, // Uses a set distance between turns. Mostly works, except the encoder is mounted a little weird.
  ULTRASONIC, // Uses no set parameters and simply goes straight until an obstacle is sensed, and then executes a turn.
  /* Unused */
  TIME_ULTRASONIC, // Uses the time method unless the ultrasonic senses an obstacle, in which case a turn is executed early.
  ENCODER_ULTRASONIC // Uses the encoder method unless the ultrasonic senses an obstacle, in which case a turn is executed early.
} TurnType;

/** @section config */

// List of sequential turns for the robot to make, in order
Direction turns[] = { LEFT, RIGHT, RIGHT, LEFT, RIGHT };
// The way the robot decides when to execute a turn--see the options/descriptions in the TurnType enum above
#define TURN_TYPE ENCODER
// This modifier changes based on the TURN_TYPE selected above
// TIME: The time to wait between the turn, in ms (default 1500)
// ENCODER: The distance to go on the straight portion between each turn, in mm (default 500)
// ULTRASONIC: The distance from a wall at which to execute a turn, in mm (default 25)
int turns_modifier[] = { 250, 500, 500, 500, 500, 1000 };
// Note: the modifier array should have one extra element (number) than the turns array.
// In TIME and ULTRASONIC modes, the final element dictates how long to drive straight (in ms) before ending.
// In ENCODER mode, the final element dictates how long to drive straight in mm instead.

/** @section advanved config */

// The speed to complete turns at (value between 150-255)
// Be careful! This value correlates to the TURN_DEG found below
#define TURN_SPEED 180
// The degree value that the code should attempt to turn to during a turn
// This should be a little smaller than the actual wanted value to compensate for overshoot because I'm too pressed for time to implement actual PID
#define TURN_DEG 67
// The amount of time to turn for if we can't rely on the IMU...this is a last-ditch effort that will hopefully get us to 90deg
#define TURN_BLIND_MS 700

// The amount of times the ultrasonic sensor is measured per cycle and averaged (due to interference)
#define ULTRASONIC_AVG 10

#define IMU 0 // set to 0 to disable IMU functionality
#if IMU
  // The value within which to discard IMU values--this is in place due to small fluctuations in the protocol
  #define IMU_DEADBAND 1
  #define IMU_CALIBRATE 1 // set to 0 to disable IMU calibration on startup, not recomended

  #define PIN_SDA 4 // alternate pins A4/D18
  #define PIN_SCL 5 // alternate pins A5/D19
  #define IMU_ADDR 0x68
  #define IMU_FREQ_KHZ 400

  MPU6050 mpu(Wire);
  float imu_zeroCalib = 0;
#endif

/** @section drivetrain */

#define NUM_TURNS (sizeof(turns) / sizeof(turns[0]))
uint8_t turnIndex = 0;

#define PIN_SPEED_R 3 // A3
#define PIN_DIR_R1  12
#define PIN_DIR_R2  11
#define PIN_SPEED_L 6 // A6
#define PIN_DIR_L1  7
#define PIN_DIR_L2  8

/**
 * Initializes the drivetrain (sets pin modes).
*/
void drivetrain_init() {
  pinMode(PIN_DIR_R1, OUTPUT); 
  pinMode(PIN_DIR_R2, OUTPUT); 
  pinMode(PIN_SPEED_L, OUTPUT);  
  pinMode(PIN_DIR_L1, OUTPUT);
  pinMode(PIN_DIR_L2, OUTPUT);
  pinMode(PIN_SPEED_R, OUTPUT); 
}

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

/** @section IMU */

#if IMU

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

#endif

/** @section ultrasonic */

#if (TURN_TYPE == ULTRASONIC || TURN_TYPE == TIME_ULTRASONIC || TURN_TYPE == ENCODER_ULTRASONIC)

  #define PIN_ECHO 2
  #define PIN_TRIG 10

  /**
    * Measures the distance from the object currently in front of the ultrasonic sensor.
    * @return the distance in mm
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
      // Get the length of the sound pulse and convert it to mm
      echo_dist = pulseIn(PIN_ECHO, HIGH);
      echo_dist *= 0.01657;
      // Add the recorded distance to the tottal that we will later average
      total_dist += round(echo_dist);
    }
    // Find the average using the amount of times we ran a measurement and return the final output
    float dist = (total_dist / ULTRASONIC_AVG);
    // Serial.print("Measured distance: ");
    // Serial.println(dist);
    return dist;
  }

#endif

/** @section encoder */

#if (TURN_TYPE == ENCODER || TURN_TYPE == ENCODER_ULTRASONIC)

  #define PIN_ENCODER 2
  #define ENCODER_CPR 40 // Counts per revolution on encoder wheel
  #define ENCODER_WHEELDIA 52 // Wheel diameter in mm

  int encoder_cts; // Tracks number of encoder pulses
  float encoder_dist; // Estimated distance covered based on how many encoder pulses have been recorded (aka how many times the wheel has turned)

  /**
   * Initializes the encoder (binds interrupt).
  */
  void encoder_init() {
    pinMode(PIN_ENCODER, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoder_trigger, RISING);
  }

  /**
   * Should be called every time the encoder is triggered, likely using an interrupt.
  */
  void encoder_trigger() {
    encoder_cts++;
    encoder_dist = (float)encoder_cts / ENCODER_CPR * M_PI * ENCODER_WHEELDIA;
    // Serial.print("Encoder dist: ");
    // Serial.println(encoder_dist);
  }

#endif

/* --- */

void executeTurn() {
  drivetrain_setSpeed(TURN_SPEED, TURN_SPEED);
  drivetrain_setDir(turns[turnIndex]);
  float turnSetpoint;
  if (turns[turnIndex] == LEFT) {
    Serial.println("Turning left...");
    // Set our turn setpoint (for later) to the current Z value plus whichever way we are turning
    #if IMU
      turnSetpoint = imu_getZ() + TURN_DEG;
    #endif
  } else if (turns[turnIndex] == RIGHT) {
    Serial.println("Turning right...");
    #if IMU
      turnSetpoint = imu_getZ() - TURN_DEG;
    #endif
  }
  #if IMU
    // Wait (aka continue turning) until we meet the turn setpoint, only until after we can proceed (and stop the turn)
    // We use absolute value here so it works for both left and right turns
    while (abs(imu_getZ()) < abs(turnSetpoint));
  #else
    // No IMU enabled, just use a random delay that should maybe probably possibly get us to 90, this is the absolute failsafe
    delay(TURN_BLIND_MS);
  #endif
  // Increment the index so we execute the next turn next time
  turnIndex++;
  drivetrain_setSpeed(0, 0);
  Serial.println("Turn complete.");
  #if IMU
    // Zero the IMU to prepare for the next turn
    imu_zero();
  #endif
  // Reset the encoder counts and therefore distance
  if (TURN_TYPE == ENCODER) {
    encoder_cts = 0;
    encoder_dist = 0;
  }
  delay(30);
  // Check to see if that was our last turn, if so, stop after the preprogrammed delay
  if (turnIndex >= NUM_TURNS) {
    drivetrain_setDir(FORWARD);
    drivetrain_setSpeed(255, 255);
    if (TURN_TYPE == TIME || TURN_TYPE == ULTRASONIC) {
      // The last value in the modifier array is how long to wait in the final straight
      delay(turns_modifier[turnIndex]);
    } else if (TURN_TYPE == ENCODER) {
      /* === ENCODER === */
      // In this mode, the last value signifies a distance rather than a time
      while ((int)encoder_dist < turns_modifier[turnIndex]) {
        Serial.print("Dist: ");
        Serial.print((int)encoder_dist);
        Serial.print(" Target: ");
        Serial.println(turns_modifier[turnIndex]);
      }
    }
    Serial.println("Stopping...");
    drivetrain_setSpeed(0, 0);
    // Infinite loop to stop execution of the main program loop and hold the robot
    while (true);
  }
}

void setup() {
  Serial.begin(115200);
  drivetrain_init();
  encoder_init();
  #if IMU
    pinMode(PIN_SDA, INPUT);
    pinMode(PIN_SCL, INPUT);
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    // Initialize and optionally calibrate IMU
    Wire.begin();
    Wire.setClock(IMU_FREQ_KHZ * 1000);
    byte imuError = mpu.begin();
    if (imuError != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(imuError);
    }
    #if IMU_CALIBRATE
      mpu.calcOffsets();
    #endif // IMU_CALIBRATE
  #endif // IMU
}

void loop() {
  drivetrain_setDir(FORWARD);
  drivetrain_setSpeed(255, 255);
  // Check for turn condition based on current TURN_TYPE
  switch (TURN_TYPE) {
    case TIME:
      // In this mode, the modifier signifies the time to wait (go straight for) in between turns
      delay(turns_modifier[turnIndex]);
      executeTurn();
      break;
    case ENCODER:
      // Here the modifier signifies the distance to go straight for before a turn, so if we've achieved that distance, it's time!
      if (encoder_dist >= turns_modifier[turnIndex])
        executeTurn();
      break;
    case ULTRASONIC:
      // Modifier signifies the distance at which to begin a turn
      if ((int)ultrasonic_measure() <= turns_modifier[turnIndex])
        executeTurn();
      break;
    default:
      Serial.println("ERROR: invalid TURN_TYPE!");
      return;
  }
}
