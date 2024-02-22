/**
 * vBot_r24.ino
 * Contributed to by MylesAndMore and other open source software authors, 02.2024
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

// GOOD LUCK!! ~ Myles

#include <Wire.h>
#include "MPU6050_light.h"
#include "QuickPID.h"

typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
} Direction;

typedef enum {
  TIME, // Uses a set delay between turns. Most robust solution but not always the most reliable.
  ENCODER, // Uses a set distance between turns. Mostly works, except the encoder is mounted a little weird.
} TurnType;

/** @section config */

// Note: if the code fails to upload, detach the black board from the top of the Arduino (blue board) and try again. Not sure why this happens.

// List of sequential turns for the robot to make, in order
Direction turns[] = { RIGHT, LEFT, LEFT };
// The way the robot decides when to execute a turn--see the options/descriptions in the TurnType enum above
#define TURN_TYPE ENCODER
// This modifier changes based on the TURN_TYPE selected above
// TIME: The time to wait between the turn, in ms (default 1500)
// ENCODER: The distance to go on the straight portion between each turn, in mm (default 500)
int turns_modifier[] = { 500, 500, 500, 1000 };
// Note: the modifier array should have one extra element (number) than the turns array.
// In TIME mode, the final element dictates how long to drive straight (in ms) before ending.
// In ENCODER mode, the final element dictates how long to drive straight in mm instead.

/** @section advanved config */

// The speed to complete turns at (value between 150-255)
#define TURN_SPEED 160
// The amount of time to turn for if we can't rely on the IMU...this is a last-ditch effort that will hopefully get us to 90deg
#define TURN_BLIND_MS 800

// In ENCODER mode, a turn will be executed this many mm before every modifier
// decelerations can take a bit of time, so this can help to combat that
#define ENCODER_OFFSET 5

// The value within which to discard IMU values--this is in place due to small fluctuations in the protocol
#define IMU_DEADBAND 1

#define PIN_SDA 4 // alternate pins A4/D18
#define PIN_SCL 5 // alternate pins A5/D19
#define IMU_ADDR 0x68
#define IMU_FREQ_KHZ 400

MPU6050 mpu(Wire);
float imu_zeroCalib = 0;

#define DRIVETRAIN_KP 8
#define DRIVETRAIN_KI 0
#define DRIVETRAIN_KD 0.55
float in, out, set;
QuickPID pid(&in, &out, &set);

#define DRIVETRAIN_BASE_SPEED 128

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

void drivetrain_set(int speedL, int speedR) {
  if (speedL > 0 && speedR > 0) {
    drivetrain_setDir(FORWARD);
  } else if (speedL > 0 && speedR < 0) {
    drivetrain_setDir(RIGHT);
  } else if (speedL < 0 && speedR > 0) {
    drivetrain_setDir(LEFT);
  } else {
    drivetrain_setDir(BACKWARD);
  }
  drivetrain_setSpeed(abs(speedL), abs(speedR));
}

/** @section IMU */

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

/** @section encoder */

#define PIN_ENCODER 2
#define ENCODER_CPR 40 // Counts per revolution on encoder wheel
#define ENCODER_WHEELDIA 64 // Wheel diameter in mm

int64_t encoder_cts; // Tracks number of encoder pulses
float encoder_dist; // Estimated distance covered based on how many encoder pulses have been recorded (aka how many times the wheel has turned)
bool useEncoder = true; // Global flag for whether the encoder should be used to count distance (currently)

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
  if (useEncoder) {
    encoder_cts++;
    encoder_dist = (float)encoder_cts / ENCODER_CPR * M_PI * ENCODER_WHEELDIA;
    // Serial.println("Encoder dist: " + encoder_dist);
  }
}

/* --- */

bool hasTurned = false;
uint32_t lastT;

bool readyToTurn() {
  if (TURN_TYPE == TIME) {
    if (!hasTurned) {
      lastT = millis();
      hasTurned = true;
    }
    if (millis() - lastT > TURN_BLIND_MS) {
      lastT = millis();
      return true;
    }
    lastT = millis();
  } else if (TURN_TYPE == ENCODER) {
    // Serial.print("dist: ");
    // Serial.print(encoder_dist);
    // Serial.print(" target: ");
    // Serial.println(turns_modifier[turnIndex]);
    return encoder_dist >= turns_modifier[turnIndex] - ENCODER_OFFSET;
  }
}

void executeTurn() {
  // Was the last turn our last turn?
  if (turnIndex > NUM_TURNS) {
    Serial.println("Stopping...");
    drivetrain_setSpeed(0, 0);
    // Infinite loop to stop execution of the main program loop and hold the robot
    while (true);
  }
  Serial.print("Executing ");
  if (turns[turnIndex] == LEFT) {
    Serial.print("LEFT ");
    set += 90;
  } else if (turns[turnIndex] == RIGHT) {
    Serial.print("RIGHT ");
    set -= 90;
  } else {
    Serial.print("FINAL ");
  }
  Serial.println("turn");
  // Reset the encoder counts and therefore distance
  encoder_cts = 0;
  encoder_dist = 0;
  turnIndex++;
}

void setup() {
  Serial.begin(115200);
  drivetrain_init();
  encoder_init();
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  // Initialize and calibrate IMU
  Wire.begin();
  Wire.setClock(IMU_FREQ_KHZ * 1000);
  byte imuError = mpu.begin();
  if (imuError != 0)
    Serial.println("Error initializing IMU: " + imuError);
  mpu.calcOffsets();
  pid.SetTunings(DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KD);
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(pid.Control::automatic);
}

void loop() {
  // Compute deviation from desired to percieved degree value
  in = imu_getZ();
  pid.Compute();
  // Apply control signal to drivetrain
  int adjustment = (int)out;
  int speedL = constrain(DRIVETRAIN_BASE_SPEED - adjustment, -255, 255);
  int speedR = constrain(DRIVETRAIN_BASE_SPEED + adjustment, -255, 255);
  // Serial.println(String("[PID] in: ") + in + ", set: " + set + ", err: " + (set - in) + ", out: " + out + ", adj: " + adjustment + ", L::" + speedL + ", R::" + speedR);
  drivetrain_set(speedL, speedR);
  // Turn if necessary
  if (readyToTurn())
    executeTurn();
  // Only update encoder if we aren't currently turning/correcting
  useEncoder = abs(set - in) < 5;
}
