/**
 * Authored by MylesAndMore [04-2023], other open source softwares
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

// Define direction macros
#define FORWARD 0
#define BACKWARD 1
#define LEFT 1
#define RIGHT 2
// List of sequential turns for the robot to make in order!
// This is what you'll need to update on competition day :)
uint8_t turns[] = { LEFT, RIGHT, LEFT };
uint8_t turnIndex = 0;
uint8_t turnTotal = sizeof(turns);

// Configuration values that we can change to change robot's behavior
// The distance threshold the robot must reach to execute a turn from the list (mesaured in cm)
#define TURN_THRESHOLD 25
// The amount of times the ultrasonic sensor is measured per cycle and averaged (due to interference)
#define ULTRASONIC_AVG 10

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
#define SERVO_OFFSET -9 // The value at which to offset the servo's position (telling the servo 90 degrees is usually not perfectly centered)

// Define servo object
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
long ultrasonic_measure() {
  // We use a for loop to measure the distance multiple times because the sensor is subject to interference
  long total_dist = 0;
  for (uint8_t i = 0; i < ULTRASONIC_AVG; i++) {
    long echo_dist;
    digitalWrite(PIN_TRIG,LOW);
    delayMicroseconds(5);                                                                              
    digitalWrite(PIN_TRIG,HIGH);
    delayMicroseconds(15);
    digitalWrite(PIN_TRIG,LOW);
    echo_dist=pulseIn(PIN_ECHO,HIGH);
    echo_dist *= 0.01657;
    total_dist += round(echo_dist);
  }
  // Find the average using the amount of times we ran a measurement and return the final output
  Serial.println(total_dist / ULTRASONIC_AVG);
  return (total_dist / ULTRASONIC_AVG);
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
}

void loop() {;
  if (ultrasonic_measure() < TURN_THRESHOLD) {
    Serial.println("Wall detected!");
    // If the measured distance is less than the turn threshold, it's time to execute a turn
    // drivetrain_setSpeed(175, 175);
    drivetrain_setDir(FORWARD, turns[turnIndex]);
    if (turns[turnIndex] == LEFT) {
      servo.write(135 + SERVO_OFFSET);
      Serial.println("Turning LEFT");      
    } else if (turns[turnIndex] == RIGHT) {
      servo.write(45 + SERVO_OFFSET);
      Serial.println("Turning RIGHT");
    } else {
      // Otherwise, we must have hit the end of our turning list, so stop
      drivetrain_setSpeed(0, 0);
      while (true) { }
    }
    delay(800);
    turnIndex++;
  }
  // Otherwise, continue straight at full speed
  drivetrain_setDir(FORWARD, FORWARD);
  drivetrain_setSpeed(255, 255);
  servo.write(90 + SERVO_OFFSET);
}
