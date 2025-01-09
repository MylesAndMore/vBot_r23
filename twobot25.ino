/**
 * twobot25.ino
 * Last updated 01.2025
 * Contributed to by: MylesAndMore (Myles Vendel)
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

template<typename T1, typename T2>
struct Pair {
    T1 first;
    T2 second;
    Pair(T1 f, T2 s) : first(f), second(s) {}
};

typedef enum Direction {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  END,
} Direction;

// List of sequential distance + turn pairs for the robot to make
// The robot will first go the distance (in mm), followed by making the specified turn, and then proceed to the next pair
Pair<int, Direction> turns[] = {
  {500, RIGHT},
  {500, LEFT},
  {500, LEFT},
  {500, RIGHT},
  {500, BACKWARD},
  {1000, END}, // The sequence must terminate with an END turn
};
#define NUM_TURNS (sizeof(turns) / sizeof(turns[0]))

/* - Drivetrain - */

#define PIN_SPEED_R 5
#define PIN_DIR_R1 7
#define PIN_DIR_R2 8
#define PIN_SPEED_L 6
#define PIN_DIR_L1 9
#define PIN_DIR_L2 10

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
 * Directly sets the speed of the drivetrain (all motors).
 * @param speedL the speed value (0 to 255) to use on the left
 * @param speedR the speed value (0 to 255) to use on the right
 */
void drivetrain_setSpeed(uint8_t left, uint8_t right) {
  analogWrite(PIN_SPEED_L, left); 
  analogWrite(PIN_SPEED_R, right); 
}

/**
 * Initializes the drivetrain (sets up pins).
 */
void drivetrain_init() {
  pinMode(PIN_SPEED_R, OUTPUT);
  pinMode(PIN_DIR_R1, OUTPUT);
  pinMode(PIN_DIR_R2, OUTPUT);
  pinMode(PIN_SPEED_L, OUTPUT);
  pinMode(PIN_DIR_L1, OUTPUT);
  pinMode(PIN_DIR_L2, OUTPUT);
}

/**
 * Sets both speed and direction of the drivetrain.
 * @param speedL the speed value (-255 to 255) to use on the left
 * @param speedR the speed value (-255 to 255) to use on the right
 */
void drivetrain_drive(int speedL, int speedR) {
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

/* - Encoder - */

#define PIN_ENCODER 2
#define ENCODER_CPR 10 // The amount of times the encoder will trigger during one full revolution of the wheel
#define WHEEL_DIAMETER 60 // The OUTER diameter of the wheel (rubber part)
#define ENCODER_OFFSET 5 // A turn will be executed this many mm early as decelerations can take a bit of time
int64_t encoder_pulses;
float encoder_dist;
bool encoder_active;

// Should be called on an encoder event.
static void encoder_onTrigger() {
  if (!encoder_active) {
    return;
  }
  encoder_pulses++;
  encoder_dist = (float)encoder_pulses / ENCODER_CPR * M_PI * WHEEL_DIAMETER;
}

/**
 * Initializes the encoder (binds interrupt).
 */
void encoder_init() {
  encoder_pulses = 0;
  encoder_dist = 0;
  encoder_active = true;
  pinMode(PIN_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoder_onTrigger, RISING);
}

/**
 * Resets the encoder distance to zero.
 */
void encoder_reset() {
  encoder_pulses = 0;
  encoder_dist = 0;
}

/* - IMU - */

// TODO: IMU communication can be unreliable at times and is the source of multiple issues (spinning in circles, missing turns)
// I believe this is due to a faulty implementation in the library/hardware issues with the Arduino UNO, so maybe trying a different library
// or using something like [https://www.amazon.com/APTINEX-Lakduino-RP2040-Development-Board/dp/B0D73KTY7J] would help?

#define PIN_SDA 4
#define PIN_SCL 5
#define FREQ_KHZ 100

MPU6050 mpu(Wire);

/**
 * Initializes communication with the IMU and performs a calibration.
 */
void imu_init() {
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(FREQ_KHZ * 1000);
  byte status = mpu.begin();
  if (status != 0)
    Serial.println("Error initializing IMU: " + status);
  mpu.calcOffsets();
}

/**
 * @return the current Z value of the IMU
 */
double imu_getZ() {
  mpu.update();
  return (double)mpu.getAngleZ();
}

/* - Main code - */

#define DRIVETRAIN_BASE_SPEED 128 // Base speed to use when driving (forward), when no adjustments are being made
// Turning/correcting PID constants + accompanying controller
#define DRIVETRAIN_KP 5
#define DRIVETRAIN_KI 0
#define DRIVETRAIN_KD 0.65
double pid_in, pid_out, pid_set;
PID pid(&pid_in, &pid_out, &pid_set, DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KD, DIRECT);

int current_turn = 0;

void setup() {
  Serial.begin(115200);
  drivetrain_init();
  encoder_init();
  imu_init();
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  // Compute deviation from desired to percieved degree value
  pid_in = -imu_getZ(); // IMU is mounted backwards, so invert value
  pid.Compute();
  // Apply control signal to drivetrain
  int adjustment = (int)pid_out;
  int speedL = constrain(DRIVETRAIN_BASE_SPEED - adjustment, -255, 255);
  int speedR = constrain(DRIVETRAIN_BASE_SPEED + adjustment, -255, 255);
  drivetrain_drive(speedL, speedR);
  // Turn if necessary
  if (encoder_dist >= turns[current_turn].first - ENCODER_OFFSET) {
    if (current_turn > NUM_TURNS) {
      drivetrain_drive(0, 0);
      while (true);
    }
    if (turns[current_turn].second == LEFT) {
      pid_set -= 90;
    } else if (turns[current_turn].second == RIGHT) {
      pid_set += 90;
    } else if (turns[current_turn].second == BACKWARD) {
      if (pid_set >= 180) {
        pid_set -= 180;
      } else {
        pid_set += 180;
      }
    }
    encoder_reset();
    current_turn++;
  }
  // Only update encoder if we aren't currently turning/correcting
  encoder_active = abs(pid_set - pid_in) < 5;
}
