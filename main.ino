#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <BluetoothSerial.h>

// Motor pins
#define RIN1 33  // Right motor direction pin 1
#define RIN2 25  // Right motor direction pin 2
#define RPWM 32  // Right motor PWM pin
#define LIN1 22  // Left motor direction pin 1
#define LIN2 21  // Left motor direction pin 2
#define LPWM 23  // Left motor PWM pin
#define RBUT 35  // Right button pin
#define LBUT 34  // Left button pin


uint8_t array_size = 8;
uint16_t sensor_values[array_size];

// PID calculation variables
int last_error = 0;                              // Last error value for PID calculation
int P = 0, I = 0, D = 0;                         // PID terms
float Kp = 4.315, Ki = 0.0, Kd = 50;             // PID constants
int center_position = 3500;                      // Default center position
uint8_t MAX_SPEED_R = 150, MAX_SPEED_L = 150;    // Maximum speed for motors
uint8_t BASE_SPEED_R = 130, BASE_SPEED_L = 130;  // Base speed for motors

// Create QTR object for IR
QTRSensors qtr;

// Create Bluetooth serial object
BluetoothSerial SerialBT;

// Create motor objects
Motor motor_r = Motor(RIN1, RIN2, RPWM, 1, 99);  // Right motor
Motor motor_l = Motor(LIN1, LIN2, LPWM, 1, 99);  // Left motor

/**
 * @brief Sets the speed for both motors.
 * 
 * @param set_speed_r Speed to set for the right motor (0-255).
 * @param set_speed_l Speed to set for the left motor (0-255).
 * @references SparkFun_TB6612::drive
 */
void set_motor_speed(int set_speed_r, int set_speed_l) {
  motor_r.drive(set_speed_r);
  motor_l.drive(set_speed_l);
}

/**
 * @brief Calculates the PID control and adjusts motor speeds accordingly.
 * 
 * Reads the line position from QTRSensors, computes the PID values, and adjusts
 * the speeds of the motors to keep the robot centered on the line.
 * 
 * @references QTRSensors::readLineBlack, constrain
 */
void pid_control() {
  // Calculate error
  int current_position = qtr.readLineBlack(sensor_values);
  int current_error = center_position - current_position;

  // Calculate PID values
  P = current_error;
  I += current_error;
  D = current_error - last_error;
  last_error = current_error;

  int PID_CONTROLLED_SPEED = P * Kp + I * Ki + D * Kd;

  // Compute final speeds
  int FINAL_SPEED_R = BASE_SPEED_R + PID_CONTROLLED_SPEED;
  int FINAL_SPEED_L = BASE_SPEED_L - PID_CONTROLLED_SPEED;

  // Limit motor speeds
  FINAL_SPEED_R = constrain(FINAL_SPEED_R, -15000, 15000);
  FINAL_SPEED_L = constrain(FINAL_SPEED_L, -15000, 15000);

  FINAL_SPEED_R = map(FINAL_SPEED_R, -15000, 15000, 0, MAX_SPEED_R);
  FINAL_SPEED_L = map(FINAL_SPEED_L, -15000, 15000, 0, MAX_SPEED_L);

  // Set motor speeds
  set_motor_speed(FINAL_SPEED_R, FINAL_SPEED_L);
}

/**
 * @brief Initializes the hardware and performs calibration.
 * 
 * Sets up the pins for motors, IR sensors, and buttons. Also calibrates the
 * IR sensors by running the motors in different directions.
 * 
 * @references QTRSensors::setTypeRC, QTRSensors::setSensorPins, Motor::brake
 */
void setup() {
  // Setup IR sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 17, 5, 26, 27, 14, 13, 4, 16 }, array_size);

  // Start Serial monitor and Bluetooth serial
  Serial.begin(9600);
  SerialBT.begin("ESP32-A");

  // Setup Motor pins
  pinMode(RIN1, OUTPUT);  // Right motor direction pin 1
  pinMode(RIN2, OUTPUT);  // Right motor direction pin 2
  pinMode(RPWM, OUTPUT);  // Right motor PWM pin
  pinMode(LIN1, OUTPUT);  // Left motor direction pin 1
  pinMode(LIN2, OUTPUT);  // Left motor direction pin 2
  pinMode(LPWM, OUTPUT);  // Left motor PWM pin

  // Initialize motors but stop initially
  motor_r.brake();
  motor_l.brake();
  delay(2);

  // Setup Buttons
  pinMode(RBUT, INPUT);  // Right button
  pinMode(LBUT, INPUT);  // Left button

  // Calibrate IR sensors
  delay(300);
  for (int i = 0; i < 100; i++) {
    set_motor_speed(100, -100);  // Clockwise calibration
    qtr.calibrate();
  }

  for (int i = 0; i < 100; i++) {
    set_motor_speed(-100, 100);  // Anticlockwise calibration
    qtr.calibrate();
  }
}

/**
 * @brief Handles Bluetooth commands to update PID variables.
 * 
 * Parses the command received via Bluetooth and updates the corresponding
 * PID variable based on the command received.
 * 
 * @param command The command received via Bluetooth.
 */
void bluetooth_command_handler(String command) {
  command.trim();  // Remove any leading or trailing whitespace

  if (command.startsWith("kp ")) {
    float value = command.substring(3).toFloat();
    Kp = value;
    SerialBT.print("Kp updated to: ");
    SerialBT.println(Kp);
  } else if (command.startsWith("ki ")) {
    float value = command.substring(3).toFloat();
    Ki = value;
    SerialBT.print("Ki updated to: ");
    SerialBT.println(Ki);
  } else if (command.startsWith("kd ")) {
    float value = command.substring(3).toFloat();
    Kd = value;
    SerialBT.print("Kd updated to: ");
    SerialBT.println(Kd);
  } else if (command.startsWith("br ")) {
    float value = command.substring(3).toFloat();
    BASE_SPEED_R = value;
    SerialBT.print("BASE_SPEED_R updated to: ");
    SerialBT.println(BASE_SPEED_R);
  } else if (command.startsWith("bl ")) {
    float value = command.substring(3).toFloat();
    BASE_SPEED_L = value;
    SerialBT.print("BASE_SPEED_L updated to: ");
    SerialBT.println(BASE_SPEED_L);
  } else if (command.startsWith("mr ")) {
    float value = command.substring(3).toFloat();
    MAX_SPEED_R = value;
    SerialBT.print("MAX_SPEED_R updated to: ");
    SerialBT.println(MAX_SPEED_R);
  } else if (command.startsWith("ml ")) {
    float value = command.substring(3).toFloat();
    MAX_SPEED_L = value;
    SerialBT.print("MAX_SPEED_L updated to: ");
    SerialBT.println(MAX_SPEED_L);
  } else {
    SerialBT.println("Unknown command.");
  }
}

/**
 * @brief Main loop function to continuously run PID control.
 * 
 * This function is called repeatedly and performs PID control to keep the robot
 * centered on the line.
 * 
 * @references pid_control
 */
void loop() {
  int current_position = qtr.readLineBlack(sensor_values);
  if (current_position == 0) {
    motor_r.drive(50);
    motor_l.drive(-50);

  } else if (current_position == 7000) {
    motor_r.drive(-50);
    motor_l.drive(50);

  }

  else {
    pid_control();
  }


  // Check for Bluetooth serial commands
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    bluetooth_command_handler(command);
  }
}
