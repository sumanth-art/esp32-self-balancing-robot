#include <MPU6050_light.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <AccelStepper.h>

// === Motor Pins ===
#define STEP_PIN1 18
#define DIR_PIN1  19
#define STEP_PIN2 25
#define DIR_PIN2  26

// === I2C Pins ===
#define MPU_SDA 21
#define MPU_SCL 22

BluetoothSerial SerialBT;
MPU6050 mpu(Wire);
Preferences preferences;

// === Stepper Motors using AccelStepper ===
AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// === PID Variables ===
float Kp_angle, Ki_speed, Kd_angle;
float totalSpeedError = 0;
float fusedAngle = 0, previousAngle = 0;
float targetAngle = 7.0;  // Calibrated upright angle
int max_output = 400;

// === Flags and Manual Control ===
bool ARMED = false;
bool AUTO_RISE = false;
bool POS_HOLD = false;
float manualOffset = 0.0;
float targetManualOffset = 0.0;
int turnOffset = 0;

void loadPID() {
  preferences.begin("pid", true);
  Kp_angle = preferences.getFloat("kp_a", 28.0);
  Ki_speed = preferences.getFloat("ki_s", 0.009);
  Kd_angle = preferences.getFloat("kd_a", 1.65);
  targetAngle = preferences.getFloat("target", 7.0);  // ← Default upright angle
  preferences.end();
}

void savePID() {
  preferences.begin("pid", false);
  preferences.putFloat("kp_a", Kp_angle);
  preferences.putFloat("ki_s", Ki_speed);
  preferences.putFloat("kd_a", Kd_angle);
  preferences.end();
}

void updateIMU() {
  mpu.update();
  float accAngle = mpu.getAngleY();      // or X if your sensor is aligned differently
  float gyroRate = mpu.getGyroY();
  float dt = 0.01;

  fusedAngle = 0.98 * (previousAngle + gyroRate * dt) + 0.02 * accAngle;
  previousAngle = fusedAngle;

  // === Fall detection with 500ms delay ===
  static unsigned long fallTimer = 0;
  
  if (fusedAngle > 60 || fusedAngle < -60){
    if (fallTimer == 0) fallTimer = millis();
    if (millis() - fallTimer > 500 && ARMED) {
      ARMED = false;
      SerialBT.println("⚠️ Fall detected! Motors disabled.");
    }
  } else {
    fallTimer = 0;
  }
}
void computePID() {
  // Smoothly transition to the target manual offset
  manualOffset += (targetManualOffset - manualOffset) * 0.1;

  float angleError = fusedAngle - targetAngle - manualOffset;

  // --- PID Terms ---
  float angleOutput = Kp_angle * angleError + Kd_angle * (fusedAngle - previousAngle);

  // Use tilt error for integral term 
  if (abs(angleError) > 0.5) {
  totalSpeedError += angleError * 0.01;
  } else {
    totalSpeedError *= 0.99;  // slow decay when nearly upright
  }
  totalSpeedError = constrain(totalSpeedError, -60, 60);
  
  float speedOutput = Ki_speed * totalSpeedError;

  // --- Final PID Output ---
  int output = angleOutput + (POS_HOLD ? speedOutput : 0);
  int outputLeft = constrain(output - turnOffset, -max_output, max_output);
  int outputRight = constrain(output + turnOffset, -max_output, max_output);

  // --- Apply motor speeds ---
  if (ARMED) {
    motor1.setSpeed(-outputLeft);  // Reverse if needed for direction
    motor2.setSpeed(outputRight);
  } else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  }

  // --- Run steppers ---
  motor1.runSpeed();
  motor2.runSpeed();

  previousAngle = fusedAngle;
}
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "ARM" || cmd == "START") {
    ARMED = true;
    SerialBT.println("Robot armed.");
  } 
  else if (cmd == "DISARM" || cmd == "STOP") {
    ARMED = false;
    manualOffset = 0;
    turnOffset = 0;
    SerialBT.println("Robot disarmed or stopped.");
  } 
  else if (cmd == "F") {
    targetManualOffset = 2.0;
    turnOffset = 0;
    SerialBT.println("Moving forward.");
  } 
  else if (cmd == "B") {
    targetManualOffset = -2.5;
    turnOffset = 0;
    SerialBT.println("Moving backward.");
  } 
  else if (cmd == "L") {
    turnOffset = -10;
    SerialBT.println("Turning left.");
  } 
  else if (cmd == "R") {
    turnOffset = 10;
    SerialBT.println("Turning right.");
  } 
  else if (cmd == "S") {
    targetManualOffset = 0;
    turnOffset = 0;
    SerialBT.println("Manual control stopped.");
  } 
  else if (cmd.startsWith("AUTO_RISE ")) {
    AUTO_RISE = cmd.endsWith("ON");
    SerialBT.printf("AUTO_RISE %s\n", AUTO_RISE ? "ENABLED" : "DISABLED");
  } 
  else if (cmd.startsWith("POS_HOLD ")) {
    POS_HOLD = cmd.endsWith("ON");
    SerialBT.printf("POS_HOLD %s\n", POS_HOLD ? "ENABLED" : "DISABLED");
  } 
  else if (cmd.startsWith("PID")) {
    float kp, ki, kd;
    int found = sscanf(cmd.c_str(), "PID %f %f %f", &kp, &ki, &kd);
    if (found == 3) {
      Kp_angle = kp;
      Ki_speed = ki;
      Kd_angle = kd;
      savePID();
      SerialBT.printf("PID set to: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", kp, ki, kd);
    } else {
      SerialBT.println("Invalid SET_PID format. Use: PID 14.0 0.02 1.2");
    }
  } 
  else if (cmd.startsWith("T")) {
    float newTarget = cmd.substring(1).toFloat();
    if (newTarget >= 0.0 && newTarget <= 10.0) {
      targetAngle = newTarget;
      preferences.begin("pid", false);
      preferences.putFloat("target", targetAngle);
      preferences.end();
      SerialBT.printf("✅ Target angle set to %.2f° and saved\n", targetAngle);
    } else {
      SerialBT.println("⚠️ Invalid target angle. Use between 0.0 to 10.0");
    }
  }
  else if (cmd == "STATUS") {
    SerialBT.printf("Tilt: %.2f deg\n", fusedAngle);
    SerialBT.printf("PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", Kp_angle, Ki_speed, Kd_angle);
    SerialBT.printf("Target=%.2f\n", targetAngle);
    SerialBT.printf("ARMED=%s, AUTO_RISE=%s, POS_HOLD=%s\n", ARMED ? "YES" : "NO", AUTO_RISE ? "YES" : "NO", POS_HOLD ? "YES" : "NO");
  } 
  else {
    SerialBT.println("Unknown command.");
  }
}
void checkBluetoothCommand() {
  static String input = "";
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL);
  SerialBT.begin("ESP32_BalanceBot");

  motor1.setMaxSpeed(600);
  motor1.setAcceleration(300);
  motor2.setMaxSpeed(600);
  motor2.setAcceleration(300);

  mpu.begin();
  loadPID();

  Serial.println("BalanceBot Ready.");
  SerialBT.println("Connected to ESP32_BalanceBot. Type STATUS or PID command.");
}
void loop() {
  updateIMU();
  computePID();
  checkBluetoothCommand();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    SerialBT.printf("[DEBUG] Tilt=%.2f  Kp=%.2f  Ki=%.3f  Kd=%.2f\n", fusedAngle, Kp_angle, Ki_speed, Kd_angle);
  }
}
