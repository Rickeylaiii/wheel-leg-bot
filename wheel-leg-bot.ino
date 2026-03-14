#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

// --- ADC ---
int BAT_PIN = 35;
static esp_adc_cal_characteristics_t adc_chars;
#define LED_BAT 13

// --- I2C & IMU ---
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MPU6050 mpu6050(I2Ctwo);

// --- Servos ---
SMS_STS sms_sts;

// --- Motors & Encoders ---
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26,27,14,12);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

int current_test = 0;
unsigned long last_print_time = 0;
bool sensors_initialized = false;
bool motors_initialized = false;
bool foc_ready = false;
float battery_filtered = 0.0f;
int motor_test_stage = 0;

void printMenu() {
  Serial.println("\n===============================");
  Serial.println(" WL-PRO Hardware Test Menu");
  Serial.println("===============================");
  Serial.println("Send a number via Serial Monitor:");
  Serial.println(" 0: Stop all tests");
  Serial.println(" 1: Battery Voltage (filtered)");
  Serial.println(" 2: IMU (angle + raw accel/gyro)");
  Serial.println(" 3: Servos (Legs slowly Up/Down)");
  Serial.println(" 4: Encoders (AS5600 angle + velocity)");
  Serial.println(" 5: Motors FOC Basic (torque-voltage)");
  Serial.println(" 6: Motors FOC Calibration Only");
  Serial.println(" 7: Motors Position Closed-loop (angle)");
  Serial.println(" 8: Motors Speed Closed-loop (velocity)");
  Serial.println(" 9: Motors Torque Sweep (M1/M2 independent)");
}

void initSensors() {
    if (sensors_initialized) return;
    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);
    sensors_initialized = true;
    Serial.println("Encoders initialized.");
}

void configureTorqueMode() {
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;
}

void configureAngleMode() {
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::angle;
    motor2.controller = MotionControlType::angle;
}

void configureVelocityMode() {
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::velocity;
    motor2.controller = MotionControlType::velocity;
}

bool runFOCCalibration() {
    if (!motors_initialized) {
        Serial.println("Motor hardware is not initialized.");
        return false;
    }
    Serial.println("Starting FOC calibration...");
    Serial.println("WARNING: Wheels will spin slightly to align sensors!");
    int ret1 = motor1.initFOC();
    int ret2 = motor2.initFOC();
    foc_ready = (ret1 == 1 && ret2 == 1);
    Serial.printf("FOC Result -> M1:%d M2:%d | Ready:%s\n", ret1, ret2, foc_ready ? "YES" : "NO");
    return foc_ready;
}

void initMotors() {
    if(motors_initialized) return;
    Serial.println("Initializing Motors & Encoders...");
    initSensors();

    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    driver1.voltage_power_supply = 8;
    driver2.voltage_power_supply = 8;
    driver1.init();
    driver2.init();
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    // Initial safe voltage and limits
    motor1.voltage_sensor_align = 6;
    motor2.voltage_sensor_align = 6;
    motor1.voltage_limit = 2.5;
    motor2.voltage_limit = 2.5;
    motor1.velocity_limit = 30;
    motor2.velocity_limit = 30;

    motor1.PID_velocity.P = 0.2;
    motor1.PID_velocity.I = 2.0;
    motor1.PID_velocity.D = 0.0;
    motor2.PID_velocity.P = 0.2;
    motor2.PID_velocity.I = 2.0;
    motor2.PID_velocity.D = 0.0;

    motor1.P_angle.P = 15.0;
    motor2.P_angle.P = 15.0;

    configureTorqueMode();

    motor1.init();
    motor2.init();

    motors_initialized = true;
    Serial.println("Motors Initialized. Run test 6 to calibrate FOC.");
}

void stopMotors() {
    if (!motors_initialized) return;
    configureTorqueMode();
    motor1.target = 0;
    motor2.target = 0;
    motor1.move();
    motor2.move();
}

void enterTest(int test_id) {
    current_test = test_id;
    last_print_time = 0;
    motor_test_stage = 0;

    Serial.print("\n>>> Starting Test ");
    Serial.print(current_test);
    Serial.println(" <<<");

    if (current_test == 0) {
        stopMotors();
        Serial.println("All tests stopped.");
        return;
    }

    if (current_test == 1) {
        Serial.println("Battery test started (with low-pass filter).");
    } else if (current_test == 2) {
        Serial.println("Calibrating IMU gyro offsets, please DO NOT move the robot...");
        mpu6050.calcGyroOffsets(true);
        Serial.println("IMU calibrated. Streaming angle + raw accel/gyro...");
    } else if (current_test == 3) {
        Serial.println("Servo test started. Note: current servo library supports write only, no readback.");
    } else if (current_test == 4) {
        initSensors();
        Serial.println("Encoder test started (no motor drive).");
    } else if (current_test == 5) {
        initMotors();
        if (!foc_ready) runFOCCalibration();
        configureTorqueMode();
        Serial.println("FOC basic torque-voltage test started.");
    } else if (current_test == 6) {
        initMotors();
        runFOCCalibration();
        Serial.println("FOC calibration done. Use 5/7/8/9 for closed-loop operation tests.");
    } else if (current_test == 7) {
        initMotors();
        if (!foc_ready) runFOCCalibration();
        configureAngleMode();
        Serial.println("Position closed-loop test started (sinusoidal target)." );
    } else if (current_test == 8) {
        initMotors();
        if (!foc_ready) runFOCCalibration();
        configureVelocityMode();
        Serial.println("Speed closed-loop test started (sinusoidal target)." );
    } else if (current_test == 9) {
        initMotors();
        if (!foc_ready) runFOCCalibration();
        configureTorqueMode();
        Serial.println("Torque sweep test started (M1/M2 independent stages)." );
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  printMenu();
  
  pinMode(LED_BAT, OUTPUT);
  
  I2Cone.begin(19, 18, 400000UL); 
  I2Ctwo.begin(23, 5, 400000UL); 
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  Serial2.begin(1000000);
  sms_sts.pSerial = &Serial2;

  mpu6050.begin();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
        if (c == 'h' || c == 'H') {
            printMenu();
        } else if (c >= '0' && c <= '9') {
            enterTest(c - '0');
    }
  }

  // --- Test 1: Battery ---
  if (current_test == 1) { 
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          uint32_t raw = analogRead(BAT_PIN);
          uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
          float battery = (voltage * 3.97) / 1000.0;
                    if (battery_filtered == 0.0f) battery_filtered = battery;
                    battery_filtered = 0.85f * battery_filtered + 0.15f * battery;
                    Serial.printf("Battery Raw: %.2f V | Filtered: %.2f V\n", battery, battery_filtered);
                    digitalWrite(LED_BAT, battery_filtered > 7.8 ? HIGH : LOW);
      }
  }
  // --- Test 2: IMU ---
  else if (current_test == 2) { 
      mpu6050.update();
      if (millis() - last_print_time > 200) {
          last_print_time = millis();
                    Serial.printf("Angle(XYZ): %.2f %.2f %.2f | Gyro(XYZ): %.2f %.2f %.2f | Acc(XYZ): %.2f %.2f %.2f\n",
                                                mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ(),
                                                mpu6050.getGyroX(),  mpu6050.getGyroY(),  mpu6050.getGyroZ(),
                                                mpu6050.getAccX(),   mpu6050.getAccY(),   mpu6050.getAccZ());
      }
  }
  // --- Test 3: Servos ---
  else if (current_test == 3) { 
      if (millis() - last_print_time > 2000) {
          last_print_time = millis();
          static bool up = true;
          byte ID[2] = {1, 2};
          s16 Position[2];
          u16 Speed[2] = {100, 100}; // very slow for safety
          byte ACC[2] = {20, 20};
          
          if (up) {
              Position[0] = 2100;
              Position[1] = 1900;
              Serial.println("Servos -> Moving UP");
          } else {
              Position[0] = 2300;
              Position[1] = 1700;
              Serial.println("Servos -> Moving DOWN");
          }
          sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
          up = !up;
      }
  }
  // --- Test 4: Encoders Only ---
  else if (current_test == 4) { 
      sensor1.update();
      sensor2.update();
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          Serial.printf("Encoder1 Angle: %.3f Vel: %.3f | Encoder2 Angle: %.3f Vel: %.3f\n", 
                        sensor1.getAngle(), sensor1.getVelocity(), sensor2.getAngle(), sensor2.getVelocity());
      }
  }
  // --- Test 5: Motors FOC Basic ---
  else if (current_test == 5) { 
      if (!foc_ready) return;
      motor1.loopFOC();
      motor2.loopFOC();
      
      motor1.target = 0.45;
      motor2.target = 0.45;
      motor1.move();
      motor2.move();
      
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          Serial.printf("Torque(V) Target=0.45 | M1 Angle: %.2f Vel: %.2f | M2 Angle: %.2f Vel: %.2f\n",
                        motor1.shaft_angle, motor1.shaft_velocity, motor2.shaft_angle, motor2.shaft_velocity);
      }
  }
  // --- Test 6: FOC Calibration Status ---
  else if (current_test == 6) {
      if (millis() - last_print_time > 1000) {
          last_print_time = millis();
          Serial.printf("FOC Ready: %s (press 6 again if you need recalibration)\n", foc_ready ? "YES" : "NO");
      }
  }
  // --- Test 7: Position Closed-loop ---
  else if (current_test == 7) {
      if (!foc_ready) return;
      motor1.loopFOC();
      motor2.loopFOC();

      float t = millis() * 0.001f;
      float target_angle = 0.8f * sinf(0.8f * t);
      motor1.target = target_angle;
      motor2.target = -target_angle;
      motor1.move();
      motor2.move();

      if (millis() - last_print_time > 300) {
          last_print_time = millis();
          Serial.printf("Angle Target: %.2f | M1: %.2f | M2: %.2f\n", target_angle, motor1.shaft_angle, motor2.shaft_angle);
      }
  }
  // --- Test 8: Velocity Closed-loop ---
  else if (current_test == 8) {
      if (!foc_ready) return;
      motor1.loopFOC();
      motor2.loopFOC();

      float t = millis() * 0.001f;
      float target_vel = 8.0f * sinf(0.6f * t);
      motor1.target = target_vel;
      motor2.target = -target_vel;
      motor1.move();
      motor2.move();

      if (millis() - last_print_time > 300) {
          last_print_time = millis();
          Serial.printf("Velocity Target: %.2f | M1: %.2f | M2: %.2f\n", target_vel, motor1.shaft_velocity, motor2.shaft_velocity);
      }
  }
  // --- Test 9: Torque Sweep + Independent Motor Test ---
  else if (current_test == 9) {
      if (!foc_ready) return;
      motor1.loopFOC();
      motor2.loopFOC();

      unsigned long sec = millis() / 1000;
      motor_test_stage = sec % 4;
      if (motor_test_stage == 0) {
          motor1.target = 0.45f; motor2.target = 0.0f;
      } else if (motor_test_stage == 1) {
          motor1.target = -0.45f; motor2.target = 0.0f;
      } else if (motor_test_stage == 2) {
          motor1.target = 0.0f; motor2.target = 0.45f;
      } else {
          motor1.target = 0.0f; motor2.target = -0.45f;
      }

      motor1.move();
      motor2.move();

      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          Serial.printf("Stage:%d | M1 Target:%.2f Vel:%.2f | M2 Target:%.2f Vel:%.2f\n",
                        motor_test_stage, motor1.target, motor1.shaft_velocity, motor2.target, motor2.shaft_velocity);
      }
  }
}
