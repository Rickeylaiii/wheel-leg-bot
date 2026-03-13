#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>
#include <Wire.h>

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
bool motors_initialized = false;

void initMotors() {
    if(motors_initialized) return;
    Serial.println("Initializing Motors & Encoders...");
    Serial.println("WARNING: Wheels will spin slightly to align sensors!");
    
    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    driver1.voltage_power_supply = 8;
    driver2.voltage_power_supply = 8;
    driver1.init();
    driver2.init();
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    // Initial safe voltage for alignment
    motor1.voltage_sensor_align = 6;
    motor2.voltage_sensor_align = 6;

    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;

    motor1.init();
    motor2.init();
    motor1.initFOC();
    motor2.initFOC();
    motors_initialized = true;
    Serial.println("Motors Initialized Successfully!");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===============================");
  Serial.println(" WL-PRO Hardware Test Menu");
  Serial.println("===============================");
  Serial.println("Send a number via Serial Monitor to test:");
  Serial.println(" 1: Battery Voltage");
  Serial.println(" 2: IMU (MPU6050)");
  Serial.println(" 3: Servos (Legs slowly Up/Down)");
  Serial.println(" 4: Encoders (AS5600 Reading)");
  Serial.println(" 5: Motors (Closed-loop spinning)");
  Serial.println(" 0: Stop all tests");
  
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
    if (c >= '0' && c <= '5') {
      current_test = c - '0';
      Serial.print("\n>>> Starting Test ");
      Serial.print(current_test);
      Serial.println(" <<<");
      
      // Stop motors if switching out of test 5
      if (motors_initialized && current_test != 5) {
          motor1.target = 0; motor2.target = 0;
          motor1.move(); motor2.move();
      }

      if (current_test == 0) {
          Serial.println("All tests stopped.");
      }
      else if (current_test == 2) {
          Serial.println("Calibrating IMU, please DO NOT move the robot...");
          mpu6050.calcGyroOffsets(true);
          Serial.println("IMU Calibrated. Printing data...");
      }
      else if (current_test == 4 || current_test == 5) {
          initMotors();
      }
    }
  }

  // --- Test 1: Battery ---
  if (current_test == 1) { 
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          uint32_t raw = analogRead(BAT_PIN);
          uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
          float battery = (voltage * 3.97) / 1000.0;
          Serial.printf("Battery: %.2f V\n", battery);
          digitalWrite(LED_BAT, battery > 7.8 ? HIGH : LOW);
      }
  }
  // --- Test 2: IMU ---
  else if (current_test == 2) { 
      mpu6050.update();
      if (millis() - last_print_time > 200) {
          last_print_time = millis();
          Serial.printf("Pitch(X): %.2f | Roll(Y): %.2f | Yaw(Z): %.2f\n", 
                        mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ());
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
              Position[0] = 2148;
              Position[1] = 1948;
              Serial.println("Servos -> Moving UP");
          } else {
              Position[0] = 2348;
              Position[1] = 1748;
              Serial.println("Servos -> Moving DOWN");
          }
          sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
          up = !up;
      }
  }
  // --- Test 4: Encoders Only ---
  else if (current_test == 4) { 
      motor1.loopFOC();
      motor2.loopFOC();
      motor1.move();
      motor2.move();
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          sensor1.update();
          sensor2.update();
          Serial.printf("Encoder 1 Angle: %.2f | Encoder 2 Angle: %.2f\n", 
                        sensor1.getAngle(), sensor2.getAngle());
      }
  }
  // --- Test 5: Motors Closed-loop ---
  else if (current_test == 5) { 
      motor1.loopFOC();
      motor2.loopFOC();
      
      motor1.target = 0.5; // very low voltage target (open loop V, closed loop phase)
      motor2.target = 0.5; 
      motor1.move();
      motor2.move();
      
      if (millis() - last_print_time > 500) {
          last_print_time = millis();
          Serial.printf("Motors Target=0.5V | M1 Vel: %.2f | M2 Vel: %.2f\n", 
                        motor1.shaft_velocity, motor2.shaft_velocity);
      }
  }
}
