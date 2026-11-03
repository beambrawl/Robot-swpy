//v3: High-speed continuous PID + Time-Based Profile (เวลาตั้งเป็นทศนิยมวินาทีได้)
#include "01_Settings.h"
#include <QTRSensors.h>

QTRSensors qtr;
uint16_t sensorValues[8];

void waitButton() {
  while (digitalRead(BUTTON_PIN) == HIGH) { }
  delay(30);
  while (digitalRead(BUTTON_PIN) == LOW) { }
  delay(80);
}

void twitchMotor() {
  moveMotor(twitchSpeed, twitchSpeed);
  delay(120);
  moveMotor(0, 0);
  delay(120);
}

void setup() {
  setupMotors();
  setupSensors();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  waitButton();
  twitchMotor();

  calibrateSensors();

  moveMotor(0, 0);
  twitchMotor();
}

void loop() {
  waitButton();      // กดแล้วปล่อย = ปล่อยรถ
  startRun();        // เริ่มจับเวลา profile + รีเซ็ตสถานะ PID

  while (digitalRead(BUTTON_PIN) == HIGH) {
    runPID();
  }

  moveMotor(0, 0);
  delay(500);
}
