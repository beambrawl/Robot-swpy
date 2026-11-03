void setupSensors() {
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
}

void calibrateSensors() {
  // เก็บค่าเซนเซอร์หลายครั้งเพื่อให้ครอบคลุมขาว/ดำ
  for (uint16_t i = 0; i < 50; i++) {
    qtr.calibrate();
    delay(20);
  }
}

inline uint16_t getLinePosition() {
  return qtr.readLineBlack(sensorValues);
}

inline uint8_t getBlackCount() {
  uint8_t blackCount = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    blackCount += (sensorValues[i] > DARK_THRESHOLD);
  }
  return blackCount;
}

inline bool isCenterOnLine() {
  return (sensorValues[3] > DARK_THRESHOLD) || (sensorValues[4] > DARK_THRESHOLD);
}
