static inline int16_t clampPWM(int32_t v) {
  if (v > maxSpeed)  return maxSpeed;
  if (v < -maxSpeed) return -maxSpeed;
  return (int16_t)v;
}

void setupMotors() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // D2, D3, D4, D7 as direct direction outputs
  DDRD |= _BV(DDD2) | _BV(DDD3) | _BV(DDD4) | _BV(DDD7);

  // Safe start: brake + zero PWM
  PORTD |= _BV(PORTD2) | _BV(PORTD3) | _BV(PORTD4) | _BV(PORTD7);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

inline void moveMotor(int16_t speedLeft, int16_t speedRight) {
  speedLeft  = clampPWM(speedLeft);
  speedRight = clampPWM(speedRight);

  // Left motor: AIN1=D2, AIN2=D3, PWM=D5
  if (speedLeft > 0) {
    PORTD |=  _BV(PORTD2);
    PORTD &= ~_BV(PORTD3);
    analogWrite(PWMA, (uint8_t)speedLeft);
  } else if (speedLeft < 0) {
    PORTD &= ~_BV(PORTD2);
    PORTD |=  _BV(PORTD3);
    analogWrite(PWMA, (uint8_t)(-speedLeft));
  } else {
    PORTD |=  _BV(PORTD2) | _BV(PORTD3);  // brake
    analogWrite(PWMA, 0);
  }

  // Right motor: BIN1=D4, BIN2=D7, PWM=D6
  if (speedRight > 0) {
    PORTD |=  _BV(PORTD4);
    PORTD &= ~_BV(PORTD7);
    analogWrite(PWMB, (uint8_t)speedRight);
  } else if (speedRight < 0) {
    PORTD &= ~_BV(PORTD4);
    PORTD |=  _BV(PORTD7);
    analogWrite(PWMB, (uint8_t)(-speedRight));
  } else {
    PORTD |=  _BV(PORTD4) | _BV(PORTD7);  // brake
    analogWrite(PWMB, 0);
  }
}