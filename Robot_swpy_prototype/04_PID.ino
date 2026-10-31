static int16_t lastError = 0;
static int16_t dFiltered = 0;

static uint8_t crossConfirm = 0;
static uint8_t lostConfirm  = 0;
static uint32_t crossUntilMs = 0;

// Gain scheduling memory
static bool cornerMode = false;
static uint8_t cornerEnterConfirm = 0;
static uint8_t cornerExitConfirm  = 0;
static int16_t kpNow = KpNormal;
static int16_t kdNow = KdNormal;

static inline int16_t iabs16(int16_t x) {
  return (x < 0) ? -x : x;
}

static inline int16_t clamp16(int32_t v) {
  if (v > maxSpeed)  return maxSpeed;
  if (v < -maxSpeed) return -maxSpeed;
  return (int16_t)v;
}

static inline uint8_t chooseBaseSpeed(int16_t absError) {
  // ความเร็วตามความโค้ง: ยิ่ง error มาก ยิ่งช้าลง
  if (cornerMode) {
    if (absError > 2600) return turnSpeed;
    if (absError > 1800) return 138;//118
    if (absError > 1200) return 160;//130
    if (absError > 700)  return 180;//140
    return baseSpeed > 10 ? (uint8_t)(baseSpeed - 10) : baseSpeed;
  }

  if (absError > 2600) return turnSpeed;
  if (absError > 1800) return 132;
  if (absError > 1100) return 146;
  return baseSpeed;
}

static inline void updatePIDProfile(int16_t error, uint8_t blackCount) {
  const int16_t absError = iabs16(error);

  // เข้าสู่ corner mode เมื่อต่อเนื่องว่า error สูงจริง
  const bool enterCandidate = (absError >= CORNER_ENTER_ERROR) ||
                              (blackCount >= 2 && absError >= (CORNER_ENTER_ERROR - 300));

  if (!cornerMode) {
    if (enterCandidate) {
      if (++cornerEnterConfirm >= CORNER_ENTER_LOOPS) {
        cornerMode = true;
        cornerEnterConfirm = 0;
        cornerExitConfirm = 0;
      }
    } else {
      cornerEnterConfirm = 0;
    }
  } else {
    // ออกจาก corner mode เมื่อกลับมาใกล้เส้นจริง
    const bool exitCandidate = (absError <= CORNER_EXIT_ERROR && blackCount >= 1);

    if (exitCandidate) {
      if (++cornerExitConfirm >= CORNER_EXIT_LOOPS) {
        cornerMode = false;
        cornerExitConfirm = 0;
        cornerEnterConfirm = 0;
      }
    } else {
      cornerExitConfirm = 0;
    }
  }

  if (cornerMode) {
    kpNow = KpCorner;
    kdNow = KdCorner;
  } else {
    kpNow = KpNormal;
    kdNow = KdNormal;
  }
}

void runPID() {
  const uint16_t position = getLinePosition();

  // ---------------------------
  // นับเซนเซอร์ดำ
  // ---------------------------
  const uint8_t blackCount = getBlackCount();

  // ---------------------------
  // Crossroad lock: ดันตรงผ่านจุดตัด
  // ---------------------------
  if (crossUntilMs && (int32_t)(millis() - crossUntilMs) < 0) {
    moveMotor(crossSpeed, crossSpeed);
    return;
  }
  if (crossUntilMs) {
    crossUntilMs = 0;
  }

  if (blackCount >= CROSS_DARK_COUNT) {
    if (++crossConfirm >= CROSS_CONFIRM_LOOPS) {
      crossConfirm = 0;
      crossUntilMs = millis() + CROSS_HOLD_MS;
      moveMotor(crossSpeed, crossSpeed);
      return;
    }
  } else {
    crossConfirm = 0;
  }

  // ---------------------------
  // Lost-line recovery
  // ---------------------------
  if (blackCount == 0) {
    if (++lostConfirm >= LOST_CONFIRM_LOOPS) {
      // หมุนหาเส้นตามทิศทาง error ล่าสุด
      cornerMode = false;
      cornerEnterConfirm = 0;
      cornerExitConfirm = 0;

      if (lastError >= 0) {
        moveMotor(searchSpeed, -searchSpeed);
      } else {
        moveMotor(-searchSpeed, searchSpeed);
      }
      return;
    }
  } else {
    lostConfirm = 0;
  }

  // ---------------------------
  // Fixed-point PID
  // ---------------------------
  const int16_t error = (int16_t)position - (LINE_CENTER + CENTER_OFFSET);

  const int16_t dRaw = error - lastError;
  dFiltered += (dRaw - dFiltered) >> 2;   // low-pass derivative แบบเบาๆ
  lastError = error;

  updatePIDProfile(error, blackCount);

  const int32_t correction = ((int32_t)error * kpNow + (int32_t)dFiltered * kdNow) / PID_SCALE;

  const int16_t absError = iabs16(error);
  const uint8_t driveBase = chooseBaseSpeed(absError);

  int16_t leftSpeed  = (int16_t)driveBase + (int16_t)correction;
  int16_t rightSpeed = (int16_t)driveBase - (int16_t)correction;

  // trim ซ้าย-ขวา เพื่อบาลานซ์มอเตอร์
  leftSpeed  += LEFT_TRIM;
  rightSpeed += RIGHT_TRIM;

  moveMotor(clamp16(leftSpeed), clamp16(rightSpeed));
}
