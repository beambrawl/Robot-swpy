static int16_t lastError = 0;
static int16_t dFiltered = 0;

static uint8_t lostConfirm  = 0;

// Corner mode (ของเดิม)
static bool cornerMode = false;
static uint8_t cornerEnterConfirm = 0;
static uint8_t cornerExitConfirm  = 0;

// ---------------------------
// Time-based profile state
// ---------------------------
static ProfileParams curP;            // ค่าที่ใช้อยู่จริงตอนนี้ (หลังไล่ค่าแล้ว)
static ProfileParams fromP;           // ค่า ณ เวลาที่เริ่มเปลี่ยนช่วง
static uint8_t  profIdx = 0;          // ช่วงที่ใช้อยู่
static bool     profFinal = false;    // true = อยู่ช่วงสุดท้ายแล้ว ไม่ต้องเช็กเวลาอีก
static uint32_t runStartMs = 0;
static uint32_t nextSwitchMs = 0;     // เวลา millis() ที่จะเปลี่ยนช่วงถัดไป
static bool     blending = false;
static uint32_t blendStartMs = 0;

static inline int16_t iabs16(int16_t x) {
  return (x < 0) ? -x : x;
}

static inline int16_t clamp16(int32_t v) {
  if (v > maxSpeed)  return maxSpeed;
  if (v < -maxSpeed) return -maxSpeed;
  return (int16_t)v;
}

static inline ProfileParams paramsOf(uint8_t i) {
  ProfileParams p;
  p.base = PROFILES[i].baseSpeed;
  p.kp   = PROFILES[i].Kp;
  p.kd   = PROFILES[i].Kd;
  p.kpC  = PROFILES[i].KpCorner;
  p.kdC  = PROFILES[i].KdCorner;
  return p;
}

// w = 0..256  (0 = ค่าเก่า, 256 = ค่าใหม่)
static inline int16_t lerp16(int16_t a, int16_t b, int16_t w) {
  return (int16_t)(a + ((int32_t)(b - a) * w) / 256);
}

// เรียกครั้งเดียวตอนปล่อยรถ: เริ่มจับเวลา + รีเซ็ตสถานะ PID สำหรับรอบใหม่
void startRun() {
  runStartMs = millis();
  profIdx = 0;
  curP = paramsOf(0);
  blending = false;

  profFinal = (PROFILES[0].endMs == PROFILE_FOREVER);
  if (!profFinal) nextSwitchMs = runStartMs + PROFILES[0].endMs;

  // รีเซ็ตสถานะ PID ไม่ให้ค่าค้างจากรอบก่อน และกัน derivative kick รอบแรก
  cornerMode = false;
  cornerEnterConfirm = 0;
  cornerExitConfirm  = 0;
  lostConfirm = 0;
  dFiltered = 0;
  lastError = (int16_t)getLinePosition() - (LINE_CENTER + CENTER_OFFSET);
}

// เรียกทุก loop แต่ปกติทำแค่เทียบเวลา 1 ครั้ง (ไม่มี delay ไม่หยุด PID)
static inline void updateProfile(uint32_t now) {
  // 1) ถึงเวลาเปลี่ยนช่วงหรือยัง (while เผื่อกรณีช่วงสั้นกว่า loop)
  while (!profFinal && (int32_t)(now - nextSwitchMs) >= 0) {
    fromP = curP;                       // เริ่มไล่ค่าจากค่าปัจจุบันจริง (ไม่กระโดด)
    profIdx++;
    if (PROFILES[profIdx].endMs == PROFILE_FOREVER) {
      profFinal = true;
    } else {
      nextSwitchMs = runStartMs + PROFILES[profIdx].endMs;
    }
    blendStartMs = now;
    blending = true;
  }

  // 2) ไล่ค่าเข้าหาชุดใหม่ (ทำเฉพาะช่วง PROFILE_BLEND_MS หลังเปลี่ยน)
  if (blending) {
    const ProfileParams tgt = paramsOf(profIdx);
    const uint32_t dt = now - blendStartMs;
    if (PROFILE_BLEND_MS == 0 || dt >= PROFILE_BLEND_MS) {
      curP = tgt;
      blending = false;
    } else {
      const int16_t w = (int16_t)((dt * 256UL) / PROFILE_BLEND_MS);
      curP.base = lerp16(fromP.base, tgt.base, w);
      curP.kp   = lerp16(fromP.kp,   tgt.kp,   w);
      curP.kd   = lerp16(fromP.kd,   tgt.kd,   w);
      curP.kpC  = lerp16(fromP.kpC,  tgt.kpC,  w);
      curP.kdC  = lerp16(fromP.kdC,  tgt.kdC,  w);
    }
  }
}

// ความเร็วตามความโค้ง: ค่าเป็น "เพิ่ม/ลดจาก base ของ profile"
// (เลขในวงเล็บ = ค่าสัมบูรณ์เดิมของ v3.3 ตอน base = 140)
static inline int16_t chooseBaseSpeed(int16_t absError) {
  const int16_t b = curP.base;
  int16_t v;

  if (cornerMode) {
    if (absError > 2600)      v = b - 30;   // (110)
    else if (absError > 1800) v = b - 2;    // (138)
    else if (absError > 1200) v = b + 20;   // (160)
    else if (absError > 700)  v = b + 40;   // (180)
    else                      v = b - 10;   // (130)
  } else {
    if (absError > 2600)      v = b - 30;   // (110)
    else if (absError > 1800) v = b - 8;    // (132)
    else if (absError > 1100) v = b + 6;    // (146)
    else                      v = b;        // (140)
  }

  if (v < 0) v = 0;
  if (v > maxSpeed) v = maxSpeed;
  return v;
}

static inline void updateCornerMode(int16_t error, uint8_t blackCount) {
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
}

void runPID() {
  updateProfile(millis());

  const uint16_t position = getLinePosition();
  const uint8_t blackCount = getBlackCount();

  // ---------------------------
  // Lost-line recovery (ของเดิม)
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
  // Fixed-point PID  (จุดตัด: ไม่มีโหมดพิเศษ PID คุมต่อเนื่อง)
  // ---------------------------
  const int16_t error = (int16_t)position - (LINE_CENTER + CENTER_OFFSET);

  const int16_t dRaw = error - lastError;
  dFiltered += (dRaw - dFiltered) >> 2;   // low-pass derivative แบบเบาๆ
  lastError = error;

  updateCornerMode(error, blackCount);

  const int16_t kp = cornerMode ? curP.kpC : curP.kp;
  const int16_t kd = cornerMode ? curP.kdC : curP.kd;

  const int32_t correction = ((int32_t)error * kp + (int32_t)dFiltered * kd) / PID_SCALE;

  const int16_t driveBase = chooseBaseSpeed(iabs16(error));

  int16_t leftSpeed  = driveBase + (int16_t)correction;
  int16_t rightSpeed = driveBase - (int16_t)correction;

  // trim ซ้าย-ขวา เพื่อบาลานซ์มอเตอร์
  leftSpeed  += LEFT_TRIM;
  rightSpeed += RIGHT_TRIM;

  moveMotor(clamp16(leftSpeed), clamp16(rightSpeed));
}
