#pragma once
#include <Arduino.h>

// ---------------------------
// ความเร็วหลัก
// (baseSpeed / Kp / Kd ย้ายไปอยู่ในตาราง PROFILES ด้านล่างแล้ว)
// ---------------------------
constexpr uint8_t maxSpeed    = 255;   // เพดาน PWM

// ความเร็วพิเศษ
constexpr uint8_t searchSpeed  = 110;    // หาเส้นเมื่อหลุด
constexpr uint8_t twitchSpeed  = 50;    // กระตุกล้อสัญญาณ

// ---------------------------
// PID แบบ fixed-point
// correction = (error*Kp + d*Kd) / PID_SCALE
// ---------------------------
constexpr int16_t PID_SCALE = 1000;

constexpr int16_t LINE_CENTER = 3500; // 0..7000 จาก QTR 8 ตัว
constexpr int16_t CENTER_OFFSET = 0;   // ถ้ารถเอียง ค่อยปรับค่านี้
constexpr int8_t LEFT_TRIM  = 0;        // ปรับสมดุลมอเตอร์ซ้าย
constexpr int8_t RIGHT_TRIM = -4;       // ปรับสมดุลมอเตอร์ขวา

// โหมดโค้ง 90° จะเข้าเมื่อ error สูงต่อเนื่อง (ของเดิม)
constexpr int16_t CORNER_ENTER_ERROR = 2100;
constexpr int16_t CORNER_EXIT_ERROR   = 1400;
constexpr uint8_t CORNER_ENTER_LOOPS  = 2;
constexpr uint8_t CORNER_EXIT_LOOPS    = 3;

// ---------------------------
// TIME-BASED PROFILE  (นับเวลาตั้งแต่ปล่อยปุ่มเริ่มวิ่ง)
// ---------------------------
// แต่ละแถว = 1 ช่วงเวลา
//   endMs     : ใช้ชุดนี้จนถึงกี่ ms นับจากปล่อยรถ
//               เขียนเป็นวินาทีได้ทั้งเต็มและทศนิยม: S(5) = 5 วิ, S(2.5) = 2.5 วิ, S(0.5) = 0.5 วิ
//               หรือเขียน ms ตรง ๆ ก็ได้ เช่น 2500UL (ต้องเรียงจากน้อยไปมาก ไม่ซ้ำกัน)
//               แถวสุดท้ายต้องเป็น PROFILE_FOREVER (คงค่านี้ไปจนหยุดรถ)
//   baseSpeed : ความเร็วฐานของช่วงนั้น (ความเร็วทางตรง)
//   Kp, Kd    : เกนตอนวิ่งปกติ
//   KpCorner, KdCorner : เกนตอนเข้า corner mode (ของเดิม ใช้ตอน error สูงต่อเนื่อง)
//                        ถ้าไม่ต้องการแยก ให้ใส่เท่า Kp/Kd
//
// หมายเหตุ: ความเร็วที่ลด/เพิ่มตาม |error| (ใน chooseBaseSpeed ของ 04_PID.ino)
//           เป็นค่า "เพิ่ม/ลดจาก baseSpeed ของช่วงนั้น" ไม่ใช่ค่าตายตัวแล้ว
// S() คำนวณตอนคอมไพล์เท่านั้น (ใช้กับค่าคงที่ในตาราง) ไม่มีการคำนวณทศนิยมตอนรันบนบอร์ด
// ห้ามใช้ S() กับตัวแปรที่เปลี่ยนค่าตอนรัน เพราะจะดึง float บน AVR เข้ามา
#define S(sec) ((uint32_t)((sec) * 1000.0 + 0.5))

struct PIDProfile {
  uint32_t endMs;
  uint8_t  baseSpeed;
  int16_t  Kp;
  int16_t  Kd;
  int16_t  KpCorner;
  int16_t  KdCorner;
};

// ค่าที่ใช้งานจริงตอนวิ่ง (อยู่ในไฟล์ .h เพราะ Arduino สร้าง prototype ไว้ก่อนโค้ดใน .ino)
struct ProfileParams {
  int16_t base, kp, kd, kpC, kdC;
};

constexpr uint32_t PROFILE_FOREVER = 0xFFFFFFFFUL;

// เวลาไล่ค่าจากชุดเก่าไปชุดใหม่ตอนเปลี่ยนช่วง (ms) ใส่ 0 = เปลี่ยนทันที
constexpr uint16_t PROFILE_BLEND_MS = 50;

// ช่วงเวลาในคอมเมนต์ท้ายแถวคำนวณจาก endMs (แถวบน -> endMs ของแถวนี้) ปรับค่าตามสนามจริง
constexpr PIDProfile PROFILES[] = {
  //  endMs            base   Kp   Kd   KpC  KdC
  {  S(0.2),             100,   30,  500,  40,  750 },   // 0-1 วิ   : เตรียม
  {  S(0.9),             160,   30,  500,  40,  750 },   // 0-1 วิ   : เร่งทางตรง 0.7
  {  S(1.3),             100,   30,  500,  40,  750 },   // 1-2 วิ   : ช่วงโค้ง 0.4
  {  S(1.4),             130,   29,  500,  40,  800 },   // 4-6 วิ   : ปกติ 0.4
  {  S(2.1),             160,   30,  500,  40,  750 },   // 2-2.5 วิ : เร่งทางตรง2 0.8
  {  S(2.5),             100,   30,  500,  40,  750 },   // 2.5-4 วิ : ช่วงโค้ง 0.4
  {  S(3.7),             160,   30,  500,  40,  750 },   // 4-6 วิ   : เร่งโค้ง 1.2
  {  S(4.1),             100,   30,  500,  40,  750 },   // 4-6 วิ   : ช่วงโค้ง 0.4
  {  S(7.9),             130,   29,  500,  40,  800 },   // 4-6 วิ   : ปกติ 3.6
  {  S(8.5),              80,   30,  500,  40,  750 },   // 4-6 วิ   : ชะลอ 0.8
  {  S(8.9),             130,   29,  500,  40,  800 },   // 4-6 วิ   : ปกติ 0.4
  {  S(9.3),             160,   30,  500,  40,  750 },   // 4-6 วิ   : เร่งโค้ง 0.4
  {  S(9.7),             185,   30,  500,  40,  750 },   // 4-6 วิ   : เร่งตรง 0.5
  {  S(10.3),            150,   30,  500,  40,  750 },   // 4-6 วิ   : เร่งโค้ง 0.4
  {  PROFILE_FOREVER,    130,    29,  500,  40,  800 },   // 6 วิ+    : ช่วงสุดท้าย   kp 25 กลม    30 เลขสี่
};
constexpr uint8_t PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);

static_assert(PROFILE_COUNT >= 1, "PROFILES ต้องมีอย่างน้อย 1 แถว");
static_assert(PROFILES[PROFILE_COUNT - 1].endMs == PROFILE_FOREVER,
              "แถวสุดท้ายของ PROFILES ต้องใช้ endMs = PROFILE_FOREVER");

// endMs ต้องเรียงจากน้อยไปมากแบบเข้มงวด (ซ้ำกัน = ช่วงยาว 0 ms ซึ่งจะถูกข้ามเงียบ ๆ)
constexpr bool profilesAscending(uint8_t i) {
  return (i + 1 >= PROFILE_COUNT) ? true
       : (PROFILES[i].endMs < PROFILES[i + 1].endMs) && profilesAscending(i + 1);
}
static_assert(profilesAscending(0),
              "endMs ใน PROFILES ต้องเรียงจากน้อยไปมาก และห้ามซ้ำกัน (เช็กการพิมพ์ S(...) หรือ ms)");

// ทุกช่วงที่มีขอบเขต (ยกเว้นแถวสุดท้าย) ต้องยาว >= PROFILE_BLEND_MS ไม่งั้นค่าไล่ไม่ถึงเป้าก่อนเปลี่ยนช่วงถัดไป
constexpr uint32_t profileWindowMs(uint8_t i) {
  return PROFILES[i].endMs - ((i == 0) ? 0UL : PROFILES[i - 1].endMs);
}
constexpr bool profileWindowsLongEnough(uint8_t i) {
  return (i + 1 >= PROFILE_COUNT) ? true
       : (profileWindowMs(i) >= PROFILE_BLEND_MS) && profileWindowsLongEnough(i + 1);
}
static_assert(profileWindowsLongEnough(0),
              "มีช่วงใน PROFILES สั้นกว่า PROFILE_BLEND_MS: ขยายช่วงนั้น หรือลด PROFILE_BLEND_MS (0 = เปลี่ยนทันที)");

// ---------------------------
// Sensor / Lost-line
// ---------------------------
constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t DARK_THRESHOLD = 500;   // (ของเดิม ดูหมายเหตุในสรุป: uint8_t เก็บ 500 ไม่ได้ ค่าจริง = 244)

// เส้นขวาง/จุดตัด: ถ้าดำพร้อมกันตั้งแต่กี่ตัวขึ้นไป ถือว่าตำแหน่งเส้นไม่มีข้อมูลซ้าย-ขวา
// -> ไม่เอามาคิด error รอบนั้น ให้วิ่งตรงด้วยความเร็วของ profile (ไม่มี timer ไม่มี lock)
// ใช้ค่าเดิมของ v3.3 (CROSS_DARK_COUNT = 6) ใส่ 9 = ปิดระบบนี้ (เพื่อทดสอบเทียบ)
constexpr uint8_t WIDE_BLACK_COUNT = 6;

constexpr uint8_t LOST_CONFIRM_LOOPS = 2;  // หลุดเส้นติดต่อกันกี่ loop ถึงเข้าสู่โหมดค้นหา//3

// ---------------------------
// ขาอุปกรณ์ (ห้ามเปลี่ยน)
// ---------------------------
constexpr uint8_t PWMA = 5;
constexpr uint8_t AIN1 = 2;
constexpr uint8_t AIN2 = 3;

constexpr uint8_t PWMB = 6;
constexpr uint8_t BIN1 = 4;
constexpr uint8_t BIN2 = 7;

constexpr uint8_t BUTTON_PIN = 8;

constexpr uint8_t SENSOR_PINS[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
