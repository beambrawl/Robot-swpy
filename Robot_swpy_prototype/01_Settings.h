#pragma once
#include <Arduino.h>

// ---------------------------
// ความเร็วหลัก
// ---------------------------
constexpr uint8_t baseSpeed  = 140;   // ทางตรง
constexpr uint8_t turnSpeed   = 110;   // โค้งแคบ / มุมหนัก
constexpr uint8_t maxSpeed    = 255;   // เพดาน PWM

// ความเร็วพิเศษ
constexpr uint8_t crossSpeed  = 180;   // วิ่งตรงผ่านจุดตัด
constexpr uint8_t searchSpeed  = 95;    // หาเส้นเมื่อหลุด
constexpr uint8_t twitchSpeed  = 50;    // กระตุกล้อสัญญาณ

// ---------------------------
// PID แบบ fixed-point
// correction = (error*Kp + d*Kd) / PID_SCALE
// ---------------------------
constexpr int16_t PID_SCALE = 1000;

// โปรไฟล์ทางตรง
constexpr int16_t KpNormal = 40;
constexpr int16_t KdNormal = 650;

// โปรไฟล์โค้ง 90° / โค้งหนัก
constexpr int16_t KpCorner = 70;//52
constexpr int16_t KdCorner = 750;//950

constexpr int16_t LINE_CENTER = 3500; // 0..7000 จาก QTR 8 ตัว
constexpr int16_t CENTER_OFFSET = 0;   // ถ้ารถเอียง ค่อยปรับค่านี้
constexpr int8_t LEFT_TRIM  = 0;        // ปรับสมดุลมอเตอร์ซ้าย
constexpr int8_t RIGHT_TRIM = -4;       // ปรับสมดุลมอเตอร์ขวา

// โหมดโค้ง 90° จะเข้าเมื่อ error สูงต่อเนื่อง
constexpr int16_t CORNER_ENTER_ERROR = 2100;
constexpr int16_t CORNER_EXIT_ERROR   = 1400;
constexpr uint8_t CORNER_ENTER_LOOPS  = 2;
constexpr uint8_t CORNER_EXIT_LOOPS    = 3;

// ---------------------------
// Sensor / Cross / Lost-line
// ---------------------------
constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t DARK_THRESHOLD = 500;

constexpr uint8_t CROSS_DARK_COUNT = 6;   // เจอดำพร้อมกันกี่ตัวถึงสงสัยว่าตัดเส้น
constexpr uint8_t CROSS_CONFIRM_LOOPS = 2; // ต้องเจอติดต่อกันกี่ loop
constexpr uint16_t CROSS_HOLD_MS = 26;     // ดันตรงผ่านจุดตัดกี่ ms

constexpr uint8_t LOST_CONFIRM_LOOPS = 3;  // หลุดเส้นติดต่อกันกี่ loop ถึงเข้าสู่โหมดค้นหา

// ---------------------------
// ขาอุปกรณ์
// ---------------------------
constexpr uint8_t PWMA = 5;
constexpr uint8_t AIN1 = 2;
constexpr uint8_t AIN2 = 3;

constexpr uint8_t PWMB = 6;
constexpr uint8_t BIN1 = 4;
constexpr uint8_t BIN2 = 7;

constexpr uint8_t BUTTON_PIN = 8;

constexpr uint8_t SENSOR_PINS[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
