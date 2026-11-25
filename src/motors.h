#pragma once
#include "config.h"

void  motorsBegin();                       // panggil dari task Core 1 (ISR encoder menempel di core ini)
void  motorsSetWheelCmS(const float v[3]); // target kecepatan linear roda (cm/s)
void  motorsUpdate(float dt);              // baca encoder + PID + PWM (tiap 20 ms)
void  motorsApplyFF();                     // update cepat (feed-forward) di luar tick PID
void  motorsStopAll();
float motorsSpeedCmS(int i);
float motorsRpm(int i);
float motorsDuty(int i);                   // |duty| terakhir 0..1
float motorsMaxDuty();                     // duty terbesar dari ketiga roda (deteksi saturasi)
float motorsSetpointRpm(int i);
void  motorsGetPID(int m, float& kp, float& ki, float& kd);                  // m = 0..2
void  motorsSetPID(int m, float kp, float ki, float kd, bool save = true);   // m = 0..2 satu roda, -1 = semua roda
void  motorsAutoTuneStart(int motor, float stepPct);   // auto-tune step response (angkat roda!). motor 0..2 = satu roda, -1 = semua roda berurutan
uint8_t motorsTuneState();                 // 0 idle,1 jalan,2 selesai,3 gagal
uint8_t motorsTuneWhy();                   // sebab gagal: 1 respons kecil, 2 encoder terbalik, 3 belum mantap, 4 dibatalkan

// Kinematika omni-wheel (PRD 5.2)
void kinInverse(float vx, float vy, float wz, float v[3]);
void kinForward(const float v[3], float& vx, float& vy, float& wz);
