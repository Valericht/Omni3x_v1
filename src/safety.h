#pragma once
#include "config.h"
// Pengaman gerak. Dipanggil sekali per siklus penuh (50 Hz) setelah encoder diperbarui.
struct SafetyIn {
  float dt;
  bool  imuOk, enabled, busy;       // busy: ada roda yang diperintah berputar
  float mvx, mvy, mw;               // kecepatan badan dari encoder (cm/s, cm/s, rad/s)
  float rateDps;                    // laju yaw dari gyro (deg/s)
  float accR, accF, accZ;           // percepatan linear IMU: kanan, depan, atas (cm/s^2)
  float wzCmd;                      // perintah putar (rad/s)
  float rpm[3], duty[3], sp[3];
};
// 0 = aman; 1 selip / menabrak (encoder cepat, IMU diam); 2 putaran liar; 3 motor macet
uint8_t safetyStep(const SafetyIn& in);
bool    safetyLiftEvent();               // true sekali bila robot terdeteksi diangkat / dipindah (dibaca lalu dihapus)
void    safetyReset();
void    safetyDebug(float out[3]);       // dv encoder, dv IMU (cm/s), laju yaw (deg/s)
