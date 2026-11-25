#pragma once
#include "config.h"
bool    imuBegin();            // cari BNO055 di 0x28/0x29, reset, set mode fusion (robot harus diam)
void    imuUpdate();           // baca yaw + percepatan linear; sambung ulang otomatis bila putus
float   imuYawRad();           // -pi..pi, CCW positif
float   imuPitchDeg();         // pitch (derajat) relatif baseline; nose-up POSITIF (IMU_PITCH_SIGN)
void    imuLevel();            // L2: ambil ulang baseline pitch/roll dari pose sekarang (panggil hanya saat datar dan diam)
float   imuRollDeg();          // roll (derajat) relatif baseline saat init; miring samping
float   imuRateDps();          // laju putar yaw (deg/s), CCW positif
bool    imuOk();
void    imuZeroYaw();
void    imuSetYawRad(float rad);       // jadikan yaw sekarang = rad (memulihkan arah dunia setelah restart)
bool    imuTakeResetFlag();            // true sekali bila chip IMU ter-reset (acuan yaw hilang)
float   imuLinAccelZ();                // percepatan linear sumbu atas badan (cm/s^2)
bool    imuLinAccelBody(float& right, float& fwd);   // percepatan linear sumbu robot (cm/s^2), false bila IMU mati
uint8_t imuAddress();          // 0x28 / 0x29
uint8_t imuCalib();            // bit 7:6 SYS, 5:4 GYR, 3:2 ACC, 1:0 MAG (0..3)
bool    imuIsNdof();
