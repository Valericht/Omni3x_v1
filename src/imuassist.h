#pragma once
#include "config.h"

// IMU sebagai "wasit": arah, medan, selip roda, stuck, dan auto-tune gain PID yaw.
// Encoder tetap sumber jarak; kepercayaannya (encoder_trust) turun saat selip.
// Semua sudut dalam DERAJAT, gyro dalam deg/s.

enum Terrain : uint8_t { TERRAIN_FLAT = 0, TERRAIN_SLOPE = 1, TERRAIN_TILT = 2 };
struct ImuFiltered { float yaw, pitch, roll, gyroZ; };
enum PidStatus : uint8_t { PID_TUNING = 0, PID_DONE = 1 };
struct PidGains { float kp, ki, kd; uint8_t status; bool adjusted; };   // adjusted = sudah pernah diubah auto-tune

float       normalizeAngle(float angle);                                              // F1: wrap -180..+180
ImuFiltered readImuFiltered();                                                        // F2: panggil sekali per tick kontrol
Terrain     detectTerrain(float pitch, float roll);                                   // F3 (L2: dengan histeresis, simpan status terakhir)
void        terrainReset();                                                           // L2: kembali FLAT (untuk tes / IMU reset)
float       slopeSpeedFactor(Terrain terrain, float pitchDeg, float vy);              // L1/L3: 1 datar; SLOPE_DESCENT_FACTOR saat turun (pitch*vy < 0); selain itu SLOPE_SPEED_FACTOR
float       translationScale(float satScale, float slopeF);                           // L3: pengali gabungan = min, bukan kali
float       computeYawCorrection(float targetYaw, float currentYaw, Terrain terrain, float kpDynamic);   // F4: kpDynamic NAN = None
bool        detectStuck(float yawErr, float gyroZ, float encoderDelta, uint32_t nowMs);                  // F6
bool        detectWheelSlip(float yawEncoder, float yawImu, float gyroZ, uint32_t nowMs);                // F7
float       updateEncoderTrust(bool slipDetected);                                    // F8
PidGains    autoTunePid(const float* yawHistory, int n, uint32_t windowMs);           // F9: satu iterasi bila n mencakup satu jendela

PidGains    assistTuneTick(float yawErrDeg);   // kumpulkan galat yaw tiap tick; panggil F9 tiap jendela penuh; kembalikan gain terkini
PidGains    assistPidCurrent();
void        assistResetStuck();
void        assistReset();                     // STATE DONE: timer stuck/slip, filter IMU, encoder_trust, buffer PID (gain hasil tuning dipertahankan)
