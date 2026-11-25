#include "imuassist.h"
#include "imu.h"

static constexpr float RAD2DEG = 57.29578f;

// ---------------- F1 ----------------
float normalizeAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ---------------- F2 ----------------
static float fYaw = 0, fPitch = 0, fRoll = 0, fGyro = 0;
static bool  fInit = false;

ImuFiltered readImuFiltered() {
  if (!imuOk()) { fInit = false; return {fYaw, fPitch, fRoll, fGyro}; }
  float yaw = imuYawRad() * RAD2DEG, pitch = imuPitchDeg(), roll = imuRollDeg(), gz = imuRateDps();
  if (!fInit) { fYaw = yaw; fPitch = pitch; fRoll = roll; fGyro = gz; fInit = true; }
  else {
    fYaw = normalizeAngle(fYaw + IMU_FILTER_ALPHA * normalizeAngle(yaw - fYaw));   // yaw: filter lewat selisih terbungkus
    fPitch += IMU_FILTER_ALPHA * (pitch - fPitch);
    fRoll  += IMU_FILTER_ALPHA * (roll - fRoll);
    fGyro  += IMU_FILTER_ALPHA * (gz - fGyro);
  }
  return {fYaw, fPitch, fRoll, fGyro};
}

// ---------------- F3 ----------------
static Terrain terrainPrev = TERRAIN_FLAT;
void terrainReset() { terrainPrev = TERRAIN_FLAT; }
Terrain detectTerrain(float pitch, float roll) {      // L2: masuk di ambang, keluar di ambang keluar (histeresis)
  float ap = fabsf(pitch), ar = fabsf(roll);
  Terrain t = TERRAIN_FLAT;
  if (ap >= PITCH_SLOPE_THRESHOLD || (terrainPrev == TERRAIN_SLOPE && ap >= PITCH_SLOPE_EXIT_DEG)) t = TERRAIN_SLOPE;
  else if (ar >= ROLL_TILT_THRESHOLD || (terrainPrev == TERRAIN_TILT && ar >= ROLL_TILT_EXIT_DEG)) t = TERRAIN_TILT;
  terrainPrev = t; return t;
}

// ---------------- L1 / L3 ----------------
float slopeSpeedFactor(Terrain terrain, float pitchDeg, float vy) {
  if (terrain == TERRAIN_FLAT) return 1.0f;
  return (terrain == TERRAIN_SLOPE && pitchDeg * vy < 0) ? SLOPE_DESCENT_FACTOR : SLOPE_SPEED_FACTOR;   // nose-up positif, maju = +vy
}
float translationScale(float satScale, float slopeF) { return fminf(satScale, slopeF); }

// ---------------- F4 ----------------
float computeYawCorrection(float targetYaw, float currentYaw, Terrain terrain, float kpDynamic) {
  float err = normalizeAngle(targetYaw - currentYaw);
  float kp  = !isnan(kpDynamic) ? kpDynamic : (terrain != TERRAIN_FLAT ? KP_YAW_SLOPE : KP_YAW_FLAT);
  return err * kp;
}

// ---------------- F6 ----------------
static bool stuckOn = false; static uint32_t stuckT0 = 0;
void assistResetStuck() { stuckOn = false; }
bool detectStuck(float yawErr, float gyroZ, float encoderDelta, uint32_t nowMs) {
  bool c = fabsf(yawErr) > STUCK_YAW_ERR && fabsf(gyroZ) < STUCK_GYRO_MIN && fabsf(encoderDelta) < STUCK_ENC_MIN;
  if (!c) { stuckOn = false; return false; }
  if (!stuckOn) { stuckOn = true; stuckT0 = nowMs; }
  return nowMs - stuckT0 >= STUCK_TIME_MS;
}

// ---------------- F7 ----------------
static bool slipOn = false; static uint32_t slipT0 = 0;
bool detectWheelSlip(float yawEncoder, float yawImu, float gyroZ, uint32_t nowMs) {
  bool c = fabsf(normalizeAngle(yawEncoder - yawImu)) > SLIP_YAW_THRESHOLD && fabsf(gyroZ) > SLIP_GYRO_MIN;
  if (!c) { slipOn = false; return false; }
  if (!slipOn) { slipOn = true; slipT0 = nowMs; }
  return nowMs - slipT0 >= SLIP_TIME_MS;
}

// ---------------- F8 ----------------
static float encoderTrust = ENCODER_TRUST_INIT;
float updateEncoderTrust(bool slip) {
  if (slip) encoderTrust = fmaxf(0.0f, encoderTrust - SLIP_TRUST_DECAY);
  else      encoderTrust = fminf(1.0f, encoderTrust + SLIP_TRUST_RECOVER);
  return encoderTrust;
}

// ---------------- F9 ----------------
static float tKp = PID_KP_INIT, tKi = PID_KI_INIT, tKd = PID_KD_INIT;
static int   tIter = 0, tPersist = 0;
static bool  tAdjusted = false;
static constexpr int TUNE_SS_N = 25;                       // 25 x 20 ms = 0,5 dtk terakhir untuk error steady

PidGains assistPidCurrent() { return {tKp, tKi, tKd, (uint8_t)(tIter >= PID_MAX_ITER ? PID_DONE : PID_TUNING), tAdjusted}; }

PidGains autoTunePid(const float* h, int n, uint32_t windowMs) {
  if (tIter >= PID_MAX_ITER) return assistPidCurrent();           // batas iterasi: pakai nilai terakhir
  if (!h || n < (int)(windowMs / CONTROL_PERIOD_MS)) return assistPidCurrent();   // jendela belum penuh
  float mn = h[0], mx = h[0];
  for (int i = 1; i < n; i++) { mn = fminf(mn, h[i]); mx = fmaxf(mx, h[i]); }
  float amp = mx - mn;
  int m = n < TUNE_SS_N ? n : TUNE_SS_N; float ss = 0;
  for (int i = n - m; i < n; i++) ss += fabsf(h[i]);
  ss /= m;

  float okp = tKp, oki = tKi, okd = tKd;
  tPersist = ss > PID_STEADY_ERR ? tPersist + 1 : 0;
  if (amp > PID_OSCILLATION_AMP)      { tKp -= PID_KP_STEP; tKd += PID_KD_STEP; }   // redam osilasi
  else if (ss > PID_STEADY_ERR)       { tKp += PID_KP_STEP; }                       // percepat konvergensi
  if (tPersist > 2) tKi += 0.01f;                                                   // error steady persisten > 2 jendela
  tKp = constrain(tKp, 0.1f, 5.0f); tKi = constrain(tKi, 0.0f, 1.0f); tKd = constrain(tKd, 0.0f, 1.0f);
  if (tKp != okp || tKi != oki || tKd != okd) tAdjusted = true;
  tIter++;
  Serial.printf("[YAWPID] iter %d/%d amp=%.1f ss=%.1f -> kp=%.2f ki=%.2f kd=%.2f%s\n", tIter, PID_MAX_ITER, amp, ss, tKp, tKi, tKd, tIter >= PID_MAX_ITER ? " (DONE)" : "");
  return assistPidCurrent();
}

// buffer galat yaw satu jendela
static constexpr int HIST_N = PID_TUNE_WINDOW_MS / CONTROL_PERIOD_MS;
static float hist[HIST_N]; static int histN = 0;
PidGains assistTuneTick(float yawErrDeg) {
  if (tIter >= PID_MAX_ITER) return assistPidCurrent();
  hist[histN++] = yawErrDeg;
  if (histN >= HIST_N) { autoTunePid(hist, histN, PID_TUNE_WINDOW_MS); histN = 0; }
  return assistPidCurrent();
}

// ---------------- STATE DONE ----------------
void assistReset() {
  stuckOn = false; slipOn = false; encoderTrust = ENCODER_TRUST_INIT;
  histN = 0; tPersist = 0; fInit = false;
}
