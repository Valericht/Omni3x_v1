#include "safety.h"

static float hpx, hpy, pmx, pmy;         // kecepatan encoder yang sudah di-high-pass
static float vix, viy, vz;               // kecepatan dari integral percepatan IMU (bocor, jadi ekuivalen high-pass)
static float slipT, spinT, stallT, liftT, spikeT, carryT;
static float dbgE, dbgI, dbgR;
static bool  firstStep = true, liftFlag = false;
static constexpr float TAU_Z = 0.6f;

void safetyReset() {
  hpx = hpy = vix = viy = vz = 0; slipT = spinT = stallT = liftT = spikeT = carryT = 0;
  dbgE = dbgI = dbgR = 0; firstStep = true; liftFlag = false;
}
bool safetyLiftEvent() { bool f = liftFlag; liftFlag = false; return f; }
void safetyDebug(float out[3]) { out[0] = dbgE; out[1] = dbgI; out[2] = dbgR; }

static float rise(float t, bool c, float dt) { return c ? t + dt : fmaxf(0.0f, t - 2.0f * dt); }

uint8_t safetyStep(const SafetyIn& in) {
  const float dt = in.dt;
  if (firstStep) { pmx = in.mvx; pmy = in.mvy; firstStep = false; }
  const float a = SAFE_TAU_S / (SAFE_TAU_S + dt), b = TAU_Z / (TAU_Z + dt);
  hpx = a * (hpx + in.mvx - pmx); hpy = a * (hpy + in.mvy - pmy); pmx = in.mvx; pmy = in.mvy;
  if (in.imuOk) { vix = a * (vix + dt * in.accR); viy = a * (viy + dt * in.accF); vz = b * (vz + dt * in.accZ); }

  const float rate = fabsf(in.rateDps), wzDeg = fabsf(in.wzCmd) * 57.2958f, encDeg = fabsf(in.mw) * 57.2958f;
  const float sE = hypotf(hpx, hpy), sI = hypotf(vix, viy);
  dbgE = sE; dbgI = sI; dbgR = rate;

  // ---- diangkat / dipindah (selalu dipantau selama IMU hidup) ----
  if (in.imuOk) {
    bool rest = !in.busy;
    for (int i = 0; i < 3; i++) if (fabsf(in.rpm[i]) >= 2.0f) rest = false;
    liftT  = rise(liftT,  fabsf(vz) > SAFE_LIFT_VZ, dt);
    spikeT = rise(spikeT, fabsf(in.accZ) > SAFE_LIFT_ACC, dt);
    carryT = rise(carryT, rest && (rate > SAFE_CARRY_DPS || hypotf(in.accR, in.accF) > SAFE_CARRY_ACC), dt);
    if (liftT >= 0.10f || spikeT >= 0.08f || carryT >= 0.15f) { liftFlag = true; liftT = spikeT = carryT = 0; vz = 0; }
  }

  if (!in.enabled || !in.busy) { slipT = spinT = stallT = 0; return 0; }

  // ---- 1. roda berputar kencang tetapi IMU tidak merasakan gerak ----
  slipT = rise(slipT, in.imuOk && sE > SAFE_SLIP_MIN_CMS && sI < SAFE_SLIP_RATIO * sE, dt);
  // ---- 2. putaran liar ----
  const float lim = fmaxf(SAFE_YAW_MIN_DPS, SAFE_YAW_GAIN * wzDeg + SAFE_YAW_ADD_DPS);
  spinT = rise(spinT, in.imuOk && (rate > lim || rate - encDeg > SAFE_YAW_ERR_DPS), dt);
  // ---- 3. motor macet: duty tinggi, roda hampir diam ----
  bool st = false;
  for (int i = 0; i < 3; i++) if (fabsf(in.duty[i]) > SAFE_STALL_DUTY && fabsf(in.rpm[i]) < SAFE_STALL_RPM && fabsf(in.sp[i]) > 20.0f) st = true;
  stallT = rise(stallT, st, dt);

  if (spinT >= SAFE_YAW_S)   return 2;
  if (slipT >= SAFE_SLIP_S)  return 1;
  if (stallT >= SAFE_STALL_S) return 3;
  return 0;
}
