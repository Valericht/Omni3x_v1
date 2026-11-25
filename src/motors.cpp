#include "motors.h"
#include <Preferences.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "store.h"

static volatile int32_t encCount[3];
static DRAM_ATTR uint32_t encBMask[3];            // mask bit pin B (semua pin B < GPIO32), diisi di motorsBegin()
// ISR memakai baca register GPIO langsung: pada 500 RPM ada ~32 ribu interupsi per detik per roda
#define ENC_ISR(N) static void IRAM_ATTR encIsr##N() { encCount[N] += (REG_READ(GPIO_IN_REG) & encBMask[N]) ? 1 : -1; }
ENC_ISR(0) ENC_ISR(1) ENC_ISR(2)
static void (*const encIsr[3])() = {encIsr0, encIsr1, encIsr2};

// PID per roda, struktur sama dengan sketch test_motorPID: satuan % PWM per RPM, integral dalam %.
static float kp[3], ki[3], kd[3];
static bool  tuneChain = false;                   // auto-tune "semua roda": lanjut ke roda berikutnya
static float rpm[3], sp[3], integ[3], prevRpm[3], dF[3];
static int32_t lastCount[3];
static volatile float lastDuty[3];                // |duty| terakhir (0..1), untuk pengaman dan pembatas daya

// PID tersimpan sebagai satu blob (satu kali tulis + verifikasi baca-ulang)
struct PidBlob { float kp[3], ki[3], kd[3]; uint32_t magic; };
static constexpr uint32_t PID_MAGIC = 0x50494433;
static bool savePid() {
  PidBlob b; b.magic = PID_MAGIC;
  for (int i = 0; i < 3; i++) { b.kp[i] = kp[i]; b.ki[i] = ki[i]; b.kd[i] = kd[i]; }
  return storeWrite("pid2", "all", &b, sizeof b, 2);
}

// ---- auto-tune step response (open loop) ----
static constexpr int AT_BUF = 500;                // 500 x 20 ms = 10 dtk
static float bufT[AT_BUF], bufY[AT_BUF];
static struct {
  volatile int req = -1;                          // permintaan dari Web (Core 0): roda 0..2
  bool active = false; uint8_t state = 0, phase = 0;   // phase 0 = tunggu diam, 1 = rekam step
  int m = 0, n = 0; uint32_t t0 = 0; float y0 = 0;
} tn;
static uint8_t tuneWhy = 0;                       // 0 -, 1 respons terlalu kecil, 2 encoder terbalik, 3 belum mantap, 4 dibatalkan

static void setDuty(int i, float u) {
  u = constrain(u, -1.0f, 1.0f);
  lastDuty[i] = fabsf(u);
  u *= MOTOR_SIGN[i];
  uint32_t d = (uint32_t)(fabsf(u) * ((1 << PWM_BITS) - 1));
  ledcWrite(2 * i,     u > 0 ? d : 0);
  ledcWrite(2 * i + 1, u < 0 ? d : 0);
}

void motorsBegin() {
  for (int i = 0; i < 3; i++) { kp[i] = PID_KP_DEF; ki[i] = PID_KI_DEF; kd[i] = PID_KD_DEF; }
  PidBlob pb;
  if (storeRead("pid2", "all", &pb, sizeof pb) == sizeof pb && pb.magic == PID_MAGIC) {
    for (int i = 0; i < 3; i++) { kp[i] = pb.kp[i]; ki[i] = pb.ki[i]; kd[i] = pb.kd[i]; }
    Serial.println("[PID] dimuat dari flash");
  } else {
    Preferences pr;                               // format lama (kunci kp0..kd2), hanya dibaca
    if (pr.begin("pid2", true)) {
      bool any = false;
      for (int i = 0; i < 3; i++) {
        char k[8];
        snprintf(k, sizeof k, "kp%d", i); if (pr.isKey(k)) { kp[i] = pr.getFloat(k, kp[i]); any = true; }
        snprintf(k, sizeof k, "ki%d", i); if (pr.isKey(k)) ki[i] = pr.getFloat(k, ki[i]);
        snprintf(k, sizeof k, "kd%d", i); if (pr.isKey(k)) kd[i] = pr.getFloat(k, kd[i]);
      }
      pr.end();
      if (any) { Serial.println("[PID] dimuat dari format lama, disimpan ulang"); savePid(); }
      else Serial.println("[PID] belum pernah disimpan: memakai default");
    } else Serial.println("[PID] belum pernah disimpan: memakai default");
  }
  for (int i = 0; i < 3; i++) {
    encBMask[i] = 1u << PIN_ENC_B[i];
    ledcSetup(2 * i, PWM_FREQ, PWM_BITS);     ledcAttachPin(PIN_M_IN1[i], 2 * i);
    ledcSetup(2 * i + 1, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_M_IN2[i], 2 * i + 1);
    pinMode(PIN_ENC_A[i], INPUT_PULLUP); pinMode(PIN_ENC_B[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[i]), encIsr[i], RISING);
    setDuty(i, 0);
  }
}

void motorsSetWheelCmS(const float v[3]) {
  for (int i = 0; i < 3; i++)
    sp[i] = constrain(v[i] * 60.0f / (2.0f * (float)M_PI * WHEEL_R_CM), -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
}

void motorsStopAll() {
  if (tn.active) { tn.active = false; tn.state = 3; tuneWhy = 4; tuneChain = false; }   // STOP / fail-safe membatalkan auto-tune
  for (int i = 0; i < 3; i++) { sp[i] = 0; integ[i] = 0; dF[i] = 0; setDuty(i, 0); }
}

// ================= auto-tune =================
static void tuneFinish(uint8_t why) {             // why 0 = sukses
  tn.active = false; tn.state = why ? 3 : 2; tuneWhy = why;
  for (int i = 0; i < 3; i++) setDuty(i, 0);
  if (!why && tuneChain && tn.m < 2) tn.req = tn.m + 1; else tuneChain = false;   // lanjut roda berikutnya (mode semua roda)
}

static float meanRange(int a, int b) { float s = 0; for (int i = a; i < b; i++) s += bufY[i]; return s / (b - a); }

// waktu saat respons ternormalisasi (y-y0)/dY pertama kali mencapai lvl (interpolasi linear)
static float crossT(float lvl, float y0, float dY) {
  for (int i = 0; i < tn.n; i++) {
    float z = (bufY[i] - y0) / dY;
    if (z >= lvl) {
      if (i == 0) return bufT[0];
      float zp = (bufY[i - 1] - y0) / dY;
      return bufT[i - 1] + (lvl - zp) / (z - zp) * (bufT[i] - bufT[i - 1]);
    }
  }
  return -1;
}

static void tuneIdentify() {
  const float Ts = CONTROL_PERIOD_MS / 1000.0f;
  if (tn.n < 2 * AT_WIN) { tuneFinish(3); return; }
  float ss = meanRange(tn.n - AT_WIN, tn.n);
  float dY = ss - tn.y0;
  if (fabsf(dY) < AT_MIN_DY_RPM) { tuneFinish(1); return; }   // motor/encoder tidak bereaksi
  if (dY < 0)                    { tuneFinish(2); return; }   // encoder terbalik terhadap motor
  float t28 = crossT(0.283f, tn.y0, dY), t63 = crossT(0.632f, tn.y0, dY);
  if (t28 < 0 || t63 < 0) { tuneFinish(3); return; }

  float K   = dY / AT_STEP_PCT;                   // RPM per % PWM
  float tau = fmaxf(1.5f * (t63 - t28), 0.02f);
  float L   = fmaxf(t63 - tau, Ts);               // dead time minimal 1 sampel
  float r   = L / tau;
  float nkp, nki, nkd;
  if (AT_METHOD == 0) {                           // Ziegler-Nichols (reaction curve)
    nkp = 1.2f * tau / (K * L); nki = nkp / (2.0f * L); nkd = nkp * 0.5f * L;
  } else if (AT_METHOD == 1) {                    // Cohen-Coon PID
    nkp = (1.0f / K) * (tau / L) * (4.0f / 3.0f + r / 4.0f);
    nki = nkp / (L * (32.0f + 6.0f * r) / (13.0f + 8.0f * r));
    nkd = nkp * 4.0f * L / (11.0f + 2.0f * r);
  } else {                                        // IMC / Lambda (PI)
    float lam = fmaxf(tau, 3.0f * L);
    nkp = tau / (K * (lam + L)); nki = nkp / tau; nkd = 0;
  }
  Serial.printf("[TUNE] roda %d: K=%.3f RPM/%%  tau=%.3f s  L=%.3f s  (steady %.1f RPM) -> Kp=%.4f Ki=%.4f Kd=%.4f\n",
                tn.m + 1, K, tau, L, ss, nkp, nki, nkd);
  motorsSetPID(tn.m, nkp, nki, nkd, true);
  tuneFinish(0);
}

static void tuneStep(uint32_t now) {
  uint32_t el = now - tn.t0;
  if (tn.phase == 0) {                            // tunggu motor diam
    for (int i = 0; i < 3; i++) setDuty(i, 0);
    if ((el >= 1000 && fabsf(rpm[tn.m]) < 3.0f) || el >= 5000) { tn.y0 = rpm[tn.m]; tn.n = 0; tn.t0 = now; tn.phase = 1; }
    return;
  }
  for (int i = 0; i < 3; i++) setDuty(i, i == tn.m ? AT_STEP_PCT / 100.0f : 0.0f);
  if (tn.n < AT_BUF) { bufT[tn.n] = el / 1000.0f; bufY[tn.n] = rpm[tn.m]; tn.n++; }
  bool steady = false;
  if (tn.n >= 2 * AT_WIN + 5 && el >= 1500) {
    float m1 = meanRange(tn.n - AT_WIN, tn.n), m2 = meanRange(tn.n - 2 * AT_WIN, tn.n - AT_WIN);
    steady = fabsf(m1 - m2) < fmaxf(0.015f * fabsf(m1), AT_STEADY_ABS);
  }
  if (steady || el >= AT_MAX_MS || tn.n >= AT_BUF) tuneIdentify();
}

// ================= loop kecepatan =================
void motorsUpdate(float dt) {
  for (int i = 0; i < 3; i++) {
    int32_t c = encCount[i];
    float r = ENC_SIGN[i] * (float)(c - lastCount[i]) / ENC_CPR / dt * 60.0f;
    lastCount[i] = c;
    rpm[i] += 0.5f * (r - rpm[i]);                     // low-pass
    if (fabsf(rpm[i]) < 0.5f && fabsf(r) < 0.5f) rpm[i] = 0;
  }
  if (tn.req >= 0 && !tn.active) {                     // permintaan auto-tune dari Web
    tn.m = constrain(tn.req, 0, 2); tn.req = -1;
    tn.active = true; tn.state = 1; tn.phase = 0; tn.t0 = millis(); tn.n = 0; tuneWhy = 0;
    for (int i = 0; i < 3; i++) integ[i] = 0;
  }
  if (tn.active) { tuneStep(millis()); return; }

  for (int i = 0; i < 3; i++) {
    if (sp[i] == 0 && fabsf(rpm[i]) < 2.0f) { integ[i] = 0; dF[i] = 0; prevRpm[i] = rpm[i]; setDuty(i, 0); continue; }
    float e = sp[i] - rpm[i];
    float dRaw = -(rpm[i] - prevRpm[i]) / dt; prevRpm[i] = rpm[i];
    dF[i] += (dt / (dt + 0.1f)) * (dRaw - dF[i]);      // filter derivative (Tf = 0.1 s)
    float P = kp[i] * e, D = kd[i] * dF[i];
    float iNew = integ[i] + ki[i] * e * dt;
    float u = P + iNew + D;
    if (!((u > 100.0f && e > 0) || (u < -100.0f && e < 0))) integ[i] = iNew;   // anti-windup
    integ[i] = constrain(integ[i], -PID_I_LIMIT, PID_I_LIMIT);
    setDuty(i, constrain(P + integ[i] + D, -100.0f, 100.0f) / 100.0f);
  }
}

void motorsApplyFF() {                                 // jalur cepat override: P + integral terakhir (tanpa menunggu tick PID penuh)
  if (tn.active) return;
  for (int i = 0; i < 3; i++) setDuty(i, sp[i] == 0 ? 0.0f : constrain(kp[i] * (sp[i] - rpm[i]) + integ[i], -100.0f, 100.0f) / 100.0f);
}

float motorsDuty(int i)        { return lastDuty[i]; }
float motorsMaxDuty()          { return fmaxf(lastDuty[0], fmaxf(lastDuty[1], lastDuty[2])); }
float motorsRpm(int i)         { return rpm[i]; }
float motorsSetpointRpm(int i) { return sp[i]; }
float motorsSpeedCmS(int i)    { return rpm[i] * 2.0f * (float)M_PI * WHEEL_R_CM / 60.0f; }
void  motorsGetPID(int m, float& a, float& b, float& c) { m = constrain(m, 0, 2); a = kp[m]; b = ki[m]; c = kd[m]; }
void  motorsSetPID(int m, float a, float b, float c, bool save) {   // m = -1: semua roda
  for (int i = 0; i < 3; i++) {
    if (m >= 0 && m != i) continue;
    kp[i] = a; ki[i] = b; kd[i] = c; integ[i] = 0;
  }
  if (save) savePid();
}
void    motorsAutoTuneStart(int m, float) {
  if (tn.active) return;                              // tune sedang berjalan
  tuneChain = (m < 0);
  tn.req = m < 0 ? 0 : constrain(m, 0, 2);
}
uint8_t motorsTuneState() { return tn.state; }
uint8_t motorsTuneWhy()   { return tuneWhy; }

// ---------- kinematika ----------
void kinInverse(float vx, float vy, float wz, float v[3]) {
  vy *= KIN_Y_SIGN; wz *= KIN_W_SIGN;
  float mx = 0;
  for (int i = 0; i < 3; i++) {
    float th = THETA_DEG[i] * (float)M_PI / 180.0f;
    v[i] = -vx * sinf(th) + vy * cosf(th) + ROBOT_L_CM * wz;
    mx = fmaxf(mx, fabsf(v[i]));
  }
  if (mx > MAX_LIN_CMS) for (int i = 0; i < 3; i++) v[i] *= MAX_LIN_CMS / mx;   // jaga rasio
}

void kinForward(const float v[3], float& vx, float& vy, float& wz) {
  float sx = 0, sy = 0, sw = 0;
  for (int i = 0; i < 3; i++) {
    float th = THETA_DEG[i] * (float)M_PI / 180.0f;
    sx += -sinf(th) * v[i]; sy += cosf(th) * v[i]; sw += v[i];
  }
  vx = sx * (2.0f / 3.0f); vy = KIN_Y_SIGN * sy * (2.0f / 3.0f); wz = KIN_W_SIGN * sw / (3.0f * ROBOT_L_CM);
}
