#include "imu.h"
#include <Wire.h>

// BNO055 (alamat 0x28 atau 0x29, dicari otomatis). Fusion dihitung di dalam chip:
// yaw dari Euler heading, percepatan linear (tanpa gravitasi) dari register LIA.
static constexpr uint8_t REG_CHIP_ID = 0x00, REG_PAGE_ID = 0x07, REG_BURST = 0x18, REG_CALIB = 0x35,
                         REG_UNIT_SEL = 0x3B, REG_OPR_MODE = 0x3D, REG_PWR_MODE = 0x3E, REG_SYS_TRIGGER = 0x3F;
static constexpr uint8_t BNO_ID = 0xA0, OPR_CONFIG = 0x00, OPR_IMU = 0x08, OPR_NDOF = 0x0C;
static constexpr uint8_t OPR_TARGET = IMU_USE_NDOF ? OPR_NDOF : OPR_IMU;
static constexpr uint8_t BURST_LEN = 22;   // 0x18..0x2D: gyro Z(2) heading(2) roll(2) pitch(2) quat(8) LIA xyz(6)

static uint8_t addr = 0x28, calib = 0, failStreak = 0;
static bool    ok = false;
static float   yawRaw = 0, yawOff = 0, rate = 0, liaR = 0, liaF = 0, liaZ = 0;   // yaw deg CCW
static float   pitchRaw = 0, rollRaw = 0, pitchOff = 0, rollOff = 0;             // pitch/roll deg; Off = baseline datar saat init
static uint32_t calMs = 0, retryMs = 0;
static volatile bool resetFlag = false;          // chip ter-reset setelah robot jalan: acuan yaw hilang

static bool wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool rd(uint8_t reg, uint8_t* b, uint8_t n) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
static float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

static bool setMode(uint8_t m) {
  if (!wr(REG_OPR_MODE, OPR_CONFIG)) return false;
  delay(25);
  if (m != OPR_CONFIG) { if (!wr(REG_OPR_MODE, m)) return false; delay(25); }
  return true;
}

// Cari chip di 0x28 lalu 0x29 lewat CHIP_ID (0xA0). Jeda 50 ms hanya di antara percobaan.
static bool probe(int tries) {
  const uint8_t addrs[2] = {0x28, 0x29};
  for (int t = 0; t < tries; t++) {
    for (int i = 0; i < 2; i++) {
      addr = addrs[i]; uint8_t id = 0;
      if (rd(REG_CHIP_ID, &id, 1) && id == BNO_ID) return true;
    }
    if (t + 1 < tries) delay(50);
  }
  return false;
}

static bool readFrame() {
  uint8_t b[BURST_LEN];
  if (!rd(REG_BURST, b, BURST_LEN)) return false;
  auto s16 = [&](int i) { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  float gz      = s16(0) / 16.0f;                 // deg/s, CCW positif
  float heading = s16(2) / 16.0f;                 // 0..360, searah jarum jam
  rollRaw  = s16(4) / 16.0f;                      // Euler roll, derajat
  pitchRaw = s16(6) / 16.0f;                      // Euler pitch, derajat
  float lx = (float)s16(16), ly = (float)s16(18), lz = (float)s16(20); // 1 LSB = 0.01 m/s^2 = 1 cm/s^2
  rate = gz * IMU_YAW_SIGN;
  if (fabsf(rate) < 0.08f) rate = 0;              // deadband noise
  yawRaw = wrap180(-heading * IMU_YAW_SIGN);      // ke konvensi CCW positif
  float fw = IMU_ACC_FWD_AXIS == 1 ? ly : lx;
  float rt = IMU_ACC_FWD_AXIS == 1 ? lx : ly;
  liaF = fw * IMU_ACC_FWD_SIGN;
  liaR = rt * IMU_ACC_RIGHT_SIGN;
  liaZ = lz;                                      // atas (tegak lurus badan), untuk deteksi diangkat
  return true;
}

bool imuBegin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, IMU_I2C_HZ);
  Wire.setTimeOut(30);
  ok = false; failStreak = 0;
  if (!probe(20)) {                               // sampai ~1 s: BNO055 butuh ~650 ms setelah power-on
    Serial.println("[IMU] BNO055 tidak ditemukan di 0x28/0x29 (cek 3V3, GND, SDA=GPIO8, SCL=GPIO9)");
    return false;
  }
  Serial.printf("[IMU] BNO055 ditemukan di 0x%02X\n", addr);

  uint8_t id = 0;
  if (!setMode(OPR_CONFIG)) return false;
  wr(REG_SYS_TRIGGER, 0x20);                      // reset (balasan bisa NACK, abaikan)
  delay(30);
  bool back = false; uint32_t t0 = millis();
  while (millis() - t0 < 1500) {                  // tunggu chip hidup lagi
    if (rd(REG_CHIP_ID, &id, 1) && id == BNO_ID) { back = true; break; }
    delay(10);
  }
  if (!back) { Serial.println("[IMU] chip tidak kembali setelah reset"); return false; }
  delay(50);
  wr(REG_PWR_MODE, 0x00);                         // power normal
  delay(10);
  wr(REG_PAGE_ID, 0x00);
  wr(REG_UNIT_SEL, 0x00);                         // m/s^2, deg/s, derajat, Celsius
  wr(REG_SYS_TRIGGER, 0x00);                      // osilator internal
  delay(10);
  if (!setMode(OPR_TARGET)) return false;
  delay(50);
  if (!readFrame()) return false;
  yawOff = yawRaw;                                // yaw mulai dari 0
  pitchOff = pitchRaw; rollOff = rollRaw;         // baseline pitch/roll: robot diasumsikan datar saat init
  calMs = millis();
  ok = true;
  Serial.printf("[IMU] siap, mode %s, I2C %u kHz\n", IMU_USE_NDOF ? "NDOF" : "IMU", (unsigned)(IMU_I2C_HZ / 1000));
  return true;
}

// Sambung ulang ringan (tanpa reset penuh, supaya task kontrol tidak tertahan lama).
static void recover() {
  if (!probe(1)) return;
  uint8_t m = 0;
  if (!rd(REG_OPR_MODE, &m, 1)) return;
  bool wasReset = (m & 0x0F) != OPR_TARGET;       // chip baru hidup / ter-reset: belum mode fusion
  if (wasReset && !setMode(OPR_TARGET)) return;
  if (!readFrame()) return;
  if (wasReset) { yawOff = yawRaw; pitchOff = pitchRaw; rollOff = rollRaw; resetFlag = true; }   // hanya nol-kan yaw bila chip ter-reset; posisi dunia jadi tidak valid
  ok = true; failStreak = 0;
  Serial.printf("[IMU] tersambung kembali (0x%02X)\n", addr);
}

void imuUpdate() {
  uint32_t now = millis();
  if (!ok) {
    if (now - retryMs >= 1000) { retryMs = now; recover(); }
    return;
  }
  if (!readFrame()) {
    if (++failStreak >= 25) { ok = false; Serial.println("[IMU] terputus (I2C gagal berturut-turut)"); }
    return;
  }
  failStreak = 0;
  if (now - calMs >= 250) { calMs = now; uint8_t c; if (rd(REG_CALIB, &c, 1)) calib = c; }
}

float   imuYawRad()  { return wrap180(yawRaw - yawOff) * (float)M_PI / 180.0f; }
float   imuRateDps() { return rate; }
float   imuPitchDeg() { return IMU_PITCH_SIGN * wrap180(pitchRaw - pitchOff); }   // L1: nose-up positif
void    imuLevel()    { pitchOff = pitchRaw; rollOff = rollRaw; }                   // L2: baseline datar = posisi sekarang
float   imuRollDeg()  { return wrap180(rollRaw - rollOff); }
bool    imuOk()      { return ok; }
void    imuZeroYaw() { yawOff = yawRaw; }
void    imuSetYawRad(float rad) { yawOff = wrap180(yawRaw - rad * 180.0f / (float)M_PI); }   // yaw sekarang dianggap = rad (pemulihan posisi)
bool    imuTakeResetFlag() { bool f = resetFlag; resetFlag = false; return f; }
float   imuLinAccelZ() { return liaZ; }
bool    imuLinAccelBody(float& right, float& fwd) { right = liaR; fwd = liaF; return ok; }
uint8_t imuAddress() { return ok ? addr : 0; }
uint8_t imuCalib()   { return calib; }
bool    imuIsNdof()  { return IMU_USE_NDOF; }
