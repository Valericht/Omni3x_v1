#include "movecheck.h"
#include "motors.h"     // kinForward

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int  reqDir = 0;
static volatile bool active = false;
static McResult res;                    // hanya ditulis task kontrol (Core 1)
static uint8_t  dir = 0;
static uint32_t t0 = 0;
static float    biasR = 0, biasF = 0, wzInt = 0, yaw0 = 0, yawNow = 0;
static int      biasN = 0;

void mcRequest(int d) { if (d >= MC_FWD && d <= MC_RIGHT) { portENTER_CRITICAL(&mux); reqDir = d; portEXIT_CRITICAL(&mux); } }
int  mcConsumeRequest() { portENTER_CRITICAL(&mux); int d = reqDir; reqDir = 0; portEXIT_CRITICAL(&mux); return d; }
bool mcActive() { return active; }
McResult mcResult() { return res; }       // dibaca task kontrol sendiri

static void dirVec(int d, float& ex, float& ey) {   // +x kanan, +y depan
  ex = ey = 0;
  if (d == MC_FWD) ey = 1; else if (d == MC_BACK) ey = -1; else if (d == MC_LEFT) ex = -1; else if (d == MC_RIGHT) ex = 1;
}
static float wrapDeg(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

bool mcStart(int d, float yawRad) {
  if (d < MC_FWD || d > MC_RIGHT) return false;
  dir = (uint8_t)d; t0 = millis();
  biasR = biasF = wzInt = 0; biasN = 0; yaw0 = yawNow = yawRad;
  res = McResult(); res.state = 1; res.dir = dir;
  active = true;
  return true;
}
void mcAbortStart(int d, uint8_t why) { res = McResult(); res.state = 3; res.dir = (uint8_t)d; res.why = why; }
void mcCancel(uint8_t why) {
  if (!active) return;
  active = false; res.state = 3; res.why = why;
}

// Nilai satu arah. main = geser searah arah yang diminta, off = geser menyamping.
static uint8_t judge(float dx, float dy, float ex, float ey, float minMove) {
  float main = dx * ex + dy * ey;
  float off  = fabsf(-dx * ey + dy * ex);
  float am = fabsf(main);
  if (am < minMove && off < minMove) return 3;          // tidak bergerak
  if (main <= -minMove) return 2;                       // arah terbalik
  if (main >= minMove && off <= 0.5f * main) return 1;  // OK
  return 4;                                             // melenceng
}

bool mcStep(uint32_t nowMs, float dt, bool full, const float mv[3], float yawRad,
            bool imuOk, float iaR, float iaF, float& vx, float& vy) {
  vx = vy = 0;
  if (!active) return false;
  uint32_t el = nowMs - t0;
  const uint32_t tRun0 = TEST_REST_MS, tRun1 = TEST_REST_MS + TEST_RUN_MS, tEnd = tRun1 + TEST_BRAKE_MS;
  float bx, by, bw; kinForward(mv, bx, by, bw);
  yawNow = yawRad;

  if (full) {
    if (el < tRun0) {                                    // diam: ukur bias akselerometer
      if (imuOk) { biasR += iaR; biasF += iaF; biasN++; }
    } else {
      res.dx += bx * dt; res.dy += by * dt; wzInt += bw * dt;
      if (el < tRun1 && imuOk) {                         // integral percepatan = perubahan kecepatan
        float bR = biasN ? biasR / biasN : 0, bF = biasN ? biasF / biasN : 0;
        res.ix += (iaR - bR) * dt; res.iy += (iaF - bF) * dt;
      }
    }
  }

  float ex, ey; dirVec(dir, ex, ey);
  if (el >= tRun0 && el < tRun1) { vx = ex * TEST_V_CMS; vy = ey * TEST_V_CMS; }

  if (el >= tEnd) {
    active = false;
    res.state = 2;
    res.yawFromImu = imuOk;
    res.dyaw = imuOk ? wrapDeg((yawNow - yaw0) * 180.0f / (float)M_PI) : wzInt * 180.0f / (float)M_PI;
    res.vOdom = judge(res.dx, res.dy, ex, ey, TEST_MIN_MOVE_CM);
    res.vImu  = 0;
    if (imuOk && biasN > 0) {
      uint8_t v = judge(res.ix, res.iy, ex, ey, TEST_IMU_MIN_DV);
      res.vImu = (v == 3) ? 5 : v;                       // IMU diam = sinyal lemah, bukan "tidak bergerak"
    }
    vx = vy = 0;
    return false;
  }
  return true;
}
