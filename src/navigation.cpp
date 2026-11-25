#include "navigation.h"
#include "motors.h"
#include "store.h"

static portMUX_TYPE nmux = portMUX_INITIALIZER_UNLOCKED;
static Waypoint wps[MAX_WP];
static uint8_t  routes[NUM_ROUTES][MAX_RT];          // daftar nomor waypoint, diakhiri 0
static float px = 0, py = 0;
static volatile bool poseOk = true;
static volatile bool poseDegradedNav = false;     // A.5.4: set by the control task
static uint32_t rampMask = 0;                      // bit (id-1) = waypoint id is a known ramp-end point
static volatile bool active = false;
static volatile int  target = 0, routeId = -1, stepIdx = 0, stepN = 0;
static volatile NavSrc src = SRC_NONE;
enum Phase : uint8_t { PH_NONE = 0, PH_MOVING = 1, PH_ROTATING = 2, PH_VERIFY = 3, PH_GRIP = 4 };
static volatile uint8_t phase = PH_NONE, doneSeq = 0;
static uint32_t rotStartMs = 0, settleMs = 0; static float yawErrF = 0;
static uint8_t  ids[MAX_RT];                         // urutan waypoint yang sedang dijalani
static uint32_t legMs = 0;
static uint8_t  retryN = 0; static uint16_t sess[4];      // R5: percobaan ulang leg; statistik sesi: tiba, gagal, retry, koreksi dipakai
static float wallMed[3] = {-1, -1, -1}; static bool wallOk[3]; static uint8_t wallStat = 0; static bool wallCounted = false;
static bool wallTilt = false;
static bool gripPend = false, gripPendOn = false; static uint32_t gripStartMs = 0;   // langkah gripper menunggu diambil task kontrol
static bool isGripStep(int v) { return v == GRIP_STEP_ON || v == GRIP_STEP_OFF; }
// L5: 5 sampel median terakhir per sensor (0 depan, 1 kiri, 2 kanan), satu sampel per >= 50 ms (siklus ping ~60 ms)
static float refHist[3][5]; static uint8_t refCnt[3]; static uint32_t refMs = 0;
// L6: VERIFY
static uint32_t vStartMs = 0, vShiftEnd = 0; static uint8_t vIter = 0; static bool vShift = false;
static ArrLog arrLog;
// L9 / L10 / L8 / L3
enum AvSt : uint8_t { AV_CLEAR = 0, AV_HOLD = 1, AV_SHIFT = 2, AV_PASS = 3 };
static bool irF = false, irL = false, irR = false;                 // IR depan/kiri/kanan (sudah debounce)
static bool avShift = false, avFrontShift = false, avMargin = false; static uint32_t avClearMs = 0; static float avTarget = 0;   // geser menerus: pemicu depan/sisi, margin sudah dipasang, IR pemicu bebas sejak, target total (cm)
static uint8_t avSt = AV_CLEAR; static bool avoidOn = false; static int8_t avoidDir = 1;   // avoidOn = avSt != AV_CLEAR
static bool avConfirmed = false; static float avSx = 0, avSy = 0;
static bool trendOn = false, warn = false, avBlind = false, slOn = false; static float trAx = 0, trAy = 0, trD0 = 0, trPrev = -1, warnMin = 0, slVx = 0, slVy = 0; static uint8_t trCnt = 0;
static float avLat = 0;                               // total geser samping (cm), dari kecepatan lateral keluaran: gerak maju sisa saat peralihan tidak ikut terhitung
static uint32_t warnLostMs = 0, warnDropMs = 0;   // warn = peringatan dini (ultrasonik: tren turun); avBlind = hindar dipicu ultrasonik (IR tak melihat); slOn/slVx/slVy = keadaan pembatas percepatan
static uint32_t avHoldMs = 0, avPulseMs = 0;
static float avOx = 0, avOy = 0;                                // awal geser (untuk total geser)
static uint32_t avoidClearMs = 0, avoidHoldUntil = 0, avoidTotalMs = 0, lastStepMs = 0, staleSince = 0;
static bool refStale = false;
static float wallVxHold = 0; static uint32_t wallVxMs = 0;
static float lastAvoidScale = 1.0f, sideMinSeen = -1.0f, lastDist = 0;
static void avoidReset() { avSt = AV_CLEAR; avConfirmed = false; trendOn = false; trCnt = 0; warn = avBlind = slOn = false; warnLostMs = 0; avShift = avFrontShift = avMargin = false; avClearMs = 0; avoidOn = false; avoidClearMs = 0; avoidHoldUntil = 0; avoidTotalMs = 0; staleSince = 0; refStale = false; lastStepMs = millis(); }
static void legStart() { wallVxMs = 0; legMs = millis(); retryN = 0; wallCounted = false; avoidReset(); }
static volatile int jogDir = 0;
static NavReq req;

struct PoseBlob { float x, y, yaw; uint32_t flags; uint32_t magic; };   // flags bit0 = posisi valid
static constexpr uint32_t POSE_MAGIC = 0x4F33504F;
static PoseBlob saved; static bool hasSaved = false;

struct WaypointZ { bool valid; float x, y; float yaw; bool hasYaw; float wallL, wallR; };   // format wp16z (tanpa referensi depan)
struct WaypointY { bool valid; float x, y; float yaw; bool hasYaw; };   // format wp16y (tanpa referensi dinding)
struct WaypointOld { bool valid; float x, y; };      // format lama (tanpa arah hadap), kunci "wp16"
static bool saveWps()    { Waypoint c[MAX_WP]; wpCopyAll(c); return storeWrite("nav", "wp16f", c, sizeof c, 1); }
static bool saveRoutes() { uint8_t c[NUM_ROUTES][MAX_RT]; routeCopyAll(c); return storeWrite("nav", "rt", c, sizeof c, 4); }

void navBegin() {
  memset(wps, 0, sizeof wps);
  for (int r = 0; r < NUM_ROUTES; r++) { memset(routes[r], 0, MAX_RT); routes[r][0] = r + 1; }   // default: rute A = WP1, B = WP2, ...
  bool migrated = false;
  WaypointY oy[MAX_WP]; WaypointZ oz[MAX_WP];
  if (storeRead("nav", "wp16f", wps, sizeof wps) != sizeof wps) {
    WaypointOld o16[MAX_WP];
    if (storeRead("nav", "wp16z", oz, sizeof oz) == sizeof oz) {              // L6: format 16 waypoint tanpa referensi depan (wallF = 0)
      for (int i = 0; i < MAX_WP; i++) wps[i] = {oz[i].valid, oz[i].x, oz[i].y, oz[i].yaw, oz[i].hasYaw, oz[i].wallL, oz[i].wallR, 0.0f};
      migrated = true;
    } else if (storeRead("nav", "wp16y", oy, sizeof oy) == sizeof oy) {       // format 16 waypoint tanpa referensi dinding
      for (int i = 0; i < MAX_WP; i++) wps[i] = {oy[i].valid, oy[i].x, oy[i].y, oy[i].yaw, oy[i].hasYaw, 0.0f, 0.0f, 0.0f};
      migrated = true;
    } else if (storeRead("nav", "wp16", o16, sizeof o16) == sizeof o16) {     // format 16 waypoint tanpa arah hadap
      for (int i = 0; i < MAX_WP; i++) wps[i] = {o16[i].valid, o16[i].x, o16[i].y, 0.0f, false, 0.0f, 0.0f, 0.0f};
      migrated = true;
    } else {
      WaypointOld o8[8];                                                      // format paling lama: 8 waypoint
      if (storeRead("nav", "wp", o8, sizeof o8) == sizeof o8) {
        for (int i = 0; i < 8; i++) wps[i] = {o8[i].valid, o8[i].x, o8[i].y, 0.0f, false, 0.0f, 0.0f, 0.0f};
        migrated = true;
      }
    }
  }
  uint8_t rt[NUM_ROUTES][MAX_RT];
  bool haveRt = storeRead("nav", "rt", rt, sizeof rt) == sizeof rt;
  if (haveRt) memcpy(routes, rt, sizeof rt);
  if (storeRead("nav", "pose", &saved, sizeof saved) == sizeof saved && saved.magic == POSE_MAGIC) hasSaved = true;

  // Skala odometri berubah (WHEEL_R_CM): waypoint dan posisi tersimpan dari firmware lama masih memakai
  // skala lama. Konversi sekali agar lokasi fisiknya tetap sama (kunci "nav/sver" mencegah konversi ganda).
  static constexpr uint16_t ODOM_SCALE_VER = 2;
  static constexpr float    ODOM_SCALE_FIX = WHEEL_R_CM / WHEEL_R_CM_OLD;
  uint16_t sver = 0;
  if (storeRead("nav", "sver", &sver, sizeof sver) != sizeof sver || sver < ODOM_SCALE_VER) {
    uint16_t ver = ODOM_SCALE_VER;
    storeWrite("nav", "sver", &ver, sizeof ver, 1);   // tulis penanda dulu: bila simpan gagal, jangan sampai konversi berulang
    for (int i = 0; i < MAX_WP; i++) if (wps[i].valid) { wps[i].x *= ODOM_SCALE_FIX; wps[i].y *= ODOM_SCALE_FIX; }
    if (hasSaved) { saved.x *= ODOM_SCALE_FIX; saved.y *= ODOM_SCALE_FIX; }
    saveWps();
    if (hasSaved) storeWrite("nav", "pose", &saved, sizeof saved, 3);
    Serial.printf("[NAV] migrasi skala odometri x%.3f (waypoint & posisi lama)\n", ODOM_SCALE_FIX);
  }
  if (hasSaved) { px = saved.x; py = saved.y; }
  uint32_t rm = 0; rampMask = storeRead("nav", "ramp", &rm, sizeof rm) == sizeof rm ? (rm & ((1u << MAX_WP) - 1)) : 0;   // R6

  int nv = 0; for (int i = 0; i < MAX_WP; i++) if (wps[i].valid) nv++;
  Serial.printf("[NAV] dimuat dari flash: %d waypoint%s, rute %s, posisi terakhir %s\n", nv, migrated ? " (format lama dipindah)" : "",
                haveRt ? "tersimpan" : "default", hasSaved ? "ada" : "belum ada");
  if (migrated) saveWps();
}

bool navSavedPose(float& x, float& y, float& yawRad, bool& valid) {
  if (!hasSaved) return false;
  x = saved.x; y = saved.y; yawRad = saved.yaw; valid = (saved.flags & 1) != 0; return true;
}

void navResetOdom() { portENTER_CRITICAL(&nmux); px = py = 0; portEXIT_CRITICAL(&nmux); }
void navPose(float& x, float& y) { portENTER_CRITICAL(&nmux); x = px; y = py; portEXIT_CRITICAL(&nmux); }
void navSetPoseValid(bool v) { poseOk = v; }
bool navPoseValid() { return poseOk; }

bool navOdom(const float wv[3], float yaw, float dt, float pitchDeg, float rollDeg, float encTrust, bool uncertain, float* yawEncDeg) {
  float bx, by, bw; kinForward(wv, bx, by, bw);
  if (yawEncDeg) {                                         // yaw is never frozen
    float y = *yawEncDeg + bw * 57.2958f * dt;
    while (y > 180.0f) y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    *yawEncDeg = y;
  }
  const float D2R = (float)M_PI / 180.0f;
  by *= by >= 0 ? ODOM_SCALE_FWD : ODOM_SCALE_BWD;         // L7: kalibrasi skala per arah
  if (fabsf(pitchDeg) >= PITCH_SLOPE_THRESHOLD) by *= (by * pitchDeg >= 0) ? ODOM_SCALE_UP : ODOM_SCALE_DOWN;   // L7: ramp (nose-up positif, maju = naik)
  by *= fmaxf(COS_CLAMP_MIN, cosf(pitchDeg * D2R));        // R1: slope projection (forward = body +y)
  bx *= fmaxf(COS_CLAMP_MIN, cosf(rollDeg * D2R));         // lateral = body +x
  bool low = uncertain && encTrust < 1.0f;
  if (uncertain) {                                         // R2: trust-weighted, frozen below ENCODER_TRUST_FREEZE
    if (encTrust < ENCODER_TRUST_FREEZE) return true;
    bx *= encTrust; by *= encTrust;
  }
  float c = cosf(yaw), s = sinf(yaw);
  float wx = bx * c - by * s, wy = bx * s + by * c;       // badan -> dunia
  portENTER_CRITICAL(&nmux); px += wx * dt; py += wy * dt; portEXIT_CRITICAL(&nmux);
  Waypoint w;
  if (active && phase == PH_MOVING && wpGet(ids[stepIdx], w)) {     // R3: koreksi pose dinding, hanya otonom MOVING
    float x, y; navPose(x, y);
    WallCorr c = wallCalc(w, x, y, yaw, wallMed, wallOk, wallTilt);
    if (c.stat != 0 && navAvoiding()) c = {0, 0, 0, 4};     // L9: koreksi pose ditahan selama AVOID + AVOID_HOLD_MS
    float n = hypotf(c.dpx, c.dpy);
    if (c.stat == 1 && n > 1e-6f) {
      float k = fminf(n * WALL_POSE_KP, WALL_POSE_STEP_CM) / n;
      portENTER_CRITICAL(&nmux); px += c.dpx * k; py += c.dpy * k; portEXIT_CRITICAL(&nmux);
      if (!wallCounted) { wallCounted = true; sess[3]++; }
    }
  }
  return low;
}

void navSetPoseDegraded(bool v) { poseDegradedNav = v; }
bool navPoseDegraded() { return poseDegradedNav; }

void navRampMark(int id, bool on) {
  if (id < 1 || id > MAX_WP) return;
  portENTER_CRITICAL(&nmux); uint32_t old = rampMask; if (on) rampMask |= 1u << (id - 1); else rampMask &= ~(1u << (id - 1)); uint32_t m = rampMask; portEXIT_CRITICAL(&nmux);
  if (m != old) storeWrite("nav", "ramp", &m, sizeof m, 1);
}

// R5: end of ramp. Snap only when the current pose is already within tolCm of a marked ramp waypoint.
bool navRampSnap(float tolCm) {
  bool done = false; int hit = 0; float bx = 0, by = 0, bd = tolCm;
  portENTER_CRITICAL(&nmux);
  for (int i = 0; i < MAX_WP; i++) {
    if (!(rampMask & (1u << i)) || !wps[i].valid) continue;
    float d = hypotf(wps[i].x - px, wps[i].y - py);
    if (d <= bd) { bd = d; hit = i + 1; bx = wps[i].x; by = wps[i].y; done = true; }
  }
  if (done) { px = bx; py = by; }
  portEXIT_CRITICAL(&nmux);
  if (done) Serial.printf("[NAV] ramp snap ke WP%d (%.1f, %.1f), selisih %.1f cm\n", hit, bx, by, bd);
  return done;
}

// ---------------- navigasi ----------------
static void enterGrip() { phase = PH_GRIP; gripStartMs = millis(); gripPend = true; gripPendOn = ids[stepIdx] == GRIP_STEP_ON; }
static void begin(int r, int n, NavSrc s) {
  routeId = r; stepN = n; stepIdx = 0; src = s; sideMinSeen = -1.0f; active = true;
  if (isGripStep(ids[0])) { legStart(); enterGrip(); return; }                 // rute diawali langkah gripper
  target = ids[0]; legStart(); phase = PH_MOVING;
}
static bool hasRef(const Waypoint& w) { return w.wallL > 0 || w.wallR > 0 || w.wallF > 0; }

bool navStart(int id, NavSrc s) {
  Waypoint w; if (!poseOk || poseDegradedNav || !wpGet(id, w) || !w.valid) return false;
  ids[0] = (uint8_t)id; begin(-1, 1, s); return true;
}

// L6: "Sinkron ke WP n": hanya VERIFY, robot diam <= 30 cm dari waypoint.
bool navStartVerify(int id, NavSrc s) {
  Waypoint w; if (!poseOk || poseDegradedNav || !wpGet(id, w) || !w.valid || !hasRef(w)) return false;
  float x, y; navPose(x, y); if (hypotf(w.x - x, w.y - y) > 30.0f) return false;
  ids[0] = (uint8_t)id; begin(-1, 1, s); vStartMs = millis(); vIter = 0; vShift = false; phase = PH_VERIFY; return true;
}

bool navStartRoute(int r, NavSrc s) {
  if (r < 0 || r >= NUM_ROUTES || !poseOk || poseDegradedNav) return false;
  int n = 0, nWp = 0;
  portENTER_CRITICAL(&nmux);
  for (int k = 0; k < MAX_RT && routes[r][k] != 0; k++) {
    int id = routes[r][k];
    if (id >= 1 && id <= MAX_WP && wps[id - 1].valid) { ids[n++] = (uint8_t)id; nWp++; }   // waypoint kosong dilewati
    else if (isGripStep(id)) ids[n++] = (uint8_t)id;
  }
  portEXIT_CRITICAL(&nmux);
  if (nWp == 0) return false;                                                  // rute tanpa waypoint valid tidak dijalankan
  begin(r, n, s); return true;
}

void   navSetIr(bool f, bool l, bool r) { irF = f; irL = l; irR = r; }
void   navCancel()    { active = false; phase = PH_NONE; src = SRC_NONE; routeId = -1; avSt = AV_CLEAR; avoidOn = false; vShift = false; }
static void navFinish() { doneSeq = doneSeq + 1; sess[0]++; navCancel(); }       // tuntas: posisi dan arah sesuai
int    navPhase()     { return active ? phase : 0; }
uint8_t navDoneSeq()  { return doneSeq; }
bool   navRouteReady(int r) {
  if (r < 0 || r >= NUM_ROUTES) return false;
  bool ok = false;
  portENTER_CRITICAL(&nmux);
  for (int k = 0; k < MAX_RT && routes[r][k] != 0; k++) { int id = routes[r][k]; if (id >= 1 && id <= MAX_WP && wps[id - 1].valid) { ok = true; break; } }   // minimal satu waypoint valid
  portEXIT_CRITICAL(&nmux);
  return ok;
}
// R5: timeout / stuck pada leg MOVING -> ulang pendekatan dari posisi sekarang; jatah habis -> cancel + alasan 6.
static bool legRetry() {
  if (retryN < NAV_RETRY) { retryN++; sess[2]++; legMs = millis(); wallCounted = false; return true; }
  sess[1]++; navPostDeny(6); navCancel(); return false;
}
bool   navRetryLeg()  { return active && phase == PH_MOVING && legRetry(); }
uint8_t navRetryNo()  { return active ? retryN : 0; }
void   navStats(uint16_t s[4]) { for (int i = 0; i < 4; i++) s[i] = sess[i]; }
void   navSetWall(const float med[3], const bool ok[3], bool tilt) {
  uint32_t now = millis(); bool push = now - refMs >= 50;     // L5: satu sampel riwayat per siklus ping
  if (push) refMs = now;
  portENTER_CRITICAL(&nmux);
  for (int i = 0; i < 3; i++) {
    wallMed[i] = med[i]; wallOk[i] = ok[i];
    if (!push) continue;
    if (!ok[i]) { refCnt[i] = 0; continue; }
    if (refCnt[i] < 5) refCnt[i]++; else for (int k = 0; k < 4; k++) refHist[i][k] = refHist[i][k + 1];
    refHist[i][refCnt[i] - 1] = med[i];
  }
  wallTilt = tilt;
  portEXIT_CRITICAL(&nmux);
}
void   navWallRefCapture(bool still, bool flat, float out[3]) {   // out: kiri, kanan, depan
  out[0] = out[1] = out[2] = 0;
  if (!still || !flat) return;
  portENTER_CRITICAL(&nmux);
  for (int k = 0; k < 3; k++) {
    int i = k == 2 ? 0 : k + 1;                                   // kiri = sensor 1, kanan = sensor 2, depan = sensor 0
    if (!wallOk[i] || refCnt[i] < 5) continue;
    float d = wallMed[i], mn = refHist[i][0], mx = refHist[i][0];
    for (int j = 1; j < 5; j++) { mn = fminf(mn, refHist[i][j]); mx = fmaxf(mx, refHist[i][j]); }
    if (d >= 5.0f && d <= WALL_REF_MAX_CM && mx - mn <= WALL_REF_SPREAD_CM) out[k] = d;
  }
  portEXIT_CRITICAL(&nmux);
}
bool   navAvoiding()    { return avoidOn || millis() < avoidHoldUntil; }
bool   navAvoidActive() { return active && avoidOn; }
uint8_t navAvoidState() { return active ? avSt : 0; }
bool   navRefStale()    { return active && refStale; }
float  navSideMin()     { return sideMinSeen; }
float  navAvoidScale()  { return lastAvoidScale; }
ArrLog navArrLog()      { portENTER_CRITICAL(&nmux); ArrLog a = arrLog; portEXIT_CRITICAL(&nmux); return a; }
void   navWallRef(float ref[2]) { Waypoint w; ref[0] = ref[1] = 0; if (active && wpGet(ids[stepIdx], w)) { ref[0] = w.wallL; ref[1] = w.wallR; } }
uint8_t navWallStat() { return active && phase == PH_MOVING ? wallStat : 0; }
bool   navActive()    { return active; }
int    navTarget()    { return active ? target : 0; }
int    navRoute()     { return active ? routeId : -1; }
int    navStepNo()    { return active ? stepIdx + 1 : 0; }
int    navStepCount() { return active ? stepN : 0; }
NavSrc navSource()    { return src; }
void   navSetJog(int d) { jogDir = d; }
int    navJog()         { return jogDir; }

static float wrapRadN(float e) { while (e > (float)M_PI) e -= 2 * (float)M_PI; while (e < -(float)M_PI) e += 2 * (float)M_PI; return e; }

// L4 + wallF: jarak untuk keputusan menghindar. Median valid; depan bukan halangan bila itu dinding referensi waypoint (d >= wallF - WALL_OBST_DELTA).
static void obstDistFor(const Waypoint* w, const float med[3], const bool ok[3], float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = ok[i] && med[i] > 0 ? med[i] : -1.0f;
  if (w && w->wallF > 0 && out[0] > 0 && out[0] >= w->wallF - WALL_OBST_DELTA) out[0] = -1.0f;
}
void navObstDist(const float med[3], const bool ok[3], float out[3]) {
  Waypoint w; bool have = active && wpGet(ids[stepIdx], w);
  obstDistFor(have ? &w : nullptr, med, ok, out);
}
bool navObstConfirmed(const float d[3], float vx, float vy) {
  for (int i = 0; i < 3; i++) {
    if (d[i] <= 0 || d[i] > OBST_STOP_CM) continue;
    float a = SENSOR_ANG_DEG[i] * (float)M_PI / 180.0f;
    if (vx * cosf(a) + vy * sinf(a) > 0.5f) return true;
  }
  return false;
}
// L3: lantai kecepatan setelah semua pengali (satScale, tanjakan, avoidObstacles, trust). Arah dari dirX/dirY.
void navSpeedFloor(float& vx, float& vy, float dirX, float dirY, float vLimCms, bool obst) {
  if (!active || phase != PH_MOVING || obst || avoidOn) return;
  float dm = hypotf(dirX, dirY); if (dm < 1e-3f) return;
  float fl = NAV_V_MIN_CMS;
  if (lastDist > NAV_SLOW_CM) fl = fmaxf(fl, rpmToCms(NAV_SLOPE_MIN_RPM));
  fl = fminf(fl, vLimCms);
  if (hypotf(vx, vy) >= fl) return;
  vx = dirX / dm * fl; vy = dirY / dm * fl;
}

// L10: jaga jarak sisi. Zona = SIDE_MIN_CM + SIDE_BUFFER_CM; di dalam zona dorong menjauh (KP x masuk zona, dibatasi), di bawah SIDE_MIN_CM komponen menuju sisi itu dihapus.
static void sideGuard(float& vx, float& vy, bool frontBlocked, float zoneOverride = 0.0f) {
  const float zone = zoneOverride > 0 ? zoneOverride : SIDE_MIN_CM + SIDE_BUFFER_CM;   // saat geser hindar: zona = AVOID_WALL_GAP_CM
  float dl = wallOk[1] ? wallMed[1] : -1.0f, dr = wallOk[2] ? wallMed[2] : -1.0f;
  bool inL = dl > 0 && dl < zone, inR = dr > 0 && dr < zone;
  if (!inL && !inR) return;
  bool fb = frontBlocked || irF;                             // depan terhalang: HOLD atau IR depan aktif
  if (inL && inR) {                                           // kedua sisi sempit: tanpa gerak lateral, maju pelan; depan terhalang = berhenti
    vx = 0; vy = fabsf(vy) > 0.5f ? copysignf(NAV_V_MIN_CMS, vy) : 0.0f;
    if (fb && vy > 0) vy = 0;
    return;
  }
  if (inL) { if (dl < SIDE_MIN_CM && vx < 0) vx = 0; vx += fminf(SIDE_PUSH_KP * (zone - dl), SIDE_PUSH_MAX_CMS); }
  else     { if (dr < SIDE_MIN_CM && vx > 0) vx = 0; vx -= fminf(SIDE_PUSH_KP * (zone - dr), SIDE_PUSH_MAX_CMS); }
}

// PRD avoid v1.0: hindar rintangan depan bertahap. CLEAR (tren) -> HOLD (diam memastikan) -> SHIFT (geser berpulsa, tanpa maju) -> PASS (maju lurus).
// vx, vy = gerak menuju waypoint (frame badan). false = navigasi dibatalkan (terhalang terlalu lama, deny 7).
static const char* const AV_NAME[4] = {"CLEAR", "HOLD", "SHIFT", "PASS"};
static void avGo(uint8_t st, uint32_t now) { avSt = st; avoidOn = st != AV_CLEAR; avShift = avFrontShift = avMargin = false; avClearMs = 0; if (st == AV_CLEAR || st == AV_PASS) avBlind = false; (void)now; }
static float sideSel(const float& dl, const float& dr) { return avoidDir > 0 ? dr : dl; }   // bacaan sisi tujuan geser (+ = kanan)
static bool sideFree(float d) { return d > SIDE_FREE_CM; }                                  // tidak valid (-1) = tidak layak

// H1: tren penurunan bacaan depan sebanding jarak tempuh. true = halangan nyata.
static bool trendStep(float x, float y, float d0now, float far) {
  if (!trendOn) { if (d0now > 0) { trendOn = true; trAx = x; trAy = y; trD0 = d0now; trCnt = 0; trPrev = d0now; } return false; }
  float tr = hypotf(x - trAx, y - trAy);
  float prev = trPrev; if (d0now > 0) trPrev = d0now;
  // halangan nyata tidak menjauh saat robot maju: bacaan naik > 3 cm dari tick sebelumnya = bukan tren
  if (d0now > 0 && prev > 0 && d0now - prev > 3.0f) { trCnt = 0; trAx = x; trAy = y; trD0 = d0now; return false; }
  // bacaan harus ikut turun sepanjang langkah (bukan hanya di ujungnya): bacaan palsu acak hampir selalu keluar jalur sebelum 10 cm
  if (tr >= 3.0f && d0now > 0 && fabsf((trD0 - d0now) - tr) > TREND_TOL_CM) { trCnt = 0; trAx = x; trAy = y; trD0 = d0now; return false; }
  if (tr < TREND_STEP_CM) return false;
  bool pass = d0now > 0 && trD0 > 0 && fabsf((trD0 - d0now) - tr) <= TREND_TOL_CM && d0now < far;
  if (pass) { trCnt++; trD0 = d0now; trAx = x; trAy = y; return trCnt >= TREND_N; }
  trCnt = 0; trendOn = false;                                  // gagal: hitung ulang dari nol (jangkar baru bila ada bacaan)
  if (d0now > 0) { trendOn = true; trAx = x; trAy = y; trD0 = d0now; }
  return false;
}

static bool avoidCore(const Waypoint& w, float x, float y, float& vx, float& vy, uint32_t now, uint32_t stepDt) {
  float d[3]; obstDistFor(&w, wallMed, wallOk, d);
  float d0 = d[0];                                             // ultrasonik depan (dinding referensi waypoint dikecualikan)
  float wvx = vx, wvy = vy;                                    // gerak menuju waypoint (sebelum diubah)
  float dl = wallOk[1] ? wallMed[1] : -1.0f, dr = wallOk[2] ? wallMed[2] : -1.0f;
  bool sIrL = irL && !(dl > 0 && dl < IR_SIDE_WALL_IGNORE_CM), sIrR = irR && !(dr > 0 && dr < IR_SIDE_WALL_IGNORE_CM);   // IR sisi dekat dinding diabaikan

  if (avSt == AV_CLEAR || avSt == AV_PASS) {
    if (avSt == AV_PASS) {                                      // PASS: maju lurus; halangan baru -> deteksi lagi
      float run = hypotf(x - avSx, y - avSy);
      if (run >= AVOID_PASS_CM || now - avPulseMs > (uint32_t)(2000.0f * AVOID_PASS_CM / AVOID_WATCH_CMS) + 1000u) {
        Serial.printf("[AVOID] PASS selesai %.0f cm\n", run); avGo(AV_CLEAR, now); avoidHoldUntil = now + AVOID_HOLD_MS; return true;
      }
    }
    bool fwd = (avSt == AV_PASS) ? true : wvy > 1.0f;
    lastAvoidScale = 1.0f;
    if (!fwd) { trendOn = false; trCnt = 0; warn = false; return true; }
    // Peringatan dini (ultrasonik depan): bacaan turun terus sebanding jarak tempuh = ada yang didekati -> melambat bertahap. Bacaan acak (palsu) tidak lolos.
    bool tUp = trendStep(x, y, d0, WARN_FAR_CM);
    if (tUp && !warn) { warn = true; warnLostMs = 0; warnMin = d0; warnDropMs = now; Serial.printf("[WARN] ultrasonik depan turun konsisten d=%.0f\n", d0); }
    if (warn) {
      if (d0 > 0) {
        warnLostMs = 0;
        if (d0 < warnMin - 1.0f) { warnMin = d0; warnDropMs = now; }
        if (d0 > warnMin + WARN_RISE_CM || d0 > WARN_FAR_CM + 10.0f || now - warnDropMs > WARN_STALL_MS) { warn = false; trendOn = false; trCnt = 0; Serial.println("[WARN] batal"); }
      } else {
        if (!warnLostMs) warnLostMs = now;
        if (now - warnLostMs > WARN_LOST_MS) { warn = false; trendOn = false; trCnt = 0; }
      }
    }
    bool blind = warn && d0 > 0 && d0 <= AVOID_BLIND_CM;        // tren terus sampai sangat dekat tapi IR diam: benda tidak terlihat IR (hitam/matte)
    bool shiftNow = false;
    if (irF || blind) {                                         // keputusan hindar: IR depan (atau cadangan ultrasonik untuk benda tak terlihat IR)
      Serial.printf("[AVOID] HOLD %s d=%.0f\n", irF ? "ir depan" : "ultrasonik (IR tak melihat)", d0);
      avGo(AV_HOLD, now); avHoldMs = now; avConfirmed = false; avBlind = !irF; warn = false; trendOn = false; trCnt = 0;
    } else if (sIrL || sIrR) {                                  // IR sisi (menghadap depan): halangan di samping jalur yang tidak terlihat IR depan
      int dir = (sIrL && sIrR) ? 0 : (sIrR ? -1 : 1);           // IR kanan aktif -> geser kiri
      float sd = dir > 0 ? dr : dl;
      if (dir != 0 && sideFree(sd)) {
        avoidDir = dir; Serial.printf("[AVOID] IR %s, geser %s\n", dir > 0 ? "kiri" : "kanan", dir > 0 ? "kanan" : "kiri");
        avGo(AV_SHIFT, now); avShift = true; avOx = x; avOy = y; avLat = 0; avTarget = AVOID_IR_SHIFT_CM; avPulseMs = now; shiftNow = true; warn = false;
      } else {                                                  // dua sisi aktif, atau sisi tujuan tidak layak: diam, batal bila terlalu lama
        legMs += stepDt; avoidTotalMs += stepDt;
        if (avoidTotalMs > AVOID_MAX_MS) { sess[1]++; navPostDeny(7); navCancel(); return false; }
        vx = vy = 0; return true;
      }
    } else {
      if (avSt == AV_PASS) { vx = 0; vy = AVOID_WATCH_CMS; }
      if (warn && d0 > 0) {                                     // melambat bertahap: AVOID_CRAWL_CMS di WARN_CRAWL_CM, kecepatan penuh di WARN_FAR_CM
        float m = hypotf(vx, vy), f = fminf(fmaxf((d0 - WARN_CRAWL_CM) / (WARN_FAR_CM - WARN_CRAWL_CM), 0.0f), 1.0f);
        float cap = AVOID_CRAWL_CMS + (m - AVOID_CRAWL_CMS) * f;
        if (m > cap && m > 0) { float k = cap / m; vx *= k; vy *= k; lastAvoidScale = k; }
      }
      return true;
    }
    (void)shiftNow;
  }

  legMs += stepDt; avoidTotalMs += stepDt;                      // timer leg diperpanjang sebesar lama AVOID
  if (avoidTotalMs > AVOID_MAX_MS) { sess[1]++; navPostDeny(7); navCancel(); return false; }
  vx = vy = 0;
  bool fReal = irF || (avBlind && d0 > 0 && d0 <= AVOID_BLIND_CM + 10.0f);   // halangan depan masih ada (IR, atau ultrasonik bila dipicu benda tak terlihat IR)

  if (avSt == AV_HOLD) {                                        // berhenti sebentar (mengerem); IR depan bebas = benda pergi; masih aktif -> pilih sisi, geser langsung
    if (now - avHoldMs < AVOID_CONFIRM_MS) return true;
    if (!fReal) { Serial.println("[AVOID] depan bebas, lanjut"); avGo(AV_CLEAR, now); avoidHoldUntil = now + AVOID_HOLD_MS; return true; }
    bool fl = sideFree(dl), fr = sideFree(dr);                  // sisi layak, bacaan lebih besar
    if (!fl && !fr) return true;                                // tidak ada: tetap diam (batal lewat AVOID_MAX_MS, atau lanjut bila IR bebas)
    if (fl && fr) avoidDir = fabsf(dl - dr) <= 3.0f ? (wvx >= 0 ? 1 : -1) : (dr > dl ? 1 : -1);
    else avoidDir = fr ? 1 : -1;
    Serial.printf("[AVOID] SIDE kiri=%.0f kanan=%.0f pilih=%s\n", dl, dr, avoidDir > 0 ? "kanan" : "kiri");
    avGo(AV_SHIFT, now); avShift = avFrontShift = true; avOx = x; avOy = y; avLat = 0; avTarget = AVOID_SHIFT_MIN_CM; avPulseMs = now;
    return true;
  }

  // SHIFT: geser menerus (tanpa maju) sampai IR pemicu bebas IR_CLEAR_MS; total >= AVOID_SHIFT_MIN_CM (depan) / AVOID_IR_SHIFT_CM (sisi) dan margin setelah bebas.
  // Berhenti lebih awal bila jarak dinding sisi tujuan <= AVOID_WALL_GAP_CM.
  float run = avLat, gap = sideSel(dl, dr);                     // run = total geser samping
  bool trig = avFrontShift ? fReal : (avoidDir > 0 ? irL : irR);  // IR pemicu (sisi: ada di sisi berlawanan arah geser)
  if (trig) avClearMs = 0; else if (!avClearMs) avClearMs = now;
  if (!avMargin && avClearMs && now - avClearMs >= IR_CLEAR_MS) {
    avMargin = true; avTarget = fmaxf(run + (avFrontShift ? AVOID_EXTRA_CM : AVOID_IR_MARGIN_CM), avTarget);
  }
  const float brakeCm = AVOID_SIDE_CMS * AVOID_SIDE_CMS / (2.0f * AVOID_BRAKE_CMS2);   // jarak berhenti geser samping (pembatas percepatan): jarak dinding minimum tetap terjaga
  bool wallStop = gap > 0 && gap <= AVOID_WALL_GAP_CM + brakeCm, timeout = now - avPulseMs > 4000u;
  if (wallStop && trig && avFrontShift) {                       // di batas jarak dinding tapi depan masih tertutup: pilih ulang sisi lain
    Serial.println("[AVOID] mentok jarak dinding, pilih ulang"); avGo(AV_HOLD, now); avHoldMs = now - AVOID_CONFIRM_MS; return true;
  }
  if ((avMargin && run >= avTarget) || wallStop || timeout) {
    Serial.printf("[AVOID] geser selesai %.0f cm -> PASS\n", run); avGo(AV_PASS, now); avSx = x; avSy = y; avPulseMs = now; return true;
  }
  vx = avoidDir * AVOID_SIDE_CMS; return true;
}

// Gerak halus hanya saat peringatan/hindar aktif: pembatas percepatan pada (vx, vy). Perintah jalan normal tidak disentuh; setelah hindar selesai kecepatan kembali ke perintah normal secara bertahap.
static bool applyAvoid(const Waypoint& w, float x, float y, float& vx, float& vy, uint32_t now, uint32_t stepDt) {
  float cx = vx, cy = vy;                                      // perintah jalan normal menuju waypoint
  if (!avoidCore(w, x, y, vx, vy, now, stepDt)) { slOn = false; return false; }
  bool engaged = warn || avSt != AV_CLEAR;
  if (engaged && !slOn) { slOn = true; slVx = cx; slVy = cy; } // mulai dari kecepatan jalan normal: tanpa lonjakan
  if (!slOn) return true;
  float dt = stepDt / 1000.0f, brake = irF ? AVOID_EBRAKE_CMS2 : AVOID_BRAKE_CMS2;
  float lx = (fabsf(vx) < fabsf(slVx) ? brake : AVOID_ACCEL_CMS2) * dt, ly = (fabsf(vy) < fabsf(slVy) ? brake : AVOID_ACCEL_CMS2) * dt;   // per sumbu: mengerem tegas, menambah kecepatan lembut
  slVx += fminf(fmaxf(vx - slVx, -lx), lx); slVy += fminf(fmaxf(vy - slVy, -ly), ly);
  if (!engaged && slVx == vx && slVy == vy) slOn = false;       // sudah mengejar perintah dan hindar selesai: kembali normal
  vx = slVx; vy = slVy;
  if (avSt == AV_SHIFT) avLat += fabsf(vx) * dt;
  return true;
}

// ---- L6 VERIFY ----
struct VRes { bool lat = false, fwd = false; float oLat = 0, oFwd = 0, m[3] = {-1, -1, -1}; };
// oLat: + = robot di kanan ref (rata-rata berbobot 1/ref, seperti wallCalc). oFwd: + = robot di depan ref (dinding depan lebih dekat dari wallF).
static VRes verifyEval(const Waypoint& w, float yaw) {
  VRes r; const float D2R = (float)M_PI / 180.0f;
  float th = w.hasYaw ? wrapRadN(yaw - w.yaw * D2R) : 0.0f;
  for (int i = 0; i < 3; i++) r.m[i] = wallOk[i] ? wallMed[i] : -1.0f;
  r.m[0] = r.m[1]; r.m[1] = r.m[2]; r.m[2] = wallOk[0] ? wallMed[0] : -1.0f;   // urutan arr: kiri, kanan, depan
  if (fabsf(th) > WALL_YAW_MAX_DEG * D2R) return r;           // arah menyimpang: bacaan ditolak (sama dengan wallCalc)
  float sum = 0, wsum = 0;
  for (int k = 0; k < 2; k++) {
    float ref = k == 0 ? w.wallL : w.wallR;
    if (ref <= 0 || !wallOk[k + 1]) continue;
    float o = wallMed[k + 1] * cosf(th) - ref;
    sum += (k == 0 ? o : -o) / ref; wsum += 1.0f / ref;
  }
  if (wsum > 0) { r.lat = true; r.oLat = sum / wsum; }
  if (w.wallF > 0 && wallOk[0]) { r.fwd = true; r.oFwd = w.wallF - wallMed[0] * cosf(th); }
  return r;
}
static void logArr(const Waypoint& w, const VRes& r, uint8_t result) {
  portENTER_CRITICAL(&nmux);
  arrLog.seq++; arrLog.id = (uint8_t)target; arrLog.verifyN = vIter; arrLog.result = result;
  for (int i = 0; i < 3; i++) arrLog.m[i] = r.m[i];
  arrLog.ref[0] = w.wallL; arrLog.ref[1] = w.wallR; arrLog.ref[2] = w.wallF;
  arrLog.resLat = r.lat ? r.oLat : 0.0f; arrLog.resFwd = r.fwd ? r.oFwd : 0.0f;
  portEXIT_CRITICAL(&nmux);
}
// Satu waypoint tuntas: lanjut ke waypoint berikutnya (true) atau selesai (false).
static bool completeWp() {
  if (stepIdx >= stepN - 1) { navFinish(); return false; }
  stepIdx++; settleMs = 0;
  if (isGripStep(ids[stepIdx])) { enterGrip(); return true; }                  // langkah gripper: robot diam
  target = ids[stepIdx]; legStart(); phase = PH_MOVING;
  return true;
}
// Posisi dan yaw waypoint sudah tercapai: VERIFY bila ada referensi dinding, selain itu selesai seperti sebelumnya.
static bool afterArrive(const Waypoint& w, bool imuOk) {
  if (hasRef(w) && imuOk) { vStartMs = millis(); vIter = 0; vShift = false; phase = PH_VERIFY; return true; }
  VRes none; logArr(w, none, 0);
  return completeWp();
}

bool navStep(float yaw, float rateDps, float vCms, bool imuOk, float& vx, float& vy, float& yawTarget, bool& rotating) {
  vx = vy = 0; rotating = false; yawTarget = yaw;
  if (!active) return false;
  float x, y; navPose(x, y);
  uint32_t nowT = millis();
  if (phase == PH_MOVING || phase == PH_VERIFY) {              // L8: bacaan sisi terkecil selama rute
    for (int i = 1; i <= 2; i++) if (wallOk[i] && wallMed[i] > 0 && (sideMinSeen < 0 || wallMed[i] < sideMinSeen)) sideMinSeen = wallMed[i];
  }

  if (phase == PH_GRIP) {                               // STATE GRIP: robot diam sampai solenoid selesai bekerja
    if (millis() - gripStartMs < GRIPPER_ACTUATE_MS) return true;
    return completeWp();
  }

  if (phase == PH_VERIFY) {                             // STATE VERIFY: bacaan sisi / depan harus cocok dengan data waypoint
    Waypoint w;
    if (!wpGet(ids[stepIdx], w) || !w.valid || !imuOk) { navCancel(); return false; }
    VRes r = verifyEval(w, yaw);
    if (!vShift) {
      if (nowT - vStartMs < NAV_VERIFY_SETTLE_MS) return true;                              // motor translasi 0
      if (!r.lat && !r.fwd) { logArr(w, r, 0); return completeWp(); }                       // sensor tidak valid / arah menyimpang: lewati
      float eL = r.lat ? fabsf(r.oLat) : 0.0f, eF = r.fwd ? fabsf(r.oFwd) : 0.0f;
      if (eL <= WALL_ARRIVE_TOL_CM && eF <= WALL_ARRIVE_TOL_CM) { logArr(w, r, 1); return completeWp(); }   // cocok
      if (eL > WALL_CORR_MAX_CM || eF > WALL_CORR_MAX_CM || vIter >= NAV_VERIFY_MAX) { logArr(w, r, 2); return completeWp(); }   // ditolak / jatah habis: selesai bertanda tidak cocok
      vIter++;
      float c = cosf(yaw), s = sinf(yaw);
      float dl = r.lat ? r.oLat - ((x - w.x) * c + (y - w.y) * s) : 0.0f;                   // koreksi pose sebesar residual
      float df = r.fwd ? r.oFwd - (-(x - w.x) * s + (y - w.y) * c) : 0.0f;
      portENTER_CRITICAL(&nmux); px += dl * c - df * s; py += dl * s + df * c; portEXIT_CRITICAL(&nmux);
      vShift = true; vShiftEnd = nowT + 1500u + (uint32_t)(1000.0f * fmaxf(eL, eF) / NAV_V_MIN_CMS);
      return true;
    }
    float eL = r.lat ? fabsf(r.oLat) : 0.0f, eF = r.fwd ? fabsf(r.oFwd) : 0.0f;
    if ((!r.lat && !r.fwd) || nowT >= vShiftEnd || (eL <= WALL_ARRIVE_TOL_CM * 0.5f && eF <= WALL_ARRIVE_TOL_CM * 0.5f)) { vShift = false; vStartMs = nowT; return true; }   // geser selesai: diam, nilai lagi
    if (r.lat && eL > WALL_ARRIVE_TOL_CM * 0.5f) vx = r.oLat > 0 ? -NAV_V_MIN_CMS : NAV_V_MIN_CMS;  // bergeser halus sampai bacaan = ref
    if (r.fwd && eF > WALL_ARRIVE_TOL_CM * 0.5f) vy = r.oFwd > 0 ? -NAV_V_MIN_CMS : NAV_V_MIN_CMS;
    sideGuard(vx, vy, false);
    return true;
  }

  if (phase == PH_ROTATING) {                           // STATE ROTATING: badan berputar di tempat, motor translasi nol
    Waypoint w; uint32_t now = millis();
    if (!wpGet(ids[stepIdx], w) || !w.valid || now - rotStartMs > NAV_ROT_TIMEOUT_MS || !imuOk) { navCancel(); return false; }
    float tgt = w.yaw * (float)M_PI / 180.0f;
    float e = tgt - yaw; while (e > M_PI) e -= 2 * M_PI; while (e < -M_PI) e += 2 * M_PI;    // normalisasi -180..+180 (359 vs 1 = 2 derajat)
    float eDeg = e * 180.0f / (float)M_PI;
    yawErrF += NAV_YAW_LPF * (eDeg - yawErrF);                                                // filter jitter IMU untuk keputusan selesai
    if (fabsf(yawErrF) <= NAV_YAW_TOL_DEG && fabsf(eDeg) <= NAV_YAW_TOL_DEG * 1.5f && fabsf(rateDps) < NAV_YAW_RATE_OK_DPS) {
      if (!settleMs) settleMs = now;
      if (now - settleMs >= NAV_YAW_SETTLE_MS) return afterArrive(w, imuOk);                         // yaw tercapai: VERIFY (bila ada ref) / DONE / waypoint berikutnya (tick ini diam)
    } else settleMs = 0;
    rotating = true; yawTarget = tgt;
    return true;
  }

  for (int guard = 0; guard <= MAX_RT; guard++) {       // STATE MOVING
    Waypoint w;
    if (!wpGet(ids[stepIdx], w) || !w.valid) { navCancel(); return false; }
    if (millis() - legMs > NAV_TIMEOUT_MS && !legRetry()) return false;
    float dx = w.x - x, dy = w.y - y, d = hypotf(dx, dy);
    lastDist = d;
    bool last = stepIdx >= stepN - 1;
    bool nextGrip = !last && isGripStep(ids[stepIdx + 1]);
    bool stopHere = last || nextGrip || (w.hasYaw && imuOk);   // waypoint terakhir, sebelum langkah gripper, atau punya yaw: berhenti dulu
    if (d <= (stopHere ? NAV_ARRIVE_CM : NAV_PASS_CM)) {
      if (stopHere) {                                   // tiba: motor berhenti, lalu putar ke arah waypoint bila ada
        if (w.hasYaw && imuOk) {
          phase = PH_ROTATING; rotStartMs = millis(); settleMs = 0;
          float e = w.yaw * (float)M_PI / 180.0f - yaw; while (e > M_PI) e -= 2 * M_PI; while (e < -M_PI) e += 2 * M_PI;
          yawErrF = e * 180.0f / (float)M_PI;
          rotating = true; yawTarget = w.yaw * (float)M_PI / 180.0f; return true;
        }
        return afterArrive(w, imuOk);                   // tanpa arah hadap: VERIFY (bila ada ref) atau selesai di posisi
      }
      stepIdx++; target = ids[stepIdx]; legStart(); // lanjut ke waypoint berikutnya tanpa berhenti
      continue;
    }
    float sp = vCms;
    if (stopHere) {                                      // melambat di waypoint terakhir dan waypoint ber-yaw
      float slow = fmaxf(NAV_SLOW_CM, vCms * NAV_SLOW_S);
      if (d < slow) sp = fmaxf(NAV_V_MIN_CMS, vCms * d / slow);
    }
    sp = fminf(sp, vCms);
    float wx = dx / d * sp, wy = dy / d * sp;
    float c = cosf(yaw), s = sinf(yaw);
    vx = wx * c + wy * s; vy = -wx * s + wy * c;         // dunia -> badan
    uint32_t stepDt = lastStepMs ? nowT - lastStepMs : 0; lastStepMs = nowT; if (stepDt > 200) stepDt = 200;
    WallCorr wc = wallCalc(w, x, y, yaw, wallMed, wallOk, wallTilt);   // R3: dorongan steering dinding (otonom MOVING saja)
    if (wc.stat != 0 && navAvoiding()) wc = {0, 0, 0, 4};              // L9: ditahan selama AVOID + AVOID_HOLD_MS
    if (wc.stat == 1) { wallVxHold = wc.vx; wallVxMs = nowT; }
    else if (wc.stat == 2 && !wallTilt && wallVxMs && nowT - wallVxMs < WALL_HOLD_MS) wc.vx = wallVxHold;   // sensor sisi putus sesaat: pertahankan koreksi terakhir
    wallStat = wc.stat; vx += wc.vx;
    if (wc.stat == 3) { if (!staleSince) staleSince = nowT; else if (nowT - staleSince > 1000) refStale = true; }   // L5: ref tidak cocok > 1 s
    else { staleSince = 0; refStale = false; }
    if (!applyAvoid(w, x, y, vx, vy, nowT, stepDt)) return false;      // PRD avoid v1.0 (H1-H5)
    if (wc.stat != 0 && navAvoiding()) { wallStat = 4; }   // AVOID baru mulai di tick ini: status dan steering dinding ikut ditahan
    sideGuard(vx, vy, avSt == AV_HOLD, avSt == AV_SHIFT ? AVOID_WALL_GAP_CM : 0.0f);                                // L10: prioritas di atas koreksi dinding dan arah waypoint
    return true;
  }
  navCancel(); return false;
}

// R3: offset = d_med*cos(dYaw) - ref (+ = menjauh dari dinding). oLat = offset lateral badan (+x kanan), rata-rata berbobot 1/ref.
// Pose: lateral pose terhadap waypoint dipaksa = oLat (waypoint direkam pada jarak ref). Dinding tidak valid -> stat 2, encoder murni.
WallCorr wallCalc(const Waypoint& w, float x, float y, float yaw, const float med[3], const bool ok[3], bool tilt) {
  WallCorr r = {0, 0, 0, 0};
  const float refs[2] = {w.wallL, w.wallR}, D2R = (float)M_PI / 180.0f;
  float th = 0;
  if (w.hasYaw) { th = yaw - w.yaw * D2R; while (th > (float)M_PI) th -= 2 * (float)M_PI; while (th < -(float)M_PI) th += 2 * (float)M_PI; }
  float sum = 0, wsum = 0; bool want = false, big = false;
  for (int k = 0; k < 2; k++) {
    if (refs[k] <= 0) continue;
    want = true;
    if (!ok[k + 1] || fabsf(th) > WALL_YAW_MAX_DEG * D2R) continue;
    float o = med[k + 1] * cosf(th) - refs[k];
    if (fabsf(o) > WALL_CORR_MAX_CM) { big = true; continue; }
    sum += (k == 0 ? o : -o) / refs[k]; wsum += 1.0f / refs[k];
  }
  if (!want) return r;
  if (tilt) { r.stat = 2; return r; }                    // L4: miring samping (roll) mengubah geometri sensor sisi: ditahan. SLOPE (pitch) tidak.
  if (wsum <= 0) { r.stat = big ? 3 : 2; return r; }
  float oLat = sum / wsum, c = cosf(yaw), s = sinf(yaw);
  float dl = oLat - ((x - w.x) * c + (y - w.y) * s);
  if (fabsf(dl) > WALL_CORR_MAX_CM) { r.stat = 3; return r; }
  r.dpx = dl * c; r.dpy = dl * s;
  float e = fabsf(oLat) - WALL_DEADZONE_CM;
  if (e > 0) r.vx = constrain(-WALL_STEER_KP * (oLat > 0 ? e : -e), -WALL_STEER_MAX_CMS, WALL_STEER_MAX_CMS);
  r.stat = 1; return r;
}

// Vector deflection: buang komponen menuju rintangan, geser ke samping, tanpa mengubah arah hadap.
// Jarak mulai menghindar membesar bersama kecepatan.
void avoidObstacles(float& vx, float& vy, const float d[3], const float wallRef[2]) {
  float vmag = hypotf(vx, vy);
  float farD = OBST_DIST_CM + OBST_SPD_GAIN * vmag, nearD = OBST_STOP_CM + OBST_SPD_GAIN * 0.4f * vmag;
  float scale = 1.0f;
  lastAvoidScale = 1.0f;
  for (int i = 0; i < 3; i++) {
    if (d[i] <= 0 || d[i] >= farD) continue;
    if (wallRef && i > 0 && wallRef[i - 1] > 0 && d[i] >= wallRef[i - 1] - WALL_OBST_DELTA) continue;   // R4: dinding normal bukan halangan
    float a = SENSOR_ANG_DEG[i] * (float)M_PI / 180.0f;
    float ux = cosf(a), uy = sinf(a);
    float prox = constrain((farD - d[i]) / (farD - nearD), 0.0f, 1.0f);
    float comp = vx * ux + vy * uy;
    if (comp <= 0) continue;                              // sedang menjauh
    float tx = -uy, ty = ux;
    float sgn = (vx * tx + vy * ty) >= 0 ? 1.0f : -1.0f;   // geser ke sisi yang searah tujuan
    vx += -comp * prox * ux + sgn * comp * prox * tx;
    vy += -comp * prox * uy + sgn * comp * prox * ty;
    scale = fminf(scale, fmaxf(0.3f, 1.0f - 0.7f * prox));
  }
  vx *= scale; vy *= scale; lastAvoidScale = scale;
}

// ---------------- permintaan lintas-thread ----------------
void navRequestGoto(int id, NavSrc s)  { portENTER_CRITICAL(&nmux); req.gotoId = id; req.route = -1; req.src = s; portEXIT_CRITICAL(&nmux); }
void navRequestRoute(int r, NavSrc s)  { portENTER_CRITICAL(&nmux); req.route = r; req.gotoId = 0; req.src = s; portEXIT_CRITICAL(&nmux); }
void navRequestCancel()                { portENTER_CRITICAL(&nmux); req.cancel = true; portEXIT_CRITICAL(&nmux); }
void navRequestZero()                  { portENTER_CRITICAL(&nmux); req.zero = true; portEXIT_CRITICAL(&nmux); }
void navRequestResume()                { portENTER_CRITICAL(&nmux); req.resume = true; portEXIT_CRITICAL(&nmux); }
void navRequestJog(int d)              { portENTER_CRITICAL(&nmux); req.jogSet = true; req.jog = d; portEXIT_CRITICAL(&nmux); }
void navRequestStopSave(int id)        { portENTER_CRITICAL(&nmux); req.stopSave = id; req.jogSet = true; req.jog = 0; portEXIT_CRITICAL(&nmux); }
void navRequestPreset(int i, NavSrc s) { portENTER_CRITICAL(&nmux); req.preset = i; req.src = s; portEXIT_CRITICAL(&nmux); }
void navRequestSync(int id, NavSrc s)   { portENTER_CRITICAL(&nmux); req.sync = id; req.src = s; portEXIT_CRITICAL(&nmux); }
static volatile uint8_t extDeny = 0;
void navPostDeny(uint8_t why)          { extDeny = why; }
bool navTakeDeny(uint8_t& why)         { why = extDeny; extDeny = 0; return why != 0; }
bool navTakeGrip(bool& on)             { if (!gripPend) return false; gripPend = false; on = gripPendOn; return true; }
void navRequestGrip(bool on)           { portENTER_CRITICAL(&nmux); req.grip = on ? 1 : 0; portEXIT_CRITICAL(&nmux); }
void navConsumeRequests(NavReq& r) {
  portENTER_CRITICAL(&nmux);
  r = req; req = NavReq();
  portEXIT_CRITICAL(&nmux);
}

// ---------------- waypoint dan rute ----------------
bool wpGet(int id, Waypoint& w) {
  if (id < 1 || id > MAX_WP) return false;
  portENTER_CRITICAL(&nmux); w = wps[id - 1]; portEXIT_CRITICAL(&nmux);
  return true;
}
static float wrap180(float d) { while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f; return d; }
bool wpSet(int id, float x, float y, float yawDeg, bool hasYaw, float wallL, float wallR, float wallF) {
  if (id < 1 || id > MAX_WP) return false;
  portENTER_CRITICAL(&nmux); wps[id - 1] = {true, x, y, hasYaw ? wrap180(yawDeg) : 0.0f, hasYaw, wallL > 0 ? wallL : 0.0f, wallR > 0 ? wallR : 0.0f, wallF > 0 ? wallF : 0.0f}; portEXIT_CRITICAL(&nmux);
  saveWps(); return true;
}
bool wpSetKeepYaw(int id, float x, float y, float wallL, float wallR, float wallF) {
  if (id < 1 || id > MAX_WP) return false;
  portENTER_CRITICAL(&nmux); Waypoint o = wps[id - 1]; wps[id - 1] = {true, x, y, o.valid ? o.yaw : 0.0f, o.valid && o.hasYaw, wallL > 0 ? wallL : 0.0f, wallR > 0 ? wallR : 0.0f, wallF > 0 ? wallF : 0.0f}; portEXIT_CRITICAL(&nmux);
  saveWps(); return true;
}
bool wpDelete(int id) {
  if (id < 1 || id > MAX_WP) return false;
  portENTER_CRITICAL(&nmux); wps[id - 1] = {false, 0, 0, 0, false, 0, 0, 0}; portEXIT_CRITICAL(&nmux);
  saveWps(); return true;
}
void wpCopyAll(Waypoint out[MAX_WP]) { portENTER_CRITICAL(&nmux); memcpy(out, wps, sizeof(wps)); portEXIT_CRITICAL(&nmux); }

bool routeSet(int r, const uint8_t* in, int n) {
  if (r < 0 || r >= NUM_ROUTES) return false;
  uint8_t tmp[MAX_RT]; memset(tmp, 0, sizeof tmp);
  int m = 0;
  for (int k = 0; k < n && m < MAX_RT; k++) if ((in[k] >= 1 && in[k] <= MAX_WP) || isGripStep(in[k])) tmp[m++] = in[k];
  portENTER_CRITICAL(&nmux); memcpy(routes[r], tmp, MAX_RT); portEXIT_CRITICAL(&nmux);
  saveRoutes(); return true;
}
void routeCopyAll(uint8_t out[NUM_ROUTES][MAX_RT]) { portENTER_CRITICAL(&nmux); memcpy(out, routes, sizeof(routes)); portEXIT_CRITICAL(&nmux); }

// ---------------- simpan posisi terakhir ----------------
// R7: tulis hanya bila diam dan bergeser > POSE_SAVE_CM / POSE_SAVE_DEG sejak tulis terakhir, atau flag validitas berubah; jeda minimum POSE_SAVE_GAP_MS.
struct PoseSaveState { float lx = 0, ly = 0, lyaw = 0; bool lvalid = true, first = true; uint32_t lastMs = 0; };
static bool poseSaveDue(PoseSaveState& s, const Telemetry& t, uint32_t now) {
  if (!t.ready) return false;                            // jangan menimpa posisi tersimpan sebelum task kontrol hidup
  if (s.first) { s.lx = t.x; s.ly = t.y; s.lyaw = t.yawDeg; s.lvalid = !t.poseLost; s.first = false; return false; }
  bool still = t.mode != 1 && t.mode != 3 && t.mode != 4 && hypotf(t.mvx, t.mvy) < 2.0f && hypotf(t.vx, t.vy) < 1.0f && fabsf(t.wz) < 0.05f;
  bool changed = hypotf(t.x - s.lx, t.y - s.ly) > POSE_SAVE_CM || fabsf(t.yawDeg - s.lyaw) > POSE_SAVE_DEG;
  bool flag = (!t.poseLost) != s.lvalid;
  if (!(((still && changed) || flag) && now - s.lastMs > (flag ? 300u : POSE_SAVE_GAP_MS))) return false;
  s.lx = t.x; s.ly = t.y; s.lyaw = t.yawDeg; s.lvalid = !t.poseLost; s.lastMs = now;
  return true;
}
static void poseTask(void*) {
  PoseSaveState st;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Telemetry t = telemetryGet();
    if (!poseSaveDue(st, t, millis())) continue;
    PoseBlob pb = {t.x, t.y, t.yawDeg * (float)M_PI / 180.0f, t.poseLost ? 0u : 1u, POSE_MAGIC};
    storeWrite("nav", "pose", &pb, sizeof pb, 3);
  }
}
void navStartPoseSaver() { xTaskCreatePinnedToCore(poseTask, "posesave", 4096, nullptr, 1, nullptr, 0); }
