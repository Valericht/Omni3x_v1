// =====================================================================
//  Omni-3X  -  Single Controller ESP32-S3
//  Core 0 : USB Host (gamepad), Web/WebSocket + link kabel (Serial JSON), scheduler ultrasonik, simpan posisi
//  Core 1 : encoder ISR, PID 50 Hz, kinematika, IMU heading lock, navigasi, pengaman
// =====================================================================
#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "imu.h"
#include "imuassist.h"
#include "ultrasonic.h"
#include "ir.h"
#include "navigation.h"
#include "gripper.h"
#include "movecheck.h"
#include "gamepad_usb.h"
#include "safety.h"
#include "settings.h"
#include "store.h"
#include "webui.h"

// ---------------- telemetri ----------------
static Telemetry tel;
static portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;
void telemetrySet(const Telemetry& t) { portENTER_CRITICAL(&telMux); tel = t; portEXIT_CRITICAL(&telMux); }
Telemetry telemetryGet()              { portENTER_CRITICAL(&telMux); Telemetry t = tel; portEXIT_CRITICAL(&telMux); return t; }
volatile uint32_t g_hbWifiMs = 0, g_hbSerialMs = 0;

static TaskHandle_t ctrlTask;
static volatile bool poseDegraded = false;   // A.5.4: pose uncertain (odometry frozen / trust low) until flat and still, or ramp snap
bool isPoseDegraded() { return poseDegraded; }

// ---------------- callback USB (Core 0) : Emergency Override < 10 ms ----------------
// Begitu stick digerakkan / B ditekan saat otonom, tes gerak, atau jalan dashboard, batalkan dan bangunkan task kontrol seketika
// (tanpa menunggu tick 20 ms berikutnya).
static void onGamepad(const GamepadState& g) {
  bool stick = hypotf(g.lx, g.ly) > PAD_DEADZONE || hypotf(g.rx, g.ry) > PAD_DEADZONE;
  if (!g.connected || ((navActive() || mcActive() || navJog()) && (stick || (g.buttons & BTN_B)))) {
    navRequestCancel();
    xTaskNotifyGive(ctrlTask);
  }
}

// ---------------- util ----------------
static float wrapPi(float a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }
static void deadzone(float& x, float& y) {                // deadzone radial + penskalaan
  float m = hypotf(x, y);
  if (m < PAD_DEADZONE) { x = y = 0; return; }
  float s = fminf(1.0f, (m - PAD_DEADZONE) / (1.0f - PAD_DEADZONE));
  x *= s / m; y *= s / m;
}
// PID sudut: suku D memakai laju gyro langsung (lebih halus daripada menurunkan yaw)
struct HeadPid {
  float i = 0;
  void reset() { i = 0; }
  // pTerm = suku P sudah jadi (rad/s); kiAdd = tambahan di atas HEAD_KI; kdScale = pengali HEAD_KD (gain hasil auto-tune yaw)
  float stepP(float pTerm, float e, float rateRad, float dt, float kiAdd, float kdScale) {
    i = constrain(i + (HEAD_KI + kiAdd) * e * dt, -HEAD_I_MAX, HEAD_I_MAX);
    return constrain(pTerm + i - HEAD_KD * kdScale * rateRad, -HEAD_WZ_MAX, HEAD_WZ_MAX);
  }
  float step(float e, float rateRad, float dt, float kpScale = 1.0f, float kiAdd = 0.0f, float kdScale = 1.0f) {
    return stepP(HEAD_KP * kpScale * e, e, rateRad, dt, kiAdd, kdScale);
  }
};

// ---------------- task kontrol (Core 1) ----------------
static void controlTask(void*) {
  motorsBegin();                         // ISR encoder ter-attach ke Core 1
  bool imuOkFlag = imuBegin();
  Serial.printf("[IMU] %s\n", imuOkFlag ? "BNO055 OK" : "TIDAK TERDETEKSI (heading lock off, mencoba lagi tiap 1 detik)");

  // pulihkan posisi terakhir (x, y sudah dimuat navBegin; arah dipulihkan di sini)
  bool poseLost = false, poseRestored = false;
  {
    float sx, sy, syaw; bool sval;
    if (navSavedPose(sx, sy, syaw, sval)) {
      if (imuOkFlag) imuSetYawRad(syaw);
      poseRestored = true; poseLost = (!sval && SAFETY_DEFAULT_ON) || !imuOkFlag;   // tanda 'tidak valid' dari sesi lama diabaikan bila pengaman mati
      Serial.printf("[NAV] posisi dipulihkan: x=%.1f y=%.1f yaw=%.1f%s\n", sx, sy, syaw * 57.2958f, poseLost ? " (TIDAK VALID: tetapkan titik 0)" : "");
    }
  }

  GamepadState gp; uint16_t prevBtn = 0;
  bool wasRotating = false;
  bool wasConnected = false, bSuppress = false, estop = false, lock = false, tripped = false;
  uint8_t tripReason = 0, denyReason = 0, denySeq = 0;
  uint32_t bDownMs = 0, battMs = 0, graceUntil = millis() + 2000, saveT0 = 0;
  int saveId = 0;
  float targetYaw = 0, headWz = 0, battV = -1, satScale = 1.0f;
  HeadPid head; uint8_t mode = 0;
  // IMU-assist: medan, selip, stuck, auto-tune PID yaw
  ImuFiltered imuf = {0, 0, 0, 0};
  float yawEncDeg = 0, yawErrDeg = 0, trustNow = ENCODER_TRUST_INIT;      // yaw dari encoder (integrasi wz roda), galat yaw aktif
  Terrain terrainNow = TERRAIN_FLAT; bool slipNow = false, stuckFlag = false, wasNav = false;
  uint32_t stillSinceMs = 0;                                               // A.5.4: robot continuously still since (0 = moving)
  Telemetry t;
  uint32_t lastUs = micros();

  uint8_t pst[4] = {0, 0, 0, 0}; int pcur = -1;   // preset A..D: 0 IDLE, 2 RUNNING
  auto presetsIdle = [&]() { for (int k = 0; k < 4; k++) pst[k] = 0; pcur = -1; };
  auto deny = [&](uint8_t why) { denyReason = why; denySeq++; };
  auto trip = [&](uint8_t why) {
    tripped = true; tripReason = why;
    navCancel(); mcCancel(1); navSetJog(0); lock = false; head.reset(); headWz = 0; motorsStopAll(); presetsIdle();
    Serial.printf("[SAFE] ROBOT DIBERHENTIKAN, sebab %u. Tunggu Lanjut dari dashboard / tombol Start\n", why);
  };

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROL_PERIOD_MS));   // bangun tiap 20 ms ATAU saat override
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    bool full = dt >= 0.015f;                                      // tick PID penuh vs jalur cepat override
    if (full) lastUs = nowUs; else dt = 0.02f;
    if (dt > 0.1f) dt = 0.1f;

    imuUpdate();
    float yaw = imuYawRad();
    if (full) {
      bool wasOnSlope = terrainNow == TERRAIN_SLOPE;          // L2: berbasis terrain (dengan histeresis), bukan ambang mentah
      imuf = readImuFiltered(); terrainNow = detectTerrain(imuf.pitch, imuf.roll);
      // A.5.4 R5: end of ramp -> snap to a marked ramp point if within RAMP_SNAP_TOL_CM (no marker: skip; poseDegraded then clears after flat + still)
      if (wasOnSlope && terrainNow != TERRAIN_SLOPE && imuOk() && navRampSnap(RAMP_SNAP_TOL_CM)) { poseDegraded = false; stillSinceMs = 0; }
    }
    gamepadGet(gp);
    uint32_t nowMs = millis();
    Settings cfg = settingsGet();
    if (imuTakeResetFlag()) { poseLost = true; poseRestored = false; navCancel(); Serial.println("[IMU] chip ter-reset: posisi tidak valid, tetapkan titik 0"); }

    // ---- permintaan lintas-thread ----
    NavReq rq; navConsumeRequests(rq);
    { uint8_t xd; if (navTakeDeny(xd)) deny(xd); }                    // penolakan dari thread Web (mis. simpan waypoint saat posisi tidak valid)
    int mreq = mcConsumeRequest();                                   // tes gerak dari dashboard
    bool failsafe = false;
    uint32_t hbW = g_hbWifiMs, hbS = g_hbSerialMs, hb = hbW > hbS ? hbW : hbS;
    bool hbOk = nowMs - hb <= WEB_FAILSAFE_MS;

    // ---- tombol (edge) ----
    uint16_t pressed = gp.buttons & ~prevBtn, released = prevBtn & ~gp.buttons;
    prevBtn = gp.buttons;
    if ((pressed & BTN_START) && tripped) rq.resume = true;                          // Start = Lanjut saat terkunci (satu-satunya fungsi Start)
    if (pressed & BTN_BACK) rq.zero = true;

    if (gp.buttons & BTN_B) { if (!bDownMs) bDownMs = nowMs; if (nowMs - bDownMs >= ESTOP_HOLD_MS) estop = true; }
    uint32_t bHeld = bDownMs ? nowMs - bDownMs : 0;
    if (!(gp.buttons & BTN_B)) { bDownMs = 0; estop = false; }

    // ---- stick ----
    float lx = gp.lx, ly = gp.ly, rx = gp.rx, ry = gp.ry;
    deadzone(lx, ly); deadzone(rx, ry);
    if (lx == 0 && ly == 0) {                                      // D-Pad sebagai pengganti stick kiri
      lx = ((gp.buttons & BTN_RIGHT) ? 1 : 0) - ((gp.buttons & BTN_LEFT) ? 1 : 0);
      ly = ((gp.buttons & BTN_UP) ? 1 : 0) - ((gp.buttons & BTN_DOWN) ? 1 : 0);
    }
    bool stickActive = lx != 0 || ly != 0 || rx != 0 || ry != 0;

    // ---- STOP, titik 0, Lanjut ----
    if (rq.cancel) { if (navActive() && (gp.buttons & BTN_B)) bSuppress = true; navCancel(); mcCancel(1); navSetJog(0); }
    if (estop) { navCancel(); mcCancel(1); navSetJog(0); }
    if (rq.cancel || estop) presetsIdle();                         // STOP saat RUNNING: batal dan kembali IDLE
    if (rq.zero) {                                                 // titik 0 = posisi DAN arah hadap sekarang
      navCancel(); mcCancel(1); navSetJog(0);
      navResetOdom(); imuZeroYaw(); targetYaw = 0; lock = false; head.reset(); headWz = 0;
      poseLost = false; poseRestored = false; safetyReset(); graceUntil = nowMs + SAFE_GRACE_MS; yaw = imuYawRad();
      stuckFlag = false; yawEncDeg = yaw * 57.2958f; poseDegraded = false; stillSinceMs = 0;
      {                                                              // L2: robot diam dan datar -> baseline pitch/roll diambil ulang; di luar itu tidak diubah
        float zm[3] = {motorsSpeedCmS(0), motorsSpeedCmS(1), motorsSpeedCmS(2)}, zx, zy, zw; kinForward(zm, zx, zy, zw);
        if (imuOk() && hypotf(zx, zy) < 2.0f && fabsf(imuRateDps()) < 5.0f && fabsf(imuf.pitch) < PITCH_SLOPE_THRESHOLD && fabsf(imuf.roll) < ROLL_TILT_THRESHOLD) {
          imuLevel(); assistReset(); terrainReset(); terrainNow = TERRAIN_FLAT; imuf.pitch = imuf.roll = 0;
        }
      }
      Serial.println("[NAV] titik 0 ditetapkan");
    }
    if (tripped && !cfg.safe) { tripped = false; tripReason = 0; Serial.println("[SAFE] pengaman dimatikan: kunci dibuka"); }   // pengaman mati = robot tidak boleh terkunci
    if (rq.resume && tripped) {
      tripped = false; tripReason = 0; safetyReset(); graceUntil = nowMs + SAFE_GRACE_MS; satScale = 1.0f; stuckFlag = false;
      Serial.println("[SAFE] dilanjutkan oleh pengguna");
    }

    // ---- fail-safe: gamepad lepas / dashboard mati (< 300 ms) ----
    if (wasConnected && !gp.connected) { navCancel(); mcCancel(1); navSetJog(0); failsafe = true; presetsIdle(); }
    wasConnected = gp.connected;
    // dua jalur monitoring (Wi-Fi + kabel): perintah dari dashboard hanya berhenti bila KEDUANYA diam
    if (navActive() && navSource() == SRC_WEB && !hbOk) { navCancel(); failsafe = true; presetsIdle(); }
    if (navJog() && !hbOk) { navSetJog(0); failsafe = true; }
    if (mcActive() && !hbOk) { mcCancel(2); failsafe = true; }     // tes gerak: hanya jalan selama dashboard hidup

    // ---- jalan-terus dari dashboard (lokalisasi waypoint) ----
    if (rq.jogSet && rq.jog == 0) navSetJog(0);
    else if (rq.jogSet && rq.jog >= 1 && rq.jog <= 6) {
      if (tripped) deny(3);
      else if (estop || navActive() || mcActive() || stickActive || !hbOk) deny(4);
      else { navSetJog(rq.jog); lock = false; head.reset(); headWz = 0; }
    }
    if (rq.stopSave > 0) {                                           // simpan posisi begitu robot benar-benar diam
      if (poseLost) deny(1); else { saveId = rq.stopSave; saveT0 = nowMs; }
    }

    // ---- mulai navigasi: satu waypoint atau rute ----
    navSetPoseValid(!poseLost); navSetPoseDegraded(poseDegraded);
    if (rq.gotoId > 0 || rq.route >= 0) {
      bool ok = false;
      if (tripped) deny(3);
      else if (estop || mcActive() || navJog()) deny(4);
      else {
        ok = rq.gotoId > 0 ? navStart(rq.gotoId, rq.src) : navStartRoute(rq.route, rq.src);
        if (!ok) deny(poseLost ? 1 : poseDegraded ? 5 : 2);
      }
      if (ok) { targetYaw = yaw; lock = true; head.reset(); headWz = 0; }
    }
    if (rq.grip >= 0) { if (tripped) deny(3); else gripperSet(rq.grip == 1); }   // gripper manual
    if (rq.sync > 0) {                                               // L6: "Sinkron ke WP n" = VERIFY manual, robot diam <= 30 cm dari WP
      if (tripped) deny(3);
      else if (estop || navActive() || mcActive() || navJog() || stickActive) deny(4);
      else if (poseLost) deny(1);
      else if (poseDegraded) deny(5);
      else if (navStartVerify(rq.sync, SRC_WEB)) { targetYaw = yaw; lock = true; head.reset(); headWz = 0; }
      else deny(8);
    }
    // ---- preset A..D (dashboard dan gamepad A/B/X/Y): satu tekan = langsung jalan ----
    // Preset i menjalankan rute i: tiap waypoint dituju, lalu robot berputar ke yaw waypoint (bila diset) sebelum lanjut.
    auto presetPress = [&](int i, NavSrc psrc) {
      if (i < 0 || i > 3) return;
      if (tripped) { deny(3); return; }
      if (estop || mcActive() || navJog() || navActive()) { deny(4); return; }
      if (poseLost) { deny(1); return; }
      if (poseDegraded) { deny(5); return; }                       // A.5.4: pose uncertain, no new navigation
      if (!navRouteReady(i)) { deny(2); return; }
      presetsIdle();
      if (navStartRoute(i, psrc)) { pst[i] = 2; pcur = i; targetYaw = yaw; lock = true; head.reset(); headWz = 0; }
      else deny(poseLost ? 1 : poseDegraded ? 5 : 2);
    };
    if (!estop && !tripped) {
      if (rq.preset >= 0) presetPress(rq.preset, rq.src == SRC_PAD ? SRC_PAD : SRC_WEB);
      int pi = -1;
      if (pressed & BTN_A) pi = 0;
      if (pressed & BTN_X) pi = 2;
      if (pressed & BTN_Y) pi = 3;
      if ((released & BTN_B) && !bSuppress && bHeld < ESTOP_HOLD_MS) pi = 1;   // B: ketuk = preset B, tahan 0,5 dtk = STOP darurat
      if (pi >= 0) presetPress(pi, SRC_PAD);
    }
    if (!(gp.buttons & BTN_B)) bSuppress = false;

    if (mcActive() && stickActive) mcCancel(1);
    if (navActive() && stickActive) { navCancel(); bSuppress = (gp.buttons & BTN_B) ? true : bSuppress; }   // override manual
    if (navJog() && stickActive) navSetJog(0);

    // ---- tepi navigasi: mulai -> sinkron yaw encoder ke IMU; berakhir (STATE DONE) -> reset timer, filter, trust, buffer PID ----
    { bool na = navActive();
      if (na && !wasNav) { assistReset(); stuckFlag = false; slipNow = false; trustNow = ENCODER_TRUST_INIT; yawEncDeg = yaw * 57.2958f; imuf = readImuFiltered(); }
      if (!na && wasNav) { assistReset(); slipNow = false; trustNow = ENCODER_TRUST_INIT; yawErrDeg = 0; }
      wasNav = na; }

    // ---- tentukan perintah (vx, vy, wz) ----
    float vx = 0, vy = 0, wz = 0;
    float asc = 1.0f, dirX = 0, dirY = 0, slopeF = 1.0f, dobs[3] = {-1, -1, -1};   // L3/L8: skala hindar, arah sebelum pengali, faktor tanjakan, jarak keputusan hindar
    float dist[3]; ultrasonicGet(dist);
    float wm[3]; bool wo[3]; ultrasonicGetMedian(wm, wo); navSetWall(wm, wo, terrainNow == TERRAIN_TILT);
    irUpdate(nowMs); navSetIr(irFront(), irLeft(), irRight());   // IR: penengah ultrasonik depan + deteksi halangan sisi depan
    bool turbo = (gp.buttons & BTN_RB) != 0;                        // R1 ditahan = turbo (hanya manual)
    float rpmLim = turbo ? cfg.turbo : cfg.man;
    float mvNow[3] = {motorsSpeedCmS(0), motorsSpeedCmS(1), motorsSpeedCmS(2)};
    float rateRad = imuRateDps() * (float)M_PI / 180.0f;
    float iaR = 0, iaF = 0; imuLinAccelBody(iaR, iaF);

    if (mreq && !mcActive()) {                                     // mulai tes gerak bila aman
      bool obstNear = false; for (int i = 0; i < 3; i++) if (dist[i] > 0 && dist[i] < OBST_DIST_CM) obstNear = true;
      if (estop || failsafe || stickActive || tripped) mcAbortStart(mreq, 1);
      else if (!hbOk)                         mcAbortStart(mreq, 2);
      else if (navActive() || navJog())       mcAbortStart(mreq, 5);
      else if (obstNear)                      mcAbortStart(mreq, 3);
      else                                    mcStart(mreq, yaw);
    }

    if (tripped || estop || failsafe) {
      mode = 2; lock = false; head.reset(); headWz = 0;
    } else if (mcActive()) {                                       // tes gerak: tanpa heading lock supaya putaran liar terlihat
      mode = 3; lock = false; head.reset(); headWz = 0;
      mcStep(nowMs, dt, full, mvNow, yaw, imuOk(), iaR, iaF, vx, vy);
      wz = 0;
    } else if (navJog()) {                                         // jalan dari dashboard sampai STOP
      mode = 4; rpmLim = cfg.loc;
      int d = navJog(); float v = rpmToCms(cfg.loc);
      if (d <= 4) {
        vy = d == 1 ? v : d == 2 ? -v : 0; vx = d == 4 ? v : d == 3 ? -v : 0;
        if (imuOk()) {
          if (!lock) { targetYaw = yaw; lock = true; head.reset(); }
          if (full) headWz = head.step(wrapPi(targetYaw - yaw), rateRad, dt);
          wz = headWz;
        }
      } else {
        wz = (d == 5 ? 1.0f : -1.0f) * (v / ROBOT_L_CM) * ROT_FRAC;   // 5 = putar kiri (CCW)
        lock = false; head.reset(); headWz = 0;
      }
    } else if (navActive()) {
      mode = 1; rpmLim = cfg.nav;
      float rotTgt = 0; bool rotating = false;
      bool navRun = navStep(yaw, imuRateDps(), rpmToCms(cfg.nav), imuOk(), vx, vy, rotTgt, rotating);
      { bool g; if (navTakeGrip(g)) gripperSet(g); }               // langkah gripper di rute (robot sedang diam)
      if (navRun) {
        if (rotating) {                                            // STATE ROTATING: motor translasi 0, spin di tempat dengan PID yaw
          if (!wasRotating) { head.reset(); headWz = 0; }
          float e = wrapPi(rotTgt - yaw);
          yawErrDeg = e * 57.2958f;
          if (fabsf(e) * 57.2958f <= NAV_YAW_TOL_DEG) { wz = 0; head.reset(); headWz = 0; }     // di dalam toleransi: PWM 0
          else {
            PidGains pg = assistPidCurrent();                      // gain putaran dari auto-tune (F9); awal = HEAD_*
            if (full) headWz = head.step(e, rateRad, dt, pg.kp / PID_KP_INIT, pg.ki, pg.kd / PID_KD_INIT);
            float wmax = fmaxf(NAV_ROT_MIN_WZ, rpmToCms(cfg.nav) / ROBOT_L_CM * ROT_FRAC);
            float mag = constrain(fabsf(headWz), NAV_ROT_MIN_WZ, wmax);
            wz = e >= 0 ? mag : -mag;
          }
        } else {                                                   // STATE MOVING: menuju x, y dengan heading lock (IMU = wasit arah)
          if (wasRotating) { targetYaw = yaw; head.reset(); headWz = 0; }   // selesai putar di waypoint perantara: kunci arah baru
          if (navPhase() == 1) {                                   // L4: keputusan menghindar memakai median valid di semua terrain; depan selalu aktif (L9 AVOID ada di navStep)
            navObstDist(wm, wo, dobs);                         // H2: avoidObstacles dihapus dari jalur nav; hindar bertahap + batas kecepatan ada di navStep, sisi dijaga L10
            asc = navAvoidScale();
          }
          dirX = vx; dirY = vy;                                    // L3: arah gerak sebelum pengali trust / sat / tanjakan
          if (full && imuOk()) {
            float tgtDeg = targetYaw * 57.2958f;
            yawErrDeg = normalizeAngle(tgtDeg - imuf.yaw);
            slipNow = detectWheelSlip(yawEncDeg, imuf.yaw, imuf.gyroZ, nowMs);
            if (!slipNow && fabsf(imuf.gyroZ) < SLIP_GYRO_MIN) yawEncDeg = imuf.yaw;   // tidak berputar: buang drift integrasi encoder
            trustNow = updateEncoderTrust(slipNow);
            PidGains pg = assistTuneTick(yawErrDeg);
            // gain hasil tuning dikalikan rasio medan agar tanjakan tetap lebih agresif; sebelum tuning pertama: None -> default F4
            float kpDyn = pg.adjusted ? pg.kp * (terrainNow != TERRAIN_FLAT ? KP_YAW_SLOPE / KP_YAW_FLAT : 1.0f) : NAN;
            float corr = computeYawCorrection(tgtDeg, imuf.yaw, terrainNow, kpDyn);      // derajat x kp
            headWz = head.stepP(corr * (float)M_PI / 180.0f * HEAD_KP, yawErrDeg * (float)M_PI / 180.0f, rateRad, dt, pg.ki, pg.kd / PID_KD_INIT);
          }
          wz = imuOk() ? headWz : 0;
          vx *= trustNow; vy *= trustNow;                          // base_speed_eff = base_speed * encoder_trust (tanpa dead-reckoning IMU)
          if (full && imuOk()) {                                   // stuck: yaw melenceng, tidak berputar, encoder diam padahal diperintah maju
            bool cmd = hypotf(vx, vy) > STUCK_ENC_MIN;
            float ex, ey, ew; kinForward(mvNow, ex, ey, ew);
            if (!cmd || navAvoiding()) assistResetStuck();         // L9: stuck dinonaktifkan selama AVOID + AVOID_HOLD_MS
            else if (detectStuck(yawErrDeg, imuf.gyroZ, hypotf(ex, ey), nowMs)) {
              if (navRetryLeg()) { assistResetStuck(); Serial.println("[ASSIST] STUCK: coba ulang pendekatan"); }   // R5
              else { stuckFlag = true; navCancel(); vx = vy = 0; wz = 0; head.reset(); headWz = 0; lock = false;
                Serial.println("[ASSIST] STUCK: robot berhenti"); }
            }
          }
        }
        wasRotating = rotating;
      } else { mode = 0; lock = false; head.reset(); headWz = 0; wasRotating = false; }
    } else {
      mode = 0;
      float vmax = rpmToCms(rpmLim);
      vx = lx * vmax; vy = ly * vmax; wz = -rx * (vmax / ROBOT_L_CM) * ROT_FRAC;
      bool moving = hypotf(vx, vy) > 2.0f;
      if (rx == 0 && moving && imuOk()) {                          // heading lock (PID sudut)
        if (!lock && fabsf(imuRateDps()) < 15.0f) { targetYaw = yaw; lock = true; head.reset(); }
        if (lock) { if (full) headWz = head.step(wrapPi(targetYaw - yaw), rateRad, dt); wz = headWz; }
      } else { lock = false; head.reset(); headWz = 0; }
    }

    // ---- preset RUNNING selesai (tuntas atau batal) -> IDLE ----
    if (pcur >= 0 && pst[pcur] == 2 && !navActive()) { pst[pcur] = 0; pcur = -1; }

    // ---- pembatas daya: bila ada roda jenuh (tanjakan, motor lemah) kecepatan translasi diturunkan
    //      supaya roda yang lemah bisa mengikuti dan robot tetap lurus ----
    if (full) {
      bool busy = false; for (int i = 0; i < 3; i++) if (fabsf(motorsSetpointRpm(i)) > 5.0f) busy = true;
      if (busy && motorsMaxDuty() > 0.95f) satScale = fmaxf(0.35f, satScale - 0.02f);
      else satScale = fminf(1.0f, satScale + 0.01f);
    }
    float tmul = satScale;                                         // L3: pengali gabungan translasi = min(satScale, faktor tanjakan), bukan hasil kali
    if (imuOk() && terrainNow != TERRAIN_FLAT) {                   // A.5.4 R4 + L1: slower on slope/tilt; stricter on descent (forward = +vy, pitch > 0 = nose up)
      slopeF = slopeSpeedFactor(terrainNow, imuf.pitch, vy); tmul = translationScale(satScale, slopeF);
    }
    vx *= tmul; vy *= tmul;
    if (mode == 1 && navPhase() == 1) navSpeedFloor(vx, vy, dirX, dirY, rpmToCms(cfg.nav), navObstConfirmed(dobs, dirX, dirY));   // L3: lantai setelah SEMUA pengali
    if (poseRestored && (hypotf(vx, vy) > 1.0f || fabsf(wz) > 0.05f)) poseRestored = false;

    // ---- kinematika -> motor ----
    float wv[3];
    kinInverse(vx, vy, wz, wv);
    motorsSetWheelCmS(wv);
    if (tripped || estop || failsafe) motorsStopAll();
    if (full) {
      motorsUpdate(dt);
      float mv[3] = {motorsSpeedCmS(0), motorsSpeedCmS(1), motorsSpeedCmS(2)};
      // slope-aware odometry (A.5.4 R1/R2); also integrates encoder yaw (for slip detection), never frozen
      if (navOdom(mv, yaw, dt, imuOk() ? imuf.pitch : 0.0f, imuOk() ? imuf.roll : 0.0f, trustNow, (imuOk() && terrainNow != TERRAIN_FLAT) || slipNow, &yawEncDeg)) poseDegraded = true;
      // R3: poseDegraded stays on until terrain is FLAT and the robot has been still for 500 ms (not merely flat again)
      if (fabsf(vx) < 0.02f && fabsf(vy) < 0.02f && fabsf(wz) < 0.05f) { if (!stillSinceMs) stillSinceMs = nowMs; } else stillSinceMs = 0;
      if (poseDegraded && terrainNow == TERRAIN_FLAT && stillSinceMs && nowMs - stillSinceMs >= 500) poseDegraded = false;

      // ---- pengaman (dilewati sepenuhnya bila dimatikan: tidak ada trip, tidak ada deteksi diangkat) ----
      if (!cfg.safe) { safetyReset(); } else {
      SafetyIn si; si.dt = dt; si.imuOk = imuOk(); si.enabled = cfg.safe && motorsTuneState() != 1;
      si.busy = false;
      for (int i = 0; i < 3; i++) { si.rpm[i] = motorsRpm(i); si.sp[i] = motorsSetpointRpm(i); si.duty[i] = motorsDuty(i); if (fabsf(si.sp[i]) > 5.0f) si.busy = true; }
      float bw; kinForward(mv, si.mvx, si.mvy, bw); si.mw = bw;
      si.rateDps = imuRateDps(); si.accR = iaR; si.accF = iaF; si.accZ = imuLinAccelZ(); si.wzCmd = wz;
      uint8_t r = safetyStep(si);
      if (nowMs > graceUntil && !tripped) {
        if (safetyLiftEvent()) {                                   // diangkat / dipindah: posisi tidak valid
          if (!poseLost) { poseLost = true; poseRestored = false; navCancel(); Serial.println("[SAFE] robot terdeteksi diangkat/dipindah: posisi tidak valid, tetapkan titik 0"); }
          if (cfg.safe && si.busy && !mcActive()) r = 4;
        }
        if (r) trip(r);
      } else safetyLiftEvent();                                    // buang event selama masa tenggang
      }
    } else {
      motorsApplyFF();                                             // override: respons langsung (jalur cepat, tanpa menunggu tick 20 ms)
    }

    // ---- simpan posisi tepat saat robot sudah diam (tombol "STOP dan Simpan") ----
    if (saveId > 0) {
      float bx, by, bw; kinForward(mvNow, bx, by, bw);
      if (!navJog() && !navActive() && (hypotf(bx, by) < 3.0f || nowMs - saveT0 > 1200)) {
        if (poseLost) deny(1);                                     // posisi jadi tidak valid selagi menunggu diam: jangan simpan
        else { float x, y, wr3[3]; navPose(x, y); navWallRefCapture(hypotf(bx, by) < 2.0f, terrainNow == TERRAIN_FLAT, wr3);   // L5: ref hanya bila valid, stabil, datar, diam
               wpSet(saveId, x, y, yaw * 57.2958f, true, wr3[0], wr3[1], wr3[2]); }   // posisi dan arah hadap
        saveId = 0;
      }
    }

    // ---- telemetri (10 Hz cukup, tapi murah) ----
    if (full) {
      if (PIN_BATT_ADC >= 0 && nowMs - battMs > 500) { battMs = nowMs; battV = analogReadMilliVolts(PIN_BATT_ADC) / 1000.0f * BATT_DIV; }
      t.ready = true;
      navPose(t.x, t.y); t.yawDeg = yaw * 180.0f / (float)M_PI;
      t.vx = vx; t.vy = vy; t.wz = wz;
      for (int i = 0; i < 3; i++) { t.rpm[i] = motorsRpm(i); t.sp[i] = motorsSetpointRpm(i); t.dist[i] = dist[i]; }
      ultrasonicGetMedian(t.usMed, t.usOk); t.ir = irBits();
      navWallRef(t.wallRef); t.wallStat = navWallStat(); t.navRetry = navRetryNo(); navStats(t.sess);
      t.mode = mode; t.navTarget = navTarget(); t.navRoute = navRoute(); t.navStep = navStepNo(); t.navSteps = navStepCount();
      t.jog = navJog(); t.rpmLimit = rpmLim; t.turbo = turbo && mode == 0; t.armed = false;
      for (int k = 0; k < 4; k++) t.pst[k] = pst[k];
      t.navPhase = navPhase(); t.navDone = navDoneSeq(); t.gripper = gripperOn();
      t.padConnected = gp.connected; strncpy(t.padType, gp.type, 7); t.padType[7] = 0;
      { float bw; kinForward(mvNow, t.mvx, t.mvy, bw); }
      t.imuAddr = imuAddress(); t.imuCal = imuCalib(); t.imuNdof = imuIsNdof(); t.mc = mcResult();
      t.imuOk = imuOk(); t.batt = battV; for (int i = 0; i < 3; i++) motorsGetPID(i, t.kp[i], t.ki[i], t.kd[i]); t.tune = motorsTuneState(); t.tuneWhy = motorsTuneWhy();
      t.safe = tripped ? tripReason : 0; t.safeOn = cfg.safe; t.poseLost = poseLost; t.poseRestored = poseRestored; t.poseDegraded = poseDegraded;
      t.sat = (uint8_t)(satScale * 100.0f + 0.5f);
      t.slopeF = slopeF; t.avoidScale = asc; t.effRpm = cmsToRpm(hypotf(vx, vy)); t.sideMin = navSideMin();   // L8
      t.avoid = navAvoidState(); t.refStale = navRefStale(); t.arr = navArrLog();
      { float sd[3]; safetyDebug(sd); t.safeDbg[0] = sd[0]; t.safeDbg[1] = sd[1]; t.safeDbg[2] = sd[2]; t.safeDbg[3] = motorsMaxDuty() * 100.0f; }
      t.terrain = terrainNow; t.yawErr = yawErrDeg; t.pitch = imuf.pitch; t.roll = imuf.roll;
      t.stuck = stuckFlag; t.slip = slipNow; t.encTrust = trustNow;
      { PidGains pg = assistPidCurrent(); t.yKp = pg.kp; t.yKi = pg.ki; t.yKd = pg.kd; t.yStatus = pg.status; }
      t.cfg[0] = cfg.man; t.cfg[1] = cfg.turbo; t.cfg[2] = cfg.nav; t.cfg[3] = cfg.loc;
      storeStatus(t.svStatus, t.svWhat, t.svCount);
      t.deny = denyReason; t.denySeq = denySeq;
      telemetrySet(t);
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n=== Omni-3X Single Controller ===");
  storeBegin();                          // harus pertama: membuat mutex penyimpanan
  settingsBegin();
  navBegin();
  gripperBegin();                        // solenoid awal OFF
  xTaskCreatePinnedToCore(controlTask, "control", 10240, nullptr, 5, &ctrlTask, 1);
  gamepadBegin(onGamepad);               // Core 0
  irBegin();
  ultrasonicBegin();                     // Core 0
  webBegin();                            // Core 0
  navStartPoseSaver();                   // Core 0: simpan posisi terakhir
}

void loop() { vTaskDelete(nullptr); }
