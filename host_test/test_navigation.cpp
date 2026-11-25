// A.5.3 (avoidObstacles), A.5.6 (navOdom, state MOVING -> ROTATING -> DONE, waypoint, rute, migrasi skala).
// navigation.cpp disertakan LANGSUNG (tanpa fork); statiknya dipakai untuk mereset keadaan antar tes.
#include "Arduino.h"
#include "navigation.cpp"
#include "ultrasonic.h"
#include "host_stubs.h"
#include "test_main.h"

namespace {

const float DT = CONTROL_PERIOD_MS / 1000.0f;
float wrapRad(float e) { while (e > (float)M_PI) e -= 2 * (float)M_PI; while (e < -(float)M_PI) e += 2 * (float)M_PI; return e; }

// Reset keadaan statis navigation.cpp (tidak ada API reset di firmware; tidak mengubah firmware).
void navFresh() {
  navSetIr(false, false, false);
  host_store_clear(); host_set_millis(1000);
  navCancel(); hasSaved = false; px = py = 0; poseOk = true; jogDir = 0; req = NavReq(); extDeny = 0;
  retryN = 0; memset(sess, 0, sizeof sess); memset(wallOk, 0, sizeof wallOk); wallStat = 0;
  for (int i = 0; i < 3; i++) wallMed[i] = -1;                 // Lapangan v2: reset keadaan baru
  avoidReset(); avoidDir = 1; sideMinSeen = -1; lastAvoidScale = 1; lastDist = 0; vShift = false; vIter = 0; vStartMs = 0; vShiftEnd = 0;
  memset(refCnt, 0, sizeof refCnt); refMs = 0; wallTilt = false; arrLog = ArrLog(); gripPend = false;
  navBegin();                                  // wps kosong, rute default A=WP1..D=WP4
  navResetOdom();
}

// Robot ideal: loop kontrol seperti main.cpp (navStep -> avoidObstacles -> kinInverse -> navOdom), roda mengikuti perintah persis.
struct Bot {
  float yaw = 0;            // rad, CCW +
  float rateDps = 0;
  bool  imu = true;
  float d[3] = {-1, -1, -1};
  float v = rpmToCms(SPD_NAV_DEF);
  float maxSpin = 2.0f;     // rad/s
  int   phaseSeen[3] = {0, 0, 0};
  int   ticksRotating = 0;
};

// satu tick 20 ms; true bila navigasi masih berjalan
bool tickBot(Bot& b) {
  host_advance_ms(CONTROL_PERIOD_MS);
  float vx, vy, tgt; bool rot = false;
  bool ok = navStep(b.yaw, b.rateDps, b.v, b.imu, vx, vy, tgt, rot);
  float wz = 0;
  if (ok && rot) { float e = wrapRad(tgt - b.yaw); wz = fmaxf(-b.maxSpin, fminf(b.maxSpin, 5.0f * e)); }
  else if (ok)   avoidObstacles(vx, vy, b.d);
  if (!ok) { vx = vy = 0; }
  float wv[3]; kinInverse(vx, vy, wz, wv);
  navOdom(wv, b.yaw, DT);
  float ex, ey, ew; kinForward(wv, ex, ey, ew);
  b.yaw = wrapRad(b.yaw + ew * DT); b.rateDps = ew * 57.29578f;
  int p = navPhase(); if (p >= 0 && p <= 2) b.phaseSeen[p]++;
  if (p == 2) b.ticksRotating++;
  return ok || navActive();
}

int runBot(Bot& b, int maxTicks) { int n = 0; while (n < maxTicks && navActive()) { tickBot(b); n++; } return n; }

void pose(float& x, float& y) { navPose(x, y); }

}  // namespace

// ---------------- navOdom (A.5.6) ----------------
TEST(test_A_5_6_navodom_forward_strafe_and_yaw_projection) {
  navFresh();
  float wv[3], x, y;
  kinInverse(0, 30, 0, wv);                                    // maju 30 cm/s, yaw 0 -> dunia +y
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT);          // 1 detik
  pose(x, y); CHECK_NEAR(x, 0, 0.05); CHECK_NEAR(y, 30, 0.05);
  navResetOdom(); pose(x, y); CHECK(x == 0 && y == 0);
  for (int i = 0; i < 50; i++) navOdom(wv, (float)M_PI / 2, DT);   // yaw +90 (CCW): depan robot = dunia -x
  pose(x, y); CHECK_NEAR(x, -30, 0.05); CHECK_NEAR(y, 0, 0.05);
  navResetOdom();
  kinInverse(30, 0, 0, wv);                                    // geser kanan, yaw 0 -> dunia +x
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT);
  pose(x, y); CHECK_NEAR(x, 30, 0.05); CHECK_NEAR(y, 0, 0.05);
}

TEST(test_A_5_6_navodom_zero_wheels_no_drift) {
  navFresh();
  float wv[3] = {0, 0, 0}, x, y;
  for (int i = 0; i < 500; i++) navOdom(wv, 1.0f, DT);
  pose(x, y); CHECK(x == 0 && y == 0);
}

// ---------------- A.5.3: avoidObstacles dengan SENSOR_ANG_DEG = {90, 180, 0} ----------------
TEST(test_A_5_3_avoid_front_removes_forward_component) {
  float vx = 0, vy = 30, d[3] = {15, -1, -1};                   // benda di DEPAN (indeks 0), robot maju
  avoidObstacles(vx, vy, d);
  CHECK_MSG(fabsf(vy) < 0.2f * 30, "komponen maju harus dibuang");
  CHECK_MSG(fabsf(vx) > 1.0f, "harus ada geser samping untuk menghindar");
  CHECK_MSG(hypotf(vx, vy) < 30.0f, "kecepatan turun");
}

TEST(test_A_5_3_avoid_left_removes_leftward_component) {
  float vx = -30, vy = 0, d[3] = {-1, 15, -1};                  // benda di KIRI (indeks 1), robot geser kiri
  avoidObstacles(vx, vy, d);
  CHECK_MSG(vx > -0.2f * 30, "geser ke kiri harus dibuang");
  CHECK(hypotf(vx, vy) < 30.0f);
}

TEST(test_A_5_3_avoid_right_removes_rightward_component) {
  float vx = 30, vy = 0, d[3] = {-1, -1, 15};                   // benda di KANAN (indeks 2), robot geser kanan
  avoidObstacles(vx, vy, d);
  CHECK_MSG(vx < 0.2f * 30, "geser ke kanan harus dibuang");
  CHECK(hypotf(vx, vy) < 30.0f);
}

TEST(test_A_5_3_avoid_ignores_obstacles_not_in_travel_direction) {
  { float vx = 0, vy = 30, d[3] = {-1, 15, 15};  avoidObstacles(vx, vy, d); CHECK(vx == 0 && vy == 30); }       // maju, benda kiri+kanan
  { float vx = 30, vy = 0, d[3] = {15, 15, -1};  avoidObstacles(vx, vy, d); CHECK(vx == 30 && vy == 0); }       // ke kanan, benda depan+kiri
  { float vx = -30, vy = 0, d[3] = {-1, -1, 15}; avoidObstacles(vx, vy, d); CHECK(vx == -30 && vy == 0); }     // ke kiri, benda kanan
  { float vx = 0, vy = -30, d[3] = {15, -1, -1}; avoidObstacles(vx, vy, d); CHECK(vx == 0 && vy == -30); }      // mundur, benda depan
  { float vx = 0, vy = 30, d[3] = {200, -1, -1}; avoidObstacles(vx, vy, d); CHECK(vx == 0 && vy == 30); }       // jauh
  { float vx = 0, vy = 30, d[3] = {-1, -1, -1};  avoidObstacles(vx, vy, d); CHECK(vx == 0 && vy == 30); }       // tidak ada rintangan
}

TEST(test_A_5_3_KNOWN_QUIRK_float_noise_slows_left_strafe_and_backward_with_side_obstacle) {
  // TEMUAN (belum diperbaiki: di luar fase 1, D.5 aturan 3). cosf(90 deg) dan sinf(180 deg) di float tidak persis 0
  // (-4.4e-8 / -8.7e-8), jadi komponen "menuju rintangan" bernilai kecil POSITIF untuk geser kiri dengan benda di depan,
  // dan untuk mundur dengan benda di kiri. Akibatnya `scale` memotong kecepatan sampai 0,3x padahal robot tidak mendekati
  // benda itu. Geser kanan / maju tidak terkena (noise negatif), jadi perilakunya tidak simetris.
  // Perbaikan yang disarankan: `if (comp <= 1e-3f * vmag) continue;`. Bila diperbaiki, balik dua CHECK di bawah ke `== -30` / `== -30`.
  { float vx = -30, vy = 0, d[3] = {15, -1, -1}; avoidObstacles(vx, vy, d); CHECK_NEAR(vx, -30 * 0.306, 0.1); }   // geser kiri, benda DEPAN
  { float vx = 0, vy = -30, d[3] = {-1, 15, -1}; avoidObstacles(vx, vy, d); CHECK_NEAR(vy, -30 * 0.306, 0.1); }   // mundur, benda KIRI
  { float vx = 30, vy = 0, d[3] = {15, -1, -1}; avoidObstacles(vx, vy, d); CHECK(vx == 30 && vy == 0); }         // geser kanan, benda depan: tidak terkena
}

TEST(test_A_5_3_avoid_output_through_KIN_Y_SIGN_stays_consistent) {
  // Keluaran avoidObstacles ada di frame badan (+x kanan, +y depan). kinInverse menerapkan KIN_Y_SIGN = -1;
  // kinForward harus mengembalikan twist yang sama, dan rintangan depan harus MENGURANGI kecepatan roda penggerak maju.
  float vx = 0, vy = 30, d[3] = {15, -1, -1};
  float w0[3]; kinInverse(0, 30, 0, w0);
  avoidObstacles(vx, vy, d);
  float w1[3], x, y, w; kinInverse(vx, vy, 0, w1); kinForward(w1, x, y, w);
  CHECK_NEAR(x, vx, 1e-3); CHECK_NEAR(y, vy, 1e-3); CHECK_NEAR(w, 0, 1e-4);
  float m0 = fmaxf(fabsf(w0[1]), fabsf(w0[2])), m1 = fmaxf(fabsf(w1[1]), fabsf(w1[2]));
  CHECK_MSG(m1 < m0, "roda maju (1 dan 2) harus melambat bila ada rintangan depan");
  CHECK_NEAR(w0[1], -KIN_Y_SIGN * 30.0 * cos(30.0 * M_PI / 180.0), 1e-3);    // maju = roda 1 positif bila KIN_Y_SIGN = -1
}

TEST(test_A_5_3_closed_loop_does_not_hit_front_obstacle) {
  navFresh();
  wpSet(1, 0, 200, 0, false);                                    // tujuan lurus ke depan, rintangan 15 cm di depan
  CHECK(navStart(1, SRC_WEB));
  Bot b; b.d[0] = 15;
  for (int i = 0; i < 50; i++) tickBot(b);                       // 1 detik
  float x, y; pose(x, y);
  CHECK_MSG(y < 15.0f, "robot maju terlalu jauh walau ada rintangan depan");
}

// ---------------- state navigasi (A.5.6) ----------------
TEST(test_A_5_6_nav_goto_moving_rotating_done) {
  navFresh();
  wpSet(1, 60, 40, 90, true);
  uint8_t seq0 = navDoneSeq();
  CHECK(navStart(1, SRC_WEB));
  CHECK(navPhase() == 1);                                        // MOVING
  Bot b; int n = runBot(b, 3000);
  float x, y; pose(x, y);
  CHECK_MSG(n < 3000, "navigasi tidak selesai");
  CHECK(!navActive());
  CHECK_MSG(navDoneSeq() == (uint8_t)(seq0 + 1), "DONE harus menaikkan navDoneSeq tepat 1");
  CHECK_MSG(b.phaseSeen[1] > 0 && b.phaseSeen[2] > 0, "harus melewati MOVING lalu ROTATING");
  CHECK_MSG((uint32_t)(b.ticksRotating * CONTROL_PERIOD_MS) >= NAV_YAW_SETTLE_MS, "ROTATING minimal selama settle");
  CHECK(hypotf(x - 60, y - 40) <= NAV_ARRIVE_CM);
  CHECK(fabsf(wrapRad(b.yaw - 90.0f * (float)M_PI / 180.0f)) * 57.29578f <= NAV_YAW_TOL_DEG);
}

TEST(test_A_5_6_nav_world_to_body_frame) {
  navFresh();
  float vx, vy, tgt; bool rot;
  wpSet(1, 0, 100, 0, false); navStart(1, SRC_WEB);              // yaw 0: depan = +y dunia
  CHECK(navStep(0.0f, 0, 50, true, vx, vy, tgt, rot)); CHECK_NEAR(vx, 0, 1e-3); CHECK(vy > 0);
  navCancel();
  wpSet(2, 100, 0, 0, false); navStart(2, SRC_WEB);              // target dunia +x = KANAN badan saat yaw 0
  CHECK(navStep(0.0f, 0, 50, true, vx, vy, tgt, rot)); CHECK(vx > 0); CHECK_NEAR(vy, 0, 1e-3);
  navCancel();
  wpSet(3, 0, 100, 0, false); navStart(3, SRC_WEB);              // yaw +90 CCW: dunia +y ada di KANAN badan
  CHECK(navStep((float)M_PI / 2, 0, 50, true, vx, vy, tgt, rot)); CHECK(vx > 0); CHECK_NEAR(vy, 0, 1e-3);
  navCancel();
}

TEST(test_A_5_6_nav_intermediate_without_yaw_is_passed_without_rotating) {
  navFresh();
  wpSet(1, 0, 60, 0, false); wpSet(2, 0, 120, 0, false);
  uint8_t ids[] = {1, 2}; routeSet(0, ids, 2);
  CHECK(navStartRoute(0, SRC_WEB)); CHECK(navStepCount() == 2);
  Bot b; bool rotatedAtStep1 = false; int n = 0;
  while (navActive() && n++ < 3000) { tickBot(b); if (navActive() && navStepNo() == 1 && navPhase() == 2) rotatedAtStep1 = true; }
  CHECK(!navActive()); CHECK(!rotatedAtStep1);
  float x, y; pose(x, y); CHECK(hypotf(x - 0, y - 120) <= NAV_ARRIVE_CM);
}

TEST(test_A_5_6_nav_intermediate_with_yaw_stops_and_rotates) {
  navFresh();
  wpSet(1, 0, 40, 45, true); wpSet(2, 0, 80, 0, false);
  uint8_t ids[] = {1, 2}; routeSet(0, ids, 2);
  CHECK(navStartRoute(0, SRC_WEB));
  Bot b; bool rotatedAtStep1 = false; int n = 0;
  while (navActive() && n++ < 3000) { tickBot(b); if (navActive() && navStepNo() == 1 && navPhase() == 2) rotatedAtStep1 = true; }
  CHECK(!navActive()); CHECK_MSG(rotatedAtStep1, "waypoint perantara ber-yaw harus berhenti dan memutar");
}

TEST(test_A_5_6_nav_without_imu_skips_rotation) {
  navFresh();
  wpSet(1, 0, 50, 90, true);
  CHECK(navStart(1, SRC_WEB));
  Bot b; b.imu = false; uint8_t seq0 = navDoneSeq(); runBot(b, 3000);
  CHECK(!navActive()); CHECK(b.phaseSeen[2] == 0); CHECK(navDoneSeq() == (uint8_t)(seq0 + 1));
}

TEST(test_A_5_6_nav_leg_timeout_cancels_without_done) {
  navFresh();
  wpSet(1, 0, 100, 0, false); CHECK(navStart(1, SRC_WEB));
  uint8_t seq0 = navDoneSeq();
  float vx, vy, tgt; bool rot;
  for (int k = 0; k < NAV_RETRY; k++) { host_advance_ms(NAV_TIMEOUT_MS + 10); navStep(0, 0, 50, true, vx, vy, tgt, rot); }   // R5: jatah coba ulang dihabiskan
  host_advance_ms(NAV_TIMEOUT_MS + 10);                          // robot tidak bergerak sama sekali
  CHECK(!navStep(0, 0, 50, true, vx, vy, tgt, rot));
  CHECK(!navActive()); CHECK(navDoneSeq() == seq0);
}

TEST(test_A_5_6_nav_rotation_timeout_cancels_without_done) {
  navFresh();
  wpSet(1, 0, 0, 90, true); CHECK(navStart(1, SRC_WEB));         // sudah di titik: langsung ROTATING
  uint8_t seq0 = navDoneSeq();
  float vx, vy, tgt; bool rot = false;
  CHECK(navStep(0, 0, 50, true, vx, vy, tgt, rot)); CHECK(rot); CHECK(navPhase() == 2);
  host_advance_ms(NAV_ROT_TIMEOUT_MS + 10);                      // yaw tidak pernah berubah
  CHECK(!navStep(0, 0, 50, true, vx, vy, tgt, rot));
  CHECK(!navActive()); CHECK(navDoneSeq() == seq0);
}

TEST(test_A_5_6_nav_yaw_wrap_359_equals_minus_1) {
  navFresh();
  wpSet(1, 0, 0, 359, true);
  Waypoint w; CHECK(wpGet(1, w)); CHECK_NEAR(w.yaw, -1.0, 1e-4);
  CHECK(navStart(1, SRC_WEB));
  Bot b; b.yaw = 1.0f * (float)M_PI / 180.0f; int n = runBot(b, 400);     // galat hanya 2 derajat
  CHECK_MSG(n < 100, "galat 2 derajat harus tuntas cepat (bukan putar 358 derajat)");
  CHECK(!navActive());
}

TEST(test_A_5_6_nav_refused_when_pose_invalid) {
  navFresh();
  wpSet(1, 10, 10, 0, false);
  navSetPoseValid(false);
  CHECK(!navStart(1, SRC_WEB)); CHECK(!navStartRoute(0, SRC_WEB)); CHECK(!navActive());
  navSetPoseValid(true);
  CHECK(navStart(1, SRC_WEB)); navCancel();
}

TEST(test_A_5_6_routes_default_set_filter_and_skip_empty) {
  navFresh();
  uint8_t rt[NUM_ROUTES][MAX_RT]; routeCopyAll(rt);
  for (int r = 0; r < NUM_ROUTES; r++) { CHECK(rt[r][0] == r + 1); CHECK(rt[r][1] == 0); }   // A=WP1 ... Y=WP4
  CHECK(!navRouteReady(0));                                      // WP1 belum valid
  wpSet(1, 10, 0, 0, false); wpSet(3, 30, 0, 0, false);          // WP2 kosong
  uint8_t ids[] = {1, 2, 3, 0, 17, 200};                         // 0 dan >16 harus dibuang
  CHECK(routeSet(0, ids, 6)); routeCopyAll(rt);
  CHECK(rt[0][0] == 1 && rt[0][1] == 2 && rt[0][2] == 3 && rt[0][3] == 0);
  CHECK(navRouteReady(0)); CHECK(navStartRoute(0, SRC_PAD)); CHECK_MSG(navStepCount() == 2, "waypoint kosong WP2 dilewati");
  navCancel();
  CHECK(routeSet(0, nullptr, 0)); CHECK(!navRouteReady(0)); CHECK(!navStartRoute(0, SRC_PAD));
  CHECK(!routeSet(-1, ids, 1) && !routeSet(NUM_ROUTES, ids, 1));
}

TEST(test_A_5_6_waypoint_set_keep_yaw_delete_and_range) {
  navFresh();
  Waypoint w;
  CHECK(wpSet(1, 1, 2, 190, true)); wpGet(1, w); CHECK(w.valid && w.hasYaw); CHECK_NEAR(w.yaw, -170, 1e-4);
  CHECK(wpSet(2, 3, 4, 90, false)); wpGet(2, w); CHECK(w.valid && !w.hasYaw); CHECK_NEAR(w.yaw, 0, 1e-6);
  CHECK(wpSetKeepYaw(1, 9, 8)); wpGet(1, w); CHECK(w.x == 9 && w.y == 8 && w.hasYaw); CHECK_NEAR(w.yaw, -170, 1e-4);
  CHECK(wpDelete(1)); wpGet(1, w); CHECK(!w.valid);
  CHECK(!wpSet(0, 0, 0, 0) && !wpSet(MAX_WP + 1, 0, 0, 0) && !wpDelete(0) && !wpGet(MAX_WP + 1, w));
  CHECK(wpSet(MAX_WP, 5, 5, 0, false));                          // WP16 dipakai
}

TEST(test_A_5_6_waypoints_and_routes_persist_through_store) {
  navFresh();
  wpSet(2, 11, 22, 33, true); uint8_t ids[] = {2, 1}; routeSet(1, ids, 2);
  CHECK(host_store_has("nav", "wp16f")); CHECK(host_store_has("nav", "rt"));
  px = py = 0; hasSaved = false; navBegin();                     // "reboot": muat ulang dari store
  Waypoint w; wpGet(2, w); CHECK(w.valid && w.hasYaw); CHECK_NEAR(w.x, 11, 1e-3); CHECK_NEAR(w.y, 22, 1e-3); CHECK_NEAR(w.yaw, 33, 1e-3);
  uint8_t rt[NUM_ROUTES][MAX_RT]; routeCopyAll(rt); CHECK(rt[1][0] == 2 && rt[1][1] == 1 && rt[1][2] == 0);
}

TEST(test_A_5_6_navbegin_scale_migration_runs_once) {
  // Fixture D.3: store lama tanpa nav/sver. WP dan posisi tersimpan dikali WHEEL_R_CM / WHEEL_R_CM_OLD sekali saja.
  host_store_clear(); host_set_millis(1000);
  navCancel(); hasSaved = false; px = py = 0; poseOk = true;
  WaypointY old[MAX_WP]; memset(old, 0, sizeof old); old[0] = {true, 100.0f, 50.0f, 30.0f, true};
  host_store_put("nav", "wp16y", old, sizeof old);
  PoseBlob pb = {200.0f, 100.0f, 0.5f, 1u, POSE_MAGIC}; host_store_put("nav", "pose", &pb, sizeof pb);
  const float k = WHEEL_R_CM / WHEEL_R_CM_OLD;
  navBegin();
  Waypoint w; wpGet(1, w); float x, y; navPose(x, y);
  CHECK_NEAR(w.x, 100 * k, 1e-2); CHECK_NEAR(w.y, 50 * k, 1e-2); CHECK_NEAR(w.yaw, 30, 1e-4);   // yaw tidak diskala
  CHECK_NEAR(x, 200 * k, 1e-2); CHECK_NEAR(y, 100 * k, 1e-2);
  CHECK(host_store_has("nav", "sver"));
  navBegin();                                                    // boot kedua: tidak boleh konversi ganda
  wpGet(1, w); navPose(x, y);
  CHECK_NEAR(w.x, 100 * k, 1e-2); CHECK_NEAR(x, 200 * k, 1e-2);
}

TEST(test_A_5_6_navbegin_legacy_16_waypoint_format_without_yaw) {
  host_store_clear(); host_set_millis(1000);
  navCancel(); hasSaved = false; px = py = 0; poseOk = true;
  WaypointOld o[MAX_WP]; memset(o, 0, sizeof o); o[2] = {true, 30.0f, 60.0f};
  host_store_put("nav", "wp16", o, sizeof o);                    // format lama: tanpa arah hadap
  navBegin();
  Waypoint w; wpGet(3, w);
  CHECK(w.valid && !w.hasYaw); CHECK_NEAR(w.x, 30 * WHEEL_R_CM / WHEEL_R_CM_OLD, 1e-2);
  CHECK(host_store_has("nav", "wp16f"));                         // dipindah ke format baru
}

// ---------------- R2: referensi dinding per waypoint ----------------
TEST(test_R2_wp16y_migrates_to_wp16f_with_zero_walls) {
  host_store_clear(); host_set_millis(1000);
  navCancel(); hasSaved = false; px = py = 0; poseOk = true;
  uint16_t sv = 2; host_store_put("nav", "sver", &sv, sizeof sv);   // skala sudah terkonversi: hanya format yang diuji
  WaypointY old[MAX_WP]; memset(old, 0, sizeof old); old[0] = {true, 100.0f, 50.0f, 30.0f, true};
  host_store_put("nav", "wp16y", old, sizeof old);
  navBegin();
  Waypoint w; wpGet(1, w);
  CHECK(w.valid && w.hasYaw); CHECK_NEAR(w.x, 100, 1e-3); CHECK_NEAR(w.yaw, 30, 1e-3); CHECK(w.wallL == 0 && w.wallR == 0 && w.wallF == 0);
  CHECK(host_store_has("nav", "wp16f"));
}

TEST(test_R2_wall_reference_roundtrip) {
  navFresh();
  wpSet(1, 10, 20, 90, true, 41.5f, 38.0f); wpSet(2, 5, 5, 0, false); wpSetKeepYaw(3, 7, 8, 30.0f, 0);
  px = py = 0; hasSaved = false; navBegin();
  Waypoint w; wpGet(1, w); CHECK_NEAR(w.wallL, 41.5f, 1e-3); CHECK_NEAR(w.wallR, 38.0f, 1e-3);
  wpGet(2, w); CHECK(w.valid && w.wallL == 0 && w.wallR == 0);
  wpGet(3, w); CHECK_NEAR(w.wallL, 30.0f, 1e-3); CHECK(w.wallR == 0);
}

// ---------------- R3: koreksi pose dan steering dari dinding ----------------
TEST(test_R3_wall_one_side_offset_to_pose_and_steering) {
  Waypoint w = {true, 0, 100, 0, true, 40.0f, 0, 0};
  const float med[3] = {-1, 45, -1}; const bool ok[3] = {false, true, false};
  WallCorr c = wallCalc(w, 3, 50, 0, med, ok);                  // kiri 5 cm lebih jauh dari ref; pose baru 3 cm di kanan waypoint
  CHECK(c.stat == 1); CHECK_NEAR(c.dpx, 2, 1e-4); CHECK_NEAR(c.dpy, 0, 1e-4);
  CHECK_NEAR(c.vx, -WALL_STEER_KP * (5 - WALL_DEADZONE_CM), 1e-4);   // robot di kanan jalur: dorong ke kiri
  const float m2[3] = {-1, 41, -1};
  c = wallCalc(w, 1, 50, 0, m2, ok); CHECK(c.stat == 1); CHECK_NEAR(c.vx, 0, 1e-6); CHECK_NEAR(c.dpx, 0, 1e-4);   // dalam deadzone
}

TEST(test_R3_wall_two_sides_weighted_average_and_fallbacks) {
  Waypoint w = {true, 0, 100, 0, true, 40.0f, 40.0f, 0};
  const float med[3] = {-1, 43, 37}; const bool ok[3] = {false, true, true};
  WallCorr c = wallCalc(w, 0, 50, 0, med, ok);                  // kiri +3, kanan -3: sama-sama 3 cm ke kanan
  CHECK(c.stat == 1); CHECK_NEAR(c.dpx, 3, 1e-4);
  const bool bad[3] = {false, false, false};
  c = wallCalc(w, 0, 50, 0, med, bad); CHECK(c.stat == 2 && c.dpx == 0 && c.vx == 0);     // tidak valid: fallback encoder
  const float big[3] = {-1, 70, -1}; const bool okL[3] = {false, true, false};
  Waypoint wl = {true, 0, 100, 0, true, 40.0f, 0, 0};
  c = wallCalc(wl, 0, 50, 0, big, okL); CHECK(c.stat == 3 && c.dpx == 0 && c.vx == 0);    // offset > batas: hanya dilaporkan
  c = wallCalc(wl, 0, 50, 0.6f, med, okL); CHECK(c.stat == 2);                            // yaw menyimpang > WALL_YAW_MAX_DEG
  Waypoint none = {true, 0, 100, 0, true, 0, 0, 0};
  c = wallCalc(none, 0, 50, 0, med, ok); CHECK(c.stat == 0 && c.dpx == 0 && c.vx == 0);   // tanpa referensi: tidak dipakai
}

TEST(test_R3_navodom_applies_wall_pose_only_while_moving) {
  navFresh();
  wpSet(1, 0, 100, 0, true, 40.0f, 0);
  const float med[3] = {-1, 45, -1}; const bool ok[3] = {false, true, false}; navSetWall(med, ok);
  float wv[3] = {0, 0, 0}, x, y, vx, vy, yt; bool rot;
  navOdom(wv, 0, DT); navPose(x, y); CHECK(x == 0);             // tidak aktif: tidak dikoreksi
  CHECK(navStart(1, SRC_WEB));
  CHECK(navStep(0, 0, 20, true, vx, vy, yt, rot)); CHECK(navWallStat() == 1); CHECK_NEAR(vx, -WALL_STEER_KP * (5 - WALL_DEADZONE_CM), 1e-3);
  for (int i = 0; i < 200; i++) navOdom(wv, 0, DT);
  navPose(x, y); CHECK_MSG(x > 4.0f && x < 5.01f, "pose lateral menuju offset terukur, halus");
  uint16_t st[4]; navStats(st); CHECK(st[3] == 1);              // dipakai, dihitung sekali per leg
  navCancel(); float x0 = x; for (int i = 0; i < 20; i++) navOdom(wv, 0, DT);
  navPose(x, y); CHECK_NEAR(x, x0, 1e-6);
}

// ---------------- R4: dinding normal bukan halangan ----------------
TEST(test_R4_side_wall_at_reference_is_not_obstacle) {
  const float ref[2] = {20.0f, 20.0f};
  { float vx = 30, vy = 0, d[3] = {-1, -1, 18}; avoidObstacles(vx, vy, d, ref); CHECK(vx == 30 && vy == 0); }   // kanan = dinding normal
  { float vx = 30, vy = 0, d[3] = {-1, -1, 18}; avoidObstacles(vx, vy, d); CHECK_MSG(vx < 30, "tanpa ref: perilaku lama"); }
  { float vx = 30, vy = 0, d[3] = {-1, -1, 4}; avoidObstacles(vx, vy, d, ref); CHECK(vx < 6.0f && hypotf(vx, vy) < 30.0f); }   // benda jauh di depan dinding
  { float a = 0, b = 30, c = 0, e = 30, d[3] = {15, -1, -1}; avoidObstacles(a, b, d, ref); avoidObstacles(c, e, d); CHECK(a == c && b == e); }   // depan tidak berubah
}

// ---------------- R5: coba ulang pendekatan ----------------
TEST(test_R5_timeout_retries_then_cancels_with_reason) {
  navFresh(); wpSet(1, 0, 500, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy, yt; bool rot;
  for (int k = 1; k <= NAV_RETRY; k++) {
    host_advance_ms(NAV_TIMEOUT_MS + 1);
    CHECK(navStep(0, 0, 20, true, vx, vy, yt, rot)); CHECK(navActive()); CHECK(navRetryNo() == k);
  }
  host_advance_ms(NAV_TIMEOUT_MS + 1);
  CHECK(!navStep(0, 0, 20, true, vx, vy, yt, rot)); CHECK(!navActive());
  uint8_t why = 0; CHECK(navTakeDeny(why)); CHECK(why == 6);
  uint16_t st[4]; navStats(st); CHECK(st[1] == 1 && st[2] == NAV_RETRY && st[0] == 0);
}

TEST(test_R5_stuck_uses_same_retry_budget_and_arrival_counts) {
  navFresh(); wpSet(1, 0, 500, 0, false); CHECK(navStart(1, SRC_WEB));
  for (int k = 0; k < NAV_RETRY; k++) CHECK(navRetryLeg());
  CHECK(!navRetryLeg()); CHECK(!navActive());
  navFresh(); wpSet(1, 0, 3, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy, yt; bool rot; navStep(0, 0, 20, true, vx, vy, yt, rot);
  uint16_t st[4]; navStats(st); CHECK(st[0] == 1 && !navActive());   // tiba = selesai
}

// ---------------- R7: simpan posisi hemat tulis ----------------
TEST(test_R7_pose_saver_rate_limited_and_fresh) {
  navFresh();
  PoseSaveState s; Telemetry t; t.ready = true; uint32_t now = 1000; int writes = 0; float saved = 0;
  for (int i = 0; i < 600; i++) { now += 500; t.x = 0.2f * i; if (poseSaveDue(s, t, now)) { writes++; saved = t.x; } }   // drift kecil 300 dtk
  CHECK(writes >= 1 && writes <= 30);
  t.x += 100; bool fresh = false;
  for (int i = 0; i < 25 && !fresh; i++) { now += 500; if (poseSaveDue(s, t, now)) { fresh = true; saved = t.x; } }
  CHECK(fresh); CHECK_NEAR(saved, t.x, 1e-6);                   // gerak besar lalu diam: tersimpan dalam <= 12,5 dtk
  int w2 = 0; float base = t.x;
  for (int i = 0; i < 400; i++) { now += 500; t.x = base + ((i & 1) ? 1.0f : -1.0f); if (poseSaveDue(s, t, now)) w2++; }
  CHECK(w2 == 0);                                               // jitter 1 cm: tidak ada tulis
  t.poseLost = true; now += 500; CHECK(poseSaveDue(s, t, now)); // flag validitas berubah: tulis segera
}

// =====================================================================
//  Lapangan v2 (PRD v2.1): L4 median valid + TILT, L5 validasi ref, L6 VERIFY, L9 AVOID, L10 jarak sisi
// =====================================================================
namespace {

// Simulasi sensor: robot menghadap +y (yaw 0). Dinding kiri di x = Lx, dinding depan di y = Fy (dunia). tx, ty = posisi FISIK
// (odometri px, py bisa berbeda: itu yang diperiksa VERIFY). Bacaan bisa dibekukan (fixedL / fixedF >= 0) atau dimatikan.
static const float kIrRangeSim = 30.0f;                            // jangkauan IR depan di simulasi (cm); modul asli: sesuaikan
struct VSim {
  float yaw = 0, rateDps = 0, tx = 0, ty = 0, v = rpmToCms(SPD_NAV_DEF);
  bool  imu = true, hasL = false, hasF = false, sensors = true;
  float Lx = -20, Fy = 250, fixedL = -1, fixedF = -1;
  int   seen[5] = {0, 0, 0, 0, 0};
  std::vector<int> order;                       // urutan fase yang terlihat (tanpa pengulangan)
  // PRD avoid: kotak diam (muka depan di y = boxY, lebar +-boxHalf di sekitar boxX), dinding kanan di Rx, bacaan palsu saat jalan
  float boxX = 0, boxY = 1e9f, boxHalf = 20, Rx = 1e9f, fixedR = -1; bool hasR = false, phantom = false, moving = false;
  bool irSim = true; float irOff = 10.0f;                      // model IR (aktif bila ada kotak)
  uint32_t rng = 12345, phMs = 0; float phVal[3] = {40, 40, 40}; int av[4] = {0, 0, 0, 0}; int firstDir = 0;
  std::vector<std::pair<bool, float>> grips;    // gripper: (on, posisi y fisik saat diperintah)
};
void vsense(const VSim& s, float med[3], bool ok[3]) {
  med[0] = med[1] = med[2] = -1; ok[0] = ok[1] = ok[2] = false;
  if (!s.sensors) return;
  if (s.hasL) { med[1] = s.fixedL >= 0 ? s.fixedL : s.tx - s.Lx; ok[1] = true; }
  if (s.hasR) { med[2] = s.fixedR >= 0 ? s.fixedR : s.Rx - s.tx; ok[2] = true; }
  if (s.hasF) { med[0] = s.fixedF >= 0 ? s.fixedF : s.Fy - s.ty; ok[0] = true; }
  if (s.boxY < 1e8f && s.ty < s.boxY && fabsf(s.tx - s.boxX) <= s.boxHalf) { med[0] = s.boxY - s.ty; ok[0] = true; }
}
// bacaan palsu: hanya saat roda berputar; data lapangan: depan 49..56 cm, sisi 29..56 cm, berganti tiap periode ping
void vphantom(VSim& s, float med[3], bool ok[3]) {
  if (!s.phantom || !s.moving) return;
  if (millis() - s.phMs >= 120) { s.phMs = millis(); for (int i = 0; i < 3; i++) {
      s.rng = s.rng * 1664525u + 1013904223u; s.phVal[i] = (i == 0 ? 45.0f : 29.0f) + (s.rng >> 8) % (i == 0 ? 12 : 28);
    }
  }
  for (int i = 0; i < 3; i++) { med[i] = s.phVal[i]; ok[i] = true; }
}
bool vtick(VSim& s) {
  host_advance_ms(CONTROL_PERIOD_MS);
  float med[3]; bool ok[3]; vsense(s, med, ok); vphantom(s, med, ok); navSetWall(med, ok);
  { bool f = false, l = false, r = false;                       // IR: sinar lurus ke depan dari depan, kiri (-irOff) dan kanan (+irOff)
    if (s.irSim && s.boxY < 1e8f && s.ty < s.boxY && s.boxY - s.ty <= kIrRangeSim) {
      f = fabsf(s.tx - s.boxX) <= s.boxHalf; l = fabsf(s.tx - s.irOff - s.boxX) <= s.boxHalf; r = fabsf(s.tx + s.irOff - s.boxX) <= s.boxHalf; }
    navSetIr(f, l, r); }
  float vx, vy, tgt; bool rot = false;
  bool act = navStep(s.yaw, s.rateDps, s.v, s.imu, vx, vy, tgt, rot);
  float wz = 0;
  if (act && rot) { float e = wrapRad(tgt - s.yaw); wz = fmaxf(-2.0f, fminf(2.0f, 5.0f * e)); }
  { bool g; if (navTakeGrip(g)) s.grips.push_back({g, s.ty}); }
  if (!act) vx = vy = 0;
  s.moving = hypotf(vx, vy) > 5.0f;
  { int a = navAvoidState(); if (a >= 0 && a <= 3) s.av[a]++; if (a == 2 && !s.firstDir) s.firstDir = (int)avoidDir; }
  float wv[3]; kinInverse(vx, vy, wz, wv); navOdom(wv, s.yaw, DT);
  float ex, ey, ew; kinForward(wv, ex, ey, ew);
  float c = cosf(s.yaw), sn = sinf(s.yaw);
  s.tx += (ex * c - ey * sn) * DT; s.ty += (ex * sn + ey * c) * DT;
  s.yaw = wrapRad(s.yaw + ew * DT); s.rateDps = ew * 57.29578f;
  int p = navPhase(); if (p >= 0 && p <= 4) s.seen[p]++;
  if (p > 0 && (s.order.empty() || s.order.back() != p)) s.order.push_back(p);
  return navActive();
}
int vrun(VSim& s, int maxTicks) { int n = 0; while (n < maxTicks && navActive()) { vtick(s); n++; } return n; }

// Satu panggilan navStep dengan bacaan tetap; hasil dalam frame badan.
bool nstep(const float med[3], const bool ok[3], float& vx, float& vy, float v = 50.0f) {
  navSetWall(med, ok); float tgt; bool rot;
  return navStep(0.0f, 0, v, true, vx, vy, tgt, rot);
}

}  // namespace

// ---------------- L4: keputusan menghindar memakai median valid ----------------
TEST(test_L4_raw_single_outlier_does_not_deflect_but_valid_median_does) {
  float hist1[3] = {3, 40, 41}, med1; bool ok1 = usMedian3(hist1, med1);            // satu bacaan mentah 3 cm di antara 40 dan 41
  CHECK(ok1); CHECK_NEAR(med1, 40, 1e-3);
  const float m[3] = {-1, med1, -1}; const bool o[3] = {false, true, false}; float d[3];
  navFresh(); navObstDist(m, o, d);
  { float vx = -30, vy = 0; avoidObstacles(vx, vy, d); CHECK(vx == -30 && vy == 0); }   // median 40 cm: tidak menggeser vektor
  { float vx = -30, vy = 0, raw[3] = {-1, 3, -1}; avoidObstacles(vx, vy, raw); CHECK_MSG(vx > -30 * 0.3f, "bacaan mentah 3 cm memang akan menggeser (alasan L4)"); }
  float hist2[3] = {3, 3.5f, 3.2f}, med2; CHECK(usMedian3(hist2, med2));            // median valid 3 cm: menggeser
  const float m2[3] = {-1, med2, -1}; navObstDist(m2, o, d); CHECK(d[1] > 0 && d[1] < 4);
  { float vx = -30, vy = 0; avoidObstacles(vx, vy, d); CHECK(vx > -30 * 0.3f); }
  const bool bad[3] = {false, false, false}; navObstDist(m2, bad, d); CHECK(d[0] < 0 && d[1] < 0 && d[2] < 0);   // tidak valid = -1
}

TEST(test_L4_slope_keeps_wall_correction_tilt_holds_it_with_stat2) {
  Waypoint w = {true, 0, 100, 0, true, 40.0f, 0, 0};
  const float med[3] = {-1, 45, -1}; const bool ok[3] = {false, true, false};
  WallCorr c = wallCalc(w, 0, 50, 0, med, ok, false); CHECK(c.stat == 1);            // SLOPE / FLAT: koreksi jalan
  c = wallCalc(w, 0, 50, 0, med, ok, true);           CHECK(c.stat == 2 && c.dpx == 0 && c.vx == 0);   // TILT: ditahan
  Waypoint none = {true, 0, 100, 0, true, 0, 0, 0};
  c = wallCalc(none, 0, 50, 0, med, ok, true);        CHECK(c.stat == 0);            // tanpa ref: tidak dipakai
  navFresh(); wpSet(1, 0, 100, 0, false, 40.0f, 0); CHECK(navStart(1, SRC_WEB));
  float vx, vy; bool okp[3] = {false, true, false};
  navSetWall(med, okp, true);  { float t; bool r; navStep(0, 0, 50, true, vx, vy, t, r); } CHECK(navWallStat() == 2);
  navSetWall(med, okp, false); { float t; bool r; navStep(0, 0, 50, true, vx, vy, t, r); } CHECK(navWallStat() == 1);   // SLOPE
  navCancel();
}


// ---------------- L5: validasi referensi dinding saat rekam ----------------
TEST(test_L5_wall_ref_only_when_stable_in_range_flat_and_still) {
  navFresh(); float out[3];
  float med[3] = {-1, 30, 221}; bool ok[3] = {false, true, true};
  for (int i = 0; i < 4; i++) { host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK_MSG(out[0] == 0 && out[1] == 0, "baru 4 sampel: belum cukup");
  host_advance_ms(60); navSetWall(med, ok);
  navWallRefCapture(true, true, out);
  CHECK_NEAR(out[0], 30, 1e-4); CHECK_MSG(out[1] == 0, "dinding 221 cm > WALL_REF_MAX_CM: ref tersimpan 0"); CHECK(out[2] == 0);
  navWallRefCapture(true, false, out); CHECK(out[0] == 0 && out[1] == 0 && out[2] == 0);     // bukan FLAT
  navWallRefCapture(false, true, out); CHECK(out[0] == 0 && out[1] == 0 && out[2] == 0);     // robot bergerak
  // sebaran > WALL_REF_SPREAD_CM
  float vals[5] = {30, 34, 30, 34, 30};
  for (int i = 0; i < 5; i++) { med[1] = vals[i]; host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK_MSG(out[0] == 0, "sebaran 4 cm > 2 cm");
  // stabil, sebaran 2 cm: lolos; batas atas 100 cm; di bawah 5 cm tidak
  float v2[5] = {50, 51, 52, 51, 50};
  for (int i = 0; i < 5; i++) { med[1] = v2[i]; host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK_NEAR(out[0], 50, 1e-4);
  med[1] = 100; med[0] = 100; ok[0] = true;
  for (int i = 0; i < 5; i++) { host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK_NEAR(out[0], 100, 1e-4); CHECK_NEAR(out[2], 100, 1e-4);   // depan ikut direkam
  med[1] = 4; med[0] = 4;
  for (int i = 0; i < 5; i++) { host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK(out[0] == 0 && out[2] == 0);
  med[1] = 30; ok[1] = false;                                                                  // sensor tidak valid memutus riwayat
  for (int i = 0; i < 5; i++) { host_advance_ms(60); navSetWall(med, ok); }
  navWallRefCapture(true, true, out); CHECK(out[0] == 0);
  ok[1] = true; host_advance_ms(60); navSetWall(med, ok); navWallRefCapture(true, true, out); CHECK(out[0] == 0);   // riwayat mulai lagi dari 1 sampel
}

TEST(test_L5_ref_stale_flag_after_1s_of_offset_beyond_limit) {
  navFresh(); wpSet(1, 0, 300, 0, false, 40.0f, 0); CHECK(navStart(1, SRC_WEB));
  const float med[3] = {-1, 75, -1}; const bool ok[3] = {false, true, false}; float vx, vy;
  CHECK(nstep(med, ok, vx, vy)); CHECK(navWallStat() == 3); CHECK(!navRefStale());          // stat 3 baru mulai
  host_advance_ms(900); nstep(med, ok, vx, vy); CHECK(!navRefStale());
  host_advance_ms(200); nstep(med, ok, vx, vy); CHECK_MSG(navRefStale(), "> 1 s offset melebihi batas: refStale");
  const float good[3] = {-1, 41, -1}; nstep(good, ok, vx, vy); CHECK(navWallStat() == 1); CHECK(!navRefStale());
  navCancel();
}

// ---------------- L6: migrasi wp16z -> wp16f, wallF ----------------
TEST(test_L6_wp16z_migrates_to_wp16f_with_zero_front_ref) {
  host_store_clear(); host_set_millis(1000);
  navCancel(); hasSaved = false; px = py = 0; poseOk = true;
  uint16_t sv = 2; host_store_put("nav", "sver", &sv, sizeof sv);
  WaypointZ oz[MAX_WP]; memset(oz, 0, sizeof oz); oz[1] = {true, 11.0f, 22.0f, 33.0f, true, 41.5f, 38.0f};
  host_store_put("nav", "wp16z", oz, sizeof oz);
  navBegin();
  Waypoint w; wpGet(2, w);
  CHECK(w.valid && w.hasYaw); CHECK_NEAR(w.x, 11, 1e-3); CHECK_NEAR(w.yaw, 33, 1e-3); CHECK_NEAR(w.wallL, 41.5, 1e-3); CHECK_NEAR(w.wallR, 38, 1e-3); CHECK(w.wallF == 0);
  CHECK(host_store_has("nav", "wp16f"));
  wpSet(3, 1, 2, 0, false, 10, 20, 30); px = py = 0; hasSaved = false; navBegin();      // wallF bertahan setelah reboot
  wpGet(3, w); CHECK_NEAR(w.wallF, 30, 1e-3); CHECK_NEAR(w.wallL, 10, 1e-3);
  wpSetKeepYaw(3, 5, 6, 0, 0, 77); wpGet(3, w); CHECK_NEAR(w.wallF, 77, 1e-3);
}

// ---------------- L6: VERIFY ----------------
TEST(test_L6_verify_runs_after_rotating_then_shifts_until_reading_matches_ref) {
  navFresh(); wpSet(1, 0, 100, 0, true, 20.0f, 0);                  // WP kiri ref 20 cm, yaw 0
  px = 0; py = 100;                                                  // odometri: sudah di WP
  VSim s; s.hasL = true; s.Lx = -20; s.tx = 8; s.ty = 100; s.yaw = 0.7f;   // fisik 8 cm di kanan jalur, arah hadap masih 40 derajat
  CHECK(navStart(1, SRC_WEB));
  int n = vrun(s, 3000);
  CHECK_MSG(n < 3000, "navigasi harus selesai");
  CHECK(s.order.size() >= 2 && s.order[0] == 2 && s.order[1] == 3);    // tiba (WP sudah di posisi) -> ROTATING -> VERIFY
  ArrLog a = navArrLog();
  CHECK(a.id == 1); CHECK(a.result == 1); CHECK(a.verifyN == 1);
  CHECK_MSG(fabsf(s.tx - 0) <= WALL_ARRIVE_TOL_CM, "bacaan kiri saat tiba = ref +- toleransi");
  CHECK_NEAR(a.ref[0], 20, 1e-4); CHECK(fabsf(a.m[0] - 20) <= WALL_ARRIVE_TOL_CM);
  float x, y; pose(x, y); CHECK(hypotf(x - 0, y - 100) <= 3.0f);   // pose odometri ikut benar
}

TEST(test_L6_verify_front_reference_corrects_along_track_error) {
  navFresh(); wpSet(1, 0, 150, 0, false, 0, 0, 100.0f);              // ref depan 100 cm; tanpa yaw
  px = 0; py = 150;                                                  // odometri mengira tiba
  VSim s; s.hasF = true; s.Fy = 250; s.tx = 0; s.ty = 140;           // fisik 10 cm kurang maju
  CHECK(navStart(1, SRC_WEB)); vrun(s, 3000);
  CHECK(!navActive()); ArrLog a = navArrLog(); CHECK(a.result == 1 && a.verifyN == 1);
  CHECK_MSG(fabsf(s.ty - 150) <= WALL_ARRIVE_TOL_CM, "robot maju sampai bacaan depan = ref");
  CHECK_NEAR(a.ref[2], 100, 1e-4);
}

TEST(test_L6_verify_match_done_without_shift_and_counts_arrival) {
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); px = 0; py = 100;
  VSim s; s.hasL = true; s.Lx = -20; s.tx = 1.0f; s.ty = 100;
  uint8_t seq0 = navDoneSeq(); CHECK(navStart(1, SRC_WEB)); vrun(s, 500);
  CHECK(!navActive()); CHECK(navDoneSeq() == (uint8_t)(seq0 + 1));
  ArrLog a = navArrLog(); CHECK(a.result == 1 && a.verifyN == 0); CHECK_NEAR(s.tx, 1.0, 1e-3);   // tidak bergeser
  CHECK(s.seen[3] * CONTROL_PERIOD_MS >= (int)NAV_VERIFY_SETTLE_MS);                              // diam dulu NAV_VERIFY_SETTLE_MS
}

TEST(test_L6_verify_residual_beyond_limit_is_rejected_pose_untouched_route_not_cancelled) {
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); px = 0; py = 100;
  VSim s; s.hasL = true; s.fixedL = 20 + WALL_CORR_MAX_CM + 10; s.tx = 0; s.ty = 100;
  uint8_t seq0 = navDoneSeq(); CHECK(navStart(1, SRC_WEB)); vrun(s, 500);
  CHECK(!navActive()); CHECK_MSG(navDoneSeq() == (uint8_t)(seq0 + 1), "selesai (tidak membatalkan rute)");
  ArrLog a = navArrLog(); CHECK(a.result == 2 && a.verifyN == 0);
  float x, y; pose(x, y); CHECK_NEAR(x, 0, 1e-3); CHECK_NEAR(y, 100, 1e-3);      // pose tidak disentuh
  CHECK_NEAR(s.tx, 0, 1e-3);
}

TEST(test_L6_verify_budget_exhausted_finishes_with_mismatch_flag) {
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); px = 0; py = 100;
  VSim s; s.hasL = true; s.fixedL = 28; s.tx = 0; s.ty = 100;        // bacaan tidak pernah berubah: tidak bisa cocok
  CHECK(navStart(1, SRC_WEB)); int n = vrun(s, 4000);
  CHECK_MSG(n < 4000, "harus selesai"); ArrLog a = navArrLog();
  CHECK(a.result == 2); CHECK(a.verifyN == NAV_VERIFY_MAX);
  CHECK_NEAR(a.resLat, 8, 0.6);
}

TEST(test_L6_verify_skipped_when_sensor_invalid_or_no_reference) {
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); px = 0; py = 100;
  VSim s; s.hasL = true; s.sensors = false; s.tx = 0; s.ty = 100;    // sensor tidak valid
  CHECK(navStart(1, SRC_WEB)); vrun(s, 500);
  CHECK(!navActive()); CHECK(navArrLog().result == 0);
  navFresh(); wpSet(1, 0, 100, 0, false); px = 0; py = 100;          // ref 0: perilaku lama, tanpa VERIFY
  VSim t; t.tx = 0; t.ty = 100; uint8_t seq0 = navDoneSeq();
  CHECK(navStart(1, SRC_WEB)); vrun(t, 100);
  CHECK(!navActive()); CHECK(t.seen[3] == 0); CHECK(navDoneSeq() == (uint8_t)(seq0 + 1));
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); px = 0; py = 100;   // tanpa IMU: VERIFY dilewati
  VSim u; u.hasL = true; u.imu = false; u.tx = 0; u.ty = 100;
  CHECK(navStart(1, SRC_WEB)); vrun(u, 100); CHECK(!navActive()); CHECK(u.seen[3] == 0);
}

TEST(test_L6_verify_only_at_waypoints_where_robot_stops_not_pass_through) {
  navFresh(); wpSet(1, 0, 60, 0, false, 20.0f, 0); wpSet(2, 0, 120, 0, false);   // WP1 perantara (dilewati) punya ref
  uint8_t ids2[] = {1, 2}; routeSet(0, ids2, 2); px = 0; py = 0;
  VSim s; s.hasL = true; s.tx = 0; s.ty = 0; CHECK(navStartRoute(0, SRC_WEB)); vrun(s, 3000);
  CHECK(!navActive()); CHECK_MSG(s.seen[3] == 0, "waypoint perantara tanpa berhenti: tidak ada VERIFY");
}

TEST(test_L6_sync_runs_verify_only_when_near_and_has_reference) {
  navFresh(); wpSet(1, 0, 100, 0, false, 20.0f, 0); wpSet(2, 0, 100, 0, false);
  px = 0; py = 100;
  CHECK(!navStartVerify(2, SRC_WEB));                               // tanpa referensi
  px = 0; py = 40; CHECK(!navStartVerify(1, SRC_WEB));              // 60 cm dari WP: terlalu jauh
  px = 0; py = 100; navSetPoseValid(false); CHECK(!navStartVerify(1, SRC_WEB)); navSetPoseValid(true);
  px = 0; py = 80;                                                   // 20 cm dari WP, fisik juga bergeser 20 cm
  VSim s; s.hasL = true; s.Lx = -20; s.tx = 6; s.ty = 80;
  CHECK(navStartVerify(1, SRC_WEB)); CHECK(navPhase() == 3);
  vrun(s, 4000); CHECK(!navActive()); CHECK(s.seen[1] == 0 && s.seen[2] == 0);   // hanya VERIFY
  ArrLog a = navArrLog(); CHECK(a.result == 1); CHECK(fabsf(s.tx - 0) <= WALL_ARRIVE_TOL_CM);
}

// ---------------- L9: hindar rintangan depan ----------------




TEST(test_L9_front_wall_reference_is_not_an_obstacle) {
  navFresh(); wpSet(1, 0, 300, 0, false, 0, 0, 40.0f); CHECK(navStart(1, SRC_WEB)); float vx, vy;
  const float med[3] = {30, -1, -1}; const bool ok[3] = {true, false, false};              // 30 >= wallF - 15: dinding tujuan
  CHECK(nstep(med, ok, vx, vy)); CHECK(!navAvoidActive()); CHECK(vy > 1.0f);
  float d[3]; navObstDist(med, ok, d); CHECK(d[0] < 0);
  navCancel();
  navFresh(); wpSet(1, 0, 300, 0, false); CHECK(navStart(1, SRC_WEB));                    // tanpa wallF: tetap halangan
  navObstDist(med, ok, d); CHECK_NEAR(d[0], 30, 1e-4); navCancel();
}

TEST(test_L9_not_triggered_when_not_moving_toward_front_sensor) {
  navFresh(); wpSet(1, 300, 0, 0, false); CHECK(navStart(1, SRC_WEB)); float vx, vy;      // tujuan di kanan: tidak bergerak ke depan
  const float med[3] = {20, -1, -1}; const bool ok[3] = {true, false, false};
  CHECK(nstep(med, ok, vx, vy)); CHECK(!navAvoidActive()); CHECK(vx > 1.0f);
  navCancel();
}

// ---------------- L10: jarak sisi >= 8 cm ----------------
TEST(test_L10_side_push_zone_and_removal_of_component_toward_side) {
  float vx, vy;
  navFresh(); wpSet(1, 0, 300, 0, false); CHECK(navStart(1, SRC_WEB));
  { const float med[3] = {-1, 10, -1}; const bool ok[3] = {false, true, false};            // kiri 10 cm: dorong ke kanan = KP x (12 - 10)
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, SIDE_PUSH_KP * 2, 1e-3); CHECK(vy > 1.0f); }
  { const float med[3] = {-1, 20, -1}; const bool ok[3] = {false, true, false};            // 20 cm: tanpa efek
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, 0, 1e-4); }
  { const float med[3] = {-1, -1, 10}; const bool ok[3] = {false, false, true};            // kanan 10 cm: dorong ke kiri
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, -SIDE_PUSH_KP * 2, 1e-3); }
  { const float med[3] = {-1, 6, -1}; const bool ok[3] = {false, false, false};            // sensor tidak valid: tanpa efek
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, 0, 1e-4); }
  { const float med[3] = {-1, 6, -1}; const bool ok[3] = {false, true, false};             // d = 6: lebih kecil dari SIDE_MIN
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, fminf(SIDE_PUSH_KP * (SIDE_MIN_CM + SIDE_BUFFER_CM - 6), SIDE_PUSH_MAX_CMS), 1e-3); }
  navCancel();
  navFresh(); wpSet(1, -80, 300, 0, false); CHECK(navStart(1, SRC_WEB));                  // tujuan di kiri, kiri 6 cm: komponen ke kiri dihapus
  { const float med[3] = {-1, 6, -1}; const bool ok[3] = {false, true, false};
    CHECK(nstep(med, ok, vx, vy)); CHECK(vx >= 0.0f); CHECK_NEAR(vx, SIDE_PUSH_KP * 6, 1e-3); }
  navCancel();
  navFresh(); wpSet(1, 80, 300, 0, false); CHECK(navStart(1, SRC_WEB));                   // tujuan di kanan, kiri 20 cm: tidak dihapus
  { const float med[3] = {-1, 20, -1}; const bool ok[3] = {false, true, false};
    CHECK(nstep(med, ok, vx, vy)); CHECK(vx > 1.0f); }
  navCancel();
}

TEST(test_L10_both_sides_narrow_lateral_zero_forward_slow_and_stop_when_front_blocked) {
  float vx, vy; navFresh(); wpSet(1, 0, 300, 0, false); CHECK(navStart(1, SRC_WEB));
  { const float med[3] = {-1, 10, 10}; const bool ok[3] = {false, true, true};
    CHECK(nstep(med, ok, vx, vy)); CHECK(vx == 0); CHECK_NEAR(vy, NAV_V_MIN_CMS, 1e-4); }                 // maju pelan
  navSetIr(true, false, false);                                                                            // IR depan aktif = depan terhalang: berhenti
  { const float med[3] = {200, 10, 10}; const bool ok[3] = {true, true, true};
    CHECK(nstep(med, ok, vx, vy)); CHECK(vx == 0 && vy == 0); }
  navCancel();
}

TEST(test_L10_side_min_tracks_smallest_reading_during_route) {
  navFresh(); wpSet(1, 0, 300, 0, false); CHECK(navStart(1, SRC_WEB)); float vx, vy;
  CHECK_NEAR(navSideMin(), -1, 1e-6);
  { const float med[3] = {-1, 30, 18}; const bool ok[3] = {false, true, true}; nstep(med, ok, vx, vy); CHECK_NEAR(navSideMin(), 18, 1e-4); }
  { const float med[3] = {-1, 14, 25}; const bool ok[3] = {false, true, true}; nstep(med, ok, vx, vy); CHECK_NEAR(navSideMin(), 14, 1e-4); }
  { const float med[3] = {-1, 40, 40}; const bool ok[3] = {false, true, true}; nstep(med, ok, vx, vy); CHECK_NEAR(navSideMin(), 14, 1e-4); }
  navCancel(); CHECK_NEAR(navSideMin(), 14, 1e-4);                                                           // bertahan sampai rute berikutnya
  wpSet(2, 0, 300, 0, false); CHECK(navStart(2, SRC_WEB)); CHECK_NEAR(navSideMin(), -1, 1e-6); navCancel();
}

// ---------------- L7: skala odometri ----------------
TEST(test_L7_odom_scale_defaults_do_not_change_pose) {
  navFresh(); float wv[3], x, y;
  kinInverse(0, 30, 0, wv); for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 0, 0);
  pose(x, y); CHECK_NEAR(y, 30 * ODOM_SCALE_FWD, 0.05);
  navResetOdom(); kinInverse(0, -30, 0, wv); for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 0, 0);
  pose(x, y); CHECK_NEAR(y, -30 * ODOM_SCALE_BWD, 0.05);
}

// ---------------- perbaikan noise dinding sisi ----------------
TEST(test_wall_steer_capped_and_held_briefly_when_side_sensor_drops) {
  navFresh(); wpSet(1, 0, 300, 0, false, 40.0f, 0); CHECK(navStart(1, SRC_WEB)); float vx, vy;
  { const float med[3] = {-1, 60, -1}; const bool ok[3] = {false, true, false};            // offset 20 cm: dorongan dibatasi
    CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, -WALL_STEER_MAX_CMS, 1e-4); CHECK(vy > 1.0f); }
  float held = vx;
  { const float med[3] = {-1, -1, -1}; const bool ok[3] = {false, false, false};           // sensor putus 100 ms: koreksi dipertahankan
    host_advance_ms(100); CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, held, 1e-4);
    host_advance_ms(300); CHECK(nstep(med, ok, vx, vy)); CHECK_NEAR(vx, 0, 1e-4); }       // lewat WALL_HOLD_MS: nol
  navCancel();
}
TEST(test_side_wall_at_20cm_does_not_slow_forward_motion) {
  navFresh(); wpSet(1, 0, 300, 0, false); CHECK(navStart(1, SRC_WEB)); float vx, vy;
  const float med[3] = {-1, 20, -1}; const bool ok[3] = {false, true, false};
  CHECK(nstep(med, ok, vx, vy, 100.0f)); CHECK_NEAR(vx, 0, 1e-4); CHECK_NEAR(vy, 100.0f, 1e-3);   // sejajar dinding 20 cm: tidak ada efek
  navCancel();
}

// ---------------- Gripper (1 solenoid) sebagai langkah rute ----------------
TEST(test_gripper_route_set_accepts_only_waypoints_and_grip_codes) {
  navFresh(); uint8_t in[] = {1, GRIP_STEP_ON, 17, 0, 103, GRIP_STEP_OFF, 2};
  routeSet(0, in, 7); uint8_t rt[NUM_ROUTES][MAX_RT]; routeCopyAll(rt);
  CHECK(rt[0][0] == 1 && rt[0][1] == GRIP_STEP_ON && rt[0][2] == GRIP_STEP_OFF && rt[0][3] == 2 && rt[0][4] == 0);
}

TEST(test_gripper_step_between_waypoints_stops_there_actuates_then_continues) {   // preset: WP1, WP2, gripper ON, WP3, gripper OFF
  navFresh(); wpSet(1, 0, 60, 0, false); wpSet(2, 0, 120, 0, false); wpSet(3, 0, 180, 0, false);
  uint8_t in[] = {1, 2, GRIP_STEP_ON, 3, GRIP_STEP_OFF}; routeSet(0, in, 5);
  px = 0; py = 0; VSim s; uint8_t seq0 = navDoneSeq();
  CHECK(navStartRoute(0, SRC_WEB)); int n = vrun(s, 6000);
  CHECK_MSG(n < 6000, "rute selesai"); CHECK(navDoneSeq() == (uint8_t)(seq0 + 1));
  CHECK(s.grips.size() == 2);
  CHECK(s.grips[0].first == true  && fabsf(s.grips[0].second - 120) < 8.0f);        // jepit di WP2 (berhenti dulu)
  CHECK(s.grips[1].first == false && fabsf(s.grips[1].second - 180) < 8.0f);        // lepas di WP3 (langkah terakhir)
  CHECK(s.seen[4] * CONTROL_PERIOD_MS >= 2 * (int)GRIPPER_ACTUATE_MS - 100);        // diam menunggu solenoid
}

TEST(test_gripper_route_starting_with_grip_and_route_without_waypoint) {
  navFresh(); wpSet(1, 0, 60, 0, false);
  uint8_t in[] = {GRIP_STEP_ON, 1}; routeSet(0, in, 2); px = 0; py = 0; VSim s;
  CHECK(navStartRoute(0, SRC_WEB)); CHECK(navPhase() == 4);
  vrun(s, 4000); CHECK(!navActive()); CHECK(s.grips.size() == 1 && s.grips[0].first && s.grips[0].second < 5.0f);   // jepit di tempat, baru jalan
  uint8_t only[] = {GRIP_STEP_ON}; routeSet(1, only, 1); CHECK(!navRouteReady(1)); CHECK(!navStartRoute(1, SRC_WEB));
}

TEST(test_gripper_cancel_during_grip_step_stops_route) {
  navFresh(); wpSet(1, 0, 60, 0, false);
  uint8_t in[] = {GRIP_STEP_ON, 1}; routeSet(0, in, 2); px = 0; py = 0;
  CHECK(navStartRoute(0, SRC_WEB)); bool g; CHECK(navTakeGrip(g) && g); CHECK(!navTakeGrip(g));   // hanya sekali per langkah
  navCancel(); CHECK(!navActive());
}

// ---------------- PRD avoid v1.0: tren, HOLD, SHIFT, PASS ----------------
TEST(test_H1_ir_front_starts_shift_immediately_and_holds_wall_correction) {
  navFresh(); wpSet(1, 0, 300, 0, false, 40.0f, 0); CHECK(navStart(1, SRC_WEB)); float vx, vy;
  const float med[3] = {8, 45, -1}; const bool ok[3] = {true, true, false}; navSetIr(true, false, false);   // IR depan aktif
  CHECK(nstep(med, ok, vx, vy)); CHECK(navAvoidState() == 2); CHECK(vx <= 0.0f); CHECK(navWallStat() == 4);   // IR aktif, kiri lapang: langsung geser kiri (tanpa jeda)
  float wv[3] = {0, 0, 0}, x0, y0, x1, y1; pose(x0, y0);
  for (int i = 0; i < 50; i++) { navOdom(wv, 0, DT); }
  pose(x1, y1); CHECK(x0 == x1 && y0 == y1);                                                       // pose tidak ditarik dinding
  CHECK(navAvoiding()); navCancel();
}

TEST(test_H7_phantom_readings_while_moving_do_not_stop_the_robot) {
  int arrived = 0, holdTicks = 0, ticksTotal = 0;
  for (uint32_t seed = 1; seed <= 20; seed++) {
    navFresh(); wpSet(1, 0, 300, 0, false); px = 0; py = 0;
    VSim s; s.phantom = true; s.hasF = true; s.Fy = 500; s.hasL = true; s.Lx = -150; s.hasR = true; s.Rx = 150; s.rng = seed * 7919u;
    CHECK(navStart(1, SRC_WEB)); int n = vrun(s, 2000);
    if (!navActive() && hypotf(s.tx, s.ty - 300) < 12.0f) arrived++;
    holdTicks += s.av[1]; ticksTotal += n;
  }
  CHECK_MSG(arrived == 20, "waypoint tercapai pada semua uji bacaan palsu");
  // derau seragam acak 29..56 cm kadang kebetulan lolos tren; HOLD-nya batal sendiri (<= ~0,5 s) karena saat diam bacaan kembali nyata
  CHECK_MSG(holdTicks * CONTROL_PERIOD_MS <= 20 * 1500, "HOLD karena bacaan palsu singkat dan batal sendiri");
  (void)ticksTotal;
}

TEST(test_H7_real_box_hold_shift_to_wider_side_pass_then_reach_waypoint) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = 0; s.boxHalf = 20; s.hasL = true; s.Lx = -100; s.hasR = true; s.Rx = 200;   // kanan lebih lapang
  float minGap = 1e9f, maxAbsX = 0; CHECK(navStart(1, SRC_WEB));
  while (navActive() && millis() < 120000) { vtick(s); if (s.ty < s.boxY && fabsf(s.tx - s.boxX) <= s.boxHalf) minGap = fminf(minGap, s.boxY - s.ty); if (s.ty > s.boxY - 5 && s.ty < s.boxY + 30) maxAbsX = fmaxf(maxAbsX, fabsf(s.tx)); }
  CHECK(!navActive()); CHECK(hypotf(s.tx, s.ty - 400) < 12.0f);
  CHECK(s.av[2] > 0 && s.av[3] > 0);                                                   // SHIFT dan PASS terlihat (HOLD sesaat, langsung geser)
  CHECK(s.firstDir == 1);                                                              // sisi lebih lapang (kanan)
  CHECK_MSG(minGap > OBST_STOP_CM, "berhenti sebelum menyentuh kotak");
  CHECK_MSG(maxAbsX >= s.boxHalf + 12.0f, "melewati kotak dengan jarak samping");
  CHECK_MSG(maxAbsX >= AVOID_SHIFT_MIN_CM - 5.0f, "geser samping total sekitar 50 cm");
  uint8_t why; CHECK(!navTakeDeny(why));
}

TEST(test_H7_both_sides_under_20cm_does_not_shift_and_cancels_with_deny7) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 150; s.boxHalf = 80; s.hasL = true; s.Lx = -18; s.hasR = true; s.Rx = 18;
  CHECK(navStart(1, SRC_WEB)); int n = vrun(s, (int)((AVOID_MAX_MS + 8000) / CONTROL_PERIOD_MS));
  CHECK(!navActive()); CHECK(n * CONTROL_PERIOD_MS > (int)AVOID_MAX_MS - 4000); CHECK(s.av[2] == 0 && s.av[3] == 0);
  uint8_t why = 0; CHECK(navTakeDeny(why)); CHECK(why == 7);
}

TEST(test_H4_equal_sides_choose_side_toward_waypoint) {
  navFresh(); wpSet(1, -120, 400, 0, false); px = 0; py = 0;                           // tujuan agak ke kiri
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = -40; s.boxHalf = 70; s.hasL = true; s.fixedL = 100; s.hasR = true; s.fixedR = 102;   // dl ~ dr (selisih <= 3 cm)
  CHECK(navStart(1, SRC_WEB));
  while (navActive() && s.av[2] == 0 && millis() < 60000) vtick(s);
  CHECK(s.firstDir == -1);
  navCancel();
}

TEST(test_H9_fast_approach_stops_before_box_even_at_nav_speed) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = 0; s.boxHalf = 20; s.hasL = true; s.Lx = -100; s.hasR = true; s.Rx = 200;
  CHECK(navStart(1, SRC_WEB)); float maxTy = 0;
  while (navActive() && s.av[2] == 0 && millis() < 60000) { vtick(s); if (s.ty > maxTy) maxTy = s.ty; }
  CHECK_MSG(maxTy <= s.boxY - OBST_STOP_CM - 5.0f, "robot berhenti sebelum menyentuh kotak sebelum mulai geser");
  navCancel();
}

// ---------------- IR (depan penengah, kiri/kanan menghadap depan) + jarak dinding 10 cm ----------------
TEST(test_H10_right_ir_only_shifts_left_at_least_30cm_then_reaches_waypoint) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.boxX = 23; s.boxY = 200; s.boxHalf = 15; s.hasL = true; s.Lx = -150; s.hasR = true; s.Rx = 250;   // kotak di kanan jalur: sinar ultrasonik tengah lolos, IR kanan kena
  CHECK(navStart(1, SRC_WEB)); float minX = 0; bool front = false;
  while (navActive() && millis() < 90000) { vtick(s); if (s.tx < minX) minX = s.tx; if (s.ty > 150 && s.ty < s.boxY) { bool f = navAvoidState() == 0; (void)f; } }
  CHECK_MSG(s.firstDir == -1, "IR kanan aktif -> geser kiri");
  CHECK_MSG(-minX >= AVOID_IR_SHIFT_CM - 1.0f, "geser samping minimal 30 cm");
  CHECK(s.av[2] > 0 && s.av[3] > 0); CHECK(s.av[1] == 0);     // tanpa HOLD: langsung geser
  CHECK(!navActive() && hypotf(s.tx, s.ty - 400) < 12.0f);
  (void)front;
}

TEST(test_H10_left_ir_only_shifts_right) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.boxX = -23; s.boxY = 200; s.boxHalf = 15; s.hasL = true; s.Lx = -250; s.hasR = true; s.Rx = 150;
  CHECK(navStart(1, SRC_WEB)); float maxX = 0;
  while (navActive() && millis() < 90000) { vtick(s); if (s.tx > maxX) maxX = s.tx; }
  CHECK(s.firstDir == 1); CHECK(maxX >= AVOID_IR_SHIFT_CM - 1.0f);
  CHECK(!navActive() && hypotf(s.tx, s.ty - 400) < 12.0f);
}

TEST(test_H10_both_side_ir_stands_still_and_cancels_with_deny7) {
  navFresh(); wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx = 1, vy = 1; const float med[3] = {-1, -1, -1}; const bool ok[3] = {false, false, false};
  navSetIr(false, true, true); int n = 0;
  while (navActive() && n < (int)(AVOID_MAX_MS / 20) + 200) { host_advance_ms(20); nstep(med, ok, vx, vy); n++; if (navActive()) CHECK(vx == 0 && vy == 0); }
  CHECK(!navActive()); uint8_t why = 0; CHECK(navTakeDeny(why)); CHECK(why == 7);
}

TEST(test_H10_side_ir_ignored_when_same_side_wall_is_close) {
  navFresh(); wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy; const float med[3] = {-1, -1, 15}; const bool ok[3] = {false, false, true};   // dinding kanan 15 cm (< 25)
  navSetIr(false, false, true);
  CHECK(nstep(med, ok, vx, vy)); CHECK(navAvoidState() == 0); CHECK(vy > 1.0f);                // IR kanan = dinding: diabaikan, maju terus
  navCancel();
}

TEST(test_H10_front_shift_goes_full_50cm_but_stops_at_10cm_from_wall) {
  for (int variant = 0; variant < 2; variant++) {
    navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
    float Lx = variant == 0 ? -75.0f : -55.0f;                   // 0: ruang >= 50 cm ke kiri; 1: ruang terbatas (45 cm sampai jarak 10 cm)
    VSim s; s.hasL = true; s.Lx = Lx; s.hasR = true; s.Rx = 15; s.boxX = 0; s.boxY = 200; s.boxHalf = 20;   // kanan sempit (15 cm) -> pilih kiri
    CHECK(navStart(1, SRC_WEB)); float minX = 0, minGap = 1e9f;
    while (navActive() && millis() < 90000) { vtick(s); if (s.tx < minX) minX = s.tx; if (s.ty < s.boxY + 5) minGap = fminf(minGap, s.tx - s.Lx); }
    CHECK(s.firstDir == -1);
    CHECK_MSG(minGap >= AVOID_WALL_GAP_CM - 1.0f, "tidak pernah lebih dekat dari ~10 cm ke dinding");
    if (variant == 0) CHECK_MSG(-minX >= AVOID_SHIFT_MIN_CM - 3.0f, "ruang cukup: geser penuh 50 cm");
    else CHECK_MSG(-minX >= 38.0f, "ruang terbatas: geser sejauh jarak dinding 10 cm mengizinkan");
    CHECK(!navActive() && hypotf(s.tx, s.ty - 400) < 15.0f);
  }
}

TEST(test_H9_single_ultrasonic_readings_never_trigger_only_ir_does) {
  navFresh(); wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy; bool ok[3] = {true, false, false};
  for (float d : {60.0f, 25.0f, 8.0f, 2.0f}) { float med[3] = {d, -1, -1}; CHECK(nstep(med, ok, vx, vy, 100.0f)); CHECK(navAvoidState() == 0); CHECK(vy > 1.0f); }   // ultrasonik depan sedekat apa pun, IR diam: jalan terus
  navSetIr(true, false, false);
  for (int i = 0; i < 40; i++) { float med[3] = {200, -1, -1}; host_advance_ms(20); nstep(med, ok, vx, vy, 100.0f); }   // IR aktif walau ultrasonik bilang jauh: nyata; sisi tidak ada -> berhenti (bertahap)
  CHECK(navAvoidState() == 1); CHECK(vx == 0 && vy == 0);
  navCancel();
}

// ---------------- Peringatan dini ultrasonik, merayap, gerak halus ----------------
TEST(test_H8_real_box_shift_starts_at_ir_and_motion_is_smooth) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = 0; s.boxHalf = 20; s.hasL = true; s.Lx = -100; s.hasR = true; s.Rx = 200;
  CHECK(navStart(1, SRC_WEB));
  float px0 = s.tx, py0 = s.ty, pvx = 0, pvy = 0, maxDv = 0, vAtIr = -1; const float dtS = CONTROL_PERIOD_MS / 1000.0f; bool seenShift = false;
  while (navActive() && millis() < 120000) {
    vtick(s); float vx = (s.tx - px0) / dtS, vy = (s.ty - py0) / dtS; px0 = s.tx; py0 = s.ty;
    if (navAvoidState() == 2 && !seenShift) { seenShift = true; vAtIr = hypotf(vx, vy); }
    if (navAvoidState() != 0 || navAvoidScale() < 0.999f) maxDv = fmaxf(maxDv, hypotf(vx - pvx, vy - pvy));
    pvx = vx; pvy = vy;
  }
  CHECK_MSG(seenShift && vAtIr <= AVOID_CRAWL_CMS + 10.0f, "saat IR mendeteksi robot sudah merayap (peringatan dini ultrasonik)");
  CHECK_MSG(maxDv <= AVOID_EBRAKE_CMS2 * dtS + 3.0f, "tidak ada lonjakan kecepatan selama hindar (perubahan per tick <= batas rem darurat)");
  CHECK(!navActive() && hypotf(s.tx, s.ty - 400) < 12.0f);
}

TEST(test_H8_warn_then_object_removed_before_ir_recovers_without_avoid) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = 0; s.boxHalf = 20; s.hasL = true; s.Lx = -100; s.hasR = true; s.Rx = 200;
  CHECK(navStart(1, SRC_WEB)); bool warned = false;
  while (navActive() && millis() < 60000) { vtick(s); if (navAvoidScale() < 0.9f && s.ty < s.boxY - 70) { warned = true; s.boxY = 1e9f; break; } }
  CHECK_MSG(warned, "tren ultrasonik yang konsisten memicu pelambatan sebelum IR");
  vrun(s, 12000);
  CHECK(s.av[2] == 0 && s.av[3] == 0); CHECK(!navActive()); CHECK(hypotf(s.tx, s.ty - 400) < 12.0f);
}

TEST(test_H8_object_invisible_to_ir_is_stopped_by_ultrasonic_trend_and_avoided) {
  navFresh(); wpSet(1, 0, 400, 0, false); px = 0; py = 0;
  VSim s; s.irSim = false; s.hasF = true; s.Fy = 600; s.boxY = 200; s.boxX = 0; s.boxHalf = 20; s.hasL = true; s.Lx = -100; s.hasR = true; s.Rx = 200;
  CHECK(navStart(1, SRC_WEB)); float maxTy = 0;
  while (navActive() && millis() < 120000) { vtick(s); if (s.av[2] == 0 && s.ty > maxTy) maxTy = s.ty; }
  CHECK_MSG(maxTy <= s.boxY - 8.0f, "berhenti sebelum menyentuh benda yang tidak terlihat IR");
  CHECK(s.av[2] > 0 && s.av[3] > 0); CHECK(!navActive() && hypotf(s.tx, s.ty - 400) < 12.0f);
}

TEST(test_H9_front_ir_active_brakes_smoothly_to_stop_when_no_side_is_free) {
  navFresh(); wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx = 0, vy = 0; const float med[3] = {200, -1, -1}; const bool ok[3] = {true, false, false};
  for (int i = 0; i < 5; i++) { host_advance_ms(20); nstep(med, ok, vx, vy, 100.0f); }
  float prev = vy; CHECK(prev > 50.0f);                                                        // jalan normal
  navSetIr(true, false, false); float maxDrop = 0;
  for (int i = 0; i < 60; i++) { host_advance_ms(20); nstep(med, ok, vx, vy, 100.0f); maxDrop = fmaxf(maxDrop, prev - vy); prev = vy; CHECK(vx == 0); }
  CHECK(navAvoidState() == 1); CHECK(vy == 0);
  CHECK_MSG(maxDrop <= AVOID_EBRAKE_CMS2 * 0.02f + 0.5f, "turun bertahap, bukan seketika");
  CHECK_MSG(maxDrop > 1.0f, "tidak melompat ke nol sebelum waktunya");
  navCancel();
}
