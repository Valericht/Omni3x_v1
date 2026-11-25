// A.5.4: slope handling. Public navigation API only (navigation.cpp is compiled in test_navigation.cpp).
#include "Arduino.h"
#include "navigation.h"
#include "motors.h"
#include "imuassist.h"
#include "host_stubs.h"
#include "test_main.h"

namespace {

const float DT = CONTROL_PERIOD_MS / 1000.0f;

void rampFresh() {
  navSetIr(false, false, false);
  host_store_clear(); host_set_millis(1000);
  navCancel(); navSetPoseValid(true); navSetPoseDegraded(false); navResetOdom();
  for (int i = 1; i <= MAX_WP; i++) { wpDelete(i); navRampMark(i, false); }
}
void pose(float& x, float& y) { navPose(x, y); }

}  // namespace

// (a) R1: forward *= cos(pitch), lateral *= cos(roll), before body->world rotation; cos clamped at COS_CLAMP_MIN.
TEST(test_A_5_4_ramp_cos_projection) {
  rampFresh();
  float wv[3], x, y;
  kinInverse(0, 30, 0, wv);                                              // forward 30 cm/s, yaw 0 -> world +y
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 10.0f, 0.0f);       // pitch 10 deg, 1 s
  pose(x, y); CHECK_NEAR(x, 0, 0.05); CHECK_NEAR(y, 30 * cosf(10 * (float)M_PI / 180), 0.05);
  navResetOdom();
  kinInverse(30, 0, 0, wv);                                              // strafe 30 cm/s
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 10.0f, 20.0f);      // pitch must not touch lateral; roll 20 deg does
  pose(x, y); CHECK_NEAR(x, 30 * cosf(20 * (float)M_PI / 180), 0.05); CHECK_NEAR(y, 0, 0.05);
  navResetOdom();
  kinInverse(0, 30, 0, wv);                                              // projection happens before rotation: yaw +90 -> world -x
  for (int i = 0; i < 50; i++) navOdom(wv, (float)M_PI / 2, DT, 10.0f, 0.0f);
  pose(x, y); CHECK_NEAR(x, -30 * cosf(10 * (float)M_PI / 180), 0.05); CHECK_NEAR(y, 0, 0.05);
  navResetOdom();
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 85.0f, 0.0f);       // 85 deg: cos = 0.087 -> clamped to COS_CLAMP_MIN, never flips sign
  pose(x, y); CHECK_NEAR(y, 30 * COS_CLAMP_MIN, 0.05);
}

// (b) R2: uncertain + encTrust < ENCODER_TRUST_FREEZE -> x,y frozen, yaw still integrates; above it, displacement is weighted.
TEST(test_A_5_4_ramp_freeze_when_trust_low_yaw_keeps_running) {
  rampFresh();
  float wv[3], x, y, yawEnc = 0;
  kinInverse(0, 30, 20 * 1.0f, wv);                                      // forward + rotation
  bool deg = navOdom(wv, 0.0f, DT, 0, 0, ENCODER_TRUST_FREEZE - 0.01f, true, &yawEnc);
  CHECK(deg);
  for (int i = 0; i < 49; i++) navOdom(wv, 0.0f, DT, 0, 0, ENCODER_TRUST_FREEZE - 0.01f, true, &yawEnc);
  pose(x, y); CHECK(x == 0 && y == 0);                                   // frozen
  CHECK_MSG(fabsf(yawEnc) > 1.0f, "yaw encoder must keep integrating while x,y are frozen");
  float yawFrozen = yawEnc;
  navOdom(wv, 0.0f, DT, 0, 0, ENCODER_TRUST_FREEZE - 0.01f, true, &yawEnc);
  CHECK(yawEnc != yawFrozen);

  navResetOdom();                                                        // trust at the threshold: not frozen, weighted by trust
  kinInverse(0, 30, 0, wv);
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 0, 0, ENCODER_TRUST_FREEZE, true);
  pose(x, y); CHECK_NEAR(y, 30 * ENCODER_TRUST_FREEZE, 0.05);

  navResetOdom();                                                        // not uncertain: trust is ignored (flat, no slip)
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT, 0, 0, 0.1f, false);
  pose(x, y); CHECK_NEAR(y, 30, 0.05);
}

// (c) R5: snap to the known ramp-end point only when the current pose is within RAMP_SNAP_TOL_CM.
TEST(test_A_5_4_ramp_snap_within_tolerance_only) {
  rampFresh();
  wpSet(1, 100.0f, 200.0f, 0.0f, false);
  float x, y;
  CHECK_MSG(!navRampSnap(RAMP_SNAP_TOL_CM), "no ramp marker -> skip snap");
  navRampMark(1, true);
  float wv[3]; kinInverse(0, 30, 0, wv);
  for (int i = 0; i < 50; i++) navOdom(wv, 0.0f, DT);                    // pose (0, 30): far from WP1
  CHECK(!navRampSnap(RAMP_SNAP_TOL_CM));
  pose(x, y); CHECK_NEAR(x, 0, 0.05); CHECK_NEAR(y, 30, 0.05);           // untouched
  navResetOdom();
  for (int i = 0; i < 50 * 6; i++) navOdom(wv, 0.0f, DT);                // pose (0, 180): 100 cm from WP1 in x -> still out
  CHECK(!navRampSnap(RAMP_SNAP_TOL_CM));
  navResetOdom();
  float v[3]; kinInverse(30, 0, 0, v);
  for (int i = 0; i < 50 * 3; i++) navOdom(v, 0.0f, DT);                 // x = 90
  for (int i = 0; i < 50 * 6; i++) navOdom(wv, 0.0f, DT);                // y = 180 -> pose (90, 180): 22.4 cm from WP1
  CHECK(!navRampSnap(RAMP_SNAP_TOL_CM));
  for (int i = 0; i < 16; i++) navOdom(v, 0.0f, DT);                     // x = 99.6
  for (int i = 0; i < 25; i++) navOdom(wv, 0.0f, DT);                    // y = 195 -> pose (99.6, 195): 5 cm from WP1
  CHECK(navRampSnap(RAMP_SNAP_TOL_CM));
  pose(x, y); CHECK_NEAR(x, 100, 1e-3); CHECK_NEAR(y, 200, 1e-3);        // snapped exactly to WP1
  navRampMark(1, false);
  navResetOdom(); CHECK(!navRampSnap(RAMP_SNAP_TOL_CM));                 // marker removed
}

// R3: navStart / navStartRoute are refused while poseDegraded; allowed again once it clears.
TEST(test_A_5_4_ramp_new_navigation_refused_while_pose_degraded) {
  rampFresh();
  wpSet(1, 50.0f, 50.0f, 0.0f, false);
  uint8_t ids[1] = {1}; routeSet(0, ids, 1);
  navSetPoseDegraded(true);
  CHECK(navPoseDegraded());
  CHECK(!navStart(1, SRC_WEB)); CHECK(!navStartRoute(0, SRC_WEB)); CHECK(!navActive());
  navSetPoseDegraded(false);
  CHECK(navStart(1, SRC_WEB)); CHECK(navActive());
  navCancel();
  CHECK(navStartRoute(0, SRC_WEB)); navCancel();
}

TEST(test_R6_ramp_marks_persist_across_reboot) {
  rampFresh();
  wpSet(3, 3, 0, 0, false); navRampMark(3, true);
  CHECK(host_store_has("nav", "ramp"));
  navBegin(); navResetOdom();                                    // "reboot": tanda dimuat dari store
  CHECK(navRampSnap(5.0f));
  navRampMark(3, false); navBegin(); navResetOdom();
  CHECK(!navRampSnap(5.0f));                                     // tanda dihapus ikut tersimpan
}

// ---------------- L3: lantai kecepatan setelah semua pengali ----------------
TEST(test_L3_speed_floor_after_all_multipliers) {
  rampFresh();
  wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy, yt; bool rot; const float lim = rpmToCms(SPD_NAV_DEF);
  CHECK(navStep(0, 0, lim, true, vx, vy, yt, rot));                      // jarak 400 cm: lantai NAV_SLOPE_MIN_RPM berlaku
  const float floorCms = rpmToCms(NAV_SLOPE_MIN_RPM);
  // kasus lapangan 1:40: batas 100 RPM x 0.35 x 0.5 x 0.3 -> ~5 RPM. Dengan min(sat, lereng) lalu lantai: >= 40 RPM
  float dx = 0, dy = rpmToCms(100.0f), ax = dx, ay = dy;
  float mul = translationScale(0.35f, 0.5f); ax *= mul; ay *= mul; ay *= 0.3f;   // avoidObstacles 0.3 sudah ikut di dirY di main; di sini dipakai sebagai pengali sisa
  navSpeedFloor(ax, ay, dx, dy, lim, false);
  CHECK_MSG(cmsToRpm(hypotf(ax, ay)) >= NAV_SLOPE_MIN_RPM - 0.01f, "sat 0.35 + lereng: >= 40 RPM");
  CHECK_NEAR(ax, 0, 1e-4); CHECK(ay > 0);                                // arah dipertahankan
  float bx = 0, by = 2.0f;                                               // trust 0 -> vektor nol: arah dari dirX/dirY
  bx = by = 0; navSpeedFloor(bx, by, 0, 80.0f, lim, false); CHECK_NEAR(by, floorCms, 1e-3);
  float cx = 0, cy = floorCms + 5.0f; navSpeedFloor(cx, cy, 0, 80.0f, lim, false); CHECK_NEAR(cy, floorCms + 5.0f, 1e-4);   // sudah cukup cepat
  float ex = 0, ey = 1.0f; navSpeedFloor(ex, ey, 0, 80.0f, 10.0f, false); CHECK_NEAR(ey, 10.0f, 1e-4);                     // dibatasi batas nav
  float fx = 0, fy = 1.0f; navSpeedFloor(fx, fy, 0, 80.0f, lim, true); CHECK_NEAR(fy, 1.0f, 1e-6);                          // rintangan terkonfirmasi: tidak dinaikkan
  navCancel();

  rampFresh();                                                            // dekat target (<= NAV_SLOW_CM): lantai NAV_V_MIN_CMS
  wpSet(1, 0, 30, 0, false); CHECK(navStart(1, SRC_WEB));
  CHECK(navStep(0, 0, lim, true, vx, vy, yt, rot));
  float gx = 0, gy = 1.0f; navSpeedFloor(gx, gy, 0, 50.0f, lim, false); CHECK_NEAR(gy, NAV_V_MIN_CMS, 1e-4);
  navCancel();
  float hx = 0, hy = 1.0f; navSpeedFloor(hx, hy, 0, 50.0f, lim, false); CHECK_NEAR(hy, 1.0f, 1e-6);                          // tidak aktif: tidak berlaku
}

TEST(test_L3_obstacle_confirmed_stops_floor_but_not_distant_or_behind) {
  const float d1[3] = {8, -1, -1}, d2[3] = {12, -1, -1}, d3[3] = {-1, 8, -1}, d4[3] = {-1, -1, -1};
  CHECK(navObstConfirmed(d1, 0, 50));                      // depan <= OBST_STOP_CM, maju
  CHECK(!navObstConfirmed(d1, 0, -50));                    // mundur: menjauh
  CHECK(!navObstConfirmed(d2, 0, 50));                     // 12 cm: belum terkonfirmasi
  CHECK(navObstConfirmed(d3, -20, 0));                     // kiri 8 cm, bergerak ke kiri
  CHECK(!navObstConfirmed(d3, 20, 0));
  CHECK(!navObstConfirmed(d4, 0, 50));
}

TEST(test_L3_floor_not_applied_outside_moving_or_during_avoid) {
  rampFresh(); wpSet(1, 0, 400, 0, false); CHECK(navStart(1, SRC_WEB));
  float vx, vy, yt; bool rot; const float lim = rpmToCms(SPD_NAV_DEF);
  const float med[3] = {8, -1, -1}; const bool ok[3] = {true, false, false}; navSetWall(med, ok); navSetIr(true, false, false);   // IR depan aktif: halangan nyata
  CHECK(navStep(0, 0, lim, true, vx, vy, yt, rot)); CHECK(navAvoidActive());   // <= OBST_STOP_CM: HOLD langsung
  float ax = 5.0f, ay = 0; navSpeedFloor(ax, ay, 1, 0, lim, false); CHECK_NEAR(ax, 5.0f, 1e-6);      // AVOID: kecepatan geser dipertahankan
  navCancel(); navSetIr(false, false, false);
  const float none[3] = {-1, -1, -1}; const bool nok[3] = {false, false, false}; navSetWall(none, nok);
}
