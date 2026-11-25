// A.5.6: logika portable imuassist.cpp (F1 normalizeAngle, F3 terrain, F4 koreksi yaw, F6 stuck, F7 detectWheelSlip, F8 encoder trust).
// imuassist.cpp disertakan LANGSUNG; imu.* diganti stub di host_stubs.cpp.
#include "Arduino.h"
#include "imuassist.cpp"
#include "host_stubs.h"
#include "test_main.h"

TEST(test_A_5_6_normalize_angle_wraps_to_pm180) {
  CHECK_NEAR(normalizeAngle(190), -170, 1e-4); CHECK_NEAR(normalizeAngle(-190), 170, 1e-4);
  CHECK_NEAR(normalizeAngle(360), 0, 1e-4);   CHECK_NEAR(normalizeAngle(721), 1, 1e-3);
  CHECK_NEAR(normalizeAngle(180), 180, 1e-4); CHECK_NEAR(normalizeAngle(-180), -180, 1e-4);
}

TEST(test_A_5_6_wheel_slip_needs_yaw_gap_gyro_and_500ms) {
  assistReset();
  CHECK(!detectWheelSlip(10, 0, 5, 0));                          // kondisi mulai: belum 500 ms
  CHECK(!detectWheelSlip(10, 0, 5, 499));
  CHECK(detectWheelSlip(10, 0, 5, 500));                         // tepat SLIP_TIME_MS
  CHECK(detectWheelSlip(10, 0, 5, 900));                         // tetap slip selama kondisi bertahan
  CHECK(!detectWheelSlip(2, 0, 5, 1000));                        // selisih kecil: reset
  CHECK(!detectWheelSlip(-10, 0, 5, 1100));                      // selisih negatif juga dihitung, timer mulai ulang
  CHECK(detectWheelSlip(-10, 0, 5, 1600));
}

TEST(test_A_5_6_wheel_slip_interrupted_condition_restarts_timer) {
  assistReset();
  CHECK(!detectWheelSlip(10, 0, 5, 0));
  CHECK(!detectWheelSlip(1, 0, 5, 300));                         // putus
  CHECK(!detectWheelSlip(10, 0, 5, 400));                        // mulai lagi di 400
  CHECK(!detectWheelSlip(10, 0, 5, 800));                        // baru 400 ms
  CHECK(detectWheelSlip(10, 0, 5, 900));
}

TEST(test_A_5_6_wheel_slip_ignores_small_gyro_and_wraps_yaw_gap) {
  assistReset();
  CHECK(!detectWheelSlip(20, 0, SLIP_GYRO_MIN - 0.5f, 0)); CHECK(!detectWheelSlip(20, 0, SLIP_GYRO_MIN - 0.5f, 5000));   // gyro lemah: bukan slip
  assistReset();
  CHECK(!detectWheelSlip(359, 1, 5, 0));   CHECK(!detectWheelSlip(359, 1, 5, 2000));                                      // selisih 2 derajat setelah dibungkus
  CHECK(!detectWheelSlip(179, -179, 5, 0)); CHECK(!detectWheelSlip(179, -179, 5, 2000));                                 // 358 derajat = -2 derajat
}

TEST(test_A_5_6_encoder_trust_decays_on_slip_and_recovers) {
  assistReset();
  float t = updateEncoderTrust(false); CHECK_NEAR(t, ENCODER_TRUST_INIT, 1e-6);   // after reset trust is at INIT (cannot exceed 1)
  t = updateEncoderTrust(true);  CHECK_NEAR(t, 1.0 - SLIP_TRUST_DECAY, 1e-5);
  for (int i = 0; i < 10; i++) t = updateEncoderTrust(true);
  CHECK_NEAR(t, 0.0, 1e-6);                                      // tidak boleh negatif
  t = updateEncoderTrust(false); CHECK_NEAR(t, SLIP_TRUST_RECOVER, 1e-6);
  for (int i = 0; i < 100; i++) t = updateEncoderTrust(false);
  CHECK_NEAR(t, 1.0, 1e-6);                                      // tidak boleh > 1
}

TEST(test_A_5_6_terrain_thresholds_and_yaw_gain) {
  CHECK(detectTerrain(PITCH_SLOPE_THRESHOLD, 0) == TERRAIN_SLOPE);
  CHECK(detectTerrain(-PITCH_SLOPE_THRESHOLD, 0) == TERRAIN_SLOPE);
  CHECK(detectTerrain(0, ROLL_TILT_THRESHOLD) == TERRAIN_TILT);
  terrainReset();                                                // L2: histeresis menyimpan status terakhir
  CHECK(detectTerrain(PITCH_SLOPE_THRESHOLD - 0.1f, ROLL_TILT_THRESHOLD - 0.1f) == TERRAIN_FLAT);
  CHECK(detectTerrain(10, 10) == TERRAIN_SLOPE);                 // pitch didahulukan
  CHECK_NEAR(computeYawCorrection(10, 0, TERRAIN_FLAT, NAN), 10 * KP_YAW_FLAT, 1e-4);
  CHECK_NEAR(computeYawCorrection(10, 0, TERRAIN_SLOPE, NAN), 10 * KP_YAW_SLOPE, 1e-4);
  CHECK_NEAR(computeYawCorrection(10, 0, TERRAIN_SLOPE, 3.0f), 30, 1e-4);   // kpDynamic menimpa
  CHECK_NEAR(computeYawCorrection(179, -179, TERRAIN_FLAT, NAN), -2 * KP_YAW_FLAT, 1e-4);   // lewat jalur terpendek
}

TEST(test_A_5_6_stuck_requires_yaw_error_no_rotation_no_motion_for_1s) {
  assistResetStuck();
  CHECK(!detectStuck(STUCK_YAW_ERR + 1, 0, 0, 0));
  CHECK(!detectStuck(STUCK_YAW_ERR + 1, 0, 0, STUCK_TIME_MS - 1));
  CHECK(detectStuck(STUCK_YAW_ERR + 1, 0, 0, STUCK_TIME_MS));
  CHECK(!detectStuck(STUCK_YAW_ERR + 1, STUCK_GYRO_MIN + 1, 0, STUCK_TIME_MS + 10));   // berputar: bukan stuck
  assistResetStuck();
}

TEST(test_A_5_6_read_imu_filtered_uses_stub_imu_and_first_sample_passthrough) {
  assistReset();
  g_hostImu = HostImu(); g_hostImu.yawRad = (float)M_PI / 2; g_hostImu.pitchDeg = 6; g_hostImu.rollDeg = 1; g_hostImu.rateDps = 12;
  ImuFiltered f = readImuFiltered();                             // sampel pertama: tanpa filter
  CHECK_NEAR(f.yaw, 90, 1e-2); CHECK_NEAR(f.pitch, 6, 1e-4); CHECK_NEAR(f.gyroZ, 12, 1e-4);
  g_hostImu.pitchDeg = 0; f = readImuFiltered();                 // low-pass: bergerak IMU_FILTER_ALPHA ke arah baru
  CHECK_NEAR(f.pitch, 6 - IMU_FILTER_ALPHA * 6, 1e-4);
  g_hostImu.ok = false; f = readImuFiltered();                   // IMU mati: filter di-reset
  g_hostImu = HostImu(); assistReset();
}

// ---------------- Lapangan v2: L1 tanda pitch, L2 histeresis, L3 pengali ----------------
static_assert(IMU_PITCH_SIGN == -1.0f, "L1: default dari bukti lapangan (pitch mentah negatif saat hidung naik)");

TEST(test_L1_slope_factor_uses_nose_up_positive_convention) {
  CHECK_NEAR(slopeSpeedFactor(TERRAIN_SLOPE, 13.5f, 40.0f), SLOPE_SPEED_FACTOR, 1e-6);     // naik: hidung naik (+), maju (vy > 0)
  CHECK_NEAR(slopeSpeedFactor(TERRAIN_SLOPE, 13.5f, -40.0f), SLOPE_DESCENT_FACTOR, 1e-6);  // turun: hidung naik (+), mundur (vy < 0)
  CHECK_NEAR(slopeSpeedFactor(TERRAIN_SLOPE, -13.5f, 40.0f), SLOPE_DESCENT_FACTOR, 1e-6);  // hidung turun (-), maju = menuruni lereng
  CHECK_NEAR(slopeSpeedFactor(TERRAIN_FLAT, 13.5f, -40.0f), 1.0f, 1e-6);
  CHECK_NEAR(slopeSpeedFactor(TERRAIN_TILT, 0.0f, -40.0f), SLOPE_SPEED_FACTOR, 1e-6);
  CHECK_NEAR(SLOPE_DESCENT_FACTOR, 0.5f, 1e-6); CHECK_NEAR(SLOPE_SPEED_FACTOR, 0.6f, 1e-6);
}

TEST(test_L2_terrain_hysteresis_does_not_flicker_near_threshold) {
  terrainReset();
  const float seq[8] = {4.8f, 5.2f, 4.8f, 5.2f, 4.8f, 5.2f, 4.8f, 5.2f};
  int flips = 0; Terrain prev = TERRAIN_FLAT;
  for (int i = 0; i < 8; i++) { Terrain t = detectTerrain(seq[i], 0); if (t != prev) flips++; prev = t; }
  CHECK_MSG(flips == 1, "FLAT -> SLOPE sekali, tidak berkedip");
  CHECK(detectTerrain(3.2f, 0) == TERRAIN_SLOPE);                                    // masih di atas ambang keluar
  CHECK(detectTerrain(PITCH_SLOPE_EXIT_DEG - 0.1f, 0) == TERRAIN_FLAT);             // keluar
  CHECK(detectTerrain(4.9f, 0) == TERRAIN_FLAT);                                     // masuk lagi perlu >= ambang masuk
  CHECK(detectTerrain(-5.0f, 0) == TERRAIN_SLOPE);
  terrainReset();
  CHECK(detectTerrain(0, 5.2f) == TERRAIN_TILT); CHECK(detectTerrain(0, 4.0f) == TERRAIN_TILT);
  CHECK(detectTerrain(0, ROLL_TILT_EXIT_DEG - 0.1f) == TERRAIN_FLAT);
  terrainReset();
}

TEST(test_L3_translation_scale_is_min_not_product) {
  CHECK_NEAR(translationScale(0.35f, 0.5f), 0.35f, 1e-6);      // bukan 0.175
  CHECK_NEAR(translationScale(1.0f, 0.6f), 0.6f, 1e-6);
  CHECK_NEAR(translationScale(0.35f, 1.0f), 0.35f, 1e-6);
  CHECK_NEAR(rpmToCms(100.0f) * 0.35f * 0.5f * 0.3f, rpmToCms(100.0f) * 0.0525f, 1e-4);   // rantai lama: 100 RPM jadi ~5 RPM
}
