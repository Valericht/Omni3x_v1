// A.5.1, A.5.2, A.5.3: konstanta mekanik, kinematika kinInverse/kinForward (disalin dari motors.cpp), sudut sensor.
#include "Arduino.h"
#include "config.h"
#include "motors.h"
#include "test_main.h"

static float maxAbs(const float v[3]) { return fmaxf(fabsf(v[0]), fmaxf(fabsf(v[1]), fabsf(v[2]))); }

TEST(test_A_5_1_wheel_radius_is_4_88_cm) {
  CHECK_NEAR(WHEEL_R_CM, 4.88, 1e-6);                                    // keputusan pengguna (PRD A.4): terkalibrasi 482 cm nyata vs 296,2 cm odometri
  CHECK_NEAR(rpmToCms(60.0f), 2.0 * M_PI * 4.88, 1e-4);                  // 1 putaran/detik
  CHECK_NEAR(MAX_LIN_CMS, 500.0 * 2.0 * M_PI * 4.88 / 60.0, 1e-3);
}

TEST(test_A_5_2_rotation_arm_is_ROBOT_L_CM_12_not_22_5) {
  // ANTI-TUJUAN (A.7, D.6): firmware memakai 12 cm untuk hitungan gerak, sim memakai 22,5 cm untuk posisi roda.
  // Beda ini SENGAJA terlihat. Bila keputusan B diambil (ROBOT_L_CM jadi 22,5), tes ini gagal bersama
  // omni_sim test_wheel_dist_differs_from_firmware_L dan test_T3_rotation_differs_from_firmware_by_22_5_over_12: perbarui ketiganya sekaligus.
  CHECK_NEAR(ROBOT_L_CM, 12.0, 1e-6);
  float v[3]; kinInverse(0, 0, 1.0f, v);                                 // putar murni 1 rad/s CCW: tiap roda = ROBOT_L_CM * wz * KIN_W_SIGN
  for (int i = 0; i < 3; i++) CHECK_NEAR(v[i], ROBOT_L_CM * KIN_W_SIGN, 1e-4);
}

TEST(test_A_5_2_kin_inverse_forward_and_strafe_patterns) {
  float v[3];
  kinInverse(0, 30, 0, v);                                               // maju: roda 0 (depan) diam, roda 1 dan 2 berlawanan tanda
  CHECK_NEAR(v[0], 0.0, 1e-3);
  CHECK_NEAR(v[1], -KIN_Y_SIGN * 30.0 * cos(30.0 * M_PI / 180.0), 1e-3);   // KIN_Y_SIGN = -1 -> +25,98
  CHECK_NEAR(v[2], KIN_Y_SIGN * 30.0 * cos(30.0 * M_PI / 180.0), 1e-3);
  CHECK(v[1] > 0 && v[2] < 0);                                           // pola golden "maju" untuk KIN_Y_SIGN = -1
  kinInverse(30, 0, 0, v);                                               // kanan: [-30, 15, 15] (tidak dipengaruhi KIN_Y_SIGN)
  CHECK_NEAR(v[0], -30.0, 1e-3); CHECK_NEAR(v[1], 15.0, 1e-3); CHECK_NEAR(v[2], 15.0, 1e-3);
  float m[3]; kinInverse(0, -30, 0, m);                                  // mundur = kebalikan maju
  for (int i = 0; i < 3; i++) CHECK_NEAR(m[i], -(i == 0 ? 0.0f : (i == 1 ? 25.9808f : -25.9808f)), 1e-3);
}

TEST(test_A_5_2_kin_roundtrip_all_directions) {
  const float cases[][3] = {{0, 30, 0}, {0, -30, 0}, {30, 0, 0}, {-30, 0, 0}, {0, 0, 1}, {0, 0, -1},
                            {20, 20, 0}, {15, -10, 0.5f}, {-40, 25, -0.8f}};
  for (auto& c : cases) {
    float v[3], x, y, w; kinInverse(c[0], c[1], c[2], v); kinForward(v, x, y, w);
    CHECK_NEAR(x, c[0], 1e-3); CHECK_NEAR(y, c[1], 1e-3); CHECK_NEAR(w, c[2], 1e-4);
  }
}

TEST(test_A_5_2_kin_saturation_keeps_ratio_and_limit) {
  float big[3], ref[3];
  kinInverse(200, 200, 3.0f, big);                                       // memicu penskalaan MAX_LIN_CMS
  CHECK_NEAR(maxAbs(big), MAX_LIN_CMS, 1e-2);
  kinInverse(2, 2, 0.03f, ref);                                          // arah sama, tidak jenuh
  for (int i = 0; i < 3; i++) CHECK_NEAR(big[i] / maxAbs(big), ref[i] / maxAbs(ref), 1e-4);
}

TEST(test_A_5_3_sensor_angles_front_left_right) {
  // config.h: 0 depan, 1 kiri, 2 kanan; sudut dari +x (kanan), CCW. Pin echo tidak berubah.
  CHECK_NEAR(SENSOR_ANG_DEG[0], 90.0, 1e-6);
  CHECK_NEAR(SENSOR_ANG_DEG[1], 180.0, 1e-6);
  CHECK_NEAR(SENSOR_ANG_DEG[2], 0.0, 1e-6);
  static_assert(sizeof(SENSOR_ANG_DEG) / sizeof(float) == 3, "jumlah sensor harus 3 (A.5.3)");
  CHECK(PIN_US_ECHO[0] == 15 && PIN_US_ECHO[1] == 16 && PIN_US_ECHO[2] == 47);
  CHECK(PIN_US_TRIG == 21);
}
