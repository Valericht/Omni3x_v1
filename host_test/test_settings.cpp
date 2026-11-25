// A.5.6: settings.cpp (kecepatan disimpan ke store; status pengaman TIDAK disimpan, lihat PRD D.4 no. 9).
#include "Arduino.h"
#include "settings.cpp"
#include "host_stubs.h"
#include "test_main.h"

static void reboot() { S = Settings(); settingsBegin(); }       // meniru boot: nilai awal struct lalu muat dari store

TEST(test_A_5_6_settings_defaults_and_safety_default_off) {
  host_store_clear(); reboot();
  Settings s = settingsGet();
  CHECK_NEAR(s.man, SPD_MAN_DEF, 1e-6); CHECK_NEAR(s.turbo, SPD_TURBO_DEF, 1e-6);
  CHECK_NEAR(s.nav, SPD_NAV_DEF, 1e-6); CHECK_NEAR(s.loc, SPD_LOC_DEF, 1e-6);
  CHECK(s.safe == SAFETY_DEFAULT_ON); CHECK(!SAFETY_DEFAULT_ON);   // config.h: pengaman mati sementara (PRD A.3)
}

TEST(test_A_5_6_settings_set_clamps_and_persists_across_reboot) {
  host_store_clear(); reboot();
  settingsSet(10, 9999, 123, 77);                                // 10 -> SPD_MIN_RPM, 9999 -> MAX_WHEEL_RPM
  Settings s = settingsGet();
  CHECK_NEAR(s.man, SPD_MIN_RPM, 1e-6); CHECK_NEAR(s.turbo, MAX_WHEEL_RPM, 1e-6); CHECK_NEAR(s.nav, 123, 1e-6); CHECK_NEAR(s.loc, 77, 1e-6);
  CHECK(host_store_has("cfg", "spd"));
  reboot(); s = settingsGet();                                   // dimuat dari store
  CHECK_NEAR(s.man, SPD_MIN_RPM, 1e-6); CHECK_NEAR(s.turbo, MAX_WHEEL_RPM, 1e-6); CHECK_NEAR(s.nav, 123, 1e-6); CHECK_NEAR(s.loc, 77, 1e-6);
}

TEST(test_A_5_6_settings_nan_keeps_previous_value) {
  host_store_clear(); reboot();
  settingsSet(150, 400, 180, 120);
  settingsSet(NAN, NAN, 200, NAN);
  Settings s = settingsGet();
  CHECK_NEAR(s.man, 150, 1e-6); CHECK_NEAR(s.turbo, 400, 1e-6); CHECK_NEAR(s.nav, 200, 1e-6); CHECK_NEAR(s.loc, 120, 1e-6);
}

TEST(test_A_5_6_settings_bad_blob_is_ignored_or_sanitised) {
  host_store_clear();
  Blob bad = {111, 222, 333, 144, 0xDEADBEEF};                   // magic salah
  host_store_put("cfg", "spd", &bad, sizeof bad); reboot();
  CHECK_NEAR(settingsGet().man, SPD_MAN_DEF, 1e-6);              // diabaikan
  Blob nanb = {NAN, 1e9f, 5.0f, 144, MAGIC};                     // magic benar, isi rusak
  host_store_put("cfg", "spd", &nanb, sizeof nanb); reboot();
  Settings s = settingsGet();
  CHECK_NEAR(s.man, SPD_MAN_DEF, 1e-6); CHECK_NEAR(s.turbo, MAX_WHEEL_RPM, 1e-6); CHECK_NEAR(s.nav, SPD_MIN_RPM, 1e-6); CHECK_NEAR(s.loc, 144, 1e-6);
  host_store_put("cfg", "spd", "x", 1); reboot();                // ukuran salah: store menolak, pakai default
  CHECK_NEAR(settingsGet().turbo, SPD_TURBO_DEF, 1e-6);
}

TEST(test_A_5_6_settings_safety_flag_is_not_persisted) {
  // PRD A.3 / D.4 no. 9: status pengaman reset ke SAFETY_DEFAULT_ON tiap boot. Tes ini MENDOKUMENTASIKAN perilaku sekarang.
  // Bila diputuskan disimpan ke NVS, tes ini harus diubah bersama shim settings di sim.
  host_store_clear(); reboot();
  unsigned w0 = host_store_writes();
  settingsSetSafety(true);
  CHECK(settingsGet().safe);
  CHECK_MSG(host_store_writes() == w0, "settingsSetSafety tidak boleh menulis ke store (perilaku sekarang)");
  reboot();
  CHECK(settingsGet().safe == SAFETY_DEFAULT_ON);
}
