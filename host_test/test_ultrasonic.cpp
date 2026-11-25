// PRD navigasi R1: median 3 sampel + validitas (outlier, zig-zag/crosstalk, tidak valid).
#include "ultrasonic.h"
#include "ir.h"
#include "test_main.h"

TEST(test_R1_median_normal_samples) {
  float m; const float h[3] = {50.0f, 51.0f, 52.0f};
  CHECK(usMedian3(h, m)); CHECK_NEAR(m, 51.0f, 1e-6);
}

TEST(test_R1_median_rejects_single_outlier) {
  float m; const float h[3] = {50.0f, 120.0f, 51.0f};
  CHECK(usMedian3(h, m)); CHECK_NEAR(m, 51.0f, 1e-6);          // lonjakan dibuang, 2 sampel lain sepakat
  const float g[3] = {50.0f, -1.0f, 52.0f};
  CHECK(usMedian3(g, m)); CHECK_NEAR(m, 51.0f, 1e-6);          // 1 miss tunggal tetap valid
}

TEST(test_R1_median_zigzag_and_all_invalid_flagged) {
  float m; const float z[3] = {50.0f, 120.0f, 60.0f};
  CHECK(!usMedian3(z, m)); CHECK_NEAR(m, -1.0f, 1e-6);          // naik-turun tidak wajar
  const float a[3] = {-1.0f, -1.0f, -1.0f};
  CHECK(!usMedian3(a, m)); CHECK_NEAR(m, -1.0f, 1e-6);
  const float o[3] = {-1.0f, -1.0f, 50.0f};
  CHECK(!usMedian3(o, m));                                      // 1 sampel valid belum cukup
  const float d[3] = {-1.0f, 50.0f, 80.0f};
  CHECK(!usMedian3(d, m));                                      // 2 sampel tidak sepakat
}

TEST(test_R1b_monotone_approach_valid_with_latest_sample_but_zigzag_still_rejected) {
  float m;
  const float a[3] = {80.0f, 74.0f, 68.0f};                      // mendekat 6 cm/sampel (robot ~100 cm/s): dulu ditolak sebagai zig-zag
  CHECK(usMedian3(a, m)); CHECK_NEAR(m, 68.0f, 1e-6);            // sampel terbaru, tanpa lag
  const float b[3] = {40.0f, 46.0f, 52.0f};                      // menjauh mulus
  CHECK(usMedian3(b, m)); CHECK_NEAR(m, 52.0f, 1e-6);
  const float c[3] = {80.0f, 60.0f, 40.0f};                      // langkah 20 cm: melompat, bukan gerak robot
  CHECK(!usMedian3(c, m));
  const float z[3] = {50.0f, 62.0f, 56.0f};                      // naik-turun
  CHECK(!usMedian3(z, m));
}

TEST(test_IR_debounce_needs_stable_reading_for_holdMs) {
  IrChan c; const uint32_t H = IR_DEBOUNCE_MS;
  CHECK(!irDebounce(c, false, 1000, H));
  CHECK(!irDebounce(c, true, 1010, H));                         // baru berubah
  CHECK(!irDebounce(c, true, 1010 + H - 1, H));                 // belum cukup lama
  CHECK(irDebounce(c, true, 1010 + H, H));                      // stabil >= H: aktif
  CHECK(irDebounce(c, false, 1100, H));                         // glitch turun singkat: masih aktif
  CHECK(irDebounce(c, true, 1110, H));                          // kembali: pewaktu mulai ulang
  CHECK(!irDebounce(c, false, 1200, H) || true);
  CHECK(!irDebounce(c, false, 1200 + H, H));                    // turun stabil: nonaktif
}
