#pragma once
#include "config.h"
// Pengaturan kecepatan (RPM roda) yang bisa diubah dari dashboard dan tersimpan di flash.
struct Settings {
  float man   = SPD_MAN_DEF;     // manual normal
  float turbo = SPD_TURBO_DEF;   // manual turbo (tahan R1)
  float nav   = SPD_NAV_DEF;     // navigasi otomatis
  float loc   = SPD_LOC_DEF;     // jalan dashboard (cari / rekam waypoint)
  bool  safe  = SAFETY_DEFAULT_ON;   // pengaman aktif. TIDAK disimpan: nilai awal saat menyala = SAFETY_DEFAULT_ON (config.h)
};
void     settingsBegin();
Settings settingsGet();
void     settingsSet(float man, float turbo, float nav, float loc);   // dibatasi lalu disimpan ke flash
void     settingsSetSafety(bool on);
