#include "settings.h"
#include "store.h"

struct Blob { float man, turbo, nav, loc; uint32_t magic; };
static constexpr uint32_t MAGIC = 0x53504431;
static Settings S;

static float clampRpm(float v, float def) { return isnan(v) ? def : constrain(v, SPD_MIN_RPM, MAX_WHEEL_RPM); }

void settingsBegin() {
  Blob b;
  if (storeRead("cfg", "spd", &b, sizeof b) == sizeof b && b.magic == MAGIC) {
    S.man = clampRpm(b.man, SPD_MAN_DEF); S.turbo = clampRpm(b.turbo, SPD_TURBO_DEF);
    S.nav = clampRpm(b.nav, SPD_NAV_DEF); S.loc = clampRpm(b.loc, SPD_LOC_DEF);
    Serial.printf("[CFG] kecepatan dimuat dari flash: manual %.0f, turbo %.0f, nav %.0f, jalan dashboard %.0f RPM\n", S.man, S.turbo, S.nav, S.loc);
  } else Serial.println("[CFG] kecepatan: memakai default");
}

Settings settingsGet() { return S; }

void settingsSet(float man, float turbo, float nav, float loc) {
  S.man = clampRpm(man, S.man); S.turbo = clampRpm(turbo, S.turbo); S.nav = clampRpm(nav, S.nav); S.loc = clampRpm(loc, S.loc);
  Blob b = {S.man, S.turbo, S.nav, S.loc, MAGIC};
  storeWrite("cfg", "spd", &b, sizeof b, 5);
}

void settingsSetSafety(bool on) { S.safe = on; Serial.printf("[SAFE] pengaman %s\n", on ? "AKTIF" : "MATI"); }
