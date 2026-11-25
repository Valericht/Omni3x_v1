#include "store.h"
#include <Preferences.h>
#include <nvs.h>

static SemaphoreHandle_t mtx = nullptr;
static volatile uint8_t  stStatus = 0, stWhat = 0;
static volatile uint16_t stCount = 0;

void storeBegin() {
  mtx = xSemaphoreCreateMutex();
  nvs_stats_t st;
  if (nvs_get_stats(NULL, &st) == ESP_OK)
    Serial.printf("[NVS] siap: %u dari %u entri terpakai, %u namespace\n", (unsigned)st.used_entries, (unsigned)st.total_entries, (unsigned)st.namespace_count);
  else
    Serial.println("[NVS] PERINGATAN: partisi NVS tidak terbaca. Data tidak akan tersimpan.");
}

bool storeWrite(const char* ns, const char* key, const void* data, size_t len, uint8_t what) {
  bool ok = false;
  if (mtx) xSemaphoreTake(mtx, portMAX_DELAY);
  Preferences p;
  if (p.begin(ns, false)) {
    if (p.putBytes(key, data, len) == len) {
      uint8_t* buf = (uint8_t*)malloc(len);
      if (buf) { ok = (p.getBytes(key, buf, len) == len) && memcmp(buf, data, len) == 0; free(buf); }
    }
    p.end();
  }
  if (mtx) xSemaphoreGive(mtx);
  Serial.printf("[NVS] tulis %s/%s (%u B): %s\n", ns, key, (unsigned)len, ok ? "OK" : "GAGAL");
  stWhat = what; stStatus = ok ? 1 : 2; stCount = stCount + 1;
  return ok;
}

size_t storeRead(const char* ns, const char* key, void* data, size_t len) {
  size_t got = 0;
  if (mtx) xSemaphoreTake(mtx, portMAX_DELAY);
  Preferences p;
  if (p.begin(ns, true)) {                       // read-only: tidak membuat namespace baru
    if (p.getBytesLength(key) == len) got = (p.getBytes(key, data, len) == len) ? len : 0;
    p.end();
  }
  if (mtx) xSemaphoreGive(mtx);
  return got;
}

void storeStatus(uint8_t& status, uint8_t& what, uint16_t& count) { status = stStatus; what = stWhat; count = stCount; }
