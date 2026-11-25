// Stub host: jam, Serial, store, imu, telemetri. Tidak ada logika firmware di sini.
#include "Arduino.h"
#include "config.h"
#include "store.h"
#include "imu.h"
#include "host_stubs.h"
#include <map>
#include <string>
#include <vector>

// ---- jam ----
static uint32_t g_ms = 0;
uint32_t millis() { return g_ms; }
uint32_t micros() { return g_ms * 1000u; }
void host_set_millis(uint32_t ms) { g_ms = ms; }
void host_advance_ms(uint32_t ms) { g_ms += ms; }

// ---- Serial ----
HostSerial Serial;
static bool g_verbose = false;
void host_serial_verbose(bool on) { g_verbose = on; }
int HostSerial::printf(const char* fmt, ...) {
  if (!g_verbose) return 0;
  va_list ap; va_start(ap, fmt); int n = vprintf(fmt, ap); va_end(ap); return n;
}
void HostSerial::println(const char* s) { if (g_verbose) puts(s); }
void HostSerial::print(const char* s)   { if (g_verbose) fputs(s, stdout); }

// ---- store (semantik sama dengan store.cpp: baca hanya bila ukuran sama) ----
static std::map<std::string, std::vector<uint8_t>> g_store;
static unsigned g_writes = 0; static bool g_failWrites = false;
static uint8_t stStatus = 0, stWhat = 0; static uint16_t stCount = 0;
static std::string K(const char* ns, const char* key) { return std::string(ns) + "/" + key; }

void host_store_clear() { g_store.clear(); g_writes = 0; g_failWrites = false; stStatus = stWhat = 0; stCount = 0; }
void host_store_put(const char* ns, const char* key, const void* d, size_t n) {
  g_store[K(ns, key)] = std::vector<uint8_t>((const uint8_t*)d, (const uint8_t*)d + n);
}
bool host_store_has(const char* ns, const char* key) { return g_store.count(K(ns, key)) != 0; }
unsigned host_store_writes() { return g_writes; }
void host_store_fail_writes(bool f) { g_failWrites = f; }

void storeBegin() {}
bool storeWrite(const char* ns, const char* key, const void* data, size_t len, uint8_t what) {
  g_writes++;
  stWhat = what; stCount++;
  if (g_failWrites) { stStatus = 2; return false; }
  host_store_put(ns, key, data, len);
  stStatus = 1; return true;
}
size_t storeRead(const char* ns, const char* key, void* data, size_t len) {
  auto it = g_store.find(K(ns, key));
  if (it == g_store.end() || it->second.size() != len) return 0;
  memcpy(data, it->second.data(), len); return len;
}
void storeStatus(uint8_t& s, uint8_t& w, uint16_t& c) { s = stStatus; w = stWhat; c = stCount; }

// ---- IMU ----
HostImu g_hostImu;
bool  imuOk()       { return g_hostImu.ok; }
float imuYawRad()   { return g_hostImu.yawRad; }
float imuPitchDeg() { return g_hostImu.pitchDeg; }
float imuRollDeg()  { return g_hostImu.rollDeg; }
float imuRateDps()  { return g_hostImu.rateDps; }

// ---- telemetri (dipakai poseTask di navigation.cpp; task tidak dijalankan di host) ----
void      telemetrySet(const Telemetry&) {}
Telemetry telemetryGet() { return Telemetry(); }
volatile uint32_t g_hbWifiMs = 0, g_hbSerialMs = 0;
