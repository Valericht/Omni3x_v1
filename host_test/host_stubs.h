#pragma once
// Kendali jam palsu, penyimpanan palsu, dan IMU palsu untuk tes host.
#include <cstddef>
#include <cstdint>

void     host_set_millis(uint32_t ms);
void     host_advance_ms(uint32_t ms);
void     host_serial_verbose(bool on);

// penyimpanan (pengganti store.cpp / NVS): ns/key -> byte
void     host_store_clear();
void     host_store_put(const char* ns, const char* key, const void* data, size_t len);   // fixture, tidak dihitung sebagai tulis
bool     host_store_has(const char* ns, const char* key);
unsigned host_store_writes();                                                              // jumlah panggilan storeWrite sejak clear
void     host_store_fail_writes(bool fail);

// IMU palsu (dibaca imuassist.cpp)
struct HostImu { bool ok = true; float yawRad = 0, pitchDeg = 0, rollDeg = 0, rateDps = 0; };
extern HostImu g_hostImu;
