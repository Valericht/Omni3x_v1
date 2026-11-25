#pragma once
// Stub Arduino/FreeRTOS minimal untuk tes host firmware (A.5.6). BUKAN shim SIL penuh (itu Fase 2, B.6.4).
// Hanya yang dipakai navigation.cpp, imuassist.cpp, settings.cpp, dan config.h.
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define IRAM_ATTR
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// ---- waktu (jam palsu, dikendalikan tes: host_stubs.h) ----
uint32_t millis();
uint32_t micros();

// ---- Serial: diam secara default ----
struct HostSerial {
  int  printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void println(const char* s);
  void print(const char* s);
};
extern HostSerial Serial;

// ---- FreeRTOS: tidak ada task nyata ----
typedef int          portMUX_TYPE;
typedef void*        TaskHandle_t;
typedef int          BaseType_t;
typedef unsigned int UBaseType_t;
typedef void*        SemaphoreHandle_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m)  ((void)(m))
#define pdMS_TO_TICKS(ms) (ms)
inline void vTaskDelay(uint32_t) {}
inline BaseType_t xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*, BaseType_t) { return 1; }
