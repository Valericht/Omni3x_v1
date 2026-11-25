#include "ultrasonic.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// 1 pin TRIG bersama, 3 pin ECHO diukur paralel lewat interrupt (lebar pulsa).
static volatile uint32_t tRise[3], widthUs[3];
static volatile bool got[3];
static float dist[3] = {-1, -1, -1};
static float medD[3] = {-1, -1, -1};
static bool  medOk[3];
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define ECHO_ISR(N) static void IRAM_ATTR echoIsr##N() { \
  uint32_t t = (uint32_t)esp_timer_get_time(); \
  if (gpio_get_level((gpio_num_t)PIN_US_ECHO[N])) tRise[N] = t; else { widthUs[N] = t - tRise[N]; got[N] = true; } }
ECHO_ISR(0) ECHO_ISR(1) ECHO_ISR(2)
static void (*const echoIsr[3])() = {echoIsr0, echoIsr1, echoIsr2};

bool usMedian3(const float h[3], float& med) {
  float v[3]; int n = 0;
  for (int i = 0; i < 3; i++) if (h[i] > 0) v[n++] = h[i];
  med = -1;
  if (n < 2) return false;
  if (n == 2) { if (fabsf(v[0] - v[1]) > US_AGREE_CM) return false; med = 0.5f * (v[0] + v[1]); return true; }
  float lo = fminf(v[0], fminf(v[1], v[2])), hi = fmaxf(v[0], fmaxf(v[1], v[2])), m = v[0] + v[1] + v[2] - lo - hi;
  if (m - lo > US_AGREE_CM && hi - m > US_AGREE_CM) {
    // R1b: mendekati/menjauhi benda saat robot jalan: 3 sampel monoton dengan langkah wajar bukan zig-zag. Ambil sampel TERBARU (tanpa lag median).
    bool up = v[0] < v[1] && v[1] < v[2], dn = v[0] > v[1] && v[1] > v[2];
    if ((up || dn) && fabsf(v[1] - v[0]) <= US_TREND_STEP_CM && fabsf(v[2] - v[1]) <= US_TREND_STEP_CM) { med = v[2]; return true; }
    return false;
  }
  med = m; return true;
}

static void usTask(void*) {
  int miss[3] = {0};
  float hist[3][3] = {{-1, -1, -1}, {-1, -1, -1}, {-1, -1, -1}};
  for (;;) {
    for (int i = 0; i < 3; i++) got[i] = false;
    digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_US_TRIG, LOW);
    vTaskDelay(pdMS_TO_TICKS(35));                       // 400 cm ~ 23 ms
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < 3; i++) {
      float d = got[i] ? widthUs[i] / 58.0f : -1.0f;
      if (d < 2.0f || d > 400.0f) d = -1.0f;
      if (d < 0) { if (++miss[i] >= 2) dist[i] = -1; }  // buang 1 miss tunggal (anti-kedip)
      else { miss[i] = 0; dist[i] = d; }
      hist[i][0] = hist[i][1]; hist[i][1] = hist[i][2]; hist[i][2] = d;
      medOk[i] = usMedian3(hist[i], medD[i]);
    }
    portEXIT_CRITICAL(&mux);
    vTaskDelay(pdMS_TO_TICKS(25));                       // siklus ~60 ms
  }
}

void ultrasonicBegin() {
  pinMode(PIN_US_TRIG, OUTPUT); digitalWrite(PIN_US_TRIG, LOW);
  for (int i = 0; i < 3; i++) { pinMode(PIN_US_ECHO[i], INPUT); attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO[i]), echoIsr[i], CHANGE); }
  xTaskCreatePinnedToCore(usTask, "ultra", 3072, nullptr, 2, nullptr, 0);
}

void ultrasonicGet(float d[3]) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 3; i++) d[i] = dist[i];
  portEXIT_CRITICAL(&mux);
}

void ultrasonicGetMedian(float d[3], bool ok[3]) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 3; i++) { d[i] = medD[i]; ok[i] = medOk[i]; }
  portEXIT_CRITICAL(&mux);
}
