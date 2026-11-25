#pragma once
#include "config.h"

enum : uint16_t {
  BTN_A = 1 << 0, BTN_B = 1 << 1, BTN_X = 1 << 2, BTN_Y = 1 << 3, BTN_LB = 1 << 4, BTN_RB = 1 << 5,
  BTN_START = 1 << 6, BTN_BACK = 1 << 7, BTN_UP = 1 << 8, BTN_DOWN = 1 << 9, BTN_LEFT = 1 << 10, BTN_RIGHT = 1 << 11
};
struct GamepadState {
  bool connected = false;
  float lx = 0, ly = 0, rx = 0, ry = 0;     // -1..1, ly/ry positif = atas
  uint16_t buttons = 0;
  char type[8] = "-";                       // "XInput" / "DInput"
};
typedef void (*GamepadCb)(const GamepadState&);   // dipanggil dari task USB (Core 0) tiap laporan / koneksi berubah

void gamepadBegin(GamepadCb cb);            // install USB Host + 2 task di Core 0
void gamepadGet(GamepadState& out);
