#pragma once
#include "config.h"

// Tes gerak: robot bergerak pendek ke satu arah, lalu arahnya dinilai dari
//  (1) odometri encoder (kinematika maju) dan (2) percepatan linear IMU.
// Semua logika jalan di task kontrol (Core 1). Dashboard hanya meminta lewat mcRequest().
enum McDir : uint8_t { MC_NONE = 0, MC_FWD = 1, MC_BACK = 2, MC_LEFT = 3, MC_RIGHT = 4 };

void     mcRequest(int dir);                    // dari Web (Core 0)
int      mcConsumeRequest();                    // task kontrol: ambil lalu kosongkan
bool     mcActive();                            // aman dipanggil dari core mana pun
bool     mcStart(int dir, float yawRad);
void     mcAbortStart(int dir, uint8_t why);    // tes tidak jadi dimulai
void     mcCancel(uint8_t why);                 // batalkan tes yang berjalan
// Panggil tiap tick selama mcActive(). mv = kecepatan linear roda terukur (cm/s).
// iaR/iaF = percepatan linear IMU (cm/s^2). Mengembalikan false saat tes selesai/batal.
bool     mcStep(uint32_t nowMs, float dt, bool full, const float mv[3], float yawRad,
                bool imuOk, float iaR, float iaF, float& vx, float& vy);
McResult mcResult();
