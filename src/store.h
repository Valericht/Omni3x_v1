#pragma once
#include "config.h"
// Penyimpanan flash (NVS) satu pintu: tulis lalu BACA ULANG untuk memastikan benar-benar tersimpan.
// what: 1 waypoint, 2 PID, 3 posisi terakhir, 4 rute, 5 kecepatan
void   storeBegin();                                                                  // panggil sekali di setup(), sebelum task lain
bool   storeWrite(const char* ns, const char* key, const void* data, size_t len, uint8_t what);
size_t storeRead(const char* ns, const char* key, void* data, size_t len);            // len bila ada DAN ukurannya sama, selain itu 0
void   storeStatus(uint8_t& status, uint8_t& what, uint16_t& count);                  // hasil simpan terakhir (1 OK, 2 GAGAL)
