#pragma once
#include "config.h"
void ultrasonicBegin();             // jalankan scheduler ping di Core 0
void ultrasonicGet(float d[3]);     // cm, -1 = tidak ada rintangan / tidak valid
void ultrasonicGetMedian(float d[3], bool ok[3]);   // median 3 sampel terakhir; ok=false -> d=-1
bool usMedian3(const float h[3], float& med);          // h: 3 sampel (<=0 = tidak valid); false bila <2 valid atau zig-zag
