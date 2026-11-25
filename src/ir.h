#pragma once
#include <Arduino.h>

// IR digital 3 kanal: depan (dekat ultrasonik depan), kiri dan kanan (menghadap depan). Aktif = ada benda.
struct IrChan { bool stable = false, last = false; uint32_t since = 0; };
bool    irDebounce(IrChan& c, bool raw, uint32_t now, uint32_t holdMs);   // keluaran stabil: berubah hanya bila bacaan sama >= holdMs
void    irBegin();
void    irUpdate(uint32_t now);                                           // panggil tiap tick kontrol
bool    irFront();
bool    irLeft();
bool    irRight();
uint8_t irBits();                                                         // bit0 depan, bit1 kiri, bit2 kanan
