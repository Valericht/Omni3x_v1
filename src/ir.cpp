#include "ir.h"
#include "config.h"

bool irDebounce(IrChan& c, bool raw, uint32_t now, uint32_t holdMs) {
  if (raw != c.last) { c.last = raw; c.since = now; }
  if (raw != c.stable && now - c.since >= holdMs) c.stable = raw;
  return c.stable;
}

static IrChan ch[3];
static const int irPins[3] = {PIN_IR_FRONT, PIN_IR_LEFT, PIN_IR_RIGHT};

void irBegin() { for (int i = 0; i < 3; i++) pinMode(irPins[i], INPUT_PULLUP); }
void irUpdate(uint32_t now) {
  for (int i = 0; i < 3; i++) { bool lvl = digitalRead(irPins[i]) != 0; irDebounce(ch[i], IR_ACTIVE_LOW ? !lvl : lvl, now, IR_DEBOUNCE_MS); }
}
bool irFront() { return ch[0].stable; }
bool irLeft()  { return ch[1].stable; }
bool irRight() { return ch[2].stable; }
uint8_t irBits() { return (ch[0].stable ? 1 : 0) | (ch[1].stable ? 2 : 0) | (ch[2].stable ? 4 : 0); }
