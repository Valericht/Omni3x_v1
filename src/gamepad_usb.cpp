#include "gamepad_usb.h"
#include <usb/usb_host.h>
#include <string.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static GamepadState gp;
static GamepadCb cb;

static usb_host_client_handle_t client;
static usb_device_handle_t dev;
static usb_transfer_t* inXfer;
static volatile bool pendingOpen, goneFlag, inflight, ctrlDone;
static volatile uint8_t newAddr;
static uint8_t ifNum;
static bool isXinput;

// Peta tombol HID generik (usage 1..12) -> tombol logis. Sesuaikan bila gamepad D-Input Anda berbeda.
static const uint16_t HID_BTN_MAP[12] = {BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, 0, 0, BTN_BACK, BTN_START, 0, 0};

// ---------------- Model HID report descriptor ----------------
struct HidField { uint8_t rid; uint16_t off; uint8_t bits; int32_t lmin, lmax; };
static HidField fBtn[16]; static uint8_t btnUsage[16]; static int nBtn;
static HidField fAxis[6]; static bool hasAxis[6];
static HidField fHat; static bool hasHat, hasRid;
static uint8_t descBuf[512];

static void parseHidDesc(const uint8_t* d, int len) {
  nBtn = 0; hasHat = false; hasRid = false; memset(hasAxis, 0, sizeof(hasAxis));
  uint16_t page = 0, usages[16]; int nU = 0; int32_t lmin = 0, lmax = 0, uMin = -1;
  uint8_t rsize = 0, rcount = 0, rid = 0; uint16_t bitPos = 0;
  for (int i = 0; i < len;) {
    uint8_t b = d[i];
    if (b == 0xFE) { i += 3 + d[i + 1]; continue; }
    int sz = b & 3; if (sz == 3) sz = 4;
    int type = (b >> 2) & 3, tag = b >> 4;
    if (i + 1 + sz > len) break;
    uint32_t u = 0; for (int k = 0; k < sz; k++) u |= (uint32_t)d[i + 1 + k] << (8 * k);
    int32_t s = sz == 0 ? 0 : sz == 1 ? (int8_t)u : sz == 2 ? (int16_t)u : (int32_t)u;
    i += 1 + sz;
    if (type == 1) {
      switch (tag) { case 0: page = u; break; case 1: lmin = s; break; case 2: lmax = s; break;
                     case 7: rsize = u; break; case 8: rid = u; hasRid = true; bitPos = 0; break; case 9: rcount = u; break; }
    } else if (type == 2) {
      switch (tag) { case 0: if (nU < 16) usages[nU++] = u; break; case 1: uMin = u; break; }
    } else if (type == 0) {
      if (tag == 8) {                                          // Input
        if (!(u & 1)) for (int n = 0; n < rcount; n++) {       // bukan konstanta
          int usage = nU > 0 ? usages[n < nU ? n : nU - 1] : (uMin >= 0 ? uMin + n : -1);
          HidField f{rid, (uint16_t)(bitPos + n * rsize), rsize, lmin, lmax};
          if (page == 9 && nBtn < 16) { btnUsage[nBtn] = usage > 0 ? usage : n + 1; fBtn[nBtn++] = f; }
          else if (page == 1 && usage >= 0x30 && usage <= 0x35) { fAxis[usage - 0x30] = f; hasAxis[usage - 0x30] = true; }
          else if (page == 1 && usage == 0x39) { fHat = f; hasHat = true; }
        }
        bitPos += rcount * rsize;
      }
      nU = 0; uMin = -1;
    }
  }
}

static uint32_t getBits(const uint8_t* d, int len, uint16_t off, uint8_t bits) {
  uint32_t v = 0;
  for (int b = 0; b < bits; b++) { int bit = off + b, by = bit >> 3; if (by >= len) break; if (d[by] & (1 << (bit & 7))) v |= 1u << b; }
  return v;
}
static bool fieldOk(const HidField& f, const uint8_t* d) { return !hasRid || f.rid == d[0]; }
static float axisNorm(const HidField& f, const uint8_t* d, int len) {
  uint16_t off = f.off + (hasRid ? 8 : 0);
  uint32_t raw = getBits(d, len, off, f.bits);
  float v = (float)raw;
  if (f.lmin < 0 && (raw & (1u << (f.bits - 1)))) v = (float)((int64_t)raw - (1LL << f.bits));
  if (f.lmax == f.lmin) return 0;
  return constrain(((v - f.lmin) / (float)(f.lmax - f.lmin)) * 2.0f - 1.0f, -1.0f, 1.0f);
}

static void publish(const GamepadState& n) {
  portENTER_CRITICAL(&mux); gp = n; portEXIT_CRITICAL(&mux);
  if (cb) cb(n);
}

static void handleHid(const uint8_t* d, int len) {
  GamepadState n; n.connected = true; strcpy(n.type, "DInput");
  if (hasAxis[0] && fieldOk(fAxis[0], d)) n.lx = axisNorm(fAxis[0], d, len);
  if (hasAxis[1] && fieldOk(fAxis[1], d)) n.ly = -axisNorm(fAxis[1], d, len);
  int rxi = (hasAxis[2] && hasAxis[5]) ? 2 : 3, ryi = (hasAxis[2] && hasAxis[5]) ? 5 : 4;   // Z/Rz atau Rx/Ry
  if (hasAxis[rxi] && fieldOk(fAxis[rxi], d)) n.rx = axisNorm(fAxis[rxi], d, len);
  if (hasAxis[ryi] && fieldOk(fAxis[ryi], d)) n.ry = -axisNorm(fAxis[ryi], d, len);
  for (int i = 0; i < nBtn; i++) {
    if (!fieldOk(fBtn[i], d)) continue;
    int u = btnUsage[i];
    if (u >= 1 && u <= 12 && getBits(d, len, fBtn[i].off + (hasRid ? 8 : 0), 1)) n.buttons |= HID_BTN_MAP[u - 1];
  }
  if (hasHat && fieldOk(fHat, d)) {
    int v = (int)getBits(d, len, fHat.off + (hasRid ? 8 : 0), fHat.bits) - fHat.lmin;
    if (v >= 0 && v <= 7) {
      if (v == 7 || v == 0 || v == 1) n.buttons |= BTN_UP;
      if (v >= 1 && v <= 3) n.buttons |= BTN_RIGHT;
      if (v >= 3 && v <= 5) n.buttons |= BTN_DOWN;
      if (v >= 5 && v <= 7) n.buttons |= BTN_LEFT;
    }
  }
  publish(n);
}

static void handleXinput(const uint8_t* d, int len) {
  if (len < 14 || d[0] != 0x00) return;
  GamepadState n; n.connected = true; strcpy(n.type, "XInput");
  uint8_t b0 = d[2], b1 = d[3];
  if (b0 & 0x01) n.buttons |= BTN_UP;   if (b0 & 0x02) n.buttons |= BTN_DOWN;
  if (b0 & 0x04) n.buttons |= BTN_LEFT; if (b0 & 0x08) n.buttons |= BTN_RIGHT;
  if (b0 & 0x10) n.buttons |= BTN_START; if (b0 & 0x20) n.buttons |= BTN_BACK;
  if (b1 & 0x01) n.buttons |= BTN_LB;   if (b1 & 0x02) n.buttons |= BTN_RB;
  if (b1 & 0x10) n.buttons |= BTN_A;    if (b1 & 0x20) n.buttons |= BTN_B;
  if (b1 & 0x40) n.buttons |= BTN_X;    if (b1 & 0x80) n.buttons |= BTN_Y;
  n.lx = (int16_t)(d[6] | d[7] << 8) / 32768.0f;  n.ly = (int16_t)(d[8] | d[9] << 8) / 32768.0f;
  n.rx = (int16_t)(d[10] | d[11] << 8) / 32768.0f; n.ry = (int16_t)(d[12] | d[13] << 8) / 32768.0f;
  publish(n);
}

// ---------------- USB transfer ----------------
static void inCb(usb_transfer_t* t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
    if (isXinput) handleXinput(t->data_buffer, t->actual_num_bytes); else handleHid(t->data_buffer, t->actual_num_bytes);
    if (!goneFlag && usb_host_transfer_submit(t) == ESP_OK) return;
  }
  inflight = false;
}
static void ctrlCb(usb_transfer_t*) { ctrlDone = true; }

static int getReportDescriptor(uint16_t len) {
  uint16_t wl = (len + 63) & ~63; if (wl > sizeof(descBuf)) wl = sizeof(descBuf);   // kelipatan MPS
  usb_transfer_t* x;
  if (usb_host_transfer_alloc(8 + wl, 0, &x) != ESP_OK) return -1;
  uint8_t* s = x->data_buffer;
  s[0] = 0x81; s[1] = 0x06; s[2] = 0x00; s[3] = 0x22; s[4] = ifNum; s[5] = 0; s[6] = wl & 0xFF; s[7] = wl >> 8;
  x->num_bytes = 8 + wl; x->device_handle = dev; x->bEndpointAddress = 0; x->callback = ctrlCb; x->context = nullptr;
  ctrlDone = false;
  if (usb_host_transfer_submit_control(client, x) != ESP_OK) { usb_host_transfer_free(x); return -1; }
  uint32_t t0 = millis();
  while (!ctrlDone && millis() - t0 < 2000) usb_host_client_handle_events(client, 20);   // pompa event sambil menunggu
  if (!ctrlDone) return -1;                                                               // (transfer dibiarkan bocor, aman)
  int got = -1;
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) { got = (int)x->actual_num_bytes - 8; if (got > 0) memcpy(descBuf, x->data_buffer + 8, got); }
  usb_host_transfer_free(x);
  return got;
}

static bool openDevice(uint8_t addr) {
  if (usb_host_device_open(client, addr, &dev) != ESP_OK) { dev = nullptr; return false; }
  const usb_config_desc_t* cfg;
  usb_host_get_active_config_descriptor(dev, &cfg);
  const uint8_t* p = (const uint8_t*)cfg; const uint8_t* end = p + cfg->wTotalLength;
  bool found = false, inCand = false, haveEp = false; uint8_t alt = 0, ep = 0; uint16_t mps = 0, hidLen = 0;
  while (p + 2 <= end && p[0] >= 2) {
    uint8_t type = p[1];
    if (type == 4 && !found) {                                    // interface
      bool xin = p[5] == 0xFF && p[6] == 0x5D && p[7] == 0x01;
      bool hid = p[5] == 0x03 && p[7] == 0x00;                     // lewati boot keyboard/mouse
      inCand = xin || hid;
      if (inCand) { ifNum = p[2]; alt = p[3]; isXinput = xin; hidLen = 0; haveEp = false; }
    } else if (type == 0x21 && inCand && p[0] >= 9) {              // HID descriptor
      hidLen = p[7] | (p[8] << 8);
    } else if (type == 5 && inCand && !haveEp && (p[3] & 3) == 3 && (p[2] & 0x80)) {   // endpoint interrupt IN
      ep = p[2]; mps = p[4] | (p[5] << 8); haveEp = true; found = true;
    }
    p += p[0];
  }
  if (!found || mps == 0) { usb_host_device_close(client, dev); dev = nullptr; return false; }
  if (usb_host_interface_claim(client, dev, ifNum, alt) != ESP_OK) { usb_host_device_close(client, dev); dev = nullptr; return false; }
  if (!isXinput) {
    int n = hidLen ? getReportDescriptor(hidLen) : -1;
    if (n <= 0) { usb_host_interface_release(client, dev, ifNum); usb_host_device_close(client, dev); dev = nullptr; return false; }
    parseHidDesc(descBuf, n);
  }
  if (usb_host_transfer_alloc(mps, 0, &inXfer) != ESP_OK) return false;
  inXfer->device_handle = dev; inXfer->bEndpointAddress = ep; inXfer->num_bytes = mps;
  inXfer->callback = inCb; inXfer->context = nullptr;
  goneFlag = false; inflight = true;
  if (usb_host_transfer_submit(inXfer) != ESP_OK) { inflight = false; return false; }
  GamepadState n; n.connected = true; strcpy(n.type, isXinput ? "XInput" : "DInput");
  publish(n);
  Serial.printf("[USB] Gamepad %s tersambung (if=%u ep=0x%02X)\n", n.type, ifNum, ep);
  return true;
}

static void closeDevice() {
  if (!dev) return;
  uint32_t t0 = millis();
  while (inflight && millis() - t0 < 500) usb_host_client_handle_events(client, 10);
  usb_host_interface_release(client, dev, ifNum);
  if (inXfer && !inflight) { usb_host_transfer_free(inXfer); inXfer = nullptr; }
  usb_host_device_close(client, dev); dev = nullptr;
  GamepadState n;                                                   // connected = false
  publish(n);
  Serial.println("[USB] Gamepad terputus");
}

static void clientEvt(const usb_host_client_event_msg_t* m, void*) {
  if (m->event == USB_HOST_CLIENT_EVENT_NEW_DEV && !dev) { newAddr = m->new_dev.address; pendingOpen = true; }
  else if (m->event == USB_HOST_CLIENT_EVENT_DEV_GONE && dev && m->dev_gone.dev_hdl == dev) goneFlag = true;
}

static void hostTask(void*) {
  for (;;) { uint32_t f; usb_host_lib_handle_events(portMAX_DELAY, &f); if (f & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all(); }
}
static void clientTask(void*) {
  usb_host_client_config_t cc = {}; cc.is_synchronous = false; cc.max_num_event_msg = 5;
  cc.async.client_event_callback = clientEvt; cc.async.callback_arg = nullptr;
  usb_host_client_register(&cc, &client);
  for (;;) {
    usb_host_client_handle_events(client, 50);
    if (pendingOpen) { pendingOpen = false; openDevice(newAddr); }
    if (goneFlag || (dev && !inflight)) { goneFlag = false; closeDevice(); }   // putus / error transfer
  }
}

void gamepadBegin(GamepadCb c) {
  cb = c;
  usb_host_config_t hc = {}; hc.skip_phy_setup = false; hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
  if (usb_host_install(&hc) != ESP_OK) { Serial.println("[USB] usb_host_install GAGAL"); return; }
  xTaskCreatePinnedToCore(hostTask, "usbhost", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(clientTask, "usbcli", 6144, nullptr, 4, nullptr, 0);
}
void gamepadGet(GamepadState& o) { portENTER_CRITICAL(&mux); o = gp; portEXIT_CRITICAL(&mux); }
