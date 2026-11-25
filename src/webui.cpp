#include "webui.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "monitor_html.h"
#include "navigation.h"
#include "movecheck.h"
#include "motors.h"
#include "settings.h"

// Dua jalur monitoring dengan protokol JSON yang sama:
//   Wi-Fi : WebSocket ws://192.168.4.1:81/ws  (satu pesan = satu JSON)
//   Kabel : Serial (port UART, SERIAL_BAUD)     (satu baris = satu JSON, diakhiri '\n')
// Telemetri lewat kabel hanya dikirim bila ada detak ("hb") dari PC dalam 2 detik terakhir,
// jadi monitor serial biasa tetap bersih. Setiap perintah yang masuk memperbarui detak jalurnya;
// fail-safe di main.cpp baru aktif bila kedua jalur diam.

static AsyncWebServer httpServer(80), wsServer(81);
static AsyncWebSocket ws("/ws");
static float r1(float v) { return roundf(v * 10.0f) / 10.0f; }

enum Link : uint8_t { LINK_WIFI, LINK_SERIAL };

static void handleMsg(const uint8_t* data, size_t len, Link link) {
  JsonDocument doc;
  if (deserializeJson(doc, data, len)) return;
  if (link == LINK_WIFI) g_hbWifiMs = millis(); else g_hbSerialMs = millis();
  const char* cmd = doc["cmd"] | "";
  int id = doc["id"] | 0;
  int m  = doc["m"] | -1;                       // roda 0..2, -1 = semua roda
  if      (!strcmp(cmd, "goto"))       navRequestGoto(id, SRC_WEB);
  else if (!strcmp(cmd, "stop"))       navRequestCancel();
  else if (!strcmp(cmd, "wp_set"))   {                                    // yaw ada: simpan dengan arah; "noyaw": tanpa arah (tidak memutar); lainnya: arah lama dipertahankan
    float wl = doc["wallL"] | 0.0f, wr = doc["wallR"] | 0.0f, wf = doc["wallF"] | 0.0f;      // R2/L6: referensi dinding manual (cm, 0 / absen = tidak ada); tidak dibatasi aturan L5
    if (doc["yaw"].is<float>())          wpSet(id, doc["x"] | 0.0f, doc["y"] | 0.0f, doc["yaw"] | 0.0f, true, wl, wr, wf);
    else if (doc["noyaw"] | false)       wpSet(id, doc["x"] | 0.0f, doc["y"] | 0.0f, 0.0f, false, wl, wr, wf);
    else                                 wpSetKeepYaw(id, doc["x"] | 0.0f, doc["y"] | 0.0f, wl, wr, wf);
  }
  else if (!strcmp(cmd, "wp_here"))  {                                    // ditolak bila posisi tidak valid
    Telemetry t = telemetryGet();
    if (t.poseLost) navPostDeny(1);
    else {                                                                // L5: ref dinding hanya bila valid (5..100 cm, stabil), datar, dan robot diam; selain itu 0
      float r3[3]; bool still = t.mode != 1 && t.mode != 3 && t.mode != 4 && hypotf(t.mvx, t.mvy) < 2.0f && fabsf(t.wz) < 0.05f;
      navWallRefCapture(still, t.terrain == 0, r3);
      wpSet(id, t.x, t.y, t.yawDeg, true, r3[0], r3[1], r3[2]);
    }
  }
  else if (!strcmp(cmd, "wp_sync"))    navRequestSync(id, SRC_WEB);       // L6: VERIFY manual di WP id (robot diam <= 30 cm darinya)
  else if (!strcmp(cmd, "wp_del"))     wpDelete(id);
  else if (!strcmp(cmd, "ramp_mark"))  navRampMark(id, doc["on"] | true);   // A.5.4: tandai WP sebagai titik ujung ramp (tidak disimpan ke flash)
  else if (!strcmp(cmd, "odom_reset")) navRequestZero();         // titik 0 = posisi + arah hadap (dijalankan task kontrol)
  else if (!strcmp(cmd, "resume"))     navRequestResume();       // buka kunci pengaman
  else if (!strcmp(cmd, "preset"))     navRequestPreset(doc["i"] | -1, SRC_WEB);   // 0..3 = A..D: satu perintah langsung menjalankan rute
  else if (!strcmp(cmd, "route_go"))   navRequestRoute(doc["r"] | -1, SRC_WEB);
  else if (!strcmp(cmd, "route_set")) {
    uint8_t ids[MAX_RT]; int n = 0;
    for (JsonVariantConst v : doc["ids"].as<JsonArrayConst>()) if (n < MAX_RT) ids[n++] = (uint8_t)constrain(v.as<int>(), 0, 255);   // routeSet menyaring: 1..MAX_WP atau kode gripper
    routeSet(doc["r"] | -1, ids, n);
  }
  else if (!strcmp(cmd, "grip"))       navRequestGrip(doc["on"] | false);   // gripper manual: {"cmd":"grip","on":true|false}
  else if (!strcmp(cmd, "jog"))        navRequestJog(doc["d"] | 0);          // 1 maju, 2 mundur, 3 kiri, 4 kanan, 5 putar kiri, 6 putar kanan, 0 berhenti
  else if (!strcmp(cmd, "stop_save"))  navRequestStopSave(id);               // berhenti lalu simpan posisi sebagai WP id
  else if (!strcmp(cmd, "speed"))    { Settings c = settingsGet(); settingsSet(doc["man"] | c.man, doc["turbo"] | c.turbo, doc["nav"] | c.nav, doc["loc"] | c.loc); }
  else if (!strcmp(cmd, "safety"))     settingsSetSafety(doc["on"] | true);
  else if (!strcmp(cmd, "mtest"))      mcRequest(doc["dir"] | 0);     // 1 maju, 2 mundur, 3 kiri, 4 kanan
  else if (!strcmp(cmd, "autotune"))   motorsAutoTuneStart(m, AT_STEP_PCT);
  else if (!strcmp(cmd, "pid"))        motorsSetPID(m, doc["kp"] | 0.0f, doc["ki"] | 0.0f, doc["kd"] | 0.0f, true);
  // "hb" dan perintah lain: hanya memperbarui detak
}

static void onWs(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DISCONNECT) { if (ws.count() <= 1) g_hbWifiMs = 0; }    // jalur Wi-Fi mati seketika (kabel tidak ikut)
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) handleMsg(data, len, LINK_WIFI);
  }
}

static void buildTelemetry(String& out) {
  Telemetry t = telemetryGet();
  Waypoint w[MAX_WP]; wpCopyAll(w);
  JsonDocument d;
  d["x"] = r1(t.x); d["y"] = r1(t.y); d["yaw"] = r1(t.yawDeg);
  d["vx"] = r1(t.vx); d["vy"] = r1(t.vy); d["wz"] = roundf(t.wz * 100) / 100;
  d["mode"] = t.mode; d["tgt"] = t.navTarget; d["spd"] = (int)(t.rpmLimit + 0.5f); d["tb"] = t.turbo; d["arm"] = t.armed;
  d["pad"] = t.padConnected; d["pt"] = t.padType; d["imu"] = t.imuOk; d["bat"] = r1(t.batt); d["tune"] = t.tune; d["tw"] = t.tuneWhy;
  for (int i = 0; i < 3; i++) {
    d["rpm"][i] = r1(t.rpm[i]); d["sp"][i] = r1(t.sp[i]); d["d"][i] = r1(t.dist[i]); d["sa"][i] = SENSOR_ANG_DEG[i];
    d["kp"][i] = t.kp[i]; d["ki"][i] = t.ki[i]; d["kd"][i] = t.kd[i];
  }
  d["mvx"] = r1(t.mvx); d["mvy"] = r1(t.mvy);
  for (int i = 0; i < 3; i++) { d["dm"][i] = r1(t.usMed[i]); d["dv"][i] = t.usOk[i]; }
  d["ir"] = t.ir;
  d["wl"] = r1(t.usMed[1]); d["wr"] = r1(t.usMed[2]); d["wlr"] = r1(t.wallRef[0]); d["wrr"] = r1(t.wallRef[1]);
  d["wstat"] = t.wallStat; d["rty"] = t.navRetry; for (int i = 0; i < 4; i++) d["ss"][i] = t.sess[i];
  // L8: rantai kecepatan, jarak sisi, referensi usang, tiba terakhir ("slf" = faktor tanjakan; "sf" sudah dipakai untuk status pengaman)
  d["slf"] = roundf(t.slopeF * 100) / 100; d["as"] = roundf(t.avoidScale * 100) / 100; d["re"] = r1(t.effRpm); d["smin"] = r1(t.sideMin); d["av"] = t.avoid; d["rst"] = t.refStale;
  { JsonObject a = d["arr"].to<JsonObject>(); a["q"] = t.arr.seq; a["id"] = t.arr.id; a["n"] = t.arr.verifyN; a["v"] = t.arr.result; a["rl"] = r1(t.arr.resLat); a["rf"] = r1(t.arr.resFwd);
    for (int i = 0; i < 3; i++) { a["m"][i] = r1(t.arr.m[i]); a["r"][i] = r1(t.arr.ref[i]); } }
  d["ia"] = t.imuAddr; d["ic"] = t.imuCal; d["in"] = t.imuNdof;
  JsonObject m = d["mc"].to<JsonObject>();
  m["s"] = t.mc.state; m["dir"] = t.mc.dir; m["why"] = t.mc.why; m["o"] = t.mc.vOdom; m["i"] = t.mc.vImu; m["yi"] = t.mc.yawFromImu;
  m["dx"] = r1(t.mc.dx); m["dy"] = r1(t.mc.dy); m["ix"] = r1(t.mc.ix); m["iy"] = r1(t.mc.iy); m["dyaw"] = r1(t.mc.dyaw);
  d["rt"] = t.navRoute; d["rs"] = t.navStep; d["rn"] = t.navSteps; d["jg"] = t.jog;
  d["sf"] = t.safe; d["so"] = t.safeOn; d["pl"] = t.poseLost; d["pr"] = t.poseRestored; d["sat"] = t.sat;
  for (int i = 0; i < 4; i++) { d["cs"][i] = (int)(t.cfg[i] + 0.5f); d["sfv"][i] = r1(t.safeDbg[i]); }
  d["sv"][0] = t.svStatus; d["sv"][1] = t.svWhat; d["sv"][2] = t.svCount;
  d["dn"][0] = t.deny; d["dn"][1] = t.denySeq;
  { uint8_t rts[NUM_ROUTES][MAX_RT]; routeCopyAll(rts); JsonArray ra = d["rts"].to<JsonArray>();
    for (int r = 0; r < NUM_ROUTES; r++) { JsonArray rr = ra.add<JsonArray>(); for (int k = 0; k < MAX_RT && rts[r][k]; k++) rr.add(rts[r][k]); } }
  for (int i = 0; i < 4; i++) d["ps"][i] = t.pst[i];
  d["np"] = t.navPhase; d["nd"] = t.navDone; d["gp"] = t.gripper;
  { static const char* const TR[3] = {"FLAT", "SLOPE", "TILT"};              // IMU-assist (PRD IMU)
    d["terrain"] = TR[t.terrain < 3 ? t.terrain : 0]; d["yaw_error"] = r1(t.yawErr); d["stuck"] = t.stuck; d["slip"] = t.slip; d["pose_degraded"] = t.poseDegraded;
    d["encoder_trust"] = roundf(t.encTrust * 100) / 100; d["pitch"] = r1(t.pitch); d["roll"] = r1(t.roll);
    JsonObject py = d["pid"].to<JsonObject>();
    py["kp"] = roundf(t.yKp * 100) / 100; py["ki"] = roundf(t.yKi * 100) / 100; py["kd"] = roundf(t.yKd * 100) / 100; py["status"] = t.yStatus ? "DONE" : "TUNING"; }
  for (int i = 0; i < MAX_WP; i++) { d["wp"][i][0] = w[i].valid ? 1 : 0; d["wp"][i][1] = r1(w[i].x); d["wp"][i][2] = r1(w[i].y); d["wp"][i][3] = r1(w[i].yaw); d["wp"][i][4] = w[i].hasYaw ? 1 : 0; d["wp"][i][5] = r1(w[i].wallL); d["wp"][i][6] = r1(w[i].wallR); d["wp"][i][7] = r1(w[i].wallF); }
  serializeJson(d, out);
}

static void broadcastTask(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    ws.cleanupClients();
    uint32_t hs = g_hbSerialMs;
    bool serialOn = hs && millis() - hs < 2000;
    if (ws.count() == 0 && !serialOn) continue;
    String out; buildTelemetry(out);
    if (ws.count()) ws.textAll(out);
    if (serialOn) { out += '\n'; Serial.write((const uint8_t*)out.c_str(), out.length()); }   // satu write: baris tidak terpotong
  }
}

// Baca perintah dari kabel: kumpulkan sampai '\n', lalu proses. Baris terlalu panjang dibuang.
static void serialRxTask(void*) {
  static char buf[256]; size_t n = 0; bool drop = false;
  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') { if (!drop && n) handleMsg((const uint8_t*)buf, n, LINK_SERIAL); n = 0; drop = false; }
      else if (c != '\r') { if (n < sizeof(buf)) buf[n++] = c; else drop = true; }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void webBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WEB] AP '%s'  http://%s/\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", MONITOR_HTML); });
  httpServer.begin();
  ws.onEvent(onWs); wsServer.addHandler(&ws); wsServer.begin();
  xTaskCreatePinnedToCore(broadcastTask, "wsTx", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(serialRxTask, "serRx", 8192, nullptr, 2, nullptr, 0);
}
