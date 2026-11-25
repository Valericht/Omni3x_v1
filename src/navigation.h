#pragma once
#include "config.h"

enum NavSrc : uint8_t { SRC_NONE = 0, SRC_PAD = 1, SRC_WEB = 2 };
struct Waypoint { bool valid; float x, y; float yaw; bool hasYaw; float wallL, wallR, wallF; };   // cm dan derajat, frame dunia (asal = posisi dan arah saat titik 0 ditetapkan); wall* = referensi ultrasonik kiri, kanan, depan (0 = tidak ada)

// Permintaan lintas-thread (Web / USB callback -> task kontrol). Dibaca dan dihapus oleh navConsumeRequests().
struct NavReq {
  int    gotoId = 0;                 // jalan ke satu waypoint (1..MAX_WP)
  int    route = -1;                 // jalankan rute 0..3 (A,B,X,Y)
  NavSrc src = SRC_NONE;
  bool   cancel = false;             // STOP
  bool   zero = false;               // tetapkan titik 0 (posisi dan arah)
  bool   resume = false;             // buka kunci pengaman
  bool   jogSet = false;             // ada perintah jalan-terus
  int    jog = 0;                    // 0 berhenti, 1 maju, 2 mundur, 3 kiri, 4 kanan, 5 putar kiri, 6 putar kanan
  int    stopSave = 0;               // berhenti lalu simpan posisi sebagai waypoint ini
  int    preset = -1;                // preset 0..3 (A..D): satu perintah langsung menjalankan rute
  int    grip = -1;                  // gripper manual: 0 lepas, 1 jepit, -1 tidak ada
  int    sync = 0;                   // L6: VERIFY manual di waypoint ini (robot diam <= 30 cm darinya)
};

void  navBegin();                                  // muat waypoint, rute, dan posisi terakhir dari flash
void  navResetOdom();
// Slope-aware odometry (A.5.4). pitch/roll in degrees (0 if IMU not ok): forward *= cos(pitch), lateral *= cos(roll) before body->world.
// uncertain (terrain != FLAT or slip): displacement *= encTrust; encTrust < ENCODER_TRUST_FREEZE freezes x,y (yaw keeps integrating).
// yawEncDeg (optional): encoder yaw in degrees, integrated here and never frozen. Returns true when the pose is uncertain and encTrust < 1.
// L7: komponen maju dikali ODOM_SCALE_FWD / BWD (dan UP / DOWN di ramp) sebelum proyeksi kemiringan.
bool  navOdom(const float wheelCmS[3], float yaw, float dt, float pitchDeg = 0, float rollDeg = 0, float encTrust = 1.0f, bool uncertain = false, float* yawEncDeg = nullptr);
void  navSetPoseDegraded(bool v);                  // true = pose uncertain (A.5.4): new navStart/navStartRoute refused
bool  navPoseDegraded();
void  navRampMark(int id, bool on);                // mark waypoint 1..MAX_WP as a known ramp-end point (persisted: nav/ramp)
bool  navRampSnap(float tolCm);                    // snap x,y to the nearest marked ramp waypoint within tolCm; false if none marked / out of tolerance
void  navPose(float& x, float& y);
bool  navSavedPose(float& x, float& y, float& yawRad, bool& valid);   // posisi terakhir yang tersimpan sebelum mati
void  navSetPoseValid(bool v);                     // false = posisi tidak valid, navigasi otomatis ditolak
bool  navPoseValid();
void  navStartPoseSaver();                         // task Core 0: simpan posisi terakhir saat robot diam

bool  navStart(int id, NavSrc src);                // satu waypoint (1..MAX_WP)
bool  navStartRoute(int r, NavSrc src);            // rute 0..3: semua waypoint berurutan
bool  navStartVerify(int id, NavSrc src);          // L6 "Sinkron ke WP": hanya fase VERIFY; false bila WP tanpa referensi, posisi > 30 cm, atau posisi tidak valid
void  navCancel();
bool  navActive();
int   navTarget();                                 // waypoint yang sedang dituju (0 = tidak ada)
int   navRoute();                                  // rute yang sedang jalan, -1 = tunggal / tidak ada
int   navStepNo();                                 // langkah sekarang (1-based)
int   navStepCount();
NavSrc navSource();
// Kecepatan frame badan. State: MOVING (menuju x,y; di dalamnya status AVOID, L9) -> ROTATING (spin di tempat ke yaw waypoint) -> VERIFY (L6, bila WP punya ref dinding) -> DONE.
// rotating = true: vx = vy = 0 dan badan harus diputar ke yawTarget (rad). false dikembalikan bila selesai / batal.
bool  navStep(float yaw, float rateDps, float vCms, bool imuOk, float& vx, float& vy, float& yawTarget, bool& rotating);
int   navPhase();                                  // 0 tidak ada, 1 MOVING, 2 ROTATING, 3 VERIFY, 4 GRIP (langkah gripper di rute, robot diam)
uint8_t navDoneSeq();                              // naik 1 setiap navigasi tuntas (bukan batal)
bool  navRouteReady(int r);                        // rute r punya minimal satu waypoint valid
// wallRef (kiri, kanan; 0 = tidak ada): sensor sisi dengan referensi hanya dianggap halangan bila d < ref - WALL_OBST_DELTA.
void  avoidObstacles(float& vx, float& vy, const float d[3], const float wallRef[2] = nullptr);
float navAvoidScale();                             // L8: skala kecepatan hasil avoidObstacles terakhir (1 = tidak ada)
// L4: jarak untuk keputusan menghindar = median valid (bukan mentah), -1 = tidak valid. Depan dikecualikan bila itu dinding referensi waypoint aktif (wallF).
void  navObstDist(const float med[3], const bool ok[3], float out[3]);
void  navSetIr(bool front, bool left, bool right);    // IR depan/kiri/kanan (sudah debounce); dipanggil tiap tick sebelum navStep
bool  navObstConfirmed(const float d[3], float vx, float vy);   // L3: ada rintangan <= OBST_STOP_CM di arah gerak (v dalam frame badan)
// L3: lantai kecepatan translasi setelah SEMUA pengali. dirX/dirY = vektor gerak sebelum pengali (arah dipertahankan). Tidak berlaku di luar MOVING, saat AVOID, atau rintangan terkonfirmasi.
void  navSpeedFloor(float& vx, float& vy, float dirX, float dirY, float vLimCms, bool obstConfirmed);
bool  navAvoiding();                               // L9: AVOID aktif ATAU masih dalam AVOID_HOLD_MS (koreksi dinding ditahan, stuck dimatikan)
bool  navAvoidActive();                            // AVOID aktif (HOLD / SHIFT / PASS)
uint8_t navAvoidState();                           // telemetri av: 0 normal, 1 HOLD, 2 SHIFT, 3 PASS
bool  navRefStale();                               // L5: ref dinding tidak cocok > 1 s pada leg MOVING
float navSideMin();                                // L8: bacaan sisi terkecil selama rute (-1 = belum ada)
ArrLog navArrLog();                                // L8: kejadian tiba terakhir
// L5: referensi dinding yang boleh direkam (out = kiri, kanan, depan; 0 = tidak lolos). Syarat: 5 <= d <= WALL_REF_MAX_CM, 5 sampel median terakhir menyebar <= WALL_REF_SPREAD_CM, datar, diam.
void  navWallRefCapture(bool still, bool flat, float out[3]);

// Koreksi dinding (R3). med/ok = median ultrasonik (depan, kiri, kanan). stat: 0 tidak dipakai, 1 aktif, 2 fallback encoder, 3 offset melebihi batas.
struct WallCorr { float dpx, dpy; float vx; uint8_t stat; };   // dpx,dpy = koreksi pose dunia penuh; vx = dorongan steering frame badan. stat 4 = menghindar (diisi navigasi, L9)
// L4: tilt = terrain TILT -> koreksi ditahan (stat 2). SLOPE tidak menahan.
WallCorr wallCalc(const Waypoint& w, float x, float y, float yaw, const float med[3], const bool ok[3], bool tilt = false);
void  navSetWall(const float med[3], const bool ok[3], bool tilt = false);
void  navWallRef(float ref[2]);                    // referensi dinding waypoint aktif (0 = tidak ada)
uint8_t navWallStat();                             // status koreksi pada leg MOVING yang berjalan
bool  navRetryLeg();                               // R5: stuck di MOVING -> coba ulang; false = jatah habis, navigasi dibatalkan (deny 6)
uint8_t navRetryNo();
void  navStats(uint16_t s[4]);                     // tiba, gagal, retry, koreksi dipakai (sesi)

// jalan-terus dari dashboard (lokalisasi waypoint)
void  navSetJog(int dir);
int   navJog();

void  navRequestGoto(int id, NavSrc src);
void  navRequestRoute(int r, NavSrc src);
void  navRequestCancel();
void  navRequestZero();
void  navRequestResume();
void  navRequestJog(int dir);
void  navRequestStopSave(int id);
void  navRequestPreset(int i, NavSrc src);
void  navRequestSync(int id, NavSrc src);          // L6: "Sinkron ke WP n"
void  navPostDeny(uint8_t why);                    // thread lain menolak perintah (1 posisi tidak valid); dilaporkan ke dashboard oleh task kontrol
bool  navTakeDeny(uint8_t& why);
bool  navTakeGrip(bool& on);                       // gripper: sekali per langkah gripper; task kontrol lalu memanggil gripperSet(on)
void  navRequestGrip(bool on);                     // gripper manual (dashboard / app)
void  navConsumeRequests(NavReq& r);

// waypoint dan rute (tersimpan di flash)
bool  wpGet(int id, Waypoint& w);
bool  wpSet(int id, float x, float y, float yawDeg, bool hasYaw = true, float wallL = 0, float wallR = 0, float wallF = 0);   // wall: cm, 0 = tidak ada
bool  wpSetKeepYaw(int id, float x, float y, float wallL = 0, float wallR = 0, float wallF = 0);      // ubah x, y saja; arah hadap lama dipertahankan
bool  wpDelete(int id);
void  wpCopyAll(Waypoint out[MAX_WP]);
bool  routeSet(int r, const uint8_t* ids, int n);  // ids: nomor waypoint 1..MAX_WP atau GRIP_STEP_ON / GRIP_STEP_OFF; n = 0 mengosongkan rute
void  routeCopyAll(uint8_t out[NUM_ROUTES][MAX_RT]);   // tiap rute diakhiri 0 bila belum penuh
