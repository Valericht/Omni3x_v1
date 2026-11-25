#pragma once
// =====================================================================
//  Omni-3X  -  konfigurasi pusat (pin, mekanik, tuning, keamanan)
// =====================================================================
#include <Arduino.h>
#include <math.h>

// ---------------- Pin (sesuai PRD 3.3) ----------------
constexpr int PIN_M_IN1[3]  = {1, 4, 6};
constexpr int PIN_M_IN2[3]  = {2, 5, 17};
constexpr int PIN_ENC_A[3]  = {18, 11, 13};
constexpr int PIN_ENC_B[3]  = {10, 12, 14};
constexpr int PIN_I2C_SDA   = 8;
constexpr int PIN_I2C_SCL   = 9;
constexpr int PIN_US_TRIG   = 21;
constexpr int PIN_US_ECHO[3]= {15, 16, 47};
// IR digital (aktif LOW pada modul umum). Keluaran 5 V WAJIB lewat pembagi tegangan / modul 3,3 V. GPIO 19/20 = USB, jangan dipakai.
constexpr int PIN_IR_FRONT  = 38;            // dekat ultrasonik depan
constexpr int PIN_IR_LEFT   = 39;            // sisi kiri, menghadap depan
constexpr int PIN_IR_RIGHT  = 40;            // sisi kanan, menghadap depan
constexpr bool IR_ACTIVE_LOW = true;
constexpr uint32_t IR_DEBOUNCE_MS = 40;
constexpr int PIN_GRIPPER   = 7;             // solenoid gripper lewat MOSFET / relay (GPIO 7 bebas); ubah sesuai wiring
constexpr bool GRIPPER_ACTIVE_HIGH = true;   // true: HIGH = solenoid ON
constexpr uint32_t GRIPPER_ACTUATE_MS = 500; // rute menunggu selama ini setelah langkah gripper sebelum lanjut
constexpr uint8_t GRIP_STEP_ON  = 101;       // kode langkah di dalam rute (1..MAX_WP = waypoint)
constexpr uint8_t GRIP_STEP_OFF = 102;
constexpr int PIN_BATT_ADC  = -1;            // -1 = nonaktif (ADC1 habis terpakai; ADC2 bentrok WiFi)
constexpr float BATT_DIV    = 4.0f;          // rasio pembagi tegangan baterai

// ---------------- Mekanik ----------------
constexpr float ENC_CPR         = 3860.0f;   // hitungan per putaran roda (edge RISING phase A)
constexpr float WHEEL_R_CM      = 4.88f;     // radius efektif roda (kalibrasi lurus: 482 cm nyata vs 296.2 cm odometri)
constexpr float WHEEL_R_CM_OLD  = 3.0f;      // nilai lama, hanya untuk migrasi skala waypoint/posisi tersimpan
constexpr float ROBOT_L_CM      = 12.0f;     // jari-jari robot (pusat -> roda)
constexpr float MAX_WHEEL_RPM   = 500.0f;     // batas keras RPM roda (turbo). Motor harus memang mampu; encoder 3860 CPR @500 RPM = ~32 k edge/s per roda
constexpr float MAX_LIN_CMS     = MAX_WHEEL_RPM * 2.0f * (float)M_PI * WHEEL_R_CM / 60.0f;
constexpr float THETA_DEG[3]    = {90.0f, 210.0f, 330.0f};
constexpr int   MOTOR_SIGN[3]   = {1, 1, 1}; // balik bila motor berputar terbalik
constexpr int   ENC_SIGN[3]     = {1, 1, 1}; // balik bila hitungan encoder negatif saat maju
constexpr float KIN_Y_SIGN      = -1.0f;     // -1 = maju/mundur dibalik (kiri/kanan tetap). Ubah ke 1 bila sudah benar tanpa koreksi
constexpr float KIN_W_SIGN      = 1.0f;      // -1 bila putaran (yaw) terbalik
// Konvensi badan robot: +x = kanan, +y = depan, yaw positif = CCW

// ---------------- PWM / loop ----------------
constexpr int   PWM_FREQ        = 10000;   // hasil lapangan: 10 kHz paling sedikit bacaan ultrasonik palsu (20 kHz, 25 kHz lebih buruk); BTS7960 aman, suara terdengar
constexpr int   PWM_BITS        = 10;
constexpr int   CONTROL_PERIOD_MS = 20;      // 50 Hz

// ---------------- PID motor (default, bisa di-tune & disimpan di NVS) ----------------
// Satuan gain: % PWM per RPM (sama dengan sketch test_motorPID, nilai bisa disalin langsung).
constexpr float PID_KP_DEF = 0.5f, PID_KI_DEF = 0.5f, PID_KD_DEF = 0.0f;
constexpr float PID_I_LIMIT     = 100.0f;    // batas integral (% PWM)
// Auto-tune step response (open loop, metode sama dengan sketch test_motorPID): model FOPDT K, tau, L
constexpr float    AT_STEP_PCT   = 50.0f;    // besar step PWM (%)
constexpr uint32_t AT_MAX_MS     = 10000;    // batas waktu rekam
constexpr int      AT_WIN        = 25;       // jendela rata-rata steady (sampel, 25 x 20 ms = 0,5 dtk)
constexpr float    AT_MIN_DY_RPM = 5.0f;     // respons minimum agar dianggap bergerak
constexpr float    AT_STEADY_ABS = 3.0f;     // toleransi mantap (RPM)
constexpr int      AT_METHOD     = 2;        // 0 Ziegler-Nichols (agresif), 1 Cohen-Coon, 2 IMC (halus)

// ---------------- Gamepad ----------------
constexpr float PAD_DEADZONE    = 0.12f;
constexpr uint32_t ESTOP_HOLD_MS = 500;      // tahan B = STOP darurat

// ---------------- Kecepatan (RPM roda). Nilai default; bisa diubah di dashboard dan tersimpan di flash ----------------
constexpr float SPD_MAN_DEF   = 200.0f;      // manual normal (stick penuh)
constexpr float SPD_TURBO_DEF = 500.0f;      // manual turbo (tahan R1)
constexpr float SPD_NAV_DEF   = 200.0f;      // navigasi otomatis (goto / rute)
constexpr float SPD_LOC_DEF   = 150.0f;      // jalan dari dashboard untuk mencari / merekam waypoint
constexpr float SPD_MIN_RPM   = 30.0f;
constexpr float ROT_FRAC      = 0.5f;        // putaran badan = ROT_FRAC x (kecepatan / ROBOT_L_CM) pada stick penuh
constexpr float rpmToCms(float rpm) { return rpm * 2.0f * (float)M_PI * WHEEL_R_CM / 60.0f; }

// ---------------- Heading lock (PID sudut) ----------------
constexpr float HEAD_KP = 4.0f, HEAD_KI = 1.2f, HEAD_KD = 0.25f, HEAD_WZ_MAX = 3.0f, HEAD_I_MAX = 1.0f;   // D memakai laju gyro langsung (rad/s)
constexpr float IMU_YAW_SIGN    = 1.0f;     // balik (-1) bila yaw terbalik: yaw positif = CCW (BNO055: heading searah jarum jam, dibalik di imu.cpp)

// ---------------- IMU BNO055 (I2C, alamat 0x28/0x29 otomatis) ----------------
constexpr uint32_t IMU_I2C_HZ   = 100000;   // 100 kHz: BNO055 memakai clock stretching
constexpr bool  IMU_USE_NDOF    = false;    // false = mode IMU (gyro+akselerometer, yaw relatif, kebal magnet motor)
                                            // true  = NDOF (yaw kompas, terganggu magnet motor)
// Pemasangan sensor -> sumbu robot (+x kanan, +y depan), dipakai tes gerak untuk percepatan linear.
constexpr int   IMU_ACC_FWD_AXIS   = 1;     // sumbu sensor yang menghadap depan: 0 = X, 1 = Y (sumbu lain = kanan)
constexpr float IMU_ACC_FWD_SIGN   = 1.0f;  // balik (-1) bila IMU bilang maju padahal robot mundur
constexpr float IMU_ACC_RIGHT_SIGN = 1.0f;  // balik (-1) bila IMU bilang kanan padahal robot ke kiri

// ---------------- Tes gerak (dashboard: maju, mundur, kiri, kanan) ----------------
constexpr float    TEST_V_CMS        = 20.0f;   // kecepatan tes (cm/s)
constexpr uint32_t TEST_REST_MS      = 400;     // diam dulu (ukur bias akselerometer)
constexpr uint32_t TEST_RUN_MS       = 1500;    // lama gerak
constexpr uint32_t TEST_BRAKE_MS     = 600;     // waktu berhenti sebelum dinilai
constexpr float    TEST_MIN_MOVE_CM  = 6.0f;    // geser odometri minimum agar dianggap bergerak
constexpr float    TEST_IMU_MIN_DV   = 8.0f;    // perubahan kecepatan IMU minimum (cm/s)

// ---------------- Navigasi & obstacle ----------------
constexpr int   MAX_WP          = 16;        // jumlah waypoint
constexpr int   NUM_ROUTES      = 4;         // rute: tombol A, B, X, Y
constexpr int   MAX_RT          = 16;        // maksimum waypoint per rute
constexpr float NAV_V_MIN_CMS   = 6.0f;
constexpr float NAV_ARRIVE_CM   = 5.0f;      // radius tiba di waypoint terakhir
constexpr float NAV_PASS_CM     = 12.0f;     // radius "lewat" di waypoint perantara (tanpa berhenti)
constexpr float NAV_SLOW_CM     = 40.0f;     // jarak minimum mulai melambat di waypoint terakhir
constexpr float NAV_SLOW_S      = 0.7f;      // atau kecepatan x detik ini, mana yang lebih besar
constexpr uint32_t NAV_TIMEOUT_MS = 90000;   // batas waktu per waypoint
constexpr uint8_t  NAV_RETRY      = 2;       // R5: percobaan ulang pendekatan bila timeout / stuck, sebelum cancel
// Memutar badan di waypoint terakhir (STATE ROTATING): spin di tempat sampai arah hadap = yaw waypoint
constexpr float    NAV_YAW_TOL_DEG    = 3.0f;    // toleransi arah hadap
constexpr float    NAV_YAW_RATE_OK_DPS = 10.0f;  // dianggap diam bila laju putar di bawah ini
constexpr uint32_t NAV_YAW_SETTLE_MS  = 250;     // harus bertahan dalam toleransi selama ini (anti jitter / overshoot)
constexpr float    NAV_YAW_LPF        = 0.35f;   // filter low-pass galat yaw untuk penentuan selesai (0..1, makin kecil makin halus)
constexpr float    NAV_ROT_MIN_WZ     = 0.35f;   // laju putar minimum saat memutar (rad/s), supaya roda tidak macet di galat kecil
constexpr uint32_t NAV_ROT_TIMEOUT_MS = 15000;   // batas waktu memutar badan
constexpr float OBST_DIST_CM    = 25.0f;     // mulai menghindar (ditambah OBST_SPD_GAIN x kecepatan)
constexpr float OBST_STOP_CM    = 10.0f;
constexpr float OBST_SPD_GAIN   = 0.4f;      // detik: makin cepat, makin jauh mulai menghindar
constexpr float SENSOR_ANG_DEG[3] = {90.0f, 180.0f, 0.0f};   // arah sensor di frame badan (dari +x kanan, CCW): 0 depan, 1 kiri, 2 kanan

// ---- Wall correction ----
constexpr float WALL_CORR_MAX_CM  = 20.0f;   // |offset| lebih besar: tidak dikoreksi, hanya dilaporkan
constexpr float WALL_POSE_KP      = 0.05f;   // low-pass koreksi pose per tick kontrol
constexpr float WALL_POSE_STEP_CM = 0.2f;    // langkah koreksi pose maksimum per tick
constexpr float WALL_STEER_KP     = 0.5f;    // cm/s dorongan vx per cm offset
constexpr float WALL_DEADZONE_CM  = 3.0f;    // naik dari 1.5: bacaan sisi berisik tidak membuat robot menyundul
constexpr float WALL_STEER_MAX_CMS = 5.0f;   // batas dorongan steering dinding: robot tetap maju sambil pelan-pelan lurus
constexpr uint32_t WALL_HOLD_MS   = 300;     // sensor sisi tidak valid sesaat: koreksi steering terakhir dipertahankan selama ini
constexpr float WALL_OBST_DELTA   = 15.0f;   // sensor sisi: d < ref - ini = halangan
constexpr float WALL_YAW_MAX_DEG  = 20.0f;   // selisih yaw terhadap arah waypoint; lebih besar = bacaan dinding ditolak
constexpr uint32_t POSE_SAVE_GAP_MS = 10000; // R7: jeda minimum antar tulis posisi
constexpr float POSE_SAVE_CM      = 5.0f;    // R7: tulis hanya bila bergeser > ini sejak tulis terakhir
constexpr float POSE_SAVE_DEG     = 5.0f;
constexpr float US_AGREE_CM = 3.0f;   // R1: median valid bila minimal 1 sampel lain berselisih <= ini; zig-zag (crosstalk/echo ganda) = tidak valid
constexpr float US_TREND_STEP_CM = 12.0f;  // R1b: bacaan mendekat/menjauh mulus (urutan waktu monoton, tiap langkah <= ini) tetap valid; robot 100 cm/s x 60 ms = 6 cm/sampel

// ---------------- IMU-assist: medan, slip, stuck, auto-tune PID yaw (PRD IMU) ----------------
// --- Terrain & Yaw ---
constexpr float    PITCH_SLOPE_THRESHOLD = 5.0f;    // derajat, deteksi tanjakan
constexpr float    ROLL_TILT_THRESHOLD   = 5.0f;    // derajat, deteksi miring samping
constexpr float    YAW_TOLERANCE         = 3.0f;    // derajat, toleransi lurus
constexpr float    KP_YAW_FLAT           = 1.0f;    // gain koreksi di medan datar
constexpr float    KP_YAW_SLOPE          = 1.8f;    // gain koreksi di tanjakan / miring (lebih agresif)
constexpr int      PWM_MIN               = 20;      // PWM minimum agar motor tidak mati saat koreksi (apply_kinematics, F5)
constexpr int      PWM_MAX               = 200;     // batas atas PWM
constexpr float    STUCK_YAW_ERR         = 10.0f;   // derajat, ambang deteksi stuck
constexpr uint32_t STUCK_TIME_MS         = 1000;    // ms, durasi sebelum dianggap stuck
constexpr float    STUCK_GYRO_MIN        = 2.0f;    // deg/s, di bawah ini badan dianggap tidak berputar
constexpr float    STUCK_ENC_MIN         = 1.0f;    // cm/s, kecepatan badan menurut encoder di bawah ini = tidak maju
constexpr float    IMU_FILTER_ALPHA      = 0.2f;    // low-pass untuk pitch/roll/yaw/gyro_z
// --- Slip detection & compensation ---
constexpr float    SLIP_YAW_THRESHOLD    = 8.0f;    // derajat, selisih yaw encoder vs IMU
constexpr float    SLIP_GYRO_MIN         = 2.0f;    // deg/s, minimal gyro_z agar dianggap benar-benar berputar
constexpr uint32_t SLIP_TIME_MS          = 500;     // ms, durasi konsisten sebelum slip dinyatakan
constexpr float    SLIP_TRUST_DECAY      = 0.3f;    // pengurang trust encoder saat slip (per loop)
constexpr float    SLIP_TRUST_RECOVER    = 0.05f;   // penambah trust encoder saat normal (per loop)
constexpr float    ENCODER_TRUST_INIT    = 1.0f;
// --- Slope / ramp handling (PRD A.5.4). Values are the PRD proposals, not yet measured on the robot. ---
constexpr float    ENCODER_TRUST_FREEZE  = 0.5f;    // TODO(tune-§7) encoder trust below this (on slope / slip) -> odometry x,y frozen
constexpr float    COS_CLAMP_MIN         = 0.5f;    // TODO(tune-§7) lower bound for cos(pitch)/cos(roll) in slope projection (~60 deg)
constexpr float    SLOPE_SPEED_FACTOR    = 0.6f;    // TODO(tune-§7) translation speed factor while terrain != FLAT
constexpr float    SLOPE_DESCENT_FACTOR  = 0.5f;    // TODO(tune-§7) stricter factor while descending (wheels outrun the body)
constexpr float    RAMP_SNAP_TOL_CM      = 20.0f;   // TODO(tune-§7) snap pose to a known ramp point only if within this distance
// --- PID yaw auto-tuning ---
constexpr float    PID_KP_INIT           = 1.0f;    // 1.0 = gain HEAD_KP/HEAD_KI/HEAD_KD apa adanya; auto-tune menskalakannya
constexpr float    PID_KI_INIT           = 0.0f;    // tambahan di atas HEAD_KI
constexpr float    PID_KD_INIT           = 0.1f;    // 0.1 = HEAD_KD; skala kd = kd / PID_KD_INIT
constexpr float    PID_OUT_MIN           = -(float)PWM_MAX;
constexpr float    PID_OUT_MAX           = (float)PWM_MAX;
constexpr uint32_t PID_TUNE_WINDOW_MS    = 3000;    // jendela observasi untuk evaluasi performa
constexpr float    PID_OSCILLATION_AMP   = 5.0f;    // derajat, amplitudo osilasi dianggap tidak stabil
constexpr float    PID_STEADY_ERR        = 2.0f;    // derajat, error dianggap steady
constexpr float    PID_KP_STEP           = 0.1f;
constexpr float    PID_KD_STEP           = 0.02f;
constexpr int      PID_MAX_ITER          = 10;      // batas iterasi tuning per sesi (sejak menyala)

// ---- Lapangan v2 ----   (PRD v2.1: perbaikan navigasi hasil uji lapangan; nilai default konservatif)
constexpr float    IMU_PITCH_SIGN        = -1.0f;   // L1: -1 = pitch mentah BNO055 negatif saat hidung naik; hasil imuPitchDeg(): nose-up positif
constexpr float    PITCH_SLOPE_EXIT_DEG  = 3.0f;    // L2: histeresis, keluar SLOPE bila |pitch| < ini (masuk di PITCH_SLOPE_THRESHOLD)
constexpr float    ROLL_TILT_EXIT_DEG    = 3.0f;    // L2: histeresis, keluar TILT bila |roll| < ini
constexpr float    NAV_SLOPE_MIN_RPM     = 40.0f;   // L3: lantai RPM translasi navigasi selama jarak ke target > NAV_SLOW_CM
constexpr float    WALL_REF_MAX_CM       = 100.0f;  // L5: ref dinding hanya direkam bila 5 <= d <= ini
constexpr float    WALL_REF_SPREAD_CM    = 2.0f;    // L5: sebaran 5 sampel median terakhir maksimum
constexpr uint32_t NAV_VERIFY_SETTLE_MS  = 300;     // L6: diam sebelum bacaan sisi dibandingkan dengan ref
constexpr float    WALL_ARRIVE_TOL_CM    = 3.0f;    // L6: |ukur - ref| <= ini = cocok
constexpr uint8_t  NAV_VERIFY_MAX        = 2;       // L6: iterasi koreksi + geser per waypoint
constexpr float    ODOM_SCALE_FWD        = 1.0f;    // L7: skala komponen maju odometri (hasil kalibrasi meteran, lihat README)
constexpr float    ODOM_SCALE_BWD        = 1.0f;    // L7: mundur
constexpr float    ODOM_SCALE_UP         = 1.0f;    // L7: naik ramp (|pitch| >= PITCH_SLOPE_THRESHOLD)
constexpr float    ODOM_SCALE_DOWN       = 1.0f;    // L7: turun ramp
constexpr float    AVOID_SIDE_CMS        = 30.0f;   // L9: kecepatan geser saat menghindar rintangan depan (dipercepat dari 15)
constexpr uint32_t AVOID_HOLD_MS         = 1500;    // L9: koreksi pose dinding ditahan selama ini sesudah AVOID
constexpr uint32_t AVOID_MAX_MS          = 20000;   // L9: total AVOID per leg lebih dari ini = batal (deny 7)
// PRD avoid v1.0: bacaan ultrasonik palsu saat jalan, hindar bertahap CLEAR -> HOLD -> SHIFT -> PASS
constexpr float    AVOID_WATCH_CMS       = 60.0f;   // kecepatan maju lurus saat PASS (melewati halangan)
constexpr uint32_t AVOID_CONFIRM_MS      = 0;       // jeda sebelum geser: 0 = langsung (IR sudah debounce); pembatas percepatan membuat peralihannya melengkung
constexpr float    SIDE_FREE_CM          = 20.0f;   // H4: sisi layak bila > ini
constexpr float    AVOID_EXTRA_CM        = 15.0f;   // H5: geser tambahan setelah depan bebas (lebar badan)
constexpr float    AVOID_SHIFT_MIN_CM    = 50.0f;   // H5: total geser samping minimal dari jalur awal (sebelum maju); dibatasi jarak aman ke sisi
constexpr float    AVOID_WALL_GAP_CM     = 10.0f;   // H5: geser hindar boleh mendekati dinding sampai jarak ini (bacaan ultrasonik sisi); tidak boleh lebih dekat
constexpr float    IR_SIDE_WALL_IGNORE_CM = 25.0f;  // IR: IR sisi diabaikan bila ultrasonik sisi yang sama < ini (itu dinding, bukan halangan)
constexpr float    AVOID_IR_SHIFT_CM     = 30.0f;   // IR: geser minimal saat IR sisi memicu
constexpr float    AVOID_IR_MARGIN_CM    = 10.0f;   // IR: geser tambahan setelah IR bebas IR_CLEAR_MS
constexpr uint32_t IR_CLEAR_MS           = 150;
// Peringatan dini dari ultrasonik depan: bacaan turun terus secara konsisten (sebanding jarak tempuh) = ada yang didekati -> melambat bertahap sampai IR depan mendeteksi.
// Keputusan hindar tetap IR depan; ultrasonik hanya menyiapkan pelambatan (dan jadi cadangan untuk benda yang tidak terlihat IR).
constexpr float    WARN_FAR_CM           = 100.0f;  // tren dipantau di bawah jarak ini
constexpr float    WARN_CRAWL_CM         = 40.0f;   // di jarak ini robot sudah merayap (jangkauan IR + margin)
constexpr float    AVOID_CRAWL_CMS       = 20.0f;   // kecepatan merayap sampai IR mendeteksi
constexpr float    AVOID_BLIND_CM        = 15.0f;   // tren terus sampai jarak ini dan IR diam = benda tak terlihat IR -> nyata
constexpr float    TREND_STEP_CM         = 6.0f;    // jarak tempuh per langkah tren
constexpr uint8_t  TREND_N               = 2;       // langkah lolos berturut-turut = tren sah
constexpr float    TREND_TOL_CM          = 5.0f;    // toleransi |penurunan bacaan - jarak tempuh|
constexpr float    WARN_RISE_CM          = 8.0f;    // bacaan naik sejauh ini dari titik terendah = peringatan batal
constexpr uint32_t WARN_STALL_MS         = 1500;    // bacaan berhenti turun selama ini = peringatan batal (bacaan palsu / menjauh)
constexpr uint32_t WARN_LOST_MS          = 500;     // bacaan tidak valid selama ini = peringatan batal
// Gerak halus saat hindar (hanya saat peringatan/hindar aktif): pembatas percepatan pada vx, vy
constexpr float    AVOID_ACCEL_CMS2      = 120.0f;  // percepatan maksimum
constexpr float    AVOID_BRAKE_CMS2      = 250.0f;  // perlambatan maksimum (IR depan belum aktif)
constexpr float    AVOID_EBRAKE_CMS2     = 600.0f;  // perlambatan saat IR depan aktif (darurat)
constexpr float    AVOID_PASS_CM         = 80.0f;   // H5: maju lurus melewati halangan
constexpr float    SIDE_MIN_CM           = 8.0f;    // L10: jarak sisi minimum
constexpr float    SIDE_BUFFER_CM        = 4.0f;    // L10: dorongan mulai di d < SIDE_MIN_CM + SIDE_BUFFER_CM
constexpr float    SIDE_PUSH_KP          = 1.5f;    // L10: cm/s per cm masuk zona
constexpr float    SIDE_PUSH_MAX_CMS     = 20.0f;
constexpr float cmsToRpm(float v) { return v * 60.0f / (2.0f * (float)M_PI * WHEEL_R_CM); }

// ---------------- Pengaman (robot berhenti dan terkunci sampai Anda menekan Lanjut) ----------------
// Pembanding encoder dan IMU memakai "washout" (high-pass) sehingga tidak terpengaruh drift akselerometer.
constexpr float    SAFE_TAU_S        = 1.0f;    // konstanta waktu washout
constexpr float    SAFE_SLIP_MIN_CMS = 40.0f;   // encoder menunjukkan perubahan kecepatan sebesar ini ...
constexpr float    SAFE_SLIP_RATIO   = 0.35f;   // ... tetapi IMU hanya merasakan < 35% darinya = selip / menabrak
constexpr float    SAFE_SLIP_S       = 0.25f;   // harus bertahan selama ini
constexpr float    SAFE_YAW_MIN_DPS  = 180.0f;  // putaran liar: laju gyro > max(ini, SAFE_YAW_GAIN x perintah + SAFE_YAW_ADD_DPS)
constexpr float    SAFE_YAW_GAIN     = 1.6f;
constexpr float    SAFE_YAW_ADD_DPS  = 60.0f;
constexpr float    SAFE_YAW_ERR_DPS  = 150.0f;  // atau gyro melebihi putaran menurut encoder sebesar ini
constexpr float    SAFE_YAW_S        = 0.12f;
constexpr float    SAFE_STALL_DUTY   = 0.70f;   // motor macet: duty > 70% tetapi RPM < SAFE_STALL_RPM
constexpr float    SAFE_STALL_RPM    = 6.0f;
constexpr float    SAFE_STALL_S      = 0.6f;
constexpr float    SAFE_LIFT_VZ      = 14.0f;   // diangkat: kecepatan vertikal (cm/s, washout) melebihi ini
constexpr float    SAFE_LIFT_ACC     = 350.0f;  // atau lonjakan percepatan vertikal (cm/s^2)
constexpr float    SAFE_CARRY_DPS    = 35.0f;   // dipindah saat roda diam: gyro > ini
constexpr float    SAFE_CARRY_ACC    = 120.0f;  // atau percepatan datar > ini (cm/s^2)
constexpr uint32_t SAFE_GRACE_MS     = 500;     // abaikan pengaman sesaat setelah Lanjut / titik 0
// Pengaman (selip / tabrakan, putaran liar, motor macet, deteksi diangkat) saat robot menyala.
// false = MATI sementara (untuk lokalisasi waypoint). Bisa dinyalakan lagi dari dashboard tanpa upload ulang.
constexpr bool     SAFETY_DEFAULT_ON = false;

// ---------------- Fail-safe ----------------
constexpr uint32_t WEB_FAILSAFE_MS = 300;    // navigasi dari dashboard berhenti bila KEDUA jalur (Wi-Fi dan kabel) diam selama ini
constexpr uint32_t SERIAL_BAUD     = 460800; // jalur kabel (port UART): satu JSON per baris

// ---------------- WiFi AP ----------------
#define WIFI_SSID "Omni3X"
#define WIFI_PASS "omni3x1234"

// ---------------- Hasil tes gerak ----------------
// verdict (vOdom / vImu): 0 tidak ada, 1 OK, 2 TERBALIK, 3 tidak bergerak, 4 melenceng, 5 sinyal lemah
// why (state 3): 1 dibatalkan (STOP/gamepad), 2 komunikasi putus, 3 ada rintangan dekat, 5 robot sedang otonom
struct McResult {
  uint8_t state = 0;                 // 0 idle, 1 jalan, 2 selesai, 3 batal
  uint8_t dir = 0;                   // 1 maju, 2 mundur, 3 kiri, 4 kanan
  uint8_t why = 0, vOdom = 0, vImu = 0;
  bool    yawFromImu = false;
  float   dx = 0, dy = 0;            // geser odometri frame badan (cm): +x kanan, +y depan
  float   ix = 0, iy = 0;            // perubahan kecepatan menurut IMU (cm/s), sumbu robot
  float   dyaw = 0;                  // perubahan arah hadap (derajat)
};

// L6/L8: kejadian tiba terakhir. result: 0 tanpa verifikasi (lewati), 1 cocok, 2 tidak cocok
struct ArrLog {
  uint8_t seq = 0, id = 0, verifyN = 0, result = 0;   // seq naik 1 tiap kejadian tiba
  float   m[3]   = {-1, -1, -1};   // ukur kiri, kanan, depan (cm, -1 = tidak valid)
  float   ref[3] = {0, 0, 0};      // referensi waypoint (0 = tidak ada)
  float   resLat = 0, resFwd = 0;  // residual lateral (+ = robot di kanan ref) dan sepanjang arah (+ = robot di depan ref), cm
};

// ---------------- Telemetri (Core 1 -> Core 0) ----------------
struct Telemetry {
  bool ready = false;        // true setelah task kontrol mengisi pertama kali
  float x = 0, y = 0, yawDeg = 0, vx = 0, vy = 0, wz = 0;
  float rpm[3] = {0}, sp[3] = {0}, dist[3] = {-1, -1, -1};
  uint8_t mode = 0;          // 0 MANUAL, 1 AUTO, 2 STOP, 3 TES GERAK, 4 JALAN DASHBOARD
  int8_t  navTarget = 0;
  int8_t  navRoute = -1;     // rute yang sedang jalan (0..3 = A,B,X,Y), -1 = bukan rute
  uint8_t navStep = 0, navSteps = 0;
  uint8_t jog = 0;           // 1 maju, 2 mundur, 3 kiri, 4 kanan, 5 putar kiri, 6 putar kanan
  float   rpmLimit = SPD_MAN_DEF;   // batas kecepatan aktif (RPM)
  bool    turbo = false;
  bool    armed = false, padConnected = false, imuOk = false;
  char    padType[8] = "-";
  float   batt = -1;
  float   kp[3] = {0}, ki[3] = {0}, kd[3] = {0};   // PID per roda
  uint8_t tune = 0;          // 0 idle, 1 jalan, 2 selesai, 3 gagal
  uint8_t tuneWhy = 0;       // sebab gagal: 1 respons kecil, 2 encoder terbalik, 3 belum mantap, 4 dibatalkan
  float   mvx = 0, mvy = 0;  // kecepatan badan terukur dari encoder (cm/s)
  uint8_t imuAddr = 0, imuCal = 0;   // alamat I2C BNO055, status kalibrasi (bit 7:6 SYS, 5:4 GYR, 3:2 ACC, 1:0 MAG)
  bool    imuNdof = false;
  // pengaman & posisi
  uint8_t safe = 0;          // 0 normal; 1 selip/menabrak, 2 putaran liar, 3 motor macet, 4 terangkat saat motor jalan
  bool    safeOn = true;
  bool    poseLost = false;  // robot terdeteksi diangkat / dipindah / IMU reset: posisi tidak valid sampai titik 0 ditetapkan
  bool    poseRestored = false;   // posisi dipulihkan dari flash saat menyala
  bool    poseDegraded = false;   // A.5.4: posisi tidak pasti (odometri dibekukan / trust rendah) sampai datar dan diam
  uint8_t sat = 100;         // pembatas daya (% kecepatan translasi yang diizinkan)
  float   safeDbg[4] = {0};  // dv encoder, dv IMU (cm/s), laju yaw (deg/s), duty maks (%)
  float   cfg[4] = {SPD_MAN_DEF, SPD_TURBO_DEF, SPD_NAV_DEF, SPD_LOC_DEF};   // kecepatan: manual, turbo, navigasi, jalan dashboard
  // umpan balik penyimpanan flash dan penolakan perintah
  uint8_t svStatus = 0, svWhat = 0; uint16_t svCount = 0;   // 0 belum ada, 1 OK, 2 GAGAL; what: 1 wp,2 pid,3 posisi,4 rute,5 kecepatan
  uint8_t deny = 0, denySeq = 0;     // 1 posisi tidak valid, 2 rute kosong, 3 pengaman, 4 sedang dipakai, 5 posisi tidak pasti (tanjakan)
  uint8_t pst[4] = {0};      // status preset A..D: 0 IDLE, 1 ARMED, 2 RUNNING
  uint8_t navPhase = 0;      // 0 tidak ada, 1 MOVING, 2 ROTATING
  uint8_t navDone = 0;       // naik 1 tiap waypoint tuntas (posisi dan arah sesuai)
  McResult mc;
  // IMU-assist (PRD IMU)
  uint8_t terrain = 0;       // 0 FLAT, 1 SLOPE, 2 TILT
  float   yawErr = 0, pitch = 0, roll = 0;   // derajat
  bool    stuck = false, slip = false;
  float   encTrust = ENCODER_TRUST_INIT;
  float   yKp = PID_KP_INIT, yKi = PID_KI_INIT, yKd = PID_KD_INIT;   // gain PID yaw hasil auto-tune
  uint8_t yStatus = 0;       // 0 TUNING, 1 DONE
  // Koreksi dinding (PRD navigasi R1/R8)
  uint8_t ir = 0;   // bit0 depan, bit1 kiri, bit2 kanan (aktif = ada benda)
  float   usMed[3] = {-1, -1, -1}; bool usOk[3] = {false, false, false};   // median 3 sampel + flag valid (depan, kiri, kanan)
  float   wallRef[2] = {0, 0};       // referensi dinding waypoint aktif: kiri, kanan (cm, 0 = tidak ada)
  uint8_t wallStat = 0;              // 0 tidak dipakai, 1 koreksi aktif, 2 fallback encoder, 3 offset melebihi batas
  uint8_t navRetry = 0;              // percobaan ulang leg yang berjalan
  uint16_t sess[4] = {0};            // statistik sesi: tiba, gagal, retry, koreksi dipakai
  // Lapangan v2 (L8): rantai kecepatan, jarak sisi, tiba terakhir
  float   slopeF = 1.0f;             // faktor tanjakan yang dipakai
  float   avoidScale = 1.0f;         // skala avoidObstacles
  float   effRpm = 0;                // RPM translasi efektif setelah semua pengali
  float   sideMin = -1;              // bacaan sisi terkecil selama rute (-1 = belum ada)
  uint8_t avoid = 0;                 // 0 normal, 1 menghindar
  bool    refStale = false;          // referensi dinding tidak cocok > 1 s: rekam ulang
  ArrLog  arr;
  bool    gripper = false;           // keadaan solenoid gripper
};
void      telemetrySet(const Telemetry& t);
Telemetry telemetryGet();
bool      isPoseDegraded();              // A.5.4 (defined in main.cpp)
extern volatile uint32_t g_hbWifiMs;     // detak terakhir dari WebSocket (Wi-Fi)
extern volatile uint32_t g_hbSerialMs;   // detak terakhir dari kabel (Serial/UART)
