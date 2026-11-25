# Omni-3X – Single Controller (PlatformIO / ESP32-S3)

Build: `pio run -t upload` · Monitor: `pio device monitor` (pakai port **UART**, bukan port USB native, 460800 baud).
Dashboard: sambung WiFi `Omni3X` / `omni3x1234` → http://192.168.4.1/

## Struktur
| File | Isi |
|---|---|
| `include/config.h` | Pin, mekanik, gain, batas keamanan (semua tuning di sini) |
| `src/main.cpp` | Task kontrol Core 1 (mode, tombol, kecepatan, heading lock, pembatas daya, fail-safe) |
| `src/safety.*` | Pengaman: roda selip / menabrak, putaran liar, motor macet, deteksi robot diangkat atau dipindah |
| `src/settings.*` | Kecepatan manual / turbo / navigasi / jalan dashboard (disimpan di flash) |
| `src/store.*` | Penyimpanan flash satu pintu: tulis lalu baca ulang untuk verifikasi |
| `src/motors.*` | PWM dual-channel, encoder ISR, PID + anti-windup, auto-tune step response, kinematika |
| `src/imu.*` | BNO055 (I2C 0x28/0x29 otomatis, 100 kHz): yaw dari fusion chip + percepatan linear, sambung ulang otomatis |
| `src/imuassist.*` | IMU sebagai wasit: terrain FLAT/SLOPE/TILT, gain yaw dinamis, deteksi selip roda + `encoder_trust`, stuck, auto-tune PID yaw (konstanta di `config.h`, bagian IMU-assist) |
| `src/movecheck.*` | Tes gerak maju/mundur/kiri/kanan: gerak pendek, dinilai dari odometri encoder dan IMU |
| `src/ultrasonic.*` | 3 sensor, 1 TRIG bersama |
| `src/navigation.*` | Odometri, 16 waypoint (x, y, yaw), 4 preset/rute, state navigasi MOVING → ROTATING → DONE, jog manual, posisi terakhir, obstacle deflection |
| `src/gamepad_usb.*` | USB Host: XInput + HID generik (parser report descriptor) |
| `src/webui.*`, `include/monitor_html.h` | AP WiFi, HTTP:80, WebSocket:81, link kabel (Serial JSON), dashboard |
| `tools/monitor.html` | Sumber dashboard (satu halaman). Setelah diedit: `python tools/pack_monitor.py` membuat `include/monitor_html.h`. `extract_monitor.py` arah sebaliknya |
| `host_test/` | Tes host tanpa ESP32 (kinematika, navigasi, imuassist, settings; file firmware disertakan langsung): `host_test/run_host_tests.sh` |

## Kontrol gamepad
Stick kiri/D-Pad = gerak (vx, vy) · stick kanan = putar · **tahan R1 = turbo** · **A, B, X, Y = Preset A, B, C, D** (dua langkah, lihat bawah) · **tahan B 0,5 s = STOP darurat** · gerak stick atau B saat otonom = batal (override).
Tombol lain diabaikan, kecuali dua fungsi pemulihan: **Start** = Lanjut saat robot terkunci pengaman, **Back** = titik 0 (posisi dan arah).

## Dashboard (satu halaman, tanpa tab)
* **Kontrol manual**: tombol Maju, Mundur, Kiri, Kanan, Putar kiri, Putar kanan. **Tahan = jalan, lepas = berhenti.** Keyboard identik: `W A S D` (gerak), `Q E` (putar), `Spasi` = STOP. Slider kecepatan manual 30 sampai 500 RPM (dikirim saat dilepas, tersimpan di flash; dipakai gamepad dan tombol layar).
* **Rekam waypoint**: selama robot digerakkan manual, dashboard mencatat jejak (X, Y, Yaw) dan menggambarnya di peta. Tombol **STOP** di panel (atau Spasi) berhenti lalu, bila "simpan otomatis saat STOP" aktif dan robot baru digerakkan manual, menyimpan posisi dan arah ke slot terpilih (WP1 sampai WP16, WP1 sampai 4 = preset A sampai D). STOP di bilah atas selalu hanya berhenti.
* **Preset A sampai D**: status per tombol IDLE / JALAN. Preset i menjalankan rute i (default A = WP1, B = WP2, C = WP3, D = WP4; rute bisa beberapa waypoint, diubah di Pengaturan lanjutan). Tombol **Jalankan** per rute juga ada di tabel rute.
  1. `IDLE` → tekan sekali: `RUNNING` langsung (tanpa tahap siap).
  2. Di tiap waypoint yang punya yaw: robot berhenti, memutar ke yaw waypoint (ROTATING), lalu lanjut ke waypoint berikutnya. Waypoint tanpa yaw hanya dilewati.
  3. `RUNNING` → waypoint terakhir tiba dan yaw sesuai: `IDLE`.
  4. STOP (tombol, Spasi, B ditahan, stick digerakkan) saat `RUNNING`: batal, kembali `IDLE`.
  Preset ditolak bila posisi tidak valid, pengaman terkunci, atau rute kosong; alasannya tampil di Kejadian.
* **Pengaturan lanjutan** (di bagian bawah halaman, bisa dibuka): rute tiap preset, edit waypoint (x, y, yaw), tes gerak, kecepatan turbo/navigasi, PID.

## Navigasi waypoint: posisi lalu arah hadap
Waypoint menyimpan x, y, **yaw** (derajat). Di waypoint terakhir sebuah rute: **MOVING** (menuju x, y dengan heading lock; jarak ≤ `NAV_ARRIVE_CM` 5 cm) → motor berhenti → **ROTATING** (spin di tempat, PID yaw dari `HEAD_*`; galat dinormalisasi −180..+180, jadi 359° ke 1° = 2°; selesai bila galat ≤ `NAV_YAW_TOL_DEG` 3° dan laju putar < `NAV_YAW_RATE_OK_DPS`, bertahan `NAV_YAW_SETTLE_MS`; galat difilter `NAV_YAW_LPF` agar jitter IMU tidak mengganggu) → **DONE** (dashboard menerima notifikasi, preset kembali IDLE). Waypoint tanpa yaw (kolom yaw kosong, atau data lama yang dimigrasi) berhenti di posisi saja. Tanpa IMU, rotasi dilewati/dibatalkan. Batas waktu memutar `NAV_ROT_TIMEOUT_MS`. Waypoint perantara dilewati tanpa berhenti dan tanpa memutar.

## Kecepatan (RPM roda)
| Mode | Default | Catatan |
|---|---|---|
| Manual normal | 200 | stick penuh |
| Manual turbo | 500 | tahan R1, hanya manual |
| Navigasi otomatis | 200 | goto dan rute |
| Jalan dashboard (rekam waypoint) | 150 | tombol jalan-terus |

Semua bisa diubah di dashboard (kartu **Kecepatan**, 30 sampai 500 RPM) dan tersimpan di flash. Batas keras `MAX_WHEEL_RPM = 500` di `config.h`. Pada 500 RPM encoder 3860 CPR menghasilkan sekitar 32 ribu interupsi per detik per roda; ISR memakai baca register langsung agar ringan. Bila RPM terbaca tidak wajar di kecepatan tinggi, turunkan batas turbo. Putaran badan = `ROT_FRAC` x (kecepatan / `ROBOT_L_CM`).

## Titik 0, waypoint, rute
* **Titik 0 hanya ditetapkan oleh Anda**: tombol **Back** di gamepad atau **Set titik 0** di dashboard. Itu menetapkan posisi (0, 0) dan arah hadap robot saat itu sebagai acuan dunia. Waypoint disimpan dalam acuan ini, jadi saat merekam dan saat menjalankan, letakkan robot di titik 0 dengan arah hadap yang sama.
* **Posisi terakhir** (x, y, yaw) disimpan ke flash tiap kali robot diam setelah bergerak, dan dipulihkan saat robot dinyalakan. Itu benar hanya bila robot tidak dipindah saat mati; dashboard menampilkan peringatan "Posisi dipulihkan dari memori".
* **Robot diangkat atau dipindah**: IMU mendeteksi lonjakan atau kecepatan vertikal (`SAFE_LIFT_*`), atau gerak / putaran saat roda diam (`SAFE_CARRY_*`). Posisi ditandai **tidak valid**, navigasi otomatis dibatalkan dan ditolak sampai Anda menetapkan titik 0 lagi. Chip IMU yang ter-reset juga menandai posisi tidak valid. Robot tidak bisa tahu posisi barunya sendiri (tidak ada sensor posisi absolut), jadi titik 0 ditetapkan ulang oleh Anda.
* **16 waypoint** (`MAX_WP`). **4 rute** (A, B, X, Y), tiap rute sampai 16 waypoint berurutan, mis. `1,2,3`. Waypoint perantara dilewati tanpa berhenti (radius `NAV_PASS_CM`), robot melambat dan berhenti di waypoint terakhir (`NAV_ARRIVE_CM`). Waypoint kosong di tengah rute dilewati. Default: rute A = WP1, B = WP2, X = WP3, Y = WP4.
* **Jalan manual dan rekam waypoint**: lihat bagian Dashboard di atas (tahan = jalan, lepas = berhenti, heading lock aktif saat translasi). Jalan dashboard otomatis berhenti bila stick digerakkan, pengaman aktif, atau dashboard berhenti mengirim detak (300 ms). Menyimpan waypoint (STOP + simpan, **Di sini**) ditolak saat posisi tidak valid; dashboard menampilkan alasannya.
* Perintah JSON: `{"cmd":"preset","i":0}` (0..3 = A..D, dua langkah) · `{"cmd":"wp_set","id":1,"x":60,"y":40,"yaw":90}` (`noyaw:true` = tanpa arah) · `{"cmd":"jog","d":1}` (0 berhenti, 1 maju, 2 mundur, 3 kiri, 4 kanan, 5 putar kiri, 6 putar kanan) · `{"cmd":"stop_save","id":3}` · `{"cmd":"route_set","r":0,"ids":[1,2,3]}` · `{"cmd":"route_go","r":0}` · `{"cmd":"odom_reset"}` · `{"cmd":"speed","man":200,"turbo":500,"nav":200,"loc":150}`.

## Koreksi dinding dan kalibrasi slope (PRD navigasi)

Waypoint menyimpan jarak dinding kiri/kanan (median ultrasonik) saat disimpan lewat `wp_here` / STOP dan Simpan; `wp_set` menerima `wallL`/`wallR` manual (0 = tidak ada). Saat otonom MOVING, selisih terhadap referensi dikonversi ke koreksi pose halus dan dorongan steering; di luar `WALL_CORR_MAX_CM` atau sensor tidak valid, encoder murni dipakai. Status tampil di kartu "Koreksi dinding". Timeout atau stuck: pendekatan diulang `NAV_RETRY` kali, lalu batal (alasan 6). Tanda ramp (`ramp_mark`) tersimpan di `nav/ramp`.

Kalibrasi slope (R9, uji lapangan, belum dijalankan): jalankan tiap uji 3 kali di ramp nyata, isi tabel, lalu ubah konstanta `TODO(tune-§7)` di `config.h` bila hasil menyimpang > 30%.

| Konstanta | Nilai awal | Hasil ukur | Nilai baru |
|---|---|---|---|
| `ENCODER_TRUST_FREEZE` | 0.5 | | |
| `COS_CLAMP_MIN` | 0.5 | | |
| `SLOPE_SPEED_FACTOR` | 0.6 | | |
| `SLOPE_DESCENT_FACTOR` | 0.5 | | |
| `RAMP_SNAP_TOL_CM` | 20 | | |

## Lapangan v2 (PRD v2.1)
* **Pitch**: `imuPitchDeg()` nose-up **positif** (`IMU_PITCH_SIGN = -1`). Naik ramp = faktor `SLOPE_SPEED_FACTOR` (0.6), turun = `SLOPE_DESCENT_FACTOR` (0.5). Terrain punya histeresis (masuk 5°, keluar 3°). **Set titik 0** di lantai datar dan diam mengambil ulang baseline pitch/roll.
* **Kecepatan**: pengali translasi = `min(satScale, faktor tanjakan)`; lantai `NAV_V_MIN_CMS` dan (jarak > `NAV_SLOW_CM`) `NAV_SLOPE_MIN_RPM` setelah semua pengali, kecuali rintangan terkonfirmasi (<= `OBST_STOP_CM`) atau AVOID.
* **Hindar rintangan bertahap (PRD avoid v1.0)**: bacaan depan yang dekat tidak menghentikan robot, hanya membatasi kecepatan (`AVOID_WATCH_CMS` bila < `FAR_WATCH_CM`). Halangan dianggap nyata bila bacaan turun sebanding jarak tempuh (`TREND_STEP_CM` x `TREND_N`) atau <= `OBST_STOP_CM`. Lalu **HOLD** (diam `AVOID_CONFIRM_MS`; benda pergi = lanjut), pilih sisi yang lebih lapang (> `SIDE_FREE_CM`), **SHIFT** berpulsa tanpa maju, lalu **PASS** maju lurus `AVOID_PASS_CM`. Kedua sisi sempit: diam, batal setelah `AVOID_MAX_MS` (alasan 7). Telemetri `av`: 0 normal, 1 HOLD, 2 SHIFT, 3 PASS. Sisi dijaga >= `SIDE_MIN_CM` (8 cm). Koreksi dinding ditahan saat TILT (stat 2) dan saat AVOID (stat 4); SLOPE tetap aktif.
* **Referensi dinding** (kiri, kanan, depan) hanya direkam bila 5..100 cm, stabil (5 sampel, sebaran <= 2 cm), datar, dan robot diam; selain itu 0. `wp_set` manual tidak dibatasi. Data NVS: `nav/wp16f` (migrasi otomatis dari `wp16z`).
* **VERIFY**: di waypoint tempat robot berhenti (setelah ROTATING) dan punya ref, bacaan sisi/depan harus cocok dengan data waypoint +- `WALL_ARRIVE_TOL_CM`; bila tidak, pose dikoreksi lalu robot bergeser halus (maks `NAV_VERIFY_MAX`). Tombol **Sinkron ke WP n** (`{"cmd":"wp_sync","id":n}`) menjalankan VERIFY manual (robot diam <= 30 cm dari WP).
* **Telemetri baru**: `slf` (faktor tanjakan; `sf` tetap status pengaman), `as`, `re`, `smin`, `av`, `rst`, `arr` {q seq, id, m ukur K/Ka/D, r ref, rl, rf residual, n verifikasi, v 0 tanpa/1 cocok/2 tidak cocok}.

### Kalibrasi lapangan (L7) — belum diukur
Ukur 3 kali per kondisi pada 300 cm dengan meteran; faktor = jarak meteran / jarak dashboard (rata-rata). Ubah `ODOM_SCALE_*` di `config.h` bila menyimpang > 3% (lantai) / 5% (ramp). Periksa juga `ENC_CPR` (3860, edge RISING phase A) terhadap datasheet: `WHEEL_R_CM` 4.88 (koreksi x1.63 dari 3.0) mungkin menutupi kesalahan hitungan encoder.

| Kondisi | Meteran (cm) | Dashboard (cm) | Faktor | Konstanta |
|---|---|---|---|---|
| Lantai maju | | | | `ODOM_SCALE_FWD` |
| Lantai mundur | | | | `ODOM_SCALE_BWD` |
| Ramp naik | | | | `ODOM_SCALE_UP` |
| Ramp turun | | | | `ODOM_SCALE_DOWN` |
| Geser kiri / kanan | | | | (belum ada konstanta) |

## Gripper (1 solenoid)
* Pin `PIN_GRIPPER` (default GPIO 7, lewat MOSFET / relay + dioda flyback), `GRIPPER_ACTIVE_HIGH`, `GRIPPER_ACTUATE_MS` di `config.h`. Awal boot = OFF.
* **Langkah gripper di dalam rute/preset**: `G1` = ON (jepit), `G0` = OFF (lepas), mis. `1,2,G1,3,G0`. Kode di firmware/protokol: 101 = ON, 102 = OFF (`route_set` ids). Robot berhenti di waypoint sebelum langkah gripper, diam `GRIPPER_ACTUATE_MS`, lalu lanjut. Rute harus punya minimal satu waypoint valid.
* Manual: `{"cmd":"grip","on":true|false}` (tombol Jepit / Lepas di dashboard dan app). Telemetri `gp` = keadaan solenoid, `np` = 4 saat langkah gripper berjalan.
* Keadaan solenoid bertahan saat rute dibatalkan atau Stop (benda tidak jatuh); gunakan tombol Lepas. Cek rating duty solenoid: dialiri lama bisa panas.

## Pengaman
Robot **berhenti dan terkunci** sampai Anda menekan **Lanjut** (dashboard, atau tombol Start di gamepad). Dashboard menampilkan sebabnya:
1. **Roda berputar kencang tetapi IMU tidak merasakan gerak** (menabrak atau selip): perubahan kecepatan menurut encoder > `SAFE_SLIP_MIN_CMS` sedangkan IMU hanya merasakan < `SAFE_SLIP_RATIO` darinya selama `SAFE_SLIP_S`. Kedua sisi dibandingkan lewat washout (high-pass `SAFE_TAU_S`) sehingga drift akselerometer tidak berpengaruh.
2. **Putaran liar**: laju gyro melebihi `max(SAFE_YAW_MIN_DPS, SAFE_YAW_GAIN x perintah + SAFE_YAW_ADD_DPS)` atau melebihi putaran menurut encoder sebesar `SAFE_YAW_ERR_DPS`.
3. **Motor macet**: duty > `SAFE_STALL_DUTY` tetapi RPM < `SAFE_STALL_RPM` selama `SAFE_STALL_S`.
4. **Terangkat saat motor jalan**.

**Saat ini pengaman MATI secara default** (`SAFETY_DEFAULT_ON = false` di `config.h`) agar lokalisasi waypoint tidak terganggu: tidak ada penguncian dan deteksi diangkat/dipindah ikut mati. Nyalakan dari kotak di kartu Pengaman, atau ubah konstanta itu ke `true` bila ingin aktif sejak robot menyala. Semua batas ada di `config.h` (`SAFE_*`). Pengaman butuh IMU hidup untuk deteksi 1, 2, 4. Matikan pengaman (kotak di kartu Pengaman) saat roda diangkat untuk tes motor atau auto-tune. Bila robot berhenti tanpa sebab, mungkin tanda sumbu IMU belum benar (cek kartu Tes gerak) atau batas terlalu ketat.

**Kestabilan lurus (tanjakan)**: heading lock memakai PID sudut dengan suku D dari gyro (`HEAD_*`, integral lebih kuat). **Pembatas daya**: bila ada roda yang duty-nya jenuh (> 95 %), kecepatan translasi diturunkan bertahap (sampai 35 %) supaya roda yang lemah bisa mengikuti dan robot tidak melenceng; pulih sendiri. Dashboard menampilkan persentasenya. Bila satu motor memang jauh lebih lemah (rusak, gearbox seret, baterai drop), perangkat lunak hanya bisa mengurangi gejalanya.

## Penyimpanan flash (NVS)
Waypoint (`nav/wp16y`, x, y, yaw; format lama `wp16` dimigrasi otomatis tanpa yaw), rute (`nav/rt`), posisi terakhir (`nav/pose`), PID (`pid2/all`), dan kecepatan (`cfg/spd`) ditulis lewat `store.cpp`, yang **membaca ulang dan membandingkan** setelah menulis. Hasilnya muncul di dashboard (Kejadian terakhir: "Tersimpan ke flash: waypoint" atau "GAGAL menyimpan") dan di serial (`[NVS] tulis ... OK/GAGAL`). Saat boot, serial mencetak apa yang dimuat (`[NAV] dimuat dari flash ...`, `[PID] dimuat dari flash`, `[CFG] ...`). Data format lama (8 waypoint, PID kunci `kp0..kd2`) dimigrasi otomatis.
Bila data hilang setiap kali program di-upload, penyebabnya biasanya flash terhapus saat upload (jangan pilih Erase Flash). Bila dashboard bilang "GAGAL", partisi NVS bermasalah.

## Monitoring ganda: Wi-Fi + kabel
Dua jalur membawa telemetri dan perintah JSON yang sama.
* **Wi-Fi**: `http://192.168.4.1/` (WebSocket :81/ws).
* **Kabel**: port **UART** ke PC, 460800 baud, satu JSON per baris. Buka `tools/monitor.html` dari PC dengan Chrome atau Edge (bukan lewat 192.168.4.1: Web Serial butuh konteks aman), tekan **Sambung kabel**, pilih port COM. Bila PC juga tersambung ke Wi-Fi Omni3X, kedua jalur aktif sekaligus.
* Dashboard memakai data kabel selama hidup, otomatis pindah ke Wi-Fi bila kabel mati. Perintah dikirim lewat kabel (cadangan Wi-Fi). Tombol STOP dikirim ke kedua jalur.
* Fail-safe 300 ms: navigasi dari dashboard berhenti hanya bila **kedua** jalur diam. Satu jalur putus tidak menghentikan robot.
* Telemetri lewat kabel hanya mengalir bila dashboard mengirim detak (hb). `pio device monitor` biasa tetap bersih.
* Satu port hanya bisa dipakai satu program: tutup `pio device monitor` sebelum menyambung kabel di dashboard. Membuka port bisa me-reset ESP32, jadi sambungkan sebelum robot bergerak.

## PID per roda
Tiap roda punya Kp/Ki/Kd sendiri, disimpan di flash (namespace `pid2`, kunci `all`, lihat Penyimpanan flash). Satuan gain: **% PWM per RPM**, sama dengan sketch `test_motorPID` (nilai bisa disalin langsung). Struktur PID juga sama dengan sketch: P + I (anti-windup) + D terfilter, tanpa feed-forward.
**Auto-tune** memakai metode sketch: roda diputar open-loop 50% PWM (`AT_STEP_PCT`), direkam sampai mantap, lalu diidentifikasi model K, tau, L dan dihitung PID (`AT_METHOD`: 0 Ziegler-Nichols, 1 Cohen-Coon, 2 IMC, default IMC). Angkat roda dulu. Hasil tersimpan per roda; "semua roda" menjalankan roda 1 sampai 3 berurutan. Bila gagal, dashboard menampilkan sebabnya (respons kecil, encoder terbalik, belum mantap, dibatalkan); ringkasan model dicetak ke Serial (`[TUNE] ...`).
Dashboard: kartu "PID per roda" (simpan per roda atau semua, auto-tune per roda atau semua). Nilai yang sedang diketik tidak lagi tertimpa telemetri.
Perintah JSON: `{"cmd":"pid","m":0,"kp":0.5,"ki":0.5,"kd":0}` (m = 0..2, -1 = semua) · `{"cmd":"autotune","m":-1}`.

## Catatan keputusan desain (hal yang tidak cocok di PRD)
* **Ultrasonik**: PRD hanya memberi 4 GPIO (13–16) untuk 3 sensor → 1 TRIG bersama + 3 ECHO. Sensor menembak bersamaan (crosstalk kecil karena beda 120°). Echo HC-SR04 = 5 V → wajib pembagi tegangan.
* **Baterai**: semua pin ADC1 dipakai motor, ADC2 tidak jalan bersama WiFi → `PIN_BATT_ADC = -1` (nonaktif). Isi pin bila Anda memakai ADC eksternal.
* **Tombol B** di PRD berarti rute B sekaligus STOP: tap singkat (saat siaga) = rute B, tahan ≥ 0,5 s = STOP, tekan saat otonom = batal.
* **Override < 10 ms**: callback USB membatalkan otonom & membangunkan task kontrol langsung (jalur cepat feed-forward), tidak menunggu tick 20 ms.
* **Fail-safe 300 ms**: gamepad lepas (event USB, instan) dan WebSocket (detak 100 ms dari dashboard) → `stopAll`.
* **Posisi** = odometri encoder + yaw gyro; akan drift (slip roda, drift gyro). Titik 0 ditetapkan dengan tombol Back atau dashboard.
* **Kalibrasi odometri (`WHEEL_R_CM`)**: nilai ini harus **radius efektif** roda, bukan radius nominal. Uji: set titik 0, jalan lurus beberapa meter tanpa berbelok, lalu bandingkan `posisi` dashboard dengan meteran; nilai baru = `WHEEL_R_CM` lama x (jarak nyata / jarak odometri). Contoh nyata: 296,2 cm di GUI vs 482 cm meteran → 3,0 x 482/296,2 = 4,88 cm (roda omni ~98 mm). Saat nilai ini diubah, waypoint dan posisi tersimpan dari firmware lama dikonversi sekali secara otomatis (penanda `nav/sver`); setelah itu set titik 0 ulang.
* **IMU BNO055**: default mode IMU (`IMU_USE_NDOF = false`): yaw relatif, kebal magnet motor, drift kecil. NDOF memberi yaw kompas tapi terganggu magnet motor. BNO055 menghitung yaw sendiri; heading lock memakai yaw itu. Wiring: VIN ke **3V3** (bukan 5 V), SDA GPIO8, SCL GPIO9, PS0/PS1 ke GND. Kalibrasi tidak disimpan ke flash: diamkan robot beberapa detik (GYR), tahan diam di beberapa posisi (ACC). Status kalibrasi tampil di dashboard.
* **Gamepad D-Input**: pemetaan tombol di `HID_BTN_MAP` (gamepad_usb.cpp), sesuaikan bila A/B/X/Y tertukar.
* Arah gerak: `KIN_Y_SIGN` (maju/mundur) dan `KIN_W_SIGN` (putaran) di `config.h` mengoreksi kinematika bila arah gerak badan terbalik walau tanda motor/encoder sudah benar.

## Tes gerak (dashboard, kartu "Tes gerak")
Tombol Maju, Mundur, Kiri, Kanan (dan "Tes 4 arah berurutan"). Robot diam 0,4 dtk, jalan 1,5 dtk pada `TEST_V_CMS` (20 cm/s), lalu berhenti dan dinilai. Letakkan robot di lantai dengan ruang kosong.
* **Odometri**: geser frame badan dari encoder (kinematika maju). **IMU**: perubahan kecepatan dari percepatan linear BNO055, bias diukur saat diam. Hasil per arah: OK, TERBALIK, tidak bergerak, melenceng, sinyal lemah.
* Diagnosa otomatis: odometri terbalik tapi IMU OK → `ENC_SIGN`; keduanya terbalik → `MOTOR_SIGN` dan `ENC_SIGN` semua roda; odometri OK tapi IMU terbalik → `IMU_ACC_FWD_SIGN`, `IMU_ACC_RIGHT_SIGN`, `IMU_ACC_FWD_AXIS`.
* Kotak tengah menampilkan arah gerak terukur saat ini (dari encoder), aktif juga saat gamepad dipakai.
* Batal otomatis: STOP, stick/tombol B, ada rintangan < `OBST_DIST_CM` saat mulai, dashboard berhenti mengirim detak (fail-safe 300 ms). Tes ditolak saat mode otonom. Heading lock dimatikan selama tes supaya putaran liar (kolom Putar) terlihat.
* Perintah JSON: `{"cmd":"mtest","dir":1}` (1 maju, 2 mundur, 3 kiri, 4 kanan). Mode telemetri 3 = tes gerak.

## Urutan commissioning (ikuti Phase 1–5 PRD)
1. Cek `[USB] Gamepad ... tersambung` di serial. 2. Cek `ENC_SIGN` / `MOTOR_SIGN` (roda terangkat, dorong stick), lalu auto-tune.
3. Cek arah yaw IMU (`IMU_YAW_SIGN`) dan tanda kinematika dengan kartu **Tes gerak** di dashboard. 4. Isi `WHEEL_R_CM`, `ROBOT_L_CM`, `ENC_CPR`, `MAX_WHEEL_RPM` sesuai robot.
