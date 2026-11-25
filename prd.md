# PRD — Perbaikan Navigasi Hasil Uji Lapangan (Tanjakan & Lurus) Omni-3X
Versi: 2.1 · Tanggal: 2026-11-18 · Basis kode: `omni3x` (ESP32-S3, PlatformIO)
Menggantikan `prd.md` v1.0 (R1–R9, wall-correction). Kode R1–R9 sudah ada di repo; PRD ini hanya memperbaiki apa yang ditemukan di lapangan.

Sumber bukti: `Masalahnya/data lapangan.txt`, `Hasil lurus.webm`, `Hasil tanjakan.webm` (rekaman layar dashboard; angka di bawah dibaca dari frame video).

---

## 0. Aturan Wajib untuk Agent AI (baca sebelum mengerjakan)

1. **Maksimal 3 pertanyaan total per fase**, satu batch. Bila tidak menghalangi, jangan bertanya; pakai default di PRD.
2. PRD adalah sumber kebenaran. Bila kode berbeda dari PRD, laporkan sekali di akhir, jangan berhenti.
3. **Baca minimal**: hanya berkas di kolom "Berkas yang disentuh" + referensi langsungnya.
4. **Edit surgikal**: tanpa refactor, tanpa ganti gaya, tanpa "sekalian rapikan".
5. **Satu requirement = satu commit**. Kerjakan urut §7 tanpa menunggu konfirmasi antar-fase.
6. **Verifikasi murah dan wajib**: `host_test/run_host_tests.sh` + `pio run`. Bila `pio run` tidak tersedia di sandbox, laporkan satu baris dan lanjut.
7. Tanpa over-engineering: tidak ada abstraksi/kelas/opsi baru di luar yang didefinisikan PRD. Konstanta baru hanya di `include/config.h`.
8. Satu requirement gagal > 3 kali perbaikan → stop, laporkan blocker.
9. `SAFETY_DEFAULT_ON` tetap `false`. Jangan menyentuh WiFi/OTA/keamanan siber.
10. **Output ke user**: hanya 3 blok — SELESAI / GAGAL-BLOCKED / VERIFIKASI. Maksimal 20 baris, tanpa pembuka/penutup.

---

## 1. Temuan dari Lapangan

### 1.1 Ringkasan kejadian (dari `data lapangan.txt`)

| Uji | Kejadian | Gejala |
|-----|----------|--------|
| Tanjakan | Turun otomatis (0:32–0:50) | Berhenti **offside** dari titik awal |
| Tanjakan | Perbaiki posisi (0:59–1:24) | Ultrasonik **tidak bertindak** |
| Tanjakan | Naik otomatis (1:27–2:15) | **Tidak sampai** titik waypoint asli |
| Lurus | Mundur otomatis (0:22–0:31) | **Offside** dari titik awal |
| Lurus | Maju otomatis (0:48–0:57) | **Kurang maju** dari waypoint |

### 1.2 Akar masalah (urut dari yang paling pasti)

**A. Dashboard selalu "tiba", padahal fisik meleset (semua uji).** Odometri hampir selalu menutup dalam radius tiba (`NAV_ARRIVE_CM` = 5 cm):

| Frame | Posisi dashboard | Waypoint | Selisih menurut firmware |
|-------|------------------|----------|--------------------------|
| lurus 0:31 | (−0.6, 4.1) | WP1 (0, 0) | ±4 cm |
| lurus 0:57 | (−4.9, 334.3) | WP2 (−4.3, 338.4) | ±4 cm |
| tanjakan 0:50 | (2.2, 3.4) | WP1 (0, 0) | ±4 cm |
| tanjakan 2:15 | (−5.3, 476.9) | WP2 (−5.1, 481) | ±4 cm |

Firmware menilai tiba hanya dari odometri encoder. Selip dan skala roda tidak terlihat oleh encoder, jadi robot berhenti di tempat yang salah sambil merasa benar. Jadi masalahnya bukan di pengendali gerak, melainkan **tidak ada pengukur posisi nyata**. `wallCalc()` hanya mengoreksi komponen **lateral**; kesalahan **sepanjang arah jalan** (kurang maju / mundur melenceng) tidak punya sensor acuan sama sekali.

**B. Tanda pitch terbalik → logika tanjakan salah arah (bug, keyakinan tinggi).** Robot selalu menghadap +y. Pitch terbaca **−13.5°** saat naik (1:40) dan **−14.3°** saat turun (0:40): tanda sama untuk dua arah karena pitch = kemiringan badan, dan nose-up ternyata **negatif** pada IMU ini. Kode `main.cpp` (blok A.5.4 R4) memakai `imuf.pitch * vy < 0` sebagai "turun" dengan asumsi nose-up positif. Akibatnya: saat **naik** dikira turun (faktor 0.5), saat **turun** dikira naik (faktor 0.6, padahal seharusnya paling ketat). Turunan jadi terlalu cepat, roda mendahului badan → offside.

**C. Kecepatan tanjakan runtuh karena pengali bertumpuk.** Frame 1:40: batas nav 100 RPM, tetapi target roda hanya **13 RPM** (≈ 7 cm/s). `satScale` (min 0.35) × faktor tanjakan (0.5) × skala `avoidObstacles` (min 0.3) dikalikan semua: 100 × 0.35 × 0.5 ≈ 17 RPM. Grafik RPM sebelum frame itu penuh lonjakan berhenti-jalan. Sisa 328 cm dengan 7 cm/s ≈ 48 detik, cocok dengan catatan "1:27–2:15". Setelah semua pengali, lantai `NAV_V_MIN_CMS` (6 cm/s) juga tidak lagi berlaku (diterapkan sebelum pengali) sehingga mendekati titik tujuan roda bisa di bawah batas gerak motor.

**D. Baseline pitch salah, margin ke ambang tipis.** `pitchOff` hanya diambil sekali saat boot (`imu.cpp`) dan "Set titik 0" tidak menyentuhnya. Di lantai datar pitch terbaca **−4.2°** (0:13), −3.5°, −3.1°; ambang `PITCH_SLOPE_THRESHOLD` = 5° hanya 0.8° di atasnya, tanpa histeresis. Di tepi ramp dan saat bergetar, status FLAT↔SLOPE berkedip → faktor kecepatan dan proyeksi odometri ikut berkedip.

**E. Referensi dinding tidak masuk akal di area uji tanjakan.** Nilai ref yang tersimpan: kiri 30 dan kanan 221 (WP1), kiri 152 (WP2). Bacaan di titik yang sama berubah-ubah: kiri 122 cm di WP1 (0:13), 376 cm (0:50), 146 cm di WP2 (2:15). Ref ≥ 150 cm itu bukan "dinding koridor", dan bacaan HC-SR04 jauh sangat berisik. Status di lapangan: "koreksi aktif" dengan ref 30 (0:40) tapi "offset melebihi batas" saat ref 152 vs ukur 9 (1:40). Koreksi pose dari ref yang salah bisa menggeser estimasi hingga `WALL_CORR_MAX_CM` (20 cm) ke arah yang salah. Tidak ada pengecekan bahwa robot memang berada di tempat waypoint direkam.

**F. Ultrasonik di tanjakan tidak bisa dipercaya dan tidak digerbangi.** Di ramp: kiri mentah **3 cm** dan 10 cm (merah, "rintangan"), depan 37–53 cm padahal tidak ada benda, sementara median kiri 22 cm. Tidak ada gerbang terrain di `wallCalc` maupun `avoidObstacles` (yang memakai nilai mentah `dist[]`).

**G. "Ultrasonik tidak bertindak" saat perbaiki posisi (0:59–1:24) memang sesuai desain, bukan bug.** Header dashboard saat itu "Mode Manual" dan kartu Koreksi Dinding "tidak dipakai": koreksi dinding hanya aktif di `navActive()` + fase MOVING (v1 §5.3). Ini yang tidak diinginkan pengguna di lapangan.

### 1.3 Yang tidak bisa dipastikan dari video

- Besar selisih **fisik** (tidak ada meteran di rekaman). Karena itu semua AC lapangan di §8 memakai meteran, dan L8 menambah log tiba untuk mengukurnya.
- Apakah penyebab dominan "kurang maju" adalah selip awal, skala roda, atau transien berhenti. Prosedur L7 memisahkannya.
- Arah kemiringan ramp diasumsikan naik ke +y (WP2 = "atas"), sesuai catatan pengguna.

---

### 1.4 Keputusan pengguna (mengikat, menimpa usulan awal PRD bila bentrok)

1. Ultrasonik **depan selalu aktif** sebagai penghindar rintangan, di semua terrain.
2. Ultrasonik **kiri/kanan menjaga jarak sisi**: bacaan sisi tidak boleh sampai < **8 cm**.
3. Perilaku hindar: depan terdeteksi halangan dan kiri lebih dekat → robot geser ke **kanan**; depan sudah tidak terdeteksi → robot **maju** dan kembali menuju waypoint berikutnya.
4. Ultrasonik kiri/kanan juga **penentu posisi waypoint**: contoh WP2 di x = 200, y = 10 dengan jarak kiri 20 cm → saat tiba, bacaan kiri harus sama dengan data waypoint (20 cm).

## 2. Tujuan

1. Robot berhenti di titik yang benar **secara fisik**, bukan hanya menurut odometri.
2. Tanjakan: kecepatan stabil, tidak runtuh, turunan terkendali.
3. Ultrasonik depan selalu menghindar; kiri/kanan menjaga jarak sisi ≥ 8 cm dan memastikan posisi di waypoint cocok dengan data waypoint.
4. Setiap kegagalan terukur di telemetri (tidak perlu menebak dari video).

## 3. Non-Tujuan

- Path planning, refactor `main.cpp`, ganti sensor, ganti wiring (TRIG tetap bersama).
- Menembak ultrasonik bergantian (mustahil dengan TRIG bersama; usulan `WALL_STAGGER` di v1 dibatalkan).
- Mengubah `SAFETY_DEFAULT_ON`.

---

## 4. Daftar Requirement

Konstanta baru di `include/config.h`, bagian `// ---- Lapangan v2 ----`.

### L1 — Perbaiki tanda pitch (akar B)
- Tambah `IMU_PITCH_SIGN = -1.0f` (default berdasarkan bukti lapangan). `imuPitchDeg()` mengembalikan `IMU_PITCH_SIGN * wrap180(pitchRaw - pitchOff)` sehingga **nose-up positif**, sesuai komentar dan logika `main.cpp`.
- Perbarui host test yang mengasumsikan tanda lama; tambah tes: naik (vy>0, nose-up) → `SLOPE_SPEED_FACTOR`; turun (vy<0, nose-up) → `SLOPE_DESCENT_FACTOR`.
- **AC**: di dashboard pitch positif saat hidung naik; turun ramp memakai faktor 0.5, naik 0.6. Host test hijau.
- **Berkas**: `src/imu.cpp`, `include/config.h`, `host_test/test_ramp.cpp` (dan tes lain yang gagal karena tanda).

### L2 — Baseline pitch/roll + histeresis (akar D)
- Pada `rq.zero` ("Set titik 0"): bila robot diam dan `|pitch| < PITCH_SLOPE_THRESHOLD` dan `|roll| < ROLL_TILT_THRESHOLD`, ambil ulang `pitchOff/rollOff` (fungsi baru `imuLevel()`). Di luar kondisi itu: tidak diubah.
- Histeresis terrain: masuk SLOPE bila `|pitch| ≥ PITCH_SLOPE_THRESHOLD`, keluar bila `|pitch| < PITCH_SLOPE_EXIT_DEG` (default **3.0**). Berlaku juga untuk TILT dengan `ROLL_TILT_EXIT_DEG` (default 3.0).
- **AC**: setelah Set titik 0 di lantai datar, pitch/roll ≈ 0 ±0.5°. Host test: sinyal pitch 4.8–5.2° yang berosilasi tidak membuat FLAT↔SLOPE berkedip.
- **Berkas**: `src/imu.cpp`, `src/imu.h`, `src/imuassist.cpp`, `src/main.cpp` (handler `rq.zero`), `include/config.h`.

### L3 — Kecepatan tanjakan tidak runtuh (akar C)
- Pengali gabungan translasi = `min(satScale, faktorTanjakan)` (bukan perkalian keduanya).
- Lantai kecepatan: setelah **semua** pengali (satScale, tanjakan, `avoidObstacles`, trust), kecepatan translasi navigasi tidak turun di bawah `NAV_V_MIN_CMS`; dan selama jarak ke target > `NAV_SLOW_CM` tidak turun di bawah `NAV_SLOPE_MIN_RPM` (default **40**), kecuali ada rintangan terkonfirmasi (jarak ≤ `OBST_STOP_CM`).
- **AC**: host test rantai pengali (kasus: sat 0.35 + lereng → ≥ 40 RPM; dekat target → ≥ `NAV_V_MIN_CMS`; rintangan dekat → berhenti/menghindar tetap jalan). Lapangan: naik ramp 328 cm ≤ 30 s.
- **Berkas**: `src/main.cpp` (blok pembatas daya + faktor tanjakan), `include/config.h`, `host_test/test_ramp.cpp`.

### L4 — Bacaan ultrasonik layak di ramp (akar F)
- Sensor **depan selalu aktif di semua terrain** (FLAT/SLOPE/TILT), tidak digerbangi. Untuk menekan bacaan palsu di ramp (depan 37–53 cm tanpa benda, kiri mentah 3 cm), keputusan menghindar memakai **median valid** (`usOk[]`) alih-alih `dist[]` mentah, di semua terrain.
- Sensor sisi: pitch memutar badan terhadap sumbu lateral, jadi sensor sisi tetap horizontal; yang mengubah geometri sisi hanya **roll**. Koreksi pose/steering dinding ditahan (stat 2) hanya saat terrain = TILT. Di SLOPE koreksi dinding **tetap aktif**.
- **AC**: host test (bacaan mentah 3 cm tunggal tidak menggeser vektor; median valid 3 cm menggeser; SLOPE: depan tetap menghindar dan koreksi dinding jalan; TILT: stat 2). Telemetri `wstat` = 2 hanya saat TILT.
- **Berkas**: `src/navigation.cpp`, `src/navigation.h`, `src/main.cpp` (panggilan), `host_test/test_navigation.cpp`.

### L5 — Validasi referensi dinding saat rekam (akar E)
- Simpan `wallL/wallR` hanya bila: 5 ≤ d ≤ `WALL_REF_MAX_CM` (default **100**), 5 sampel median terakhir menyebar ≤ `WALL_REF_SPREAD_CM` (default **2**), terrain FLAT, dan robot diam. Selain itu simpan 0 (= tidak ada).
- Berlaku di semua jalur rekam (`wp_here`, `stop_save`, simpan-diam di `main.cpp`). `wp_set` manual tidak dibatasi.
- Pada leg MOVING, bila ukur dan ref selisih > `WALL_CORR_MAX_CM` selama > 1 s → stat 3 tetap, **dan** pose tidak disentuh (sudah demikian); tambahkan flag `refStale` di telemetri agar UI menandai "referensi tidak cocok, rekam ulang".
- **AC**: rekam dengan dinding 221 cm → ref tersimpan 0. Rekam dinding 30 cm stabil → tersimpan. Host test untuk ketiga syarat.
- **Berkas**: `src/webui.cpp`, `src/main.cpp` (saveId), `src/navigation.cpp`, `include/config.h`, `host_test/test_navigation.cpp`.

### L6 — Tiba harus cocok dengan data waypoint (sensor sisi = penentu posisi) (akar A, G)
- Prinsip: waypoint = (x, y, yaw, wallL, wallR[, wallF]). Contoh: WP2 di x = 200, y = 10 dengan wallL = 20 cm → saat robot dinyatakan tiba, sensor kiri harus membaca 20 ± `WALL_ARRIVE_TOL_CM` (default **3**). Odometri saja tidak cukup untuk dinyatakan tiba.
- Fase **VERIFY** dijalankan setelah posisi masuk radius tiba dan, bila waypoint punya yaw, **setelah ROTATING selesai** (karena `wallCalc` menolak bacaan bila yaw menyimpang > `WALL_YAW_MAX_DEG` dari yaw waypoint). Motor translasi 0 selama `NAV_VERIFY_SETTLE_MS` (default **300**), lalu median valid sisi dibandingkan dengan ref (depan vs `wallF` bila ada).
- Cocok (|selisih| ≤ tol): DONE. Tidak cocok dan ≤ `WALL_CORR_MAX_CM`: koreksi `px,py` sebesar residual, lalu robot **bergeser halus sampai bacaan = ref**, maks `NAV_VERIFY_MAX` (default **2**) iterasi. Tetap tidak cocok: selesai dengan penanda "tidak cocok" di log `arr` (tidak membatalkan rute). Sensor tidak valid atau ref 0: lewati VERIFY seperti perilaku sekarang.
- Waypoint perantara (tanpa berhenti): koreksi lateral tetap lewat `wallCalc` yang ada.
- **Referensi depan** `wallF` (opsional): field baru `Waypoint`, disimpan dengan aturan L5, migrasi NVS `wp16z` → `wp16f` sekali (pola sama dengan migrasi sebelumnya; `wallF` lama = 0). Tanpa `wallF`, koreksi sepanjang arah tidak ada (dilaporkan di telemetri); itu batas fisik bila tidak ada dinding depan ≤ 100 cm.
- Tombol dashboard **"Sinkron ke WP n"** (perintah `wp_sync`): VERIFY manual saat robot diam ≤ 30 cm dari WP n. Ini jawaban untuk "perbaiki posisi" yang sebelumnya tidak dibantu ultrasonik.
- **AC**: host test VERIFY (cocok → DONE; selisih kecil → geser lalu DONE; selisih > batas → ditolak; jatah habis → selesai dengan flag; sensor tidak valid → lewati; VERIFY berjalan setelah ROTATING). Migrasi `wp16z`→`wp16f`. Perilaku tanpa ref tidak berubah. Lapangan: contoh WP2 (ref kiri 20 cm) → bacaan kiri saat tiba 20 ± 3.
- **Berkas**: `src/navigation.cpp/.h`, `src/main.cpp`, `src/webui.cpp`, `tools/monitor.html` (+ `pack_monitor.py`), `include/config.h`, `host_test/test_navigation.cpp`.

### L7 — Kalibrasi odometri per arah & ramp (prosedur + 2 konstanta)
- Konstanta baru `ODOM_SCALE_FWD`, `ODOM_SCALE_BWD` (default **1.0**), dipakai di `navOdom` pada komponen `by` sesuai tanda. Opsional serupa untuk ramp: `ODOM_SCALE_UP`, `ODOM_SCALE_DOWN` (default 1.0).
- Prosedur ukur (hasil dicatat di tabel README §Kalibrasi lapangan): 3 kali per kondisi pada 300 cm: lantai maju, lantai mundur, ramp naik, ramp turun, geser kiri/kanan. Faktor = jarak meteran ÷ jarak dashboard (rata-rata). Ubah konstanta bila menyimpang > 3% (lantai) / 5% (ramp).
- Catatan: `WHEEL_R_CM` 4.88 hasil koreksi ×1.63 dari 3.0 (README) menunjukkan skala encoder sangat jauh dari nominal. Periksa `ENC_CPR` (3860, edge RISING phase A) terhadap datasheet; bila hitungan sebenarnya berbeda, `WHEEL_R_CM` sedang menutupi kesalahan itu dan skala geser/maju bisa berbeda.
- **AC**: tabel hasil ukur ada; galat odometri vs meteran ≤ 3% lantai, ≤ 5% ramp di uji ulang.
- **Berkas**: `src/navigation.cpp`, `include/config.h`, `README.md`.

### L8 — Telemetri diagnostik (agar tidak menebak dari video)
- Tambah JSON: `sf` (faktor tanjakan yang dipakai), `as` (skala `avoidObstacles`), `re` (RPM efektif nav setelah semua pengali), `smin` (bacaan sisi terkecil selama leg), `av` (0 normal, 1 menghindar), `arr` (kejadian tiba: id WP, ukur L/R/F vs ref, residual lateral & sepanjang arah, jumlah VERIFY), `rst` (`refStale`).
- `tools/monitor.html`: kartu "Kecepatan efektif" (rantai: batas → sat → lereng → hindar → hasil) dan baris "Tiba terakhir" di dekat kartu Koreksi Dinding; dipak ulang `pack_monitor.py`.
- **AC**: angka berubah saat naik/turun ramp; log tiba tampil setelah tiap waypoint selesai; build UI sukses.
- **Berkas**: `src/webui.cpp`, `src/main.cpp` (isi telemetri), `include/config.h` (struct `Telemetry`), `tools/monitor.html`, `tools/pack_monitor.py`.

### L9 — Hindar rintangan depan dengan geser ke sisi lapang (keputusan 1, 3)
- Pemicu: median depan < `OBST_DIST_CM + OBST_SPD_GAIN × v` (rumus sekarang), di semua terrain (L4).
- Perilaku (status AVOID di dalam MOVING, bukan fase baru): saat depan terhalang, komponen maju menuju rintangan = 0 dan robot **bergeser menjauhi sisi yang lebih dekat** (kiri lebih dekat → geser kanan; kanan lebih dekat → geser kiri). Bila kedua sisi tidak valid atau selisihnya < 3 cm: geser ke sisi yang searah tujuan waypoint. Bila kedua sisi < `SIDE_MIN_CM + SIDE_BUFFER_CM`: berhenti dan tunggu. Kecepatan geser `AVOID_SIDE_CMS` (default **15**).
- Depan bebas selama `AVOID_CLEAR_MS` (default **300**) → keluar AVOID dan robot **maju**; vektor menuju waypoint berikutnya dihitung dari pose, sehingga robot kembali ke garis tujuan. Jarak sisi dekat rintangan dijaga L10.
- Selama AVOID dan `AVOID_HOLD_MS` (default **1500**) sesudahnya: koreksi pose dinding ditahan (stat 4 "menghindar"), karena koreksi itu memaksa pose lateral = ref dan menghapus pergeseran yang disengaja; deteksi stuck dinonaktifkan; timer leg diperpanjang sebesar lama AVOID.
- Batas `AVOID_MAX_MS` (default **20000**): terhalang lebih lama → navigasi dibatalkan dengan alasan baru (deny 7 "terhalang").
- **AC**: host test: depan terhalang + kiri lebih dekat → `vx > 0` (kanan), komponen maju 0; depan bebas → `vy > 0` menuju waypoint; kedua sisi sempit → berhenti; melewati `AVOID_MAX_MS` → cancel deny 7; koreksi dinding tertahan selama AVOID. Lapangan: rintangan di jalur, robot menghindar tanpa menyentuh dan tiba di waypoint berikutnya (cek meteran dan log `arr`).
- **Berkas**: `src/navigation.cpp/.h`, `src/main.cpp`, `include/config.h`, `host_test/test_navigation.cpp`.

### L10 — Jaga jarak sisi ≥ 8 cm (keputusan 2)
- Konstanta: `SIDE_MIN_CM` = **8**, `SIDE_BUFFER_CM` = **4** (dorongan mulai di < 12 cm), `SIDE_PUSH_KP` = **1.5** (cm/s per cm masuk zona), `SIDE_PUSH_MAX_CMS` = **20**.
- Mode otonom (MOVING dan AVOID), semua terrain, memakai median valid: bila sisi < `SIDE_MIN_CM + SIDE_BUFFER_CM`, tambah `vx` menjauhi sisi itu = `SIDE_PUSH_KP × (12 − d)` (dibatasi `SIDE_PUSH_MAX_CMS`); bila d < `SIDE_MIN_CM`, komponen gerak menuju sisi itu dihapus. Prioritas di atas koreksi dinding dan arah waypoint.
- Kedua sisi masuk zona: komponen lateral 0, maju pelan (tidak di bawah `NAV_V_MIN_CMS`); bila depan juga terhalang → berhenti.
- Manual dan jog dashboard tidak diubah.
- **AC**: host test: d = 10 → dorong menjauh; d = 20 → tanpa efek; d = 6 → dorongan maksimum; kedua sisi sempit; sisi tidak valid → tanpa efek. Lapangan: `smin` ≥ 8 cm sepanjang rute.
- **Berkas**: `src/navigation.cpp/.h`, `src/main.cpp`, `include/config.h`, `host_test/test_navigation.cpp`.

---

## 5. Konstanta Baru (ringkasan, semua default konservatif)

| Konstanta | Default | Requirement |
|-----------|---------|-------------|
| `IMU_PITCH_SIGN` | −1.0 | L1 |
| `PITCH_SLOPE_EXIT_DEG`, `ROLL_TILT_EXIT_DEG` | 3.0, 3.0 | L2 |
| `NAV_SLOPE_MIN_RPM` | 40 | L3 |
| `WALL_REF_MAX_CM`, `WALL_REF_SPREAD_CM` | 100, 2 | L5 |
| `NAV_VERIFY_SETTLE_MS`, `WALL_ARRIVE_TOL_CM`, `NAV_VERIFY_MAX` | 300, 3, 2 | L6 |
| `AVOID_SIDE_CMS`, `AVOID_CLEAR_MS`, `AVOID_HOLD_MS`, `AVOID_MAX_MS` | 15, 300, 1500, 20000 | L9 |
| `SIDE_MIN_CM`, `SIDE_BUFFER_CM`, `SIDE_PUSH_KP`, `SIDE_PUSH_MAX_CMS` | 8, 4, 1.5, 20 | L10 |
| `ODOM_SCALE_FWD/BWD/UP/DOWN` | 1.0 | L7 |

## 6. Risiko & Mitigasi

| Risiko | Mitigasi |
|--------|----------|
| Tanda pitch berbeda bila IMU dipasang ulang | `IMU_PITCH_SIGN` satu konstanta; dashboard menampilkan pitch hasil koreksi, cek nose-up = positif |
| Re-baseline di ramp mengacaukan pitch | Hanya jika `|pitch| < ambang` dan diam (L2) |
| Menaikkan kecepatan tanjakan membuat roda macet | Lantai 40 RPM dapat diturunkan di config; pembatas `satScale` tetap jalan lewat `min()` |
| VERIFY membuat robot bolak-balik | Maks 2 kali per waypoint, toleransi 3 cm, lalu selesai dengan flag |
| Depan selalu aktif memicu hindar palsu di ramp | Keputusan memakai median valid (L4), `AVOID_CLEAR_MS`, batas `AVOID_MAX_MS`; pantau `av` di telemetri |
| Zona sisi 12 cm membuat robot ditolak di koridor sempit | Konstanta `SIDE_*` bisa diturunkan di config; kedua sisi sempit = maju pelan, bukan berhenti |
| Tidak ada dinding depan ≤ 100 cm | Tidak ada koreksi sepanjang arah; andalkan L7; dilaporkan di `arr` |
| Migrasi NVS `wp16f` gagal | Pola migrasi sama dengan `wp16y`→`wp16z`; wall/wallF lama = 0; host test migrasi |

## 7. Urutan Implementasi (agent: ikuti ini, jangan bertanya)

| Fase | Requirement | Alasan urutan |
|------|-------------|---------------|
| 1 | L1, L2, L8 | Bug tanda + baseline dulu; telemetri jadi alat ukur fase berikutnya |
| 2 | L3, L4, L9, L10 | Kestabilan tanjakan, ultrasonik di ramp, hindar rintangan, jarak sisi |
| 3 | L5, L6 | Referensi valid dulu, baru tiba harus cocok dengan data waypoint |
| 4 | L7 | Kalibrasi lapangan memakai telemetri L8 |

## 8. Kriteria Terima Keseluruhan

1. `host_test/run_host_tests.sh` hijau (tes lama + baru); `pio run` sukses tanpa warning baru.
2. **Ulang uji lurus** (maju–mundur–maju, 300 cm, ukur meteran): selisih fisik dari waypoint ≤ 5 cm pada 3 dari 3 pengulangan.
3. **Ulang uji tanjakan** (turun–naik, ramp yang sama): selisih fisik ≤ 10 cm; naik ≤ 30 s; kecepatan target roda tidak turun di bawah 40 RPM selama pendakian.
4. Dashboard: pitch positif saat hidung naik; kartu Koreksi Dinding menunjukkan status 2 hanya saat TILT dan alasan bila "tidak dipakai".
5. Rekam waypoint dengan dinding > 100 cm: ref tersimpan 0, UI menandai "tanpa referensi".
6. "Sinkron ke WP n" dari posisi bergeser ≤ 30 cm memperbaiki posisi (bila dinding valid), terlihat di log `arr`.
7. Setelah reboot: ref dinding, `wallF`, dan ramp mask bertahan.
8. **Hindar rintangan**: benda di jalur depan, kiri lebih dekat → robot geser kanan; depan bebas → maju dan tiba di waypoint berikutnya, tanpa menyentuh benda. Berlaku juga di ramp.
9. **Jarak sisi**: selama rute otonom `smin` tidak pernah < 8 cm.
10. **Cocok dengan data waypoint**: di waypoint dengan ref kiri 20 cm, bacaan kiri saat tiba 20 ± 3 cm (log `arr`).
