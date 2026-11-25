# PRD — OmniMonitor: APK Monitoring & Kontrol Omni-3X
Versi 1.0 · Tanggal: 2026-11-18 · Basis kode: `omni3x` (ESP32-S3, PlatformIO). **Firmware tidak diubah.**

---

## 0. Aturan Wajib untuk Agent AI (baca sebelum mengerjakan)

Aturan ini mengikat setiap agent yang mengerjakan PRD ini. Pelanggaran = pekerjaan ulang.

1. **Maksimal 3 pertanyaan total per fase.** Kumpulkan semua keraguan jadi satu batch; bila tidak ada yang menghalangi, pilih default yang tertulis di PRD.
2. **Tidak ada pertanyaan untuk yang sudah tertulis di PRD.** Bila protokol firmware berbeda dari tabel §5, laporkan sekali di akhir, jangan berhenti.
3. **Baca minimal**: `src/webui.cpp` (protokol), `tools/monitor.html` (kartu UI & peta label konstanta), `README.md`. Repo app diisolasi; dilarang menjelajah firmware lainnya.
4. **Dilarang mengubah firmware omni3x.** Semua lewat protokol §5 yang sudah ada. Field/perintah yang ternyata tidak ada → laporkan, jangan tambah.
5. **Satu requirement = satu commit**, urutan sesuai §7 tanpa menunggu konfirmasi antar-fase.
6. **Verifikasi murah dan wajib**: `./gradlew assembleDebug` hijau + uji dengan `tools/mock_ws.py` (R11) berjalan di PC dan app tersambung ke host HP yang sama. Ringkasan akhir maksimal 20 baris.
7. **Dependency tetap** (dilarang menambah): Kotlin, OkHttp 4, kotlinx-coroutines, ViewBinding. UI = layout XML + custom View, tanpa Compose.
8. **Dilarang over-engineering**: tanpa DI framework, tanpa abstraksi berlapis, tanpa opsi konfigurasi di luar R11. Komentar kode hanya bila logika tidak jelas.
9. **Batas token**: satu requirement melewati 3× percobaan gagal → stop dan laporkan blocker.
10. Output ke user hanya: apa yang selesai, apa gagal/blocked, hasil verifikasi.

---

## 1. Latar Belakang & Masalah

- Dashboard robot saat ini halaman web (`tools/monitor.html`) yang nyaman di laptop, tetapi kurang di HP: harus buka browser lewat `192.168.4.1`, tanpa ikon app, tanpa layar penuh, dan jalur kabel (Web Serial) tidak tersedia di Android.
- Operator lapangan membawa HP. Dibutuhkan: memonitor posisi, sensor, pengaman, koreksi dinding, dan menjalankan kontrol (STOP, jog, preset, rekam waypoint, PID) langsung dari HP tanpa PC.
- Protokol JSON dua arah (WebSocket `ws://192.168.4.1:81/ws`) sudah stabil dan mencakup semua itu — tidak perlu perubahan firmware.

## 2. Tujuan

1. APK Android (Kotlin) yang tersambung ke AP robot `Omni3X` dan **menduplikasi seluruh fungsi dashboard web** (full paritas).
2. Monitoring lengkap: peta posisi + jejak, motor, sensor, pengaman, koreksi dinding, IMU-assist, kejadian — semua dari telemetri `webui.cpp`.
3. Kontrol lengkap: STOP, jog, preset/rute, rekam & edit waypoint, titik 0, tes gerak, kecepatan, PID per roda, auto-tune.
4. Sambungan andal: detak 100 ms, pulih sendiri saat koneksi putus, indikator latensi.

## 3. Non-Tujuan (Di Luar Lingkup)

- Mengubah firmware `omni3x` (protokol, PID, kinematika, safety tetap).
- Fitur visi/kamera (tinggal di proyek OmniVision terpisah), iOS, keamanan siber/HTTPS.
- Jalur kabel USB-OTG di app (dipertimbangkan PRD lanjutan; firmware jalur serial sudah ada).
- Logika navigasi/baru di HP; app hanya menampilkan dan meneruskan perintah.

## 4. Teknologi & Arsitektur

- **Android native Kotlin**, minSdk 26, target SDK terbaru; Gradle Kotlin DSL; proyek baru terpisah dari repo firmware (mis. `Documents/Test/omni/OmniApp`).
- Nama app **Omni-3X Monitor**, id `com.omni.monitor`, tema gelap mengikuti sistem (persis palet CSS `monitor.html`).
- Struktur (pola OmniVision: satu Activity per layar + modul `.kt` per domain):

| File | Isi |
|---|---|
| `MainActivity.kt` | Beranda: semua kartu monitor + kontrol manual + preset + waypoint |
| `AdvancedActivity.kt` | Pengaturan lanjutan: rute preset, edit waypoint, tes gerak, kecepatan, PID/auto-tune, pengaturan app (R11) |
| `WsLink.kt` | WebSocket, detak `hb` 100 ms, reconnect, ukur latensi |
| `RobotState.kt` | Parse telemetri JSON → snapshot state; kirim perintah; callback per pembaruan |
| `MapView.kt` | Peta posisi (custom View: grid, waypoint, jejak, pose) |
| `RpmChart.kt` | Grafik RPM 3 roda 30 dtk terakhir (custom View) |

- State parsing & render peta/grafik berjalan di coroutine IO/UI terpisah; UI thread hanya menggambar.

## 5. Protokol (sumber kebenaran: `src/webui.cpp`)

- WebSocket **text**, satu pesan = satu JSON. Robot **broadcast telemetri tiap 100 ms** selama ada klien WS.
- App mengirim `{"cmd":"hb"}` tiap 100 ms untuk menjaga detak. Failsafe robot 300 ms: app adalah satu-satunya jalur aktif → **koneksi putus = robot berhenti sendiri ≤ 300 ms** (aman, tanpa aksi app).
- Alamat bawaan: host `192.168.4.1`, WS path `/ws`, port 81 (override di R11).

### 5.1 Perintah app → robot (`handleMsg`, `webui.cpp:24-59`)

| cmd | Payload | Efek |
|---|---|---|
| `hb` | — | Detak (wajib tiap 100 ms) |
| `stop` | — | Berhenti/batal otonom & jog |
| `jog` | `d`: 0 diam, 1 maju, 2 mundur, 3 kiri, 4 kanan, 5 putar kiri, 6 putar kanan | Gerak manual (tahan = jalan) |
| `speed` | `man`,`turbo`,`nav`,`loc` (opsional) | Batas RPM (30–500, disimpan flash) |
| `odom_reset` | — | Titik 0: posisi + arah sekarang |
| `goto` | `id` | Jalan ke 1 waypoint |
| `preset` | `i` 0..3 (A–D) | Jalankan rute preset (2 langkah) |
| `route_go` | `r` | Jalankan rute |
| `route_set` | `r`, `ids[]` | Isi rute |
| `wp_here` | `id` | Simpan posisi+arah sekarang sebagai WP (+ referensi dinding otomatis) |
| `wp_set` | `id`,`x`,`y`[,`yaw`|`noyaw`][,`wallL`,`wallR`] | Tulis WP (absen yaw = pertahankan) |
| `wp_del` | `id` | Hapus WP |
| `stop_save` | `id` | Berhenti lalu simpan WP |
| `ramp_mark` | `id`,`on` | Tanda titik ramp |
| `safety` | `on` | Aktif/matikan pengaman |
| `resume` | — | Buka kunci pengaman (Lanjut) |
| `mtest` | `dir` 1..4 | Tes gerak maju/mundur/kiri/kanan |
| `pid` | `m` 0..2/-1, `kp`,`ki`,`kd` | Simpan PID roda ke flash |
| `autotune` | `m` 0..2/-1 | Auto-tune PID roda |

### 5.2 Telemetri robot → app (`buildTelemetry`, `webui.cpp:69-105`; 10 Hz)

| Kelompok | Field | Isi |
|---|---|---|
| Pose & gerak | `x y yaw vx vy wz mvx mvy` | cm, derajat, cm/s, rad/s |
| Mode & nav | `mode tgt spd tb arm rt rs rn np nd jg rty ps[4] pl pr sat` | mode, target, RPM limit, turbo, arm, rute/step, fase nav, jog, retry, status preset, posisi hilang/dipulihkan |
| Motor | `rpm[3] sp[3] d[3] sa[3] kp[3] ki[3] kd[3]` | RPM, target RPM, jarak, sudut sensor, PID roda |
| Ultrasonik | `dm[3] dv[3]` | median cm + valid |
| Koreksi dinding | `wl wr wlr wrr wstat` | terukur & referensi kiri/kanan, status (0–3) |
| IMU | `imu ia ic in terrain pitch roll yaw_error encoder_trust stuck slip pose_degraded pid{kp ki kd status}` | status, kalibrasi, medan, selip, PID yaw |
| Pengaman | `sf so sfv[4] cs[4]` | sebab, aktif, debug |
| Waypoint & rute | `wp[16][7] rts[4][]` | valid,x,y,yaw,hasYaw,wallL,wallR; isi rute |
| Sistem | `pad pt bat tune tw sv[3] dn[2] ss[4] mc{...}` | gamepad, baterai, tune, NVS tersimpan, alasan ditolak, statistik, tes gerak |

- **Peta label** (teks Indonesia) ikut konstanta `monitor.html:203-206`: `MODES, TUNE, TW, SF, DENY, SVW, JOGN, WHY` — app menampilkan teks yang sama, jangan menerjemahkan ulang.

## 6. Daftar Requirement

Tiap requirement: deskripsi + kriteria terima (AC). Palet warna & gaya = CSS `monitor.html` (gelap/sistem).

### R1 — Proyek & kerangka app
- Proyek Gradle Kotlin sesuai §4: dua Activity, tema gelap, ikon app, manifest tanpa izin berbahaya (hanya `INTERNET`, `ACCESS_NETWORK_STATE`).
- Layar Beranda menampilkan kerangka kartu kosong; tanpa koneksi menampilkan pesan "belum tersambung".
- **AC**: `assembleDebug` hijau; app terpasang di HP ≥ Android 8 dan menampilkan kerangka.

### R2 — Koneksi WebSocket & detak
- `WsLink`: connect `ws://<host>:81/ws`, kirim `hb` tiap 100 ms, auto-reconnect mundur (mulai 1 dtk), ukur latensi (RTT pesan terkirim → balasan telemetri).
- Status koneksi: chip "WiFi: menyambung/terhubung/putus" + latensi ms + ikon robot, persis bilah atas dashboard.
- Deteksi tidak terhubung ke AP: bila host tak terjangkau 5 dtk, tampil instruksi "sambungkan WiFi Omni3X (sandianya omni3x1234)". App **tidak membaca SSID** (tanpa izin lokasi) dan tidak mengubah WiFi HP.
- **AC**: putus sambungan saat robot jalan → robot berhenti ≤ 300 ms (failsafe firmware); app pulih sendiri tanpa buka ulang saat AP kembali.

### R3 — Model telemetri & pengirim perintah
- `RobotState`: parse semua field §5.2 (field tidak ada = nilai default, tanpa crash); snapshot dibagikan ke UI; antrian perintah JSON.
- Peta label konstanta `monitor.html` dimuat (MODES, DENY, dst.).
- **AC**: mock server (R11) memancarkan telemetri lengkap + field acak → semua kartu terisi; JSON rusak diabaikan tanpa crash; perintah dikirim sesuai §5.1 (terverifikasi log mock).

### R4 — Beranda: status, kejadian & latar
- Bilah atas: nama app, chip koneksi/latensi/mode/gamepad, tombol **STOP** besar (kirim `stop`, warna merah, mudah dijangkau jempol).
- Banner koneksi hilang (semua kartu meredup) dan alert pengaman (`sf`≠0 → tampil teks `SF` + tombol **Lanjut** (`resume`)).
- Kartu **Kejadian**: daftar event seperti dashboard (deny `dn`, alasan; "Tersimpan ke flash: …" dari `sv`; tiba/fail preset dari `ps`/`nd`; retry `rty`); terbaru di atas, maks 50 baris.
- **AC**: alarm pengaman muncul saat `sf` berubah; Lanjut membuka kunci (robot nyata); event log konsisten dengan dashboard web yang dibuka bersamaan.

### R5 — Peta posisi
- `MapView` kanvas: pose robot (x, y, yaw), 16 waypoint (nomor + titik, WP ber-yaw beda tanda), jejak (trail) saat digerakkan manual, skala otomatis mengikuti bounding trail+waypoint.
- Tombol: **Set titik 0** (`odom_reset`), hapus jejak. Peringatan `pl` (posisi tidak valid) dan `pr` (dipulihkan dari memori) tampil di bawah peta.
- **AC**: peta identik orientasi dashboard (x kanan, y atas); digeser dekat dinding → posisi berubah halus (koreksi dinding R3 `wstat`).

### R6 — Monitor motor & koreksi dinding
- Tiga kartu motor: RPM besar, target, status (persis `monitor.html:477`).
- Grafik RPM 30 dtk (`RpmChart`, 3 warna, garis tipis target, batas 500).
- Kartu **Koreksi dinding**: `wl/wr` terukur, `wlr/wrr` referensi, status `wstat` (0 tidak dipakai, 1 aktif, 2 fallback, 3 melebihi batas) + retry `rty` + statistik `ss`.
- **AC**: grafik menggambar mulus ≥ 30 dtk tanpa kebocoran memori; angka berubah saat robot digeser dekat/menjauh dinding.

### R7 — IMU, ultrasonik & pengaman
- Kartu **IMU BNO055**: alamat, kalibrasi, mode, pitch/roll, terrain.
- Kartu **Ultrasonik**: 3 sensor median + valid (999/absen = "tak terbaca"), urutan Depan/Kiri/Kanan.
- Kartu **Pengaman**: checkbox aktif (`safety`), chip sebab (`SF`), detail `sfv`.
- Kartu **Medan · selip · PID yaw**: terrain, `yaw_error`, `stuck`, `slip`, `encoder_trust`, `pose_degraded`, PID yaw + status auto-tune.
- **AC**: semua nilai hidup dari telemetri; toggle pengaman tersimpan di flash (verifikasi serial `[NVS]`).

### R8 — Kontrol manual & rekam waypoint
- Pad jog 7 tombol (persis dashboard): tahan = kirim `jog d` berulang, lepas = `jog 0`; statik `jg` tampil ("diam/maju/…").
- Slider kecepatan manual 30–500 RPM (`speed.man`, dikirim saat lepas).
- "Simpan sebagai" pilih WP1–16 + checkbox simpan otomatis saat STOP + tombol **STOP dan simpan posisi** (`stop_save`); ditolak → tampil alasan `DENY`.
- **AC**: perilaku tahan-lepas tanpa tombol lengket (lifecycle pause = kirim jog 0); rekam WP via app terbaca di dashboard web.

### R9 — Preset, waypoint & rute
- Kartu preset A–D: status per preset (`ps`: IDLE/JALAN) + tombol jalankan (`preset`) — tekan pertama langsung jalan, STOP = batal.
- Tabel waypoint: semua kolom `wp[16]`, tombol **Jalan** (`goto`) dan **Di sini** (`wp_here`) per WP.
- Di Lanjutan: edit waypoint (x, y, yaw kosong = tanpa arah, `wp_set`), hapus (`wp_del`), tanda ramp (`ramp_mark`), rute tiap preset (`route_set`/`route_go`).
- **AC**: rekam rute 3 WP via app, preset A jalan penuh; WP tampil di peta (R5) sinkron dengan dashboard.

### R10 — Tes gerak, kecepatan lanjutan, PID & auto-tune (Layarn Lanjutan)
- Tes gerak: 4 tombol (`mtest`) + "Tes 4 arah berurutan" + status/kolom hasil (`mc`) identik dashboard.
- Kecepatan lanjutan: turbo/nav/loc (`speed`).
- PID per roda: tabel kp/ki/kd 3 roda (`pid`, simpan flash), auto-tune per roda/semua (`autotune`), status `tune` + sebab `tw`.
- **AC**: hasil auto-tune tersimpan & termuat ulang; semua nilai telemetri tampil.

### R11 — Pengaturan app & mock server uji
- Di Lanjutan: host robot (default `192.168.4.1`), path/port default; tersimpan SharedPreferences;ubah terpasang tanpa sambung robot (uji mock).
- `tools/mock_ws.py` di repo app (Python stdlib saja): WS server yang memancarkan contoh telemetri lengkap (angka jalan: sinus RPM, pose bergerak) + mencetak perintah yang diterima.
- **AC**: app tersambung ke mock, semua kartu hidup; host diubah → tersambung ulang ke host baru setelah restart app.

## 7. Urutan Implementasi (agent: ikuti ini, jangan bertanya)

| Fase | Requirement | Alasan urutan |
|------|-------------|---------------|
| 1 | R1, R2, R3 | Kerangka + koneksi + data; fondasi semua UI |
| 2 | R11, R4 | Alat uji tanpa robot; lalu kerangka beranda hidup |
| 3 | R5, R6, R7 | Kartu monitoring (paritas penuh) |
| 4 | R8, R9 | Kontrol & waypoint |
| 5 | R10 | Lanjutan |

## 8. Kriteria Terima Keseluruhan

1. `./gradlew assembleDebug` hijau, tanpa warning baru; lint dasar bersih.
2. Dengan `tools/mock_ws.py`: semua kartu terisi, semua tombol mengirim perintah benar (log mock), app tidak crash saat data bolong/rusak.
3. Robot nyata: app + dashboard web (PC) terbuka bersamaan; semua perubahan via app tercermin di dashboard dan sebaliknya; latensi chip < 200 ms di jangkauan AP.
4. STOP di app menghentikan robot; koneksi app putus saat otonom → robot berhenti ≤ 300 ms (failsafe).
5. **Paritas penuh**: setiap kartu & kontrol `monitor.html` (§5.1–5.2) punya padanan app yang berfungsi — dicek menyilang daftar kartu dashboard.

## 9. Risiko & Mitigasi

| Risiko | Mitigasi |
|--------|----------|
| HP tidak otomatis join AP `Omni3X` | Instruksi satu kali di layar; Android mengingat AP setelahnya; app pulih sendiri (R2) |
| HP "terjebak" mencari internet di AP tanpa internet | App tidak butuh internet; instruksi biarkan jaringan; telemetri tetap mengalir via AP |
| Android 12+ minta izin lokasi bila membaca SSID | App tidak membaca SSID — deteksi via jangkauan host (TCP/WS), tanpa izin lokasi |
| Telemetri 10 Hz membanjiri UI thread | Parse di coroutine IO; render dibatasi (peta/grafik diredraw ≤ 20 Hz) |
| Grafik 300 titik berat di HP lama | Kanvas sederhana tanpa library; sampling adaptif bila diperlukan |
| Dua klien dashboard (web + app) bersamaan | Firmware mendukung banyak klien WS; hb dari salah satu menjaga detak; perilaku sudah teruji di dashboard |
| Jitter WS mengganggu failsafe 300 ms | hb 100 ms cukup; reconnect cepat; tes AC R2 saat robot jalan |
