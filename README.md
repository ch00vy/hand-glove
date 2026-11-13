# Rokoko Smartgloves / MyoHand Control Center

GUI Tkinter Windows memakai tampilan dan kontrol dari ZIP Control Center: konfigurasi Wi-Fi, pengaturan lokasi SDK/project dan port, tombol Start/Stop Driver, Solver, Viewer, serta log proses.

## Menjalankan

```powershell
cd C:\hand-glove
.\.venv\Scripts\python.exe main.py
```

`rokoko_control_gui.py` juga dapat dijalankan langsung dengan interpreter yang sama. SDK menggunakan `ROKOKO_SDK_HOME` atau fallback `%LOCALAPPDATA%\Rokoko Device SDK`; lokasinya dapat diedit di GUI. Viewer memakai `.venv` dari folder project yang dipilih.

## Wi-Fi dan password SDK terpasang

Sudah diperiksa menggunakan SDK lokal:

```powershell
.\.venv\Scripts\python.exe -B "$env:LOCALAPPDATA\Rokoko Device SDK\examples\python\network_configuration.py" set --help
```

Versi ini **tidak memiliki argumen `--password`**. Opsi `set` adalah `--ssid`, `--receiver-address`, `--receiver-port`, dan `--security`. Script meminta password melalui `getpass.getpass()` untuk WPA2/WPA3.

GUI memakai `sdk_network_runner.py` untuk menjalankan script SDK asli dengan password dari stdin dan memasoknya ke prompt `getpass`. Password tidak masuk argumen proses, JSON, maupun log GUI; SDK tidak dimodifikasi. GUI menyediakan WPA2 sesuai ZIP. Password wajib diisi saat Set Wi-Fi.

Tombol **Save** menyimpan SSID, lokasi, dan port di `.rokoko_myohand_gui.json` dalam folder aplikasi. Jika paket opsional `keyring` tersedia, password disimpan melalui Windows Credential Manager. Tanpa paket itu, password hanya bertahan selama aplikasi terbuka. Untuk mengaktifkan penyimpanan password:

```powershell
.\.venv\Scripts\python.exe -m pip install keyring
```

## Urutan penggunaan

1. Periksa SDK folder dan Project folder; isi SSID, password, dan port, lalu **Save**.
2. Klik **Start** pada SDK Driver dan hubungkan Smartgloves via USB. Script konfigurasi membutuhkan driver RCSP lokal (default `localhost:45451`) dan memilih perangkat pertama yang dilaporkan SDK.
3. Klik **Set Wi-Fi**, kemudian **Verify (Get)**. Berhasil menyimpan konfigurasi belum membuktikan perangkat sudah terhubung ke access point.
4. Jalankan Solver dan Viewer, atau gunakan **Start Pipeline** untuk menjalankan Driver, Solver, Viewer secara berurutan. Tahan tangan kanan terbuka dan stabil selama kalibrasi viewer.
5. Gunakan **Stop All** untuk menghentikan proses yang dibuat GUI. Proses eksternal tidak dihentikan.

Port yang diterapkan ke perintah SDK:

| Pengaturan | Default | Penggunaan |
| --- | --- | --- |
| Receiver port | 14041 | UDP perangkat ke Driver (`--driver-udp-port`) dan konfigurasi Wi-Fi |
| Driver RGMP | 12276 | TCP Driver (`--rgmp-output-addr`) ke Solver |
| Solver output | 12277 | TCP Solver ke Viewer; `--emit both` menyertakan `joints_world` |

GUI menghindari duplikasi Driver/Solver berdasarkan nama executable Windows. Status `External` tidak memeriksa kesesuaian port proses tersebut. `Running` hanya berarti proses aktif; Start Pipeline memakai jeda antarproses dan belum memverifikasi frame sensor. Stop All membatalkan peluncuran yang masih terjadwal.

`rgmp_client.py`, pipeline real-time, dan analisis offline tetap tersedia. Viewer menampilkan retargeting kinematik, tanpa estimasi gaya atau aktivasi otot.

Pemeriksaan tanpa perangkat:

```powershell
.\.venv\Scripts\python.exe -m py_compile main.py rokoko_control_gui.py sdk_network_runner.py
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## Analisis offline Hand Solver

Pasang dependensi ke `.venv` yang sudah ada, lalu jalankan rekaman:

```powershell
cd C:\hand-glove
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe scripts\analyze_recording.py --input recordings\solved_hand.csv --calibration-start 0 --calibration-end 1.5
```

`--help` menjelaskan opsi input, output, ambang stabilitas, konfigurasi sumbu, dan model XML. Input bisa berupa CSV UTF-8 atau UTF-16, dengan atau tanpa header, dengan 7 atau 8 kolom resmi. Baris rusak dilaporkan dan dilewati. Timestamp diperlakukan sebagai mikrodetik seperti output `rgmp_stream.py` SDK. Rekaman asli hanya dibaca.

Hasil disimpan di `outputs/`: `joint_inventory.csv`, `recording_summary.json`, `skeleton.md`, `neutral_calibration.json`, `axis_candidates.json`, `joint_angles_candidate.csv`, `world_geometry.json`, `myohand_model_inventory.json`, dan empat gambar pada `plots/`. `config/myohand_mapping.json` dibuat sekali sebagai template. Script tidak menimpa mapping yang sudah diedit.

Orientasi lokal `(x,y,z,w)` berasal dari `joints_local`; root saja memiliki `TRANSFORM FLOAT[7]` dengan posisi. Offset lokal untuk joint lain tidak tersedia dalam data tersebut. World-space `TRANSFORM FLOAT[7]` pada `joints_world` menyediakan posisi untuk analisis jarak ujung jari. SDK menetapkan frame dunia `LTP_NED`. Kalibrasi memakai mean rotasi SciPy, sehingga sampel `q` dan `-q` setara. Kestabilan dilaporkan sebagai deviasi geodesik p95 dari mean; status `unstable` adalah peringatan, dan kandidat sudut tetap ditandai dengan status tersebut.

`R_delta = R_neutral^-1 × R_current` dihitung pada frame lokal joint. Secara default, sumbu kandidat diambil dari arah utama variasi rotation-vector yang memiliki dominansi ≥0,8 dan rentang gerak ≥10°. Ini hanya sumbu gerak yang teramati, belum sumbu fleksi anatomis atau tanda positif yang tervalidasi. Tip, forearm, dan hand tidak diperlakukan sebagai hinge aktif. Kolom abduksi kosong sampai sumbu sekundernya ditetapkan.

Untuk menetapkan sumbu lokal secara eksplisit, edit `config/axis_mapping.json`, misalnya:

```json
{
  "joints": {
    "right_finger2_proximal": {
      "flexion_axis": [1, 0, 0],
      "abduction_axis": [0, 1, 0]
    }
  }
}
```

**Angka contoh di atas bukan kalibrasi anatomis**; tentukan dari uji gerak dan transform koordinat yang tervalidasi. Untuk dua sumbu, dekomposisi berurutan `R_delta = R_flex × R_residual` dipakai: flexion adalah twist terhadap sumbu pertama, abduction adalah twist residual terhadap sumbu kedua. Sudut dalam radian dan derajat tersedia; kontinuitas temporal dijaga dengan unwrap untuk flexion. Abduction masih memerlukan validasi konvensi dan coupling.

Model `myohand.xml` diinspeksi lewat API MuJoCo untuk nama dan tipe joint, axis, range, alamat `qpos`/`qvel`, DoF, pose default, dan hierarki body. Template mapping mencantumkan joint target yang benar-benar ditemukan, semuanya `unmapped`. Untuk mengaktifkan retargeting, isi `rokoko_joint`, `axis`, `sign`, `scale`, `offset`, dan ubah `mapping_status` menjadi `validated` hanya setelah memvalidasi pasangan joint dan tanda sumbu. Source juga harus memiliki `flexion_axis` eksplisit dalam konfigurasi sumbu. Dengan mapping tervalidasi, script menghasilkan `outputs/retargeted_qpos.csv` dan membatasi setiap qpos ke range model. Hasil qpos perlu diuji dengan forward kinematics MuJoCo sebelum dipakai sebagai interpretasi gerakan tangan.

Ada tiga tingkat validasi yang berbeda: **numerik** (parsing, quaternion, interval, dan rangkaian qpos), **kinematik** (arah/sumbu, pose forward kinematics, konsistensi open–grasp), dan **anatomis** (label MCP/PIP/DIP dan rentang yang diverifikasi dengan referensi gerak). Hasil saat ini memenuhi pemeriksaan numerik dan geometri rekaman; sumbu anatomis serta mapping myoHand belum divalidasi. `qpos` tidak mengestimasi aktivasi otot, fatigue, atau sEMG.

Unit test sintetis tanpa perangkat:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## Memutar gerak open–grasp di myoHand

Pipeline kedua memecahkan inverse kinematics posisi landmark menggunakan `joints_world` pada rekaman dan forward kinematics MuJoCo. Pasangan landmark yang dipakai dapat ditinjau di `config/myohand_landmarks.json`: body ruas jari myoHand dan site `THtip`, `IFtip`, `MFtip`, `RFtip`, `LFtip` dipasangkan dengan nama joint solver yang sesuai. Ini adalah korespondensi geometri; konfigurasi sudut anatomis `config/myohand_mapping.json` tetap `unmapped`.

```powershell
.\.venv\Scripts\python.exe scripts\retarget_recording.py --input recordings\solved_hand.csv --stride 4
.\.venv\Scripts\python.exe scripts\play_retargeting.py --trajectory outputs\myohand_ik_trajectory.npz
```

Viewer MuJoCo memerlukan desktop dan dukungan OpenGL. Gunakan `--loop` untuk mengulang atau `--speed 0.5` untuk memperlambat. Script `--help` menjelaskan opsi lainnya. MuJoCo menampilkan pose melalui `qpos` dan `mj_forward`, tanpa simulasi gaya atau aktivasi otot. Pergelangan tetap pada pose default; yang direproduksi adalah gerak jari relatif terhadap tangan. Model maupun rekaman asli tidak diubah.

Metode ini menghitung posisi Rokoko dalam frame `right_hand`, menghitung pose netral dari 0–0,5 detik, lalu mencari rotasi dan skala seragam yang paling cocok dengan landmark model dalam frame `capitate`. Model myoHand memiliki ukuran tulang berbeda; target IK dibuat dari **perpindahan landmark terhadap pose netral**, ditransformasikan ke ukuran model. Ini mempertahankan pola gerak open–grasp tanpa berpura-pura bahwa kedua skeleton mempunyai ukuran sama. Optimizer membatasi joint pada range MuJoCo dan memberi bobot lebih pada fingertip. Setiap frame keempat dioptimalkan; qpos di antaranya diinterpolasi dan galatnya diukur secara terpisah. `--stride 1` mengoptimalkan setiap frame dengan waktu proses lebih lama.

Output baru di `outputs/`:

- `myohand_ik_trajectory.npz`: waktu, qpos 23 joint, target dan hasil landmark pada frame model, galat, serta flag frame yang dioptimalkan.
- `myohand_ik_trajectory.csv`: qpos dan galat per frame agar mudah dianalisis.
- `myohand_ik_validation.json`: galat semua landmark, ujung jari, frame interpolasi, per ujung jari, skala registrasi, perbedaan bentuk pose netral, dan joint yang dekat batas.
- `myohand_ik_error.png` dan `myohand_tip_comparison.png`: galat ujung jari dan perbandingan jarak ujung jari target versus model sepanjang rekaman.

Galat IK mengukur kecocokan terhadap target **yang telah diregistrasi dan diskalakan**, bukan terhadap posisi absolut sensor di ruang LTP_NED. Perbedaan bentuk netral dan banyaknya joint yang menyentuh batas dapat membatasi akurasi. Kecocokan numerik belum menjadi validasi anatomis atau bukti bahwa model meniru gaya genggaman, kontak objek, atau aktivitas otot. Periksa laporan galat sebelum menyebut reproduksi akurat untuk tujuan penelitian.

## Retargeting kinematik real-time (port 12277)

Jalankan Hand Solver secara manual di terminal terpisah dan pastikan `joints_world` dikirim di port 12277. Saat memulai client, **tahan tangan kanan terbuka dan stabil** selama sekitar 0,5 detik untuk registrasi netral. Dari terminal proyek:

```powershell
.\.venv\Scripts\python.exe real_time\run_retargeting.py --host 127.0.0.1 --port 12277
```

MuJoCo viewer akan menampilkan pose `qpos` hasil IK secara langsung. Tutup viewer atau tekan Ctrl+C di terminal untuk mengakhiri client. Proses SDK Driver, Hand Solver, dan koneksi Wi-Fi tidak dihentikan. Jika ada lebih dari satu perangkat pada server RGMP, tentukan `--device-id ID`. Untuk pemeriksaan tanpa jendela, gunakan `--headless --duration 10`; perintah ini mengembalikan status gagal jika tidak ada frame world yang berhasil diproses. Semua pilihan tersedia lewat `--help`.

Pipeline real-time berada di `real_time/`. Callback RGMP resmi hanya memasukkan data ke antrean berukuran satu; worker lain mem-parsing `joints_world`, mengkalibrasi pose netral, dan menjalankan IK. Frame lama dibuang bila input lebih cepat daripada IK agar latensi tidak terus bertambah. Viewer menggunakan pose terbaru di thread utama. Status setiap detik di terminal menampilkan jumlah frame world, frame yang di-fit, frame yang dibuang, data tidak valid, usia frame terakhir, galat fingertip, dan latensi pemrosesan. `Connected` pada RGMP sendiri belum berarti retargeting aktif; tunggu pesan `Kalibrasi netral selesai; retargeting aktif`.

Kalibrasi live menghitung posisi 24 landmark relatif terhadap `right_hand`, lalu menunggu jendela 0,5 detik dengan deviasi posisi p95 di bawah 5 mm. Pose yang stabil tetapi **tertutup** masih dapat diterima oleh detektor; operator harus memastikan tangan terbuka. Titik world diregistrasi ke model myoHand memakai metode yang sama dengan analisis offline. Pergelangan tetap pada pose default model dan tidak ada estimasi kontak, gaya, atau aktivasi otot. Bila `joints_world` berhenti tiba, model menahan pose terakhir sampai data baru atau koneksi ditutup.

Parameter `--max-ik-evaluations` (default 12), `--viewer-fps` (default 30), `--neutral-window`, `--stability-mm`, dan `--tip-weight` dapat disesuaikan. Menaikkan iterasi dapat memperbaiki fit tetapi menambah latensi dan jumlah frame yang dibuang. Kinerja nyata bergantung pada CPU, laju SDK, dan viewer. Dalam replay rekaman sekitar 83 Hz pada lingkungan ini, kalibrasi berhasil, IK menyelesaikan 117 dari 155 frame yang diterima (frame awal untuk kalibrasi), 5 frame lama dibuang, dan contoh latensi terakhir sekitar 16 ms. Ini bukan pengukuran pada perangkat langsung.

Pengujian tanpa Smartgloves menggunakan server TCP RGMP v2 sintetis dan model MyoSuite yang terpasang:

```powershell
.\.venv\Scripts\python.exe -m unittest real_time.test_pipeline -v
```

## Benchmark diagnostik optimizer IK

Jalankan offline dengan Python 3.12 dari virtual environment proyek:

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_ik.py --input recordings\solved_hand.csv --max-nfev 8 12 20 30
```

Script memakai parser, registrasi netral, model, dan `KinematicFitter` yang sama dengan retargeting yang sudah ada. Seluruh frame diurutkan menurut timestamp lalu dioptimalkan, tanpa antrean streaming, dropped frames, atau interpolasi. Target dibuat sekali untuk semua skenario. Kalibrasi memakai mean posisi hand-relative pada 0–0,5 detik seperti script offline `retarget_recording.py`; ini bukan detektor jendela stabil otomatis pipeline live. Interval netral harus berisi tangan terbuka. Pilihan `--neutral-end`, `--model`, `--landmarks`, `--tip-weight`, `--temporal-weight`, dan `--neutral-weight` diterapkan identik ke seluruh skenario.

Setiap skenario memperoleh fitter baru dengan pose awal default yang sama. Di dalam skenario, solusi frame sebelumnya menjadi warm start frame berikutnya, sama seperti pipeline live. Karena solusi sebelumnya dapat berbeda antarbudget, nilai warm start frame lanjut juga dapat berbeda; aturan warm start dan kondisi awal skenario tetap identik. Tiga frame warm-up dengan budget 12 dijalankan identik sebelum tiap skenario, kemudian fitter direset; warm-up tidak masuk statistik. Hanya `max_nfev` eksperimen yang berubah. Objective, Jacobian analitik, bounds, dan toleransi `ftol=1e-8`, `xtol=1e-8`, `gtol=1e-9` dipertahankan.

Hasil tersimpan di `outputs/ik_benchmark/`:

- `benchmark_summary.csv`: mean/P95 error, persentase sukses dan batas evaluasi, mean nfev, mean/median/P95 compute time, dan effective IK FPS.
- `benchmark_per_frame.csv`: indeks/timestamp, error rata-rata dan masing-masing fingertip, `optimizer_success`, `optimizer_status`, `optimizer_message`, `optimizer_nfev`, `optimizer_njev`, `optimizer_cost`, `optimizer_optimality`, dan `ik_compute_ms`.
- `ik_error_comparison.png`, `ik_compute_time_comparison.png`, `optimizer_success_comparison.png`.
- `benchmark_metadata.json`: parameter, versi library, hash input/target/konfigurasi landmark, kalibrasi, pose awal, serta definisi metrik untuk penelusuran eksperimen.

Error per frame adalah mean lima jarak Euclidean fingertip terhadap target yang sudah diregistrasi dan diskalakan. Mean dan P95 pada summary dihitung atas nilai per-frame tersebut, termasuk frame netral; P95 bukan persentil gabungan semua pengukuran tip. Error ini mengukur kecocokan target kinematik, bukan ground truth anatomis. `optimizer_success=True` berarti solver memenuhi salah satu kriteria penghentian (`status > 0`), sedangkan `status=0` berarti batas evaluasi tercapai. Konvergensi tidak menjamin error fingertip kecil; cost juga mencakup bobot landmark dan regularisasi.

`ik_compute_ms` mengukur hanya pemanggilan `least_squares`, termasuk residual dan Jacobian. Pengukuran ini mengecualikan parsing, kalibrasi, pembuatan target, evaluasi error akhir, logging, dan rendering. Effective IK FPS = `1000 / mean(ik_compute_ms)`: **estimasi kapasitas komputasi offline, bukan FPS streaming end-to-end**. Waktu berasal dari satu lintasan per skenario dan dipengaruhi beban CPU serta urutan skenario; perbedaan waktu kecil perlu diuji ulang sebelum dianggap signifikan.

Pada pipeline live, `latency_ms` tetap berarti waktu sejak callback RGMP sampai pose selesai, termasuk waktu tunggu antrean dan pemrosesan. Callback dan penyelesaian pose memakai `perf_counter` untuk presisi durasi pendek di Windows; pelacakan usia frame tetap memakai `monotonic`. Diagnostik optimizer diteruskan melalui antrean pose tanpa mengubah strategi latest-frame. Script real-time mencatat diagnostik pose terbaru sekitar sekali per detik; CSV benchmark menyimpan setiap frame offline.
