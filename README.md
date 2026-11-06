# Smartgloves Connector

GUI Tkinter sederhana untuk membaca stream TCP **RGMP v2**. Aplikasi tidak menjalankan atau menghentikan SDK Driver, Hand Solver, maupun koneksi Wi-Fi Smartgloves.

## Menjalankan (PowerShell)

Python 3.12 dengan Tcl/Tk diperlukan. `.venv` proyek yang sudah tersedia telah diperiksa menggunakan Python 3.12.1. GUI RGMP memakai standard library; analisis offline membutuhkan paket pada `requirements.txt`.

```powershell
cd C:\hand-glove
.\.venv\Scripts\python.exe main.py
```

Interpreter Python 3.12 sistem juga dapat digunakan:

```powershell
py -3.12 main.py
```

Tidak perlu membuat ulang `.venv` atau menjalankan `install-device-sdk.ps1`.

## Lokasi SDK eksternal

`config.py` membaca `ROKOKO_SDK_HOME`, dengan fallback `%LOCALAPPDATA%\Rokoko Device SDK` (umumnya `C:\Users\Lenovo\AppData\Local\Rokoko Device SDK`). Jika instalasi berbeda, atur sebelum menjalankan aplikasi:

```powershell
$env:ROKOKO_SDK_HOME = 'C:\Users\Lenovo\AppData\Local\Rokoko Device SDK'
.\.venv\Scripts\python.exe main.py
```

Adaptor mengimpor `examples\python\rgmp_core` langsung dari instalasi itu. SDK tidak disalin atau dimodifikasi; penulisan bytecode SDK dinonaktifkan saat impor. API telah diperiksa dari `rgmp_stream.py`, `rgmp_client_example.py`, serta `rgmp_core/{client,frame,layout,definition}.py` yang terpasang. Modul parser ini hanya memerlukan Python standard library.

## Menghubungkan

1. Jalankan SDK Driver secara manual di terminal terpisah. Untuk joint hasil solver, jalankan juga Hand Solver secara manual sesuai konfigurasi SDK Anda.
2. Isi Host `127.0.0.1` untuk server lokal, atau IP/hostname server lain.
3. Gunakan port **12276** untuk stream sensor mentah dari SDK Driver, atau **12277** untuk joint hasil Hand Solver. Server terkait harus sudah mendengarkan port tersebut.
4. Klik **Connect**. Klik **Disconnect** untuk menutup hanya koneksi pembaca GUI. Sambungkan ulang jika ingin mengganti host/port.

Status `Connecting` berarti percobaan koneksi sedang berjalan. `Connected` berarti TCP tersambung. **Streaming hanya ditampilkan setelah frame DATA yang sesuai definition dan berhasil didekode diterima**; definition saja atau byte sembarang tidak dihitung. Sesudah 3 detik tanpa data valid, status menjadi `Connected — No Data`; koneksi tetap terbuka dan otomatis kembali `Streaming` jika data valid masuk. `Error` menunjukkan koneksi ditolak, timeout, koneksi hilang, atau kesalahan protokol; penyebab ditampilkan di log. Setelah error, gunakan Connect untuk mencoba lagi.

Penghitung adalah jumlah frame DATA valid, bukan jumlah stream di dalam frame atau jumlah definition. Timestamp terakhir mencantumkan waktu penerimaan lokal dan timestamp RGMP asli dalam mikrodetik, tanpa mengasumsikan epoch server.

## Cara kerja dan batasan

- `main.py`: widget Tkinter, queue, dan polling `after()` pada thread GUI.
- `rgmp_client.py`: worker TCP, adaptor `RgmpClient` resmi, serta decoding slot melalui `rgmp_core.layout.read_slot`. Framing biner memakai header `<II>` dan data header `<IIQ>` resmi, bukan newline.
- `config.py`: host/port default, lokasi SDK, timeout koneksi 3 detik dan polling socket 250 ms.
- Disconnect memberi sinyal ke worker; socket ditutup oleh worker. Penutupan jendela menunggu worker melalui `after()` tanpa memblokir GUI. Percobaan koneksi yang sedang berlangsung dapat memerlukan waktu sampai timeout; resolusi DNS mengikuti resolver OS.
- Potongan frame tetap disimpan saat timeout baca, sehingga fragmentasi TCP tidak merusak framing. Definition dan layout dipisahkan per perangkat oleh parser resmi.
- Log data dibatasi sekitar sekali per detik, tampilan penghitung sekitar 10 kali per detik, log maksimal sekitar 500 baris, dan queue dibatasi untuk menjaga GUI responsif. Semua frame valid tetap dihitung.
- Untuk penelitian lanjutan, `RGMPClient(on_data=callback)` meneruskan setiap frame terdekode berisi identitas perangkat/grup, timestamp, metadata stream, dan tuple nilai. Callback berjalan di worker: harus cepat dan tidak memanggil Tkinter. `Recording.add_rgmp_message(message)` menerima bentuk ini; GUI belum menjalankan retargeting.

Pemeriksaan syntax tanpa menjalankan GUI:

```powershell
.\.venv\Scripts\python.exe -m py_compile main.py rgmp_client.py config.py
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
