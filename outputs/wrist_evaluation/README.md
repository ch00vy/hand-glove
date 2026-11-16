# Evaluasi wrist dan forearm

Implementasi memakai model MyoSuite yang terpasang, FK MuJoCo, residual SO(3),
bounds dari compiled model, dan regularisasi temporal. Pengujian di sini sintetis;
belum ada validasi Smartgloves langsung atau pemeriksaan visual GUI.

## Konfigurasi model aktual

Hasil API MuJoCo lengkap: [joint_inventory.json](joint_inventory.json).
Axis di tabel adalah axis lokal body, bukan axis global viewer. Semua joint hinge,
limited, dan qpos default 0 radian. qpos/qvel address merupakan hasil inspeksi,
bukan konstanta yang dipakai implementasi.

| Joint | Body / parent | Axis lokal (x,y,z) | Range rad | Range derajat | qpos / qvel |
|---|---|---|---|---|---|
| pro_sup | radius / ulna | (-0.01716099, 0.99266564, -0.11966796) | [-1.5708, 1.5708] | [-90.00021, 90.00021] | 0 / 0 |
| deviation | lunate / radius | (0,0,1) | [-0.174533, 0.436332] | [-10.00000, 24.99998] | 1 / 1 |
| flexion | lunate / radius | (1,0,0) | [-0.785398, 0.785398] | [-44.99999, 44.99999] | 2 / 2 |

Urutan: ulna → radius (`pro_sup`) → lunate (`deviation`, lalu `flexion`
pada body yang sama) → capitate. Transformasi statis parent/body juga berpengaruh.
Gunakan FK, jangan menafsirkan tabel sebagai Euler global X/Y/Z.

## Pemakaian

Dari `C:\hand-glove`, jalankan Driver dan Hand Solver seperti biasa. Solver harus
mengirim `joints_world` lewat TCP 12277, misalnya konfigurasi existing `--emit both`.

```powershell
.\.venv\Scripts\python.exe real_time\run_retargeting.py --port 12277 --wrist-mode forearm-twist
```

`forearm-twist` mengaktifkan ketiga joint; pronasi/supinasi adalah proxy twist
forearm terhadap referensi netral. Default `relative` hanya mengaktifkan
flexion/deviation dan menahan pro_sup pada default model karena pronasi anatomis
tidak dapat diamati secara terpisah dari gerakan bahu/siku. Gunakan `--wrist-mode off`
untuk menjalankan finger-only. Command peluncuran GUI kini memilih
`forearm-twist` secara eksplisit; tidak ada kontrol GUI baru. GUI juga menambahkan
log input/output pada `outputs/wrist_evaluation/live_metrics.csv`. Mode CLI tetap
default `relative`; pilih `forearm-twist` lewat argumen untuk mengaktifkan pro_sup.

Netral yang harus diambil: tangan kanan terbuka, jari lurus, wrist lurus tanpa
deviasi, siku sekitar 90°, forearm horizontal, ibu jari menghadap atas. Tahan
stabil minimal 0,5 detik. Posisi forearm ini mendefinisikan referensi sumber;
tidak mengubah orientasi dasar model. Stabilitas bentuk jari tidak membuktikan
posisi anatomis netral; operator tetap harus memeriksanya.

Kalibrasi wrist memerlukan minimal 15 frame dan cakupan minimal 80% jendela,
dengan deviasi geodesik maksimum forearm dan hand terhadap mean SO(3) <= 3°.
Ubah ambang melalui `--wrist-stability-deg`. Mean quaternion tahan terhadap
antipodal; quaternion input dinormalisasi parser existing. File
`neutral.json` berisi quaternion netral xyzw, reference_frame, dan matriks rotasi
registrasi. File ditulis saat kalibrasi, tidak otomatis dimuat pada sesi baru.

Tekan **R** saat viewer fokus untuk mengulang kalibrasi wrist; dalam mode headless,
panggil `LiveRetargeter.recalibrate_wrist()`. Kembalikan wrist ke netral terlebih
dahulu. Registrasi jari dipertahankan; pose wrist ditahan selama kalibrasi dan
akan kembali ke default model saat netral baru aktif. Gunakan gerakan terkendali
ketika melakukan reset referensi.

## Rotasi, observabilitas, dan sinkronisasi

Sumber diperiksa dari SDK lokal: `hand_solver/emit.py`, `solver_core/smartglove.py`,
dan `solver_core/finger_poser.py`. Wire memakai xyzw aktif; `joints_world`
menyatakan hand dan forearm terhadap `LTP_NED`. `joints_local` menyatakan hand
terhadap `right_fore_arm`, dan forearm terhadap `LTP_NED`. Ekstraktor mendukung
keduanya secara terpisah. Pipeline live memakai **hanya satu pesan world** untuk
landmark dan orientasi, device dan timestamp identik (toleransi sinkronisasi 0 µs).
Tidak ada cache local/world, penantian grup lain, atau pencampuran timestamp.
Sumber yang hanya mengirim local tidak cukup untuk Finger IK world existing.

Untuk `F,H` sumber, relatif wrist adalah `F.inv()*H`. Jika `C` adalah registrasi
rotasi vektor dari frame hand sumber ke capitate model, target relatif model:

```text
S0 = F0.inv() * H0
S  = F.inv() * H
Mrel_target = Mrel0 * C * S0.inv() * S * C.inv()
```

`C` berasal dari registrasi landmark netral jari; tidak memerlukan asumsi sumbu
global Rokoko sama dengan MuJoCo. Fit relatif memakai FK radius→capitate.
Mode twist memetakan delta forearm ke world model menggunakan orientasi netral,
lalu mengambil swing–twist pada axis pro_sup aktual dari FK. Swing forearm
tidak ditransfer ke model. Residual mencakup rotation vector relatif wrist,
selisih twist terhadap pro_sup, dan perubahan joint terhadap solusi sebelumnya.
Ini optimasi terpisah dari Finger IK beserta Jacobian analitiknya.

SDK membentuk forearm dari hub + arm sensor; Coil Pro dapat menambahkan anchoring.
Tanpa referensi upper arm, twist sumber **bukan pengukuran pronasi anatomis yang
terisolasi**. Perubahan bahu/siku, drift heading, dan re-anchoring bisa masuk ke
proxy. Selain itu, solver SDK mengganti sensor yang hilang dengan identity;
pesan solved tidak menyediakan quality sensor mentah. Quaternion numerik valid
tidak membuktikan sensor arm tersedia. Label `forearm_twist_proxy` menyatakan
keterbatasan ini; tidak ada klaim orientasi global 3D atau translasi terlacak.

Queue input/output berkapasitas satu tetap mengutamakan frame terbaru. Frame
duplikat/out-of-order dan device lain ditolak. Jika forearm hilang/invalid,
finger tetap diproses dan wrist ditahan; `wrist_sync_status=missing_or_invalid`,
error orientasi `None`, optimizer success `False`. Saat seluruh stream berhenti,
viewer menahan pose terakhir dan menampilkan usia world terakhir. Perubahan
reference_frame memerlukan kalibrasi ulang. Perubahan anchoring tanpa perubahan
nama reference tidak dapat otomatis dideteksi dengan data solved ini.

Quaternion antipodal tidak mengubah rotasi. Twist di-unwrap sebelum diberi bounds
agar melewati ±180° tidak berpindah dari batas positif ke negatif. Unwrap
membutuhkan perubahan antar-frame kurang dari 180°; setelah kehilangan data lama,
kembalikan netral dan kalibrasi ulang. Gerakan di luar range menghasilkan error
orientasi yang tetap dilaporkan, walaupun optimizer sukses.

## Metrik

Output pose pipeline dan log runner memisahkan `wrist_orientation_error_deg`,
`wrist_compute_ms`, tiga `*_angle_deg`, `wrist_optimizer_success/status` dari
`tip_mean_error_mm`, `ik_compute_ms`, `latency_ms`, `optimizer_success`, dan
`dropped_input`. Pose juga memuat `wrist_relative_error_deg`,
`forearm_twist_error_deg`, `wrist_observability`, dan `wrist_sync_status`.

Error orientasi adalah sudut geodesik hand model terhadap **target yang didukung
mode aktif**, bukan error orientasi global hardware. Pada relative, target
forearm tetap default; pada forearm-twist, hanya twist diterapkan ke forearm
default. Error relatif wrist dan twist terpisah membantu melihat saturasi DoF.
Optimizer sukses tidak menjamin error kecil. `wrist_compute_ms` mencakup
pembuatan target, optimasi dan evaluasi akhir, tanpa kalibrasi/rendering;
`latency_ms` tetap sejak callback sampai pose selesai, kini termasuk wrist.

## Pengujian otomatis

```powershell
.\.venv\Scripts\python.exe scripts\inspect_wrist_joints.py --output outputs\wrist_evaluation\joint_inventory.json
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m unittest real_time.test_pipeline -v
```

Hasil setelah perbaikan peluncuran GUI: 32 test pada `tests/` dan 3 test pipeline TCP lulus.
Delapan test wrist mencakup:

- Netral, palm down/up, flexion/extension, dua arah deviation, dan kombinasi:
  solusi sintetis in-range berbeda < 0,002 rad dari target, error orientasi < 0,1°.
- Quaternion antipodal, noise kalibrasi, penolakan jendela bergerak,
  reference berubah, serta lintasan 160°–200° tanpa lompatan ±180°.
- Target di luar range tetap dibatasi; error > 5° tetap dilaporkan.
- Deviation dapat berbalik arah setelah saturasi kedua sisi, termasuk saat
  forearm juga berotasi; CSV input/output diuji melalui runner TCP sintetis.
- Forearm global tidak menggerakkan pro_sup dalam mode relative.
- Gabungan wrist dan grasp mempertahankan qpos jari persis serta landmark
  hand-relative sampai toleransi numerik 1e-12 m.
- Pipeline live sintetis: timestamp konsisten, wrist+grasp, forearm hilang,
  foreign device, local group, out-of-order, dan kalibrasi ulang wrist.

Waktu test bergantung komputer dan bukan pengukuran frekuensi hardware.

## Pengujian manual yang masih diperlukan

Jalankan command forearm-twist di atas dan rekam metrik terminal tiap skenario.
Lakukan gerakan perlahan dahulu, lalu ulang dengan menggenggam.

| Skenario | Tindakan | Pemeriksaan |
|---|---|---|
| Neutral | Tahan pose netral eksplisit | Kalibrasi wrist selesai; sudut dekat 0; error kecil |
| Palm down | Pronasi dari netral | Arah visual sesuai, pro_sup menuju salah satu sisi; tidak melampaui ±90° |
| Palm up | Supinasi dari netral | Arah berlawanan; tidak ada flip saat quaternion berubah tanda |
| Flexion | Tekuk palmar | Arah visual benar; flexion dalam ±45° |
| Extension | Tekuk dorsal | Arah berlawanan; flexion dalam ±45° |
| Radial/ulnar | Gerak ke ibu jari/kelingking | Arah anatomis dicek secara visual; deviation dalam [-10°,25°] |
| Combined grasp | Gabungkan pronasi, wrist, dan grasp | Jari tetap mengikuti; bandingkan tip error dengan mode off |

Untuk setiap skenario catat arah benar/terbalik, error orientasi rata-rata/maks,
tip error, wrist/IK compute, latency, dropped_input, kelancaran dan saturasi joint.
Uji pula memindahkan bahu/siku dengan wrist tetap lurus untuk memperlihatkan
ambiguitas proxy. Periksa tidak ada freeze tanpa status ketika input dihentikan,
serta gunakan R dengan jari menggenggam untuk memastikan kalibrasi jari tetap.
Tentukan ambang performa dan error yang diterima berdasarkan pengukuran hardware;
hasil sintetis di atas tidak menetapkan akurasi anatomis perangkat.

## Diagnosis pronasi/deviation yang tidak bergerak

Kalibrasi pengguna yang diperiksa ketika masalah dilaporkan menyebut
`mode=relative`; mode ini mengunci pro_sup pada nol. Command GUI sekarang
mengaktifkan `forearm-twist`. Restart hanya tahap viewer/retargeting, lalu
kembalikan tangan ke pose netral untuk kalibrasi sesi baru.

Jika deviation masih tidak mengikuti, command terminal dengan log adalah:

```powershell
.\.venv\Scripts\python.exe real_time\run_retargeting.py --wrist-mode forearm-twist --wrist-log outputs\wrist_evaluation\live_metrics.csv
```

Lakukan pronasi/supinasi lalu radial/ulnar secara terpisah. CSV di-append dengan
`session_id` berbeda tiap proses; baris merepresentasikan pose yang dikonsumsi
viewer, bukan seluruh input RGMP. Periksa kolom:

- `wrist_input_forearm_twist_deg` dibanding `pro_sup_angle_deg`.
- `wrist_input_deviation_component_deg` dibanding `deviation_angle_deg`.
- `wrist_input_relative_deg`, `wrist_relative_error_deg`, dan `deviation_at_limit`.

Komponen deviation/flexion adalah proyeksi rotation vector delta relatif pada
axis FK netral, **hanya diagnostik**; nilainya bukan sudut joint target exact
untuk gerakan gabungan. Jika hand dan forearm bergerak bersama sehingga relatif
hampir nol, pipeline tidak memiliki sinyal wrist relatif untuk dipetakan.
Jika joint mencapai batas, `deviation_at_limit=True` (jarak ke batas <0,5°).
Jangan menaikkan gain atau mengubah axis berdasarkan dugaan sebelum memeriksa
sinyal ini. Log yang sama mempertahankan error jari dan waktu komputasi.

Replay 2.073 frame pada rekaman lama `recordings/solved_hand.csv`, memakai
mean netral 0–0,5 detik, menghasilkan pro_sup sekitar [-0,44°,10,86°] dan
deviation [-10°,0,62°]. Komponen input deviation mencapai -19,42°, sementara
batas model -10°; ini menunjukkan saturasi pada rekaman tersebut, bukan joint
terkunci. Ringkasan tersimpan di `recording_diagnostic.json`. Rekaman itu bukan
sesi uji radial/ulnar khusus dan tidak membuktikan penyebab sesi hardware terbaru.
