# Eksperimen budget optimizer IK myoHand

Dijalankan 13 November 2026 memakai Python 3.12.1 dan virtual environment proyek.

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_ik.py --input recordings\solved_hand.csv --max-nfev 8 12 20 30
```

Setiap skenario memproses seluruh 2.073 frame rekaman (24,864 detik), dengan 42 frame netral pada interval 0–0,5 detik. Target, registrasi, bobot objective, batas joint, toleransi, dan pose awal skenario identik. Fitter direset antar skenario; di dalam skenario digunakan solusi sebelumnya sebagai warm start. Tidak ada interpolasi atau dropped frames. Pemeriksaan CSV hasil mengonfirmasi 8.292 baris diagnostik, urutan timestamp identik, dan metrik finite.

| Skenario | max_nfev | Mean tip (mm) | P95 tip (mm) | Sukses (%) | Batas evaluasi/status=0 (%) | Mean nfev |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A | 8 | 6,229113 | 9,934922 | 75,49 | 24,51 | 5,89 |
| B | 12 | 6,228977 | 9,934922 | 83,70 | 16,30 | 6,74 |
| C | 20 | 6,228970 | 9,934922 | 99,57 | 0,43 | 7,27 |
| D | 30 | 6,228969 | 9,934922 | 99,90 | 0,10 | 7,29 |

| max_nfev | Mean compute (ms) | Median compute (ms) | P95 compute (ms) | Effective IK FPS |
| ---: | ---: | ---: | ---: | ---: |
| 8 | 4,53 | 4,45 | 7,25 | 220,8 |
| 12 | 4,86 | 4,39 | 8,71 | 205,6 |
| 20 | 5,38 | 4,66 | 10,84 | 185,8 |
| 30 | 5,51 | 4,60 | 11,35 | 181,3 |

Rekomendasi eksperimen: **max_nfev=20** untuk kompromi konvergensi dan compute time. Dibanding baseline 12, konvergensi naik 15,87 poin persentase (338 menjadi 9 frame dengan status=0), dengan tambahan mean compute sekitar 0,52 ms atau 10,7%. Peningkatan ke 30 hanya menambah 0,34 poin persentase keberhasilan (9 menjadi 2 frame dengan status=0), sedangkan P95 compute naik dari 10,84 ke 11,35 ms. Perubahan mean error 20 ke 30 sekitar 0,000001 mm, sehingga manfaat akurasinya tidak berarti pada rekaman ini.

Error yang praktis sama meskipun status konvergensi berbeda menunjukkan bahwa solver sering sudah dekat solusi sebelum toleransi penghentian tercapai. Status sukses mengukur pemenuhan toleransi numerik; status=0 tidak otomatis berarti pose buruk. Untuk prioritas latensi ketat, budget 8/12 tetap masuk akal pada rekaman ini, dengan konsekuensi lebih banyak penghentian karena batas evaluasi.

P95 error dihitung atas mean lima error fingertip per frame, termasuk frame netral. Ini error terhadap target displacement yang telah diregistrasi/diskalakan, bukan ground truth anatomis. Waktu compute mengukur hanya pemanggilan least_squares, termasuk residual/Jacobian. Effective IK FPS = 1000 / mean compute ms, **estimasi kemampuan komputasi offline, bukan FPS streaming end-to-end**. Parsing, kalibrasi, pembuatan target, error akhir, logging, antrean, serta rendering tidak masuk waktu compute. Untuk input sekitar 83 Hz (interval 12 ms), P95 compute 20 masih di bawah interval input, tetapi menyisakan margin terbatas untuk pekerjaan pipeline lainnya; throughput live harus diukur langsung.

Pengukuran memakai satu lintasan per skenario dengan warm-up identik; selisih waktu kecil bisa dipengaruhi beban CPU dan urutan skenario. Default real-time tetap 12. Rekomendasi 20 dapat diuji tanpa mengubah default:

```powershell
.\.venv\Scripts\python.exe real_time\run_retargeting.py --host 127.0.0.1 --port 12277 --max-ik-evaluations 20
```

GUI, objective, Jacobian, strategi antrean, SDK eksternal, serta rekaman asli dipertahankan. Diagnostik tambahan memakai timer resolusi tinggi untuk membedakan waktu optimizer dari latency callback ke pose.

Validasi: 24 unit test proyek dan 3 tes pipeline real-time lulus. Metadata parameter, versi, hash input/target, pose awal, dan registrasi berada di benchmark_metadata.json; data lengkap di benchmark_summary.csv dan benchmark_per_frame.csv.
