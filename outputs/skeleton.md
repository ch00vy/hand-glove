# Skeleton teramati dari `joints_local`

Parent berasal dari `reference_frame` CSV. `LTP_NED` adalah frame dunia, bukan joint.

```text
right_fore_arm
  right_hand
    right_finger1_metacarpal
      right_finger1_proximal
        right_finger1_distal
          right_finger1_tip
    right_finger2_metacarpal
      right_finger2_proximal
        right_finger2_medial
          right_finger2_distal
            right_finger2_tip
    right_finger3_metacarpal
      right_finger3_proximal
        right_finger3_medial
          right_finger3_distal
            right_finger3_tip
    right_finger4_metacarpal
      right_finger4_proximal
        right_finger4_medial
          right_finger4_distal
            right_finger4_tip
    right_finger5_metacarpal
      right_finger5_proximal
        right_finger5_medial
          right_finger5_distal
            right_finger5_tip
```

## Validasi

- Joint lokal: 26
- Parent konsisten: 26/26
- `ORIENTATION FLOAT[4]` hanya memiliki rotasi; offset lokal tidak dapat disimpulkan dari CSV ini.
- Nama jari dan sendi di atas adalah nama solver, belum label anatomi tervalidasi.
