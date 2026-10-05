# IMU 3D Visualizer

1. Flash firmware proyek ini ke BlackPill dan tutup Serial Monitor PlatformIO.
2. Dari root proyek, jalankan `python3 -m http.server 8000 -d tools/imu-visualizer`.
3. Buka `http://localhost:8000` memakai Chrome atau Edge.
4. Pilih **Hubungkan USB**, kemudian pilih perangkat `BLACKPILL_F411CE CDC in FS Mode`.

Halaman menghitung roll/pitch dengan complementary filter dari gyro dan
accelerometer. Yaw hanya diintegrasikan dari gyro sehingga akan mengalami drift;
itu normal karena BMI160 tidak mempunyai magnetometer.

## Kalibrasi

Kalibrasi berjalan di firmware dan langsung diterapkan ke stream IMU selama
board menyala. Nilainya belum disimpan permanen ke flash.

1. Letakkan FC di meja yang stabil lalu klik **Kalibrasi gyro**. Jangan sentuh
   FC selama lima detik.
2. Klik **Calibrate Accelerometer**, lalu ikuti enam posisi yang ditampilkan:
   normal, terbalik, hidung ke atas, ekor ke atas, sisi kanan, dan sisi kiri.
   Tahan diam lalu klik *Capture position*. Firmware otomatis mengenali sumbu
   dan arah gravitasi, sehingga pengguna tidak perlu memilih `X+`/`X-`.
3. Setelah enam posisi, nilai accelerometer yang sudah terkoreksi akan
   digunakan otomatis. Saat FC datar sesuai orientasi sensor, salah satu sumbu
   akan mendekati `+1 g` atau `-1 g` dan dua sumbu lainnya mendekati nol.

Jika arah kubus tidak cocok dengan posisi fisik board, ubah pemetaan sumbu pada
firmware setelah orientasi pemasangan IMU pada frame quadcopter sudah ditetapkan.
