# Status Pengembangan — Custom STM32 Flight Controller

Dokumen ini mencatat kondisi proyek saat ini, hasil pengujian, batasan yang
masih ada, dan urutan pengembangan berikutnya. Ini adalah flight controller
custom berbasis **STM32F411CEU6 BlackPill**, bukan PX4.

## Ringkasan saat ini

Fondasi flight controller sudah berjalan di bench:

- BMI160 terbaca melalui SPI dan mengirim telemetry USB CDC.
- Roll, pitch, dan yaw relatif dihitung dengan low-pass + complementary filter.
- Receiver FlySky FS-iA6B sudah bind dan iBUS terbaca melalui PA3.
- GCS browser menampilkan Radio realtime pada 25 Hz.
- Konfigurasi channel arm dapat dipilih dari GCS (CH1–CH14, active-high/low).
- Output ESC PWM Quad-X pada PB6–PB9 telah berhasil diuji satu per satu.
- Motor-test tersedia di GCS dengan slider bench-test 1000–2200 µs dan
  auto-stop 3 detik.

Sistem **belum flight-ready**. Mixer fisik, arah koreksi gyro, failsafe motor,
pre-arm checks, battery monitor, dan PID masih harus divalidasi sebelum
propeller dipasang.

## Hardware dan wiring

| Perangkat | Pin perangkat | STM32F411CEU6 | Keterangan |
| --- | --- | --- | --- |
| GY-BMI160 | VCC | 3V3 | Jangan gunakan 5 V |
| GY-BMI160 | GND | GND | Ground bersama |
| GY-BMI160 | SCK | PA5 | SPI1 SCK |
| GY-BMI160 | SDI/MOSI | PA7 | SPI1 MOSI |
| GY-BMI160 | SDO/MISO | PA6 | SPI1 MISO |
| GY-BMI160 | CS | PA4 | Chip select |
| FS-iA6B | i-BUS signal | PA3 | USART2 RX; jangan gunakan PPM/CH1 atau SENS |
| FS-iA6B | VCC | 5V | Catu receiver |
| FS-iA6B | GND | GND | Ground bersama |
| ESC M1 | Signal | PB6 | TIM4_CH1 |
| ESC M2 | Signal | PB7 | TIM4_CH2 |
| ESC M3 | Signal | PB8 | TIM4_CH3 |
| ESC M4 | Signal | PB9 | TIM4_CH4 |
| Battery monitor | Divider midpoint | PA0 | Belum diaktifkan secara default |

Orientasi yang dipakai proyek ini: sisi **7-pin BMI160 adalah arah depan/nose**.

## Konfigurasi motor Quad-X

Dilihat dari atas, dengan nose menghadap depan:

```text
M4 front-left       M1 front-right

M3 rear-left        M2 rear-right
```

| Motor | Posisi | Pin | Arah target |
| --- | --- | --- | --- |
| M1 | Front-right | PB6 / TIM4_CH1 | CW |
| M2 | Rear-right | PB7 / TIM4_CH2 | CCW |
| M3 | Rear-left | PB8 / TIM4_CH3 | CW |
| M4 | Front-left | PB9 / TIM4_CH4 | CCW |

Jika putaran fisik motor salah, tukar dua kabel fase antara motor dan ESC.
Jangan membalik arah motor pertama kali melalui perubahan mixer firmware.

## Firmware saat ini

Komponen utama pada `src/main.cpp`:

- Control loop IMU: 500 Hz.
- BMI160 ODR: 800 Hz.
- Telemetry attitude USB: 50 Hz.
- Telemetry radio USB: 25 Hz.
- iBUS: 115200 baud, 14 channel, timeout failsafe 100 ms.
- Arm default: CH5, `HIGH` di atas 1500 µs; dapat dikonfigurasi dari GCS dan
  kembali default setelah board restart.
- PWM ESC: TIM4, 400 Hz.
- Mixer: Quad-X.
- Controller: basic rate PID roll/pitch/yaw.
- Battery input: PA0 melalui divider 100 kΩ : 10 kΩ; fitur masih disabled.

Yaw bersifat relatif dan akan drift karena BMI160 tidak memiliki magnetometer.

## GCS browser

Jalankan local GCS:

```bash
cd "/home/ros2class/Downloads/Project FC/stm32FC"
python3 -m http.server 8000 -d tools/imu-visualizer
```

Buka `http://localhost:8000` menggunakan Chrome/Chromium, lalu pilih STM32
melalui tombol **Connect**.

Modul yang tersedia:

- **Fly View** — map, HUD attitude, heading relatif, flight status.
- **Sensors** — visualisasi attitude dan kalibrasi gyro/accelerometer.
- **Radio** — monitor CH1–CH8, visual stick, serta konfigurasi channel arm.
- **Battery** — status dan peringatan wiring divider PA0.
- **Actuators** — diagram motor Quad-X, output motor, dan slider PWM test.
- **Parameters** — nilai firmware dasar yang masih read-only.

## Hasil pengujian yang sudah lulus

- [x] USB CDC STM32 terdeteksi.
- [x] BMI160 terdeteksi via SPI.
- [x] Kalibrasi gyro dan accelerometer enam posisi.
- [x] Visualisasi roll/pitch/yaw GCS.
- [x] Binding FS-i6 ke FS-iA6B.
- [x] Receiver iBUS terbaca pada PA3 (`RX_OK`).
- [x] Radio GCS menampilkan channel dan stick secara realtime.
- [x] Arm switch dapat dikonfigurasi dan diuji.
- [x] PWM ESC dan motor M1–M4 dapat diuji tanpa propeller.

## Safety penting

1. Lepaskan seluruh propeller untuk semua motor-test dan mixer-test di meja.
2. ESC harus mendapat daya dari baterai/PSU yang sesuai; signal ESC dan GND
   harus common dengan STM32.
3. Jika STM32 diberi daya dari USB, jangan juga menyatukan output 5 V BEC ESC
   ke pin 5V STM32. Gunakan satu sumber 5 V untuk board.
4. Slider 2200 µs hanya untuk bench test sesuai permintaan proyek. Mixer
   penerbangan normal tetap dibatasi 2000 µs. Nilai 2200 µs dapat berada di
   luar rentang standar sebagian ESC, sehingga jangan gunakan dengan propeller.
5. Saat motor test aktif, firmware memaksa mode flight mixer disarm.

## Tahapan yang wajib sebelum propeller dipasang

### 1. Verifikasi motor order dan arah putar

Tanpa propeller, pastikan M1–M4 berada pada posisi fisik yang benar dan berputar
sesuai arah target di atas.

### 2. Verifikasi mixer dari transmitter

Tanpa propeller, arm normal dengan throttle serendah mungkin. Periksa nilai
output motor saat memberi input roll, pitch, dan yaw. Setiap sumbu harus
mengubah pasangan motor yang sesuai.

### 3. Verifikasi arah koreksi gyro

Tanpa propeller dan dengan throttle rendah, miringkan frame secara manual.
Koreksi motor harus **melawan** arah gerakan, bukan memperparahnya. Jika salah,
jangan pasang propeller; perbaiki tanda gyro atau mixer terlebih dahulu.

### 4. Verifikasi failsafe

Dengan receiver valid dan FC armed pada throttle rendah:

- matikan transmitter;
- atau lepas signal iBUS;
- pastikan status menjadi `FAILSAFE` dalam sekitar 100 ms;
- pastikan FC disarm dan output motor kembali 1000 µs.

### 5. Implementasikan pre-arm checks

Sebelum penerbangan, firmware harus menolak arm bila salah satu kondisi berikut
tidak aman:

- BMI160 tidak healthy;
- receiver bukan `RX_OK`;
- throttle tidak rendah;
- motor-test aktif;
- gyro belum dikalibrasi;
- accelerometer belum dikalibrasi;
- battery monitor aktif tetapi baterai tidak valid/rendah.

### 6. Tuning PID

Tuning dimulai dengan `Ki = 0`, lalu dilakukan berurutan per sumbu:

```text
Kp roll → Kp pitch → Kd roll/pitch → Ki kecil → yaw
```

Gunakan test jig atau bench yang aman. Hover pertama hanya dilakukan setelah
mixer, koreksi gyro, dan failsafe lulus.

## Roadmap MAVLink / QGroundControl

MAVLink dapat dimulai setelah mixer dan failsafe tervalidasi. Tahap pertama
harus **read-only telemetry**; QGroundControl belum boleh diberi wewenang
arm/motor command pada tahap ini.

Urutan pesan MAVLink yang direkomendasikan:

1. `HEARTBEAT`
2. `SYS_STATUS`
3. `ATTITUDE`
4. `RAW_IMU`
5. `RC_CHANNELS`
6. `SERVO_OUTPUT_RAW`
7. `BATTERY_STATUS`
8. `PARAM_VALUE` / `PARAM_SET`
9. `COMMAND_ACK`
10. command calibration dan actuator setelah safety tervalidasi

Target awal MAVLink adalah QGroundControl berhasil connect dan hanya membaca
telemetry. Target berikutnya adalah parameter persistence, calibration command,
dan baru kemudian command arm/actuator dari QGC.

## Perintah build dan upload

Build firmware:

```bash
pio run -e stm32f411ceu6_stlink
```

Upload melalui ST-LINK:

```bash
pio run -e stm32f411ceu6_stlink -t upload
```

Upload USB DFU membutuhkan mode bootloader BOOT0 dan environment utama:

```bash
pio run -e stm32f411ceu6 -t upload
```
