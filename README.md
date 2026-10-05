# STM32 Custom Flight Controller

Flight controller custom untuk **STM32F411CEU6 BlackPill**. Proyek ini
mengintegrasikan BMI160, receiver FlySky iBUS, Quad-X mixer, PWM ESC, dan
Ground Control Station (GCS) browser untuk bench testing.

> Status: **bench-tested, belum flight-ready.** Jangan memasang propeller
> sebelum motor order, arah koreksi gyro, failsafe, pre-arm checks, dan PID
> telah tervalidasi.

## Fitur saat ini

- BMI160 melalui SPI1: gyro + accelerometer 6-DoF.
- IMU control loop 500 Hz dan telemetry attitude USB 50 Hz.
- Low-pass dan complementary filter untuk roll/pitch/yaw relatif.
- Kalibrasi gyro serta accelerometer enam posisi.
- FlySky FS-iA6B iBUS melalui USART2 RX / PA3.
- Telemetry Radio realtime 25 Hz dengan monitor channel GCS.
- Arm switch runtime yang dapat dipilih CH1–CH14 dan active-high/low.
- Failsafe iBUS dasar dengan timeout 100 ms.
- PWM ESC 400 Hz pada TIM4 dan Quad-X mixer.
- Bench actuator test per motor melalui slider PWM 1000–2200 µs.
- GCS browser: Fly View, Sensors, Radio, Battery, Actuators, Parameters.

## Hardware

| Perangkat | Pin perangkat | STM32F411CEU6 |
| --- | --- | --- |
| GY-BMI160 | VCC / GND | 3V3 / GND |
| GY-BMI160 | SCK / MOSI / MISO / CS | PA5 / PA7 / PA6 / PA4 |
| FS-iA6B | i-BUS signal | PA3 |
| FS-iA6B | VCC / GND | 5V / GND |
| ESC M1–M4 | Signal | PB6 / PB7 / PB8 / PB9 |
| Battery divider midpoint | ADC | PA0 |

Gunakan port **i-BUS** receiver, bukan `PPM/CH1` atau `SENS`. Semua perangkat
harus memiliki ground bersama.

## Quad-X mapping

Orientasi depan mengikuti sisi **7-pin GY-BMI160**.

```text
M4 front-left       M1 front-right

M3 rear-left        M2 rear-right
```

| Motor | STM32 pin | Timer | Target rotation |
| --- | --- | --- | --- |
| M1 | PB6 | TIM4_CH1 | CW |
| M2 | PB7 | TIM4_CH2 | CCW |
| M3 | PB8 | TIM4_CH3 | CW |
| M4 | PB9 | TIM4_CH4 | CCW |

## Quick start

### Build dan upload firmware

Gunakan ST-LINK:

```bash
pio run -e stm32f411ceu6_stlink -t upload
```

Atau USB DFU (masuk bootloader menggunakan BOOT0 + NRST terlebih dahulu):

```bash
pio run -e stm32f411ceu6 -t upload
```

### Menjalankan GCS lokal

```bash
python3 -m http.server 8000 -d tools/imu-visualizer
```

Buka `http://localhost:8000` dengan Chrome/Chromium, klik **Connect**, dan
pilih USB serial STM32.

## Safety

- Lepaskan seluruh propeller pada semua bench dan actuator test.
- Jika STM32 diberi daya USB, jangan menghubungkan output 5 V BEC ESC ke pin
  5V STM32 secara bersamaan.
- GND ESC dan STM32 harus common.
- Slider 2200 µs hanya untuk bench test; mixer penerbangan normal maksimum
  2000 µs. Nilai 2200 µs bisa berada di luar spesifikasi sebagian ESC.
- BMI160 tidak memiliki magnetometer; yaw bersifat relatif dan akan drift.

## Dokumentasi

- [Status pengembangan, hasil test, safety, dan roadmap MAVLink](docs/DEVELOPMENT_STATUS.md)
- [Panduan visualizer / GCS](tools/imu-visualizer/README.md)

## Roadmap singkat

1. Validasi motor order, arah putar, mixer, dan koreksi gyro tanpa propeller.
2. Uji failsafe motor serta tambah pre-arm checks.
3. Tuning rate PID dan hover test terkendali.
4. Implementasi MAVLink read-only telemetry.
5. Koneksi telemetry ke QGroundControl.

## License

Proyek ini berada di bawah [MIT License](LICENSE).

Copyright © 2026 Robotic Telkom University.
