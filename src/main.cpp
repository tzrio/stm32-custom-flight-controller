#include <Arduino.h>
#include <HardwareTimer.h>
#include <SPI.h>
#include <string.h>

/* BlackPill F411CE: LED bawaan PC13 aktif-low. */
constexpr uint32_t LED_PIN = PC13;

/* BMI160 / SPI1: CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7. */
constexpr uint32_t BMI160_CS_PIN = PA4;
constexpr uint8_t BMI160_CHIP_ID = 0xD1;
constexpr uint8_t BMI160_REG_CHIP_ID = 0x00;
constexpr uint8_t BMI160_REG_ERR = 0x02;
constexpr uint8_t BMI160_REG_PMU_STATUS = 0x03;
constexpr uint8_t BMI160_REG_GYR_X_LSB = 0x0C;
constexpr uint8_t BMI160_REG_ACC_CONF = 0x40;
constexpr uint8_t BMI160_REG_ACC_RANGE = 0x41;
constexpr uint8_t BMI160_REG_GYR_CONF = 0x42;
constexpr uint8_t BMI160_REG_GYR_RANGE = 0x43;
constexpr uint8_t BMI160_REG_CMD = 0x7E;
constexpr uint32_t IMU_SAMPLE_PERIOD_US = 2000;       // control loop 500 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;          // USB telemetry 50 Hz
constexpr uint32_t RADIO_TELEMETRY_PERIOD_MS = 40;    // Radio GCS 25 Hz
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 2500;   // 5 detik pada 500 Hz
constexpr uint16_t ACC_CALIBRATION_SAMPLES = 1000;    // 2 detik tiap posisi
constexpr float ACC_LSB_PER_G = 16384.0f;
constexpr float GYRO_LSB_PER_DPS = 65.6f;             // ±500 dps
constexpr uint8_t IBUS_CHANNEL_COUNT = 14;
constexpr uint8_t IBUS_PACKET_SIZE = 32;
constexpr uint32_t RECEIVER_TIMEOUT_MS = 100;
constexpr uint8_t IBUS_THROTTLE_CHANNEL = 2;
constexpr uint16_t THROTTLE_LOW_US = 1050;
constexpr uint16_t ARM_SWITCH_ON_US = 1500;

/* ESC / Quad-X: output PWM TIM4 channels 1..4 pada PB6..PB9.
 * Lepaskan SEMUA propeller sebelum mode test motor. */
constexpr uint32_t MOTOR_1_PIN = PB6;  // front-right, CW
constexpr uint32_t MOTOR_2_PIN = PB7;  // rear-right, CCW
constexpr uint32_t MOTOR_3_PIN = PB8;  // rear-left, CW
constexpr uint32_t MOTOR_4_PIN = PB9;  // front-left, CCW
constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_IDLE_US = 1050;
constexpr uint16_t ESC_MAX_US = 2000;
constexpr uint16_t MOTOR_TEST_MAX_US = 2200;  // bench test only; flight mixer stays at 2000 µs
constexpr uint32_t MOTOR_TEST_TIMEOUT_MS = 3000;
constexpr float MAX_RATE_DPS = 220.0f;
constexpr float RATE_KP[3] = {0.055f, 0.055f, 0.080f};
constexpr float RATE_KI[3] = {0.030f, 0.030f, 0.035f};
constexpr float RATE_KD[3] = {0.0012f, 0.0012f, 0.0000f};

// PA0 wajib diberi voltage divider; jangan pernah hubungkan baterai langsung.
constexpr uint32_t BATTERY_ADC_PIN = PA0;
constexpr bool BATTERY_MONITOR_ENABLED = false;
constexpr float BATTERY_DIVIDER_RATIO = 11.0f;  // 100kΩ atas, 10kΩ bawah
constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;

enum class CalibrationMode : uint8_t {
    None,
    Gyro,
    AccXPositive,
    AccXNegative,
    AccYPositive,
    AccYNegative,
    AccZPositive,
    AccZNegative,
    AccAuto,
};

static CalibrationMode calibrationMode = CalibrationMode::None;
static int64_t calibrationSum[3] = {0, 0, 0};
static uint16_t calibrationCount = 0;
static float gyroBias[3] = {0.0f, 0.0f, 0.0f};
static float accBias[3] = {0.0f, 0.0f, 0.0f};
static float accScale[3] = {1.0f, 1.0f, 1.0f};
static float accFaceMean[6][3] = {};
static bool accFaceCaptured[6] = {};
static char commandBuffer[32] = {};
static uint8_t commandLength = 0;
static float gyroLowPass[3] = {};
static float accLowPass[3] = {};
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float yawDeg = 0.0f;
static bool attitudeInitialized = false;
static Uart IbusSerial(PA3, PA2);  // USART2: RX, TX (TX tidak digunakan)
static uint8_t ibusBuffer[IBUS_PACKET_SIZE] = {};
static uint8_t ibusIndex = 0;
static uint16_t receiverChannels[IBUS_CHANNEL_COUNT] = {};
static uint32_t lastReceiverFrameMs = 0;
static bool receiverValid = false;
static bool failsafeActive = true;
static bool armed = false;
// Runtime arm parameter: default CH5, active-high. Resets to this default on boot.
static uint8_t armChannelIndex = 4;
static bool armSwitchHigh = true;
static float batteryVoltage = 0.0f;
static HardwareTimer *motorTimer = nullptr;
static uint16_t motorOutputUs[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
static bool motorTestEnabled = false;
static uint8_t motorTestIndex = 0xFF;
static uint16_t motorTestPulseUs = ESC_MIN_US;
static uint32_t motorTestStartedMs = 0;
static float rateIntegral[3] = {};
static float previousRateError[3] = {};

static SPISettings bmi160_spi(1000000, MSBFIRST, SPI_MODE0);

static float constrainFloat(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static void writeMotors(const uint16_t output[4])
{
    for (uint8_t index = 0; index < 4; ++index) motorOutputUs[index] = output[index];
    if (!motorTimer) return;
    motorTimer->setCaptureCompare(1, output[0], MICROSEC_COMPARE_FORMAT);
    motorTimer->setCaptureCompare(2, output[1], MICROSEC_COMPARE_FORMAT);
    motorTimer->setCaptureCompare(3, output[2], MICROSEC_COMPARE_FORMAT);
    motorTimer->setCaptureCompare(4, output[3], MICROSEC_COMPARE_FORMAT);
}

static void stopMotors()
{
    const uint16_t stopped[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
    writeMotors(stopped);
    memset(rateIntegral, 0, sizeof(rateIntegral));
    memset(previousRateError, 0, sizeof(previousRateError));
}

static void beginMotors()
{
    motorTimer = new HardwareTimer(TIM4);
    motorTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_1_PIN);
    motorTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_2_PIN);
    motorTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_3_PIN);
    motorTimer->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_4_PIN);
    motorTimer->setOverflow(400, HERTZ_FORMAT);  // ESC protocol: PWM 400 Hz
    stopMotors();
    motorTimer->resume();
}

static void updateMotors(const int32_t gyro[3], float dt, uint32_t nowMs)
{
    if (motorTestIndex != 0xFF && nowMs - motorTestStartedMs >= MOTOR_TEST_TIMEOUT_MS) {
        motorTestIndex = 0xFF;
        motorTestPulseUs = ESC_MIN_US;
    }
    if (motorTestIndex != 0xFF) {
        uint16_t testOutput[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
        testOutput[motorTestIndex] = motorTestPulseUs;
        writeMotors(testOutput);
        return;
    }
    if (!armed || failsafeActive || calibrationMode != CalibrationMode::None) {
        stopMotors();
        return;
    }

    const float throttle = constrainFloat(
        (static_cast<float>(receiverChannels[IBUS_THROTTLE_CHANNEL]) - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    const float stickRoll = constrainFloat((static_cast<float>(receiverChannels[0]) - 1500.0f) / 500.0f, -1.0f, 1.0f);
    const float stickPitch = constrainFloat((static_cast<float>(receiverChannels[1]) - 1500.0f) / 500.0f, -1.0f, 1.0f);
    const float stickYaw = constrainFloat((static_cast<float>(receiverChannels[3]) - 1500.0f) / 500.0f, -1.0f, 1.0f);
    const float desiredRate[3] = {stickRoll * MAX_RATE_DPS, stickPitch * MAX_RATE_DPS, stickYaw * MAX_RATE_DPS};
    float correction[3] = {};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        const float measuredRate = static_cast<float>(gyro[axis]) / GYRO_LSB_PER_DPS;
        const float error = desiredRate[axis] - measuredRate;
        rateIntegral[axis] = constrainFloat(rateIntegral[axis] + error * dt, -150.0f, 150.0f);
        const float derivative = (error - previousRateError[axis]) / dt;
        previousRateError[axis] = error;
        correction[axis] = RATE_KP[axis] * error + RATE_KI[axis] * rateIntegral[axis] + RATE_KD[axis] * derivative;
    }
    const float base = ESC_IDLE_US + throttle * (ESC_MAX_US - ESC_IDLE_US);
    // Quad-X: M1 front-right CW, M2 rear-right CCW, M3 rear-left CW, M4 front-left CCW.
    const float mixed[4] = {
        base - correction[0] - correction[1] + correction[2],
        base - correction[0] + correction[1] - correction[2],
        base + correction[0] + correction[1] + correction[2],
        base + correction[0] - correction[1] - correction[2],
    };
    uint16_t output[4];
    for (uint8_t index = 0; index < 4; ++index) {
        output[index] = static_cast<uint16_t>(constrainFloat(mixed[index], ESC_IDLE_US, ESC_MAX_US));
    }
    writeMotors(output);
}

static void bmi160Select()
{
    digitalWrite(BMI160_CS_PIN, LOW);
}

static void bmi160Deselect()
{
    digitalWrite(BMI160_CS_PIN, HIGH);
}

static uint8_t bmi160Read(uint8_t reg)
{
    SPI.beginTransaction(bmi160_spi);
    bmi160Select();
    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0);
    bmi160Deselect();
    SPI.endTransaction();
    return value;
}

static void bmi160Write(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(bmi160_spi);
    bmi160Select();
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);
    bmi160Deselect();
    SPI.endTransaction();
}

static void bmi160ReadBurst(uint8_t reg, uint8_t *data, size_t length)
{
    SPI.beginTransaction(bmi160_spi);
    bmi160Select();
    SPI.transfer(reg | 0x80);
    for (size_t index = 0; index < length; ++index) {
        data[index] = SPI.transfer(0);
    }
    bmi160Deselect();
    SPI.endTransaction();
}

static int16_t int16LE(const uint8_t *bytes)
{
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8));
}

static const char *calibrationName(CalibrationMode mode)
{
    switch (mode) {
    case CalibrationMode::Gyro: return "GYRO";
    case CalibrationMode::AccXPositive: return "X+";
    case CalibrationMode::AccXNegative: return "X-";
    case CalibrationMode::AccYPositive: return "Y+";
    case CalibrationMode::AccYNegative: return "Y-";
    case CalibrationMode::AccZPositive: return "Z+";
    case CalibrationMode::AccZNegative: return "Z-";
    case CalibrationMode::AccAuto: return "AUTO";
    default: return "NONE";
    }
}

static int accFaceIndex(CalibrationMode mode)
{
    return static_cast<int>(mode) - static_cast<int>(CalibrationMode::AccXPositive);
}

static void startCalibration(CalibrationMode mode)
{
    calibrationMode = mode;
    calibrationCount = 0;
    memset(calibrationSum, 0, sizeof(calibrationSum));
    Serial.print("CAL,START,");
    Serial.println(calibrationName(mode));
}

static void applyAccelerometerCalibration()
{
    for (uint8_t axis = 0; axis < 3; ++axis) {
        const float positive = accFaceMean[axis * 2][axis];
        const float negative = accFaceMean[axis * 2 + 1][axis];
        accBias[axis] = (positive + negative) * 0.5f;
        const float halfSpan = (positive - negative) * 0.5f;
        if (fabsf(halfSpan) > 1000.0f) {
            accScale[axis] = ACC_LSB_PER_G / halfSpan;
        }
    }
    Serial.print("CAL,RESULT,ACC,");
    for (uint8_t axis = 0; axis < 3; ++axis) {
        Serial.print(accBias[axis], 2);
        Serial.print(',');
        Serial.print(accScale[axis], 6);
        if (axis < 2) Serial.print(',');
    }
    Serial.println();
}

static void collectCalibration(const int16_t gyro[3], const int16_t acc[3])
{
    if (calibrationMode == CalibrationMode::None) return;

    const int16_t *sample = calibrationMode == CalibrationMode::Gyro ? gyro : acc;
    const uint16_t target = calibrationMode == CalibrationMode::Gyro
                                ? GYRO_CALIBRATION_SAMPLES
                                : ACC_CALIBRATION_SAMPLES;
    for (uint8_t axis = 0; axis < 3; ++axis) calibrationSum[axis] += sample[axis];
    ++calibrationCount;
    if (calibrationCount < target) return;

    if (calibrationMode == CalibrationMode::Gyro) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            gyroBias[axis] = static_cast<float>(calibrationSum[axis]) / target;
        }
        Serial.print("CAL,RESULT,GYRO,");
        Serial.print(gyroBias[0], 2); Serial.print(',');
        Serial.print(gyroBias[1], 2); Serial.print(',');
        Serial.println(gyroBias[2], 2);
        attitudeInitialized = false;
    } else {
        int face = -1;
        if (calibrationMode == CalibrationMode::AccAuto) {
            int dominantAxis = 0;
            for (uint8_t axis = 1; axis < 3; ++axis) {
                const int64_t candidate = calibrationSum[axis] < 0 ? -calibrationSum[axis] : calibrationSum[axis];
                const int64_t current = calibrationSum[dominantAxis] < 0 ? -calibrationSum[dominantAxis] : calibrationSum[dominantAxis];
                if (candidate > current) {
                    dominantAxis = axis;
                }
            }
            const float dominantMean = static_cast<float>(calibrationSum[dominantAxis]) / target;
            face = dominantAxis * 2 + (dominantMean >= 0.0f ? 0 : 1);
        } else {
            face = accFaceIndex(calibrationMode);
        }
        if (accFaceCaptured[face]) {
            Serial.println("CAL,ERROR,DUPLICATE_FACE");
            calibrationMode = CalibrationMode::None;
            return;
        }
        for (uint8_t axis = 0; axis < 3; ++axis) {
            accFaceMean[face][axis] = static_cast<float>(calibrationSum[axis]) / target;
        }
        accFaceCaptured[face] = true;
        Serial.print("CAL,RESULT,FACE,");
        Serial.println(calibrationName(static_cast<CalibrationMode>(static_cast<int>(CalibrationMode::AccXPositive) + face)));
        bool allFacesReady = true;
        for (bool captured : accFaceCaptured) allFacesReady &= captured;
        if (allFacesReady) {
            applyAccelerometerCalibration();
            attitudeInitialized = false;
        }
    }
    calibrationMode = CalibrationMode::None;
}

static void updateAttitude(const int32_t gyro[3], const int32_t acc[3], float dt)
{
    constexpr float gyroFilterGain = 0.18f;
    constexpr float accFilterGain = 0.12f;
    constexpr float complementaryAlpha = 0.995f;
    constexpr float radiansToDegrees = 57.2957795f;

    for (uint8_t axis = 0; axis < 3; ++axis) {
        const float gyroDps = static_cast<float>(gyro[axis]) / GYRO_LSB_PER_DPS;
        const float accG = static_cast<float>(acc[axis]) / ACC_LSB_PER_G;
        if (!attitudeInitialized) {
            gyroLowPass[axis] = gyroDps;
            accLowPass[axis] = accG;
        } else {
            gyroLowPass[axis] += gyroFilterGain * (gyroDps - gyroLowPass[axis]);
            accLowPass[axis] += accFilterGain * (accG - accLowPass[axis]);
        }
    }

    const float rollAcc = atan2f(accLowPass[1], accLowPass[2]) * radiansToDegrees;
    // Sensor pitch direction pada GY-BMI160 terbalik terhadap tampilan GCS.
    const float pitchAcc = atan2f(accLowPass[0], hypotf(accLowPass[1], accLowPass[2])) * radiansToDegrees;
    if (!attitudeInitialized) {
        rollDeg = rollAcc;
        pitchDeg = pitchAcc;
        yawDeg = 0.0f;
        attitudeInitialized = true;
        return;
    }

    rollDeg = complementaryAlpha * (rollDeg + gyroLowPass[0] * dt) +
              (1.0f - complementaryAlpha) * rollAcc;
    pitchDeg = complementaryAlpha * (pitchDeg - gyroLowPass[1] * dt) +
               (1.0f - complementaryAlpha) * pitchAcc;
    yawDeg += gyroLowPass[2] * dt;
    if (yawDeg > 180.0f) yawDeg -= 360.0f;
    if (yawDeg < -180.0f) yawDeg += 360.0f;
}

static void processCommand(const char *command)
{
    if (strcmp(command, "CAL,GYRO") == 0) return startCalibration(CalibrationMode::Gyro);
    if (strcmp(command, "CAL,ACC,X+") == 0) return startCalibration(CalibrationMode::AccXPositive);
    if (strcmp(command, "CAL,ACC,X-") == 0) return startCalibration(CalibrationMode::AccXNegative);
    if (strcmp(command, "CAL,ACC,Y+") == 0) return startCalibration(CalibrationMode::AccYPositive);
    if (strcmp(command, "CAL,ACC,Y-") == 0) return startCalibration(CalibrationMode::AccYNegative);
    if (strcmp(command, "CAL,ACC,Z+") == 0) return startCalibration(CalibrationMode::AccZPositive);
    if (strcmp(command, "CAL,ACC,Z-") == 0) return startCalibration(CalibrationMode::AccZNegative);
    if (strcmp(command, "CAL,ACC,AUTO") == 0) return startCalibration(CalibrationMode::AccAuto);
    if (strcmp(command, "CFG,ARM,GET") == 0) {
        Serial.print("CFG,ARM,");
        Serial.print(armChannelIndex + 1);
        Serial.print(',');
        Serial.println(armSwitchHigh ? "HIGH" : "LOW");
        return;
    }
    unsigned int armChannel = 0;
    char armPolarity[8] = {};
    if (sscanf(command, "CFG,ARM,%u,%7s", &armChannel, armPolarity) == 2) {
        if (armChannel < 1 || armChannel > IBUS_CHANNEL_COUNT ||
            (strcmp(armPolarity, "HIGH") != 0 && strcmp(armPolarity, "LOW") != 0)) {
            Serial.println("CFG,ERROR,ARM_INVALID");
            return;
        }
        armed = false;  // Konfigurasi tidak boleh berubah saat motor sedang armed.
        armChannelIndex = static_cast<uint8_t>(armChannel - 1);
        armSwitchHigh = strcmp(armPolarity, "HIGH") == 0;
        Serial.print("CFG,ARM,");
        Serial.print(armChannel);
        Serial.print(',');
        Serial.println(armPolarity);
        return;
    }
    if (strcmp(command, "MOTOR,TEST,ENABLE") == 0) {
        /* QGC-style actuator bench test: radio link is not required. The
         * test path takes control away from the flight mixer and forces FC
         * disarmed until the operator disables it. Remove propellers first. */
        armed = false;
        motorTestEnabled = true;
        motorTestIndex = 0xFF;
        Serial.println("MOT,TEST_ENABLED");
        return;
    }
    if (strcmp(command, "MOTOR,TEST,DISABLE") == 0) {
        motorTestEnabled = false;
        motorTestIndex = 0xFF;
        stopMotors();
        Serial.println("MOT,TEST_DISABLED");
        return;
    }
    unsigned int motor = 0;
    unsigned int pulse = 0;
    if (sscanf(command, "MOTOR,TEST,%u,%u", &motor, &pulse) == 2) {
        if (!motorTestEnabled || armed || failsafeActive || motor < 1 || motor > 4) {
            Serial.println("MOT,ERROR,TEST_LOCKED");
        } else {
            motorTestIndex = static_cast<uint8_t>(motor - 1);
            motorTestPulseUs = static_cast<uint16_t>(pulse > MOTOR_TEST_MAX_US ? MOTOR_TEST_MAX_US :
                                                      (pulse < ESC_MIN_US ? ESC_MIN_US : pulse));
            motorTestStartedMs = millis();
            Serial.print("MOT,TEST,"); Serial.print(motor); Serial.print(','); Serial.println(motorTestPulseUs);
        }
        return;
    }
    Serial.print("CAL,ERROR,UNKNOWN_COMMAND,");
    Serial.println(command);
}

static void readCommands()
{
    while (Serial.available()) {
        const char input = static_cast<char>(Serial.read());
        if (input == '\n' || input == '\r') {
            if (commandLength > 0) {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer);
                commandLength = 0;
            }
        } else if (commandLength < sizeof(commandBuffer) - 1) {
            commandBuffer[commandLength++] = input;
        } else {
            commandLength = 0;
        }
    }
}

static void updateReceiver()
{
    while (IbusSerial.available()) {
        const uint8_t byte = static_cast<uint8_t>(IbusSerial.read());
        if (ibusIndex == 0) {
            if (byte == 0x20) ibusBuffer[ibusIndex++] = byte;
            continue;
        }
        if (ibusIndex == 1 && byte != 0x40) {
            ibusIndex = 0;
            continue;
        }
        ibusBuffer[ibusIndex++] = byte;
        if (ibusIndex != IBUS_PACKET_SIZE) continue;

        uint16_t sum = 0;
        for (uint8_t index = 0; index < IBUS_PACKET_SIZE - 2; ++index) sum += ibusBuffer[index];
        const uint16_t checksum = static_cast<uint16_t>(ibusBuffer[30]) |
                                  (static_cast<uint16_t>(ibusBuffer[31]) << 8);
        if (static_cast<uint16_t>(0xFFFF - sum) == checksum) {
            for (uint8_t channel = 0; channel < IBUS_CHANNEL_COUNT; ++channel) {
                receiverChannels[channel] = static_cast<uint16_t>(ibusBuffer[2 + channel * 2]) |
                                            (static_cast<uint16_t>(ibusBuffer[3 + channel * 2]) << 8);
            }
            lastReceiverFrameMs = millis();
            receiverValid = true;
        }
        ibusIndex = 0;
    }
}

static void updateSafetyState(uint32_t nowMs)
{
    failsafeActive = !receiverValid || (nowMs - lastReceiverFrameMs > RECEIVER_TIMEOUT_MS);
    if (failsafeActive) {
        armed = false;
        return;
    }
    if (motorTestEnabled) {
        armed = false;
        return;
    }
    const bool throttleLow = receiverChannels[IBUS_THROTTLE_CHANNEL] < THROTTLE_LOW_US;
    const bool armRequested = armSwitchHigh
        ? receiverChannels[armChannelIndex] > ARM_SWITCH_ON_US
        : receiverChannels[armChannelIndex] < ARM_SWITCH_ON_US;
    if (!armRequested) armed = false;
    if (armRequested && throttleLow) armed = true;
    if (armed) {
        motorTestEnabled = false;
        motorTestIndex = 0xFF;
    }
}

static void updateBatteryVoltage()
{
    if (!BATTERY_MONITOR_ENABLED) return;
    const uint32_t raw = analogRead(BATTERY_ADC_PIN);
    batteryVoltage = (static_cast<float>(raw) / 4095.0f) * ADC_REFERENCE_VOLTAGE * BATTERY_DIVIDER_RATIO;
}

static bool bmi160Begin()
{
    /* Setelah power-on, pembacaan 0x7F mengaktifkan antarmuka SPI BMI160. */
    (void)bmi160Read(0x7F);
    delay(2);
    if (bmi160Read(BMI160_REG_CHIP_ID) != BMI160_CHIP_ID) {
        return false;
    }

    bmi160Write(BMI160_REG_CMD, 0xB6);  // soft reset
    delay(100);
    bmi160Write(BMI160_REG_CMD, 0x11);  // accelerometer normal mode
    delay(50);
    bmi160Write(BMI160_REG_CMD, 0x15);  // gyroscope normal mode
    delay(100);

    bmi160Write(BMI160_REG_ACC_CONF, 0x2B);   // 800 Hz ODR, normal bandwidth
    bmi160Write(BMI160_REG_ACC_RANGE, 0x03);  // ±2 g
    bmi160Write(BMI160_REG_GYR_CONF, 0x2B);   // 800 Hz ODR, normal bandwidth
    bmi160Write(BMI160_REG_GYR_RANGE, 0x02);  // ±500 dps
    delay(20);

    // Beberapa modul membutuhkan perintah ACC normal kedua setelah reset.
    // Pastikan statusnya benar sebelum mulai mengambil sampel.
    uint8_t pmuStatus = bmi160Read(BMI160_REG_PMU_STATUS);
    if ((pmuStatus & 0x30) != 0x10) {
        bmi160Write(BMI160_REG_CMD, 0x11);
        delay(100);
        pmuStatus = bmi160Read(BMI160_REG_PMU_STATUS);
    }
    return (pmuStatus & 0x30) == 0x10 && (pmuStatus & 0x0C) == 0x04;
}

static void printBmi160Status()
{
    Serial.print("BMI160 reg: PMU=0x");
    Serial.print(bmi160Read(BMI160_REG_PMU_STATUS), HEX);
    Serial.print(" ERR=0x");
    Serial.print(bmi160Read(BMI160_REG_ERR), HEX);
    Serial.print(" ACC_CONF=0x");
    Serial.print(bmi160Read(BMI160_REG_ACC_CONF), HEX);
    Serial.print(" ACC_RANGE=0x");
    Serial.print(bmi160Read(BMI160_REG_ACC_RANGE), HEX);
    Serial.print(" GYR_CONF=0x");
    Serial.println(bmi160Read(BMI160_REG_GYR_CONF), HEX);
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED off (active-low)
    pinMode(BMI160_CS_PIN, OUTPUT);
    bmi160Deselect();

    SPI.begin();
    Serial.begin(115200);  // USB CDC pada USB-C BlackPill
    IbusSerial.begin(115200);
    beginMotors();  // Selalu mulai dari 1000 µs; ESC tidak boleh berputar saat boot.
    analogReadResolution(12);
    pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);
    delay(500);            // beri waktu laptop mengenali USB serial

    Serial.println("BMI160 USB CDC test");
    if (!bmi160Begin()) {
        Serial.println("BMI160 NOT FOUND: cek PA4-PA7, 3V3, dan GND");
        while (true) {
            digitalWrite(LED_PIN, LOW);
            delay(100);
            digitalWrite(LED_PIN, HIGH);
            delay(100);
        }
    }
    Serial.println("BMI160 OK: streaming IMU,<ms>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>");
    printBmi160Status();
}

void loop()
{
    static uint32_t lastStatusMs = 0;
    static uint32_t lastFcStatusMs = 0;
    static uint32_t lastRadioTelemetryMs = 0;
    static uint32_t lastTelemetryMs = 0;
    static uint32_t lastSampleUs = 0;
    static uint8_t ledDivider = 0;
    const uint32_t nowUs = micros();

    readCommands();
    updateReceiver();

    if (lastSampleUs != 0 && static_cast<uint32_t>(nowUs - lastSampleUs) < IMU_SAMPLE_PERIOD_US) {
        return;
    }
    const float dt = lastSampleUs == 0 ?
        static_cast<float>(IMU_SAMPLE_PERIOD_US) / 1000000.0f :
        static_cast<float>(nowUs - lastSampleUs) / 1000000.0f;
    lastSampleUs = nowUs;
    const uint32_t nowMs = millis();
    updateSafetyState(nowMs);

    uint8_t raw[12];
    bmi160ReadBurst(BMI160_REG_GYR_X_LSB, raw, sizeof(raw));
    const int16_t gyroRaw[3] = {int16LE(&raw[0]), int16LE(&raw[2]), int16LE(&raw[4])};
    const int16_t accRaw[3] = {int16LE(&raw[6]), int16LE(&raw[8]), int16LE(&raw[10])};
    collectCalibration(gyroRaw, accRaw);

    int32_t gyroCalibrated[3];
    int32_t accCalibrated[3];
    for (uint8_t axis = 0; axis < 3; ++axis) {
        gyroCalibrated[axis] = lroundf(static_cast<float>(gyroRaw[axis]) - gyroBias[axis]);
        accCalibrated[axis] = lroundf((static_cast<float>(accRaw[axis]) - accBias[axis]) * accScale[axis]);
    }
    updateAttitude(gyroCalibrated, accCalibrated, dt);
    updateMotors(gyroCalibrated, dt, nowMs);

    // Telemetry dipisahkan dari control loop agar USB tidak menambah jitter IMU.
    if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = nowMs;
        Serial.print("ATT,");
        Serial.print(nowMs);
        Serial.print(','); Serial.print(rollDeg, 2);
        Serial.print(','); Serial.print(pitchDeg, 2);
        Serial.print(','); Serial.print(yawDeg, 2);
        for (uint8_t axis = 0; axis < 3; ++axis) {
            Serial.print(','); Serial.print(gyroCalibrated[axis]);
        }
        for (uint8_t axis = 0; axis < 3; ++axis) {
            Serial.print(','); Serial.print(accCalibrated[axis]);
        }
        Serial.println();
    }

    // Dipisahkan dari status FC 5 Hz agar visual stick GCS responsif, tanpa
    // mengubah frekuensi control loop atau menunggu telemetry attitude.
    if (nowMs - lastRadioTelemetryMs >= RADIO_TELEMETRY_PERIOD_MS) {
        lastRadioTelemetryMs = nowMs;
        Serial.print("RAD,");
        Serial.print(receiverValid && !failsafeActive ? "RX_OK" : "FAILSAFE");
        for (uint8_t channel = 0; channel < IBUS_CHANNEL_COUNT; ++channel) {
            Serial.print(',');
            Serial.print(receiverChannels[channel]);
        }
        Serial.println();
    }

    if (nowMs - lastStatusMs >= 5000) {
        lastStatusMs = nowMs;
        if ((bmi160Read(BMI160_REG_PMU_STATUS) & 0x30) != 0x10) {
            Serial.println("ACC suspended; retry normal-mode command");
            bmi160Write(BMI160_REG_CMD, 0x11);
            delay(100);
        }
        printBmi160Status();
    }

    if (nowMs - lastFcStatusMs >= 200) {
        lastFcStatusMs = nowMs;
        updateBatteryVoltage();
        Serial.print("FC,");
        Serial.print(receiverValid && !failsafeActive ? "RX_OK" : "FAILSAFE");
        Serial.print(',');
        Serial.print(armed ? "ARMED" : "DISARMED");
        Serial.print(',');
        Serial.print(receiverChannels[IBUS_THROTTLE_CHANNEL]);
        Serial.print(',');
        Serial.print(BATTERY_MONITOR_ENABLED ? batteryVoltage : -1.0f, 2);
        Serial.println();
        Serial.print("MOT,");
        Serial.print(motorTestEnabled ? "TEST_READY" : "LOCKED");
        Serial.print(','); Serial.print(motorOutputUs[0]);
        Serial.print(','); Serial.print(motorOutputUs[1]);
        Serial.print(','); Serial.print(motorOutputUs[2]);
        Serial.print(','); Serial.println(motorOutputUs[3]);
    }

    if (++ledDivider == 250) {  // indikator hidup, tanpa berkedip 500 Hz
        ledDivider = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}
