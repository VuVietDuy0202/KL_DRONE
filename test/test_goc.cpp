#include <Arduino.h>
#include <Wire.h>

// ============================================================
// CALIBRATION TOOL — MPU6050
// Chạy file này riêng, sau đó copy giá trị vào receiver.ino
// ============================================================

#define MPU_ADDR 0x68
#define I2C_SDA 11
#define I2C_SCL 10
#define CALIB_SAMPLES 2000
#define LOOP_TIME_US 4000 // 250Hz — giống receiver
#define TELEPLOT_MS 20

// Gyro ±500°/s → LSB = 65.5  (PHẢI GIỐNG receiver.ino)
#define GYRO_SCALE 65.5f
#define GYRO_CONFIG 0x08 // 0x08 = ±500°/s

// Accel ±8g → LSB = 4096
#define ACCEL_SCALE 4096.0f
#define ACCEL_CONFIG 0x10 // 0x10 = ±8g

float t = 0.004f;

// ============================================================
// Raw IMU values (CHƯA trừ calibration)
// ============================================================
float RawGyroX, RawGyroY, RawGyroZ;
float RawAccX, RawAccY, RawAccZ;

// ============================================================
// Calibration offsets (sẽ được tính)
// ============================================================
float RateCalibRoll = 0;
float RateCalibPitch = 0;
float RateCalibYaw = 0;
float AccCalibX = 0;
float AccCalibY = 0;
float AccCalibZ = 0;

// ============================================================
// Sau calibration — giá trị đã trừ offset
// ============================================================
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float cfAngleRoll = 0, cfAnglePitch = 0;

unsigned long lastPrintTime = 0;

// ============================================================
// I2C INIT
// ============================================================
void i2c_init()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    delay(200);

    // Scan
    Serial.print("[I2C] Scanning...");
    for (uint8_t a = 1; a < 127; a++)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf(" 0x%02X", a);
        }
    }
    Serial.println(" done.");
}

// ============================================================
// MPU INIT
// ============================================================
void mpu_init()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x19);
    Wire.write(0x00); // Sample rate 8kHz
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03); // DLPF ~44Hz
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(GYRO_CONFIG); // ±500°/s
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(ACCEL_CONFIG); // ±8g
    Wire.endTransmission();

    delay(50);
    Serial.println("[MPU] Init OK — Gyro:±500°/s, Accel:±8g, DLPF:44Hz");
}

// ============================================================
// ĐỌC RAW — KHÔNG trừ offset
// Dùng cho calibration
// ============================================================
void mpu_read_raw()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    // Chỉ chuyển đơn vị, KHÔNG trừ offset
    RawGyroX = (float)gx / GYRO_SCALE;
    RawGyroY = (float)gy / GYRO_SCALE;
    RawGyroZ = (float)gz / GYRO_SCALE;
    RawAccX = (float)ax / ACCEL_SCALE;
    RawAccY = (float)ay / ACCEL_SCALE;
    RawAccZ = (float)az / ACCEL_SCALE;
}

// ============================================================
// ĐỌC ĐÃ CALIBRATE — dùng sau khi có offset
// ============================================================
void mpu_read()
{
    mpu_read_raw();
    RateRoll = RawGyroX - RateCalibRoll;
    RatePitch = RawGyroY - RateCalibPitch;
    RateYaw = RawGyroZ - RateCalibYaw;
    AccX = RawAccX - AccCalibX;
    AccY = RawAccY - AccCalibY;
    AccZ = RawAccZ - AccCalibZ;
}

// ============================================================
// COMPLEMENTARY FILTER
// Dùng hệ số giống receiver.ino
// ============================================================
void complementary_filter()
{
    float angleRollAcc = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29f;
    float anglePitchAcc = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29f;

    cfAngleRoll = 0.991f * (cfAngleRoll + RateRoll * t) + 0.009f * angleRollAcc;
    cfAnglePitch = 0.991f * (cfAnglePitch + RatePitch * t) + 0.009f * anglePitchAcc;

    cfAngleRoll = constrain(cfAngleRoll, -90.0f, 90.0f);
    cfAnglePitch = constrain(cfAnglePitch, -90.0f, 90.0f);
}

// ============================================================
// AUTO CALIBRATION
// Đọc raw, tính trung bình → ra offset
// ============================================================
void performAutoCalibration()
{
    Serial.println("\n=== AUTO CALIBRATION ===");
    Serial.println("Đặt drone nằm ngang, giữ yên.");
    Serial.println("Ấn ENTER để bắt đầu...");
    while (!Serial.available())
        ;
    Serial.read();
    delay(500);

    double sumGX = 0, sumGY = 0, sumGZ = 0;
    double sumAX = 0, sumAY = 0, sumAZ = 0;

    Serial.print("Sampling");
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        mpu_read_raw(); // Đọc RAW, không trừ offset

        sumGX += RawGyroX;
        sumGY += RawGyroY;
        sumGZ += RawGyroZ;
        sumAX += RawAccX;
        sumAY += RawAccY;
        sumAZ += RawAccZ;

        if (i % 200 == 0)
            Serial.print(".");
        delay(2);
    }
    Serial.println(" Done!");

    // Tính offset
    RateCalibRoll = sumGX / CALIB_SAMPLES;
    RateCalibPitch = sumGY / CALIB_SAMPLES;
    RateCalibYaw = sumGZ / CALIB_SAMPLES;
    AccCalibX = sumAX / CALIB_SAMPLES;
    AccCalibY = sumAY / CALIB_SAMPLES;
    AccCalibZ = (sumAZ / CALIB_SAMPLES) - 1.0f; // Trừ 1g (gravity)

    // Reset filter sau calibration
    cfAngleRoll = 0;
    cfAnglePitch = 0;

    Serial.println("\n=== COPY GIÁ TRỊ NÀY VÀO receiver.ino ===");
    Serial.printf("float RateCalibrationRoll  = %.4f;\n", RateCalibRoll);
    Serial.printf("float RateCalibrationPitch = %.4f;\n", RateCalibPitch);
    Serial.printf("float RateCalibrationYaw   = %.4f;\n", RateCalibYaw);
    Serial.printf("float AccXCalibration      = %.4f;\n", AccCalibX);
    Serial.printf("float AccYCalibration      = %.4f;\n", AccCalibY);
    Serial.printf("float AccZCalibration      = %.4f;\n", AccCalibZ);
    Serial.println("==========================================\n");
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== CALIBRATION TOOL ===");

    i2c_init();
    mpu_init();
    performAutoCalibration();

    Serial.println("Bắt đầu hiển thị góc sau calibration...");
    Serial.println("(Kiểm tra Roll và Pitch có về ~0 khi drone nằm ngang không)");
}

// ============================================================
// LOOP — hiển thị góc sau calibration để kiểm tra
// ============================================================
void loop()
{
    unsigned long loopStart = micros();

    mpu_read();             // Đọc + trừ offset
    complementary_filter(); // Tính góc

    // Teleplot format (dùng Teleplot extension trong VSCode)
    if (millis() - lastPrintTime >= TELEPLOT_MS)
    {
        lastPrintTime = millis();
        Serial.printf(">Roll:%.2f\n", cfAngleRoll);
        Serial.printf(">Pitch:%.2f\n", cfAnglePitch);
        Serial.printf(">RateRoll:%.2f\n", RateRoll);
        Serial.printf(">RatePitch:%.2f\n", RatePitch);
        Serial.printf(">RateYaw:%.2f\n", RateYaw);
    }

    // Giữ timing 250Hz
    unsigned long elapsed = micros() - loopStart;
    if (elapsed < LOOP_TIME_US)
    {
        delayMicroseconds(LOOP_TIME_US - elapsed);
    }
}
