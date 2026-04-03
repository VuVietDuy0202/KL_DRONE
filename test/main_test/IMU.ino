// ============================================================
// ICM-20602 — scale factors
// Gyro  ±2000 dps  → sensitivity = 16.4 LSB/dps → scale = 1/16.4
// Accel ±16g       → sensitivity = 2048 LSB/g   → scale = 1/2048 * 9.81
// ============================================================
#define GYRO_SCALE    (1.0f / 16.4f)          // dps/LSB
#define ACCEL_SCALE   (9.81f / 2048.0f)       // m/s² / LSB

// Velocity (tích phân accel, reset drift bằng zero-velocity detection)
float vel[3]     = {0};
float vel_prv[3] = {0};
float acc_prev[3] = {0};
uint32_t last_time_us = 0;

// Ngưỡng zero-velocity detection (ZUPT)
#define ZUPT_GYRO_THRESH   0.5f    // dps  — nếu gyro < ngưỡng này
#define ZUPT_ACC_THRESH    0.05f   // m/s² — và accel gần 1g → coi như đứng yên

void icm20602_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delayMicroseconds(10);
}

uint8_t icm20602_read_reg(uint8_t reg) {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20602_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF; // error
}

bool icm20602_config_reg(uint8_t reg, uint8_t value, uint8_t expected, const char* reg_name, int retries) {
  for (int i = 0; i < retries; i++) {
    icm20602_write_reg(reg, value);
    delay(2);
    uint8_t read_value = icm20602_read_reg(reg);
    Serial.printf("%s (lần %d): 0x%02X (Mong muốn: 0x%02X)\n", reg_name, i + 1, read_value, expected);
    if (read_value == expected) return true;
    Serial.printf("Thử lại %s...\n", reg_name);
    delay(5);
  }
  Serial.printf("Lỗi: Không thể cấu hình %s sau %d lần thử!\n", reg_name, retries);
  return false;
}

void READ_GYRO() {
  uint8_t rawData[14];

  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(DATA_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20602_ADDR, 14);

  for (int i = 0; i < 14; i++) {
    rawData[i] = Wire.available() ? Wire.read() : 0;
  }

  // Parse raw
  for (int i = 0; i < 3; i++) {
    acc[i] = (int16_t)(rawData[2*i]     << 8 | rawData[2*i+1]);
    gyr[i] = (int16_t)(rawData[8 + 2*i] << 8 | rawData[9 + 2*i]);

    // Low-pass filter
    gyr_f[i] = (1.0f - B_gyro)  * gyr_f_prv[i] + B_gyro  * gyr[i];
    acc_f[i] = (1.0f - B_accel) * acc_f_prv[i] + B_accel * acc[i];

    // Scale — ICM-20602 ±2000dps / ±16g (đã fix)
    gyr_dps[i] = (gyr_f[i] - gyr_bias[i]) * GYRO_SCALE;
    acc_mps[i] = (acc_f[i] - acc_bias[i]) * ACCEL_SCALE;
  }

  // Frame transform (giữ nguyên)
  gyr_fr_dps[0] = -gyr_dps[1];
  gyr_fr_dps[1] =  gyr_dps[0];
  gyr_fr_dps[2] =  gyr_dps[2];

  // ── Velocity integration (ZUPT) ──────────────────────────
  uint32_t now = micros();
  float dt = (last_time_us == 0) ? 0.0f : (now - last_time_us) * 1e-6f;
  last_time_us = now;

  if (dt > 0.0f && dt < 0.1f) { // bỏ qua dt bất thường
    // Tính magnitude gyro và accel
    float gyro_mag = sqrtf(gyr_dps[0]*gyr_dps[0] + gyr_dps[1]*gyr_dps[1] + gyr_dps[2]*gyr_dps[2]);
    float acc_mag  = sqrtf(acc_mps[0]*acc_mps[0]  + acc_mps[1]*acc_mps[1]  + acc_mps[2]*acc_mps[2]);
    float acc_net  = fabsf(acc_mag - 9.81f); // độ lệch so với 1g

    // ZUPT: nếu gần như đứng yên → reset vel về 0
    if (gyro_mag < ZUPT_GYRO_THRESH && acc_net < ZUPT_ACC_THRESH) {
      vel[0] = vel[1] = vel[2] = 0.0f;
    } else {
      // Tích phân trapezoidal (chính xác hơn Euler)
      for (int i = 0; i < 3; i++) {
        vel[i] += 0.5f * (acc_mps[i] + acc_prev[i]) * dt;
        acc_prev[i] = acc_mps[i];
      }
    }
  }

  // Update previous
  for (int i = 0; i < 3; i++) {
    acc_f_prv[i] = acc_f[i];
    gyr_f_prv[i] = gyr_f[i];
    vel_prv[i]   = vel[i];
  }
}

void GYRO_CALIBRATE(bool cali_acc) {
  int count = 0;
  float bias[6] = {0};

  // Reset bias trước (fix bug cộng dồn)
  for (int i = 0; i < 3; i++) {
    acc_bias[i] = 0;
    gyr_bias[i] = 0;
  }

  uint32_t t = micros();

  if (cali_acc) {
    while (count < 4000) {
      READ_GYRO();
      for (int i = 0; i < 3; i++) {
        bias[i]     += acc[i];
        bias[i + 3] += gyr[i];
      }
      count++;
      while (micros() - t < 2500);
      t = micros();
    }

    for (int i = 0; i < 6; i++) bias[i] /= count;

    bias[2] -= 2048.0f; // bù 1g trên trục Z cho ±16g

    for (int i = 0; i < 3; i++) {
      acc_bias[i] = bias[i];
      gyr_bias[i] = bias[i + 3];
    }
  }

  // Reset velocity sau calibrate
  vel[0] = vel[1] = vel[2] = 0.0f;
  last_time_us = 0;
}

void SENSOR_CONFIG(void) {
  Wire.begin(SDA_PIN, SCL_PIN);

  uint8_t id = icm20602_read_reg(WHO_AM_I);
  Serial.printf("ICM-20602 WHO_AM_I: 0x%02X (Expected: 0x12)\n", id);

  Serial.println("Wake up ICM-20602...");
  icm20602_write_reg(PWR_MGMT_1, 0x01); // clock source: PLL với gyro (tốt hơn 0x00)
  delay(100);

  bool config_ok = true;
  config_ok &= icm20602_config_reg(SMPLRT_DIV,   0x00, 0x00, "SMPLRT_DIV",   3);
  config_ok &= icm20602_config_reg(CONFIG,        0x03, 0x03, "CONFIG",        3);
  config_ok &= icm20602_config_reg(GYRO_CONFIG,   0x18, 0x18, "GYRO_CONFIG",   3); // ±2000 dps
  config_ok &= icm20602_config_reg(ACCEL_CONFIG,  0x18, 0x18, "ACCEL_CONFIG",  3); // ±16g
  config_ok &= icm20602_config_reg(ACCEL_CONFIG2, 0x03, 0x03, "ACCEL_CONFIG2", 3); // DLPF accel 44Hz

  if (!config_ok) Serial.println("Lỗi config!");
  else Serial.println("ICM-20602 ready!");
}