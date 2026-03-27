// #include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>
// #include <Wire.h>
// #include <ESP32Servo.h>
// #include <esp_wifi.h>
// #include "esp_idf_version.h"

// const uint8_t ESPNOW_CHANNEL = 6;
// const bool ENABLE_WEB_TUNING = true;

// // ============================================================
// // WIFI CREDENTIALS
// // ============================================================
// const char *ssid = "Duong Uyen";
// const char *password = "vuduy2000";

// // ============================================================
// // TIMING
// // ============================================================
// float t = 0.004;
// uint32_t LoopTimer;
// unsigned long lastStatusPrint = 0;

// // ============================================================
// // PID GAINS — RATE
// // ============================================================
// float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.008;
// float PRatePitch = 0.625, IRatePitch = 2.1, DRatePitch = 0.008;
// float PRateYaw = 4.0, IRateYaw = 3.0, DRateYaw = 0.0;

// // ============================================================
// // PID GAINS — ANGLE
// // ============================================================
// float PAngleRoll = 2.0, IAngleRoll = 0.0, DAngleRoll = 0.007;
// float PAnglePitch = 2.0, IAnglePitch = 0.0, DAnglePitch = 0.007;

// // ============================================================
// // MOTOR PINS & OBJECTS
// // ============================================================
// Servo mot1, mot2, mot3, mot4;
// const int mot1_pin = 17;
// const int mot2_pin = 18;
// const int mot3_pin = 8;
// const int mot4_pin = 3;
// const int ESCfreq = 500;

// int ThrottleIdle = 1170;
// int ThrottleCutOff = 1000;
// float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// // ============================================================
// // ESP-NOW STRUCT
// // ============================================================
// typedef struct
// {
//     uint16_t ch1_roll;
//     uint16_t ch2_pitch;
//     uint16_t ch3_throttle;
//     uint16_t ch4_yaw;
//     uint16_t ch5_loadPID;
//     uint16_t ch6_arm;
// } DroneCommand;

// volatile uint16_t ReceiverValue[6] = {1500, 1500, 1000, 1500, 1000, 1000};
// volatile unsigned long lastPacketTime = 0;
// const unsigned long FAILSAFE_MS = 500;

// // ============================================================
// // CALLBACK nhận — tương thích cả IDF 4.x và 5.x
// // ============================================================
// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
// void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
// #else
// void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len)
// #endif
// {
//     if (len != sizeof(DroneCommand))
//         return;
//     DroneCommand *cmd = (DroneCommand *)data;
//     ReceiverValue[0] = cmd->ch1_roll;
//     ReceiverValue[1] = cmd->ch2_pitch;
//     ReceiverValue[2] = cmd->ch3_throttle;
//     ReceiverValue[3] = cmd->ch4_yaw;
//     ReceiverValue[4] = cmd->ch5_loadPID;
//     ReceiverValue[5] = cmd->ch6_arm;
//     lastPacketTime = millis();
// }

// bool isFailsafe()
// {
//     return (millis() - lastPacketTime > FAILSAFE_MS);
// }

// // ============================================================
// // Raw IMU values (CHƯA trừ calibration)
// // ============================================================
// float RawGyroX, RawGyroY, RawGyroZ;
// float RawAccX, RawAccY, RawAccZ;

// // ============================================================
// // Sau calibration — giá trị đã trừ offset
// // ============================================================
// float RateRoll, RatePitch, RateYaw;
// float AccX, AccY, AccZ;
// float cfAngleRoll = 0, cfAnglePitch = 0;
// // ============================================================
// // Calibration offsets (sẽ được tính)
// // ============================================================

// float RateCalibrationRoll = 0.8767;
// float RateCalibrationPitch = 0.1922;
// float RateCalibrationYaw = -0.1513;
// float AccXCalibration = 0.0580;
// float AccYCalibration = -0.0031;
// float AccZCalibration = -0.0148;

// #define I2C_SDA 11
// #define I2C_SCL 10

// #define MPU_ADDR 0x68
// // Gyro ±500°/s → LSB = 65.5  (PHẢI GIỐNG receiver.ino)
// #define GYRO_SCALE 65.5f
// #define GYRO_CONFIG 0x08 // 0x08 = ±500°/s

// // Accel ±8g → LSB = 4096
// #define ACCEL_SCALE 4096.0f
// #define ACCEL_CONFIG 0x10 // 0x10 = ±8g
// // ============================================================
// // FILTER
// // ============================================================
// float complementaryAngleRoll = 0.0f;
// float complementaryAnglePitch = 0.0f;

// // ============================================================
// // PID STATE — ANGLE
// // ============================================================
// float DesiredAngleRoll, DesiredAnglePitch;
// float ErrorAngleRoll, ErrorAnglePitch;
// float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
// float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0;

// // ============================================================
// // PID STATE — RATE
// // ============================================================
// float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
// float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
// float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
// float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;

// // ============================================================
// // FLIGHT INPUTS
// // ============================================================
// float InputRoll, InputPitch, InputYaw, InputThrottle;

// // ============================================================
// // WEB SERVER
// // ============================================================
// AsyncWebServer server(80);

// const char index_html[] PROGMEM = R"rawliteral(
// <!DOCTYPE HTML><html><head>
//   <title>Drone PID Tuning</title>
//   <meta name="viewport" content="width=device-width, initial-scale=1">
//   <script>
//     function submitMessage() {
//       alert("Saved!");
//       setTimeout(function(){ document.location.reload(false); }, 500);
//     }
//   </script></head><body>
//   <h2>Drone PID Tuning</h2>
//   <form action="/get" target="hidden-form">
//     P Rate (current: %pGain%): <input type="number" step="any" name="pGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     I Rate (current: %iGain%): <input type="number" step="any" name="iGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     D Rate (current: %dGain%): <input type="number" step="any" name="dGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     P Angle (current: %pAGain%): <input type="number" step="any" name="pAGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     I Angle (current: %iAGain%): <input type="number" step="any" name="iAGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     D Angle (current: %dAGain%): <input type="number" step="any" name="dAGain">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     P Yaw (current: %pYaw%): <input type="number" step="any" name="pYaw">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     I Yaw (current: %iYaw%): <input type="number" step="any" name="iYaw">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     D Yaw (current: %dYaw%): <input type="number" step="any" name="dYaw">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     Time cycle (current: %tc%): <input type="number" step="any" name="tc">
//     <input type="submit" value="Set" onclick="submitMessage()">
//   </form>
//   <iframe style="display:none" name="hidden-form"></iframe>
// </body></html>)rawliteral";

// // ============================================================
// // MODULE 1 — SPIFFS
// // ============================================================
// String readFile(fs::FS &fs, const char *path)
// {
//     File file = fs.open(path, "r");
//     if (!file || file.isDirectory())
//         return String();
//     String content;
//     while (file.available())
//         content += String((char)file.read());
//     file.close();
//     return content;
// }

// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     File file = fs.open(path, "w");
//     if (!file)
//         return;
//     file.print(message);
//     file.close();
// }

// // ============================================================
// // MODULE 2 — WEB SERVER
// // ============================================================
// String processor(const String &var)
// {
//     if (var == "pGain")
//         return readFile(SPIFFS, "/pGain.txt");
//     else if (var == "iGain")
//         return readFile(SPIFFS, "/iGain.txt");
//     else if (var == "dGain")
//         return readFile(SPIFFS, "/dGain.txt");
//     else if (var == "pAGain")
//         return readFile(SPIFFS, "/pAGain.txt");
//     else if (var == "iAGain")
//         return readFile(SPIFFS, "/iAGain.txt");
//     else if (var == "dAGain")
//         return readFile(SPIFFS, "/dAGain.txt");
//     else if (var == "pYaw")
//         return readFile(SPIFFS, "/pYaw.txt");
//     else if (var == "iYaw")
//         return readFile(SPIFFS, "/iYaw.txt");
//     else if (var == "dYaw")
//         return readFile(SPIFFS, "/dYaw.txt");
//     else if (var == "tc")
//         return readFile(SPIFFS, "/tc.txt");
//     return String();
// }

// void webServerSetup()
// {
//     server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
//               { request->send(200, "text/html", index_html, processor); });
//     server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     String val;
//     String param_name = "Unknown";
//     if      (request->hasParam("pGain"))  { val = request->getParam("pGain")->value();  writeFile(SPIFFS, "/pGain.txt",  val.c_str()); param_name = "P Rate"; }
//     else if (request->hasParam("iGain"))  { val = request->getParam("iGain")->value();  writeFile(SPIFFS, "/iGain.txt",  val.c_str()); param_name = "I Rate"; }
//     else if (request->hasParam("dGain"))  { val = request->getParam("dGain")->value();  writeFile(SPIFFS, "/dGain.txt",  val.c_str()); param_name = "D Rate"; }
//     else if (request->hasParam("pAGain")) { val = request->getParam("pAGain")->value(); writeFile(SPIFFS, "/pAGain.txt", val.c_str()); param_name = "P Angle"; }
//     else if (request->hasParam("iAGain")) { val = request->getParam("iAGain")->value(); writeFile(SPIFFS, "/iAGain.txt", val.c_str()); param_name = "I Angle"; }
//     else if (request->hasParam("dAGain")) { val = request->getParam("dAGain")->value(); writeFile(SPIFFS, "/dAGain.txt", val.c_str()); param_name = "D Angle"; }
//     else if (request->hasParam("pYaw"))   { val = request->getParam("pYaw")->value();   writeFile(SPIFFS, "/pYaw.txt",   val.c_str()); param_name = "P Yaw"; }
//     else if (request->hasParam("iYaw"))   { val = request->getParam("iYaw")->value();   writeFile(SPIFFS, "/iYaw.txt",   val.c_str()); param_name = "I Yaw"; }
//     else if (request->hasParam("dYaw"))   { val = request->getParam("dYaw")->value();   writeFile(SPIFFS, "/dYaw.txt",   val.c_str()); param_name = "D Yaw"; }
//     else if (request->hasParam("tc"))     { val = request->getParam("tc")->value();     writeFile(SPIFFS, "/tc.txt",     val.c_str()); param_name = "Time Cycle"; }
//     else val = "No message sent";
//     if (val.length() > 0)
//       Serial.printf("[WEB] Saved %s = %s\n", param_name.c_str(), val.c_str());
//     request->send(200, "text/plain", val); });
//     server.onNotFound([](AsyncWebServerRequest *request)
//                       { request->send(404, "text/plain", "Not found"); });
//     server.begin();
// }

// // ============================================================
// // MODULE 3 — LOAD PID FROM SPIFFS
// // ============================================================
// void loadPIDFromSPIFFS()
// {
//     Serial.println("\n=== LOADING PID FROM SPIFFS ===");

//     String str;
//     float v;

//     str = readFile(SPIFFS, "/pGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         PRateRoll = PRatePitch = v;
//         Serial.printf("P Rate: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/iGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         IRateRoll = IRatePitch = v;
//         Serial.printf("I Rate: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/dGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         DRateRoll = DRatePitch = v;
//         Serial.printf("D Rate: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/pAGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         PAngleRoll = PAnglePitch = v;
//         Serial.printf("P Angle: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/iAGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         IAngleRoll = IAnglePitch = v;
//         Serial.printf("I Angle: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/dAGain.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         DAngleRoll = DAnglePitch = v;
//         Serial.printf("D Angle: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/pYaw.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         PRateYaw = v;
//         Serial.printf("P Yaw: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/iYaw.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         IRateYaw = v;
//         Serial.printf("I Yaw: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/dYaw.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         DRateYaw = v;
//         Serial.printf("D Yaw: %.4f\n", v);
//     }

//     str = readFile(SPIFFS, "/tc.txt");
//     if (str.length() > 0)
//     {
//         v = str.toFloat();
//         t = v;
//         Serial.printf("Time Cycle: %.6f\n", v);
//     }

//     Serial.println("=== PID LOAD COMPLETE ===\n");
// }

// // ============================================================
// // MODULE 4 — IMU (MPU6050)
// // ============================================================

// void i2c_init()
// {
//     Wire.begin(I2C_SDA, I2C_SCL);
//     Wire.setClock(400000);
//     delay(200);

//     // Scan
//     Serial.print("[I2C] Scanning...");
//     for (uint8_t a = 1; a < 127; a++)
//     {
//         Wire.beginTransmission(a);
//         if (Wire.endTransmission() == 0)
//         {
//             Serial.printf(" 0x%02X", a);
//         }
//     }
//     Serial.println(" done.");
// }
// void mpu_init()
// {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x6B);
//     Wire.write(0x00); // Wake up
//     Wire.endTransmission();
//     delay(100);

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x19);
//     Wire.write(0x00); // Sample rate 8kHz
//     Wire.endTransmission();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1A);
//     Wire.write(0x03); // DLPF ~44Hz
//     Wire.endTransmission();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1B);
//     Wire.write(GYRO_CONFIG); // ±500°/s
//     Wire.endTransmission();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1C);
//     Wire.write(ACCEL_CONFIG); // ±8g
//     Wire.endTransmission();

//     delay(50);
//     Serial.println("[MPU] Init OK — Gyro:±500°/s, Accel:±8g, DLPF:44Hz");
// }
// void mpu_read_raw()
// {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x3B);
//     Wire.endTransmission();
//     Wire.requestFrom(MPU_ADDR, 6);
//     int16_t ax = Wire.read() << 8 | Wire.read();
//     int16_t ay = Wire.read() << 8 | Wire.read();
//     int16_t az = Wire.read() << 8 | Wire.read();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x43);
//     Wire.endTransmission();
//     Wire.requestFrom(MPU_ADDR, 6);
//     int16_t gx = Wire.read() << 8 | Wire.read();
//     int16_t gy = Wire.read() << 8 | Wire.read();
//     int16_t gz = Wire.read() << 8 | Wire.read();

//     // Chỉ chuyển đơn vị, KHÔNG trừ offset
//     RawGyroX = (float)gx / GYRO_SCALE;
//     RawGyroY = (float)gy / GYRO_SCALE;
//     RawGyroZ = (float)gz / GYRO_SCALE;
//     RawAccX = (float)ax / ACCEL_SCALE;
//     RawAccY = (float)ay / ACCEL_SCALE;
//     RawAccZ = (float)az / ACCEL_SCALE;
// }

// // ============================================================
// // ĐỌC ĐÃ CALIBRATE — dùng sau khi có offset
// // ============================================================
// void mpu_read()
// {
//     mpu_read_raw();
//     RateRoll = RawGyroX - RateCalibrationRoll;
//     RatePitch = RawGyroY - RateCalibrationPitch;
//     RateYaw = RawGyroZ - RateCalibrationYaw;
//     AccX = RawAccX - AccXCalibration;
//     AccY = RawAccY - AccYCalibration;
//     AccZ = RawAccZ - AccZCalibration;
// }
// // ============================================================
// // MODULE 5 — COMPLEMENTARY FILTER
// // ============================================================
// void complementary_filter()
// {
//     float angleRollAcc = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29f;
//     float anglePitchAcc = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29f;

//     cfAngleRoll = 0.991f * (cfAngleRoll + RateRoll * t) + 0.009f * angleRollAcc;
//     cfAnglePitch = 0.991f * (cfAnglePitch + RatePitch * t) + 0.009f * anglePitchAcc;

//     cfAngleRoll = constrain(cfAngleRoll, -90.0f, 90.0f);
//     cfAnglePitch = constrain(cfAnglePitch, -90.0f, 90.0f);
// }

// // ============================================================
// // MODULE 6 — PID ANGLE
// // ============================================================
// void pidAngle()
// {
//     DesiredAngleRoll = 0.1f * (ReceiverValue[0] - 1500);
//     DesiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);

//     ErrorAngleRoll = DesiredAngleRoll - cfAngleRoll;
//     float Pterm = PAngleRoll * ErrorAngleRoll;
//     float Iterm = PrevItermAngleRoll + IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2);
//     Iterm = constrain(Iterm, -400.0f, 400.0f);
//     float Dterm = DAngleRoll * (ErrorAngleRoll - PrevErrorAngleRoll) / t;
//     DesiredRateRoll = constrain(Pterm + Iterm + Dterm, -400.0f, 400.0f);
//     PrevErrorAngleRoll = ErrorAngleRoll;
//     PrevItermAngleRoll = Iterm;

//     ErrorAnglePitch = DesiredAnglePitch - cfAnglePitch;
//     Pterm = PAnglePitch * ErrorAnglePitch;
//     Iterm = PrevItermAnglePitch + IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2);
//     Iterm = constrain(Iterm, -400.0f, 400.0f);
//     Dterm = DAnglePitch * (ErrorAnglePitch - PrevErrorAnglePitch) / t;
//     DesiredRatePitch = constrain(Pterm + Iterm + Dterm, -400.0f, 400.0f);
//     PrevErrorAnglePitch = ErrorAnglePitch;
//     PrevItermAnglePitch = Iterm;

//     DesiredRateYaw = 0.15f * (ReceiverValue[3] - 1500);
// }

// // ============================================================
// // MODULE 7 — PID RATE
// // ============================================================
// void pidRate()
// {
//     ErrorRateRoll = DesiredRateRoll - RateRoll;
//     ErrorRatePitch = DesiredRatePitch - RatePitch;
//     ErrorRateYaw = DesiredRateYaw - RateYaw;

//     float Pterm = PRateRoll * ErrorRateRoll;
//     float Iterm = PrevItermRateRoll + IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2);
//     Iterm = constrain(Iterm, -400.0f, 400.0f);
//     float Dterm = DRateRoll * (ErrorRateRoll - PrevErrorRateRoll) / t;
//     InputRoll = constrain(Pterm + Iterm + Dterm, -400.0f, 400.0f);
//     PrevErrorRateRoll = ErrorRateRoll;
//     PrevItermRateRoll = Iterm;

//     Pterm = PRatePitch * ErrorRatePitch;
//     Iterm = PrevItermRatePitch + IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2);
//     Iterm = constrain(Iterm, -400.0f, 400.0f);
//     Dterm = DRatePitch * (ErrorRatePitch - PrevErrorRatePitch) / t;
//     InputPitch = constrain(Pterm + Iterm + Dterm, -400.0f, 400.0f);
//     PrevErrorRatePitch = ErrorRatePitch;
//     PrevItermRatePitch = Iterm;

//     Pterm = PRateYaw * ErrorRateYaw;
//     Iterm = PrevItermRateYaw + IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2);
//     Iterm = constrain(Iterm, -400.0f, 400.0f);
//     Dterm = DRateYaw * (ErrorRateYaw - PrevErrorRateYaw) / t;
//     InputYaw = constrain(Pterm + Iterm + Dterm, -400.0f, 400.0f);
//     PrevErrorRateYaw = ErrorRateYaw;
//     PrevItermRateYaw = Iterm;
// }

// // ============================================================
// // MODULE 8 — MOTOR MIXING + SAFETY
// // ============================================================
// void resetPID()
// {
//     PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
//     PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
//     PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
//     PrevItermAngleRoll = PrevItermAnglePitch = 0;
// }

// void mixMotors()
// {
//     InputThrottle = constrain((float)ReceiverValue[2], 1000.0f, 1800.0f);

//     MotorInput1 = InputThrottle - InputRoll - InputPitch;
//     MotorInput2 = InputThrottle - InputRoll + InputPitch;
//     MotorInput3 = InputThrottle + InputRoll + InputPitch;
//     MotorInput4 = InputThrottle + InputRoll - InputPitch;

//     MotorInput1 = constrain(MotorInput1, 1000.0f, 1999.0f);
//     MotorInput2 = constrain(MotorInput2, 1000.0f, 1999.0f);
//     MotorInput3 = constrain(MotorInput3, 1000.0f, 1999.0f);
//     MotorInput4 = constrain(MotorInput4, 1000.0f, 1999.0f);

//     MotorInput1 = max(MotorInput1, (float)ThrottleIdle);
//     MotorInput2 = max(MotorInput2, (float)ThrottleIdle);
//     MotorInput3 = max(MotorInput3, (float)ThrottleIdle);
//     MotorInput4 = max(MotorInput4, (float)ThrottleIdle);

//     bool disarm = (ReceiverValue[2] < 1030) || (ReceiverValue[5] < 1500) || isFailsafe();

//     if (disarm)
//     {
//         MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
//         resetPID();
//         // if (isFailsafe())
//         //   Serial.println("!! FAILSAFE ACTIVE !!");
//     }
// }

// // ============================================================
// // SETUP
// // ============================================================
// void setup()
// {
//     Serial.begin(115200);
//     delay(500);

//     // SPIFFS
//     if (!SPIFFS.begin(true))
//     {
//         Serial.println("SPIFFS mount failed — continuing without web tuning");
//     }
//     else
//     {
//         Serial.println("SPIFFS OK");
//     }

//     // WiFi mode phải set TRƯỚC esp_now_init
//     WiFi.mode(WIFI_STA);
//     WiFi.disconnect();

//     esp_wifi_set_promiscuous(true);
//     esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
//     esp_wifi_set_promiscuous(false);

//     // In MAC để copy vào transmitter
//     Serial.print(">>> Drone MAC (copy vao transmitter): ");
//     Serial.println(WiFi.macAddress());
//     Serial.printf(">>> ESP-NOW channel: %u\n", ESPNOW_CHANNEL);

//     // ESP-NOW
//     if (esp_now_init() != ESP_OK)
//     {
//         Serial.println("ESP-NOW init failed!");
//     }
//     else
//     {
//         esp_now_register_recv_cb(onDataReceived);
//         Serial.println("ESP-NOW receiver ready.");
//     }

//     // WiFi cho web tuning (tắt mặc định để không phá channel ESP-NOW)
//     if (ENABLE_WEB_TUNING)
//     {
//         WiFi.begin(ssid, password);
//         Serial.print("Connecting WiFi");
//         int retry = 0;
//         while (WiFi.status() != WL_CONNECTED && retry < 20)
//         {
//             delay(500);
//             Serial.print(".");
//             retry++;
//         }
//         if (WiFi.status() == WL_CONNECTED)
//         {
//             Serial.println();
//             Serial.print(">>> Web tuning IP: ");
//             Serial.println(WiFi.localIP());
//             Serial.printf(">>> Router channel: %d\n", WiFi.channel());
//             if (WiFi.channel() != ESPNOW_CHANNEL)
//             {
//                 Serial.println("WARNING: Router channel khac ESPNOW channel, co the mat link");
//             }
//             webServerSetup();
//             loadPIDFromSPIFFS();
//         }
//         else
//         {
//             Serial.println("\nWiFi failed — web tuning unavailable");
//         }
//     }
//     else
//     {
//         Serial.println(">>> Web tuning disabled for stable ESP-NOW link");
//     }

//     // LED
//     pinMode(15, OUTPUT);
//     for (int i = 0; i < 6; i++)
//     {
//         digitalWrite(15, i % 2 == 0 ? LOW : HIGH);
//         delay(100);
//     }

//     i2c_init();
//     mpu_init();
//     Serial.println("IMU OK");

//     // ESC
//     ESP32PWM::allocateTimer(0);
//     ESP32PWM::allocateTimer(1);
//     ESP32PWM::allocateTimer(2);
//     ESP32PWM::allocateTimer(3);

//     mot1.attach(mot1_pin, 1000, 2000);
//     delay(1000);
//     mot1.setPeriodHertz(ESCfreq);
//     delay(100);
//     mot2.attach(mot2_pin, 1000, 2000);
//     delay(1000);
//     mot2.setPeriodHertz(ESCfreq);
//     delay(100);
//     mot3.attach(mot3_pin, 1000, 2000);
//     delay(1000);
//     mot3.setPeriodHertz(ESCfreq);
//     delay(100);
//     mot4.attach(mot4_pin, 1000, 2000);
//     delay(1000);
//     mot4.setPeriodHertz(ESCfreq);
//     delay(100);

//     mot1.writeMicroseconds(1000);
//     mot2.writeMicroseconds(1000);
//     mot3.writeMicroseconds(1000);
//     mot4.writeMicroseconds(1000);
//     delay(500);

//     digitalWrite(15, HIGH);
//     delay(500);
//     digitalWrite(15, LOW);
//     delay(500);

//     lastPacketTime = millis();
//     LoopTimer = micros();
//     Serial.println("Flight controller ready. Waiting for transmitter...");
// }

// // ============================================================
// // LOOP — 250Hz
// // ============================================================
// bool ch5_prev = false;

// void loop()
// {
//     bool ch5_now = (ReceiverValue[4] > 1500);
//     if (ch5_now && !ch5_prev)
//         loadPIDFromSPIFFS();
//     ch5_prev = ch5_now;

//     mpu_read();             // Đọc + trừ offset
//     complementary_filter(); // Tính góc
//     pidAngle();
//     pidRate();
//     mixMotors();

//     mot1.writeMicroseconds(MotorInput1);
//     mot2.writeMicroseconds(MotorInput2);
//     mot3.writeMicroseconds(MotorInput3);
//     mot4.writeMicroseconds(MotorInput4);

//     if (millis() - lastStatusPrint >= 100)
//     {
//         lastStatusPrint = millis();
//         unsigned long ageMs = millis() - lastPacketTime;
//         Serial.printf("R: %.1f, P: %.1f, | Throttle: %.0f | m1 m2 m3 m4: %.0f %.0f %.0f %.0f \n ",
//                       cfAngleRoll, cfAnglePitch, InputThrottle, MotorInput1, MotorInput2, MotorInput3, MotorInput4, (ageMs > FAILSAFE_MS) ? "!! FAILSAFE !!" : "");

//         while (micros() - LoopTimer < (uint32_t)(t * 1000000))
//             yield();
//         LoopTimer = micros();
//     }
// }