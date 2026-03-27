// #include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>
// #include <Wire.h>
// #include <ESP32Servo.h>
// #include <esp_wifi.h>
// #include <ESPmDNS.h>
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
// float PRateRoll = 0.625, IRateRoll = 0, DRateRoll = 0.008;
// float PRatePitch = 0.625, IRatePitch = 0, DRatePitch = 0.008;
// float PRateYaw = 4.0, IRateYaw = 0, DRateYaw = 0.0;

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
// volatile int LastMotorInput1 = 1000, LastMotorInput2 = 1000,
//              LastMotorInput3 = 1000, LastMotorInput4 = 1000;

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
// // CALLBACK — tương thích IDF 4.x và 5.x
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
// // IMU
// // ============================================================
// #define MPU_ADDR 0x68
// #define I2C_SDA 11
// #define I2C_SCL 10
// #define GYRO_SCALE 65.5f
// #define GYRO_CONFIG 0x08
// #define ACCEL_SCALE 4096.0f
// #define ACCEL_CONFIG 0x10

// float RawGyroX, RawGyroY, RawGyroZ;
// float RawAccX, RawAccY, RawAccZ;
// float RateRoll, RatePitch, RateYaw;
// float AccX, AccY, AccZ;
// float cfAngleRoll = 0, cfAnglePitch = 0;

// float RateCalibrationRoll = 0.8767;
// float RateCalibrationPitch = 0.1922;
// float RateCalibrationYaw = -0.1513;
// float AccXCalibration = 0.0580;
// float AccYCalibration = -0.0031;
// float AccZCalibration = -0.0148;

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

// float InputRoll, InputPitch, InputYaw, InputThrottle;

// // ============================================================
// // WEB SERVER
// // ============================================================
// AsyncWebServer server(80);

// #include "../include/web_page.h"

// // ============================================================
// // MODULE 1 — SPIFFS & LOAD PID
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
// // MODULE 2 — LOAD PID FROM SPIFFS
// // Tách riêng để gọi từ cả web lẫn CH5
// // ============================================================
// void loadPIDFromSPIFFS()
// {
//     String s;

//     s = readFile(SPIFFS, "/pGain.txt");
//     if (s.length() > 0)
//         PRateRoll = PRatePitch = s.toFloat();

//     s = readFile(SPIFFS, "/iGain.txt");
//     if (s.length() > 0)
//         IRateRoll = IRatePitch = s.toFloat();

//     s = readFile(SPIFFS, "/dGain.txt");
//     if (s.length() > 0)
//         DRateRoll = DRatePitch = s.toFloat();

//     s = readFile(SPIFFS, "/pAGain.txt");
//     if (s.length() > 0)
//         PAngleRoll = PAnglePitch = s.toFloat();

//     s = readFile(SPIFFS, "/iAGain.txt");
//     if (s.length() > 0)
//         IAngleRoll = IAnglePitch = s.toFloat();

//     s = readFile(SPIFFS, "/dAGain.txt");
//     if (s.length() > 0)
//         DAngleRoll = DAnglePitch = s.toFloat();

//     s = readFile(SPIFFS, "/pYaw.txt");
//     if (s.length() > 0)
//         PRateYaw = s.toFloat();

//     s = readFile(SPIFFS, "/iYaw.txt");
//     if (s.length() > 0)
//         IRateYaw = s.toFloat();

//     s = readFile(SPIFFS, "/dYaw.txt");
//     if (s.length() > 0)
//         DRateYaw = s.toFloat();

//     s = readFile(SPIFFS, "/tc.txt");
//     if (s.length() > 0)
//         t = s.toFloat();

//     Serial.printf("[PID] Loaded: PR=%.3f IR=%.3f DR=%.3f | PA=%.3f IA=%.3f DA=%.3f | T=%.6f\n",
//                   PRateRoll, IRateRoll, DRateRoll, PAngleRoll, IAngleRoll, DAngleRoll, t);
// }

// // ============================================================
// // MODULE 3 — WEB SERVER ROUTES
// // ============================================================

// void webServerSetup()
// {
//     // Trang chính
//     server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
//               { request->send(200, "text/html", index_html); });

//     // Test endpoint
//     server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request)
//               { request->send(200, "text/plain", "Web server OK!"); });

//     // Lưu giá trị vào SPIFFS + cập nhật biến global
//     server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     String val;
//     if      (request->hasParam("pGain"))  { val = request->getParam("pGain")->value();  writeFile(SPIFFS, "/pGain.txt",  val.c_str()); PRateRoll = PRatePitch = val.toFloat(); }
//     else if (request->hasParam("iGain"))  { val = request->getParam("iGain")->value();  writeFile(SPIFFS, "/iGain.txt",  val.c_str()); IRateRoll = IRatePitch = val.toFloat(); }
//     else if (request->hasParam("dGain"))  { val = request->getParam("dGain")->value();  writeFile(SPIFFS, "/dGain.txt",  val.c_str()); DRateRoll = DRatePitch = val.toFloat(); }
//     else if (request->hasParam("pAGain")) { val = request->getParam("pAGain")->value(); writeFile(SPIFFS, "/pAGain.txt", val.c_str()); PAngleRoll = PAnglePitch = val.toFloat(); }
//     else if (request->hasParam("iAGain")) { val = request->getParam("iAGain")->value(); writeFile(SPIFFS, "/iAGain.txt", val.c_str()); IAngleRoll = IAnglePitch = val.toFloat(); }
//     else if (request->hasParam("dAGain")) { val = request->getParam("dAGain")->value(); writeFile(SPIFFS, "/dAGain.txt", val.c_str()); DAngleRoll = DAnglePitch = val.toFloat(); }
//     else if (request->hasParam("pYaw"))   { val = request->getParam("pYaw")->value();   writeFile(SPIFFS, "/pYaw.txt",   val.c_str()); PRateYaw = val.toFloat(); }
//     else if (request->hasParam("iYaw"))   { val = request->getParam("iYaw")->value();   writeFile(SPIFFS, "/iYaw.txt",   val.c_str()); IRateYaw = val.toFloat(); }
//     else if (request->hasParam("dYaw"))   { val = request->getParam("dYaw")->value();   writeFile(SPIFFS, "/dYaw.txt",   val.c_str()); DRateYaw = val.toFloat(); }
//     else if (request->hasParam("tc"))     { val = request->getParam("tc")->value();     writeFile(SPIFFS, "/tc.txt",     val.c_str()); t = val.toFloat(); }
//     else val = "unknown param";
//     Serial.printf("[WEB] Set %s = %s\n", request->url().c_str(), val.c_str());
//     request->send(200, "text/plain", val); });

//     // ✅ Load PID ngay — gọi từ nút web, không cần CH5
//     server.on("/loadpid", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     loadPIDFromSPIFFS();
//     String msg = "PID loaded! PR=" + String(PRateRoll,3)
//                + " IR=" + String(IRateRoll,3)
//                + " DR=" + String(DRateRoll,3);
//     request->send(200, "text/plain", msg); });

//     // Realtime data — motor + góc + trạng thái
//     server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     String json = "{";
//     json += "\"m1\":"    + String(LastMotorInput1);
//     json += ",\"m2\":"   + String(LastMotorInput2);
//     json += ",\"m3\":"   + String(LastMotorInput3);
//     json += ",\"m4\":"   + String(LastMotorInput4);
//     json += ",\"roll\":" + String(cfAngleRoll,  1);
//     json += ",\"pitch\":"+ String(cfAnglePitch, 1);
//     json += ",\"fs\":"   + String(isFailsafe() ? 1 : 0);
//     json += ",\"arm\":"  + String(ReceiverValue[5] > 1500 ? 1 : 0);
//     json += "}";
//     request->send(200, "application/json", json); });

//     // Status/Debug endpoint
//     server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     String json = "{";
//     json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
//     json += ",\"channel\":" + String(WiFi.channel());
//     json += ",\"signal\":" + String(WiFi.RSSI());
//     json += ",\"spiffs_total\":" + String(SPIFFS.totalBytes());
//     json += ",\"spiffs_used\":" + String(SPIFFS.usedBytes());
//     json += ",\"pGain\":" + String(PRateRoll, 6);
//     json += ",\"iGain\":" + String(IRateRoll, 6);
//     json += ",\"dGain\":" + String(DRateRoll, 6);
//     json += ",\"pAGain\":" + String(PAngleRoll, 6);
//     json += ",\"iAGain\":" + String(IAngleRoll, 6);
//     json += ",\"dAGain\":" + String(DAngleRoll, 6);
//     json += ",\"pYaw\":" + String(PRateYaw, 6);
//     json += ",\"iYaw\":" + String(IRateYaw, 6);
//     json += ",\"dYaw\":" + String(DRateYaw, 6);
//     json += ",\"tc\":" + String(t, 6);
//     json += "}";
//     request->send(200, "application/json", json); });

//     server.onNotFound([](AsyncWebServerRequest *request)
//                       { request->send(404, "text/plain", "Not found"); });
//     server.begin();

//     // ✅ Start mDNS
//     if (MDNS.begin("drone"))
//     {
//         MDNS.addService("http", "tcp", 80);
//         Serial.println("[mDNS] Access via: http://drone.local");
//     }
// }

// // ============================================================
// // MODULE 4 — IMU
// // ============================================================
// void i2c_init()
// {
//     Wire.begin(I2C_SDA, I2C_SCL);
//     Wire.setClock(400000);
//     delay(200);
//     Serial.print("[I2C] Scanning...");
//     for (uint8_t a = 1; a < 127; a++)
//     {
//         Wire.beginTransmission(a);
//         if (Wire.endTransmission() == 0)
//             Serial.printf(" 0x%02X", a);
//     }
//     Serial.println(" done.");
// }

// void mpu_init()
// {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x6B);
//     Wire.write(0x00);
//     Wire.endTransmission();
//     delay(100);
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x19);
//     Wire.write(0x00);
//     Wire.endTransmission();
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1A);
//     Wire.write(0x03);
//     Wire.endTransmission();
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1B);
//     Wire.write(GYRO_CONFIG);
//     Wire.endTransmission();
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1C);
//     Wire.write(ACCEL_CONFIG);
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

//     RawGyroX = (float)gx / GYRO_SCALE;
//     RawGyroY = (float)gy / GYRO_SCALE;
//     RawGyroZ = (float)gz / GYRO_SCALE;
//     RawAccX = (float)ax / ACCEL_SCALE;
//     RawAccY = (float)ay / ACCEL_SCALE;
//     RawAccZ = (float)az / ACCEL_SCALE;
// }

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
//     Iterm = constrain(Iterm, -400, 400);
//     float Dterm = DAngleRoll * (ErrorAngleRoll - PrevErrorAngleRoll) / t;
//     DesiredRateRoll = constrain(Pterm + Iterm + Dterm, -400, 400);
//     PrevErrorAngleRoll = ErrorAngleRoll;
//     PrevItermAngleRoll = Iterm;

//     ErrorAnglePitch = DesiredAnglePitch - cfAnglePitch;
//     Pterm = PAnglePitch * ErrorAnglePitch;
//     Iterm = PrevItermAnglePitch + IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2);
//     Iterm = constrain(Iterm, -400, 400);
//     Dterm = DAnglePitch * (ErrorAnglePitch - PrevErrorAnglePitch) / t;
//     DesiredRatePitch = constrain(Pterm + Iterm + Dterm, -400, 400);
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
//     Iterm = constrain(Iterm, -400, 400);
//     float Dterm = DRateRoll * (ErrorRateRoll - PrevErrorRateRoll) / t;
//     InputRoll = constrain(Pterm + Iterm + Dterm, -400, 400);
//     PrevErrorRateRoll = ErrorRateRoll;
//     PrevItermRateRoll = Iterm;

//     Pterm = PRatePitch * ErrorRatePitch;
//     Iterm = PrevItermRatePitch + IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2);
//     Iterm = constrain(Iterm, -400, 400);
//     Dterm = DRatePitch * (ErrorRatePitch - PrevErrorRatePitch) / t;
//     InputPitch = constrain(Pterm + Iterm + Dterm, -400, 400);
//     PrevErrorRatePitch = ErrorRatePitch;
//     PrevItermRatePitch = Iterm;

//     Pterm = PRateYaw * ErrorRateYaw;
//     Iterm = PrevItermRateYaw + IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2);
//     Iterm = constrain(Iterm, -400, 400);
//     Dterm = DRateYaw * (ErrorRateYaw - PrevErrorRateYaw) / t;
//     InputYaw = constrain(Pterm + Iterm + Dterm, -400, 400);
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

//     MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw;
//     MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw;
//     MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw;
//     MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw;

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
//         if (isFailsafe())
//             Serial.println("!! FAILSAFE ACTIVE !!");
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
//         Serial.println("SPIFFS mount failed");
//     }
//     else
//     {
//         Serial.println("SPIFFS OK");
//     }

//     // Bước 1: mode AP_STA — bắt buộc để dùng cả ESP-NOW lẫn WiFi
//     WiFi.mode(WIFI_AP_STA);

//     // Bước 2: In MAC ngay sau khi set mode
//     Serial.print(">>> Drone MAC: ");
//     Serial.println(WiFi.macAddress());

//     // Bước 3: Init ESP-NOW TRƯỚC khi connect WiFi
//     if (esp_now_init() != ESP_OK)
//     {
//         Serial.println("ESP-NOW init failed!");
//     }
//     else
//     {
//         esp_now_register_recv_cb(onDataReceived);
//         Serial.println("ESP-NOW ready.");
//     }

//     // Bước 4: Connect WiFi cho web tuning
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
//             Serial.print(">>> Web IP: ");
//             Serial.println(WiFi.localIP());
//             Serial.printf(">>> Router channel: %d\n", WiFi.channel());
//             // Bước 5: Start web server SAU khi WiFi connected
//             webServerSetup();
//             Serial.println("Web server started.");
//         }
//         else
//         {
//             Serial.println("\nWiFi failed — web tuning unavailable");
//         }
//     }

//     // LED
//     pinMode(15, OUTPUT);
//     for (int i = 0; i < 6; i++)
//     {
//         digitalWrite(15, i % 2 == 0 ? LOW : HIGH);
//         delay(100);
//     }

//     // IMU
//     i2c_init();
//     mpu_init();

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
//     Serial.println("Flight controller ready!");
// }

// // ============================================================
// // LOOP — 250Hz
// // ============================================================
// void loop()
// {
//     mpu_read();
//     complementary_filter();
//     pidAngle();
//     pidRate();
//     mixMotors();

//     LastMotorInput1 = (int)MotorInput1;
//     LastMotorInput2 = (int)MotorInput2;
//     LastMotorInput3 = (int)MotorInput3;
//     LastMotorInput4 = (int)MotorInput4;

//     mot1.writeMicroseconds(MotorInput1);
//     mot2.writeMicroseconds(MotorInput2);
//     mot3.writeMicroseconds(MotorInput3);
//     mot4.writeMicroseconds(MotorInput4);

//     if (millis() - lastStatusPrint >= 500)
//     {
//         lastStatusPrint = millis();
//         Serial.printf(" P R:");
//         Serial.print(PRateRoll, 3);
//         Serial.printf(" I R:");
//         Serial.print(IRateRoll, 3);
//         Serial.printf(" D R:");
//         Serial.print(DRateRoll, 3);
//         Serial.printf(" | P A:");
//         Serial.print(PAngleRoll, 3);
//         Serial.printf(" I A:");
//         Serial.print(IAngleRoll, 3);
//         Serial.printf(" D A:");
//         Serial.println(DAngleRoll, 3);
//     }
//     while (micros() - LoopTimer < (uint32_t)(t * 1000000))
//     {
//         yield(); // ← cho FreeRTOS task khác chạy, kể cả web server
//     }
//     LoopTimer = micros();
// }
