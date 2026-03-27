
// // THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR
// // OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// // OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE
// // SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

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
// const bool ENABLE_WEB_TUNING = false;

// const char *ssid = "AI Beacon_2.4G";
// const char *password = "666688889";

// float PRateRoll = 0.6;
// float IRateRoll = 2.1;
// float DRateRoll = 0.0034;

// float PAngleRoll = 4;
// float IAngleRoll = 0.004;
// float DAngleRoll = 0.0022;

// float PRateYaw = 4;
// float IRateYaw = 3;
// float DRateYaw = 0;

// int ESCfreq = 500;

// float PRatePitch = PRateRoll;
// float IRatePitch = IRateRoll;
// float DRatePitch = DRateRoll;
// float PAnglePitch = PAngleRoll;
// float IAnglePitch = IAngleRoll;
// float DAnglePitch = DAngleRoll;
// uint32_t LoopTimer;
// float t = 0.004; // time cycle

// volatile float RatePitch, RateRoll, RateYaw;
// volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;
// int RateCalibrationNumber;

// Servo mot1;
// Servo mot2;
// Servo mot3;
// Servo mot4;
// const int mot1_pin = 17;
// const int mot2_pin = 18;
// const int mot3_pin = 8; // 14 for perf board
// const int mot4_pin = 3;

// typedef struct
// {
//     uint16_t ch1_roll;
//     uint16_t ch2_pitch;
//     uint16_t ch3_throttle;
//     uint16_t ch4_yaw;
//     uint16_t ch5_loadPID;
//     uint16_t ch6_aux;
// } DroneCommand;

// volatile int ReceiverValue[6] = {1500, 1500, 1000, 1500, 1000, 1000};
// volatile unsigned long lastPacketTime = 0;
// const unsigned long FAILSAFE_MS = 500;

// volatile float PtermRoll;
// volatile float ItermRoll;
// volatile float DtermRoll;
// volatile float PIDOutputRoll;
// volatile float PtermPitch;
// volatile float ItermPitch;
// volatile float DtermPitch;
// volatile float PIDOutputPitch;
// volatile float PtermYaw;
// volatile float ItermYaw;
// volatile float DtermYaw;
// volatile float PIDOutputYaw;
// volatile float KalmanGainPitch;
// volatile float KalmanGainRoll;

// int ThrottleIdle = 1170;
// int ThrottleCutOff = 1000;

// volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
// volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
// volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
// volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
// volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
// volatile float PIDReturn[] = {0, 0, 0};

// float complementaryAngleRoll = 0.0f;
// float complementaryAnglePitch = 0.0f;

// // Kalman filters for angle mode
// volatile float AccX, AccY, AccZ;
// volatile float AngleRoll, AnglePitch;
// volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
// volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
// volatile float Kalman1DOutput[] = {0, 0};
// volatile float DesiredAngleRoll, DesiredAnglePitch;
// volatile float ErrorAngleRoll, ErrorAnglePitch;
// volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
// volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

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
//     ReceiverValue[5] = cmd->ch6_aux;
//     lastPacketTime = millis();
// }

// bool isFailsafe()
// {
//     return (millis() - lastPacketTime > FAILSAFE_MS);
// }

// // WIFI tuning global code
// AsyncWebServer server(80);

// const char *PARAM_P_GAIN = "pGain"; // For Pitch & Roll RATE
// const char *PARAM_I_GAIN = "iGain";
// const char *PARAM_D_GAIN = "dGain";

// const char *PARAM_P_A_GAIN = "pAGain"; // For Pitch & Roll ANGLE
// const char *PARAM_I_A_GAIN = "iAGain";
// const char *PARAM_D_A_GAIN = "dAGain";

// const char *PARAM_P_YAW = "pYaw"; // For Yaw
// const char *PARAM_I_YAW = "iYaw";
// const char *PARAM_D_YAW = "dYaw";

// const char *PARAM_TIME_CYCLE = "tc"; // Computation time cycle

// // HTML web page to handle 6 input fields of PID gains
// const char index_html[] PROGMEM = R"rawliteral(
// <!DOCTYPE HTML><html><head>
//   <title>ESP Input Form</title>
//   <meta name="viewport" content="width=device-width, initial-scale=1">
//   <script>
//     function submitMessage() {
//       alert("Saved value to ESP SPIFFS");
//       setTimeout(function(){ document.location.reload(false); }, 500);
//     }
//   </script></head><body>

//      <form action="/get" target="hidden-form"><br>
//     ESP32 Webserver for PID Gain value tuning of Quadcopter
//   </form><br><br>

//   <form action="/get" target="hidden-form">
//     P Pitch & Roll Gain (current value %pGain%): <input type="number" step="any" name="pGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     I Pitch & Roll Gain (current value %iGain%): <input type="number" step="any" name="iGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     D Pitch & Roll Gain (current value %dGain%): <input type="number" step="any" name="dGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     P Pitch & Roll Angle Gain (current value %pAGain%): <input type="number" step="any" name="pAGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//     <form action="/get" target="hidden-form">
//     I Pitch & Roll Angle Gain (current value %iAGain%): <input type="number" step="any" name="iAGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//     <form action="/get" target="hidden-form">
//     D Pitch & Roll Angle Gain (current value %dAGain%): <input type="number" step="any" name="dAGain">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     P Yaw Gain (current value %pYaw%): <input type="number" step="any" name="pYaw">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     I Yaw Gain (current value %iYaw%): <input type="number" step="any" name="iYaw">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br>
//   <form action="/get" target="hidden-form">
//     D Yaw Gain (current value %dYaw%): <input type="number" step="any" name="dYaw">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br><br>
//     <form action="/get" target="hidden-form">
//     Time cycle (current value %tc%): <input type="number" step="any" name="tc">
//     <input type="submit" value="Submit" onclick="submitMessage()">
//   </form><br><br>
// ----------------
//   <iframe style="display:none" name="hidden-form"></iframe>
// </body></html>)rawliteral";

// void notFound(AsyncWebServerRequest *request)-- -- -- --
// {
//     request->send(404, "text/plain", "Not found");
// }

// String readFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Reading file: %s\r\n", path);
//     File file = fs.open(path, "r");
//     if (!file || file.isDirectory())
//     {
//         Serial.println("- empty file or failed to open file");
//         return String();
//     }
//     Serial.println("- read from file:");
//     String fileContent;
//     while (file.available())
//     {
//         fileContent += String((char)file.read());
//     }
//     file.close();
//     Serial.println(fileContent);
//     return fileContent;
// }

// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Writing file: %s\r\n", path);
//     File file = fs.open(path, "w");
//     if (!file)
//     {
//         Serial.println("- failed to open file for writing");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("- file written");
//     }
//     else
//     {
//         Serial.println("- write failed");
//     }
//     file.close();
// }

// // Replaces placeholder with stored values
// String processor(const String &var)
// {
//     // Serial.println(var);
//     if (var == "pGain")
//     {
//         return readFile(SPIFFS, "/pGain.txt");
//     }
//     else if (var == "iGain")
//     {
//         return readFile(SPIFFS, "/iGain.txt");
//     }
//     else if (var == "dGain")
//     {
//         return readFile(SPIFFS, "/dGain.txt");
//     }
//     else if (var == "pAGain")
//     {
//         return readFile(SPIFFS, "/pAGain.txt");
//     }
//     else if (var == "iAGain")
//     {
//         return readFile(SPIFFS, "/iAGain.txt");
//     }
//     else if (var == "dAGain")
//     {
//         return readFile(SPIFFS, "/dAGain.txt");
//     }
//     else if (var == "pYaw")
//     {
//         return readFile(SPIFFS, "/pYaw.txt");
//     }
//     else if (var == "dYaw")
//     {
//         return readFile(SPIFFS, "/dYaw.txt");
//     }
//     else if (var == "iYaw")
//     {
//         return readFile(SPIFFS, "/iYaw.txt");
//     }
//     else if (var == "tc")
//     {
//         return readFile(SPIFFS, "/tc.txt");
//     }
// }

// void setup(void)
// {

//     Serial.begin(115200);

//     // WIFI server setup START
//     //  Initialize SPIFFS
// #ifdef ESP32
//     if (!SPIFFS.begin(true))
//     {
//         Serial.println("An Error has occurred while mounting SPIFFS");
//         return;
//     }
// #else
//     if (!SPIFFS.begin())
//     {
//         Serial.println("An Error has occurred while mounting SPIFFS");
//         return;
//     }
// #endif
//     WiFi.mode(WIFI_STA);
//     WiFi.begin(ssid, password);
//     if (WiFi.waitForConnectResult() != WL_CONNECTED)
//     {
//         Serial.println("WiFi Failed!");
//         return;
//     }
//     Serial.println();
//     Serial.print("IP Address: ");

//     Serial.println(WiFi.localIP());
//     delay(2000);
//     // Send web page with input fields to client
//     server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
//               { request->send_P(200, "text/html", index_html, processor); });

//     // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
//     server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
//               {
//     String inputMessage;
//     // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
// if (request->hasParam(PARAM_P_GAIN)) {
//     inputMessage = request->getParam(PARAM_P_GAIN)->value();
//     writeFile(SPIFFS, "/pGain.txt", inputMessage.c_str());
// }
// // GET I Gain value on <ESP_IP>/get?iGain=<inputMessage>
// else if (request->hasParam(PARAM_I_GAIN)) {
//     inputMessage = request->getParam(PARAM_I_GAIN)->value();
//     writeFile(SPIFFS, "/iGain.txt", inputMessage.c_str());
// }
// // GET D Gain value on <ESP_IP>/get?dGain=<inputMessage>
// else if (request->hasParam(PARAM_D_GAIN)) {
//     inputMessage = request->getParam(PARAM_D_GAIN)->value();
//     writeFile(SPIFFS, "/dGain.txt", inputMessage.c_str());
// }
// else if (request->hasParam(PARAM_P_A_GAIN)) {
//     inputMessage = request->getParam(PARAM_P_A_GAIN)->value();
//     writeFile(SPIFFS, "/pAGain.txt", inputMessage.c_str());
// }
// else if (request->hasParam(PARAM_I_A_GAIN)) {
//     inputMessage = request->getParam(PARAM_I_A_GAIN)->value();
//     writeFile(SPIFFS, "/iAGain.txt", inputMessage.c_str());
// }
// else if (request->hasParam(PARAM_D_A_GAIN)) {
//     inputMessage = request->getParam(PARAM_D_A_GAIN)->value();
//     writeFile(SPIFFS, "/dAGain.txt", inputMessage.c_str());
// }
// else if (request->hasParam(PARAM_P_YAW)) {
//     inputMessage = request->getParam(PARAM_P_YAW)->value();
//     writeFile(SPIFFS, "/pYaw.txt", inputMessage.c_str());
//   } else if (request->hasParam(PARAM_I_YAW)) {
//     inputMessage = request->getParam(PARAM_I_YAW)->value();
//     writeFile(SPIFFS, "/iYaw.txt", inputMessage.c_str());
//   } else if (request->hasParam(PARAM_D_YAW)) {
//     inputMessage = request->getParam(PARAM_D_YAW)->value();
//     writeFile(SPIFFS, "/dYaw.txt", inputMessage.c_str());
//   }
//  else if (request->hasParam(PARAM_TIME_CYCLE)) {
//     inputMessage = request->getParam(PARAM_TIME_CYCLE)->value();
//     writeFile(SPIFFS, "/tc.txt", inputMessage.c_str());
//   }
//     else {
//       inputMessage = "No message sent";
//     }
//     Serial.println(inputMessage);
//     request->send(200, "text/text", inputMessage); });

//     server.onNotFound(notFound);
//     server.begin();
//     // WIFI server setup END
//     // WiFi mode MUST be set BEFORE esp_now_init
//     WiFi.mode(WIFI_STA);
//     WiFi.disconnect();

//     esp_wifi_set_promiscuous(true);
//     esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
//     esp_wifi_set_promiscuous(false);

//     Serial.print(">>> Drone MAC (copy to transmitter): ");
//     Serial.println(WiFi.macAddress());
//     Serial.printf(">>> ESP-NOW channel: %u\n", ESPNOW_CHANNEL);

//     // ESP-NOW Init
//     if (esp_now_init() != ESP_OK)
//     {
//         Serial.println("ESP-NOW init failed!");
//     }
//     else
//     {
//         esp_now_register_recv_cb(onDataReceived);
//         Serial.println("ESP-NOW receiver ready.");
//     }

//     // Web tuning
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
//                 Serial.println("WARNING: Router channel differs from ESPNOW channel");
//             }
//         }
//         else
//         {
//             Serial.println("\nWiFi failed");
//         }
//     }

//     delay(100);

//     Wire.setClock(400000);
//     Wire.begin(11, 10);
//     delay(250);
//     Wire.beginTransmission(0x68);
//     Wire.write(0x6B);
//     Wire.write(0x00);
//     Wire.endTransmission();

//     ESP32PWM::allocateTimer(0);
//     ESP32PWM::allocateTimer(1);
//     ESP32PWM::allocateTimer(2);
//     ESP32PWM::allocateTimer(3);

//     delay(1000);
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

//     RateCalibrationRoll = 0.85;
//     RateCalibrationPitch = 0.15;
//     RateCalibrationYaw = -0.16;
//     AccXCalibration = 0.05;
//     AccYCalibration = 0.01;
//     AccZCalibration = -0.02;

//     LoopTimer = micros();
// }

// void loop(void)
// {

//     if (ReceiverValue[4] > 1500) // channel 5 for uploading values
//     {

//         PRateRoll = readFile(SPIFFS, "/pGain.txt").toFloat();
//         IRateRoll = readFile(SPIFFS, "/iGain.txt").toFloat();
//         DRateRoll = readFile(SPIFFS, "/dGain.txt").toFloat();

//         PRatePitch = PRateRoll;
//         IRatePitch = IRateRoll;
//         DRatePitch = DRateRoll;

//         PAngleRoll = readFile(SPIFFS, "/pAGain.txt").toFloat();
//         ;
//         IAngleRoll = readFile(SPIFFS, "/iAGain.txt").toFloat();
//         ;
//         DAngleRoll = readFile(SPIFFS, "/dAGain.txt").toFloat();
//         ;

//         PAnglePitch = PAngleRoll;
//         IAnglePitch = IAngleRoll;
//         DAnglePitch = DAngleRoll;

//         PRateYaw = readFile(SPIFFS, "/pYaw.txt").toFloat();
//         IRateYaw = readFile(SPIFFS, "/iYaw.txt").toFloat();
//         DRateYaw = readFile(SPIFFS, "/dYaw.txt").toFloat();

//         t = readFile(SPIFFS, "/tc.txt").toFloat();
//     }

//     // enter your loop code here
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1A);
//     Wire.write(0x05);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1C);
//     Wire.write(0x10);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x3B);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1B);
//     Wire.write(0x8);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x43);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t GyroX = Wire.read() << 8 | Wire.read();
//     int16_t GyroY = Wire.read() << 8 | Wire.read();
//     int16_t GyroZ = Wire.read() << 8 | Wire.read();
//     RateRoll = (float)GyroX / 65.5;
//     RatePitch = (float)GyroY / 65.5;
//     RateYaw = (float)GyroZ / 65.5;
//     AccX = (float)AccXLSB / 4096;
//     AccY = (float)AccYLSB / 4096;
//     AccZ = (float)AccZLSB / 4096;

//     RateRoll -= RateCalibrationRoll;
//     RatePitch -= RateCalibrationPitch;
//     RateYaw -= RateCalibrationYaw;

//     AccX -= AccXCalibration;
//     AccY -= AccYCalibration;
//     AccZ -= AccZCalibration;

//     AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
//     AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

//     // Debug: print raw sensor values ONCE per loop
//     static unsigned long lastDbgTime = 0;
//     if (millis() - lastDbgTime > 500)
//     {
//         Serial.printf("[SENSOR] AccX:%.3f AccY:%.3f | RateRoll:%.2f RatePitch:%.2f | AngleRoll:%.2f AnglePitch:%.2f\n",
//                       AccX, AccY, RateRoll, RatePitch, AngleRoll, AnglePitch);
//         lastDbgTime = millis();
//     }

//     complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
//     complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;
//     // Clamping complementary filter roll angle to ±20 degrees
//     complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
//     complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

//     DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
//     DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
//     InputThrottle = ReceiverValue[2];
//     DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500); // Inverted for correct rotation direction

//     // Inlined PID equation for Roll
//     ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
//     PtermRoll = PAngleRoll * ErrorAngleRoll;
//     ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
//     ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
//     DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
//     PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
//     PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
//     DesiredRateRoll = PIDOutputRoll;
//     PrevErrorAngleRoll = ErrorAngleRoll;
//     PrevItermAngleRoll = ItermRoll;

//     ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
//     PtermPitch = PAnglePitch * ErrorAnglePitch;
//     ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
//     ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
//     DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
//     PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
//     PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
//     DesiredRatePitch = PIDOutputPitch;
//     PrevErrorAnglePitch = ErrorAnglePitch;
//     PrevItermAnglePitch = ItermPitch;

//     // Compute errors
//     ErrorRateRoll = DesiredRateRoll - RateRoll;
//     ErrorRatePitch = DesiredRatePitch - RatePitch;
//     ErrorRateYaw = DesiredRateYaw - RateYaw;

//     // Roll Axis PID
//     PtermRoll = PRateRoll * ErrorRateRoll;
//     ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
//     ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
//     DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
//     PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
//     PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

//     // Update output and previous values for Roll
//     InputRoll = PIDOutputRoll;
//     PrevErrorRateRoll = ErrorRateRoll;
//     PrevItermRateRoll = ItermRoll;

//     // Pitch Axis PID
//     PtermPitch = PRatePitch * ErrorRatePitch;
//     ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
//     ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
//     DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
//     PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
//     PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

//     // Update output and previous values for Pitch
//     InputPitch = PIDOutputPitch;
//     PrevErrorRatePitch = ErrorRatePitch;
//     PrevItermRatePitch = ItermPitch;

//     // Yaw Axis PID
//     PtermYaw = PRateYaw * ErrorRateYaw;
//     ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
//     ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw); // Clamp ItermYaw to [-400, 400]
//     DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
//     PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
//     PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw); // Clamp PIDOutputYaw to [-400, 400]

//     // Update output and previous values for Yaw
//     InputYaw = PIDOutputYaw;
//     PrevErrorRateYaw = ErrorRateYaw;
//     PrevItermRateYaw = ItermYaw;

//     if (InputThrottle > 1800)
//     {
//         InputThrottle = 1800;
//     }

//     MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
//     MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
//     MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
//     MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // front left - clockwise

//     if (MotorInput1 > 2000)
//     {
//         MotorInput1 = 1999;
//     }

//     if (MotorInput2 > 2000)
//     {
//         MotorInput2 = 1999;
//     }

//     if (MotorInput3 > 2000)
//     {
//         MotorInput3 = 1999;
//     }

//     if (MotorInput4 > 2000)
//     {
//         MotorInput4 = 1999;
//     }

//     // int ThrottleIdle = 1150;
//     // int ThrottleCutOff = 1000;
//     if (MotorInput1 < ThrottleIdle)
//     {
//         MotorInput1 = ThrottleIdle;
//     }
//     if (MotorInput2 < ThrottleIdle)
//     {
//         MotorInput2 = ThrottleIdle;
//     }
//     if (MotorInput3 < ThrottleIdle)
//     {
//         MotorInput3 = ThrottleIdle;
//     }
//     if (MotorInput4 < ThrottleIdle)
//     {
//         MotorInput4 = ThrottleIdle;
//     }

//     if (ReceiverValue[2] < 1030 || isFailsafe()) // dont Arm the motors OR signal lost (failsafe)
//     {

//         MotorInput1 = ThrottleCutOff;
//         MotorInput2 = ThrottleCutOff;
//         MotorInput3 = ThrottleCutOff;
//         MotorInput4 = ThrottleCutOff;

//         PrevErrorRateRoll = 0;
//         PrevErrorRatePitch = 0;
//         PrevErrorRateYaw = 0;
//         PrevItermRateRoll = 0;
//         PrevItermRatePitch = 0;
//         PrevItermRateYaw = 0;
//         PrevErrorAngleRoll = 0;
//         PrevErrorAnglePitch = 0;
//         PrevItermAngleRoll = 0;
//         PrevItermAnglePitch = 0;

//         // Debug failsafe status
//         if (isFailsafe())
//         {
//             Serial.println("!!! FAILSAFE: Signal lost from TX - AUTO DISARM !!!");
//         }
//     }

//     // Calculate motor control values directly
//     mot1.writeMicroseconds(MotorInput1);
//     mot2.writeMicroseconds(MotorInput2);
//     mot3.writeMicroseconds(MotorInput3);
//     mot4.writeMicroseconds(MotorInput4);

//     // Debug output
//     Serial.printf("TX_Roll:%d TX_Pitch:%d | Roll:%.2f rate roll %.2f | Pitch:%.2f | PID_R:%.2f PID_P:%.2f PID_Y:%.2f | M1:%.0f M2:%.0f M3:%.0f M4:%.0f\n",
//                   ReceiverValue[0], ReceiverValue[1],
//                   complementaryAngleRoll, RateRoll,
//                   complementaryAnglePitch,
//                   InputRoll, InputPitch, InputYaw,
//                   MotorInput1, MotorInput2, MotorInput3, MotorInput4);

//     while (micros() - LoopTimer < (t * 1000000))
//         ;
//     LoopTimer = micros();
// }
