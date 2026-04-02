#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char *AP_SSID = "KL_Drone";
const char *AP_PASS = "88888888";
AsyncWebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KL DRONE PID Tuner</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui;background:#1a1a2e;color:#eee;padding:12px;max-width:480px;margin:auto}
h1{text-align:center;font-size:1.3em;margin:10px 0;color:#0ff}
.card{background:#16213e;border-radius:10px;padding:14px;margin:10px 0;border:1px solid #0f3460}
.card h2{font-size:1em;margin-bottom:10px;color:#e94560}
.row{display:flex;align-items:center;margin:6px 0;gap:8px}
.row label{width:28px;font-weight:bold;font-size:0.9em}
.row input[type=number]{flex:1;background:#0f3460;border:1px solid #555;color:#fff;padding:8px;border-radius:6px;font-size:1em;text-align:center}
.btn{width:100%;padding:12px;border:none;border-radius:8px;font-size:1em;font-weight:bold;cursor:pointer;margin:4px 0}
.btn-send{background:#e94560;color:#fff}
.btn-send:active{background:#c73650}
.btn-refresh{background:#0f3460;color:#0ff}
.status{text-align:center;padding:10px;font-size:0.85em;color:#0f0;min-height:24px;border-radius:6px;margin:8px 0;font-weight:bold;word-break:break-all}
.live{background:#0d1b2a;border-radius:8px;padding:10px;margin:8px 0;font-family:monospace;font-size:0.85em;line-height:1.6}
</style>
</head><body>
<h1>KL DRONE - PID Tuner</h1>

<div class="card">
<h2>Roll / Pitch (Rate PID)</h2>
<div class="row"><label>Kp</label><input type="number" id="rp_kp" step="0.01" value="1.64"></div>
<div class="row"><label>Ki</label><input type="number" id="rp_ki" step="0.01" value="0.935"></div>
<div class="row"><label>Kd</label><input type="number" id="rp_kd" step="0.001" value="0.085"></div>
</div>

<div class="card">
<h2>Yaw (Rate PID)</h2>
<div class="row"><label>Kp</label><input type="number" id="y_kp" step="0.01" value="0.05"></div>
<div class="row"><label>Ki</label><input type="number" id="y_ki" step="0.01" value="0.05"></div>
<div class="row"><label>Kd</label><input type="number" id="y_kd" step="0.0001" value="0.0005"></div>
</div>

<div class="card">
<h2>Angle P (outer loop)</h2>
<div class="row"><label>Kp</label><input type="number" id="a_kp" step="0.1" value="4.0"></div>
</div>

<button class="btn btn-send" onclick="sendPID()">SEND</button>
<button class="btn btn-refresh" onclick="loadPID()">REFRESH</button>
<div class="status" id="st"></div>

<div class="live" id="live">Roll: -- | Pitch: -- | Yaw rate: --</div>

<script>
function $(id){return document.getElementById(id)}
function st(msg,ok){$('st').textContent=msg;$('st').style.color=ok?'#0f0':'#f55'}

function loadPID(){
  fetch('/values').then(r=>r.json()).then(d=>{
    $('rp_kp').value=d.rp_kp;$('rp_ki').value=d.rp_ki;$('rp_kd').value=d.rp_kd;
    $('y_kp').value=d.y_kp;$('y_ki').value=d.y_ki;$('y_kd').value=d.y_kd;
    $('a_kp').value=d.a_kp;
    st('Loaded OK',true);
  }).catch(e=>st('Load fail: '+e,false));
}

function sendPID(){
  let v={rp_kp:$('rp_kp').value, rp_ki:$('rp_ki').value, rp_kd:$('rp_kd').value,
    y_kp:$('y_kp').value, y_ki:$('y_ki').value, y_kd:$('y_kd').value,
    a_kp:$('a_kp').value};
  st('Dang gui...',true);
  fetch('/set?'+new URLSearchParams(v)).then(r=>r.json()).then(d=>{
    if(d.ok){
      st('OK! RP('+v.rp_kp+','+v.rp_ki+','+v.rp_kd+') Y('+v.y_kp+','+v.y_ki+','+v.y_kd+') Angle='+v.a_kp,true);
    }else{
      st(d.err||'Loi khong xac dinh',false);
    }
  }).catch(e=>st('Mat ket noi: '+e,false));
}

function pollLive(){
  fetch('/live').then(r=>r.json()).then(d=>{
    $('live').innerHTML='Roll: '+d.roll.toFixed(1)+'&deg; | Pitch: '+d.pitch.toFixed(1)+'&deg;<br>'
      +'Arm: '+(d.armed?'<span style="color:#0f0">YES</span>':'<span style="color:#f55">NO</span>')
      +' | Thr: '+d.thr
      +' | CH6: '+(d.ch6?'<span style="color:#0f0">ON</span>':'<span style="color:#f55">OFF</span>');
  }).catch(()=>{});
}

loadPID();
setInterval(pollLive,500);
</script>
</body></html>
)rawliteral";

// mot4   mot1
//     x
// mot3   mot2
Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 17;
const int mot2_pin = 18;
const int mot3_pin = 8;
const int mot4_pin = 3;

static const int ESC_HZ = 50;
static const int IDLE_US = 1080;
static const int THR_MAX_FOR_PID = 1850;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0, last_channel_2 = 0, last_channel_3 = 0, last_channel_4 = 0, last_channel_5 = 0, last_channel_6 = 0;
volatile uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;

volatile int ReceiverValue[6] = {1500, 1500, 1000, 1500, 1000, 1000};

const int channel_1_pin = 37;
const int channel_2_pin = 38;
const int channel_3_pin = 39;
const int channel_4_pin = 40;
const int channel_5_pin = 41;
const int channel_6_pin = 42;
void IRAM_ATTR channelInterruptHandler()
{
    current_time = micros();

    if (digitalRead(channel_1_pin))
    {
        if (!last_channel_1)
        {
            last_channel_1 = 1;
            timer_1 = current_time;
        }
    }
    else
    {
        if (last_channel_1)
        {
            last_channel_1 = 0;
            ReceiverValue[0] = (int)(current_time - timer_1);
        }
    }

    if (digitalRead(channel_2_pin))
    {
        if (!last_channel_2)
        {
            last_channel_2 = 1;
            timer_2 = current_time;
        }
    }
    else
    {
        if (last_channel_2)
        {
            last_channel_2 = 0;
            ReceiverValue[1] = (int)(current_time - timer_2);
        }
    }

    if (digitalRead(channel_3_pin))
    {
        if (!last_channel_3)
        {
            last_channel_3 = 1;
            timer_3 = current_time;
        }
    }
    else
    {
        if (last_channel_3)
        {
            last_channel_3 = 0;
            ReceiverValue[2] = (int)(current_time - timer_3);
        }
    }

    if (digitalRead(channel_4_pin))
    {
        if (!last_channel_4)
        {
            last_channel_4 = 1;
            timer_4 = current_time;
        }
    }
    else
    {
        if (last_channel_4)
        {
            last_channel_4 = 0;
            ReceiverValue[3] = (int)(current_time - timer_4);
        }
    }

    if (digitalRead(channel_5_pin))
    {
        if (!last_channel_5)
        {
            last_channel_5 = 1;
            timer_5 = current_time;
        }
    }
    else
    {
        if (last_channel_5)
        {
            last_channel_5 = 0;
            ReceiverValue[4] = (int)(current_time - timer_5);
        }
    }

    if (digitalRead(channel_6_pin))
    {
        if (!last_channel_6)
        {
            last_channel_6 = 1;
            timer_6 = current_time;
        }
    }
    else
    {
        if (last_channel_6)
        {
            last_channel_6 = 0;
            ReceiverValue[5] = (int)(current_time - timer_6);
        }
    }
}

static const uint8_t MPU_ADDR = 0x68;
static const int I2C_SDA = 11;
static const int I2C_SCL = 10;

static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float GYRO_LSB_PER_DPS = 16.4f;
static const float ACC_LSB_PER_G = 16384.0f;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float levelOffsetRollDeg = 0, levelOffsetPitchDeg = 0;

static const int SIGN_GX = +1;
static const int SIGN_GY = +1;
static const int SIGN_GZ = +1;

bool i2cWriteByte(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool i2cReadBytes(uint8_t reg, uint8_t n, uint8_t *buf)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)n, (int)true);
    if (got != n)
        return false;
    for (uint8_t i = 0; i < n; i++)
        buf[i] = Wire.read();
    return true;
}

void mpuInit()
{
    i2cWriteByte(REG_PWR_MGMT_1, 0x00);
    delay(50);

    i2cWriteByte(REG_CONFIG, 0x04);
    i2cWriteByte(REG_SMPLRT_DIV, 0x03);
    i2cWriteByte(REG_GYRO_CONFIG, 0x18);
    delay(50);
}

bool mpuReadAccelGyro(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t b[14];
    if (!i2cReadBytes(REG_ACCEL_XOUT_H, 14, b))
        return false;

    ax = (int16_t)((b[0] << 8) | b[1]);
    ay = (int16_t)((b[2] << 8) | b[3]);
    az = (int16_t)((b[4] << 8) | b[5]);
    gx = (int16_t)((b[8] << 8) | b[9]);
    gy = (int16_t)((b[10] << 8) | b[11]);
    gz = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

void mpuCalibrateGyro(int samples = 2000)
{
    long sx = 0, sy = 0, sz = 0;
    int good = 0;

    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (mpuReadAccelGyro(ax, ay, az, gx, gy, gz))
        {
            sx += gx;
            sy += gy;
            sz += gz;
            good++;
        }
        delay(2);
    }

    if (good > 0)
    {
        gyroBiasX = (float)sx / good;
        gyroBiasY = (float)sy / good;
        gyroBiasZ = (float)sz / good;
    }
}

void mpuCalibrateLevel(int samples = 400)
{
    float sumRoll = 0.0f, sumPitch = 0.0f;
    int good = 0;

    for (int i = 0; i < samples; i++)
    {
        int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
        if (mpuReadAccelGyro(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw))
        {
            float ax = axRaw / ACC_LSB_PER_G;
            float ay = ayRaw / ACC_LSB_PER_G;
            float az = azRaw / ACC_LSB_PER_G;

            sumRoll += atan2f(ay, az) * RAD_TO_DEG;
            sumPitch += atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
            good++;
        }
        delay(2);
    }

    if (good > 0)
    {
        levelOffsetRollDeg = sumRoll / good;
        levelOffsetPitchDeg = sumPitch / good;
    }
}

struct PID
{
    float kp, ki, kd;
    float integrator, prevError;
    float outMin, outMax;
    float iLimit;
    float alpha_derivative;      // Filter coefficient
    float derivative_filtered;   // Filtered derivative value

    PID()
        : kp(0), ki(0), kd(0),
          integrator(0), prevError(0),
          outMin(-400), outMax(400), iLimit(200),
          alpha_derivative(0.012f), derivative_filtered(0) {}

    PID(float p, float i, float d, float omin, float omax, float ilimit)
        : kp(p), ki(i), kd(d),
          integrator(0), prevError(0),
          outMin(omin), outMax(omax), iLimit(ilimit),
          alpha_derivative(0.012f), derivative_filtered(0) {}

    PID(float p, float i, float d, float omin, float omax, float ilimit, float alpha)
        : kp(p), ki(i), kd(d),
          integrator(0), prevError(0),
          outMin(omin), outMax(omax), iLimit(ilimit),
          alpha_derivative(alpha), derivative_filtered(0) {}

    float update(float setpoint, float measurement, float dt)
    {
        float error = setpoint - measurement;

        // Integral term
        integrator += error * ki * dt;
        if (integrator > iLimit)
            integrator = iLimit;
        if (integrator < -iLimit)
            integrator = -iLimit;

        // Derivative term with low-pass filter (EMA)
        float derivative_raw = (error - prevError) / dt;
        derivative_filtered = (1.0f - alpha_derivative) * derivative_filtered
                            + alpha_derivative * derivative_raw;
        prevError = error;

        // PID output
        float out = kp * error + integrator + kd * derivative_filtered;
        if (out > outMax)
            out = outMax;
        if (out < outMin)
            out = outMin;
        return out;
    }

    void reset()
    {
        integrator = 0;
        prevError = 0;
        derivative_filtered = 0;
    }
};

PID pidRoll(1.6f, 1.5f, 0.0085f, -400, 400, 400);
PID pidPitch(1.6f, 1.5f, 0.0085f, -400, 400, 400);
PID pidYaw(4.0f, 3.0f, 0.0005f, -300, 300, 150);

float KpAngleRoll = 4.0f;
float KpAnglePitch = 4.0f;

portMUX_TYPE pidMux = portMUX_INITIALIZER_UNLOCKED;

float rollTrimDeg = 0.38f;
float pitchTrimDeg = 0.0f;

const float MAX_ANGLE_DEG = 30.0f;
const float MAX_RATE_RP = 200.0f;
const float MAX_RATE_YAW = 120.0f;

static float rollDeg = 0.0f, pitchDeg = 0.0f;

static const float RAD2DEG = 57.2957795f;

static const float ALPHA_BASE = 0.9885f;
static const float ACC_TRUST_ERR_G = 0.20f;

static const uint32_t LOOP_US = 4000;
uint32_t LoopTimer = 0;

static inline int clampInt(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static inline float applyDeadband(float x, float db)
{
    if (x > -db && x < db)
        return 0;
    return x;
}

float rcNorm(int rcUs)
{
    rcUs = clampInt(rcUs, 1000, 2000);
    float x = (rcUs - 1500) / 500.0f;
    return applyDeadband(x, 0.04f);
}

float rcToAngle(int rcUs, float maxAngleDeg) { return rcNorm(rcUs) * maxAngleDeg; }
float rcToRate(int rcUs, float maxRateDps) { return rcNorm(rcUs) * maxRateDps; }

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void setupWebServer()
{
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("WiFi AP started: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    server.on("/values", HTTP_GET, [](AsyncWebServerRequest *req) {
        char buf[256];
        portENTER_CRITICAL(&pidMux);
        snprintf(buf, sizeof(buf),
            "{\"rp_kp\":%.4f,\"rp_ki\":%.4f,\"rp_kd\":%.4f,"
            "\"y_kp\":%.4f,\"y_ki\":%.4f,\"y_kd\":%.4f,"
            "\"a_kp\":%.2f}",
            pidRoll.kp, pidRoll.ki, pidRoll.kd,
            pidYaw.kp, pidYaw.ki, pidYaw.kd,
            KpAngleRoll);
        portEXIT_CRITICAL(&pidMux);
        req->send(200, "application/json", buf);
    });

    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (ReceiverValue[5] < 1500) {
            req->send(200, "application/json", "{\"ok\":false,\"err\":\"Bat CH6 truoc khi chinh PID!\"}");
            return;
        }
        portENTER_CRITICAL(&pidMux);
        if (req->hasParam("rp_kp")) { pidRoll.kp = pidPitch.kp = req->getParam("rp_kp")->value().toFloat(); }
        if (req->hasParam("rp_ki")) { pidRoll.ki = pidPitch.ki = req->getParam("rp_ki")->value().toFloat(); }
        if (req->hasParam("rp_kd")) { pidRoll.kd = pidPitch.kd = req->getParam("rp_kd")->value().toFloat(); }
        if (req->hasParam("y_kp"))  { pidYaw.kp = req->getParam("y_kp")->value().toFloat(); }
        if (req->hasParam("y_ki"))  { pidYaw.ki = req->getParam("y_ki")->value().toFloat(); }
        if (req->hasParam("y_kd"))  { pidYaw.kd = req->getParam("y_kd")->value().toFloat(); }
        if (req->hasParam("a_kp"))  { KpAngleRoll = KpAnglePitch = req->getParam("a_kp")->value().toFloat(); }
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
        portEXIT_CRITICAL(&pidMux);
        Serial.printf("PID updated: RP(%.3f,%.3f,%.3f) Y(%.3f,%.3f,%.4f) A(%.1f)\n",
            pidRoll.kp, pidRoll.ki, pidRoll.kd,
            pidYaw.kp, pidYaw.ki, pidYaw.kd, KpAngleRoll);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/live", HTTP_GET, [](AsyncWebServerRequest *req) {
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"roll\":%.2f,\"pitch\":%.2f,\"armed\":%s,\"thr\":%d,\"ch6\":%s}",
            rollDeg, pitchDeg,
            (ReceiverValue[4] > 1500) ? "true" : "false",
            ReceiverValue[2],
            (ReceiverValue[5] > 1500) ? "true" : "false");
        req->send(200, "application/json", buf);
    });

    server.begin();
    Serial.println("Web server started on port 80");
}

void setup()
{
    Serial.begin(115200);
    setupWebServer();

    pinMode(channel_1_pin, INPUT);
    pinMode(channel_2_pin, INPUT);
    pinMode(channel_3_pin, INPUT);
    pinMode(channel_4_pin, INPUT);
    pinMode(channel_5_pin, INPUT);
    pinMode(channel_6_pin, INPUT);

    attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    mpuInit();
    Serial.println("Dang calib gyro... DUNG YEN!");
    mpuCalibrateGyro(2000);
    Serial.println("Calib xong.");
    Serial.println("Dang calib level... DAT MAY PHANG!");
    mpuCalibrateLevel(500);
    Serial.print("Level offset roll=");
    Serial.print(levelOffsetRollDeg, 2);
    Serial.print(" pitch=");
    Serial.println(levelOffsetPitchDeg, 2);

    // Lấy góc ban đầu từ accel
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (mpuReadAccelGyro(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw))
    {
        float ax = axRaw / ACC_LSB_PER_G;
        float ay = ayRaw / ACC_LSB_PER_G;
        float az = azRaw / ACC_LSB_PER_G;

        float rollAcc = atan2f(ay, az) * RAD2DEG - levelOffsetRollDeg;
        float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG - levelOffsetPitchDeg;

        rollDeg = rollAcc;
        pitchDeg = pitchAcc;
    }

    mot1.setPeriodHertz(ESC_HZ);
    mot2.setPeriodHertz(ESC_HZ);
    mot3.setPeriodHertz(ESC_HZ);
    mot4.setPeriodHertz(ESC_HZ);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);

    // Arm ESC
    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
    delay(1500);

    LoopTimer = micros();
    Serial.println("Setup xong -> vao loop dieu khien");
}

void loop()
{
    static uint32_t lastDbgMs = 0;

    while ((uint32_t)(micros() - LoopTimer) < LOOP_US)
    {
    }
    LoopTimer += LOOP_US;
    const float dt = LOOP_US * 1e-6f;

    int ch[6];
    noInterrupts();
    for (int i = 0; i < 6; i++)
        ch[i] = ReceiverValue[i];
    interrupts();

    for (int i = 0; i < 6; i++)
    {
        if (ch[i] < 900 || ch[i] > 2100)
            ch[i] = (i == 2) ? 1000 : 1500;
    }

    int rcRoll = clampInt(ch[0], 1000, 2000);
    int rcPitch = clampInt(ch[1], 1000, 2000);
    int rcThrottle = clampInt(ch[2], 1000, 2000);
    int rcYaw = clampInt(ch[3], 1000, 2000);
    int rcArm = clampInt(ch[4], 1000, 2000);

    bool armed = (rcArm > 1500);

    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    bool ok = mpuReadAccelGyro(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

    float rateRoll = 0, ratePitch = 0, rateYaw = 0;

    if (ok)
    {
        // Gyro deg/s
        rateRoll = SIGN_GX * (((float)gxRaw - gyroBiasX) / GYRO_LSB_PER_DPS);
        ratePitch = SIGN_GY * (((float)gyRaw - gyroBiasY) / GYRO_LSB_PER_DPS);
        rateYaw = SIGN_GZ * (((float)gzRaw - gyroBiasZ) / GYRO_LSB_PER_DPS);

        // Acc g
        float ax = axRaw / ACC_LSB_PER_G;
        float ay = ayRaw / ACC_LSB_PER_G;
        float az = azRaw / ACC_LSB_PER_G;

        // Góc từ accel (deg)
        float rollAcc = atan2f(ay, az) * RAD_TO_DEG - levelOffsetRollDeg;
        float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - levelOffsetPitchDeg;

        // ====== “tin accel” theo độ gần 1g (chặn rung/tăng tốc) ======
        float aMag = sqrtf(ax * ax + ay * ay + az * az); // ~1.0 khi đứng yên
        float err = fabsf(aMag - 1.0f);

        // trust: 1 (tin accel) khi err nhỏ; 0 (không tin accel) khi err lớn
        float trust = 1.0f - (err / ACC_TRUST_ERR_G);
        trust = clampf(trust, 0.0f, 1.0f);

        // alpha động: rung mạnh => alpha gần 1 (chủ yếu gyro)
        float alpha = 1.0f - trust * (1.0f - ALPHA_BASE);

        // ====== Complementary filter (THAY KALMAN) ======
        rollDeg = alpha * (rollDeg + rateRoll * dt) + (1.0f - alpha) * rollAcc;
        pitchDeg = alpha * (pitchDeg + ratePitch * dt) + (1.0f - alpha) * pitchAcc;
    }

    float targetRollDeg = rcToAngle(rcRoll, MAX_ANGLE_DEG) + rollTrimDeg;
    float targetPitchDeg = rcToAngle(rcPitch, MAX_ANGLE_DEG) + pitchTrimDeg;

    float spRollRate = KpAngleRoll * (targetRollDeg - rollDeg);
    float spPitchRate = KpAnglePitch * (targetPitchDeg - pitchDeg);

    spRollRate = constrain(spRollRate, -MAX_RATE_RP, MAX_RATE_RP);
    spPitchRate = constrain(spPitchRate, -MAX_RATE_RP, MAX_RATE_RP);

    float spYawRate = rcToRate(rcYaw, MAX_RATE_YAW);

    float outRoll = 0, outPitch = 0, outYaw = 0;

    portENTER_CRITICAL(&pidMux);
    if (!armed)
    {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
    }
    else
    {
        outRoll = pidRoll.update(spRollRate, rateRoll, dt);
        outPitch = pidPitch.update(spPitchRate, ratePitch, dt);
        //outPitch= 0;
        outYaw = pidYaw.update(spYawRate, rateYaw, dt);
       // outYaw = 0;
    }
    portEXIT_CRITICAL(&pidMux);

    int baseThrottle = rcThrottle;
    if (baseThrottle > THR_MAX_FOR_PID)
        baseThrottle = THR_MAX_FOR_PID;

    float m1 = baseThrottle + (-outPitch) + (-outRoll) + (outYaw);
    float m2 = baseThrottle + (+outPitch) + (-outRoll) + (-outYaw);
    float m3 = baseThrottle + (+outPitch) + (+outRoll) + (outYaw);
    float m4 = baseThrottle + (-outPitch) + (+outRoll) + (-outYaw);

    int motor1 = clampInt((int)m1, 1000, 2000);
    int motor2 = clampInt((int)m2, 1000, 2000);
    int motor3 = clampInt((int)m3, 1000, 2000);
    int motor4 = clampInt((int)m4, 1000, 2000);

    mot1.writeMicroseconds(motor1);
    mot2.writeMicroseconds(motor2);
    mot3.writeMicroseconds(motor3);
    mot4.writeMicroseconds(motor4);

    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastDbgMs) >= 500)
    {
        lastDbgMs = nowMs;
        Serial.print("loop ok | arm=");
        Serial.print(armed ? 1 : 0);
        Serial.print(" ch3(thr)=");
        Serial.print(rcThrottle);
        Serial.print(" ch5(arm)=");
        Serial.print(rcArm);
        Serial.print(" mpu=");
        Serial.print(ok ? "OK" : "ERR");
        Serial.print(" roll=");
        Serial.print(rollDeg, 2);
        Serial.print(" pitch=");
        Serial.println(pitchDeg, 2);
        Serial.print(" M1 = ");
        Serial.print(motor1);
        Serial.print(" M2 = ");
        Serial.print(motor2);
        Serial.print(" M3 = ");
        Serial.print(motor3);
        Serial.print(" M4 = ");
        Serial.println(motor4);
    }
}
