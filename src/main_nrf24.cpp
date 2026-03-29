// #include <Arduino.h>
// #include <ESP32Servo.h>

// // mot4   mot1
// //     x
// // mot3   mot2
// Servo mot1, mot2, mot3, mot4;
// const int mot1_pin = 5;
// const int mot2_pin = 7;
// const int mot3_pin = 21;
// const int mot4_pin = 47;

// void setup()
// {
//     Serial.begin(115200);
//     delay(500);

//     mot1.setPeriodHertz(50);
//     mot2.setPeriodHertz(50);
//     mot3.setPeriodHertz(50);
//     mot4.setPeriodHertz(50);

//     mot1.attach(mot1_pin, 1000, 2000);
//     mot2.attach(mot2_pin, 1000, 2000);
//     mot3.attach(mot3_pin, 1000, 2000);
//     mot4.attach(mot4_pin, 1000, 2000);

//     // Arm tất cả ESC
//     mot1.writeMicroseconds(1000);
//     mot2.writeMicroseconds(1000);
//     mot3.writeMicroseconds(1000);
//     mot4.writeMicroseconds(1000);
//     Serial.println("Arming ESC... cho 3 giay");
//     delay(3000);
//     Serial.println("Arm xong! Bat dau test...");
//     delay(1000);

//     // ===== TEST M1 =====
//     Serial.println("=== TEST M1 (pin 5) - dong co nao quay? ===");
//     mot1.writeMicroseconds(1250);
//     delay(3000);
//     mot1.writeMicroseconds(1000);
//     Serial.println("M1 xong");
//     delay(2000);

//     // ===== TEST M2 =====
//     Serial.println("=== TEST M2 (pin 7) - dong co nao quay? ===");
//     mot2.writeMicroseconds(1250);
//     delay(3000);
//     mot2.writeMicroseconds(1000);
//     Serial.println("M2 xong");
//     delay(2000);

//     // ===== TEST M3 =====
//     Serial.println("=== TEST M3 (pin 21) - dong co nao quay? ===");
//     mot3.writeMicroseconds(1250);
//     delay(3000);
//     mot3.writeMicroseconds(1000);
//     Serial.println("M3 xong");
//     delay(2000);

//     // ===== TEST M4 =====
//     Serial.println("=== TEST M4 (pin 47) - dong co nao quay? ===");
//     mot4.writeMicroseconds(1250);
//     delay(3000);
//     mot4.writeMicroseconds(1000);
//     Serial.println("M4 xong");
//     delay(2000);

//     Serial.println("=== TEST HOAN THANH ===");
//     Serial.println("Bao lai dong co vat ly nao quay theo M1/M2/M3/M4");
// }

// void loop() {}