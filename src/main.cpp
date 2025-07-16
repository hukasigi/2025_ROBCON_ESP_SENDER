#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>

const int8_t SLAVE_1 = 0x001; // スレーブ1のID
const int8_t SLAVE_2 = 0x002; // スレーブ2のID
const int8_t SLAVE_3 = 0x003; // スレーブ3のID

// デッドゾーン処理（–128…127 の範囲で扱う）
int8_t DeadZone(int16_t value, int ZONE = 10) {
    return (abs(value) < ZONE) ? 0 : value;
}

// ボタン８つを 1 バイトにビットパック
uint8_t packButtons(bool circle, bool triangle, bool square, bool cross, bool L1, bool L2, bool R1, bool R2) {
    return (circle ? (1 << 0) : 0) | (triangle ? (1 << 1) : 0) | (square ? (1 << 2) : 0) | (cross ? (1 << 3) : 0) |
           (L1 ? (1 << 4) : 0) | (L2 ? (1 << 5) : 0) | (R1 ? (1 << 6) : 0) | (R2 ? (1 << 7) : 0);
}

void setup() {
    Serial.begin(115200);
    // while (!Serial)
    //     ;
    // PS4.begin("08:b6:1f:ed:44:32");
    PS4.begin("48:e7:29:a3:c5:0e");

    CAN.setPins(4, 5);
    if (!CAN.begin(1000E3)) {
        Serial.println("CAN Init Failed");
        while (1)
            ;
    }
    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;
    Serial.println("Ready");
}

void loop() {
    // 1) スティック値取得（–128…127）
    int8_t l_x = DeadZone(PS4.LStickX());
    int8_t l_y = DeadZone(PS4.LStickY());
    int8_t r_x = DeadZone(PS4.RStickX());
    int8_t r_y = DeadZone(PS4.RStickY());

    // 2) ボタンをビットパック
    uint8_t btns = packButtons(PS4.Circle(), PS4.Triangle(), PS4.Square(), PS4.Cross(), PS4.L1(), PS4.L2(), PS4.R1(), PS4.R2());

    // 3) CANフレーム送信
    CAN.beginPacket(SLAVE_1);

    CAN.write(l_x);  // Byte1: LStick X
    CAN.write(l_y);  // Byte2: LStick Y
    CAN.write(r_x);  // Byte3: RStick X
    CAN.write(r_y);  // Byte4: RStick Y
    CAN.write(btns); // Byte0: ボタン８つ
    CAN.write(1);
    CAN.write(2);
    CAN.write(3);

    CAN.endPacket();

    Serial.println(l_x);
    Serial.println(l_y);
    Serial.println(r_x);
    Serial.println(r_y);
    Serial.println(btns);

    Serial.println("Sent button+stick");
    delay(10);
}