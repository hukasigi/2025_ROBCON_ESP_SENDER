#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <cmath>

#include "Debug.hpp"

bool g_debug_enabled = true;

const int8_t SLAVE_1 = 0x001; // スレーブ1のID（メインデータ送信用）
const int8_t SLAVE_2 = 0x002; // スレーブ2のID（予備）
const int8_t SLAVE_3 = 0x003; // スレーブ3のID（予備）

const int TX_PIN = 5; // CAN送信ピン（ESP32のGPIO5）
const int RX_PIN = 4; // CAN受信ピン（ESP32のGPIO4）

const double  MIN_CURRENT = -20.0;  // モーターの最小電流値（A）- 逆転最大
const double  MAX_CURRENT = 20.0;   // モーターの最大電流値（A）- 正転最大
const int16_t MIN_SENDNUM = -16384; // CAN送信データの最小値（16bit符号付き）
const int16_t MAX_SENDNUM = 16384;  // CAN送信データの最大値（16bit符号付き）

const uint8_t DEADZONE_STICK = 40; // スティックのデッドゾーン（テスト用に縮小）
const uint8_t DEADZONE_R2_L2 = 20; // R2/L2トリガーのデッドゾーン（テスト用に縮小）

const int SERIAL_BAUDRATE = 9600;
const int CAN_BAUDRATE = 100E3;

const char* PS4_BT_ADDRESS = "e4:65:b8:7e:07:02";

int16_t mapping_data(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
    double proportion = (x - in_min) / (in_max - in_min);
    double out_base = (double)(out_max - out_min);
    return out_min + out_base * proportion;
}

std::pair<int8_t, int8_t> split_data(int16_t formatted_data) {
    int8_t first_data  = (formatted_data >> 8) & 0xFF; // 上位バイト取得
    int8_t second_data = formatted_data & 0xFF;        // 下位バイト取得
    return {first_data, second_data};
}



class Packet {
    private:
        int id;
        int8_t buf[8];
    public:
        Packet(int set_id) {
            id = set_id;
            Init();
        }
        void Init() { // データバッファの初期化。インスタンス作成時以外も呼び出し可能とするため別関数に。
            for (int8_t& data : buf)
                data = 0x00;
        }
        int8_t& At(int num) { return buf[num]; }
        int Id() { return id; }
        void Send() {
            CAN.beginPacket(id);    // パケット送信開始
            for (int8_t data : buf) // 8バイト全てを送信
                CAN.write(data);
            CAN.endPacket(); // パケット送信完了
            delay(10);       // 安定性のための待機時間
        }
};

class RoboMasMotor {
    private:
        int id;
    public:
        RoboMasMotor(int set_id) { id = set_id; }
        int Id() { return id; }
        std::pair<int, int> ID_DATE() {
            int first  = (id - 1) * 2;     // 上位バイトの位置
            int second = (id - 1) * 2 + 1; // 下位バイトの位置
            return {first, second};
        }
        std::pair<int8_t, int8_t> SendBufByte(double speed_percentage) {
            double proportion = speed_percentage / 100.0;
            return split_data(mapping_data(20 * proportion, MIN_CURRENT, MAX_CURRENT, MIN_SENDNUM, MAX_SENDNUM));
        }
};

class Omnix4 {
    private:
        RoboMasMotor FrontLeftOmni  = RoboMasMotor(3); // 前左モーター(ID:3)
        RoboMasMotor BackLeftOmni   = RoboMasMotor(1); // 後左モーター(ID:1)
        RoboMasMotor BackRightOmni  = RoboMasMotor(2); // 後右モーター(ID:2)
        RoboMasMotor FrontRightOmni = RoboMasMotor(4); // 前右モーター(ID:4)
        Packet TxBuf = Packet(0x200);

        const double MAX_CONTROLLER_INPUT = 127.0;

        void MotorSpeedChange(RoboMasMotor motor, int speed_percentage) {
            auto byte_data = motor.SendBufByte(speed_percentage);
            auto position  = motor.ID_DATE();

            TxBuf.At(position.first)  = byte_data.first;  // 上位バイト
            TxBuf.At(position.second) = byte_data.second; // 下位バイト
        }

    public:
        Omnix4() {}

        void SendPacket() { TxBuf.Send(); }
        void Shift(int x, int y, double max_speed_percentage) {
            double distance = std::sqrt(x * x + y * y); // ピタゴラスの定理でベクトル長を算出

            if (distance == 0) {
                MotorSpeedChange(FrontLeftOmni, 0);
                MotorSpeedChange(BackLeftOmni, 0);
                MotorSpeedChange(BackRightOmni, 0);
                MotorSpeedChange(FrontRightOmni, 0);
                return;
            }

            double nx = x / distance; // X方向成分（-1.0～1.0）
            double ny = y / distance; // Y方向成分（-1.0～1.0）

            double max_input = 127.0;                       // PS4スティックの最大入力値
            if (distance > max_input) distance = max_input; // 上限制限
            double magnitude = distance / max_input;        // 0.0～1.0の強度

            double radian = atan2(ny, nx);

            radian -= PI / 4;

            double vector13 = std::cos(radian) * max_speed_percentage * magnitude; // 対角ベクトル1-3
            double vector24 = std::sin(radian) * max_speed_percentage * magnitude; // 対角ベクトル2-4

            debug_print("MOVE Vector13=");
            debug_print(vector13);
            debug_print("% Vector24=");
            debug_print(vector24);
            debug_println("%");
            debug_print("Motors: FL=");
            debug_print(vector13);
            debug_print(" BL=");
            debug_print(vector24);
            debug_print(" BR=");
            debug_print(-vector13);
            debug_print(" FR=");
            debug_println(-vector24);

            MotorSpeedChange(FrontLeftOmni, vector13);   // 前左 = ベクトル1-3成分
            MotorSpeedChange(BackLeftOmni, vector24);    // 後左 = ベクトル2-4成分
            MotorSpeedChange(BackRightOmni, -vector13);  // 後右 = ベクトル1-3の逆
            MotorSpeedChange(FrontRightOmni, -vector24); // 前右 = ベクトル2-4の逆
        }
        void Front() {
            MotorSpeedChange(FrontLeftOmni, 100);
            MotorSpeedChange(FrontRightOmni, 100);
            MotorSpeedChange(BackLeftOmni, -100);
            MotorSpeedChange(BackRightOmni, -100);
        }
        void Back() {
            MotorSpeedChange(FrontLeftOmni, -100);
            MotorSpeedChange(FrontRightOmni, -100);
            MotorSpeedChange(BackLeftOmni, 100);
            MotorSpeedChange(BackRightOmni, 100);
        }

        void R_Turn(u_int8_t R2_val, double speed_percentage) {
            double R2_persentage = R2_val / 255.0;

            MotorSpeedChange(FrontLeftOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(BackLeftOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(BackRightOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(FrontRightOmni, speed_percentage * R2_persentage);
        }

        void L_Turn(uint8_t L2_val, double speed_percentage) {
            double L2_persetage = L2_val / 255.0;

            MotorSpeedChange(FrontLeftOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(BackLeftOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(BackRightOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(FrontRightOmni, -speed_percentage * L2_persetage);
        }

        void Stop() {
            MotorSpeedChange(FrontLeftOmni, 0);
            MotorSpeedChange(BackLeftOmni, 0);
            MotorSpeedChange(BackRightOmni, 0);
            MotorSpeedChange(FrontRightOmni, 0);
        }

        void TestMove(double x) {
            MotorSpeedChange(FrontLeftOmni, x);     // 前左モーター
            MotorSpeedChange(BackRightOmni, 0 - x); // 後右モーター（逆方向）
        }
};

Omnix4 TestOmni = Omnix4();

int8_t DeadZone(int16_t value, int ZONE) {
    return (abs(value) < ZONE) ? 0 : value;
}

uint8_t packButtons(bool circle, bool triangle, bool square, bool cross, bool L1, bool L2, bool R1, bool R2) {
    return (circle ? (1 << 0) : 0) |   // bit0: Circle
           (triangle ? (1 << 1) : 0) | // bit1: Triangle
           (square ? (1 << 2) : 0) |   // bit2: Square
           (cross ? (1 << 3) : 0) |    // bit3: Cross
           (L1 ? (1 << 4) : 0) |       // bit4: L1
           (L2 ? (1 << 5) : 0) |       // bit5: L2
           (R1 ? (1 << 6) : 0) |       // bit6: R1
           (R2 ? (1 << 7) : 0);        // bit7: R2
}

void setup() {
    debug_begin(SERIAL_BAUDRATE);
    PS4.begin(PS4_BT_ADDRESS);

    CAN.setPins(RX_PIN, TX_PIN);

    if (!CAN.begin(1000E3)) {
        debug_println("CAN Init Failed");
        while (1)
            ; // CAN初期化失敗時は無限ループで停止
    }

    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;

    debug_println("Ready"); // システム準備完了の通知
}

void loop() {
    // 1. コントローラー接続状態の確認と安全処理
    if (!PS4.isConnected()) {
        // コントローラー未接続時は安全のため全モーター停止
        TestOmni.Stop();
        return; // 以降の処理をスキップしてloop()を再開
    }

    int8_t  l_x = DeadZone(PS4.LStickX(), DEADZONE_STICK); // 左スティックX軸（移動用）
    int8_t  l_y = DeadZone(PS4.LStickY(), DEADZONE_STICK); // 左スティックY軸（移動用）
    uint8_t r_x = DeadZone(PS4.RStickX(), DEADZONE_STICK); // 右スティックX軸（未使用）
    uint8_t r_y = DeadZone(PS4.RStickY(), DEADZONE_STICK); // 右スティックY軸（未使用）

    int R2_val = PS4.R2Value(); // 右回転用トリガー
    int L2_val = PS4.L2Value(); // 左回転用トリガー

    uint8_t btns_1 =
        packButtons(PS4.Circle(), PS4.Triangle(), PS4.Square(), PS4.Cross(), PS4.L1(), PS4.R1(), PS4.Left(), PS4.Right());

    CAN.beginPacket(SLAVE_1);

    CAN.write(btns_1); // Byte0: 8ボタン状態（ビットパック）
    CAN.write(r_x);    // Byte1: 右スティックX軸
    CAN.write(r_y);    // Byte2: 右スティックY軸
    CAN.write(1);      // Byte3: 予備データ
    CAN.write(1);      // Byte4: 予備データ
    CAN.write(1);      // Byte5: 予備データ
    CAN.write(1);      // Byte6: 予備データ
    CAN.write(1);      // Byte7: 予備データ

    CAN.endPacket(); // パケット送信完了
    CAN.beginPacket(SLAVE_2);

    CAN.write(btns_1); // Byte0: 8ボタン状態（ビットパック）
    CAN.write(r_x);    // Byte1: 右スティックX軸
    CAN.write(r_y);    // Byte2: 右スティックY軸
    CAN.write(1);      // Byte3: 予備データ
    CAN.write(1);      // Byte4: 予備データ
    CAN.write(1);      // Byte5: 予備データ
    CAN.write(1);      // Byte6: 予備データ
    CAN.write(1);      // Byte7: 予備データ

    CAN.endPacket(); // パケット送信完了

    if (R2_val > 0) {
        TestOmni.R_Turn(R2_val, 100.0); // 100%を最大回転速度として設定
        TestOmni.SendPacket();          // モーター制御データをCAN送信

    } else if (L2_val > 0) {
        TestOmni.L_Turn(L2_val, 100.0); // 100%を最大回転速度として設定
        TestOmni.SendPacket();
        debug_println("L2"); // モーター制御データをCAN送信

    } else if (PS4.Up()) {
        TestOmni.Front();
        TestOmni.SendPacket();

    } else if (PS4.Down()) {
        TestOmni.Back();
        TestOmni.SendPacket();

    } else {
        TestOmni.Shift(l_x, l_y, 100.0); // 100%を最大移動速度として設定
        TestOmni.SendPacket();           // モーター制御データをCAN送信
    }

    debug_print("LStick: X=");
    debug_print(l_x);
    debug_print(" Y=");
    debug_println(l_y);
    debug_print("Triggers: R2=");
    debug_print(R2_val);
    debug_print(" L2=");
    debug_println(L2_val);
    debug_print(PS4.R2Value());

    debug_print("RIGHT TURN - Intensity: ");
    debug_print((R2_val / 255.0) * 100);
    debug_println("%");
    debug_print("LEFT TURN - Intensity: ");
    debug_print((L2_val / 255.0) * 100);
    debug_println("%");
    double distance  = std::sqrt(l_x * l_x + l_y * l_y);
    double magnitude = distance / 127.0;
    debug_print("MOVE - Distance: ");
    debug_print(distance);
    debug_print(" Intensity: ");
    debug_print(magnitude * 100);
    debug_println("%");
    debug_println("---");

    delay(5);
}