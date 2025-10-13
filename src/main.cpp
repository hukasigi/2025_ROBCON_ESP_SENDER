#include <PS4Controller.h>
#include <cmath>

#include "packet.hpp"

const int8_t SLAVE_1 = 0x001; // スレーブ1のID
const int8_t SLAVE_2 = 0x002; // スレーブ2のID

const int     TX_PIN                 = 5;
const int     RX_PIN                 = 4;
const double  MIN_CURRENT            = -20.0;
const double  MAX_CURRENT            = 20.0;
const int16_t MIN_SENDNUM            = -16384;
const int16_t MAX_SENDNUM            = 16384;
const double  MOTOR_PERSENTAGE       = 0.15;
const double  NOMAL_MOTOR_PERSENTAGE = 0.3;
const uint8_t DEADZONE_STICK         = 40;
const uint8_t DEADZONE_R2_L2         = 40;

const int   SERIAL_BAUDRATE = 115200;
const int   CAN_BAUDRATE    = 1000E3;
const char* PS4_BT_ADDRESS  = "48:e7:29:a3:b2:26";

const double INPUT_COUNT_PER_EXEC{0.01};
const double INPUT_COUNT_DEFAULT{0.25};

//-20 20 の電流値を -16384 16384にmap
int16_t format_send_data(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
    double proportion = (x - in_min) / (in_max - in_min);
    double out_base   = (double)(out_max - out_min);
    return out_min + out_base * proportion;
}
// 16bitのデータを、8bit 8bitにわける
std::pair<int8_t, int8_t> split_data(int16_t formatted_data) {
    int8_t first_data  = (formatted_data >> 8) & 0xFF;
    int8_t second_data = formatted_data & 0xFF;
    return {first_data, second_data};
};
class RoboMasMotor {
    private:
        int id;

    public:
        RoboMasMotor(int set_id) { id = set_id; }
        int                 Id() { return id; }
        std::pair<int, int> SendBufNum() {
            int first  = (id - 1) * 2;
            int second = (id - 1) * 2 + 1;
            return {first, second};
        }
        std::pair<int8_t, int8_t> SendBufByte(double speed_percentage) {
            double proportion = speed_percentage / 100.0;
            return split_data(format_send_data(20 * proportion, MIN_CURRENT, MAX_CURRENT, MIN_SENDNUM, MAX_SENDNUM));
        }
};
class Omnix4 {
    private:
        RoboMasMotor FrontLeftOmni        = RoboMasMotor(3);
        RoboMasMotor BackLeftOmni         = RoboMasMotor(1);
        RoboMasMotor BackRightOmni        = RoboMasMotor(2);
        RoboMasMotor FrontRightOmni       = RoboMasMotor(4);
        Packet       TxBuf                = Packet(0x200);
        const double MAX_CONTROLLER_INPUT = 127.0;
        void         MotorSpeedChange(RoboMasMotor motor, double speed_percentage) {
            TxBuf.At(motor.SendBufNum().first)  = motor.SendBufByte(speed_percentage).first;
            TxBuf.At(motor.SendBufNum().second) = motor.SendBufByte(speed_percentage).second;
        }

        double MotorPower(double motor_power, bool PS4_circle = PS4.Circle()) {
            return motor_power * (PS4_circle ? 0.15 : 1.0);
        }

    public:
        Omnix4() {}
        void SendPacket() { TxBuf.Send(); }
        void Shift(int x, int y, double max_speed_percentage, double count) {
            double distance = std::sqrt(x * x + y * y); // 倒し具合
            if (distance == 0) {
                MotorSpeedChange(FrontLeftOmni, 0);
                MotorSpeedChange(BackLeftOmni, 0);
                MotorSpeedChange(BackRightOmni, 0);
                MotorSpeedChange(FrontRightOmni, 0);
                return;
            }

            // 正規化した入力ベクトル
            double nx = x / distance;
            double ny = y / distance;

            // 倒し具合を 0〜1 にスケーリング（必要なら最大値を決める）
            double max_input = 127.0; // PS4のスティック最大
            if (distance > max_input) distance = max_input;
            double magnitude = distance / max_input;

            // 角度を求めて45度回転
            double radian = atan2(ny, nx);
            radian -= PI / 4;
            
            // countに応じて補正
            max_speed_percentage *= count;

            // 倒し具合を掛けて速度を決定
            double vector13 = std::cos(radian) * max_speed_percentage * magnitude;
            double vector24 = std::sin(radian) * max_speed_percentage * magnitude;

            // Serial.println(vector13);
            // Serial.println(vector24);

            MotorSpeedChange(FrontLeftOmni, vector13);
            MotorSpeedChange(BackLeftOmni, vector24);
            MotorSpeedChange(BackRightOmni, -vector13);
            MotorSpeedChange(FrontRightOmni, -vector24);
        }

        void R_Turn(u_int8_t R2_val, double speed_percentage) {
            double R2_persentage = double(R2_val) / 255.0 * speed_percentage;

            MotorSpeedChange(FrontLeftOmni, R2_persentage);
            MotorSpeedChange(BackLeftOmni, R2_persentage);
            MotorSpeedChange(BackRightOmni, R2_persentage);
            MotorSpeedChange(FrontRightOmni, R2_persentage);
        }
        void L_Turn(uint8_t L2_val, double speed_percentage) {
            double L2_persetage = double(L2_val) / 255.0 * speed_percentage;
            // Serial.println(L2_persetage);

            MotorSpeedChange(FrontLeftOmni, -L2_persetage);
            MotorSpeedChange(BackLeftOmni, -L2_persetage);
            MotorSpeedChange(BackRightOmni, -L2_persetage);
            MotorSpeedChange(FrontRightOmni, -L2_persetage);
        }
        void Stop() {
            MotorSpeedChange(FrontLeftOmni, 0);
            MotorSpeedChange(BackLeftOmni, 0);
            MotorSpeedChange(BackRightOmni, 0);
            MotorSpeedChange(FrontRightOmni, 0);
        }
        void TestMove(double x) {
            MotorSpeedChange(FrontLeftOmni, x);
            MotorSpeedChange(BackRightOmni, 0 - x);
        }
};

Omnix4 TestOmni = Omnix4();

// デッドゾーン処理（–128…127 の範囲で扱う）
int8_t DeadZone_int8_t(int16_t value, int ZONE) {
    return (abs(value) < ZONE) ? 0 : value;
}
uint8_t DeadZone_uint8_t(u_int8_t value, uint8_t ZONE) {
    return (abs(value) < ZONE) ? 0 : value;
}

// ボタン８つを 1 バイトにビットパック
uint8_t packButtons(bool circle, bool triangle, bool square, bool cross, bool L1, bool R1, bool left, bool right) {
    return (circle ? (1 << 0) : 0) | (triangle ? (1 << 1) : 0) | (square ? (1 << 2) : 0) | (cross ? (1 << 3) : 0) |
           (L1 ? (1 << 4) : 0) | (R1 ? (1 << 5) : 0) | (left ? (1 << 6) : 0) | (right ? (1 << 7) : 0);
}

int16_t unit_data(int8_t upper_data, int8_t lower_data) {
    return upper_data << 8 | lower_data;
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    PS4.begin(PS4_BT_ADDRESS);
    CAN.setPins(RX_PIN, TX_PIN);
    if (!CAN.begin(CAN_BAUDRATE)) {
        Serial.println("CAN Init Failed");
        while (1)
            ;
    }
    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;
    // Serial.println("Ready");
}

double input_count{0.0};
void check_and_count(double& count, double change) {
    count += change;

    if(count > 1.0) count = 1.0;
    if(count < 0.0) count = 0.0;
}

void loop() {

    int packetSize = CAN.parsePacket();
    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);
    while (CAN.available()) {
        Serial.println(CAN.read());
    }

    if (!PS4.isConnected()) {
        TestOmni.Stop();
        return;
    }
    // 1) スティック値取得（–128…127）
    int8_t  l_x    = DeadZone_int8_t(PS4.LStickX(), DEADZONE_STICK);
    int8_t  l_y    = DeadZone_int8_t(PS4.LStickY(), DEADZONE_STICK);
    int8_t  r_x    = DeadZone_int8_t(PS4.RStickX(), DEADZONE_STICK);
    int8_t  r_y    = DeadZone_int8_t(PS4.RStickY(), DEADZONE_STICK);
    uint8_t R2_val = DeadZone_uint8_t(PS4.R2Value(), DEADZONE_R2_L2);
    uint8_t L2_val = DeadZone_uint8_t(PS4.L2Value(), DEADZONE_R2_L2);

    // 2) ボタンをビットパック
    uint8_t btns =
        packButtons(PS4.Circle(), PS4.Triangle(), PS4.Square(), PS4.Cross(), PS4.L1(), PS4.R1(), PS4.Left(), PS4.Right());

    // 3) CANフレーム送信
    CAN.beginPacket(SLAVE_1);

    CAN.write(btns); // Byte0: ボタン８つ
    CAN.write(r_x);  // Byte3: RStick X
    CAN.write(r_y);  // Byte4: RStick Y

    CAN.endPacket();
    CAN.beginPacket(SLAVE_2);

    CAN.write(btns); // Byte0: ボタン８つ
    CAN.write(r_x);  // Byte3: RStick X
    CAN.write(r_y);  // Byte4: RStick Y

    CAN.endPacket();

    if (R2_val > 0 && L2_val > 0) {
        TestOmni.Stop();
        input_count = INPUT_COUNT_DEFAULT;
    } else if (R2_val > 0) {
        TestOmni.R_Turn(R2_val, 30.0);
        check_and_count(input_count, INPUT_COUNT_PER_EXEC);
        // Serial.println("R_turn");
    } else if (L2_val > 0) {
        TestOmni.L_Turn(L2_val, 30.0);
        check_and_count(input_count, INPUT_COUNT_PER_EXEC);
        // Serial.println("L_turn");
    } else if (l_x != INPUT_COUNT_DEFAULT || l_y != 0) {
        TestOmni.Shift(l_x, l_y, 30.0, input_count);
        check_and_count(input_count, INPUT_COUNT_PER_EXEC);
    } else {
        TestOmni.Stop();
        input_count = INPUT_COUNT_DEFAULT;
    }

    TestOmni.SendPacket();
    // Serial.println(l_x);
    // Serial.println(l_y);
    // Serial.println(r_x);
    // Serial.println(r_y);
    // Serial.println(L2_val);
    // Serial.println(btns);

    // Serial.println("Sent button+stick");
    delay(1);
}
