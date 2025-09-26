/*
 * ESP32 CAN通信制御プログラム - オムニホイール4輪ロボット制御
 *
 * 機能概要:
 * - PS4コントローラーのアナログスティック入力を受け取り
 * - アナログ値をモーター制御用のデジタル値に変換
 * - CAN通信でモータードライバーに制御信号を送信
 * - 4輪オムニホイールによる全方向移動制御を実現
 */

#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <cmath>

// =============================================================================
// 定数定義 - システム全体の設定値
// =============================================================================

// CAN通信でのデバイスID定義
const int8_t SLAVE_1 = 0x001; // スレーブ1のID（メインデータ送信用）
const int8_t SLAVE_2 = 0x002; // スレーブ2のID（予備）
const int8_t SLAVE_3 = 0x003; // スレーブ3のID（予備）

// CAN通信ピン設定
const int TX_PIN = 5; // CAN送信ピン（ESP32のGPIO5）
const int RX_PIN = 4; // CAN受信ピン（ESP32のGPIO4）

// アナログ値からモーター制御値への変換定数
const double  MIN_CURRENT = -20.0;  // モーターの最小電流値（A）- 逆転最大
const double  MAX_CURRENT = 20.0;   // モーターの最大電流値（A）- 正転最大
const int16_t MIN_SENDNUM = -16384; // CAN送信データの最小値（16bit符号付き）
const int16_t MAX_SENDNUM = 16384;  // CAN送信データの最大値（16bit符号付き）

// アナログ入力のデッドゾーン設定（微細な揺れを無視するための閾値）
// テスト用に小さく設定 - アナログ制御がわかりやすくなる
const uint8_t DEADZONE_STICK = 40; // スティックのデッドゾーン（テスト用に縮小）
const uint8_t DEADZONE_R2_L2 = 20; // R2/L2トリガーのデッドゾーン（テスト用に縮小）

// =============================================================================
// アナログ値処理関数群
// =============================================================================

/**
 * アナログ値のリニアマッピング関数
 *
 * 目的: PS4コントローラーのアナログ値を物理的な電流値に変換し、
 *      さらにCAN通信用のデジタル値にマッピングする
 *
 * @param x アナログ入力値（例：-20.0～20.0の電流値）
 * @param in_min 入力範囲の最小値（MIN_CURRENT = -20.0）
 * @param in_max 入力範囲の最大値（MAX_CURRENT = 20.0）
 * @param out_min 出力範囲の最小値（MIN_SENDNUM = -16384）
 * @param out_max 出力範囲の最大値（MAX_SENDNUM = 16384）
 * @return マッピングされた16bit整数値
 *
 * 計算式の詳細:
 * 1. proportion = (x - in_min) / (in_max - in_min) → 入力を0～1の比率に正規化
 * 2. out_base = (out_max - out_min) → 出力範囲の幅を計算
 * 3. result = out_min + out_base * proportion → 比率に基づいて出力値を算出
 */
int16_t mapping_data(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
    // 入力値を0～1の比率に正規化
    double proportion = (x - in_min) / (in_max - in_min);

    // 出力範囲の幅を計算
    double out_base = (double)(out_max - out_min);

    // 正規化された比率を出力範囲にスケーリング
    return out_min + out_base * proportion;
}

/**
 * 16bitデータの分割関数 - CAN通信用バイト分割
 *
 * 目的: 16bit（2バイト）のモーター制御値を、CAN通信で送信するために
 *      8bit（1バイト）×2に分割する
 *
 * @param formatted_data 16bit整数データ（-16384～16384）
 * @return std::pair<int8_t, int8_t> {上位バイト, 下位バイト}
 *
 * ビット操作の詳細:
 * - 上位バイト: (data >> 8) & 0xFF → 右に8bit シフトして上位8bitを取得
 * - 下位バイト: data & 0xFF → 下位8bitのマスクを適用して下位8bitを取得
 *
 * 例: 0x1234 (4660) → first_data = 0x12 (18), second_data = 0x34 (52)
 */
std::pair<int8_t, int8_t> split_data(int16_t formatted_data) {
    // 16bitデータを上位8bit（first）と下位8bit（second）に分割
    int8_t first_data  = (formatted_data >> 8) & 0xFF; // 上位バイト取得
    int8_t second_data = formatted_data & 0xFF;        // 下位バイト取得
    return {first_data, second_data};
}
// =============================================================================
// CAN通信パケット管理クラス
// =============================================================================

/**
 * Packetクラス - CAN通信用データパケットの管理
 *
 * 目的: CAN通信で送信するデータパケットを効率的に管理する
 *      アナログ制御データをCAN規格に準拠した形式で送信
 *
 * CAN通信の基本仕様:
 * - 1パケット最大8バイトのデータを送信可能
 * - 各パケットには一意のID（識別子）が必要
 * - バイト単位でのデータ送信が基本
 */
class Packet {
    private:
        // CAN通信用パケットID（送信先を識別する）
        int id;

        // 8バイトデータバッファ（CANパケットの最大サイズ）
        // アナログ制御では以下のように使用:
        // buf[0-1]: モーター1の16bit制御値（上位・下位バイト）
        // buf[2-3]: モーター2の16bit制御値（上位・下位バイト）
        // buf[4-5]: モーター3の16bit制御値（上位・下位バイト）
        // buf[6-7]: モーター4の16bit制御値（上位・下位バイト）
        int8_t buf[8];

    public:
        /**
         * コンストラクタ - パケットIDを設定し初期化
         * @param set_id CAN通信用のパケットID（例：0x200）
         */
        Packet(int set_id) {
            id = set_id;
            Init(); // バッファを0クリア
        }

        /**
         * データバッファの初期化
         * 全8バイトを0x00でクリアし、安全な初期状態にする
         */
        void Init() {
            for (int8_t& data : buf)
                data = 0x00;
        }

        /**
         * バッファへの直接アクセス（参照返し）
         *
         * 参照を返すことで、呼び出し側で以下のような直接書き込みが可能:
         * packet.At(2) = motor_high_byte;
         * packet.At(3) = motor_low_byte;
         *
         * @param num バッファのインデックス（0-7）
         * @return バッファ要素への参照
         */
        int8_t& At(int num) { return buf[num]; }

        /**
         * パケットIDの取得
         * @return 設定されているCAN ID
         */
        int Id() { return id; }

        /**
         * CAN通信でのパケット送信
         *
         * 送信プロセス:
         * 1. CAN.beginPacket(id) - パケット送信開始、IDを設定
         * 2. CAN.write(data) - 8バイトのデータを順次送信
         * 3. CAN.endPacket() - パケット送信完了
         * 4. delay(10) - 次の送信まで10ms待機（通信安定性確保）
         */
        void Send() {
            CAN.beginPacket(id);    // パケット送信開始
            for (int8_t data : buf) // 8バイト全てを送信
                CAN.write(data);
            CAN.endPacket(); // パケット送信完了
            delay(10);       // 安定性のための待機時間
        }
};
// =============================================================================
// モーター制御クラス - アナログ制御の中核
// =============================================================================

/**
 * RoboMasMotorクラス - 個別モーターのアナログ制御管理
 *
 * 目的: 1つのモーターに対するアナログ制御を管理
 *      スピード比率（%）から実際の電流値、そしてCAN送信データへの変換を担当
 *
 * アナログ制御フロー:
 * 1. PS4スティック入力（-127～127） → スピード比率（-100%～100%）
 * 2. スピード比率 → 物理的電流値（-20A～20A）
 * 3. 電流値 → CAN送信用16bit整数（-16384～16384）
 * 4. 16bit整数 → 8bit×2バイトに分割してCAN送信
 */
class RoboMasMotor {
    private:
        // モーターの識別ID（1-4の範囲で各モーターを識別）
        int id;

    public:
        /**
         * コンストラクタ - モーターIDを設定
         * @param set_id モーターの識別番号（1:左後, 2:右後, 3:左前, 4:右前）
         */
        RoboMasMotor(int set_id) { id = set_id; }

        /**
         * モーターIDの取得
         * @return 設定されているモーターID
         */
        int Id() { return id; }

        /**
         * CANパケット内でのデータ位置計算
         *
         * 各モーターは16bit（2バイト）のデータを使用するため、
         * CANパケット8バイト内での配置を計算する
         *
         * 配置ルール:
         * - モーター1: バイト0-1 (first=0, second=1)
         * - モーター2: バイト2-3 (first=2, second=3)
         * - モーター3: バイト4-5 (first=4, second=5)
         * - モーター4: バイト6-7 (first=6, second=7)
         *
         * @return std::pair<int, int> {上位バイト位置, 下位バイト位置}
         */
        std::pair<int, int> ID_DATE() {
            int first  = (id - 1) * 2;     // 上位バイトの位置
            int second = (id - 1) * 2 + 1; // 下位バイトの位置
            return {first, second};
        }

        /**
         * アナログスピード値からCAN送信バイトへの変換
         *
         * 変換プロセス:
         * 1. スピード比率(%)を0.0-1.0の比率に正規化
         * 2. 最大電流20Aを乗算して物理的電流値を算出
         * 3. 電流値(-20A～20A)をCAN送信値(-16384～16384)にマッピング
         * 4. 16bit整数を上位・下位バイトに分割
         *
         * @param speed_percentage スピード比率（-100.0～100.0%）
         * @return std::pair<int8_t, int8_t> {上位バイト, 下位バイト}
         *
         * 計算例:
         * - 入力: 50% → 0.5 → 10A → 8192 → {0x20, 0x00}
         * - 入力: -25% → -0.25 → -5A → -4096 → {0xF0, 0x00}
         */
        std::pair<int8_t, int8_t> SendBufByte(double speed_percentage) {
            // スピード比率を0.0-1.0の範囲に正規化
            double proportion = speed_percentage / 100.0;

            // 比率に最大電流を乗算して実際の電流値を算出し、
            // mapping_data()でCAN送信用の16bit整数に変換後、
            // split_data()で上位・下位バイトに分割
            return split_data(mapping_data(20 * proportion, MIN_CURRENT, MAX_CURRENT, MIN_SENDNUM, MAX_SENDNUM));
        }
};
// =============================================================================
// オムニホイール4輪制御クラス - アナログ制御の統合システム
// =============================================================================

/**
 * Omnix4クラス - 4輪オムニホイールロボットのアナログ制御統合管理
 *
 * オムニホイール配置（上から見た図）:
 *   前進方向 ↑
 *   [3]←---→[4]   FrontLeft(ID:3)  FrontRight(ID:4)
 *    |       |
 *    |   ●   |    ● = ロボット中心
 *    |       |
 *   [1]←---→[2]   BackLeft(ID:1)   BackRight(ID:2)
 *
 * アナログ制御の核心原理:
 * 1. PS4スティックの2軸入力(X,Y)から移動ベクトルを算出
 * 2. ベクトルを45度回転させてオムニホイールの配置に適合
 * 3. 各ホイールに適切な回転方向・速度を分配
 * 4. アナログ入力強度に応じてスムーズな速度制御を実現
 */
class Omnix4 {
    private:
        // 4つのオムニホイールモーター（物理配置順にID設定）
        RoboMasMotor FrontLeftOmni  = RoboMasMotor(3); // 前左モーター(ID:3)
        RoboMasMotor BackLeftOmni   = RoboMasMotor(1); // 後左モーター(ID:1)
        RoboMasMotor BackRightOmni  = RoboMasMotor(2); // 後右モーター(ID:2)
        RoboMasMotor FrontRightOmni = RoboMasMotor(4); // 前右モーター(ID:4)

        // CAN通信用パケット（4モーター分のデータを8バイトで送信）
        Packet TxBuf = Packet(0x200);

        // PS4コントローラーアナログスティックの最大入力値
        const double MAX_CONTROLLER_INPUT = 127.0;

        /**
         * 個別モーターの速度変更（内部関数）
         *
         * @param motor 制御対象のモーターオブジェクト
         * @param speed_percentage スピード比率（-100～100%）
         *
         * 処理流れ:
         * 1. モーターのSendBufByte()でアナログ値をCAN用バイトデータに変換
         * 2. モーターのID_DATE()でCANパケット内の位置を取得
         * 3. TxBuf（送信バッファ）の適切な位置にデータを格納
         */
        void MotorSpeedChange(RoboMasMotor motor, int speed_percentage) {
            // アナログスピード値をCAN送信用バイトデータに変換
            auto byte_data = motor.SendBufByte(speed_percentage);
            auto position  = motor.ID_DATE();

            // CANパケットの指定位置にバイトデータを格納
            TxBuf.At(position.first)  = byte_data.first;  // 上位バイト
            TxBuf.At(position.second) = byte_data.second; // 下位バイト
        }

    public:
        /**
         * デフォルトコンストラクタ
         */
        Omnix4() {}

        /**
         * CAN通信でのパケット送信実行
         *
         * 内部のTxBufに格納された4モーター分のアナログ制御データを
         * CAN通信で送信する
         */
        void SendPacket() { TxBuf.Send(); }
        /**
         * アナログスティック入力による全方向移動制御（メイン制御関数）
         *
         * オムニホイール制御のアナログ制御アルゴリズム:
         *
         * 【ステップ1: 入力解析】
         * PS4スティックの2軸入力(x,y)からベクトル長と方向を算出
         *
         * 【ステップ2: デッドゾーン処理】
         * 微細な入力（中央付近）では全モーターを停止
         *
         * 【ステップ3: ベクトル正規化】
         * 入力ベクトルを単位ベクトルに正規化して方向を純粋に抽出
         *
         * 【ステップ4: 強度計算】
         * スティックの傾き具合(0～1)を速度の強度として使用
         *
         * 【ステップ5: オムニホイール座標系変換】
         * 45度回転によりX-Y座標系をオムニホイールの対角配置に適合
         *
         * 【ステップ6: モーター速度配分】
         * 対角ペアに同じベクトル成分を、隣接ペアに逆ベクトル成分を配分
         *
         * @param x PS4左スティックX値（-127～127）
         * @param y PS4左スティックY値（-127～127）
         * @param max_speed_percentage 最大速度制限（0～100%）
         */
        void Shift(int x, int y, double max_speed_percentage) {
            // ===============================================
            // ステップ1: ベクトル長（スティックの傾き強度）計算
            // ===============================================
            double distance = std::sqrt(x * x + y * y); // ピタゴラスの定理でベクトル長を算出

            // ===============================================
            // ステップ2: デッドゾーン処理（中立位置での停止）
            // ===============================================
            if (distance == 0) {
                // スティックが中央位置の場合、全モーターを停止
                MotorSpeedChange(FrontLeftOmni, 0);
                MotorSpeedChange(BackLeftOmni, 0);
                MotorSpeedChange(BackRightOmni, 0);
                MotorSpeedChange(FrontRightOmni, 0);
                return;
            }

            // ===============================================
            // ステップ3: 方向ベクトルの正規化
            // ===============================================
            // 入力ベクトルを単位ベクトルに正規化（方向のみを抽出）
            double nx = x / distance; // X方向成分（-1.0～1.0）
            double ny = y / distance; // Y方向成分（-1.0～1.0）

            // ===============================================
            // ステップ4: アナログ強度の計算
            // ===============================================
            // スティックの傾き具合を0～1の範囲にスケーリング
            double max_input = 127.0;                       // PS4スティックの最大入力値
            if (distance > max_input) distance = max_input; // 上限制限
            double magnitude = distance / max_input;        // 0.0～1.0の強度

            // ===============================================
            // ステップ5: オムニホイール座標系への変換
            // ===============================================
            // 入力角度を算出（atan2で全象限対応）
            double radian = atan2(ny, nx);

            // オムニホイール配置に合わせて45度（π/4ラジアン）回転
            // これによりX-Y軸がオムニホイールの対角線方向に整列
            radian -= PI / 4;

            // ===============================================
            // ステップ6: 各モーターへの速度配分計算
            // ===============================================
            // 回転後の角度からcos/sin成分を計算し、強度と最大速度を適用
            double vector13 = std::cos(radian) * max_speed_percentage * magnitude; // 対角ベクトル1-3
            double vector24 = std::sin(radian) * max_speed_percentage * magnitude; // 対角ベクトル2-4

            // アナログ制御値の詳細デバッグ出力
            Serial.print("MOVE Vector13=");
            Serial.print(vector13);
            Serial.print("% Vector24=");
            Serial.print(vector24);
            Serial.println("%");
            Serial.print("Motors: FL=");
            Serial.print(vector13);
            Serial.print(" BL=");
            Serial.print(vector24);
            Serial.print(" BR=");
            Serial.print(-vector13);
            Serial.print(" FR=");
            Serial.println(-vector24);

            // ===============================================
            // ステップ7: 各モーターへの速度設定
            // ===============================================
            // オムニホイールの物理配置に基づく速度配分:
            // - 対角ペア（1-3, 2-4）は同じベクトル成分を使用
            // - 隣接ペア（前後、左右）は逆方向で相殺効果を生成
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
        /**
         * 右回転制御（アナログトリガー制御）
         *
         * PS4コントローラーのR2トリガーによるアナログ回転制御
         *
         * 回転原理:
         * 全てのモーターを同じ方向（時計回り）に回転させることで、
         * ロボット全体を右方向に回転させる
         *
         * アナログ制御の特徴:
         * - R2トリガーの押し込み具合（0～255）に応じて回転速度を調整
         * - 軽く押せば低速回転、深く押せば高速回転
         * - リニアな速度制御により滑らかな回転動作を実現
         *
         * @param R2_val R2トリガーの押し込み値（0～255、アナログ入力）
         * @param speed_percentage 最大回転速度の制限（0～100%）
         */
        void R_Turn(u_int8_t R2_val, double speed_percentage) {
            // R2トリガー値を0.0～1.0の比率に正規化
            double R2_persentage = R2_val / 255.0;

            // 全モーターを同じ正方向に回転（右回転効果）
            // 速度 = 最大速度 × R2トリガー強度
            MotorSpeedChange(FrontLeftOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(BackLeftOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(BackRightOmni, speed_percentage * R2_persentage);
            MotorSpeedChange(FrontRightOmni, speed_percentage * R2_persentage);
        }

        /**
         * 左回転制御（アナログトリガー制御）
         *
         * PS4コントローラーのL2トリガーによるアナログ回転制御
         *
         * 回転原理:
         * 全てのモーターを同じ方向（反時計回り）に回転させることで、
         * ロボット全体を左方向に回転させる
         *
         * アナログ制御の特徴:
         * - L2トリガーの押し込み具合（0～255）に応じて回転速度を調整
         * - R_Turn()と逆方向の回転を生成
         * - 同様にリニアな速度制御による滑らかな動作
         *
         * @param L2_val L2トリガーの押し込み値（0～255、アナログ入力）
         * @param speed_percentage 最大回転速度の制限（0～100%）
         */
        void L_Turn(uint8_t L2_val, double speed_percentage) {
            // L2トリガー値を0.0～1.0の比率に正規化
            double L2_persetage = L2_val / 255.0;

            // 全モーターを同じ負方向に回転（左回転効果）
            // 速度 = 最大速度 × L2トリガー強度 × (-1)
            MotorSpeedChange(FrontLeftOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(BackLeftOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(BackRightOmni, -speed_percentage * L2_persetage);
            MotorSpeedChange(FrontRightOmni, -speed_percentage * L2_persetage);
        }

        /**
         * 全モーター停止
         *
         * 緊急停止やコントローラー未接続時の安全措置として使用
         * 全4つのモーターに速度0を設定してロボットを完全停止
         */
        void Stop() {
            MotorSpeedChange(FrontLeftOmni, 0);
            MotorSpeedChange(BackLeftOmni, 0);
            MotorSpeedChange(BackRightOmni, 0);
            MotorSpeedChange(FrontRightOmni, 0);
        }

        /**
         * テスト用動作関数
         *
         * 開発・デバッグ用の簡単な動作テスト
         * 対角モーター（前左と後右）を逆方向に回転させる
         *
         * @param x テスト用速度値
         */
        void TestMove(double x) {
            MotorSpeedChange(FrontLeftOmni, x);     // 前左モーター
            MotorSpeedChange(BackRightOmni, 0 - x); // 後右モーター（逆方向）
        }
};

// =============================================================================
// グローバルオブジェクト インスタンス
// =============================================================================

/**
 * オムニホイール制御システムのメインインスタンス
 *
 * このオブジェクトを通じて全てのアナログ制御が実行される
 */
Omnix4 TestOmni = Omnix4();

// =============================================================================
// アナログ入力処理関数群 - デッドゾーンとデータパッキング
// =============================================================================

/**
 * デッドゾーン処理関数 - アナログ制御の精度向上
 *
 * 目的: アナログ入力の微細なノイズや揺れを除去し、
 *      意図しない動作を防ぐための重要な処理
 *
 * デッドゾーンとは:
 * - アナログスティックやトリガーの中央付近の不感帯
 * - コントローラーの物理的な特性により、完全に0にならない微細な入力が発生
 * - これらの微細な入力を0として扱うことで、意図しない動作を防ぐ
 *
 * 処理対象:
 * - PS4スティック: -127～127の範囲（DEADZONE_STICK = 40）
 * - PS4トリガー: 0～255の範囲（DEADZONE_R2_L2 = 40）
 *
 * @param value アナログ入力値（-128～127 or 0～255）
 * @param ZONE デッドゾーンの閾値（この値以下は0として扱う）
 * @return デッドゾーン処理後の値（閾値以下なら0、それ以外は元の値）
 *
 * 動作例:
 * - DeadZone(35, 40) → 0 （35 < 40 なので0）
 * - DeadZone(50, 40) → 50 （50 > 40 なので50のまま）
 * - DeadZone(-25, 40) → 0 （|-25| = 25 < 40 なので0）
 */
int8_t DeadZone(int16_t value, int ZONE) {
    // 絶対値が閾値未満なら0、そうでなければ元の値を返す
    return (abs(value) < ZONE) ? 0 : value;
}

/**
 * ボタン状態のビットパッキング関数
 *
 * 目的: 8つのボタン状態を1バイト（8bit）に効率的に格納
 *      CAN通信でのデータ量削減とパケット効率化
 *
 * ビット配置:
 * bit0(LSB): Circle    bit4: L1
 * bit1:      Triangle  bit5: L2
 * bit2:      Square    bit6: R1
 * bit3:      Cross     bit7(MSB): R2
 *
 * @param circle   ○ボタンの状態（true/false）
 * @param triangle △ボタンの状態（true/false）
 * @param square   □ボタンの状態（true/false）
 * @param cross    ×ボタンの状態（true/false）
 * @param L1       L1ボタンの状態（true/false）
 * @param L2       L2ボタンの状態（true/false）
 * @param R1       R1ボタンの状態（true/false）
 * @param R2       R2ボタンの状態（true/false）
 * @return 8つのボタン状態を格納した1バイト値
 *
 * 例: Circle=true, Triangle=false, Square=true の場合
 * → 結果: 0b00000101 = 0x05
 */
uint8_t packButtons(bool circle, bool triangle, bool square, bool cross, bool L1, bool L2, bool R1, bool R2) {
    // 各ボタンの状態を対応するビット位置にマッピング
    return (circle ? (1 << 0) : 0) |   // bit0: Circle
           (triangle ? (1 << 1) : 0) | // bit1: Triangle
           (square ? (1 << 2) : 0) |   // bit2: Square
           (cross ? (1 << 3) : 0) |    // bit3: Cross
           (L1 ? (1 << 4) : 0) |       // bit4: L1
           (L2 ? (1 << 5) : 0) |       // bit5: L2
           (R1 ? (1 << 6) : 0) |       // bit6: R1
           (R2 ? (1 << 7) : 0);        // bit7: R2
}

// =============================================================================
// システム初期化関数 - ハードウェアセットアップ
// =============================================================================

/**
 * setup関数 - ESP32システムとアナログ制御の初期化
 *
 * Arduino環境では、setup()は電源投入時に1度だけ実行される初期化関数
 *
 * 初期化項目:
 * 1. シリアル通信設定（デバッグ用）
 * 2. PS4コントローラー接続設定（Bluetooth）
 * 3. CAN通信システム初期化
 * 4. ハードウェア割り込み設定
 */
void setup() {
    // ===============================================
    // 1. シリアル通信初期化（デバッグ用）
    // ===============================================
    Serial.begin(115200); // 115200bpsでシリアル通信開始
    // while (!Serial); // シリアル接続待機（必要に応じてコメントアウト）

    // ===============================================
    // 2. PS4コントローラーBluetooth接続設定
    // ===============================================
    // PS4コントローラーのMACアドレス設定
    // 複数の候補がコメントアウトされている（開発用の履歴）
    // PS4.begin("08:b6:1f:ed:44:32"); // コントローラー1
    // PS4.begin("48:e7:29:a3:c5:0e"); // コントローラー2
    // PS4.begin("9c:58:84:86:b6:28"); // コントローラー3
    // PS4.begin("e4:65:b8:7e:0c:2c"); // 現在使用中のコントローラー
    PS4.begin("e4:65:b8:7e:07:02");

    // ===============================================
    // 3. CAN通信システム初期化
    // ===============================================
    // CANピン設定（TX: GPIO5, RX: GPIO4）
    CAN.setPins(RX_PIN, TX_PIN);

    // CAN通信速度設定: 1Mbps (1000E3 = 1,000,000 bps)
    // この速度はモータードライバーと合わせる必要がある
    if (!CAN.begin(1000E3)) {
        Serial.println("CAN Init Failed");
        while (1)
            ; // CAN初期化失敗時は無限ループで停止
    }

    // ===============================================
    // 4. ESP32固有の割り込み設定（上級者向け）
    // ===============================================
    // ESP32のレジスタ直接操作によるCANコントローラー設定
    // 特定の割り込みを無効化して通信の安定性を向上
    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;

    // ===============================================
    // 初期化完了メッセージ
    // ===============================================
    Serial.println("Ready"); // システム準備完了の通知
}

// =============================================================================
// メイン制御ループ - アナログ制御の実行エンジン
// =============================================================================

/**
 * loop関数 - アナログ制御のメインループ
 *
 * Arduino環境では、setup()完了後にloop()が無限に繰り返し実行される
 * このシステムの心臓部であり、リアルタイムでアナログ制御を実行
 *
 * 制御フロー:
 * 1. PS4コントローラー接続確認
 * 2. アナログ入力の取得とデッドゾーン処理
 * 3. ボタン状態の取得とビットパック
 * 4. CAN通信でのセンサー・状態データ送信
 * 5. 動作モード判定と実行（回転 or 移動）
 * 6. モーター制御データのCAN送信
 * 7. デバッグ出力
 *
 * 実行頻度: 約100Hz（delay(10)により10ms周期）
 */
void loop() {
    // ===============================================
    // 1. コントローラー接続状態の確認と安全処理
    // ===============================================
    if (!PS4.isConnected()) {
        // コントローラー未接続時は安全のため全モーター停止
        TestOmni.Stop();
        return; // 以降の処理をスキップしてloop()を再開
    }

    // ===============================================
    // 2. アナログ入力の取得とデッドゾーン処理
    // ===============================================
    // PS4スティック入力取得（-127～127の範囲）
    // DeadZone()により微細な揺れを除去してアナログ制御の品質を向上
    int8_t  l_x = DeadZone(PS4.LStickX(), DEADZONE_STICK); // 左スティックX軸（移動用）
    int8_t  l_y = DeadZone(PS4.LStickY(), DEADZONE_STICK); // 左スティックY軸（移動用）
    uint8_t r_x = DeadZone(PS4.RStickX(), DEADZONE_STICK); // 右スティックX軸（未使用）
    uint8_t r_y = DeadZone(PS4.RStickY(), DEADZONE_STICK); // 右スティックY軸（未使用）

    // PS4トリガー入力取得（0～255の範囲）
    // アナログトリガーによる滑らかな回転制御
    // int R2_val = DeadZone(PS4.R2Value(), DEADZONE_R2_L2); // 右回転用トリガー
    // int L2_val = DeadZone(PS4.L2Value(), DEADZONE_R2_L2); // 左回転用トリガー
    int R2_val = PS4.R2Value(); // 右回転用トリガー
    int L2_val = PS4.L2Value(); // 左回転用トリガー

    // ===============================================
    // 3. ボタン状態の取得と効率的なデータパッキング
    // ===============================================
    // 8つのボタン状態を1バイトに圧縮してCAN通信効率化
    uint8_t btns_1 =
        packButtons(PS4.Circle(), PS4.Triangle(), PS4.Square(), PS4.Cross(), PS4.L1(), PS4.R1(), PS4.Left(), PS4.Right());

    // ===============================================
    // 4. センサーデータのCAN送信（SLAVE_1宛）
    // ===============================================
    // コントローラーの状態をCAN通信で外部デバイスに送信
    // 受信側システムでの状態監視や追加制御に使用
    CAN.beginPacket(SLAVE_1);

    // CANパケット構成（8バイト）:
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

    // CANパケット構成（8バイト）:
    CAN.write(btns_1); // Byte0: 8ボタン状態（ビットパック）
    CAN.write(r_x);    // Byte1: 右スティックX軸
    CAN.write(r_y);    // Byte2: 右スティックY軸
    CAN.write(1);      // Byte3: 予備データ
    CAN.write(1);      // Byte4: 予備データ
    CAN.write(1);      // Byte5: 予備データ
    CAN.write(1);      // Byte6: 予備データ
    CAN.write(1);      // Byte7: 予備データ

    CAN.endPacket(); // パケット送信完了

    // ===============================================
    // 5. 動作モードの判定とアナログ制御実行
    // ===============================================
    // トリガー入力が優先、スティック入力は二次的
    // アナログ値の強度に応じて滑らかな制御を実現

    if (R2_val > 0) {
        // 【右回転モード】R2トリガー押下時
        // R2の押し込み具合に応じて右回転速度を調整
        TestOmni.R_Turn(R2_val, 100.0); // 100%を最大回転速度として設定
        TestOmni.SendPacket();          // モーター制御データをCAN送信

    } else if (L2_val > 0) {
        // 【左回転モード】L2トリガー押下時
        // L2の押し込み具合に応じて左回転速度を調整
        TestOmni.L_Turn(L2_val, 100.0); // 100%を最大回転速度として設定
        TestOmni.SendPacket();
        Serial.println("L2"); // モーター制御データをCAN送信

    } else if (PS4.Up()) {
        TestOmni.Front();
        TestOmni.SendPacket();

    } else if (PS4.Down()) {
        TestOmni.Back();
        TestOmni.SendPacket();

    } else {
        // 【移動モード】トリガー未使用時はスティックで移動制御
        // 左スティックの2軸入力による全方向移動
        // アナログ入力強度に応じた滑らかな速度制御
        TestOmni.Shift(l_x, l_y, 100.0); // 100%を最大移動速度として設定
        TestOmni.SendPacket();           // モーター制御データをCAN送信
    }

    // ===============================================
    // 6. デバッグ出力（アナログ制御確認用）
    // ===============================================
    // アナログ値の確認 - スティックとトリガーの生値を表示
    Serial.print("LStick: X=");
    Serial.print(l_x);
    Serial.print(" Y=");
    Serial.println(l_y);
    Serial.print("Triggers: R2=");
    Serial.print(R2_val);
    Serial.print(" L2=");
    Serial.println(L2_val);
    Serial.print(PS4.R2Value());

    // 動作モードの表示
    // if (R2_val > 0) {
    Serial.print("RIGHT TURN - Intensity: ");
    Serial.print((R2_val / 255.0) * 100);
    Serial.println("%");
    // } else if (L2_val > 0) {
    Serial.print("LEFT TURN - Intensity: ");
    Serial.print((L2_val / 255.0) * 100);
    Serial.println("%");
    // } else if (l_x != 0 || l_y != 0) {
    double distance  = std::sqrt(l_x * l_x + l_y * l_y);
    double magnitude = distance / 127.0;
    Serial.print("MOVE - Distance: ");
    Serial.print(distance);
    Serial.print(" Intensity: ");
    Serial.print(magnitude * 100);
    Serial.println("%");
    // }
    Serial.println("---");

    // ===============================================
    // 7. ループ周期制御
    // ===============================================
    // Serial.println("Sent button+stick"); // 送信完了メッセージ
    delay(5);
}