#ifndef PACKET_SEEN
#define PACKET_SEEN

#include <CAN.h>

class Packet {
    public:
        // idとデータを入れる関数
        Packet(int set_id) {
            id = set_id;
            Init();
        }
        void Init() {
            for (int8_t& data : buf)
                data = 0x00;
        }
        // 参照している　このように返すと、呼び出し側で
        // packet.At(2) = someValue; のように書いたときに、実際に buf[2] に直接値を書き込むことができる。
        int8_t& At(int num) { return buf[num]; }
        int     Id() { return id; }
        void    Send() {
            CAN.beginPacket(id);
            for (int8_t data : buf)
                CAN.write(data);
            CAN.endPacket();
            delay(1);
        }
    private:
        // id：CAN通信で使う送信IDを保持する整数。
        int id;
        // buf[8]：8バイトのデータバッファ。CANパケットのデータをここに格納します。
        int8_t buf[8];

};

#endif