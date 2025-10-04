#ifndef CANPACKET_SEEN
#define CANPACKET_SEEN

#include <CAN.h>

class CANPacket {
    public:
        CANPacket(int packet_id);
        virtual void BufInit();
    protected:
        int m_packet_id;
        uint8_t m_buf[8];
};

CANPacket::CANPacket(int packet_id) {
    m_packet_id = packet_id;
    BufInit();
}

void CANPacket::BufInit() {
    for(uint8_t& b : m_buf) b = 0x00;
}

#endif