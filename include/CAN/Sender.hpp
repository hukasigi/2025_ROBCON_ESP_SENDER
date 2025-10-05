#ifndef CANSENDPACKET_SEEN
#define CANSENDPACKET_SEEN

#include "Packet.hpp"

class CANSendPacket : public CANPacket {
    public:
        using CANPacket::CANPacket;
        virtual void SetByte(int byte_id, uint8_t set_byte);
        virtual void SetBuf(uint8_t set_buf[8]);
        virtual void Send();
};

void CANSendPacket::SetByte(int byte_id, uint8_t set_byte) {
    m_buf[byte_id] = set_byte;
}

void CANSendPacket::SetBuf(uint8_t set_buf[8]) {
    for(int i = 0; i < 8; i++) m_buf[i] = set_buf[i];
}

void CANSendPacket::Send() {
    CAN.beginPacket(m_packet_id);
    CAN.write(m_buf, 8);
    CAN.endPacket();
}

#endif