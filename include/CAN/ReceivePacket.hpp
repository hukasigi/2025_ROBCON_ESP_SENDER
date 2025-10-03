#ifndef CANRECEIVEPACKET_SEEN
#define CANRECEIVEPACKET_SEEN

#include "Packet.hpp"

class CANReceivePacket : public CANPacket {
    public:
        using CANPacket::CANPacket;
        virtual uint8_t GetByte(int byte_id);
        virtual bool Receive();
};

uint8_t CANReceivePacket::GetByte(int byte_id) {
    return m_buf[byte_id];
}

bool CANReceivePacket::Receive() {
    if(CAN.packetId() == m_packet_id) return false;
    int read_count = 0;
    while (CAN.available()) {
        m_buf[read_count] = CAN.read();
        read_count++;
    }
}

#endif