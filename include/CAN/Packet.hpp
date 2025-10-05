#ifndef CANPACKET_SEEN
#define CANPACKET_SEEN

#include <array>

struct CANPacket {
    CANPacket(int set_id);
    int id;
    std::array<uint8_t, 8> buf;
    virtual void Clear();
};

CANPacket::CANPacket(int set_id) : id(set_id) { Clear(); }

void CANPacket::Clear() { this->buf = {0, 0, 0, 0, 0, 0, 0, 0}; }

#endif