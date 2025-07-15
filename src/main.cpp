#include <CAN.h>

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("CAN Sender");

    // start the CAN bus at 500 kbps
    if (!CAN.begin(1000E3)) {
        Serial.println("CANの開始に失敗しました");
        while (1)
            ;
    }

    volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;
}

void loop() {
    // send packet: id is 11 bits, packet can contain up to 8 bytes of data
    Serial.print("Sending packet ... ");

    CAN.beginPacket(0x001);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);
    CAN.write(1);

    CAN.endPacket();

    Serial.println("done");

    delay(1000);
}
