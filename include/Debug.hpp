#ifndef DEBUG_SEEN
#define DEBUG_SEEN

#include <Arduino.h>

extern bool DEBUG_FLAG;

void DebugBegin(int serial_baudrate) {
    if(DEBUG_FLAG) Serial.begin(serial_baudrate);
}

void DebugPrintln(char* debug_msg) {
    if(DEBUG_FLAG) Serial.println(debug_msg);
}

template<typename debug_num_type>
void DebugPrintlnNum(char* debug_msg, debug_num_type debug_num) {
    if(DEBUG_FLAG) {
        Serial.print(debug_msg);
        Serial.println(debug_num);
    }
}

#endif