#ifndef DEBUG_SEEN
#define DEBUG_SEEN

#include <Arduino.h>

extern bool g_debug_enabled;

void DebugBegin(int serial_baudrate) {
    if(g_debug_enabled) Serial.begin(serial_baudrate);
}

void DebugPrintln(char* debug_msg) {
    if(g_debug_enabled) Serial.println(debug_msg);
}

template<typename debug_num_type>
void DebugPrintlnNum(char* debug_msg, debug_num_type debug_num) {
    if(g_debug_enabled) {
        Serial.print(debug_msg);
        Serial.println(debug_num);
    }
}

#endif