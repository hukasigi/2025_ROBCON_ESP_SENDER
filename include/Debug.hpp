#ifndef DEBUG_SEEN
#define DEBUG_SEEN

#include <Arduino.h>

extern bool g_debug_enabled;

void debug_begin(int serial_baudrate) {
    if(g_debug_enabled) Serial.begin(serial_baudrate);
}

template<typename T>
void debug_print(T debug_msg) {
    if(g_debug_enabled) Serial.print(debug_msg);
}

template<typename T>
void debug_println(T debug_msg) {
    if(g_debug_enabled) Serial.println(debug_msg);
}

#endif