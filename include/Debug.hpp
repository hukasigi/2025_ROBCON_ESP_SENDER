#ifndef DEBUG_SEEN
#define DEBUG_SEEN

#include <Arduino.h>

extern const bool G_DEBUG_ENABLED;

void debug_begin(int serial_baudrate) {
    if(G_DEBUG_ENABLED) Serial.begin(serial_baudrate);
}

template<typename T>
void debug_print(T debug_msg) {
    if(G_DEBUG_ENABLED) Serial.print(debug_msg);
}

template<typename T>
void debug_println(T debug_msg) {
    if(G_DEBUG_ENABLED) Serial.println(debug_msg);
}

#endif