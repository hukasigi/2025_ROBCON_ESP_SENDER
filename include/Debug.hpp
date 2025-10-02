#ifndef DEBUG_SEEN
#define DEBUG_SEEN

#include <Arduino.h>

extern bool g_debug_enabled;

template<typename T>
void DebugPrint(T debug_msg) {
    if(g_debug_enabled) Serial.print(debug_msg);
}

template<typename T>
void DebugPrintln(T debug_msg) {
    if(g_debug_enabled) Serial.println(debug_msg);
}

#endif