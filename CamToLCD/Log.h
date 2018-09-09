#pragma once

// Pete Wentworth cspwcspw@gmail.com.  Sept 2018
// Released under Apache License 2.0 

#include "Arduino.h"
#include<stdarg.h>

#ifdef DebuggingPort

class DebugPort
{   // This simple debugging output facility that can be dynamically
    // switched on or off as the program is running.
    // It sends output to Serial.
  private:
    bool enabled;
    HardwareSerial *serialDevice;

  public:
    // Set up to a debugger that sends stuff to this serial port
    DebugPort(HardwareSerial *device, bool enableOutput) {
      serialDevice = device;
      enabled = enableOutput;
    }
    
    void enableOutput(bool v) {
      enabled = v;
    }

    void print(int a) {
      if (enabled) serialDevice->print(a);
    }

    void println(int a) {
      if (enabled) serialDevice->println(a);
    }
    
    void print(const char* s) {
      if (enabled) serialDevice->print(s);
    }

    void println(const char* s) {
      if (enabled) serialDevice->println(s);
    }

    void println() {
      if (enabled) serialDevice->println();
    }

    void printf(char *format, ...) {
      if (!enabled) return;
      char buf[512];
      va_list arg;
      va_start (arg, format);
      vsprintf(buf, format, arg);
      va_end(arg);
      serialDevice->print(buf);
    }
};

// And a declaration that somewhere, somebody will create the object called debug
extern DebugPort debug;

#define debug_println(a) debug.println(a)
#define debug_print(a) debug.print(a)
#define debug_printf(fmt, ...) debug.printf(fmt, ##__VA_ARGS__)
#define debug_enableOutput(val)  debug.enableOutput(val)

#else
#define debug_println(a)
#define debug_print(a)
#define debug_printf(fmt, ...)
#define debug_enableOutput(enabled)
#endif

// Another way to debug is to directly toggle a GPIO pin and
// watch it with a LED, a logic analyzer, or a scope.

// #define debugPin  13
#ifdef debugPin
  #define debugSetup()    pinMode(debugPin, OUTPUT)
  #define debugHigh()     digitalWrite(debugPin, HIGH)
  #define debugLow()      digitalWrite(debugPin, LOW)
#endif




