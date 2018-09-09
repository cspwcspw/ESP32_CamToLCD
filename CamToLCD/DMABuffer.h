#pragma once

// Pete Wentworth cspwcspw@gmail.com.  Sept 2018
// Released under Apache License 2.0 
// Big chunks of the camera-side code were lifted from or inspired by
// https://github.com/bitluni/ESP32CameraI2S by Bitluni, and 
// https://github.com/igrr/esp32-cam-demo by Ivan Grokhotkov (igrr)

#include "rom/lldesc.h"

class DMABuffer
{
  public:
  lldesc_t descriptor;
  unsigned char* buffer;
  DMABuffer(int bytes)
  {
    buffer = (unsigned char *)malloc(bytes);
    descriptor.length = bytes;
    descriptor.size = descriptor.length;
    descriptor.owner = 1;
    descriptor.sosf = 1;
    descriptor.buf = (uint8_t*) buffer;
    descriptor.offset = 0;
    descriptor.empty = 0;
    descriptor.eof = 1;
    descriptor.qe.stqe_next = 0;
  }

  void next(DMABuffer *next)
  {
    descriptor.qe.stqe_next = &(next->descriptor);
  }

  int sampleCount() const
  {
    return descriptor.length / 4;
  }

  ~DMABuffer()
  {
    if(buffer)
      delete(buffer);
  }
};

