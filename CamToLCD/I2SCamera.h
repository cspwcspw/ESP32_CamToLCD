
// Pete Wentworth cspwcspw@gmail.com.  Sept 2018
// Released under Apache License 2.0 
// Big chunks of the camera-side code were lifted from or inspired by
// https://github.com/bitluni/ESP32CameraI2S by Bitluni, and 
// https://github.com/igrr/esp32-cam-demo by Ivan Grokhotkov (igrr)
// The LCD side of things started from some driver code on the Banggood site.

#pragma once

#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "DMABuffer.h"
#include "Log.h"

class I2SCamera
{
  public:
  static gpio_num_t vSyncPin;
  static int framesReceived;
  static int xres;
  static int yres;
  static intr_handle_t i2sInterruptHandle;
  static intr_handle_t vSyncInterruptHandle;
  static int dmaBufferCount;
  static int dmaBufferActive;
  static DMABuffer **dmaBuffer;
  static int frameBytes;
  static volatile bool stopSignal;

  // Pete: Some type definitions for the callback functions
  static void (*BlockListener)(DMABuffer *buf);
  static void (*VSYNCListener)();

  typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
  } i2s_sampling_mode_t;


  static inline void i2sConfReset()
  {debug_println("i2sConfReset");
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;
    
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back);
  }
  
  void start()
  { debug_printf("start camera is called\n");
    i2sRun();
  }

  void stop()
  {  debug_printf("stop camera is called, but is it running?=%d\n", I2S0.conf.rx_start);
    // Make this safe to call even if camera has never been started.
    if (! I2S0.conf.rx_start) return;
    // if it is running, busy-wait till the end of the current frame.
    stopSignal = true;
    while(stopSignal);   
  }

  void oneFrame()    // Deprecated, because I don't like the busy loop. 
  {
    start();
    stop();
  }

  void i2sRestart();  // Pete
  
  static void i2sStop();
  static void i2sRun();

  static void dmaBufferInit(int bytes);
  static void dmaBufferDeinit();

  static bool initVSyncInterrupt(int pin);
  static void deinitVSyncInterrupt();
  
  static void IRAM_ATTR i2sInterrupt(void* arg);
  static void IRAM_ATTR vSyncInterrupt(void* arg);
  
  
  static bool i2sInit(const int VSYNC, const int HREF, const int PCLK, const int D0, const int D1, const int D2, const int D3, const int D4, const int D5, const int D6, const int D7);
  static bool init(const int VSYNC, const int HREF, const int PCLK, const int databus[], 
                 void (*blockListener)(DMABuffer *buf), 
                 void (*vSyncListener)());
 
};
