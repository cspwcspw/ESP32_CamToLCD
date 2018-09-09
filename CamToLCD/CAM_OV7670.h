
#pragma once

// Pete Wentworth cspwcspw@gmail.com.  Sept 2018
// Released under Apache License 2.0 
// Big chunks of the camera-side code were lifted from or inspired by
// https://github.com/bitluni/ESP32CameraI2S by Bitluni, and 
// https://github.com/igrr/esp32-cam-demo by Ivan Grokhotkov (igrr)

// The all-important reference manual for the camera:
// [1] http://web.mit.edu/6.111/www/f2016/tools/OV7670_2006.pdf


#include "Wire.h"
#include "I2SCamera.h"
#include "driver/ledc.h"

bool ClockEnable(int pin, int Hz)
{
  periph_module_enable(PERIPH_LEDC_MODULE);

  ledc_timer_config_t timer_conf;
  timer_conf.bit_num = (ledc_timer_bit_t)1;
  timer_conf.freq_hz = Hz;
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_0;
  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) {
    return false;
  }

  ledc_channel_config_t ch_conf;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.duty = 1;
  ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  ch_conf.gpio_num = pin;
  err = ledc_channel_config(&ch_conf);
  if (err != ESP_OK) {
    return false;
  }
  return true;
}

void ClockDisable()
{
  periph_module_disable(PERIPH_LEDC_MODULE);
}

// From igrr project

//camera registers
static const int REG_GAIN = 0x00;
static const int REG_BLUE = 0x01;
static const int REG_RED = 0x02;
static const int REG_COM1 = 0x04;
static const int REG_VREF = 0x03;
static const int REG_COM2 = 0x09;
static const int REG_COM4 = 0x0d;
static const int REG_COM5 = 0x0e;
static const int REG_COM6 = 0x0f;
static const int REG_AECH = 0x10;
static const int REG_CLKRC = 0x11;
static const int REG_COM7 = 0x12;
static const int COM7_RGB = 0x04;
static const int REG_COM8 = 0x13;
static const int COM8_FASTAEC = 0x80;    // Enable fast AGC/AEC
static const int COM8_AECSTEP = 0x40;    // Unlimited AEC step size
static const int COM8_BFILT = 0x20;    // Band filter enable
static const int COM8_AGC = 0x04;    // Auto gain enable
static const int COM8_AWB = 0x02;    // White balance enable
static const int COM8_AEC = 0x0;
static const int REG_COM9 = 0x14;
static const int REG_COM10 = 0x15;
static const int REG_COM14 = 0x3E;
static const int REG_COM11 = 0x3B;
static const int COM11_NIGHT = 0x80;
static const int COM11_NMFR = 0x60;
static const int COM11_HZAUTO = 0x10;
static const int COM11_50HZ = 0x08;
static const int COM11_EXP = 0x0;
static const int REG_TSLB = 0x3A;
static const int REG_RGB444 = 0x8C;
static const int REG_COM15 = 0x40;
static const int COM15_RGB565 = 0x10;
static const int COM15_R00FF = 0xc0;
static const int REG_HSTART = 0x17;
static const int REG_HSTOP = 0x18;
static const int REG_HREF = 0x32;
static const int REG_VSTART = 0x19;
static const int REG_VSTOP = 0x1A;
static const int REG_COM3 = 0x0C;
static const int REG_MVFP = 0x1E;
static const int REG_COM13 = 0x3d;
static const int COM13_UVSAT = 0x40;
static const int REG_SCALING_XSC = 0x70;
static const int REG_SCALING_YSC = 0x71;
static const int REG_SCALING_DCWCTR = 0x72;
static const int REG_SCALING_PCLK_DIV = 0x73;
static const int REG_SCALING_PCLK_DELAY = 0xa2;
static const int REG_BD50MAX = 0xa5;
static const int REG_BD60MAX = 0xab;
static const int REG_AEW = 0x24;
static const int REG_AEB = 0x25;
static const int REG_VPT = 0x26;
static const int REG_HAECC1 = 0x9f;
static const int REG_HAECC2 = 0xa0;
static const int REG_HAECC3 = 0xa6;
static const int REG_HAECC4 = 0xa7;
static const int REG_HAECC5 = 0xa8;
static const int REG_HAECC6 = 0xa9;
static const int REG_HAECC7 = 0xaa;
static const int REG_COM12 = 0x3c;
static const int REG_GFIX = 0x69;
static const int REG_COM16 = 0x41;
static const int COM16_AWBGAIN = 0x08;
static const int REG_EDGE = 0x3f;
static const int REG_REG76 = 0x76;
static const int ADCCTR0 = 0x20;


class pw_OV7670: public I2SCamera
{

    enum Mode
    {
      QQQVGA_RGB565,
      QQVGA_RGB565,
      QVGA_RGB565
    };

  public:

    int mode;

    int readRegister(unsigned char reg) {

      // Magic SCCB 8-bit write address hard-wired into OV7670 is 0x42.
      // ([1] pg 11). Wire wants a 7-bit base address, i.e. 0x21
      Wire.beginTransmission(0x21);
      Wire.write(reg);
      Wire.endTransmission(1);               // the argument is critical
      Wire.requestFrom(0x21, 1, 1);          // request 1 byte from slave device
      int timeout = 100000;
      while (--timeout > 0)  {
        int val = Wire.read();      // returns -1 if no data yet available.
        if (val >= 0) return val;
      }
      return -1; // Give up
    }

    void writeRegister(unsigned char reg, unsigned char val) {
      Wire.beginTransmission(0x21);
      Wire.write(reg);
      Wire.write(val);
      Wire.endTransmission(1);
    }

    void testPattern(int kind) // 0-none, or 1,2,3
    {
      writeRegister(0x70, (readRegister(0x70) & 0x7F) | ((kind & 0x01) << 7));
      writeRegister(0x71, (readRegister(0x71) & 0x7F) | ((kind & 0x02) << 6));
    }

    void subsamplingControl(int com14, int downSample, int pclk_div)
    {
      writeRegister(REG_COM3, 0x04);  //DCW enable

      writeRegister(REG_COM14, com14);       //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register
      writeRegister(REG_SCALING_XSC, 0x3a);
      writeRegister(REG_SCALING_YSC, 0x35);

      writeRegister(REG_SCALING_DCWCTR, downSample);
      writeRegister(REG_SCALING_PCLK_DIV, pclk_div); //pixel clock divided by 4
      writeRegister(REG_SCALING_PCLK_DELAY, 0x02);
    }

    void frameControl(int hStart,  int vStart)
    {
      int hStop = (hStart + 640) % 784;
      writeRegister(REG_HSTART, hStart >> 3);
      writeRegister(REG_HSTOP,  hStop >> 3);
      writeRegister(REG_HREF, ((hStop & 0b111) << 3) | (hStart & 0b111));

      int vStop = (vStart + 480);
      writeRegister(REG_VSTART, vStart >> 2);
      writeRegister(REG_VSTOP, vStop >> 2);
      writeRegister(REG_VREF, ((vStop & 0b11) << 2) | (vStart & 0b11));
    }

    void saturation(int s)  //-2 to 2
    {
      //color matrix values
      writeRegister(0x4f, 0x80 + 0x20 * s);
      writeRegister(0x50, 0x80 + 0x20 * s);
      writeRegister(0x51, 0x00);
      writeRegister(0x52, 0x22 + (0x11 * s) / 2);
      writeRegister(0x53, 0x5e + (0x2f * s) / 2);
      writeRegister(0x54, 0x80 + 0x20 * s);
      writeRegister(0x58, 0x9e);  //matrix signs
    }

    void autoDeNoise(bool wanted)
    {
      if (wanted) {
        writeRegister(REG_COM16, readRegister(REG_COM16) | (1 << 4)); // set bit 4
      }
      else {
        writeRegister(REG_COM16, readRegister(REG_COM16) & (~(1 << 4))); // unset bit 4
      }
    }

    void softSleep(bool mustSleep) // false will wake up camera, true will make it sleep
    {
      if (mustSleep) {
        writeRegister(REG_COM2, (readRegister(REG_COM2) | (1 << 4)));
      }
      else {
        writeRegister(REG_COM2, (readRegister(REG_COM2) & (~(1 << 4))));
      }
    }

    void setDriveStrength(int val) {  // 0 to 3
      val = val % 4;
      writeRegister(REG_COM2, (readRegister(REG_COM2) & 0xFC) | val);
    }

    

    void setMode(int mode, bool autoStart) {       // 0,1 or 2 for QQQ, QQ, or Q VGA.  All are RGB565

      stop();     // In case the camera is already running

      if (mode < 0 || mode > 2) {  // Sanity check on argument
        mode = 2;
      }

      this->mode = mode;

      writeRegister(REG_COM7, 0b10000000);     // all registers default
      writeRegister(REG_CLKRC, 0b10000000);    // double clock?? My spec sheet says Reserved
      writeRegister(REG_COM11, 0b1000 | 0b10); // enable auto 50/60Hz detect + exposure timing can be less...
      writeRegister(REG_COM7, 0b100);          // RGB
      writeRegister(REG_COM15, 0b11000000 | 0b010000); //RGB565

      switch (mode) {
        case 0:
          xres = 80;
          yres = 60;
          subsamplingControl(0x1B, 0x33, 0xF3);
          frameControl(196, 14);
          break;
        case 1:
          xres = 160;
          yres = 120;
          subsamplingControl(0x1A, 0x22, 0xF2);
          frameControl(174, 14);
          break;
        case 2:
          xres = 320;
          yres = 240;
          subsamplingControl(0x19, 0x11, 0xF1);
          frameControl(154, 14);
          break;
      }

      //writeRegister(REG_COM10, 0x02); //VSYNC negative
      //writeRegister(REG_MVFP, 0x2b);  //mirror flip

      writeRegister(0xb0, 0x84); // no clue what this is but it's most important for colors
      saturation(1);
      writeRegister(0x13, 0xe7); // AGC AWB AEC all on
      writeRegister(0x6f, 0x9f); // Simple AWB

      // What is this comment about? I tried but don't see any difference.
      // Line 1029 of https://github.com/yandex/smart/blob/master/drivers/media/i2c/ov7670.c
      writeRegister(REG_CLKRC, 0b10000000);


      if (autoStart) {
        start();
      }
      debug_printf("Mode set to %d with xres=%d,  autoStart=%d\n", mode, xres, autoStart);
    }

    pw_OV7670(const int SIOD, const int SIOC, const int VSYNC, const int HREF, const int XCLK, const int PCLK, const int databus[],
              void (*scanlineListener)(DMABuffer *buf),
              void (*vSyncListener)())
    {
      Wire.begin(SIOD, SIOC); // join the i2c bus (address optional for master)
      Wire.reset();           // Without this, we cannot always get XCLK clock going.

      while (true) {
        bool succeeded = true;
        ClockEnable(XCLK, 20000000);  // base is 80MHz, at 20MHz I get 25 VSYNCs per sec

        // Observation: this seems to work on the scope at 40Mhz, 20Mhz, etc.   At fast rates, maybe
        // because of long leads and capacitance on my breadboard, the peak-to-peak voltage reduces
        // from 3.3v down to about 2V p-p at 20MHz.  At 10Mhz the wave starts to show a flat top and
        // flat bottom, and it is no longer de-stabilized by flashing a torch into the camera.
        // http://embeddedprogrammer.blogspot.com/2012/07/hacking-ov7670-camera-module-sccb-cheat.html
        // says spec sheet limits for clock are 10MHz - 48MHz.  Others report running it slower than
        // 10MHz with success.

        // Once the clock is running the camera will deliver frames, and a VSYNC at the
        // end of each frame.  So this is really just a sanity check.
        pinMode(VSYNC, INPUT);
        int ttl = 1000000;
        while (!digitalRead(VSYNC) && --ttl > 0);  // Wait, but fail if it takes too long.
        if (ttl <= 0) {
          succeeded = false;
          Serial.println(" VSYNC never went low");
        }
        ttl == 100000;
        while (digitalRead(VSYNC) && --ttl > 0);
        if (ttl <= 0) {
          succeeded = false;
          Serial.println(" VSYNC never went high");
        }
        if (succeeded) {
          //Serial.println(" VSYNC done");
          break; // out of the retry loop
        }
      }
      delay(10);

      // Once we can talk to the camera using SCCB, set up the i2s data path and the interrupts, etc.
      I2SCamera::init(VSYNC, HREF, PCLK, databus, scanlineListener, vSyncListener);
    }

    void TestSuite(char *caller)
    {
      // The camera can auto adjust gain, exposure, etc. and as it does
      // so it writes current settings to some registers.
      // So we use -1 in the table below to mean "unpredictable" value.
      // This is just part of a test harness to ensure some sanity.
      // It may well be that other versions of the camera have some
      // different values in the registers.
      // Once we write setup data to the registers, some tests may fail.
      // So this test is probably only valid directly after a reset of the camera.
      const int NumRegs = 0xCA;
      int expectedInRegister[NumRegs] =
      { -1, -1, -1, -1, -1, -1, -1, -1,
        -1, 0x01, 0x76, 0x73, 0x00, 0x00, 0x01, 0x43,
        -1, 0x80, 0x00, 0x8F, 0x4A, 0x00, 0x00, 0x11,
        0x61, 0x03, 0x7B, 0x00, 0x7F, 0xA2, 0x01, 0x00,
        0x04, 0x02, 0x01, 0x00, 0x75, 0x63, 0xD4, 0x80,
        0x80, -1,   0x00, 0x00, 0x80, 0x00, 0x00, -1,
        0x08, 0x30, 0x80, 0x08, 0x11, -1, -1, 0x3F,
        0x01, 0x00, 0x0D, 0x00, 0x68, 0x88, 0x00, 0x00,
        0xC0, 0x08, 0x00, 0X14, 0xF0, 0x45, 0x61, 0x51,
        0x79, -1, -1, 0X00, 0x00, -1, -1, 0x40,
        0X34, 0X0C, 0X17, 0X29, 0X40, 0X00, 0X40, 0X80,
        0X1E, 0X91, 0X94, 0XAA, 0X71, 0X8D, 0X0F, 0XF0,
        0XF0, 0XF0, 0X00, 0X00, 0X50, 0X30, 0X00, 0X80,
        0X80, 0X00, -1, 0X0A, 0X02, 0X55, 0XC0, 0X9A,
        0X3A, 0X35, 0X11, 0X00, 0X00, 0X0F, 0X01, 0X10,
        0X00, 0X00, 0X24, 0X04, 0X07, 0X10, 0X28, 0X36,
        0X44, 0X52, 0X60, 0X6C, 0X78, 0X8C, 0X9E, 0XBB,
        0XD2, 0XE5, 0X00, 0X00, 0X00, 0X0F, 0X00, 0X00,
        0X00, 0X00, 0X00, 0X00, 0X50, 0X50, 0X01, 0X01,
        0X10, 0X40, 0X40, 0X20, 0X00, 0X99, 0X7F, 0XC0,
        0X90, 0X03, 0X02, -1, 0X00, 0X0F, 0XF0, 0XC1,
        0XF0, 0XC1, 0X14, 0X0F, 0X00, 0X80, 0X80, 0X80,
        0X00, 0X00, 0X00, 0X80, 0X00, 0X04, 0X00, 0X66,
        0X00, 0X06, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
        0X00, 0X00, -1, -1, -1, -1, -1, -1,
        -1, 0xC0
      };

      Wire.reset();

      Serial.printf("--- OV7670 sanity unit tests (called from %s) ---\n", caller);
      // Can we read correct expected values from registers?
      int passed = 0, failed = 0, skipped = 0;
      bool hadPasses = false;
      for (int i = 0; i < NumRegs; i++) {
        int expected = expectedInRegister[i];
        if (expected < 0) {
          skipped++;
        }
        else {
          int val = readRegister(i);
          if (val == expected) {
            // Serial.printf("%02x=%02x ", i, val);
            passed++;
            hadPasses = true;
          }
          else {
            if (hadPasses) {
              printf("passed.\n");
              hadPasses = false;
            }
            if (failed < 10) {
              Serial.printf("Reg %02x: %02x,  fail - expected to get %02x.\n", i, val, expected);
            }
            failed++;
          }
        }
      }
      if (failed == 0) {
        Serial.printf("--- YAY! - readRegister() tests all passed!\n");
      }
      else {
        Serial.printf("\n--- readRegister: Passed=%d, Failed=%d, Skipped=%d\n", passed, failed, skipped);
      }

      // Now write to a register and see that it works OK.
      int testReg = 0x1A; // this register looks innocuous enough...
      int val1 = readRegister(testReg);
      int val2 =  0x55;
      writeRegister(testReg, val2);
      int val3 = readRegister(testReg);
      // Put the original value back and see that it also gets there.
      writeRegister(testReg, val1);
      int val4 = readRegister(testReg);
      if (val1 == val4 && val2 == val3) {
        Serial.printf("--- YAY! - registerWrite() / readback passed!\n");
      }
      else
      {
        Serial.printf("Register write: failed.\nStarted as %02X, wrote %02X, read back %02X, and finally had %02X .\n", val1, val2, val3, val4);
      }
      if (failed > 0) {
        Serial.printf("Register values are only relaible after a camera RESET.\n");
        Serial.printf("If your camera RESET pin is wired high, you'll need to power everything off.\n");
      }
      Serial.printf("--- OV7670 test suite completed. ---\n");
    }

};

