
#pragma once

// Parts of this code (the LCD setup, drawing rectangles, clearing the screen) started from code at
// https://www.banggood.com/2_4-Inch-TFT-LCD-Shield-240320-Touch-Board-Display-Module-With-Touch-Pen-For-Arduino-UNO-p-1171082.html
// LCD_Touch\2.4inch_Arduino_HX8347G_V1.0\Arduino Demo_ArduinoUNO&Mega2560\Example01-Simple test\Simple test for UNO\_HX8347_uno
// His "Simple test" for the UNO has no dependecies on any libraries (i.e. doesn't use AdaFruit, etc.)
// It just clears the screen with some colours, and draws some rectangles.

// My contribution was to clean it up, port it to the ESP32, wrap it into a class, and find some optimizations.
// Pete Wentworth, September 2018.

// This LCD is not a serial SPI LCD - it is a parallel, 8-bit-at-a-time model.

#include "DMABuffer.h" // Because we want to unpack camera pixels from the DMA buffer ...


//******** The LCD pins and mapping.  **********
const int TX2 = 17;
const int RX2 = 16;
int lcdDataBus[] = {18, 4, 19, 21, 22, 23, TX2, RX2};  // D7, D6, D5, D4, D3, D2, D1, D0

#define LCD_WR    2   // When it goes high, the LCD latches bits from the output pin
#define LCD_WR_SETOUTPUT() pinMode(LCD_WR, OUTPUT)
#define LCD_WR_HIGH()      GPIO.out_w1ts=(1 << LCD_WR)
#define LCD_WR_LOW()       GPIO.out_w1tc=(1 << LCD_WR)


#define LCD_DC   15  // HIGH means LCD interprets bits as data, LOW means the bits are a command.
#define LCD_DC_SETOUTPUT() pinMode(LCD_DC, OUTPUT)
#define LCD_DC_HIGH()      GPIO.out_w1ts=(1 << LCD_DC)
#define LCD_DC_LOW()       GPIO.out_w1tc=(1 << LCD_DC)


// Uncomment this line if you have a GPIO pin for LCD_RD
// If you don't need to read from the device, strap it's LCD_RD pin high.
// #define LCD_RD   GPIO-pin-needs-to-be-assigned

#ifdef LCD_RD
#define LCD_RD_SETOUTPUT()    pinMode(LCD_RD, OUTPUT)
#define LCD_RD_HIGH()         GPIO.out_w1ts=(1 << LCD_RD)
#define LCD_RD_LOW()          GPIO.out_w1tc=(1 << LCD_RD))
#else
#define LCD_RD_SETOUTPUT()
#define LCD_RD_HIGH()
#define LCD_RD_LOW()
#endif


// Uncomment this line if you have a GPIO pin for LCD_CS
// If you want the device permanently selected, strap this pin low.
// #define LCD_CS   GPIO-pin-needs-to-be-assigned

#ifdef LCD_CS
#define LCD_CS_SETOUTPUT()    pinMode(LCD_CS, OUTPUT)
#define LCD_CS_HIGH()         GPIO.out_w1ts=(1 << LCD_CS)
#define LCD_CS_LOW()          GPIO.out_w1tc=(1 << LCD_CS)
#else
#define LCD_CS_SETOUTPUT()
#define LCD_CS_HIGH()
#define LCD_CS_LOW()
#endif


// Uncomment this line if you have a GPIO pin for LCD_RESET
// Consider tying this pin to EN so that the LCD resets when the ESP32 resets.
// #define LCD_RESET   GPIO-pin-needs-to-be-assigned

#ifdef LCD_RESET
#define LCD_RESET_SETOUTPUT()    pinMode(LCD_RESET, OUTPUT)
#define LCD_RESET_HIGH(dly)      { GPIO.out_w1ts=(1 << LCD_RESET); delay(dly); }
#define LCD_RESET_LOW(dly)       { GPIO.out_w1tc=(1 << LCD_RESET); delay(dly); }
#else
#define LCD_RESET_SETOUTPUT()
#define LCD_RESET_HIGH(dly)
#define LCD_RESET_LOW(dly)
#endif



const int RED  = 0xf800;
const int GREEN = 0x07E0;
const int BLUE = 0x001F;
const int YELLOW = 0xFFE0;
const int MAGENTA = 0xF81F;
const int CYAN = 0x07FF;

int backGroundColours[] = {RED, GREEN, BLUE}; // Red, Green, Blue in RGB565 format

class LCD
{
  public:

    unsigned int outputMaskMap[256];  // Each 8-bit byte (used as an index) has a 32-bit mask to set the appropriate GPIO lines
    unsigned int busPinsLowMask;      // This has clear bits for all the lcdDataBus pins and also for the LCD_WR pin,
    //                                   which we set low at the same time.

    // Key optimization idea: Ahead of time, prepare output masks for all possible byte values. Store them in outputMaskMap.
    //  To write a byte to the LCD, set all databus GPIOs low (also include LCD_WR) by writing a fixed mask to w1tc.
    //  Then use the byte to be written as an index into outputMaskMap, and write that mask to w1ts to set the GPIOs.
    //  Then latch that data onto the LCD device by making LCD_WR go high.
#define Write_Byte(d) { GPIO.out_w1tc=busPinsLowMask; GPIO.out_w1ts=outputMaskMap[d]; LCD_WR_HIGH(); }


    void Write_Command(unsigned char d)
    {
      LCD_DC_LOW();    // Here comes a command
      Write_Byte(d);
    }

    void Write_Data(unsigned char d)
    {
      LCD_DC_HIGH();    // Here comes data
      Write_Byte(d);
    }


    // Where on the LCD do you want the data to go?
    void Address_set(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
    {
      Write_Command(0x2a);
      Write_Data(x1 >> 8);
      Write_Data(x1);
      Write_Data(x2 >> 8);
      Write_Data(x2);
      Write_Command(0x2b);
      Write_Data(y1 >> 8);
      Write_Data(y1);
      Write_Data(y2 >> 8);
      Write_Data(y2);
      Write_Command(0x2c);
    }

    void Init(void)
    {
      LCD_RESET_HIGH(5);
      LCD_RESET_LOW(15);
      LCD_RESET_HIGH(15);

      LCD_CS_LOW();

      Write_Command(0xCB);
      Write_Data(0x39);
      Write_Data(0x2C);
      Write_Data(0x00);
      Write_Data(0x34);
      Write_Data(0x02);

      Write_Command(0xCF);
      Write_Data(0x00);
      Write_Data(0XC1);
      Write_Data(0X30);

      Write_Command(0xE8);
      Write_Data(0x85);
      Write_Data(0x00);
      Write_Data(0x78);

      Write_Command(0xEA);
      Write_Data(0x00);
      Write_Data(0x00);

      Write_Command(0xED);
      Write_Data(0x64);
      Write_Data(0x03);
      Write_Data(0X12);
      Write_Data(0X81);

      Write_Command(0xF7);
      Write_Data(0x20);

      Write_Command(0xC0);    //Power control
      Write_Data(0x23);       //VRH[5:0]

      Write_Command(0xC1);    //Power control
      Write_Data(0x10);       //SAP[2:0];BT[3:0]

      Write_Command(0xC5);    //VCM control
      Write_Data(0x3e);       //Contrast
      Write_Data(0x28);

      Write_Command(0xC7);    //VCM control2
      Write_Data(0x86);       //--

      Write_Command(0x36);    // Memory Access Control
      // I like the settings for a forward-facing camera, so text appears normally on the LCD.
      // Write_Data(0x28);    // Exchange rows and cols for landscape
      Write_Data(0xE8);       // Exchange rows and cols for landscape, and flip X (which is now Y) and flip Y


      Write_Command(0x3A);
      Write_Data(0x55);

      Write_Command(0xB1);
      Write_Data(0x00);
      Write_Data(0x18);

      Write_Command(0xB6);    // Display Function Control
      Write_Data(0x08);

      Write_Data(0x82);
      Write_Data(0x27);

      Write_Command(0x11);    //Exit Sleep
      delay(120);

      Write_Command(0x29);    //Display on
      Write_Command(0x2c);
    }


    void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)
    {
      unsigned int i, j;
      Write_Command(0x02c); //write_memory_start

      LCD_CS_LOW();
      l = l + x;
      Address_set(x, y, l, y);
      j = l * 2;
      for (i = 1; i <= j; i++)
      {
        Write_Data(c);
      }
      LCD_CS_HIGH();
    }

    void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)
    {
      unsigned int i, j;
      Write_Command(0x02c); //write_memory_start

      LCD_CS_LOW();
      l = l + y;
      Address_set(x, y, x, l);
      j = l * 2;
      for (i = 1; i <= j; i++)
      {
        Write_Data(c);
      }
      LCD_CS_HIGH();
    }

    void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c)
    {
      H_line(x  , y  , w, c);
      H_line(x  , y + h, w, c);
      V_line(x  , y  , h, c);
      V_line(x + w, y  , h, c);
    }

    void ClearScreen(unsigned int backColour)
    {
      LCD_CS_LOW();
      // Assume we're in landscape
      Address_set(0, 0, 320, 240);

      unsigned char u = backColour >> 8;
      unsigned char v = backColour;
      LCD_DC_HIGH();
      for (int i = 0; i < 240 * 320; i++)
      {
        Write_Byte(u);
        Write_Byte(v);
      }
      LCD_CS_HIGH();
    }

    void createOutputMasks()
    {
      busPinsLowMask = (1 << LCD_WR);
      for (int i = 0; i < 256; i++) { // possible byte values
        int v = i;
        for (int k = 7; k >= 0; k--) { // look at each bit in turn, starting at least significant
          int pinNum = lcdDataBus[k];
          unsigned int pinBit = (0x0001 << pinNum);
          if (v & 0x01) {
            outputMaskMap[i] |=  pinBit;
          }
          else {
            if (i == 0) {
              busPinsLowMask |= pinBit;
              //   Serial.printf("Setting pinbit %08x so busPinsLowMask is %08x\n", pinBit, busPinsLowMask);
            }
          }
          v >>= 1;
        }
      }
      // Let's print the masks, in hex.
      /*
        Serial.println("Table of pins to make high for each byte value (in Hex):");
        for (int row = 0; row < 8; row++)
        {
          char sep = ' ';
          for (int col = 0; col < 8; col++) {
            int indx = row * 8 + col;
            Serial.printf("%c %08x", sep, outputMaskMap[indx]);
            sep = ',';
          }
          Serial.println();
        }
        Serial.println("-----------------");
      */
    }


    void SinkDMABuf(int xres, DMABuffer *buf) {
      // Here we unpack the DMA buffer to get at the bytes,
      // and send them to the LCD.
      // This is the "hotspot" code, so we optimize all we can.
      // My elapsed time call to this function with xres=320
      // shows just under 100 microsecs.
      // 320x240 frames at 25fps stablize when I can keep this
      // below 140 microsecs, so I have some room to play.
      // Unrolling the loop doesn't make it go any faster.

      unsigned char *p = buf->buffer;      // Pointer to start of data
      unsigned char *limit = p + xres * 4 - 0; // Pointer just beyond end of data

      LCD_DC_HIGH();                       // Hey LCD! Expect some data!

      while (p < limit) {
        Write_Byte(*p);         // Every second byte of the DMA buffer has pixel data
        Write_Byte(*(p + 2));
        p += 4;
      }
    }

    void Setup()
    {
      Serial.println("Setting up LCD data and control pins");
      for (int p = 0; p < 8; p++)
      {
        pinMode(lcdDataBus[p], OUTPUT);
      }

      createOutputMasks();

      LCD_RD_SETOUTPUT();
      LCD_RD_HIGH();

      LCD_WR_SETOUTPUT();
      LCD_WR_HIGH();

      LCD_DC_SETOUTPUT();
      LCD_DC_HIGH();

      LCD_CS_SETOUTPUT();
      LCD_CS_HIGH();

      LCD_RESET_SETOUTPUT();
      LCD_RESET_HIGH(15);

      Init();
    }

    void TestSuite()
    { // It's not really a test suite, it just does some colourful things.
      int dly = 512;
      long t0, t1;
      for (int i = 0; i < 8; i++)
      {
        if (dly > 0 && i > 0) {
          delay(dly);
          dly >>= 1;
        }
        t0 = micros();
        int backColour = backGroundColours[i % 3];
        ClearScreen(backColour);
        t1 = micros();
        if (i >= 3)
        {
          // Now draw some rectangles on the background.
          for (int i = 0; i < 100; i++)
          {
            Rect(random(300), random(300), random(300), random(300), random(65535));
            // rectangle at x, y, width, height, color
          }
        }
      }
      Serial.printf("%d microsecs to set all screen pixels\n", t1 - t0);
      float fps = 1000000.0 / (t1 - t0);
      float dataRate = ((fps / (1024.0 * 1024.0)) * 240 * 320 * 2);
      Serial.printf("Theoretical max LCD FPS=%.2f.\nClearScreen() data rate=%.2f MB per sec.\n", fps, dataRate);
    }
};

