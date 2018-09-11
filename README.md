
# ESP32_CamToLCD

Pete Wentworth  cspwcspw@gmail.com



The project uses an ESP32 processor to 
stream video from my OV7670 camera, and to send
it in real time to a 320x240 LCD screen. 

I am able to get 25 frames per second throughput at
320x240 pixels, each pixel is 16-bits, or two bytes, 
RGB565 colour format.  This format is supported by both the 
camera the screen, so no format conversion is required.

I discuss some key overview ideas here to help clarify what I did.  

Thes rest of this document is in sections:
* The Hardware,
* The Camera Side of Things,
* The Main Program,
* The LCD Side of Things,
* A Summary.

## The Hardware 

All are cheap goodies from Banggood: 

* The [OV7670 Camera](https://www.banggood.com/VGA-OV7670-CMOS-Camera-Module-Lens-CMOS-640X480-SCCB-With-I2C-Interface-p-1320365.html) has no FIFO nor does it have its own clock.  

* The [320x240 LCD Arduino shield](https://www.banggood.com/2_4-Inch-TFT-LCD-Shield-240320-Touch-Board-Display-Module-With-Touch-Pen-For-Arduino-UNO-p-1171082.html) has an 8-bit parallel data bus - it is not a serial SPI device. 

* And an [ESP32 Devkit](https://www.banggood.com/ESP32-Development-Board-WiFiBluetooth-Ultra-Low-Power-Consumption-Dual-Cores-ESP-32-ESP-32S-Board-p-1109512.html).  

I didn't do a schematic. Instead, I wrote a more general document [ESP_32_Pin_Usage](https://github.com/cspwcspw/ESP32_CamToLCD/blob/master/ESP32_Pin_Usage.pdf) which summarizes some pin usage limitations I found helpful.  The exact pin connections for this project at the end of that document.

## The Camera Side of Things

My starting point was the [ESP32CameraI2S project](https://github.com/bitluni/ESP32CameraI2S) by Bitluni, in turn built on the [esp32-cam-demo](https://github.com/igrr/esp32-cam-demo) by Ivan Grokhotkov (igrr).

They use one of the two ESP32's 
I2S engines in conjunction with
Direct Memory Access (DMA).  The camera is the master 
device controlling the I2S bus, and sends pixel and clocking
information to the ESP32. With the correct setup, bytes are 
directly stored into a DMA buffer in memory.

All this takes place in the background - it leaves
the ESP32 free to do other things.  (Thats what all these extra
peripheral controllers in the ESP32 are meant for: to relieve
the burden on the main processors.) 

When the IS2 controller has gathered a whole scan line into the
buffer, it generates an interrupt. Now the main processor can deal
with the latest scanline.  

There is a second interrupt in play too: at the end of each frame
the camera generates a VSYNC signal, which is used to 
cause a VSYNC interrupt.  This allows us to deal with the 
finalization of the current frame and set up for the start of the next frame.  

At 25 frames per second, 320x240, we get _**6000**_ scanlines
arriving per second. Allowing some padding between frames, 
we have about _**156**_ microseconds between successive scanline interrupts.

We can also expect 25 VSYNC interrupts per second.   

Compared to the two projects mentioned above, I do some things differently.

Firstly, they allocate memory for the whole video frame, and re-pack new
scanlines into the frame memory as they arrive.  But the ESP32 doesn't have
enough memory for a 320x240x2-byte frame (150KB), so this approach 
limits us to lower resolutions.  

I do not move the scanlines out of the DMA buffer. I consume the data in real time and free up the DMA buffer before it is needed again.  There are two DMA buffers used alternately. So while the I2S hardware is filling one, we need to be emptying the other one.  

In the original camera code, when capturing a frame, 
the code waits for VSYNC to synchronize with the
camera, starts the IS2 engine to collect the scan lines, 
and then sits in a busy loop waiting until the whole frame
has arrived.  

Sitting in a busy loop somewhat defeats the purpose 
of having a specialized I2S engine running on its own silicon.   

So I keep the camera and the I2S capture engine running continuously.  After each VSYNC I immediately stop and restart the I2S engine.  More about that in a moment. 

I added two callback functions from the existing interrupt 
handlers into the main code, 
so when the I2S hardware gets to the end of a scanline and 
generates a "DMABuffer Ready" interrupt, it in turn calls
back to the main program to deal with the scanline.  

So in summary, my camera code is a direct descendant of the camera 
projects with a few conceptual differences:
* The idea is to consume or dispose of scanlines as they arrive. 
* I don't allocate memory or pack scanlines into a frame buffer.
* I run the camera and I2S engine (almost) continuously, 
so I avoid the expensive wait for 
a fresh VSYNC to re-synchonize I2S and camera.
* I get the interrupt handlers to call back to my main program so that
the application can deal with them.
* I don't wait in any busy loops in the camera code.  My only busy loop is ``loop()`` in the main program. 

After each VSYNC I can momentarily stop and restart the I2S engine, but quickly enough so that I know the next scanline will be the first of the next frame.
I do this in case we do lose scanlines or pixels somehow. (I tested this by momentarily grounding the PCLK signal from the camera to confuse the I2S engine. Of course I get bad-looking frames, but once I remove
the ground, things correct themselves after the next VSYNC.)


## The Main Program

It sets up the camera mode, sets up the LCD, and starts the camera.
Interrupts start pouring in, each with a DMABuffer that holds
a scanline of pixel data.  After some scanlines we get a VSYNC.

I like to organize complex logic like this as a state machine. 
So we consider the different states the system could be in,
and what events or processes should move the state machine to another state.  

My design identified five different states, and the two interrupt events,
plus some other things that cause a state change. 

```
enum State {
  Lost,    // We don't know where we are, and need a VSYNC to synchronize ourselves.
  Priming, // When we hit this state we'll prime the sink: e.g. send a frame header, open a file, set up LCD etc.
  Running, // Queueing blocks as they arrive in the interrupt handler, and sinking the data in the main loop.
  Wrapup,  // We got a VSYNC. We can wrap up the frame / close a file, restart the I2S engine, print stats, etc.
  Overrun  // If either VSYNC or a scanline interrupts before we're finalized, we've lost the beat.
};
```

When the system starts (or after an error or change of camera mode), we
start off in the _**Lost**_ state.  The main loop ignores any scanlines that arrive.  We remain in this state until we get a VSYNC.  

The VSYNC arrival transfers us to _**Priming**_ state.
Now the main loop primes the LCD: 
_"Expect scanlines, here is where I want you to put them on your screen"_.  Once priming tasks have taken place, the main loop advances the state to _**Running**_. 

While we're running, when a scanline arrives (via the callback), 
we copy the DMABuffer address into a variable, and exit the interrupt.
The main loop continuously tests the variable, and when it finds a 
buffer address it passes it to the LCD to consume the buffer contents, 
and then sets local variable back to NULL.
So the variable is also acting as a flag for the main loop to know 
if a fresh DMAbuffer is waiting.   

If a new scanline arrives before the main loop has managed 
to clear the old one, we just overwrite the local variable. 
Our image will get corrupted, but it is a bit like a 
best effort audio stream - there is nothing we can do to recover
the lost data.

A VSYNC finally take us out of the _**Running**_ state to _**Wrapup**_ 
state.  This is when we have the most time available in the main loop. 
We assume the final scanline has already been flushed (there is a
considerable timing gap before VSYNC) 
we count the frame, reset the I2S engine,
occasionally print diagnostic information, and even poll for user input 
to change the camera mode or tweak some camera register settings.  

If either callback occurs during wrap-up, we'll 
be put into the _**Overrun**_ state.  Once we've done all the 
wrap-up tasks for the current frame, we've either beaten our 
deadline and can immediately go back to _**Priming**_ 
for the next frame.  If we've overrun, we're lost 
we have no option but to skip a frame and wait for the next VSYNC.
This we accomplish by transitioning back to state _**Lost**_.   

Besides some initialization, the only two ways our main 
program interacts with the LCD is to prime it to tell it 
where to put the next pixels, and then to pass it the DMABuffer 
addresses whenever a buffer needs to be consumed. 


## The LCD Side of Things

We've covered elsewhere how we drive the LCD at high speed.  In summary,
we send a byte of data at a time to the LCD.  The fast way to do this is to have available
a pre-arranged 32-bit mask.  Suppose the byte I want to send is `0x52`.  Its bit pattern is `01010010`. (i.e. three GPIO lines need to be high).

Depending on which GPIOs are wired up to each LCD data bit, we can construct a 
32-bit "set-these-bits-mask" with three one-bits in the correct places. Direct port writes allow us to also use a mask to clear all 8 data lines at once.  

Since there are only 256 possible bytes to send to the LCD, once we know 
GPIO pin numbers for the data bus we can pre-compute all the bit-setting masks.

So at initialization we compute a 256-entry table.  Sending a byte onto the bus requires
clearing the bus, using the byte as an array index to find the bitmask 
that will set the appropriate bits. We can then use a direct port write 
to output the byte.  Once the byte is on the bus we need to strobe high
the LCD Write line. A trick of taking it low at the same time
as we take other bus lines low means writing a byte becomes a really
efficient macro:
```
#define Write_Byte(d) { GPIO.out_w1tc=busPinsLowMask; GPIO.out_w1ts=outputMaskMap[d]; LCD_WR_HIGH(); }
```


The other important method in the LCD code is the code for consuming the DMABuffer, and sending bytes as fast as we can to the LCD device.    
```
    void SinkDMABuf(int xres, DMABuffer *buf) {
      
      unsigned char *p = buf->buffer;      // Pointer to start of data

      LCD_DC_HIGH();                       // Bytes are data, not LCD commands.     
      for (int i = 0; i < xres * 4; i += 4) {
        Write_Byte(p[i]);        // Every second byte of the DMA buffer has pixel data
        Write_Byte(p[i + 2]);
      }
    }
```

We said earlier that we expect 6000 scanlines per second. 
So we need to handle the scanline in less than 156 microseconds.  We
put timing measurements around the call to this method, 
and measured just under 100 microseconds to send out 640 bytes 
(each pixel is two bytes). 

6000 x 100 microsec is about 600ms. So this hotspot method accounts for 
about 60% of our processor time on the core that we're running on.

## Summary

We kept the interrupt routines short, and we don't fall foul of the 
Watchdog Timer. We don't use intermediate buffer memory,
we don't do format conversion of the data. This all helps speed things up.

A fun project, I learned plenty, and I hope you do too. For what we get for 
about $7, the ESP32 is amazing!  
  



 

