//parts of his code are taken from
//https://github.com/igrr/esp32-cam-demo
//by Ivan Grokhotkov
//released under Apache License 2.0
//
// Pete Wentworth: I further hacked the code quite extensively, Sept 2018.


#include "I2SCamera.h"
#include "Log.h"

int I2SCamera::framesReceived = 0;
int I2SCamera::xres = 640;
int I2SCamera::yres = 480;
gpio_num_t I2SCamera::vSyncPin = (gpio_num_t)0;
intr_handle_t I2SCamera::i2sInterruptHandle = 0;
intr_handle_t I2SCamera::vSyncInterruptHandle = 0;
int I2SCamera::dmaBufferCount = 0;
int I2SCamera::dmaBufferActive = 0;
DMABuffer **I2SCamera::dmaBuffer = 0;

//unsigned char* I2SCamera::frame = 0;
//int I2SCamera::framePointer = 0;

int I2SCamera::frameBytes = 0;
volatile bool I2SCamera::stopSignal = false;

// The callbacks to a listener who is interested...
void (*I2SCamera::BlockListener)(DMABuffer *buf);
void (*I2SCamera::VSYNCListener)();

void IRAM_ATTR I2SCamera::i2sInterrupt(void* arg)
{
  I2S0.int_clr.val = I2S0.int_raw.val;
  //debug_printf("low level SL");
  // Grab a reference to the dma buffer with newest data,
  // and switch active DMA buffer to use the other one.
  DMABuffer *readyBlock = dmaBuffer[dmaBufferActive];
  dmaBufferActive = (dmaBufferActive + 1) % dmaBufferCount;

  if (BlockListener) // If a listener is registered to sink blocks, do the callback.
  {
    BlockListener(readyBlock);
  }

  if (stopSignal)
  {
    i2sStop();
    stopSignal = false;
  }
}

void IRAM_ATTR I2SCamera::vSyncInterrupt(void* arg)
{
  // debug_printf("low level vsync interrupt\n");
  GPIO.status1_w1tc.val = GPIO.status1.val;
  GPIO.status_w1tc = GPIO.status;
  if (VSYNCListener) // If a listener is registered to listen to VSYNC events, do the callback
  {
    VSYNCListener();
  }
}

void I2SCamera::i2sStop()
{
  debug_printf("i2sStop called isr=0x%08x\n", i2sInterruptHandle);
  // Make this safe to call even if camera has never been started.
  if (! i2sInterruptHandle) return;
  esp_intr_disable(i2sInterruptHandle);
  esp_intr_disable(vSyncInterruptHandle);
  i2sConfReset();
  I2S0.conf.rx_start = 0;
}

void I2SCamera::i2sRun()
{
  debug_print ("I2S Run ");

  // Whenever i2sRun is called, we might potentially discard the old DMA
  // buffer if it is the wrong size, and then allocate a new one.
  // This allows us to stop the camera, select a different mode, and
  // start it again.
  int requiredSize = xres * 2 * 2;
  if (dmaBuffer && dmaBuffer[0]->descriptor.length != requiredSize) {
    dmaBufferDeinit();
  }
  if (!dmaBuffer) {
    dmaBufferInit(requiredSize);
  }

  while (gpio_get_level(vSyncPin) == 0);
  while (gpio_get_level(vSyncPin) != 0);

  esp_intr_disable(i2sInterruptHandle);
  i2sConfReset();
  dmaBufferActive = 0;

  debug_print("Sample count ");
  debug_println(dmaBuffer[0]->sampleCount());
  I2S0.rx_eof_num = dmaBuffer[0]->sampleCount();

  I2S0.in_link.addr = (uint32_t) & (dmaBuffer[0]->descriptor);
  I2S0.in_link.start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;
  debug_printf("enabling i2sInterrupt %08x\n", i2sInterruptHandle);
  esp_intr_enable(i2sInterruptHandle);
  esp_intr_enable(vSyncInterruptHandle);
  I2S0.conf.rx_start = 1;
  debug_printf("IS2 started for camera.\n");
}

bool I2SCamera::initVSyncInterrupt(int pin)
{
  debug_print("Initializing VSYNC... ");
  vSyncPin = (gpio_num_t)pin;
  gpio_set_intr_type(vSyncPin, GPIO_INTR_POSEDGE);
  gpio_intr_enable(vSyncPin);
  // Bind the interrupt to our handler.
  if (gpio_isr_register(&vSyncInterrupt, (void*)"vSyncInterrupt", ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_IRAM, &vSyncInterruptHandle) != ESP_OK)
  {
    debug_println("failed!");
    return false;
  }
  debug_println("done.");
  return true;
}

void I2SCamera::deinitVSyncInterrupt()
{
  esp_intr_disable(vSyncInterruptHandle);
}

// This init should only ever be called once, after the camera is already generating VSYNCs
bool I2SCamera::init(const int VSYNC, const int HREF, const int PCLK, const int databus[],
                     void (*blockListener)(DMABuffer *buf),
                     void (*vSyncListener)())
{
  // Setup the callbacks to the main program
  BlockListener = blockListener;
  VSYNCListener = vSyncListener;

  i2sInit(VSYNC, HREF, PCLK, databus[7], databus[6], databus[5], databus[4], databus[3], databus[2], databus[1], databus[0]);

  initVSyncInterrupt(VSYNC);
  return true;
}

bool I2SCamera::i2sInit(const int VSYNC, const int HREF, const int PCLK, const int D0, const int D1, const int D2, const int D3, const int D4, const int D5, const int D6, const int D7)
{
  int pins[] = {VSYNC, HREF, PCLK, D0, D1, D2, D3, D4, D5, D6, D7};
  gpio_config_t conf = {
    .pin_bit_mask = 0,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
    conf.pin_bit_mask = 1LL << pins[i];
    gpio_config(&conf);
  }

  // Route input GPIOs to I2S peripheral using GPIO matrix, last parameter is invert
  gpio_matrix_in(D0,    I2S0I_DATA_IN0_IDX, false);
  gpio_matrix_in(D1,    I2S0I_DATA_IN1_IDX, false);
  gpio_matrix_in(D2,    I2S0I_DATA_IN2_IDX, false);
  gpio_matrix_in(D3,    I2S0I_DATA_IN3_IDX, false);
  gpio_matrix_in(D4,    I2S0I_DATA_IN4_IDX, false);
  gpio_matrix_in(D5,    I2S0I_DATA_IN5_IDX, false);
  gpio_matrix_in(D6,    I2S0I_DATA_IN6_IDX, false);
  gpio_matrix_in(D7,    I2S0I_DATA_IN7_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN8_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN9_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN10_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN11_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN12_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN13_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN14_IDX, false);
  gpio_matrix_in(0x30,  I2S0I_DATA_IN15_IDX, false);

  gpio_matrix_in(VSYNC, I2S0I_V_SYNC_IDX, true);
  gpio_matrix_in(0x38,  I2S0I_H_SYNC_IDX, false);  //0x30 sends 0, 0x38 sends 1
  gpio_matrix_in(HREF,  I2S0I_H_ENABLE_IDX, false);
  gpio_matrix_in(PCLK,  I2S0I_WS_IN_IDX, false);

  // Enable and configure I2S peripheral
  periph_module_enable(PERIPH_I2S0_MODULE);

  // Toggle some reset bits in LC_CONF register
  // Toggle some reset bits in CONF register
  i2sConfReset();
  // Enable slave mode (sampling clock is external)
  I2S0.conf.rx_slave_mod = 1;
  // Enable parallel mode
  I2S0.conf2.lcd_en = 1;
  // Use HSYNC/VSYNC/HREF to control sampling
  I2S0.conf2.camera_en = 1;
  // Configure clock divider
  I2S0.clkm_conf.clkm_div_a = 1;
  I2S0.clkm_conf.clkm_div_b = 0;
  I2S0.clkm_conf.clkm_div_num = 2;
  // FIFO will sink data to DMA
  I2S0.fifo_conf.dscr_en = 1;
  // FIFO configuration
  //two bytes per dword packing
  I2S0.fifo_conf.rx_fifo_mod = SM_0A0B_0C0D;  //pack two bytes in one dword see :https://github.com/igrr/esp32-cam-demo/issues/29
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
  I2S0.conf_chan.rx_chan_mod = 1;
  // Clear flags which are used in I2S serial mode
  I2S0.sample_rate_conf.rx_bits_mod = 0;
  I2S0.conf.rx_right_first = 0;
  I2S0.conf.rx_msb_right = 0;
  I2S0.conf.rx_msb_shift = 0;
  I2S0.conf.rx_mono = 0;
  I2S0.conf.rx_short_sync = 0;
  I2S0.timing.val = 0;

  debug_printf("i2sInit allocating interrupt, handle before = %08x.\n", i2sInterruptHandle);
  esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, &i2sInterrupt, NULL, &i2sInterruptHandle);
  debug_printf("i2sInit completed with handle = %08x.\n", i2sInterruptHandle);
  return true;
}

void I2SCamera::i2sRestart()  {
  // Pete: At the end of every frame is is possible that the is2 logic and the camera logic might 
  // have lost tight synchronization (I can deliberately provoke this by momentarily clamping PCLK 
  // low, as if the signal is noisy). I want to recover synchronization on the next frame.  But 
  // I don't want to "waste" waiting for another VSYNC to ensure we start with a "clean slate". 
  I2S0.conf.rx_start = 0;

  i2sConfReset();
  dmaBufferActive = 0;
  I2S0.rx_eof_num = dmaBuffer[0]->sampleCount();
  I2S0.in_link.addr = (uint32_t) & (dmaBuffer[0]->descriptor);
  I2S0.in_link.start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;
  I2S0.conf.rx_start = 1;
}

void I2SCamera::dmaBufferInit(int bytes)
{
  dmaBufferCount = 2;
  dmaBuffer = (DMABuffer**) malloc(sizeof(DMABuffer*) * dmaBufferCount);
  for (int i = 0; i < dmaBufferCount; i++)
  {
    dmaBuffer[i] = new DMABuffer(bytes);
    if (i)
      dmaBuffer[i - 1]->next(dmaBuffer[i]);
  }
  dmaBuffer[dmaBufferCount - 1]->next(dmaBuffer[0]);
  debug_printf("DMA buffers (%d), each of size %d bytes were allocated.\n", dmaBufferCount, bytes);
}

void I2SCamera::dmaBufferDeinit()
{
  if (!dmaBuffer) return;
  debug_println("Deleting DMA buffers.");
  for (int i = 0; i < dmaBufferCount; i++)
    delete(dmaBuffer[i]);
  delete(dmaBuffer);
  dmaBuffer = 0;
  dmaBufferCount = 0;
}
