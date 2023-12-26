#include "epd.h"
#include "nrf_drv_spi.h"
#include "font5x5.h"
#include "font8x8.h"

#include "src/gj/src/gj/base.h"
#include "src/gj/src/gj/eventmanager.h"

/*

128x256
---- 128 ---- X
|
|
256
|
|
|
Y

*/

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done = true;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    SER("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0)
    {
        SER(" Received: \r\n");
        //NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

extern "C" 
{
  void spi_write(const void * data, size_t size)
  {
    //spi_xfer_done = false;
      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t*)data, size, NULL, 0));

      //while(spi_xfer_done == false)
      //{
      //  __WFE();
      //}
      

  }

  void spi_read(void * data, size_t size)
  {
    //spi_xfer_done = false;
      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, (uint8_t*)data, size));

      //while(spi_xfer_done == false)
      //{
      //  __WFE();
      //}
      

  }
}


uint8_t GetRTCmd(Epaper &epd)
{
  if (epd.m_rt == EPD_RT_PREV)
    return 0x26;
  else
    return 0x24;  //EPD_RT_MAIN
}

void write_command(Epaper &epd, uint8_t c)
{
    WritePin(epd.m_pins.m_dc, 0);
    WritePin(epd.m_pins.m_cs, 0);
    spi_write(&c, sizeof(c));
    WritePin(epd.m_pins.m_cs, 1);
    WritePin(epd.m_pins.m_dc, 1);
}

void write_data_begin(Epaper &epd)
{
    //WritePin(epd.m_pins.m_dc, 1);
    WritePin(epd.m_pins.m_cs, 0);
}

void write_data_end(Epaper &epd)
{
    WritePin(epd.m_pins.m_cs, 1);
}

void write_data(uint8_t c)
{
    spi_write(&c, sizeof(c));
}

void write_data(Epaper &epd, uint8_t c)
{
    write_data_begin(epd);
    spi_write(&c, sizeof(c));
    write_data_end(epd);
}

void write_data(Epaper &epd, const uint8_t *d, uint32_t l)
{
  for (int i = 0 ; i < l ; ++i)
    write_data(epd, d[i]);
}

void write_command_and_data(Epaper &epd, uint8_t c, const uint8_t *d, uint32_t l)
{
  write_command(epd, c);
  write_data(epd, d, l);
}


void write_command_and_arg(Epaper &epd, uint8_t c, uint8_t a)
{
  write_command(epd, c);
  write_data(epd, a);
}


void write_command_and_args(Epaper &epd, uint8_t c, uint8_t a, uint8_t a2)
{
  write_command(epd, c);
  write_data(epd, a);
  write_data(epd, a2);
}


uint8_t epd_is_busy(Epaper &epd)
{
  const bool busy = ReadPin(epd.m_pins.m_busy) != 0;

  return busy;
}

uint8_t epd_wait_busy(Epaper &epd)
{
  uint32_t timeout = 0;
  uint32_t printTimeout = 2500;

  while (epd_is_busy(epd))
  {
    if (timeout == printTimeout)
    {
      printf("busy\n\r");
    }

    timeout++;
    if (timeout > 4000)
    {
      printf("timeout\n\r");
      return 1;
    }
    Delay(100);
  }

  if (timeout >= printTimeout)
  {
    printf("done\n\r");
  }
  
  return 0;
}

void epd_wait_busy_async(Epaper *epd)
{
  if (epd_is_busy(*epd))
  {
    if (epd->m_waitCount >= 20)
    {
      (*epd->m_cb)(epd, epd->m_cookie, false);
      return;
    }

    auto event = [=]()
    {
      epd_wait_busy_async(epd);
    };

    const uint32_t delay = epd->m_cbDelay;
    epd->m_cbDelay = 100 * 1000;

    GJEventManager->DelayAdd(event, delay);
  }
  else
  {
    (*epd->m_cb)(epd, epd->m_cookie, true);
  }
}

void epd_wait_busy_async(Epaper &epd, EpdCallback cb, uint32_t cookie)
{
  epd.m_cb = cb;
  epd.m_cookie = cookie;
  epd.m_waitCount = 0;
  epd.m_cbDelay = 2000 * 1000;

  epd_wait_busy_async(&epd);
}

void SetCursor(Epaper &epd, uint16_t x, uint16_t y)
{
    write_command(epd, 0x4e);
    write_data(epd, (x >> 3) & 0xff);

    write_command(epd, 0x4f);
    write_data(epd, y & 0xFF);
    write_data(epd, (y >> 8) & 0x01);
}

void SetViewport(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint8_t data44[] = {(uint8_t)(x / 8), (uint8_t)((x + w - 1) / 8)};
  write_command_and_data(epd, 0x44, data44, sizeof(data44)); // set Ram-X address start/end position

  uint16_t beginY = y;
  uint16_t endY = y + h;

  uint8_t data45[] = {(uint8_t)(beginY & 255), (uint8_t)(beginY >> 8), (uint8_t)(endY & 255), (uint8_t)(endY >> 8)};
  write_command_and_data(epd, 0x45, data45, sizeof(data45)); // set Ram-Y address start/end position

  DataEntryMode dataEntryMode = {1, 1, 0};
  write_command_and_data(epd, 0x11, (uint8_t*)&dataEntryMode, sizeof(dataEntryMode)); // data entry mode

}

static void FillBox8_Internal(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t innerCol, uint8_t outerCol)
{
  //SetViewport(epd, x, y, w, h);
  
  const int32_t writeCount = w * h / 8;

  uint16_t endX = (x + w);
  uint16_t endInnerX = (x + w) & ~7;

  uint8_t firstVal;
  uint8_t lastVal;

  if (x % 8)
  {
    uint8_t count = 8 - (x % 8);
    uint8_t mask = 0xff >> (8 - count);
    uint8_t invMask = mask ^ 0xff;
    firstVal = (mask & innerCol) | (invMask & outerCol);
  }
    
  if (endX % 8)
  {
    uint8_t count = (endX % 8);
    uint8_t mask = 0xff << (8 - count);
    uint8_t invMask = mask ^ 0xff;
    lastVal = (mask & innerCol) | (invMask & outerCol);
  }

  for (int yi = 0 ; yi < h ; yi++)
  {
    SetCursor(epd, x, y + yi);
    write_command(epd, GetRTCmd(epd));
    int xi = x;

    if (xi % 8)
    {
      write_data(epd, firstVal);
      xi = (xi + 7) & ~7;
    }
    
    for ( ; xi < endInnerX ; xi += 8)
    {
      write_data(epd, innerCol);
    }

    if (endX % 8)
    {
      write_data(epd, lastVal);
    }
  }

  //write_command(epd, 0x26);
//
  //for (int i = 0 ; i < writeCount ; i++)
  //{
  //    write_data(epd, innerCol);
  //}

}

void FillBox8(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t innerCol, uint8_t outerCol)
{
  FillBox8_Internal(epd, x, y, w, h, innerCol, outerCol);

  bool b = epd_is_busy(epd);

  epd_wait_busy(epd);
}

void FillBox8(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t innerCol, uint8_t outerCol, EpdCallback cb, uint32_t cookie)
{
  FillBox8_Internal(epd, x, y, w, h, innerCol, outerCol);

  epd_wait_busy_async(epd, cb, cookie);
}


void FillBox(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t colBit)
{
  const uint8_t innerVal = 0xff + (1 - colBit);
  const uint8_t outerVal = innerVal ^ 0xff;

  FillBox8(epd, x, y, w, h, innerVal, outerVal);
}

void ClearEpd(Epaper &epd, uint8_t colBit)
{
  FillBox(epd, 0, 0, 122, 250, colBit);
}

bool RefreshEdp(Epaper &epd)
{
  static uint8_t data22[] = {0xf7};
  write_command_and_data(epd, 0x22, data22, sizeof(data22)); //  Display Update Control

  write_command(epd, 0x20); // Activate Display Update Sequence

  return epd_wait_busy(epd) == 0;
}

void RefreshEdp(Epaper &epd, EpdCallback cb, uint32_t cookie)
{
  static uint8_t data22[] = {0xf7};
  write_command_and_data(epd, 0x22, data22, sizeof(data22)); //  Display Update Control

  write_command(epd, 0x20); // Activate Display Update Sequence

  epd_wait_busy_async(epd, cb, cookie);
}

bool RefreshPartialEdp(Epaper &epd)
{
  
  //SetCursor(epd, 0, 0);

  static uint8_t data22[] = {0xCC};
  write_command_and_data(epd, 0x22, data22, sizeof(data22)); //  Display Update Control

  write_command(epd, 0x20); // Activate Display Update Sequence

  return epd_wait_busy(epd) == 0;
}

struct Font
{
  const uint8_t *m_data;
  uint8_t m_begin;
  uint8_t m_end;

  uint8_t m_charSize;

  uint8_t m_width;
  uint8_t m_height;
};

const Font font8x8 = {
  (uint8_t*)font8x8_basic,
  0,
  127,
  8,
  8,
  8
};

void WriteChar(Epaper &epd, uint32_t x, uint32_t y, uint8_t c, uint8_t col, int32_t rotate)
{
  uint8_t rotated[8] = {};

  const uint8_t *font = (uint8_t*)font8x8_basic[c];

  if (rotate)
  {
    for (int i = 0;  i < 8 ; ++i)
    {
      uint8_t d = font[i];

      for (int j = 0 ; j < 8 ; ++j)
      {
        uint8_t bit = (d >> j) & 1;

        rotated[j] |= bit << i;
      }
    }

    font = rotated;
  }

  for (int i = 0;  i < 8 ; ++i)
  {
    SetCursor(epd, x, y+i);

    write_command(epd, GetRTCmd(epd));

    uint8_t c1 = font[i];

    if (col == 0)
      c1 = c1 ^ 0xff;

    write_data(epd, c1);
  }
}

void WriteString(Epaper &epd, uint32_t x, uint32_t y, const char *string, uint8_t col, int32_t rotate)
{
  while(string && string[0])
  {
    WriteChar(epd, x, y, string[0], col, rotate);

    if (rotate)
      y += 8;
    else
      x += 8;

    string++;
  }
}

void WriteImage(Epaper &epd, uint8_t xDiv8, uint8_t y, uint8_t wDiv8, uint8_t h, const void *data, uint16_t rowPitch)
{
  const uint8_t *it = (uint8_t*)data;

  for (int yi = 0 ; yi < h ; yi++)
  {
    SetCursor(epd, xDiv8 << 3, y + yi);
    write_command(epd, GetRTCmd(epd));

    write_data_begin(epd);
    
    for (int xi = 0 ; xi < wDiv8 ; xi++)
    {
      write_data(it[xi]);
    }

    it += rowPitch;

    write_data_end(epd);
  }
}

void SleepEpd(Epaper &epd)
{
  static uint8_t data10[] = {0x1};    // 0x1 == sleep mode 1
  write_command_and_data(epd, 0x10, data10, sizeof(data10)); // Deep sleep mode
}

void EpdReadBuffer(Epaper &epd)
{
  SetCursor(epd, 0, 0);
  write_command(epd, 0x41);
  write_data(epd, 0x0);

  //SetupPin(epd.m_pins.m_data, true, 0);

  write_command(epd, 0x27);
  uint8_t b;
  for (int i = 0 ; i < 128 ; ++i)
  {
    spi_read(&b, 1);
  }
  
  //SetupPin(epd.m_pins.m_data, false, 0);
}

void SetCurrentRT(Epaper &epd, uint8_t rt)
{
  epd.m_rt = rt;
}

bool epdInit = false;

void InitEpdSpi(int clk, int data)
{
  if (epdInit == false)
  {
    epdInit = true;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.mosi_pin = data;
    spi_config.sck_pin  = clk;
    spi_config.frequency  = NRF_DRV_SPI_FREQ_8M;
    spi_config.frequency  = NRF_DRV_SPI_FREQ_125K;
    //APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, nullptr));    
  }

}

void TermEpdSpi()
{
  if (epdInit == true)
  {
    epdInit = false;
    nrf_drv_spi_uninit(&spi);    
  }

}

static const unsigned char ut_partial[] =
{
  0x0, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0A, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2,
  0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
};

bool InitPartialEpd(Epaper &epd)
{
  InitEpd(epd);

  write_command(epd, 0x32);
  write_data_begin(epd);
  //pd_cs_reset();
  for (int j = 0; j < sizeof(ut_partial); j++)
  {
    write_data(ut_partial[j]);
  }
  write_data_end(epd);
  return epd_wait_busy(epd) == 0;
}

void ResetEpd(Epaper &epd, bool wait)
{
  //HW reset
  WritePin(epd.m_pins.m_reset, 0);
  Delay(100);
  WritePin(epd.m_pins.m_reset, 1);
  Delay(100);

  if (wait)
    epd_wait_busy(epd);
}

bool InitEpd(Epaper &epd)
{  
  if (epd_wait_busy(epd) != 0)  
    return false;

  write_command(epd, 0x12);
  Delay(10);

  if (epd_wait_busy(epd) != 0)  
    return false;

  static uint8_t data01[] = {0x27, 0x01, 0x00};
  write_command_and_data(epd, 0x01, data01, sizeof(data01)); // Driver output control

  static DataEntryMode dataEntryMode = {1, 1, 0};
  write_command_and_data(epd, 0x11, (uint8_t*)&dataEntryMode, sizeof(dataEntryMode)); // data entry mode

  SetViewport(epd, 0, 0, 122, 250);

  write_command_and_arg(epd, 0x3C, 0x05); // BorderWavefrom
  
  write_command_and_args(epd, 0x21, 0x00, 0x80); //  Display update control

  write_command_and_arg(epd, 0x18, 0x80); // Read built-in temperature sensor

  SetCursor(epd, 0, 0);
  
  if (epd_wait_busy(epd) != 0)  
    return false;

  epd.m_rt = 0;

  return true;
}

