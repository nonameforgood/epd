#include "gj/base.h"

struct Epaper;

typedef void (*EpdCallback)(Epaper *epd, uint32_t cookie, bool success);


struct Epaper
{
  struct
  {
    uint8_t m_clk;
    uint8_t m_dc;
    uint8_t m_cs;
    uint8_t m_data;
    uint8_t m_busy;
    uint8_t m_reset;
  } m_pins;

  uint16_t m_width;
  uint16_t m_height;

  struct
  {
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_width;
    uint16_t m_height;
  } m_viewport;

  EpdCallback m_cb;
  uint32_t m_cookie;
  uint32_t m_waitCount;
  uint32_t m_cbDelay = 100;

  uint8_t m_rt;
};

struct DisplayUpdateControl
{
  uint8_t RedRamOptions : 4;
  uint8_t BWRamOptions : 4;
  uint8_t _pad : 7;
  uint8_t SourceOutputMode : 1;
};

struct DataEntryMode
{
  uint8_t xIncrement : 1;
  uint8_t yIncrement : 1;
  uint8_t am : 1;
};

void SetCursor(Epaper &epd, uint16_t x, uint16_t y);
void SetViewport(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

void FillBox(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t colBit);
void FillBox8(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t innerCol, uint8_t outerCol);

void FillBox8(Epaper &epd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t innerCol, uint8_t outerCol, EpdCallback cb, uint32_t cookie);

void ClearEpd(Epaper &epd, uint8_t colBit);

void WriteChar(Epaper &epd, uint32_t x, uint32_t y, uint8_t c, uint8_t col, int32_t rotate);
void WriteString(Epaper &epd, uint32_t x, uint32_t y, const char *string, uint8_t col, int32_t rotate);
void WriteString(Epaper &epd, uint32_t x, uint32_t y, const char *string, uint8_t col, int32_t rotate, EpdCallback cb, uint32_t cookie);

void WriteImage(Epaper &epd, uint8_t xDiv8, uint8_t y, uint8_t wDiv8, uint8_t h, const void *data, uint16_t rowPitch);



bool RefreshEdp(Epaper &epd);
void RefreshEdp(Epaper &epd, EpdCallback cb, uint32_t cookie);
bool RefreshPartialEdp(Epaper &epd);

void SleepEpd(Epaper &epd);

#define EPD_RT_MAIN 0
#define EPD_RT_PREV 1
void SetCurrentRT(Epaper &epd, uint8_t rt);

void InitEpdSpi(int clk, int data);
void TermEpdSpi();

void ResetEpd(Epaper &epd, bool wait = true);
bool InitEpd(Epaper &epd);
bool InitPartialEpd(Epaper &epd);