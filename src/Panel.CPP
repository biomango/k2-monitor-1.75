#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "panel.h"

// 腳位來自 Waveshare 官方 HARDWARE_REFERENCE / pin_config.h
#define LCD_CS    12
#define LCD_SCLK  38
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 39

static Arduino_DataBus *bus = nullptr;
static Arduino_CO5300 *gfx = nullptr;

bool panelBegin() {
  bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
  // 466x466 的 CO5300 需要 column offset 6（官方範例同樣設定）
  gfx = new Arduino_CO5300(bus, LCD_RESET, 0, PANEL_W, PANEL_H, 6, 0, 0, 0);
  if (!gfx->begin()) return false;
  gfx->fillScreen(RGB565_BLACK);
  return true;
}

void panelBrightness(uint8_t v) {
  if (gfx) gfx->setBrightness(v);
}

void panelPush(int x, int y, int w, int h, uint16_t *rgb565) {
  if (gfx) gfx->draw16bitRGBBitmap(x, y, rgb565, w, h);
}
