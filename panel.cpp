#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "TouchDrvCSTXXX.hpp"
#include "panel.h"

// 腳位來自 Waveshare 官方 HARDWARE_REFERENCE / pin_config.h
#define LCD_CS    12
#define LCD_SCLK  38
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 39
#define IIC_SDA   15
#define IIC_SCL   14
#define TP_INT    11
#define TP_RESET  40

static Arduino_DataBus *bus = nullptr;
static Arduino_CO5300 *gfx = nullptr;

bool panelBegin() {
  bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
  // 466x466 的 CO5300 需要 column offset 6（官方範例同樣設定）
  gfx = new Arduino_CO5300(bus, LCD_RESET, 0, PANEL_W, PANEL_H, 6, 0, 0, 0);
  if (!gfx->begin()) return false;
  gfx->fillScreen(RGB565_BLACK);
  // 初始化序列結束時亮度暫存器是 0xD0（約 80%）；這裡明確寫成最亮，
  // 並在醒來的穩定時間之後再寫一次，避免第一次寫入被面板吃掉
  gfx->setBrightness(255);
  delay(120);
  gfx->setBrightness(255);
  return true;
}

void panelBrightness(uint8_t v) {
  if (gfx) gfx->setBrightness(v);
}

void panelPush(int x, int y, int w, int h, uint16_t *rgb565) {
  if (gfx) gfx->draw16bitRGBBitmap(x, y, rgb565, w, h);
}

// ---------------- 觸控 ----------------
static TouchDrvCST92xx touch;
static QueueHandle_t touchQ = nullptr;

static void touchTask(void *) {
  bool down = false;
  int16_t lx = 0, ly = 0, sx = 0, sy = 0;
  uint8_t misses = 0;
  for (;;) {
    int16_t x[1], y[1];
    uint8_t n = touch.getPoint(x, y, 1);
    TouchEvt e;
    if (n > 0) {
      misses = 0;
      lx = x[0];
      ly = y[0];
      if (!down) {
        down = true;
        sx = lx;
        sy = ly;
        e = { 0, lx, ly };
        xQueueSend(touchQ, &e, 0);
      } else if (abs(lx - sx) >= 2 || abs(ly - sy) >= 2) {
        sx = lx;
        sy = ly;
        e = { 1, lx, ly };
        xQueueSend(touchQ, &e, 0);   // 佇列滿了就丟掉這筆移動，無所謂
      }
    } else if (down && ++misses >= 3) {   // 連續 3 次（約 30ms）沒偵測到才算放開，避免抖動
      down = false;
      e = { 2, lx, ly };
      xQueueSend(touchQ, &e, pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool panelTouchBegin() {
  pinMode(TP_RESET, OUTPUT);
  digitalWrite(TP_RESET, LOW);
  delay(30);
  digitalWrite(TP_RESET, HIGH);
  delay(50);
  Wire.begin(IIC_SDA, IIC_SCL);
  touch.setPins(TP_RESET, TP_INT);
  if (!touch.begin(Wire, 0x5A, IIC_SDA, IIC_SCL)) return false;
  // 跟官方 LVGL 範例相同的設定：座標 0~465、XY 鏡像，這樣座標才會跟畫面對上
  touch.sleep();
  touch.reset();
  touch.setMaxCoordinates(PANEL_W, PANEL_H);
  touch.setMirrorXY(true, true);
  touchQ = xQueueCreate(32, sizeof(TouchEvt));
  xTaskCreatePinnedToCore(touchTask, "touch", 4096, nullptr, 3, nullptr, 1);
  return true;
}

bool panelTouchGet(TouchEvt &e) {
  return touchQ && xQueueReceive(touchQ, &e, 0) == pdTRUE;
}
