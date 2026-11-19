// AMOLED 面板（Waveshare ESP32-S3-Touch-AMOLED-1.75：CO5300，QSPI，466x466 圓形）＋ 觸控（CST9217，I2C）
// 刻意把 Arduino_GFX / SensorLib 隔離在 panel.cpp，避免它們的巨集跟 TFT_eSPI 打架。
#pragma once
#include <stdint.h>

#define PANEL_W 466
#define PANEL_H 466

bool panelBegin();                                              // 初始化並清成全黑，亮度設為最亮
void panelBrightness(uint8_t v);                                // 0~255
void panelPush(int x, int y, int w, int h, uint16_t *rgb565);   // 本機位元組序的 RGB565

// ---- 觸控：背景工作會以約 100Hz 取樣，產生 按下 / 移動 / 放開 事件放進佇列 ----
struct TouchEvt {
  uint8_t type;      // 0 = 按下, 1 = 移動, 2 = 放開
  int16_t x, y;      // 面板實體座標 0~465
};
bool panelTouchBegin();
bool panelTouchGet(TouchEvt &e);                                // 取出一筆事件；沒有就回傳 false
