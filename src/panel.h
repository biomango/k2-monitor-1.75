// AMOLED 面板（Waveshare ESP32-S3-Touch-AMOLED-1.75：CO5300，QSPI，466x466 圓形）
// 刻意把 Arduino_GFX 隔離在 panel.cpp，避免它的巨集跟 TFT_eSPI 打架。
#pragma once
#include <stdint.h>

#define PANEL_W 466
#define PANEL_H 466

bool panelBegin();                                              // 初始化並清成全黑
void panelBrightness(uint8_t v);                                // 0~255
void panelPush(int x, int y, int w, int h, uint16_t *rgb565);   // 本機位元組序的 RGB565
