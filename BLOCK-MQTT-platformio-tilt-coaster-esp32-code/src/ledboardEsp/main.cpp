#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "../shared/env.h"

/* ===== CONFIGURATIE ===== */
#define LED_PIN_TOP    23  
#define LED_PIN_MID    22  
#define LED_PIN_BTM    21  

#define WIDTH          118
#define HEIGHT_TOP     10
#define HEIGHT_MID     9
#define HEIGHT_BTM     8
#define TOTAL_HEIGHT   (HEIGHT_TOP + HEIGHT_MID + HEIGHT_BTM)

#define NUM_LEDS_TOP   (WIDTH * HEIGHT_TOP)
#define NUM_LEDS_MID   (WIDTH * HEIGHT_MID)
#define NUM_LEDS_BTM   (WIDTH * HEIGHT_BTM)
#define TOTAL_LEDS     (NUM_LEDS_TOP + NUM_LEDS_MID + NUM_LEDS_BTM)

CRGB leds[TOTAL_LEDS];

// Toestanden voor de demo
enum CoasterState { STATION, LIFTHILL, LAUNCH, SPEED };
CoasterState currentState = STATION;

/* ===== 5x7 FONT DATA ===== */
const uint8_t Font5x7[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14, 
    0x24, 0x2a, 0x7f, 0x2a, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 
    0x00, 0x1c, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00, 0x14, 0x08, 0x3e, 0x08, 0x14, 0x08, 0x08, 0x3e, 0x08, 0x08, 
    0x00, 0x50, 0x30, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 
    0x3e, 0x51, 0x49, 0x45, 0x3e, 0x00, 0x42, 0x7f, 0x40, 0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4b, 0x31, 
    0x18, 0x14, 0x12, 0x7f, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39, 0x3c, 0x4a, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03, 
    0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1e, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x56, 0x36, 0x00, 0x00, 
    0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 
    0x32, 0x49, 0x79, 0x41, 0x3e, 0x7e, 0x11, 0x11, 0x11, 0x7e, 0x7f, 0x49, 0x49, 0x49, 0x36, 0x3e, 0x41, 0x41, 0x41, 0x22, 
    0x7f, 0x41, 0x41, 0x22, 0x1c, 0x7f, 0x49, 0x49, 0x49, 0x41, 0x7f, 0x09, 0x09, 0x09, 0x01, 0x3e, 0x41, 0x49, 0x49, 0x7a, 
    0x7f, 0x08, 0x08, 0x08, 0x7f, 0x00, 0x41, 0x7f, 0x41, 0x00, 0x20, 0x40, 0x41, 0x3f, 0x01, 0x7f, 0x08, 0x14, 0x22, 0x41, 
    0x7f, 0x40, 0x40, 0x40, 0x40, 0x7f, 0x02, 0x0c, 0x02, 0x7f, 0x7f, 0x04, 0x08, 0x10, 0x7f, 0x3e, 0x41, 0x41, 0x41, 0x3e, 
    0x7f, 0x09, 0x09, 0x09, 0x06, 0x3e, 0x41, 0x51, 0x21, 0x5e, 0x7f, 0x09, 0x19, 0x29, 0x46, 0x46, 0x49, 0x49, 0x49, 0x31, 
    0x01, 0x01, 0x7f, 0x01, 0x01, 0x3f, 0x40, 0x40, 0x40, 0x3f, 0x1f, 0x20, 0x40, 0x20, 0x1f, 0x3f, 0x40, 0x38, 0x40, 0x3f, 
    0x63, 0x14, 0x08, 0x14, 0x63, 0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43
};

/* ===== MAPPING & DRAWING ===== */

uint16_t getZigzagIndex(uint8_t x, uint8_t y, uint8_t w) {
    if (y % 2 == 0) return (y * w) + x;
    else return (y * w) + (w - 1 - x);
}

void setPixel(int x, int y, CRGB color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= TOTAL_HEIGHT) return;
    uint16_t targetIndex = 0;
    if (y < HEIGHT_TOP) {
        targetIndex = getZigzagIndex(x, y, WIDTH);
    } else if (y < (HEIGHT_TOP + HEIGHT_MID)) {
        uint8_t localY = y - HEIGHT_TOP;
        targetIndex = NUM_LEDS_TOP + getZigzagIndex(x, localY, WIDTH);
    } else {
        uint8_t localY = y - (HEIGHT_TOP + HEIGHT_MID);
        targetIndex = NUM_LEDS_TOP + NUM_LEDS_MID + getZigzagIndex(x, localY, WIDTH);
    }
    leds[targetIndex] = color;
}

void drawChar(int x, int y, char c, CRGB color) {
    if (c < 32 || c > 126) return;
    uint8_t index = c - 32;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = pgm_read_byte(&Font5x7[index * 5 + i]);
        for (uint8_t j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                setPixel(x + i, y + j, color);
            }
        }
    }
}

void drawText(int x, int y, String text, CRGB color) {
    for (int i = 0; i < text.length(); i++) {
        drawChar(x + (i * 6), y, text[i], color);
    }
}

/* ===== ANIMATIES ===== */

// 1. STATION: Safety scan
void animationStation() {
    static int scanX = 0;
    drawText(15, 10, "SYSTEM CHECK...", CRGB::DeepSkyBlue);
    for(int y = 0; y < TOTAL_HEIGHT; y++) {
        setPixel(scanX, y, CRGB::Lime);
        setPixel(scanX-1, y, CRGB::Green); 
    }
    scanX = (scanX + 1) % WIDTH;
}

// 2. LIFTHILL: Werkende Chevrons (V-vorm)
void animationLifthill() {
    static int scrollY = 0;
    fadeToBlackBy(leds, TOTAL_LEDS, 100); 
    for (int x = 0; x < WIDTH; x += 20) {
        for (int i = 0; i < 8; i++) {
            int yPos = (scrollY + i) % TOTAL_HEIGHT;
            setPixel(x + i, yPos, CRGB::Yellow);
            setPixel(x + 16 - i, yPos, CRGB::Yellow);
        }
    }
    scrollY = (scrollY + 1) % TOTAL_HEIGHT;
}

// 3. LAUNCH: Countdown
void animationLaunch() {
    static unsigned long timer = 0;
    static int count = 3;
    if (millis() - timer > 1000) {
        count--;
        if (count < 0) count = 3;
        timer = millis();
    }
    if (count > 0) {
        drawText(55, 10, String(count), CRGB::Red);
    } else {
        fill_solid(leds, TOTAL_LEDS, CRGB::White);
        drawText(50, 10, "GO!", CRGB::Black);
    }
}

// 4. SPEED: Snelle diagonale lijnen
void animationSpeed() {
    static int offset = 0;
    fadeToBlackBy(leds, TOTAL_LEDS, 64);
    for (int x = 0; x < WIDTH; x += 10) {
        int y = (x + offset) % TOTAL_HEIGHT;
        for(int beam = 0; beam < 3; beam++) {
            setPixel(x + beam, y, CHSV(160, 200, 255)); 
        }
    }
    offset++;
}

/* ===== MAIN LOOP ===== */

void setup() {
    FastLED.addLeds<WS2812B, LED_PIN_TOP, GRB>(leds, NUM_LEDS_TOP);
    FastLED.addLeds<WS2812B, LED_PIN_MID, GRB>(leds + NUM_LEDS_TOP, NUM_LEDS_MID);
    FastLED.addLeds<WS2812B, LED_PIN_BTM, GRB>(leds + NUM_LEDS_TOP + NUM_LEDS_MID, NUM_LEDS_BTM);
    
    FastLED.setBrightness(50);
    FastLED.clear();
}

void loop() {
    static unsigned long lastFrame = 0;
    if (millis() - lastFrame < 30) return;
    lastFrame = millis();

    FastLED.clear();

    switch (currentState) {
        case STATION:  animationStation();  break;
        case LIFTHILL: animationLifthill(); break;
        case LAUNCH:   animationLaunch();   break;
        case SPEED:    animationSpeed();    break;
    }

    FastLED.show();

    // DEMO SWITCH: Elke 6 seconden naar de volgende fase
    static unsigned long lastSwitch = 0;
    if (millis() - lastSwitch > 6000) {
        currentState = (CoasterState)((currentState + 1) % 4);
        lastSwitch = millis();
    }
}