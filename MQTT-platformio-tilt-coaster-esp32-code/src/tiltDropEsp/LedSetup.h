// === LED Config (Onveranderd) ===
#define NUM_LEDS_TILT 3
#define LEDS_TILT_PIN 13
CRGB tiltLeds[NUM_LEDS_TILT];
unsigned long previousMillis = 0;
unsigned long effectInterval = 50;
CRGB colors[3] = {CRGB::Red, CRGB::Purple, CRGB::Green};
int colorIndex[NUM_LEDS_TILT] = {0, 1, 2};
int brightness[NUM_LEDS_TILT] = {255, 200, 150};
int step[NUM_LEDS_TILT] = {15, 10, 20};

CRGB currentColor = CRGB::Blue;
CRGB targetColor = CRGB::Blue;
int fadeBrightness = 50;
bool fadeIncreasing = true;
unsigned long lastFadeMillis = 0;
int fadeInterval = 30;

unsigned long flashStartMillis = 0;
int flashCount = 0;
bool flashing = false;

int redBrightness = 50;
bool redIncreasing = true;



// === LED HELPERS (Onveranderd) ===
void setAllLeds(CRGB color, int brightnessVal)
{
    for (int i = 0; i < NUM_LEDS_TILT; i++)
    {
        tiltLeds[i] = color;
        tiltLeds[i].nscale8_video(brightnessVal);
    }
    FastLED.show();
}

void SetTargetColor(CRGB newColor) { targetColor = newColor; }

void UpdateLedFade()
{
    unsigned long now = millis();
    if (now - lastFadeMillis < fadeInterval)
        return;
    lastFadeMillis = now;

    currentColor.r += (targetColor.r - currentColor.r) / 5;
    currentColor.g += (targetColor.g - currentColor.g) / 5;
    currentColor.b += (targetColor.b - currentColor.b) / 5;

    if (fadeIncreasing)
        fadeBrightness += 5;
    else
        fadeBrightness -= 5;
    if (fadeBrightness >= 150)
        fadeIncreasing = false;
    if (fadeBrightness <= 50)
        fadeIncreasing = true;

    setAllLeds(currentColor, fadeBrightness);
}

bool LedWhiteFlash(int numFlashes, int intervalMs)
{
    unsigned long now = millis();
    static bool localFlashing = false;
    static int localFlashCount = 0;
    static unsigned long localStart = 0;

    if (!localFlashing)
    {
        localStart = now;
        localFlashCount = 0;
        localFlashing = true;
    }

    if (now - localStart >= intervalMs)
    {
        localStart = now;
        localFlashCount++;
        if (localFlashCount % 2 == 1)
            setAllLeds(CRGB::White, 255);
        else
            setAllLeds(CRGB::Black, 0);
        if (localFlashCount >= numFlashes * 2)
        {
            localFlashing = false;
            return true;
        }
    }
    return false;
}

void LedRedPulseFast()
{
    unsigned long now = millis();
    if (now - lastFadeMillis < 30)
        return;
    lastFadeMillis = now;

    if (redIncreasing)
        redBrightness += 20;
    else
        redBrightness -= 20;
    if (redBrightness >= 255)
        redIncreasing = false;
    if (redBrightness <= 50)
        redIncreasing = true;

    setAllLeds(CRGB::Red, redBrightness);
}

