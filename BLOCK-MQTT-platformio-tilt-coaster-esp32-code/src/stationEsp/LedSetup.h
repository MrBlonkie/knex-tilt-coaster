// === LED Config === 
#define NUM_LEDS_EXIT 8 
#define LEDS_EXIT_PIN 21 
CRGB exitLeds[NUM_LEDS_EXIT];
unsigned long lastLedUpdate = 0;
float ledPhase = 0;
// === LED Helpers ===
bool fadeIncreasing[NUM_LEDS_EXIT];
uint8_t fadeBrightness[NUM_LEDS_EXIT];
unsigned long lastFadeMillis = 0;
const unsigned long fadeInterval = 20; // snelheid fade

void setupLedFadeVars() {
    for(int i = 0; i < NUM_LEDS_EXIT; i++){
        fadeIncreasing[i] = random(0,2); // true/false
        fadeBrightness[i] = random(0,255); // volledige range
    }
}

void setAllLeds(CRGB color, int brightnessVal)
{
    for (int i = 0; i < NUM_LEDS_EXIT; i++)
    {
        exitLeds[i] = color;
        exitLeds[i].nscale8_video(brightnessVal);
    }
    FastLED.show();
}

// === Idle mode: blauw, zachte onafhankelijke breathing ===
void UpdateLedFadeIdle()
{
    unsigned long now = millis();
    if(now - lastFadeMillis < fadeInterval) return;
    lastFadeMillis = now;

    for(int i = 0; i < NUM_LEDS_EXIT; i++){
        // subtiele aan/uit beweging
        if(fadeIncreasing[i]) fadeBrightness[i] += 1;
        else fadeBrightness[i] -= 1;

        if(fadeBrightness[i] >= 125) fadeIncreasing[i] = false;
        if(fadeBrightness[i] <= 35) fadeIncreasing[i] = true;

        exitLeds[i] = CRGB::Blue;
        exitLeds[i].nscale8_video(fadeBrightness[i]);
    }

    FastLED.show();
}

// === Motor draait: fade-achtige beweging, iets intenser/sneller ===
void UpdateLedFadeMotor()
{
    unsigned long now = millis();
    if(now - lastFadeMillis < fadeInterval/2) return; // sneller dan idle
    lastFadeMillis = now;

    for(int i = 0; i < NUM_LEDS_EXIT; i++){
        // intensere fade range
        if(fadeIncreasing[i]) fadeBrightness[i] += 2; // sneller
        else fadeBrightness[i] -= 2;

        if(fadeBrightness[i] >= 180) fadeIncreasing[i] = false; // max helderheid hoger
        if(fadeBrightness[i] <= 70) fadeIncreasing[i] = true;   // min iets hoger

        exitLeds[i] = CRGB::Blue; // of kies gradient met meerdere kleuren
        exitLeds[i].nscale8_video(fadeBrightness[i]);
    }

    FastLED.show();
}

