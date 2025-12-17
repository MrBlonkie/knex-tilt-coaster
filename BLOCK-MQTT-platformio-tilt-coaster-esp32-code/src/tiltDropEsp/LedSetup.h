// === LED Config ===
#define NUM_LEDS_TILT 3
#define LEDS_TILT_PIN 13

#define NUM_LEDS_DROP 8
#define LEDS_DROP_PIN 12

#define NUM_LEDS_TOWER 52
#define LEDS_TOWER_PIN 14

CRGB tiltLeds[NUM_LEDS_TILT];
CRGB dropLeds[NUM_LEDS_DROP];
CRGB towerLeds[NUM_LEDS_TOWER];

// === TILT VARIABLES (Blauw ademen) ===
CRGB tiltCurrentColor = CRGB::Blue;
CRGB tiltTargetColor = CRGB::Blue;
int tiltFadeBrightness = 50;
bool tiltFadeIncreasing = true;
unsigned long tiltLastFadeMillis = 0;

// === DROP VARIABLES (Groen ademen + Flash) ===
CRGB dropCurrentColor = CRGB::Green;
int dropFadeBrightness = 50;
bool dropFadeIncreasing = true;
unsigned long dropLastFadeMillis = 0;
// Flash vars
bool dropIsFlashing = false;
unsigned long dropLastFlashMillis = 0;
int dropFlashState = 0; // 0=Red, 1=White

// === TOWER VARIABLES ===
unsigned long towerLastUpdate = 0;
int towerAnimPos = 0;
int towerHue = 0;

// ==========================================
//               TILT LOGIC
// ==========================================

void setTiltLeds(CRGB color, int brightnessVal)
{
    for (int i = 0; i < NUM_LEDS_TILT; i++)
    {
        tiltLeds[i] = color;
        tiltLeds[i].nscale8_video(brightnessVal);
    }
}

// Standaard Idle fade voor Tilt (Blauw)
void UpdateTiltIdle()
{
    unsigned long now = millis();
    if (now - tiltLastFadeMillis < 30) return;
    tiltLastFadeMillis = now;

    // Fade logic
    if (tiltFadeIncreasing) tiltFadeBrightness += 5;
    else tiltFadeBrightness -= 5;
    
    if (tiltFadeBrightness >= 200) tiltFadeIncreasing = false;
    if (tiltFadeBrightness <= 50) tiltFadeIncreasing = true;

    setTiltLeds(CRGB::Blue, tiltFadeBrightness);
}

// Witte flash (bijvoorbeeld bij beweging)
bool UpdateTiltFlash(int numFlashes, int intervalMs)
{
    static int flashCount = 0;
    static unsigned long lastFlash = 0;
    static bool isActive = false;

    if (!isActive) { isActive = true; flashCount = 0; lastFlash = millis(); }
    
    if (millis() - lastFlash >= intervalMs) {
        lastFlash = millis();
        flashCount++;
        
        if (flashCount % 2 == 1) setTiltLeds(CRGB::White, 255);
        else setTiltLeds(CRGB::Black, 0);

        if (flashCount >= numFlashes * 2) {
            isActive = false;
            return true; // Done
        }
    }
    return false; // Still busy
}

void UpdateTiltRedPulse()
{
    unsigned long now = millis();
    if (now - tiltLastFadeMillis < 20) return;
    tiltLastFadeMillis = now;

    static int redBri = 50;
    static bool redUp = true;

    if (redUp) redBri += 10; else redBri -= 10;
    if (redBri >= 255) redUp = false;
    if (redBri <= 50) redUp = true;

    setTiltLeds(CRGB::Red, redBri);
}


// ==========================================
//               DROP LOGIC
// ==========================================
// Idle = Groen ademen
// Triggered (offTiltdrop) = Rood/Wit flashen

void UpdateDropLeds(bool isTriggered)
{
    unsigned long now = millis();

    if (isTriggered)
    {
        // Rood / Wit Flash Alarm
        if (now - dropLastFlashMillis > 100) // Snel flashen
        {
            dropLastFlashMillis = now;
            dropFlashState = !dropFlashState;
            
            CRGB flashColor = (dropFlashState == 0) ? CRGB::Red : CRGB::White;
            
            for(int i=0; i<NUM_LEDS_DROP; i++) {
                dropLeds[i] = flashColor;
            }
        }
    }
    else
    {
        // Idle: Groen Ademen (Zelfde logic als Tilt, aparte vars)
        if (now - dropLastFadeMillis > 30)
        {
            dropLastFadeMillis = now;
            if (dropFadeIncreasing) dropFadeBrightness += 5;
            else dropFadeBrightness -= 5;

            if (dropFadeBrightness >= 200) dropFadeIncreasing = false;
            if (dropFadeBrightness <= 50) dropFadeIncreasing = true;

            for(int i=0; i<NUM_LEDS_DROP; i++) {
                dropLeds[i] = CRGB::Green;
                dropLeds[i].nscale8_video(dropFadeBrightness);
            }
        }
    }
}


// ==========================================
//               TOWER LOGIC
// ==========================================
// Idle = Rustige regenboog / kleur
// Lifthill = Animatie omhoog

void UpdateTowerLeds(bool isLifthillActive)
{
    unsigned long now = millis();

    if (isLifthillActive)
    {
        // === LIFTHILL ANIMATIE ===
        // Een oranje "ketting" die omhoog loopt
        if (now - towerLastUpdate > 30) // Snelheid van animatie
        {
            towerLastUpdate = now;
            towerAnimPos++;
            if (towerAnimPos >= NUM_LEDS_TOWER) towerAnimPos = 0;

            // Fade alles een beetje uit (trail effect)
            fadeToBlackBy(towerLeds, NUM_LEDS_TOWER, 60);

            // Zet nieuwe pixel aan
            towerLeds[towerAnimPos] = CRGB::Orange;
            // Maak hem fel
            towerLeds[towerAnimPos].maximizeBrightness();
        }
    }
    else
    {
        // === IDLE ANIMATIE ===
        // Rustige regenboog golf
        fill_solid(towerLeds, NUM_LEDS_TOWER, CRGB::Black);

    }
}