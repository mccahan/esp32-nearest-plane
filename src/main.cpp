#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <TAMC_GT911.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "web_server.h"
#include "screenshot.h"
#include <ElegantOTA.h>
#include <math.h>
#include <time.h>
#include <esp_task_wdt.h>
#include "rom/cache.h"

// Custom Space Grotesk fonts (compiled separately)
LV_FONT_DECLARE(space_grotesk_bold_48);
LV_FONT_DECLARE(space_grotesk_bold_24);
LV_FONT_DECLARE(space_grotesk_bold_14);

// Arrow image (compiled separately)
LV_IMG_DECLARE(arrow_130);

// Background image (compiled separately)
LV_IMG_DECLARE(bg_image);

// Optional: include secrets.h for default WiFi credentials
#if __has_include("secrets.h")
#include "secrets.h"
#define HAS_DEFAULT_WIFI 1
#else
#define HAS_DEFAULT_WIFI 0
#endif

// ============================================================================
// USER LOCATION - Stored in Preferences, configured via web interface
// ============================================================================
float userLat = 0;
float userLon = 0;
bool locationConfigured = false;

// Filter setting: hide private/anonymous aircraft (where registration == callsign)
bool hidePrivatePlanes = false;

// Bounding box for API query (degrees around user location)
#define BBOX_RANGE 2.0

// ============================================================================
// PWM BACKLIGHT CONFIGURATION
// ============================================================================


// ============================================================================
// DISPLAY SCHEDULE CONFIGURATION
// ============================================================================

struct ScheduleConfig {
    char timezone[64] = "MST7MDT,M3.2.0,M11.1.0";  // Mountain Time with DST
    bool enabled = false;
    uint8_t daytime_start = 7;     // 7 AM
    uint8_t daytime_end = 21;      // 9 PM
    uint8_t dim_end = 23;          // 11 PM (off is 11PM-7AM)
    uint8_t daytime_brightness = 100;
    uint8_t dim_brightness = 50;   // Testing with lower freq PWM
};

ScheduleConfig scheduleConfig;
enum DisplayMode { MODE_DAYTIME, MODE_DIM, MODE_OFF };
DisplayMode currentDisplayMode = MODE_DAYTIME;
bool ntpSynced = false;
unsigned long lastNtpCheck = 0;
unsigned long lastScheduleCheck = 0;

// Forward declaration for PWM backlight function
void setBacklightBrightness(uint8_t percent);

// ============================================================================
// WATCHDOG TIMER CONFIGURATION
// ============================================================================

const uint32_t WDT_TIMEOUT_SECONDS = 300;  // 5 minutes
bool watchdogEnabled = false;

// ============================================================================
// PIN DEFINITIONS for Guition ESP32-S3-4848S040
// ============================================================================

#define TOUCH_SDA 19
#define TOUCH_SCL 45
#define TOUCH_INT -1
#define TOUCH_RST -1
#define GFX_BL 38
#define TFT_WIDTH 480
#define TFT_HEIGHT 480

// ============================================================================
// DISPLAY HARDWARE CONFIGURATION
// ============================================================================

TAMC_GT911 touchController(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TFT_WIDTH, TFT_HEIGHT);

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    39, 48, 47,
    18, 17, 16, 21,
    11, 12, 13, 14, 0,
    8, 20, 3, 46, 9, 10,
    4, 5, 6, 7, 15
);

Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
    bus, GFX_NOT_DEFINED, 0,
    true, TFT_WIDTH, TFT_HEIGHT,
    st7701_type1_init_operations, sizeof(st7701_type1_init_operations),
    true,
    10, 8, 50,
    10, 8, 20
);

// ============================================================================
// LVGL CONFIGURATION
// ============================================================================

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf1;
static lv_color_t *disp_draw_buf2;
static lv_disp_drv_t disp_drv;
static unsigned long last_tick = 0;
static lv_indev_drv_t indev_drv;
static lv_indev_t *touch_indev = nullptr;

volatile bool simulated_touch_active = false;
volatile int16_t simulated_touch_x = 0;
volatile int16_t simulated_touch_y = 0;
volatile unsigned long simulated_touch_start = 0;
const unsigned long SIMULATED_TOUCH_DURATION = 150;

Preferences wifi_prefs;

// ============================================================================
// SERIAL LOG BUFFER (for remote debugging)
// ============================================================================

#define LOG_BUFFER_SIZE 4096
char logBuffer[LOG_BUFFER_SIZE];
volatile size_t logWritePos = 0;
volatile size_t logReadPos = 0;

void logPrint(const char* msg) {
    Serial.print(msg);
    // Also write to circular buffer
    size_t len = strlen(msg);
    for (size_t i = 0; i < len; i++) {
        logBuffer[logWritePos] = msg[i];
        logWritePos = (logWritePos + 1) % LOG_BUFFER_SIZE;
        if (logWritePos == logReadPos) {
            // Buffer full, advance read pointer
            logReadPos = (logReadPos + 1) % LOG_BUFFER_SIZE;
        }
    }
}

void logPrintf(const char* format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    logPrint(buf);
}

String getLogBuffer() {
    String result;
    result.reserve(LOG_BUFFER_SIZE);
    size_t pos = logReadPos;
    while (pos != logWritePos) {
        result += logBuffer[pos];
        pos = (pos + 1) % LOG_BUFFER_SIZE;
    }
    return result;
}

void clearLogBuffer() {
    logReadPos = logWritePos;
}

// ============================================================================
// FLIGHT DATA STRUCTURES
// ============================================================================

struct AircraftData {
    bool valid = false;
    char callsign[16] = "";
    char icao24[16] = "";
    float latitude = 0;
    float longitude = 0;
    float altitude = 0;       // meters
    float velocity = 0;       // m/s
    float heading = 0;        // degrees (track)
    float distance_miles = 0;
    float bearing = 0;        // degrees from user to aircraft

    // Additional info from adsbdb
    char aircraft_type[32] = "";
    char registration[16] = "";
    char origin[8] = "";
    char destination[8] = "";
};

AircraftData nearestPlane;
AircraftData nearbyPlanes[5];  // Next 5 closest aircraft for minimap
int nearbyPlaneCount = 0;
unsigned long lastApiUpdate = 0;
bool firstApiFetch = true;  // Flag to trigger immediate first fetch
unsigned long lastPositionUpdate = 0;  // Timestamp when position was received (for interpolation)
const unsigned long API_UPDATE_INTERVAL = 30000; // 30 seconds (OpenSky rate limits aggressively)
unsigned long rateLimitBackoff = 0; // Extra delay when rate limited

// Dynamic search radius management
int currentSearchRadiusMiles = 25;    // Current search radius (starts at 25 miles)
const int MIN_SEARCH_RADIUS = 10;     // Minimum 10 miles (avoid empty results)
const int MAX_SEARCH_RADIUS = 100;    // Maximum 100 miles
const int TARGET_AIRCRAFT_COUNT = 6;  // Target number of aircraft to display
const int REDUCE_THRESHOLD = 20;      // Only reduce radius when above this count

// Smooth arrow animation state
float currentArrowAngle = 0;      // Current displayed angle (degrees)
float targetArrowAngle = 0;       // Target angle to animate toward (degrees)
const float ARROW_ROTATION_SPEED = 90.0f;  // Degrees per second
const float MAX_DELTA_MS = 50.0f;          // Cap frame delta to prevent jumps after HTTP calls

// Performance monitoring
unsigned long fpsFrameCount = 0;
unsigned long fpsLastReport = 0;
float currentFps = 0;
const unsigned long FPS_REPORT_INTERVAL = 5000;  // Report every 5 seconds

// Split-flap animation state
char previousCallsign[16] = "";
char displayedCallsign[16] = "";
int splitFlapCharIndex = -1;         // Current character being animated (-1 = not animating)
unsigned long splitFlapStartTime = 0;
const int SPLIT_FLAP_CHAR_DURATION = 80;  // ms per character flip
const int SPLIT_FLAP_CYCLES = 3;          // Number of random chars before settling

// ============================================================================
// UI ELEMENTS
// ============================================================================

static lv_obj_t *ui_screen = nullptr;
static lv_obj_t *ui_callsign = nullptr;
static lv_obj_t *ui_route_badge = nullptr;
static lv_obj_t *ui_route_label = nullptr;
static lv_obj_t *ui_compass_container = nullptr;
static lv_obj_t *ui_arrow = nullptr;
static lv_obj_t *ui_airspeed_value = nullptr;
static lv_obj_t *ui_aircraft_value = nullptr;
static lv_obj_t *ui_distance_label = nullptr;

// Minimap arrows for nearby aircraft
static lv_obj_t *ui_mini_arrows[5] = {nullptr};

// Setup screen elements (shown when location not configured)
static lv_obj_t *ui_setup_screen = nullptr;
static lv_obj_t *ui_qrcode = nullptr;

// Small arrow image for minimap (36x38 with transparency)
LV_IMG_DECLARE(small_arrow);

// Colors matching the design
#define COLOR_SKY_LIGHT     lv_color_hex(0xE0F2FE)
#define COLOR_SKY_BLUE      lv_color_hex(0xBAE6FD)
#define COLOR_SKY_DEEP      lv_color_hex(0x0284C7)
#define COLOR_TEXT_PRIMARY  lv_color_hex(0x0c4a6e)
#define COLOR_TEXT_SECONDARY lv_color_hex(0x075985)
#define COLOR_WHITE_30      lv_color_hex(0xFFFFFF)

// Pre-computed constants for performance
static const float DEG2RAD = M_PI / 180.0f;
static const float METERS_PER_DEGREE_LAT = 111000.0f;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Calculate distance between two lat/lon points in miles using Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 3959.0f; // Earth radius in miles

    float dLat = (lat2 - lat1) * DEG2RAD;
    float dLon = (lon2 - lon1) * DEG2RAD;

    float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
              cosf(lat1 * DEG2RAD) * cosf(lat2 * DEG2RAD) *
              sinf(dLon * 0.5f) * sinf(dLon * 0.5f);

    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return R * c;
}

// Calculate bearing from point 1 to point 2 in degrees (0 = North, 90 = East)
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG2RAD;
    float lat1Rad = lat1 * DEG2RAD;
    float lat2Rad = lat2 * DEG2RAD;

    float y = sinf(dLon) * cosf(lat2Rad);
    float x = cosf(lat1Rad) * sinf(lat2Rad) - sinf(lat1Rad) * cosf(lat2Rad) * cosf(dLon);

    float bearing = atan2f(y, x) * (180.0f / M_PI);

    // Normalize to 0-360
    bearing = fmodf((bearing + 360.0f), 360.0f);

    return bearing;
}

// Convert m/s to knots
float msToKnots(float ms) {
    return ms * 1.94384;
}

// Predict aircraft position based on elapsed time, velocity, and heading
// Returns predicted lat/lon and updates bearing/distance from user
void predictAircraftPosition(float elapsedSeconds, float* outLat, float* outLon,
                              float* outBearing, float* outDistance) {
    if (!nearestPlane.valid || nearestPlane.velocity < 1.0f) {
        // Not moving or no data - use last known position
        *outLat = nearestPlane.latitude;
        *outLon = nearestPlane.longitude;
        *outBearing = nearestPlane.bearing;
        *outDistance = nearestPlane.distance_miles;
        return;
    }

    // Convert heading to radians (heading 0 = North, 90 = East)
    float headingRad = nearestPlane.heading * DEG2RAD;

    // Calculate velocity components (m/s)
    float velocityNorth = nearestPlane.velocity * cosf(headingRad);  // North component
    float velocityEast = nearestPlane.velocity * sinf(headingRad);   // East component

    // Convert to degrees per second
    // 1 degree latitude ≈ 111,000 meters
    // 1 degree longitude ≈ 111,000 * cos(latitude) meters
    float metersPerDegreeLon = METERS_PER_DEGREE_LAT * cosf(nearestPlane.latitude * DEG2RAD);

    float latChangePerSecond = velocityNorth / METERS_PER_DEGREE_LAT;
    float lonChangePerSecond = velocityEast / metersPerDegreeLon;

    // Predict new position
    *outLat = nearestPlane.latitude + (latChangePerSecond * elapsedSeconds);
    *outLon = nearestPlane.longitude + (lonChangePerSecond * elapsedSeconds);

    // Calculate new bearing and distance from user to predicted position
    *outBearing = calculateBearing(userLat, userLon, *outLat, *outLon);
    *outDistance = calculateDistance(userLat, userLon, *outLat, *outLon);
}

// Normalize angle to 0-360 range
float normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

// Calculate shortest angular distance (handles wraparound)
float shortestAngularDistance(float from, float to) {
    float diff = normalizeAngle(to) - normalizeAngle(from);
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

// Set target bearing for smooth animation
void setTargetBearing(float bearing) {
    targetArrowAngle = normalizeAngle(bearing);
}

// ============================================================================
// LOCATION MANAGEMENT
// ============================================================================

Preferences location_prefs;

void loadLocation() {
    location_prefs.begin("location", true);  // read-only
    userLat = location_prefs.getFloat("lat", 0);
    userLon = location_prefs.getFloat("lon", 0);
    location_prefs.end();

    // Location is configured if both values are non-zero
    locationConfigured = (userLat != 0 || userLon != 0);

    if (locationConfigured) {
        Serial.printf("Location loaded: %.4f, %.4f\n", userLat, userLon);
    } else {
        Serial.println("No location configured - setup required");
    }
}

void saveLocation(float lat, float lon) {
    location_prefs.begin("location", false);  // read-write
    location_prefs.putFloat("lat", lat);
    location_prefs.putFloat("lon", lon);
    location_prefs.end();

    userLat = lat;
    userLon = lon;
    locationConfigured = true;

    Serial.printf("Location saved: %.4f, %.4f\n", lat, lon);
}

float getUserLat() { return userLat; }
float getUserLon() { return userLon; }
bool isLocationConfigured() { return locationConfigured; }

// ============================================================================
// FILTER SETTINGS
// ============================================================================

Preferences filter_prefs;

void loadFilterSettings() {
    filter_prefs.begin("filters", true);  // read-only
    hidePrivatePlanes = filter_prefs.getBool("hide_priv", false);
    filter_prefs.end();
    Serial.printf("Filter settings loaded: hidePrivate=%d\n", hidePrivatePlanes);
}

void saveFilterSettings() {
    filter_prefs.begin("filters", false);  // read-write
    filter_prefs.putBool("hide_priv", hidePrivatePlanes);
    filter_prefs.end();
    Serial.printf("Filter settings saved: hidePrivate=%d\n", hidePrivatePlanes);
}

bool getHidePrivatePlanes() { return hidePrivatePlanes; }
void setHidePrivatePlanes(bool val) { hidePrivatePlanes = val; }

// ============================================================================
// SCHEDULE MANAGEMENT
// ============================================================================

Preferences schedule_prefs;

void loadScheduleConfig() {
    schedule_prefs.begin("schedule", true);  // read-only

    String tz = schedule_prefs.getString("tz", "MST7MDT,M3.2.0,M11.1.0");
    strncpy(scheduleConfig.timezone, tz.c_str(), sizeof(scheduleConfig.timezone) - 1);

    scheduleConfig.enabled = schedule_prefs.getBool("enabled", false);
    scheduleConfig.daytime_start = schedule_prefs.getUChar("day_start", 7);
    scheduleConfig.daytime_end = schedule_prefs.getUChar("day_end", 21);
    scheduleConfig.dim_end = schedule_prefs.getUChar("dim_end", 23);
    scheduleConfig.daytime_brightness = schedule_prefs.getUChar("day_bright", 100);
    scheduleConfig.dim_brightness = schedule_prefs.getUChar("dim_bright", 50);

    schedule_prefs.end();

    Serial.printf("Schedule loaded: enabled=%d, tz=%s\n", scheduleConfig.enabled, scheduleConfig.timezone);
}

void saveScheduleConfig() {
    schedule_prefs.begin("schedule", false);  // read-write

    schedule_prefs.putString("tz", scheduleConfig.timezone);
    schedule_prefs.putBool("enabled", scheduleConfig.enabled);
    schedule_prefs.putUChar("day_start", scheduleConfig.daytime_start);
    schedule_prefs.putUChar("day_end", scheduleConfig.daytime_end);
    schedule_prefs.putUChar("dim_end", scheduleConfig.dim_end);
    schedule_prefs.putUChar("day_bright", scheduleConfig.daytime_brightness);
    schedule_prefs.putUChar("dim_bright", scheduleConfig.dim_brightness);

    schedule_prefs.end();

    Serial.println("Schedule config saved");
}

ScheduleConfig& getScheduleConfig() { return scheduleConfig; }

// ============================================================================
// NTP TIME SYNC
// ============================================================================

void setupNTP() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", scheduleConfig.timezone, 1);
    tzset();
    Serial.printf("NTP initialized with timezone: %s\n", scheduleConfig.timezone);
}

void updateTimezone() {
    setenv("TZ", scheduleConfig.timezone, 1);
    tzset();
    Serial.printf("Timezone updated to: %s\n", scheduleConfig.timezone);
}

bool isNtpSynced() {
    time_t now;
    time(&now);
    return now > 1577836800;  // After Jan 1, 2020
}

int getCurrentHour() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_hour;
}

String getCurrentTimeString() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%I:%M:%S %p", &timeinfo);
    return String(buf);
}

// ============================================================================
// DISPLAY SCHEDULE LOGIC
// ============================================================================

DisplayMode calculateScheduleMode(int hour) {
    if (!scheduleConfig.enabled) return MODE_DAYTIME;

    // Handle the case where dim_end wraps past midnight
    // Daytime: daytime_start to daytime_end
    // Dim: daytime_end to dim_end
    // Off: dim_end to daytime_start (wraps through midnight)

    if (hour >= scheduleConfig.daytime_start && hour < scheduleConfig.daytime_end) {
        return MODE_DAYTIME;
    }
    if (hour >= scheduleConfig.daytime_end && hour < scheduleConfig.dim_end) {
        return MODE_DIM;
    }
    return MODE_OFF;
}

void applyDisplayMode(DisplayMode mode) {
    if (mode == currentDisplayMode) return;

    currentDisplayMode = mode;
    switch (mode) {
        case MODE_DAYTIME:
            setBacklightBrightness(scheduleConfig.daytime_brightness);
            Serial.println("Display mode: DAYTIME");
            break;
        case MODE_DIM:
            setBacklightBrightness(scheduleConfig.dim_brightness);
            Serial.println("Display mode: DIM");
            break;
        case MODE_OFF:
            setBacklightBrightness(0);
            Serial.println("Display mode: OFF (API polling paused)");
            break;
    }
}

DisplayMode getCurrentDisplayMode() { return currentDisplayMode; }

void reapplyCurrentBrightness() {
    // Force reapply brightness for current mode (used after schedule config changes)
    switch (currentDisplayMode) {
        case MODE_DAYTIME:
            setBacklightBrightness(scheduleConfig.daytime_brightness);
            break;
        case MODE_DIM:
            setBacklightBrightness(scheduleConfig.dim_brightness);
            break;
        case MODE_OFF:
            setBacklightBrightness(0);
            break;
    }
    Serial.printf("Reapplied brightness for mode %d\n", currentDisplayMode);
}

// ============================================================================
// WATCHDOG TIMER FUNCTIONS
// ============================================================================

void setupWatchdog() {
    // Initialize watchdog with 5 minute timeout (panic on timeout = true)
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);  // Add current task (loop task) to watchdog
    watchdogEnabled = true;
    Serial.printf("Watchdog enabled: %lu second timeout\n", WDT_TIMEOUT_SECONDS);
}

void feedWatchdog() {
    if (watchdogEnabled) {
        esp_task_wdt_reset();
    }
}

void disableWatchdog() {
    if (watchdogEnabled) {
        esp_task_wdt_delete(NULL);
        watchdogEnabled = false;
        Serial.println("Watchdog disabled");
    }
}

// Cached arrow angle to avoid redundant LVGL calls
static int16_t cachedArrowAngle = 0;

// Update arrow animation - call every frame for smooth movement
bool updateArrowAnimation(float deltaMs) {
    if (!ui_arrow) return false;

    // Cap deltaMs to prevent jumps after HTTP requests block the loop
    if (deltaMs > MAX_DELTA_MS) deltaMs = MAX_DELTA_MS;

    float diff = shortestAngularDistance(currentArrowAngle, targetArrowAngle);
    float absDiff = fabsf(diff);

    // If very close, snap to target
    if (absDiff < 0.5f) {
        currentArrowAngle = targetArrowAngle;
        int16_t newAngle = (int16_t)(currentArrowAngle * 10);
        if (newAngle != cachedArrowAngle) {
            lv_img_set_angle(ui_arrow, newAngle);
            cachedArrowAngle = newAngle;
        }
        return false;
    }

    // Calculate rotation for this frame
    float maxRotation = ARROW_ROTATION_SPEED * (deltaMs / 1000.0f);
    float rotation = diff;
    if (fabsf(rotation) > maxRotation) {
        rotation = (diff > 0) ? maxRotation : -maxRotation;
    }

    currentArrowAngle = normalizeAngle(currentArrowAngle + rotation);
    int16_t newAngle = (int16_t)(currentArrowAngle * 10);
    if (newAngle != cachedArrowAngle) {
        lv_img_set_angle(ui_arrow, newAngle);
        cachedArrowAngle = newAngle;
    }
    return true;
}

// Cached minimap state to avoid redundant LVGL calls
static int16_t cachedMiniArrowAngles[5] = {0};
static int16_t cachedMiniArrowX[5] = {0};
static int16_t cachedMiniArrowY[5] = {0};
static bool cachedMiniArrowVisible[5] = {false};

// Update minimap arrows at screen edges pointing to nearby aircraft
// Supports position interpolation when elapsedSeconds > 0
void updateMiniArrows(float elapsedSeconds = 0) {
    for (int i = 0; i < 5; i++) {
        bool shouldBeVisible = (i < nearbyPlaneCount && nearbyPlanes[i].valid);

        if (!shouldBeVisible) {
            if (cachedMiniArrowVisible[i]) {
                lv_obj_add_flag(ui_mini_arrows[i], LV_OBJ_FLAG_HIDDEN);
                cachedMiniArrowVisible[i] = false;
            }
            continue;
        }

        // Show the arrow if hidden
        if (!cachedMiniArrowVisible[i]) {
            lv_obj_clear_flag(ui_mini_arrows[i], LV_OBJ_FLAG_HIDDEN);
            cachedMiniArrowVisible[i] = true;
        }

        // Get bearing - interpolate position if we have velocity data
        float bearing = nearbyPlanes[i].bearing;

        if (elapsedSeconds > 0 && nearbyPlanes[i].velocity > 1.0f) {
            // Predict position based on velocity and heading
            float headingRad = nearbyPlanes[i].heading * DEG2RAD;
            float velocityNorth = nearbyPlanes[i].velocity * cosf(headingRad);
            float velocityEast = nearbyPlanes[i].velocity * sinf(headingRad);

            float metersPerDegreeLon = METERS_PER_DEGREE_LAT * cosf(nearbyPlanes[i].latitude * DEG2RAD);

            float predLat = nearbyPlanes[i].latitude + (velocityNorth / METERS_PER_DEGREE_LAT) * elapsedSeconds;
            float predLon = nearbyPlanes[i].longitude + (velocityEast / metersPerDegreeLon) * elapsedSeconds;

            bearing = calculateBearing(userLat, userLon, predLat, predLon);
        }

        // Only update position if bearing changed significantly (>0.5 degrees)
        int16_t newAngle = (int16_t)(bearing * 10);

        float bearingRad = bearing * DEG2RAD;
        float sinB = sinf(bearingRad);
        float cosB = cosf(bearingRad);

        // Calculate where ray from center at this bearing hits screen edge
        // Use 240 as center (half of 480)
        float r;

        // Simplified edge intersection - find closest edge
        float rRight = (sinB > 0.001f) ? 240.0f / sinB : 99999.0f;
        float rLeft = (sinB < -0.001f) ? -240.0f / sinB : 99999.0f;
        float rBottom = (cosB < -0.001f) ? 240.0f / cosB : 99999.0f;
        float rTop = (cosB > 0.001f) ? 240.0f / cosB : 99999.0f;

        // Find minimum positive radius
        r = 99999.0f;
        if (rRight > 0 && rRight < r) r = rRight;
        if (rLeft > 0 && rLeft < r) r = rLeft;
        if (rBottom > 0 && rBottom < r) r = rBottom;
        if (rTop > 0 && rTop < r) r = rTop;

        // Calculate tip position at edge
        int16_t newX = (int16_t)(240.0f + r * sinB) - 18;
        int16_t newY = (int16_t)(240.0f - r * cosB);

        // Only update LVGL if values changed (LVGL calls are expensive)
        if (newX != cachedMiniArrowX[i] || newY != cachedMiniArrowY[i]) {
            lv_obj_set_pos(ui_mini_arrows[i], newX, newY);
            cachedMiniArrowX[i] = newX;
            cachedMiniArrowY[i] = newY;
        }

        if (newAngle != cachedMiniArrowAngles[i]) {
            lv_img_set_angle(ui_mini_arrows[i], newAngle);
            cachedMiniArrowAngles[i] = newAngle;
        }
    }
}

// Split-flap characters for randomization
const char FLAP_CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
const int FLAP_CHARS_LEN = sizeof(FLAP_CHARS) - 1;

// Start split-flap animation for a new callsign
void startSplitFlapAnimation(const char* newCallsign) {
    // Only animate if callsign actually changed
    if (strcmp(previousCallsign, newCallsign) == 0) return;

    strncpy(previousCallsign, newCallsign, sizeof(previousCallsign) - 1);
    splitFlapCharIndex = 0;
    splitFlapStartTime = millis();

    // Initialize displayed callsign with spaces (will be revealed char by char)
    int len = strlen(newCallsign);
    for (int i = 0; i < len && i < 15; i++) {
        displayedCallsign[i] = ' ';
    }
    displayedCallsign[len] = '\0';
}

// Update split-flap animation - returns true if still animating
bool updateSplitFlapAnimation() {
    if (splitFlapCharIndex < 0) return false;
    if (!ui_callsign) return false;

    int targetLen = strlen(previousCallsign);
    if (splitFlapCharIndex >= targetLen) {
        // Animation complete
        splitFlapCharIndex = -1;
        lv_label_set_text(ui_callsign, previousCallsign);
        return false;
    }

    unsigned long elapsed = millis() - splitFlapStartTime;
    int cycleTime = SPLIT_FLAP_CHAR_DURATION * SPLIT_FLAP_CYCLES;
    int totalCharTime = cycleTime + SPLIT_FLAP_CHAR_DURATION; // cycles + final settle

    // Which character should we be animating based on time?
    int charFromTime = elapsed / totalCharTime;

    // Update all characters up to the current one
    while (splitFlapCharIndex <= charFromTime && splitFlapCharIndex < targetLen) {
        // This character is done, show the final value
        displayedCallsign[splitFlapCharIndex] = previousCallsign[splitFlapCharIndex];
        splitFlapCharIndex++;
    }

    // If there's a character currently animating, show random char
    if (splitFlapCharIndex < targetLen) {
        int charElapsed = elapsed - (splitFlapCharIndex * totalCharTime);
        if (charElapsed < cycleTime) {
            // Still cycling through random characters
            displayedCallsign[splitFlapCharIndex] = FLAP_CHARS[random(FLAP_CHARS_LEN)];
        } else {
            // Settling on final character
            displayedCallsign[splitFlapCharIndex] = previousCallsign[splitFlapCharIndex];
        }
    }

    lv_label_set_text(ui_callsign, displayedCallsign);
    return true;
}

// ============================================================================
// LVGL CALLBACKS
// ============================================================================

// Cached framebuffer pointer for direct flush (avoids virtual dispatch per frame)
static uint16_t *hw_framebuffer = nullptr;
static volatile uint32_t flushCount = 0;  // Count actual display flushes for FPS

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint16_t *src = (uint16_t *)&color_p->full;
    uint16_t *dst = hw_framebuffer + ((int32_t)area->y1 * TFT_WIDTH) + area->x1;

    if (w == TFT_WIDTH) {
        // Full-width flush - single contiguous memcpy
        memcpy(dst, src, w * h * sizeof(uint16_t));
    } else {
        // Partial width - memcpy per row
        for (uint32_t y = 0; y < h; y++) {
            memcpy(dst, src, w * sizeof(uint16_t));
            src += w;
            dst += TFT_WIDTH;
        }
    }

    // Flush CPU cache so RGB LCD DMA sees updated pixels
    Cache_WriteBack_Addr((uint32_t)(hw_framebuffer + (int32_t)area->y1 * TFT_WIDTH),
                         TFT_WIDTH * h * sizeof(uint16_t));

    flushCount++;
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    if (simulated_touch_active) {
        unsigned long elapsed = millis() - simulated_touch_start;
        if (elapsed < SIMULATED_TOUCH_DURATION) {
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = simulated_touch_x;
            data->point.y = simulated_touch_y;
            return;
        } else {
            simulated_touch_active = false;
        }
    }

    touchController.read();

    if (touchController.isTouched) {
        data->state = LV_INDEV_STATE_PRESSED;
        int16_t raw_x = touchController.points[0].x;
        int16_t raw_y = touchController.points[0].y;
        data->point.x = TFT_WIDTH - 1 - raw_x;
        data->point.y = TFT_HEIGHT - 1 - raw_y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ============================================================================
// SETUP FUNCTIONS
// ============================================================================

// ============================================================================
// PWM BACKLIGHT FUNCTIONS
// ============================================================================

void setBacklightBrightness(uint8_t percent) {
    // Simple on/off control - this backlight doesn't support PWM dimming
    if (percent > 0) {
        digitalWrite(GFX_BL, HIGH);
    } else {
        digitalWrite(GFX_BL, LOW);
    }
    Serial.printf("Backlight %s\n", percent > 0 ? "ON" : "OFF");
}

void setupDisplay() {
    Serial.println("Initializing display...");
    gfx->begin(8000000);  // 8MHz pixel clock - stable
    gfx->fillScreen(BLACK);
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);

    // Cache framebuffer pointer for direct flush (avoids virtual dispatch)
    hw_framebuffer = gfx->getFramebuffer();
    Serial.printf("Display initialized (framebuffer: %p)\n", hw_framebuffer);
}

void setupLVGL() {
    Serial.println("Initializing LVGL...");

    lv_init();

    // Double-buffered full-frame rendering in PSRAM
    // This prevents tearing but limits FPS due to PSRAM copy overhead
    size_t buf_size = TFT_WIDTH * TFT_HEIGHT;
    disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!disp_draw_buf1 || !disp_draw_buf2) {
        Serial.println("Failed to allocate display buffers!");
        while (1) { delay(1000); }
    }

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, buf_size);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = TFT_WIDTH;
    disp_drv.ver_res = TFT_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 0;  // Partial refresh - only redraw changed areas
    lv_disp_drv_register(&disp_drv);

    Serial.println("LVGL initialized (partial refresh mode)");
}

void setupTouch() {
    Serial.println("Initializing touch...");
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    touchController.begin();
    touchController.setRotation(ROTATION_NORMAL);

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    touch_indev = lv_indev_drv_register(&indev_drv);
    Serial.println("Touch initialized");
}

void setupWiFi() {
    Serial.println("Setting up WiFi...");

    wifi_prefs.begin("wifi", false);
    String ssid = wifi_prefs.getString("ssid", "");
    String password = wifi_prefs.getString("password", "");
    wifi_prefs.end();

#if HAS_DEFAULT_WIFI
    if (ssid.length() == 0) {
        ssid = WIFI_SSID;
        password = WIFI_PASSWORD;
    }
#endif

    if (ssid.length() > 0) {
        Serial.printf("Connecting to: %s\n", ssid.c_str());
        WiFi.begin(ssid.c_str(), password.c_str());

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            return;
        }
    }

    Serial.println("Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Display", "configure");
    Serial.printf("AP: ESP32-Display, IP: %s\n", WiFi.softAPIP().toString().c_str());
}

// ============================================================================
// UI CREATION
// ============================================================================

void createUI() {
    Serial.println("Creating Flight Tracker UI...");

    ui_screen = lv_scr_act();
    lv_obj_clear_flag(ui_screen, LV_OBJ_FLAG_SCROLLABLE);

    // ========== BACKGROUND IMAGE ==========
    // Single image contains: gradient, route badge outline, compass circles,
    // cardinal labels (N/E/S/W), and bottom card backgrounds with labels
    lv_obj_t *bg = lv_img_create(ui_screen);
    lv_img_set_src(bg, &bg_image);
    lv_obj_set_pos(bg, 0, 0);

    // ========== DYNAMIC OVERLAYS ==========
    // Only create the elements that change: callsign, route text, arrow, values

    // Callsign (top center)
    ui_callsign = lv_label_create(ui_screen);
    lv_label_set_text(ui_callsign, "---");
    lv_obj_set_style_text_font(ui_callsign, &space_grotesk_bold_48, 0);
    lv_obj_set_style_text_color(ui_callsign, COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(ui_callsign, LV_ALIGN_TOP_MID, 0, 35);

    // Route label (in the badge area)
    ui_route_label = lv_label_create(ui_screen);
    lv_label_set_text(ui_route_label, "Searching...");
    lv_obj_set_style_text_font(ui_route_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(ui_route_label, COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(ui_route_label, LV_ALIGN_TOP_MID, 0, 98);

    // Arrow (center of compass)
    ui_arrow = lv_img_create(ui_screen);
    lv_img_set_src(ui_arrow, &arrow_130);
    lv_obj_align(ui_arrow, LV_ALIGN_CENTER, 0, 12);  // Centered in compass
    lv_img_set_pivot(ui_arrow, 65, 65);  // Center pivot for rotation
    lv_img_set_angle(ui_arrow, 0);

    // Airspeed value (bottom left card - centered)
    ui_airspeed_value = lv_label_create(ui_screen);
    lv_label_set_text(ui_airspeed_value, "---");
    lv_obj_set_style_text_font(ui_airspeed_value, &space_grotesk_bold_24, 0);
    lv_obj_set_style_text_color(ui_airspeed_value, COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_align(ui_airspeed_value, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(ui_airspeed_value, 20, 413);
    lv_obj_set_width(ui_airspeed_value, 140);

    // Altitude value (bottom center card - centered, same Y as airspeed)
    ui_distance_label = lv_label_create(ui_screen);
    lv_label_set_text(ui_distance_label, "---");
    lv_obj_set_style_text_font(ui_distance_label, &space_grotesk_bold_24, 0);
    lv_obj_set_style_text_color(ui_distance_label, COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_align(ui_distance_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(ui_distance_label, 170, 413);
    lv_obj_set_width(ui_distance_label, 140);

    // Aircraft value (bottom right card - centered vertically in value area)
    ui_aircraft_value = lv_label_create(ui_screen);
    lv_label_set_text(ui_aircraft_value, "---");
    lv_obj_set_style_text_font(ui_aircraft_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_aircraft_value, COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_align(ui_aircraft_value, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(ui_aircraft_value, 320, 418);
    lv_obj_set_width(ui_aircraft_value, 140);

    // Keep references for unused pointers (for compatibility)
    ui_route_badge = nullptr;
    ui_compass_container = nullptr;

    // Create minimap arrows for nearby aircraft (using small arrow image at 50% size)
    for (int i = 0; i < 5; i++) {
        ui_mini_arrows[i] = lv_img_create(ui_screen);
        lv_img_set_src(ui_mini_arrows[i], &small_arrow);
        lv_img_set_zoom(ui_mini_arrows[i], 128);  // 50% size (256 = 100%)
        // Pivot at arrow tip (top center of 36x38 image) - tip is at (18, 0)
        lv_img_set_pivot(ui_mini_arrows[i], 18, 0);
        lv_obj_add_flag(ui_mini_arrows[i], LV_OBJ_FLAG_HIDDEN);  // Hidden until data available
    }

    Serial.println("UI created");
}

// ============================================================================
// SETUP SCREEN (shown when location not configured)
// ============================================================================

void createSetupScreen() {
    Serial.println("Creating Setup Screen with QR code...");

    ui_setup_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_setup_screen, LV_OBJ_FLAG_SCROLLABLE);

    // Sky blue gradient background
    lv_obj_set_style_bg_color(ui_setup_screen, lv_color_hex(0xBAE6FD), 0);
    lv_obj_set_style_bg_grad_color(ui_setup_screen, lv_color_hex(0xE0F2FE), 0);
    lv_obj_set_style_bg_grad_dir(ui_setup_screen, LV_GRAD_DIR_VER, 0);

    // Title
    lv_obj_t *title = lv_label_create(ui_setup_screen);
    lv_label_set_text(title, "Flight Tracker");
    lv_obj_set_style_text_font(title, &space_grotesk_bold_48, 0);
    lv_obj_set_style_text_color(title, COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    // Subtitle
    lv_obj_t *subtitle = lv_label_create(ui_setup_screen);
    lv_label_set_text(subtitle, "Setup Required");
    lv_obj_set_style_text_font(subtitle, &space_grotesk_bold_24, 0);
    lv_obj_set_style_text_color(subtitle, COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 90);

    // Build the URL for the QR code
    String ip = WiFi.status() == WL_CONNECTED ?
                WiFi.localIP().toString() :
                WiFi.softAPIP().toString();
    String url = "http://" + ip + "/";

    // QR code (200x200 pixels)
    ui_qrcode = lv_qrcode_create(ui_setup_screen, 200, lv_color_hex(0x0c4a6e), lv_color_white());
    lv_qrcode_update(ui_qrcode, url.c_str(), url.length());
    lv_obj_align(ui_qrcode, LV_ALIGN_CENTER, 0, -10);

    // Add white border around QR code
    lv_obj_set_style_border_color(ui_qrcode, lv_color_white(), 0);
    lv_obj_set_style_border_width(ui_qrcode, 10, 0);

    // Instructions
    lv_obj_t *instr1 = lv_label_create(ui_setup_screen);
    lv_label_set_text(instr1, "Scan to configure your location");
    lv_obj_set_style_text_font(instr1, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(instr1, COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(instr1, LV_ALIGN_CENTER, 0, 130);

    // IP address display
    lv_obj_t *ipLabel = lv_label_create(ui_setup_screen);
    char ipText[64];
    snprintf(ipText, sizeof(ipText), "Or visit: %s", ip.c_str());
    lv_label_set_text(ipLabel, ipText);
    lv_obj_set_style_text_font(ipLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ipLabel, COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(ipLabel, LV_ALIGN_CENTER, 0, 160);

    // WiFi status at bottom
    lv_obj_t *wifiStatus = lv_label_create(ui_setup_screen);
    if (WiFi.status() == WL_CONNECTED) {
        char wifiText[64];
        snprintf(wifiText, sizeof(wifiText), "WiFi: %s", WiFi.SSID().c_str());
        lv_label_set_text(wifiStatus, wifiText);
    } else {
        lv_label_set_text(wifiStatus, "WiFi: ESP32-Display (AP Mode)");
    }
    lv_obj_set_style_text_font(wifiStatus, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(wifiStatus, COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(wifiStatus, LV_ALIGN_BOTTOM_MID, 0, -20);

    Serial.println("Setup screen created");
}

void showSetupScreen() {
    if (!ui_setup_screen) {
        createSetupScreen();
    }
    lv_scr_load(ui_setup_screen);
    Serial.println("Showing setup screen");
}

void showFlightUI() {
    if (!ui_screen) {
        createUI();
    }
    lv_scr_load(ui_screen);
    firstApiFetch = true;  // Trigger immediate fetch when switching to flight UI

    // Enable watchdog when transitioning to flight tracker mode
    if (!watchdogEnabled) {
        setupWatchdog();
    }

    Serial.println("Showing flight tracker UI");
}

// ============================================================================
// UPDATE UI
// ============================================================================

void updateUI() {
    // Only log on actual data changes (called after API fetch)
    Serial.printf("updateUI: %s at %.1f mi\n",
        nearestPlane.valid ? nearestPlane.callsign : "---", nearestPlane.distance_miles);
    if (!nearestPlane.valid) {
        lv_label_set_text(ui_callsign, "---");
        if (rateLimitBackoff > 0) {
            char msg[32];
            snprintf(msg, sizeof(msg), "Retry in %lus", rateLimitBackoff / 1000);
            lv_label_set_text(ui_route_label, msg);
        } else {
            lv_label_set_text(ui_route_label, "Searching...");
        }
        lv_label_set_text(ui_airspeed_value, "---");
        lv_label_set_text(ui_distance_label, "---");
        lv_label_set_text(ui_aircraft_value, "---");
        return;
    }

    // Update callsign with split-flap animation
    const char* newCallsign = strlen(nearestPlane.callsign) > 0 ?
                              nearestPlane.callsign : nearestPlane.icao24;
    startSplitFlapAnimation(newCallsign);
    logPrintf("Set callsign to: %s\n", newCallsign);

    // Update route badge
    if (strlen(nearestPlane.origin) > 0 && strlen(nearestPlane.destination) > 0) {
        char route[32];
        snprintf(route, sizeof(route), "%s  >  %s", nearestPlane.origin, nearestPlane.destination);
        lv_label_set_text(ui_route_label, route);
    } else {
        // Show bearing direction
        const char* dir;
        float b = nearestPlane.bearing;
        if (b >= 337.5 || b < 22.5) dir = "North";
        else if (b >= 22.5 && b < 67.5) dir = "Northeast";
        else if (b >= 67.5 && b < 112.5) dir = "East";
        else if (b >= 112.5 && b < 157.5) dir = "Southeast";
        else if (b >= 157.5 && b < 202.5) dir = "South";
        else if (b >= 202.5 && b < 247.5) dir = "Southwest";
        else if (b >= 247.5 && b < 292.5) dir = "West";
        else dir = "Northwest";
        lv_label_set_text(ui_route_label, dir);
    }

    // Update airspeed (with kts suffix)
    char speedStr[16];
    int knots = (int)msToKnots(nearestPlane.velocity);
    snprintf(speedStr, sizeof(speedStr), "%d kts", knots);
    lv_label_set_text(ui_airspeed_value, speedStr);

    // Update altitude (convert meters to feet, format with commas and apostrophe)
    char altStr[16];
    int altFeet = (int)(nearestPlane.altitude * 3.28084f);
    if (altFeet >= 10000) {
        snprintf(altStr, sizeof(altStr), "%d,%03d'", altFeet / 1000, altFeet % 1000);
    } else if (altFeet >= 1000) {
        snprintf(altStr, sizeof(altStr), "%d,%03d'", altFeet / 1000, altFeet % 1000);
    } else {
        snprintf(altStr, sizeof(altStr), "%d'", altFeet);
    }
    lv_label_set_text(ui_distance_label, altStr);

    // Update aircraft type
    if (strlen(nearestPlane.aircraft_type) > 0) {
        lv_label_set_text(ui_aircraft_value, nearestPlane.aircraft_type);
    } else {
        lv_label_set_text(ui_aircraft_value, "Unknown");
    }

    // Update arrow direction (smooth animation)
    setTargetBearing(nearestPlane.bearing);

    // Update minimap arrows for nearby aircraft (no interpolation on fresh data)
    updateMiniArrows(0);
}

// ============================================================================
// API FUNCTIONS
// ============================================================================

// API selection: 0 = OpenSky, 1 = airplanes.live
int currentApi = 1; // Start with airplanes.live (less aggressive rate limiting)
unsigned long apiSwitchTime = 0;

// Background API fetch task (runs HTTP on core 0, keeps render loop smooth)
static TaskHandle_t apiTaskHandle = nullptr;
static volatile bool apiInProgress = false;
static volatile bool apiResultReady = false;

bool fetchAirplanesLive() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    HTTPClient http;

    // airplanes.live API: /v2/point/{lat}/{lon}/{radius_nm}
    char url[128];
    snprintf(url, sizeof(url),
        "https://api.airplanes.live/v2/point/%.4f/%.4f/%d",
        userLat, userLon, currentSearchRadiusMiles
    );

    logPrintf("Fetching (airplanes.live): %s\n", url);

    http.begin(url);
    http.setTimeout(10000);

    logPrint("Sending request...\n");
    int httpCode = http.GET();
    logPrintf("Response code: %d\n", httpCode);

    if (httpCode != HTTP_CODE_OK) {
        logPrintf("airplanes.live HTTP error: %d\n", httpCode);
        if (httpCode == 429) {
            // Switch back to OpenSky
            currentApi = 0;
            apiSwitchTime = millis();
            logPrint("airplanes.live rate limited, switching to OpenSky\n");
        }
        http.end();
        return false;
    }

    logPrint("Getting response...\n");
    String payload = http.getString();
    http.end();
    logPrintf("Response length: %d bytes\n", payload.length());

    DynamicJsonDocument doc(98304);  // 96KB for large responses
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        logPrintf("JSON parse error: %s\n", error.c_str());
        return false;
    }

    JsonArray aircraft = doc["ac"];
    if (aircraft.isNull() || aircraft.size() == 0) {
        logPrint("No aircraft found\n");
        nearestPlane.valid = false;
        nearbyPlaneCount = 0;  // Clear minimap data
        return false;
    }

    logPrintf("Found %d aircraft (airplanes.live)\n", (int)aircraft.size());

    // Two-pass approach to avoid stack overflow:
    // Pass 1: Find indices and distances of 6 nearest aircraft
    // Pass 2: Extract full data only for those 6

    struct NearestCandidate {
        int index;
        float distance;
    };

    NearestCandidate nearest6[6] = {{-1, 999999}, {-1, 999999}, {-1, 999999},
                                     {-1, 999999}, {-1, 999999}, {-1, 999999}};

    // Pass 1: Find 6 nearest by distance (keep sorted list)
    int acIndex = 0;
    int skippedGround = 0;
    for (JsonObject ac : aircraft) {
        if (ac["lat"].isNull() || ac["lon"].isNull()) {
            acIndex++;
            continue;
        }

        // Filter out ground/taxiing aircraft
        // Check altitude (in feet from API) - skip if < 50 feet
        if (!ac["alt_baro"].isNull()) {
            float altFeet = ac["alt_baro"].as<float>();
            if (altFeet < 50) {
                skippedGround++;
                acIndex++;
                continue;
            }
        } else if (!ac["alt_geom"].isNull()) {
            float altFeet = ac["alt_geom"].as<float>();
            if (altFeet < 50) {
                skippedGround++;
                acIndex++;
                continue;
            }
        }

        // Check ground speed (in knots from API) - skip if < 10 kts
        if (!ac["gs"].isNull()) {
            float speedKts = ac["gs"].as<float>();
            if (speedKts < 10) {
                skippedGround++;
                acIndex++;
                continue;
            }
        }

        float lat = ac["lat"].as<float>();
        float lon = ac["lon"].as<float>();
        float dist = calculateDistance(userLat, userLon, lat, lon);

        // Filter private/anonymous aircraft (registration == callsign)
        // unless very close (< 0.25 miles)
        if (hidePrivatePlanes && dist >= 0.25f) {
            const char* reg = ac["r"];
            const char* flight = ac["flight"];
            if (reg && flight) {
                // Trim trailing spaces from flight
                char trimmedFlight[16];
                strncpy(trimmedFlight, flight, sizeof(trimmedFlight) - 1);
                trimmedFlight[sizeof(trimmedFlight) - 1] = '\0';
                int len = strlen(trimmedFlight);
                while (len > 0 && trimmedFlight[len-1] == ' ') {
                    trimmedFlight[--len] = '\0';
                }
                if (strcmp(reg, trimmedFlight) == 0) {
                    skippedGround++;
                    acIndex++;
                    continue;
                }
            }
        }

        // Insert into sorted list if closer than any current candidate
        for (int i = 0; i < 6; i++) {
            if (dist < nearest6[i].distance) {
                // Shift others down
                for (int j = 5; j > i; j--) {
                    nearest6[j] = nearest6[j - 1];
                }
                nearest6[i].index = acIndex;
                nearest6[i].distance = dist;
                break;
            }
        }
        acIndex++;
    }

    // Pass 2: Extract data for the 6 nearest aircraft
    nearestPlane.valid = false;
    nearbyPlaneCount = 0;

    // Initialize nearby planes as invalid
    for (int i = 0; i < 5; i++) {
        nearbyPlanes[i].valid = false;
    }

    acIndex = 0;
    for (JsonObject ac : aircraft) {
        // Check if this index matches any of our 6 targets
        for (int slot = 0; slot < 6; slot++) {
            if (acIndex == nearest6[slot].index) {
                // Parse aircraft data inline
                AircraftData* data = (slot == 0) ? &nearestPlane : &nearbyPlanes[slot - 1];

                data->valid = true;
                data->latitude = ac["lat"].as<float>();
                data->longitude = ac["lon"].as<float>();
                data->distance_miles = calculateDistance(userLat, userLon, data->latitude, data->longitude);
                data->bearing = calculateBearing(userLat, userLon, data->latitude, data->longitude);

                // Callsign
                const char* flight = ac["flight"];
                if (flight) {
                    strncpy(data->callsign, flight, sizeof(data->callsign) - 1);
                    int len = strlen(data->callsign);
                    while (len > 0 && data->callsign[len-1] == ' ') {
                        data->callsign[--len] = '\0';
                    }
                }

                // ICAO hex code
                const char* hex = ac["hex"];
                if (hex) strncpy(data->icao24, hex, sizeof(data->icao24) - 1);

                // Aircraft type
                const char* type = ac["desc"];
                if (type) {
                    strncpy(data->aircraft_type, type, sizeof(data->aircraft_type) - 1);
                } else {
                    const char* t = ac["t"];
                    if (t) strncpy(data->aircraft_type, t, sizeof(data->aircraft_type) - 1);
                }

                // Ground speed
                if (!ac["gs"].isNull()) {
                    data->velocity = ac["gs"].as<float>() / 1.94384f;
                }

                // Track/heading
                if (!ac["track"].isNull()) {
                    data->heading = ac["track"].as<float>();
                }

                // Altitude
                if (!ac["alt_baro"].isNull()) {
                    data->altitude = ac["alt_baro"].as<float>() * 0.3048f;
                } else if (!ac["alt_geom"].isNull()) {
                    data->altitude = ac["alt_geom"].as<float>() * 0.3048f;
                }

                if (slot == 0) {
                    logPrintf("Nearest: %s at %.1f mi, bearing %.0f°, type: %s\n",
                        nearestPlane.callsign, nearestPlane.distance_miles,
                        nearestPlane.bearing, nearestPlane.aircraft_type);
                }
                break;
            }
        }
        acIndex++;
    }

    // Count valid nearby planes (count all valid, they should be contiguous)
    nearbyPlaneCount = 0;
    for (int i = 0; i < 5; i++) {
        if (nearbyPlanes[i].valid) nearbyPlaneCount++;
    }

    if (!nearestPlane.valid) {
        logPrint("No valid aircraft found in results\n");
    }
    if (skippedGround > 0) {
        logPrintf("Filtered %d ground/slow aircraft\n", skippedGround);
    }
    logPrintf("Minimap: %d nearby aircraft\n", nearbyPlaneCount);

    // Adjust search radius based on aircraft count
    int totalAircraft = aircraft.size();
    int previousRadius = currentSearchRadiusMiles;

    if (totalAircraft < TARGET_AIRCRAFT_COUNT && currentSearchRadiusMiles < MAX_SEARCH_RADIUS) {
        // Not enough aircraft - increase radius
        // Increase by 50% or at least 10 miles, whichever is larger
        int increase = currentSearchRadiusMiles / 2;
        if (increase < 10) increase = 10;
        currentSearchRadiusMiles += increase;
        if (currentSearchRadiusMiles > MAX_SEARCH_RADIUS) {
            currentSearchRadiusMiles = MAX_SEARCH_RADIUS;
        }
        logPrintf("Radius %d->%d mi (found %d, need %d)\n",
            previousRadius, currentSearchRadiusMiles, totalAircraft, TARGET_AIRCRAFT_COUNT);
    } else if (totalAircraft > REDUCE_THRESHOLD && currentSearchRadiusMiles > MIN_SEARCH_RADIUS) {
        // Too many aircraft - decrease radius to reduce data transfer
        // Decrease by 20% (gentler reduction)
        int decrease = currentSearchRadiusMiles / 5;
        if (decrease < 5) decrease = 5;
        currentSearchRadiusMiles -= decrease;
        if (currentSearchRadiusMiles < MIN_SEARCH_RADIUS) {
            currentSearchRadiusMiles = MIN_SEARCH_RADIUS;
        }
        logPrintf("Radius %d->%d mi (found %d, reducing)\n",
            previousRadius, currentSearchRadiusMiles, totalAircraft);
    }

    return nearestPlane.valid;
}

bool fetchOpenSkyData() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return false;
    }

    HTTPClient http;

    // Build URL with bounding box around user location
    char url[256];
    snprintf(url, sizeof(url),
        "https://opensky-network.org/api/states/all?lamin=%.2f&lomin=%.2f&lamax=%.2f&lomax=%.2f",
        userLat - BBOX_RANGE, userLon - BBOX_RANGE,
        userLat + BBOX_RANGE, userLon + BBOX_RANGE
    );

    logPrintf("Fetching OpenSky: %s\n", url);

    http.begin(url);
    http.setTimeout(10000);

    // Collect rate limit header
    const char* headerKeys[] = {"x-rate-limit-retry-after-seconds"};
    http.collectHeaders(headerKeys, 1);

    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        logPrintf("OpenSky HTTP error: %d\n", httpCode);
        if (httpCode == 429) {
            // Rate limited - check header for retry time
            String retryAfter = http.header("x-rate-limit-retry-after-seconds");
            if (retryAfter.length() > 0) {
                rateLimitBackoff = retryAfter.toInt() * 1000UL;
                logPrintf("Rate limited! Retry after %s seconds\n", retryAfter.c_str());
            } else {
                rateLimitBackoff = 60000; // Default 60s
                logPrint("Rate limited! Backing off 60s...\n");
            }
        }
        http.end();
        return false;
    }
    rateLimitBackoff = 0; // Reset backoff on success

    String payload = http.getString();
    http.end();

    // Parse JSON
    DynamicJsonDocument doc(98304);  // 96KB for large responses
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        logPrintf("JSON parse error: %s\n", error.c_str());
        return false;
    }

    JsonArray states = doc["states"];
    if (states.isNull() || states.size() == 0) {
        logPrint("No aircraft found in area\n");
        nearestPlane.valid = false;
        return false;
    }

    logPrintf("Found %d aircraft (OpenSky)\n", states.size());

    // Find nearest aircraft
    float minDistance = 999999;
    AircraftData nearest;
    nearest.valid = false;

    for (JsonArray state : states) {
        // OpenSky state vector indices:
        // 0: icao24, 1: callsign, 2: origin_country, 3: time_position
        // 4: last_contact, 5: longitude, 6: latitude, 7: baro_altitude
        // 8: on_ground, 9: velocity, 10: true_track, 11: vertical_rate
        // 12: sensors, 13: geo_altitude, 14: squawk, 15: spi, 16: position_source

        if (state[6].isNull() || state[5].isNull()) continue; // No position
        if (state[8].as<bool>()) continue; // Skip aircraft on ground

        float lat = state[6].as<float>();
        float lon = state[5].as<float>();

        float dist = calculateDistance(userLat, userLon, lat, lon);

        if (dist < minDistance) {
            minDistance = dist;

            nearest.valid = true;
            nearest.latitude = lat;
            nearest.longitude = lon;
            nearest.distance_miles = dist;
            nearest.bearing = calculateBearing(userLat, userLon, lat, lon);

            // Copy callsign (trim whitespace)
            const char* cs = state[1].as<const char*>();
            if (cs) {
                strncpy(nearest.callsign, cs, sizeof(nearest.callsign) - 1);
                // Trim trailing whitespace
                int len = strlen(nearest.callsign);
                while (len > 0 && nearest.callsign[len-1] == ' ') {
                    nearest.callsign[--len] = '\0';
                }
            }

            // Copy icao24
            const char* icao = state[0].as<const char*>();
            if (icao) {
                strncpy(nearest.icao24, icao, sizeof(nearest.icao24) - 1);
            }

            // Altitude and velocity
            if (!state[7].isNull()) nearest.altitude = state[7].as<float>();
            if (!state[9].isNull()) nearest.velocity = state[9].as<float>();
            if (!state[10].isNull()) nearest.heading = state[10].as<float>();
        }
    }

    if (nearest.valid) {
        memcpy(&nearestPlane, &nearest, sizeof(AircraftData));
        Serial.printf("Nearest: %s at %.1f miles, bearing %.0f°\n",
            nearest.callsign, nearest.distance_miles, nearest.bearing);
    }

    return nearest.valid;
}

bool fetchAircraftInfo(const char* icao24) {
    if (WiFi.status() != WL_CONNECTED || strlen(icao24) == 0) {
        return false;
    }

    HTTPClient http;

    char url[128];
    snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s", icao24);

    Serial.printf("Fetching aircraft info: %s\n", url);

    http.begin(url);
    http.setTimeout(5000);

    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("ADSBDB HTTP error: %d\n", httpCode);
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.printf("ADSBDB JSON error: %s\n", error.c_str());
        return false;
    }

    // Extract aircraft type
    JsonObject response = doc["response"];
    if (!response.isNull()) {
        JsonObject aircraft = response["aircraft"];
        if (!aircraft.isNull()) {
            const char* type = aircraft["type"];
            if (type) {
                strncpy(nearestPlane.aircraft_type, type, sizeof(nearestPlane.aircraft_type) - 1);
                Serial.printf("Aircraft type: %s\n", type);
            }

            const char* reg = aircraft["registration"];
            if (reg) {
                strncpy(nearestPlane.registration, reg, sizeof(nearestPlane.registration) - 1);
            }
        }
    }

    return true;
}

bool fetchCallsignRoute(const char* callsign) {
    if (WiFi.status() != WL_CONNECTED || strlen(callsign) == 0) {
        return false;
    }

    HTTPClient http;

    char url[128];
    snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", callsign);

    Serial.printf("Fetching route: %s\n", url);

    http.begin(url);
    http.setTimeout(5000);

    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("Route API error: %d\n", httpCode);
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.printf("Route JSON error: %s\n", error.c_str());
        return false;
    }

    // Extract origin and destination
    JsonObject response = doc["response"];
    if (!response.isNull()) {
        JsonObject flightroute = response["flightroute"];
        if (!flightroute.isNull()) {
            JsonObject origin = flightroute["origin"];
            JsonObject destination = flightroute["destination"];

            if (!origin.isNull()) {
                const char* iata = origin["iata_code"];
                if (iata) {
                    strncpy(nearestPlane.origin, iata, sizeof(nearestPlane.origin) - 1);
                }
            }

            if (!destination.isNull()) {
                const char* iata = destination["iata_code"];
                if (iata) {
                    strncpy(nearestPlane.destination, iata, sizeof(nearestPlane.destination) - 1);
                }
            }

            if (strlen(nearestPlane.origin) > 0 && strlen(nearestPlane.destination) > 0) {
                Serial.printf("Route: %s -> %s\n", nearestPlane.origin, nearestPlane.destination);
                return true;
            }
        }
    }

    return false;
}

// ============================================================================
// SERIAL COMMAND HANDLER
// ============================================================================

String serialBuffer = "";

void handleSerialCommand(const String& cmd) {
    if (cmd == "STATUS") {
        Serial.println("---STATUS_BEGIN---");
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WIFI:CONNECTED");
            Serial.print("IP:");
            Serial.println(WiFi.localIP().toString());
            Serial.print("SSID:");
            Serial.println(WiFi.SSID());
        } else if (WiFi.getMode() == WIFI_AP) {
            Serial.println("WIFI:AP_MODE");
            Serial.print("IP:");
            Serial.println(WiFi.softAPIP().toString());
            Serial.println("SSID:ESP32-Display");
        } else {
            Serial.println("WIFI:DISCONNECTED");
        }
        Serial.print("HEAP:");
        Serial.println(ESP.getFreeHeap());
        Serial.print("UPTIME:");
        Serial.println(millis() / 1000);
        Serial.println("---STATUS_END---");
    }
}

void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                handleSerialCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
        }
    }
}

// ============================================================================
// BACKGROUND API TASK (runs on core 0, separate from render loop)
// ============================================================================

void apiTask(void *param) {
    for (;;) {
        // Block until main loop requests a fetch
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bool success = false;

        if (currentApi == 0) {
            success = fetchOpenSkyData();
            if (!success && rateLimitBackoff > 0) {
                currentApi = 1;
                apiSwitchTime = millis();
                logPrint("Switching to airplanes.live\n");
                success = fetchAirplanesLive();
            }
        } else {
            success = fetchAirplanesLive();
            if (millis() - apiSwitchTime > 300000) {
                currentApi = 0;
                rateLimitBackoff = 0;
            }
        }

        if (success) {
            fetchAircraftInfo(nearestPlane.icao24);
            fetchCallsignRoute(nearestPlane.callsign);
        }

        apiResultReady = true;
        apiInProgress = false;
    }
}

// ============================================================================
// ARDUINO SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n========================================");
    Serial.println("ESP32 Flight Tracker Starting...");
    Serial.println("========================================\n");

    if (psramFound()) {
        Serial.printf("PSRAM: %d MB\n", ESP.getPsramSize() / 1024 / 1024);
    }

    initScreenshot();
    setupDisplay();
    setupLVGL();
    setupTouch();

    // Load configuration from preferences before creating UI
    loadLocation();
    loadScheduleConfig();
    loadFilterSettings();

    // Create the appropriate screen based on location configuration
    if (locationConfigured) {
        createUI();
    } else {
        // Create setup screen after WiFi is up so we have the IP
    }

    lv_timer_handler();

    setupWiFi();

    // Initialize NTP after WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
        setupNTP();
    }

    webServer.begin();

    // Start background API task on core 0 (WiFi core) - keeps render loop on core 1 smooth
    xTaskCreatePinnedToCore(apiTask, "api_fetch", 16384, NULL, 1, &apiTaskHandle, 0);
    Serial.println("Background API task started on core 0");

    // Now that WiFi is up, create setup screen if needed
    if (!locationConfigured) {
        createSetupScreen();
        lv_scr_load(ui_setup_screen);
        lv_timer_handler();  // Force render after loading setup screen
    }

    // Enable watchdog timer when location is configured (API fetching active)
    if (locationConfigured) {
        setupWatchdog();
    }

    Serial.println("\n========================================");
    Serial.println("System Ready!");
    if (locationConfigured) {
        Serial.printf("Location: %.4f, %.4f\n", userLat, userLon);
    } else {
        Serial.println("Location: NOT CONFIGURED - Setup required");
    }
    Serial.printf("Web: http://%s\n",
                  WiFi.status() == WL_CONNECTED ?
                  WiFi.localIP().toString().c_str() :
                  WiFi.softAPIP().toString().c_str());
    Serial.println("========================================\n");
}

void loop() {
    unsigned long now = millis();
    float deltaMs = now - last_tick;
    lv_tick_inc(now - last_tick);
    last_tick = now;

    lv_timer_handler();
    processSerial();

    // FPS tracking (measures actual display flushes, not loop iterations)
    fpsFrameCount++;
    if (now - fpsLastReport >= FPS_REPORT_INTERVAL) {
        float loopRate = (float)fpsFrameCount * 1000.0f / (now - fpsLastReport);
        currentFps = (float)flushCount * 1000.0f / (now - fpsLastReport);
        Serial.printf("FPS: %.1f render, %.0f loop/s\n", currentFps, loopRate);
        fpsFrameCount = 0;
        flushCount = 0;
        fpsLastReport = now;
    }

    // Feed the watchdog to prevent reset (must be called at least every 5 minutes)
    feedWatchdog();

    // Check NTP sync status (once per minute)
    if (WiFi.status() == WL_CONNECTED && now - lastNtpCheck > 60000) {
        bool wasSynced = ntpSynced;
        ntpSynced = isNtpSynced();
        if (!wasSynced && ntpSynced) {
            Serial.printf("NTP synced! Current time: %s\n", getCurrentTimeString().c_str());
        }
        lastNtpCheck = now;
    }

    // Update display schedule (every second when NTP is synced)
    if (ntpSynced && scheduleConfig.enabled && now - lastScheduleCheck > 1000) {
        DisplayMode newMode = calculateScheduleMode(getCurrentHour());
        if (newMode != currentDisplayMode) {
            applyDisplayMode(newMode);
        }
        lastScheduleCheck = now;
    }

    // Only run flight tracker logic when location is configured
    if (!locationConfigured) {
        static unsigned long lastLocDebug = 0;
        if (now - lastLocDebug > 5000) {
            Serial.println("Waiting for location config...");
            lastLocDebug = now;
        }
        ElegantOTA.loop();
        taskYIELD();
        return;
    }

    // Skip flight tracker when screen is off (pause API polling)
    if (currentDisplayMode == MODE_OFF) {
        ElegantOTA.loop();
        delay(50);  // Longer delay when screen is off to save power
        return;
    }

    // Smooth arrow animation - always run to keep animating even during data updates
    updateArrowAnimation(deltaMs);

    // Split-flap callsign animation
    updateSplitFlapAnimation();

    // Position interpolation - predict aircraft location and update arrow bearing
    if (nearestPlane.valid && lastPositionUpdate > 0) {
        float elapsedSeconds = (now - lastPositionUpdate) / 1000.0f;
        float predLat, predLon, predBearing, predDistance;
        predictAircraftPosition(elapsedSeconds, &predLat, &predLon, &predBearing, &predDistance);
        setTargetBearing(predBearing);

        // Also update minimap arrows with interpolation
        updateMiniArrows(elapsedSeconds);
    }

    // Check for API results from background task (non-blocking)
    if (apiResultReady) {
        apiResultReady = false;
        lastPositionUpdate = now;
        updateUI();
    }

    // Trigger background API fetch if needed
    unsigned long effectiveInterval = API_UPDATE_INTERVAL + rateLimitBackoff;
    bool shouldFetch = firstApiFetch || (now - lastApiUpdate > effectiveInterval);

    if (!apiInProgress && WiFi.status() == WL_CONNECTED && shouldFetch) {
        lastApiUpdate = now;
        firstApiFetch = false;
        apiInProgress = true;
        xTaskNotifyGive(apiTaskHandle);
    }

    ElegantOTA.loop();

    // Yield to other tasks but don't add unnecessary delay
    // LVGL handles timing internally via lv_timer_handler()
    taskYIELD();
}
