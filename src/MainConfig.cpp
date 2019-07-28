// Batteries configuration
#define ENABLE_BATTERY_MONITOR
#ifdef PWR_LED_PIN
// Brightness level: 0 - 255. Auto = -1. Any number different from: -1, 0 - 255 = LED OFF
const int LOW_BATTERY_LED_BRIGHTNESS = -1;
// Battery percentage threshold to activate the LED
#define LOW_BATTERY_THRESHOLD 10
// Blink time (ms)
#define LOW_BATTERY_BLINK_TIME 300
#endif
#define BATTERY_PERCENT_TOLERANCE 5
// AAA batteries = 1, CR2032 = 2
#define BATTERY_TYPE 1
// AAA batteries chemistry: 0 = Autodetect, 1 = Custom V min and V max, 2 = NiMH (rechargeable), 3 = Alkaline (disposable)
#define AAA_BATT_CHEMISTRY 0
const float CUSTOM_V_MIN = 1.0;
const float CUSTOM_V_MAX = 3.3;
// Milliseconds to wait after radio module activity (necessary only with CR2032 batteries)
#define CR2032_RADIO_WAIT_TIME 400

// Booster configuration
#ifdef BOOSTER_PIN
#define ENABLE_BOOSTER_MONITOR
// booster policy: ALWAYS OFF = 0, ALWAYS ON = 1, AUTO = 2
#define BOOSTER_POLICY 2
// Vcc min voltage to activate the booster when BOOSTER_POLICY = 2
const float BoostThreshold = 2.7;
#endif

// Ext power monitor
#define ENABLE_EXT_PWR_MONITOR
#ifdef PWR_LED_PIN
// Brightness level: 0 - 255. Auto = -1. Any number different from: -1 or 0 - 255 = LED OFF
const int EXT_POWER_LED_BRIGHTNESS = 255;
#endif

// Vcc read configuration
#define ENABLE_VCC_MONITOR
#define MEAN_VCC_READS 5
// Measured Vcc by multimeter divided by reported Vcc
const float VccCorrection = 1.0 / 1.0;
const float VccTol = 0.2;

// Heartbeat
#define ENABLE_HEARTBEAT

// Motion led
#ifdef FRONT_PIR_PIN
#ifdef MOTION_LED_PIN
#define ENABLE_MOTION_LED
#ifdef ENABLE_MOTION_LED
// Brightness level: 0 - 255. Auto = -1. Any number different from: -1, 0 - 255 = LED OFF
const int MOTION_LED_BRIGHTNESS = -1;
#endif
#endif
#endif

// SI7021 temperature and humidity
#define ENABLE_SI7021
#ifdef ENABLE_SI7021
const float TempTol = 0.3;
const float HumTol = 0.3;
#endif

// Photoresistor
#ifdef PHOTORES_PIN
#define PHOTORES_TOLERANCE 2
#endif