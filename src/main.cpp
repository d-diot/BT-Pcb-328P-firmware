// **************************** PLATFORMIO ********************************

// Platformio libs
#include <stdint.h>
#include <Arduino.h>

// ******************************* NODE CONFIGURATION **********************************

// Sampling interval configuration
static const uint64_t UPDATE_INTERVAL = 600000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

// Debug configuration
#define MY_DEBUG

//  Pin configuration
#include <PinConfig.cpp>

// Devices configuration
#include <MainConfig.cpp>

// MySensors configuration
#include <MySConfig.cpp>

// ************************ END OF CONFIG **********************************

// **************************** INIT ***************************************

// Libraries
#include <MySensors.h>
#include <Vcc.h>
#ifdef ENABLE_SI7021
#include <SI7021.h>
#endif

// Object initialization
Vcc vcc(VccCorrection);
#ifdef ENABLE_SI7021
static SI7021 sensor;
#endif

// CHILD_ID
#ifdef ENABLE_VCC_MONITOR
#define CHILD_ID_VCC_VOLTAGE 1
#endif
#ifdef ENABLE_EXT_PWR_MONITOR
#define CHILD_ID_EXT_POWER 2
#endif
#ifdef ENABLE_BOOSTER_MONITOR
#define CHILD_ID_BOOSTER 3
#endif
#ifdef PHOTORES_PIN
#define CHILD_ID_LIGHT_LEVEL 4
#endif
#ifdef FRONT_PIR_PIN
#define CHILD_ID_FRONT_PIR 5
#endif
#ifdef ENABLE_SI7021
#define CHILD_ID_HUM 6
#define CHILD_ID_TEMP 7
#endif

// Messages
#ifdef CHILD_ID_VCC_VOLTAGE
MyMessage msgVccValue(CHILD_ID_VCC_VOLTAGE, V_VOLTAGE);
#endif
#ifdef CHILD_ID_EXT_POWER
MyMessage msgExtPower(CHILD_ID_EXT_POWER, V_STATUS);
#endif
#ifdef CHILD_ID_BOOSTER
MyMessage msgBooster(CHILD_ID_BOOSTER, V_STATUS);
#endif
#ifdef CHILD_ID_LIGHT_LEVEL
MyMessage msgLightLevel(CHILD_ID_LIGHT_LEVEL, V_LIGHT_LEVEL);
#endif
#ifdef CHILD_ID_FRONT_PIR
MyMessage msgFrontPir(CHILD_ID_FRONT_PIR, V_TRIPPED);
#endif
#ifdef CHILD_ID_HUM
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
#endif
#ifdef CHILD_ID_TEMP
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
#endif

// Vcc read status variables
float initial_vcc_voltage;
float vcc_voltage;
float last_vcc_voltage;
uint8_t nNoUpdatesVccLevel = 0;

// Booster status variables
#ifdef BOOSTER_PIN
bool boost_status = false;
bool last_boost_status = false;
uint8_t nNoUpdatesBooster = 0;
#endif

// External power status variable
bool ext_power = false;
bool last_ext_power = false;
uint8_t nNoUpdatesExtPwr = 0;

// First run
bool first_run = true;

// Smartleep
int8_t wake_up_mode;

// Photoresitor status variables
#ifdef PHOTORES_PIN
int lastLightLevel;
uint8_t nNoUpdatesLightLevel;
#endif

// Front PIR status variables
#ifdef FRONT_PIR_PIN
bool front_pir = false;
bool last_front_pir = false;
uint8_t nNoUpdatesFrontPir = 0;
#endif

// Batteries status variables
uint8_t batt_percent_value;
uint8_t last_batt_percent_value;
uint8_t nNoUpdatesBattPercent = 0;

// Motion led status variables
#ifdef ENABLE_MOTION_LED
bool motion_led_status = false;
bool last_motion_led_status = false;
int motion_led_brighteness_status;
#endif

// Power LED status variables
int pwr_led_brighteness_status;
unsigned long low_batt_led_on_start_time = 0;

// SI7021
#ifdef ENABLE_SI7021
static bool metric = true;
bool sensor_status = false;
uint8_t nNoUpdatesTemp = 0;
uint8_t nNoUpdatesHum = 0;
float temperature;
float humidity;
float last_temp = 0;
float last_hum = 0;
#endif

// ********************************* END OF INIT **********************************

// ****************************** CUSTOM FUNCTIONS ********************************

// Mean and smooth the analog reads
int analog_smooth(int PIN, int reads)
{
  int smoothed = 0;
  for (int i = 1; i <= reads; i++)
  {
    smoothed = (smoothed * (i - 1) + analogRead(PIN)) / i;
  }
  return smoothed;
}

// Mean VCC reads
float read_vcc(int reads = 5)
{
  float sum = 0;
  for (int i = 1; i <= reads; i++)
  {
    sum += vcc.Read_Volts();
    //Serial.println(sum);
  }
  return (float)round(sum / reads * 100) / 100;
}

// Wait before radio message (for CR2032 batteries)
void cr2032_wait()
{
  if (BATTERY_TYPE == 2)
  {
    wait(CR2032_RADIO_WAIT_TIME);
  }
}

// Read Vcc
void update_Vcc_level()
{
  vcc_voltage = read_vcc(MEAN_VCC_READS);
  if (first_run || vcc_voltage != last_vcc_voltage || nNoUpdatesVccLevel == FORCE_UPDATE_N_READS)
  {
    last_vcc_voltage = vcc_voltage;
    nNoUpdatesVccLevel = 0;
#ifdef ENABLE_VCC_MONITOR
    send(msgVccValue.set((float)vcc_voltage, 2), ack);
    cr2032_wait();
#endif
  }
  else
  {
    nNoUpdatesVccLevel++;
  }
#ifdef MY_DEBUG
  Serial.print("Mean Vcc:");
  Serial.println(vcc_voltage);
#endif
}

// Light Level read and update
#ifdef CHILD_ID_LIGHT_LEVEL
void update_light_level(int tol = 2, int reads = 10)
{
  int value = map(analog_smooth(PHOTORES_PIN, reads), 0, 1023, 0, 100);
#ifdef MY_DEBUG
  Serial.print("Light Level: ");
  Serial.println(value);
#endif
  if (value > lastLightLevel + tol || value < lastLightLevel - tol || nNoUpdatesLightLevel == FORCE_UPDATE_N_READS)
  {
    nNoUpdatesLightLevel = 0;
    lastLightLevel = value;
    send(msgLightLevel.set(lastLightLevel), ack);
    cr2032_wait();
  }
  else
  {
    nNoUpdatesLightLevel++;
  }
}
#endif

// Front PIR read and update
#ifdef CHILD_ID_FRONT_PIR
void update_fornt_pir()
{
  // Logic is reversed
  front_pir = digitalRead(FRONT_PIR_PIN) ? false : true;
  if (front_pir != last_front_pir || first_run || nNoUpdatesFrontPir == FORCE_UPDATE_N_READS)
  {
    last_front_pir = front_pir;
    nNoUpdatesFrontPir = 0;
    send(msgFrontPir.set(front_pir ? 1 : 0), ack);
    cr2032_wait();
#ifdef MY_DEBUG
    Serial.print("Front PIR: ");
    Serial.println(front_pir);
#endif
  }
  else
  {
    nNoUpdatesFrontPir++;
  }
}
#endif

// ************************** END OF CUSTOM FUNCTIONS *****************************/

// ******************** MYSESNSORS PRESENTANTION FUNCTION ***************************/

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BT-Pcb-328P-firmware", "1.0", ack);
  cr2032_wait();
// Register all sensors to gw (they will be created as child devices)
#ifdef CHILD_ID_VCC_VOLTAGE
  present(CHILD_ID_VCC_VOLTAGE, S_MULTIMETER, "Vcc Voltage", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_EXT_POWER
  present(CHILD_ID_EXT_POWER, S_BINARY, "External power", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_BOOSTER
  present(CHILD_ID_BOOSTER, S_BINARY, "Booster", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_LIGHT_LEVEL
  present(CHILD_ID_LIGHT_LEVEL, S_LIGHT_LEVEL, "Light Level", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_FRONT_PIR
  present(CHILD_ID_FRONT_PIR, S_MOTION, "Front Motion Sensor", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_HUM
  present(CHILD_ID_HUM, S_HUM, "Humidity", ack);
  cr2032_wait();
#endif
#ifdef CHILD_ID_TEMP
  present(CHILD_ID_TEMP, S_TEMP, "Temperature", ack);
  cr2032_wait();
#endif
#ifdef ENABLE_SI7021
  metric = getControllerConfig().isMetric;
#endif
}

// ******************** END OF MYSESNSORS PRESENTANTION FUNCTION ************************/

// ********************************** ARDUINO SETUP *************************************/

void setup()
{
#ifdef POWER_PIN
  pinMode(POWER_PIN, OUTPUT);
#endif
#ifdef EXT_PWR_SENSE_PIN
  pinMode(EXT_PWR_SENSE_PIN, INPUT);
#endif
#ifdef BOOSTER_PIN
  pinMode(BOOSTER_PIN, OUTPUT);
  digitalWrite(BOOSTER_PIN, LOW);
#endif
#ifdef PWR_LED_PIN
  pinMode(PWR_LED_PIN, OUTPUT);
  digitalWrite(PWR_LED_PIN, LOW);
#endif
#ifdef PHOTORES_PIN
  pinMode(PHOTORES_PIN, INPUT);
#endif
#ifdef FRONT_PIR_PIN
  pinMode(FRONT_PIR_PIN, INPUT);
#endif
#ifdef MOTION_LED_PIN
  pinMode(MOTION_LED_PIN, OUTPUT);
  digitalWrite(MOTION_LED_PIN, LOW);
#endif
  initial_vcc_voltage = vcc.Read_Volts();
#ifdef ENABLE_SI7021
#ifdef POWER_PIN
  digitalWrite(POWER_PIN, HIGH);
  wait(300);
#endif
  sensor_status = sensor.begin();
#endif
}

// ***************************** END OF ARDUINO SETUP ******************************

// ******************************* ARDUINO LOOP ************************************

void loop()
{
// Turn ON power PIN
#ifdef POWER_PIN
  digitalWrite(POWER_PIN, HIGH);
#endif

// Send heartbeat
#ifdef ENABLE_HEARTBEAT
  sendHeartbeat();
  cr2032_wait();
#endif

  // Read Vcc
  update_Vcc_level();

// Activate Booster if necessary
#ifdef BOOSTER_PIN
  if (BOOSTER_POLICY == 0)
  {
    boost_status = false;
  }
  if (BOOSTER_POLICY == 1)
  {
    boost_status = true;
  }
  if (BOOSTER_POLICY == 2)
  {
    boost_status = (vcc_voltage < BoostThreshold) ? true : false;
  }
  if (first_run || boost_status != last_boost_status || nNoUpdatesBooster == FORCE_UPDATE_N_READS)
  {
    digitalWrite(BOOSTER_PIN, (boost_status) ? HIGH : LOW);
    last_boost_status = boost_status;
    nNoUpdatesBooster = 0;
#ifdef ENABLE_BOOSTER_MONITOR
    send(msgBooster.set(boost_status ? 1 : 0), ack);
    cr2032_wait();
#endif
  }
  else
  {
    nNoUpdatesBooster++;
  }
#ifdef MY_DEBUG
  Serial.print("Booster:");
  Serial.println(boost_status);
#endif
#endif

  // Read light value
  update_light_level(0, 10);

  // Read Pir value
  update_fornt_pir();

// Motion LED
#ifdef ENABLE_MOTION_LED
  motion_led_status = front_pir;
  if (first_run || motion_led_status != last_motion_led_status)
  {
    last_motion_led_status = motion_led_status;
    if (motion_led_status)
    {
      if (MOTION_LED_BRIGHTNESS >= 0 && MOTION_LED_BRIGHTNESS <= 255)
      {
        motion_led_brighteness_status = MOTION_LED_BRIGHTNESS;
      }
      else if (MOTION_LED_BRIGHTNESS == -1)
      {
#ifdef CHILD_ID_LIGHT_LEVEL
        motion_led_brighteness_status = map(lastLightLevel, 0, 100, 50, 255);
#endif
#ifndef CHILD_ID_LIGHT_LEVEL
        motion_led_brighteness_status = 255;
#endif
      }
      else
      {
        motion_led_brighteness_status = 0;
      }
    }
    else
    {
      motion_led_brighteness_status = 0;
    }
    analogWrite(MOTION_LED_PIN, motion_led_brighteness_status);
  }
#ifdef MY_DEBUG
  Serial.print("Motion LED status:");
  Serial.println(motion_led_brighteness_status);
#endif
#endif

  // Detect external power presence. Logic is reversed: HIGH = no external power, LOW = external_power
  ext_power = digitalRead(EXT_PWR_SENSE_PIN) ? false : true;
  if (first_run || ext_power != last_ext_power || nNoUpdatesExtPwr == FORCE_UPDATE_N_READS)
  {
    last_ext_power = ext_power;
    nNoUpdatesExtPwr = 0;
#ifdef CHILD_ID_EXT_POWER
    send(msgExtPower.set(ext_power ? 1 : 0), ack);
    cr2032_wait();
#endif
#ifdef PWR_LED_PIN
    if (ext_power)
    {
      if (EXT_POWER_LED_BRIGHTNESS >= 0 && EXT_POWER_LED_BRIGHTNESS <= 255)
      {
        pwr_led_brighteness_status = EXT_POWER_LED_BRIGHTNESS;
      }
      else if (EXT_POWER_LED_BRIGHTNESS == -1)
      {
#ifdef CHILD_ID_LIGHT_LEVEL
        pwr_led_brighteness_status = map(lastLightLevel, 0, 100, 50, 255);
#endif
#ifndef CHILD_ID_LIGHT_LEVEL
        pwr_led_brighteness_status = 255;
#endif
      }
      else
      {
        pwr_led_brighteness_status = 0;
      }
    }
    else if (!ext_power)
    {
      pwr_led_brighteness_status = 0;
    }
    analogWrite(PWR_LED_PIN, pwr_led_brighteness_status);
#ifdef MY_DEBUG
    Serial.print("Power LED status (ext power):");
    Serial.println(pwr_led_brighteness_status);
#endif
#endif
  }
  else
  {
    nNoUpdatesExtPwr++;
  }
#ifdef MY_DEBUG
  Serial.print("External Power:");
  Serial.println(ext_power);
#endif

// Read battery level
#ifdef ENABLE_BATTERY_MONITOR
  if (ext_power)
  {
    batt_percent_value = 0;
  }
  else if (!ext_power)
  {
    // CR2032: V min = 2.4, V max = 3.0
    if (BATTERY_TYPE == 2)
    {
      batt_percent_value = (uint8_t)round(vcc.Read_Perc(2.4, 3.0));
    }
    // AAA bateries
    else if (BATTERY_TYPE == 1)
    {
      // Autodetect battery type
      if (AAA_BATT_CHEMISTRY == 0)
      {
        if (initial_vcc_voltage > 2.5)
        {
          batt_percent_value = (uint8_t)round(vcc.Read_Perc(1.2, 3.0));
        }
        else
        {
          batt_percent_value = (uint8_t)round(vcc.Read_Perc(1.2, 2.4));
        }
      }
      // Custom V min and V max
      else if (AAA_BATT_CHEMISTRY == 1)
      {
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(CUSTOM_V_MIN, CUSTOM_V_MAX));
      }
      // NiMH batteries (rechargeable)
      else if (AAA_BATT_CHEMISTRY == 2)
      {
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(1.2, 2.4));
      }
      // Alkaline batteries (disposable)
      else if (AAA_BATT_CHEMISTRY == 3)
      {
        batt_percent_value = (uint8_t)round(vcc.Read_Perc(1.2, 3.0));
      }
      // Undefined battery chemistry
      else
      {
        batt_percent_value = 0;
      }
    }
    // undefined battery type
    else
    {
      batt_percent_value = 0;
    }
  }
  if (first_run || batt_percent_value != last_batt_percent_value || nNoUpdatesBattPercent == FORCE_UPDATE_N_READS)
  {
    nNoUpdatesBattPercent = 0;
    last_batt_percent_value = batt_percent_value;
    sendBatteryLevel(batt_percent_value);
    cr2032_wait();
  }
  else
  {
    nNoUpdatesBattPercent++;
  }
#ifdef MY_DEBUG
  Serial.print("Battery Percent:");
  Serial.println(batt_percent_value);
#endif
#endif

// Low battery pwr LED blink
#ifdef PWR_LED_PIN
  if (!ext_power)
  {
    if (batt_percent_value <= LOW_BATTERY_THRESHOLD)
    {
      if (LOW_BATTERY_LED_BRIGHTNESS >= 0 && LOW_BATTERY_LED_BRIGHTNESS <= 255)
      {
        pwr_led_brighteness_status = LOW_BATTERY_LED_BRIGHTNESS;
      }
      else if (LOW_BATTERY_LED_BRIGHTNESS == -1)
      {
#ifdef CHILD_ID_LIGHT_LEVEL
        pwr_led_brighteness_status = map(lastLightLevel, 0, 100, 50, 255);
#endif
#ifndef CHILD_ID_LIGHT_LEVEL
        pwr_led_brighteness_status = 255;
#endif
      }
      else
      {
        pwr_led_brighteness_status = 0;
      }
    }
    else
    {
      pwr_led_brighteness_status = 0;
    }
    analogWrite(PWR_LED_PIN, pwr_led_brighteness_status);
    if (pwr_led_brighteness_status != 0)
    {
      low_batt_led_on_start_time = millis();
    }
#ifdef MY_DEBUG
    Serial.print("Power LED status (low battery):");
    Serial.println(pwr_led_brighteness_status);
#endif
  }
#endif

// Read temp and hum
#ifdef ENABLE_SI7021
  if (sensor_status)
  {
    temperature = float(metric ? sensor.getCelsiusHundredths() : sensor.getFahrenheitHundredths()) / 100.0;
    humidity = float(sensor.getHumidityBasisPoints()) / 100.0;
    if (first_run || last_temp != temperature || nNoUpdatesTemp == FORCE_UPDATE_N_READS)
    {
      last_temp = temperature;
      nNoUpdatesTemp = 0;
      send(msgTemp.set(temperature, 2), ack);
      cr2032_wait();
    }
    else
    {
      nNoUpdatesTemp++;
    }
#ifdef MY_DEBUG
    Serial.print("Temp:");
    Serial.println(temperature);
#endif
    if (first_run || last_hum != humidity || nNoUpdatesHum == FORCE_UPDATE_N_READS)
    {
      last_hum = humidity;
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity, 2), ack);
      cr2032_wait();
    }
    else
    {
      nNoUpdatesHum++;
    }
#ifdef MY_DEBUG
    Serial.print("Hum:");
    Serial.println(humidity);
#endif
  }
  else
  {
#ifdef MY_DEBUG
    Serial.print("SI7021 status:");
    Serial.println(sensor_status);
#endif
  }
#endif

// Turn OFF power pin before sleeping
#ifdef POWER_PIN
  digitalWrite(POWER_PIN, LOW);
#endif

  // Before sleeping turn of the PWR LED if battery il LOW
  if (!ext_power && pwr_led_brighteness_status != 0)
  {
    while (millis() < low_batt_led_on_start_time + LOW_BATTERY_BLINK_TIME)
    {
      continue;
    }
    pwr_led_brighteness_status = 0;
    low_batt_led_on_start_time = 0;
    analogWrite(PWR_LED_PIN, pwr_led_brighteness_status);
  }
  // Set first run to false
  if (first_run)
  {
    first_run = false;
  }

  // Smartsleep
  wake_up_mode = smartSleep(1, CHANGE, UPDATE_INTERVAL);
}
// **************************** END OF LOOP *********************************