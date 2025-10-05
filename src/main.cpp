#include <Arduino.h>

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TM1652.h>
#include <TM16xxDisplay.h>
#include <Wire.h>
#include <SPI.h>
#include <Dadafruit_CAP1106.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 33
OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);
TM1652 module(17);                 // data pin = 10
TM16xxDisplay display(&module, 2); // TM16xx object, 2 digits for display object
Adafruit_CAP1106 cap = Adafruit_CAP1106();

#define SET_TEMP_MIN 20
#define SET_TEMP_MAX 35
#define TEMP_HYSTERSIS 1

#define FAN_SLOW_PIN 25
#define FAN_MED_PIN  26
#define FAN_FAST_PIN 27
#define HOT_VALVE_PIN 14
#define COLD_VALVE_PIN 12
#define BUZZER_PIN 4
#define KEYS_BL_PIN 13

uint8_t device_onoff = 0; // 0: off, 1: on
float roomTemp = 0;
bool roomTempValid = false;
float outTemp = 0;
float setTemp = 21;
unsigned long setTemp_ts = 0;
int fanSpeed = 0;            // 0:off, 1:low, 2:med, 3:high, 4:auto
uint8_t thermostat_mode = 0; // 1: eco, 2: fan-only, 3: heating, 4: cooling,
uint8_t display_mode = 0;    // 0: Room-temp, 1: set-Temp

/*
 * The setup function. We only start the sensors here
 */
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  Serial.println("Thermostat Demo");

  pinMode(FAN_SLOW_PIN, OUTPUT);
  pinMode(FAN_MED_PIN, OUTPUT);
  pinMode(FAN_FAST_PIN, OUTPUT);
  pinMode(HOT_VALVE_PIN, OUTPUT);
  pinMode(COLD_VALVE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(KEYS_BL_PIN, OUTPUT);
  digitalWrite(FAN_SLOW_PIN, 0);
  digitalWrite(FAN_MED_PIN, 0);
  digitalWrite(FAN_FAST_PIN, 0);
  digitalWrite(HOT_VALVE_PIN, 0);
  digitalWrite(COLD_VALVE_PIN, 0);
  digitalWrite(KEYS_BL_PIN, 0);
  noTone(BUZZER_PIN);
  // Start up the library
  sensors.begin();
  // display.clear();
  // display.setCursor(0); 
  // display.print("--");

  // Initialize the sensor, if using i2c you can pass in the i2c address
  if (!cap.begin())
  {
    Serial.println("CAP1106 not found");
  }
  else
  {
    delay(100); // wait a moment to settle power-up noises
      
  cap.writeRegister(CAP1106_MTBLK, 0); // allow multiple touches
  // cap.writeRegister(CAP1106_STANDBYCFG, 0x30); // speed up a bit
    cap.writeRegister(CAP1106_SENSITIVITY, 0x2F); // set Gain
    // cap.writeRegister(0x22, 0x24); // MAX_DUR = 1120ms
    // cap.writeRegister(0x24, 0x59); // AVG = 32
    cap.writeRegister(0x2F, 0x0A);   // Each Sensor Input X Threshold register is updated individually
    cap.writeRegister(0x21, 0xFE); // disable CS1
    cap.writeRegister(0x31, 60); // set threshold for cs1
    cap.writeRegister(0x32, 90); // set threshold for cs2 
    cap.writeRegister(0x33, 70); // set threshold for cs3 
    cap.writeRegister(0x34, 40); // set threshold for cs4 
    cap.writeRegister(0x35, 40); // set threshold for cs5 
    // cap.writeRegister(0x26, 0x3f); // recalibrate all sensors 
  }
}

void touch_task()
{
  uint8_t touch;
  uint8_t btn[5];
  static uint8_t btn_prev[5];
  static uint64_t task_ts_prev = 0;
  const uint32_t touchTaskInterval_ms = 50;

  if (millis() - task_ts_prev > touchTaskInterval_ms || task_ts_prev == 0)
  {
    noTone(BUZZER_PIN);
    task_ts_prev = millis();
    touch = cap.touched();
    for (uint8_t i = 1; i < 6; i++) // check which key is pressed
    {
      btn[i] = (touch >> i) & 1;
      if ((btn[i] != btn_prev[i]) && btn[i])
      {
        Serial.printf("btn[%d]=%d\n\r", i, btn[i]);
        
        switch (i)
        {
        case 1:                  // decrease setTemp
          if (device_onoff == 0) break;
          tone(BUZZER_PIN, 5000);
          setTemp_ts = millis(); // time-stamp of entring setTemp display mode
          if (display_mode != 1)
          {
            display_mode = 1; // switch to setTemp display Mode and exit
          }
          else
          {
            if (setTemp > SET_TEMP_MIN)
              setTemp--;
          }
          break;

        case 5:                  // increase setTemp
          if (device_onoff == 0) break;
          tone(BUZZER_PIN, 5000);
          setTemp_ts = millis(); // time-stamp of entring setTemp display mode
          if (display_mode != 1)
          {
            display_mode = 1; // switch to setTemp display Mode and exit
          }
          else
          {
            if (setTemp < SET_TEMP_MAX)
              setTemp++;
          }
          Serial.print("setTemp = "); Serial.println(setTemp);
          break;
        
        case 2: // change fanSpeed
          if (device_onoff == 0) break;
          tone(BUZZER_PIN, 5000);
          if (fanSpeed < 4)
            fanSpeed++;
          else
            fanSpeed = 0; // reset
          Serial.printf("fanSpeed = %d\n\r", fanSpeed);
          break;
  
        case 3:
          tone(BUZZER_PIN, 5000);
          if (device_onoff == 0) 
          {
            device_onoff = 1; // turn on device
          }
          else
          {
            device_onoff = 0;
          }
          Serial.printf("device_onoff = %d\n\r", device_onoff);
          break;
        
        case 4:
          if (device_onoff == 0) break;
          tone(BUZZER_PIN, 5000);
          thermostat_mode++;
          if (thermostat_mode == 4) thermostat_mode = 0; // max 4 different modes
          Serial.printf("thermostat_mode = %d\n\r", thermostat_mode);
          break;

        } // switch
      } // if
      btn_prev[i] = btn[i];
    } // for
  }
}

void readTemp_task()
{
  static uint64_t task_ts_prev = 0;
  const uint32_t readTempTaskInterval_ms = 15000;

  if (millis() - task_ts_prev > readTempTaskInterval_ms || task_ts_prev == 0)
  {
    task_ts_prev = millis();
    sensors.requestTemperatures(); // Send the command to get temperatures
    roomTemp = sensors.getTempCByIndex(0); // 0: external sensor
    outTemp = sensors.getTempCByIndex(1);
    if (roomTemp == DEVICE_DISCONNECTED_C)
    {
      Serial.println("Error: Could not read temperature data");
      roomTempValid = false;
    }
    else
    {
      roomTempValid = true;
      Serial.print("roomTemp = ");
      Serial.println(roomTemp);
      Serial.print("outTemp = ");
      Serial.println(outTemp);
    }
  }
}

void display_task()
{
  int roomTempInt = 0;
  // int outTempInt = 0;
  // int setTempInt = 0;
  char roomTempStr[4] = {0};
  static uint64_t task_ts_prev = 0;
  const uint32_t displayTaskInterval_ms = 50;

  if (millis() - task_ts_prev > displayTaskInterval_ms || task_ts_prev == 0)
  {
    task_ts_prev = millis();

    // display set Temp
    if (display_mode == 1) 
    {
      if (millis() - setTemp_ts > 5000)
      {
        display_mode = 0; // exit setTempShow mode
      }
      else
      {
        // setTempInt = round(setTemp);
        display.setCursor(0);
        display.print(setTemp);
        
      }
    }

    // display room temp
    else if (display_mode == 0)
    {
      roomTempInt = round(roomTemp);
      //display.setCursor(0); // caused DP flickering
      sprintf(roomTempStr, "%2d.", roomTempInt);
      display.print(roomTempStr);
      // display.printf("%2d.", roomTempInt);//append dot(.) as degC
    }

    // update segment 3 leds
    uint8_t segment_3_val = 0;
    if (thermostat_mode == 0)      segment_3_val |= (1 << 0);
    else if (thermostat_mode == 1) segment_3_val |= (1 << 1);
    else if (thermostat_mode == 2) segment_3_val |= (1 << 2);
    else if (thermostat_mode == 3) segment_3_val |= (1 << 3);
    if (fanSpeed == 1)      segment_3_val |= (1 << 4);
    else if (fanSpeed == 2) segment_3_val |= (1 << 5);
    else if (fanSpeed == 3) segment_3_val |= (1 << 6);
    else if (fanSpeed == 4) segment_3_val |= (1 << 7);   
    
    module.setSegments(segment_3_val, 2);

    // update segment_4_val
    uint8_t segment_4_val = 0;
    module.setSegments(segment_4_val, 3);
  }
}

void drive_task()
{
  static uint64_t task_ts_prev = 0;
  const uint32_t driveTaskInterval_ms = 1000;

  if (millis() - task_ts_prev > driveTaskInterval_ms)
  {
    task_ts_prev = millis();

    // drive fan relays
    switch (fanSpeed){
      case 0: // off 
        digitalWrite(FAN_SLOW_PIN, 0);
        digitalWrite(FAN_MED_PIN, 0);
        digitalWrite(FAN_FAST_PIN, 0);
        break;
      case 1: // slow
        digitalWrite(FAN_SLOW_PIN, 1);
        digitalWrite(FAN_MED_PIN, 0);
        digitalWrite(FAN_FAST_PIN, 0);
        break;
      case 2: //medium
        digitalWrite(FAN_SLOW_PIN, 0);
        digitalWrite(FAN_MED_PIN, 1);
        digitalWrite(FAN_FAST_PIN, 0);
        break;
      case 3: // fast
        digitalWrite(FAN_SLOW_PIN, 0);
        digitalWrite(FAN_MED_PIN, 0);
        digitalWrite(FAN_FAST_PIN, 1);
        break;
    }

    // drive hot/cold valves
    if (thermostat_mode == 2) // hot
    {
      digitalWrite(COLD_VALVE_PIN, 0);
      if (roomTemp < setTemp - TEMP_HYSTERSIS)
        digitalWrite(HOT_VALVE_PIN, 1); // turn on
      else if (roomTemp > setTemp + TEMP_HYSTERSIS)
        digitalWrite(HOT_VALVE_PIN, 0); // turn off
    }
    else if (thermostat_mode == 3) // cold
    {
      digitalWrite(HOT_VALVE_PIN, 0);
      if (roomTemp > setTemp - TEMP_HYSTERSIS)
        digitalWrite(COLD_VALVE_PIN, 1); // turn on
      else if (roomTemp < setTemp + TEMP_HYSTERSIS)
        digitalWrite(COLD_VALVE_PIN, 0); // turn off
    }
  }
}

uint8_t i = 0;
  /*
   * Main function, get and show the temperature
   */
  void loop(void)
  {
    
    touch_task();
    readTemp_task();
    if (device_onoff == 1)
    {
      digitalWrite(KEYS_BL_PIN, 1);
      display.setIntensity(6);
      display_task();
      drive_task();
    }
    else
    {
      digitalWrite(KEYS_BL_PIN, 0);
      display.clear();
      digitalWrite(FAN_SLOW_PIN, 0);
      digitalWrite(FAN_MED_PIN, 0);
      digitalWrite(FAN_FAST_PIN, 0);
      digitalWrite(HOT_VALVE_PIN, 0);
      digitalWrite(COLD_VALVE_PIN, 0);
    }
    
    // Serial.printf("reg 0x10-0x15: \n\r");
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x10));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x11));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x12));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x13));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x14));
    // Serial.printf("%d \n\r", (int8_t)cap.readRegister(0x15));
    
    // Serial.printf("reg 0x30-0x35: \n\r");
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x30));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x31));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x32));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x33));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x34));
    // Serial.printf("%d \n\r", (int8_t)cap.readRegister(0x35));

    // Serial.printf("reg 0x50-0x55 (base count): \n\r");
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x50));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x51));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x52));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x53));
    // Serial.printf("%d \t", (int8_t)cap.readRegister(0x54));
    // Serial.printf("%d \n\r", (int8_t)cap.readRegister(0x55));

    // Serial.printf("reg 0x20: %x\n\r", (int8_t)cap.readRegister(0x20));
    // Serial.printf("reg 0x44: %x\n\r", (int8_t)cap.readRegister(0x44));
    
    
    // delay(100);
  }
