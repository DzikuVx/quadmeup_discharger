#include "Arduino.h"
#include "pid.h"
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "tactile.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#define ADC_PIN A0
#define OUTPUT_PIN 10
#define LOAD_RESISTANCE 2.5f
#define MAX_VOLTAGE 20.0f
#define MAX_CURRENT MAX_VOLTAGE / LOAD_RESISTANCE
#define DELAY 500
#define OLED_RESET 4
#define ONE_WIRE_BUS 2
#define DEFAULT_VOLTAGE_SCALE (3150.0f + 1000.0f) / 1000.0f
#define MAX_VOLTAGE_SCALE 5.0f
#define MIN_VOLTAGE_SCALE 1.0f

#define EEPROM_ADDRESS_VOLTAGE_CALIBRATION 0

#define TARGET_CURRENT 0.2f

PIDController pidController(15, 3, 0, 0);
Adafruit_SSD1306 display(OLED_RESET);

Tactile button0(8);  // left
Tactile button1(9);  // right
Tactile button2(7);  // setting
Tactile button3(6);  // Start/Stop

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
   const byte* p = (const byte*)(const void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
   return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
   byte* p = (byte*)(void*)&value;
   int i;
   for (i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return i;
}

int32_t smooth(uint32_t data, float filterVal, float smoothedVal)
{
    if (filterVal > 1)
    {
        filterVal = .99;
    }
    else if (filterVal <= 0)
    {
        filterVal = 0;
    }

    smoothedVal = (data * (1 - filterVal)) + (smoothedVal * filterVal);

    return (uint32_t)smoothedVal;
}

uint8_t outputPower = 0;
float targetCurrent = 0.2f; //We begin with target o 0A
float cutoffVoltage = 3.0f;
float voltageScale = 0;

void setPower(uint8_t power) {
    outputPower = power;
    analogWrite(OUTPUT_PIN, outputPower);
}

float getPowerFactor() {
    return (float) outputPower / 255.0f;
}

void stopPower() {
    outputPower = 0;
    analogWrite(OUTPUT_PIN, 0);
}

void eepromStoreVoltageScale(void) {
    int32_t tmp = voltageScale * 1000;
    EEPROM_writeAnything(EEPROM_ADDRESS_VOLTAGE_CALIBRATION, tmp);
}

void setup()
{
    button0.start();
    button1.start();
    button2.start();

    button3.start();

    pinMode(ADC_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);
    Serial.begin(115200);

    pidController.setItermProperties(-20, 20);
    pidController.setSetpoint(targetCurrent);
    setPower(0);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();

    int32_t tmp;
    EEPROM_readAnything(EEPROM_ADDRESS_VOLTAGE_CALIBRATION, tmp);
    voltageScale = tmp / 1000.0f;

    if (voltageScale < MIN_VOLTAGE_SCALE || voltageScale > MAX_VOLTAGE_SCALE) {
        voltageScale = DEFAULT_VOLTAGE_SCALE;
        eepromStoreVoltageScale();
    }
}

float joules = 0;
float mAh = 0;
float Wh = 0;
float power = 0;

float getVin(float vout) {
    return vout * voltageScale;
}

float currentVoltage = 0.0f;

void processVolatge() {
    static int32_t filteredAdc = -1;
    uint32_t adcOutput = analogRead(ADC_PIN);

    if (filteredAdc == -1 || abs(filteredAdc - adcOutput) > 250)
    {
        filteredAdc = adcOutput;
    }

    filteredAdc = smooth(adcOutput, 0.99, filteredAdc);

    currentVoltage = getVin(map(filteredAdc, 0, 1023, 0, 5000) / 1000.f);
}

float getFilteredV() {
    return currentVoltage;
}

enum settingPages {
    SETTING_PAGE_CURRENT = 0,
    SETTING_PAGE_CUTOFF,
    SETTING_PAGE_VOLTAGE_SCALE,
    SETTING_PAGE_LAST
};

enum runningStates {
    RUNNING_STATE_IDLE = 0,
    RUNNING_STATE_DISCHARGE,
    RUNNING_STATE_LAST
};

enum inputStates {
    INPUT_STATE_NO_INPUT,
    INPUT_STATE_OK,
    INPUT_STATE_THRESHOLD_REACHED
};

int currentSettingPage = SETTING_PAGE_CURRENT;
int currentRunningState = RUNNING_STATE_IDLE;
int inputState = INPUT_STATE_NO_INPUT;

float currentTemperature = 0.0f;

void loop()
{
    processVolatge();

    if (getFilteredV() < 0.5f) {
        inputState = INPUT_STATE_NO_INPUT;
    }

    if (inputState == INPUT_STATE_NO_INPUT && getFilteredV() >= 0.5f) {
        inputState = INPUT_STATE_OK;
    }

    if (inputState == INPUT_STATE_OK && currentRunningState == RUNNING_STATE_DISCHARGE && getFilteredV() < cutoffVoltage) {
        inputState = INPUT_STATE_THRESHOLD_REACHED;
    }

    button0.loop();
    button1.loop();
    button2.loop();
    button3.loop();

    static float voltage = 0;
    static float current = 0;
    static float requitedR = 0;

    if (button0.getState() == TACTILE_STATE_SHORT_PRESS) {

        if (currentSettingPage == SETTING_PAGE_CURRENT) {
            targetCurrent -= 0.1f;
            if (targetCurrent < 0) {
                targetCurrent = 0;
            }
        } else if (currentSettingPage == SETTING_PAGE_CUTOFF) {
            cutoffVoltage -= 0.1f;
            if (cutoffVoltage < 0.5) {
                cutoffVoltage = 0.5;
            }
        } else if (currentSettingPage == SETTING_PAGE_VOLTAGE_SCALE) {
            voltageScale -= 0.01f;
            if (voltageScale < MIN_VOLTAGE_SCALE) {
                voltageScale = MIN_VOLTAGE_SCALE;
            }
            eepromStoreVoltageScale();
        }
    }

    if (button1.getState() == TACTILE_STATE_SHORT_PRESS) {
        if (currentSettingPage == SETTING_PAGE_CURRENT) {
            targetCurrent += 0.1f;
            if (targetCurrent > MAX_CURRENT) {
                targetCurrent = MAX_CURRENT;
            }
        } else if (currentSettingPage == SETTING_PAGE_CUTOFF) {
            cutoffVoltage += 0.1f;
            if (cutoffVoltage > MAX_VOLTAGE) {
                cutoffVoltage = MAX_VOLTAGE;
            }
        } else if (currentSettingPage == SETTING_PAGE_VOLTAGE_SCALE) {
            voltageScale += 0.01f;
            if (voltageScale > MAX_VOLTAGE_SCALE) {
                voltageScale = MAX_VOLTAGE_SCALE;
            }
            eepromStoreVoltageScale();
        }
    }

    if (button2.getState() == TACTILE_STATE_SHORT_PRESS) {
        currentSettingPage++;
        if (currentSettingPage == SETTING_PAGE_LAST) {
            currentSettingPage = SETTING_PAGE_CURRENT;
        }
    }

    if (button3.getState() == TACTILE_STATE_LONG_PRESS) {
        currentRunningState++;
        if (currentRunningState == RUNNING_STATE_LAST) {
            currentRunningState = RUNNING_STATE_IDLE;
        }
        if (currentRunningState == RUNNING_STATE_DISCHARGE) {
            pidController.resetIterm();
            joules = 0;
            Wh = 0;
            mAh = 0;
        }
    }

    static uint32_t nextPidTask = millis();
    static uint32_t nextTemperatureTask = millis();

    if (millis() > nextTemperatureTask) {
        sensors.requestTemperatures();
        currentTemperature = sensors.getTempCByIndex(0);

        nextTemperatureTask = millis() + 1000;
    }

    if (millis() > nextPidTask) {

        sensors.requestTemperatures();

        voltage = getFilteredV();
        current = getPowerFactor() * voltage / LOAD_RESISTANCE;

        requitedR = voltage / TARGET_CURRENT;
        float ff = (255 * LOAD_RESISTANCE / requitedR) / TARGET_CURRENT; 
        pidController.setFfGain(ff);
        pidController.setSetpoint(targetCurrent);
        int outputCandidate = pidController.compute(current, millis());

        joules += (voltage * current) * (DELAY / 1000.f);
        Wh = joules / 3600.0f;
        mAh += current * (DELAY / 1000.f) * 1000 / 3600;
        power = voltage * current;

        if (inputState == INPUT_STATE_OK && currentRunningState == RUNNING_STATE_DISCHARGE) {
            setPower(outputCandidate);
        } else {
            stopPower();
        }

        nextPidTask = millis() + DELAY;
    }

    static uint32_t nextSerialTask = millis();

    if (millis() > nextSerialTask) {

        // Serial.print(voltageAdc.getRawValue());
        // Serial.print(" : ");
        // Serial.println(voltageAdc.getValue());

        nextSerialTask = millis() + 200;
    }

    static uint32_t nextOledTask = millis();

    if (millis() > nextOledTask) {

        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("Vin: ");
        display.print(voltage);

        display.setCursor(64, 0);
        display.print("I: ");
        display.print(current);

        display.setCursor(0, 10);
        display.print("T: ");
        display.print(currentTemperature);

        display.setCursor(64, 10);
        display.print("P: ");
        display.print(power);

        display.setCursor(0, 22);
        display.print("Wh: ");
        display.print(Wh);
        display.setCursor(0, 32);
        display.print("mAh: ");
        display.print(mAh);

        if (currentRunningState == RUNNING_STATE_IDLE) {
            display.setCursor(48, 44);
            display.print("IDLE");
        } else if (currentRunningState == RUNNING_STATE_DISCHARGE) {
            display.setCursor(16, 44);
            display.print("-- DISCHARGE --");
        }

        if (currentSettingPage == SETTING_PAGE_CURRENT) {
            display.setCursor(0, 54);
            display.print("Current: ");
            display.print(targetCurrent);
        } else if (currentSettingPage == SETTING_PAGE_CUTOFF) {
            display.setCursor(0, 54);
            display.print("Cutoff: ");
            display.print(cutoffVoltage);
        } else if (currentSettingPage == SETTING_PAGE_VOLTAGE_SCALE) {
            display.setCursor(0, 54);
            display.print("V scale: ");
            display.print(voltageScale);
        }

        display.display();

        nextOledTask = millis() + 150;
    }
}
