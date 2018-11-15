#include "Arduino.h"
#include "pid.h"
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "tactile.h"

#define ADC_PIN A0
#define OUTPUT_PIN 10
#define LOAD_RESISTANCE 10
#define DELAY 500
#define OLED_RESET 4

#define TARGET_CURRENT 0.2f

PIDController pidController(15, 2, 0, 0);
Adafruit_SSD1306 display(OLED_RESET);

Tactile button0(8);  
Tactile button1(9);
Tactile button2(7);

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
float targetCurrent = 0.0f; //We begin with target o 0A

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

void setup()
{
    button0.start();
    button1.start();
    button2.start();

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
}

float joules = 0;

float getVin(float vout, float r1, float r2) {
    return (vout * (r1 + r2)) / r2;
}

float getFilteredV() {
    static int32_t filteredAdc = -1;
    uint32_t adcOutput = analogRead(ADC_PIN);

    if (filteredAdc == -1 || abs(filteredAdc - adcOutput) > 250)
    {
        filteredAdc = adcOutput;
    }

    filteredAdc = smooth(adcOutput, 0.91, filteredAdc);

    return getVin(map(filteredAdc, 0, 1023, 0, 5000) / 1000.f, 3235, 1000);
}

// uint32_t nextLooptime

void loop()
{
    button0.loop();
    button1.loop();
    button2.loop();

    static float voltage = 0;
    static float current = 0;
    static float requitedR = 0;

    if (button0.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetCurrent -= 0.1f;

        if (targetCurrent < 0) {
            targetCurrent = 0;
        }
    }

    if (button1.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetCurrent += 0.1f;

        if (targetCurrent > 2.0f) {
            targetCurrent = 2.0f;
        }
    }

    static uint32_t nextPidTask = millis();

    if (millis() > nextPidTask) {

        voltage = getFilteredV();
        current = getPowerFactor() * voltage / LOAD_RESISTANCE;

        requitedR = voltage / TARGET_CURRENT;
        float ff = (255 * LOAD_RESISTANCE / requitedR) / TARGET_CURRENT; 
        pidController.setFfGain(ff);
        pidController.setSetpoint(targetCurrent);
        int outputCandidate = pidController.compute(current, millis());

        joules += (voltage * current) * (DELAY / 1000.f);

        setPower(outputCandidate);

        if (voltage < 10.5f) {
            stopPower();
        }

        nextPidTask = millis() + DELAY;
    }

    static uint32_t nextSerialTask = millis();

    if (millis() > nextSerialTask) {

        Serial.print(outputPower);
        Serial.print(" : ");
        Serial.print(pidController.getIterm());
        Serial.print(" : ");
        Serial.print(pidController.getPterm());
        Serial.println(" : ");

        nextSerialTask = millis() + 200;
    }

    // Serial.print(voltage);
    // Serial.print(" : ");
    // Serial.print(current);
    // Serial.print(" : ");
    // Serial.print(joules);
    // Serial.print(" : ");
    // Serial.print(outputCandidate);
    // Serial.print(" : ");
    // Serial.print(pidController.getIterm());
    // Serial.print(" : ");
    // Serial.print(ff);
    // Serial.print(" : ");
    // Serial.print(requitedR);
    // Serial.println(" : ");
    // delay(DELAY);

    static uint32_t nextOledTask = millis();

    if (millis() > nextOledTask) {

        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("Vin: ");
        display.print(voltage);

        display.setCursor(0, 12);
        display.print("I: ");
        display.print(current);

        display.setCursor(0, 54);
        display.print("Current: ");
        display.print(targetCurrent);

        display.display();

        nextOledTask = millis() + 150;
    }
}
