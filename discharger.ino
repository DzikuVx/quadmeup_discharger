#include "Arduino.h"

#define ADC_PIN A0
#define OUTPUT_PIN 10
#define LOAD_RESISTANCE 10
#define DELAY 100

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
    pinMode(ADC_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);
    Serial.begin(115200);

    setPower(255);
}

float joules = 0;

float getVin(float vout, float r1, float r2) {
    return (vout * (r1 + r2)) / r2;
}

void loop()
{
    

    static int32_t filteredAdc = -1;
    uint32_t adcOutput = analogRead(ADC_PIN);

    if (filteredAdc == -1 || abs(filteredAdc - adcOutput) > 250)
    {
        filteredAdc = adcOutput;
    }

    filteredAdc = smooth(adcOutput, 0.91, filteredAdc);

    float voltage = getVin(map(filteredAdc, 0, 1023, 0, 5000) / 1000.f, 3235, 1000);
    float current = getPowerFactor() * voltage / LOAD_RESISTANCE;
    joules += (voltage * current) * (DELAY / 1000.f);

    // if (voltage < 4.00f) {
    //     stopPower();
    // } else {
    //     setPower(255);
    // }

    Serial.print(voltage);
    Serial.print(" : ");
    Serial.print(current);
    Serial.print(" : ");
    Serial.print(joules);
    Serial.println(" : ");
    delay(DELAY);
}
