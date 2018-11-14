#include "Arduino.h"
#include "pid.h"

#define ADC_PIN A0
#define OUTPUT_PIN 10
#define LOAD_RESISTANCE 10
#define DELAY 500

#define TARGET_CURRENT 0.2f

PIDController pidController(5, 1, 0, 0);

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

    pidController.setItermProperties(-20, 20);
    pidController.setSetpoint(TARGET_CURRENT);
    setPower(40);
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

    float requitedR = voltage / TARGET_CURRENT;
    float ff = (255 * LOAD_RESISTANCE / requitedR) / TARGET_CURRENT; 
    pidController.setFfGain(ff);
    int outputCandidate = pidController.compute(current, millis());

    joules += (voltage * current) * (DELAY / 1000.f);

    setPower(outputCandidate);

    if (voltage < 10.5f) {
        stopPower();
    }

    Serial.print(voltage);
    Serial.print(" : ");
    Serial.print(current);
    Serial.print(" : ");
    Serial.print(joules);
    Serial.print(" : ");
    Serial.print(outputCandidate);
    Serial.print(" : ");
    Serial.print(pidController.getIterm());
    Serial.print(" : ");
    Serial.print(ff);
    Serial.print(" : ");
    Serial.print(requitedR);
    Serial.println(" : ");
    delay(DELAY);
}
