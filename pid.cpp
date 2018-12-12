#include "pid.h"
#include <Arduino.h>

PIDController::PIDController(float pGain, float iGain, float dGain, float ffGain)
{
    _pGain = pGain;
    _iGain = iGain;
    _dGain = dGain;
    _ffGain = ffGain;

    _minIterm = -250;
    _maxIterm = 250;

    _min = 0;
    _max = 255;
    _previousError = 0;

    _outputThreshold = NULL;
    _isNegativeFeedback = false;
}

void PIDController::setIsNegativeFeedback(bool value)
{
    _isNegativeFeedback = value;
}

void PIDController::setOutputThreshold(int value)
{
    _outputThreshold = value;
}

void PIDController::setProperties(int minOutput, int maxOutput)
{
    _min = minOutput;
    _max = maxOutput;
}

void PIDController::setItermProperties(float minIterm, float maxIterm) 
{
    _minIterm = minIterm;
    _maxIterm = maxIterm;
}

void PIDController::setSetpoint(float setpoint) 
{
    _setpoint = setpoint;
}

void PIDController::setFfGain(float ffGain) 
{
    _ffGain = ffGain;
}

int PIDController::compute(float measurement, unsigned long timestamp)
{
    int output = 0;
    float error = _setpoint - measurement;

    //Do not run update if pid loop is called too often
    float dT = (timestamp - _prevExecutionMillis) / 1000.0f;

    //pTerm
    _pTerm = error * _pGain;
    output += (int) _pTerm;

    //Apply and constrain iTerm
    _iTerm += error * _iGain * dT;
    _iTerm = constrain(_iTerm, _minIterm, _maxIterm);
    output += (int) _iTerm;

    //dTerm
    _dTerm = (float)(error - _previousError) * _dGain * dT;
    output += _dTerm;

    //ffTerm
    _ffTerm = (float) _setpoint * _ffGain;
    output += _ffTerm;

    _previousError = error;
    _previousMeasurement = measurement;
    _prevExecutionMillis = timestamp;

    if (_isNegativeFeedback) {
        output = output * -1;
    }

    if (_outputThreshold != NULL && output < _outputThreshold) {
        output = _min;
    }

    return constrain(output, _min, _max);
}

void PIDController::resetIterm() {
    _iTerm = 0;
}

float PIDController::getPterm() {
    return _pTerm;
}

float PIDController::getIterm() {
    return _iTerm;
}

float PIDController::getDterm() {
    return _dTerm;
}