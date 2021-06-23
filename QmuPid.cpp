#include "QmuPid.h"
#include <Arduino.h>

QmuPid::QmuPid(float pGain, float iGain, float dGain, float ffGain)
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

    _outputThreshold = 0;
    _isNegativeFeedback = false;
}

void QmuPid::setIsNegativeFeedback(bool value)
{
    _isNegativeFeedback = value;
}

void QmuPid::setOutputThreshold(int value)
{
    _outputThreshold = value;
}

void QmuPid::setProperties(int minOutput, int maxOutput)
{
    _min = minOutput;
    _max = maxOutput;
}

void QmuPid::setItermProperties(float minIterm, float maxIterm) 
{
    _minIterm = minIterm;
    _maxIterm = maxIterm;
}

void QmuPid::setSetpoint(float setpoint) 
{
    _setpoint = setpoint;
}

void QmuPid::setFfGain(float ffGain) 
{
    _ffGain = ffGain;
}

float QmuPid::compute(float measurement, unsigned long timestamp)
{
    float output = 0;
    float error = _setpoint - measurement;

    //Do not run update if pid loop is called too often
    float dT = (timestamp - _prevExecutionMillis) / 1000.0f;

    //pTerm
    _pTerm = error * _pGain;
    output += _pTerm;

    //Apply and constrain iTerm
    _iTerm += error * _iGain * dT;
    _iTerm = constrain(_iTerm, _minIterm, _maxIterm);
    output += _iTerm;

    //dTerm
    _dTerm = (float)(_previousMeasurement - measurement) * _dGain * dT;
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

    if (_outputThreshold != 0 && output < _outputThreshold) {
        output = _min;
    }

    return constrain(output, _min, _max);
}

void QmuPid::resetIterm() {
    _iTerm = 0;
}

float QmuPid::getPterm() {
    return _pTerm;
}

float QmuPid::getIterm() {
    return _iTerm;
}

float QmuPid::getDterm() {
    return _dTerm;
}