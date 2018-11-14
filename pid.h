#ifndef PPMReader_h

#define PPMReader_h

class PIDController
{
  public:
    PIDController(float pGain, float iGain, float dGain, float ffGain);
    int compute(float measurement, unsigned long timestamp);
    void setSetpoint(float setpoint);
    void resetIterm();
    float getIterm();
    float getDterm();
    void setProperties(int minOutput, int maxOutput);
    void setItermProperties(float minIterm, float maxIterm);
    void setFfGain(float ffGain);

  private:
    float _pGain;
    float _iGain;
    float _dGain;
    float _ffGain;    
    float _iTerm;
    float _dTerm;
    float _ffTerm;
    float _error;
    float _minIterm;
    float _maxIterm;
    float _setpoint;
    int _min;
    int _max;
    float _previousError;
    float _previousMeasurement;
    unsigned long _prevExecutionMillis;
};

#endif