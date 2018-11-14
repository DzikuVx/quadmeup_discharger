#ifndef PPMReader_h

#define PPMReader_h

class PIDController
{
  public:
    PIDController(float pGain, float iGain, float dGain, float ffGain);
    int compute(int measurement, unsigned long timestamp);
    void setSetpoint(int setpoint);
    void resetIterm();
    float getIterm();
    float getDterm();
    void setProperties(int minOutput, int maxOutput);
    void setItermProperties(int minIterm, int maxIterm);

  private:
    float _pGain;
    float _iGain;
    float _dGain;
    float _ffGain;    
    float _iTerm;
    float _dTerm;
    float _ffTerm;
    int _error;
    int _minIterm;
    int _maxIterm;
    int _setpoint;
    int _min;
    int _max;
    int _previousError;
    int _previousMeasurement;
    unsigned long _prevExecutionMillis;
};

#endif