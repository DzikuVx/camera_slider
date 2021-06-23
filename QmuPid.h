#ifndef QmuPid_h

#define QmuPid_h

class QmuPid
{
  public:
    QmuPid(float pGain, float iGain, float dGain, float ffGain);
    float compute(float measurement, unsigned long timestamp);
    void setSetpoint(float setpoint);
    void resetIterm();
    float getIterm();
    float getDterm();
    float getPterm();
    void setProperties(int minOutput, int maxOutput);
    void setItermProperties(float minIterm, float maxIterm);
    void setFfGain(float ffGain);
    void setOutputThreshold(int value);
    void setIsNegativeFeedback(bool value);

  private:
    float _pGain;
    float _iGain;
    float _dGain;
    float _ffGain;    
    float _pTerm;
    float _iTerm;
    float _dTerm;
    float _ffTerm;
    float _error;
    float _minIterm;
    float _maxIterm;
    float _setpoint;
    int _min;
    int _max;
    int _outputThreshold;
    bool _isNegativeFeedback;
    float _previousError;
    float _previousMeasurement;
    unsigned long _prevExecutionMillis;
};

#endif