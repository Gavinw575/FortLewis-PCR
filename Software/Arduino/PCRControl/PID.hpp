// // class for creating a pid system
// class PID {
//   private:
//   double kp; // higher moves faster
//   double ki; // higher fixes ofset and faster
//   double kd; // higher settes faster but creastes ofset
//   unsigned long currentTime, lastTime; // the time in millisecconds of this timesep and the last timestep
//   double pError, lError, iError, dError; // error values for pid calculations. p: porportional, l: last, i: integerl, d: derivitive
//   double iErrorLimit = 255;
  
//   public:
//   PID (double proportionalGain = 1, double integralGain = 0, double derivativeGain = 0) {
//     kp = proportionalGain; // higher moves faster
//     ki = integralGain; // higher fixes ofset and faster
//     kd = derivativeGain; // higher settes faster but creastes ofset and amplifies noise
//   }

//   void reset(){
//     iError = 0;
//   }
  
//   double calculate(double currentTemp, double targetTemp) { // performs pid calculation returns error
//     lastTime = currentTime;
//     currentTime = millis();
//     lError = pError;
//     pError = targetTemp - currentTemp;
//     iError = min(max(pError * (double)(currentTime - lastTime) / 1000 + iError, -iErrorLimit), iErrorLimit);
//     dError = (pError - lError) / (double)(currentTime - lastTime) / 1000;
//     return kp * pError + ki * iError + kd * dError;
//   }
  
//   void setKp(float value) {
//     kp = value;
//   }
  
//   void setKi(float value) {
//     ki = value;
//   }
  
//   void setKd(float value) {
//     kd = value;
//   }
// };

class PID {
  private:
    double kp, ki, kd;
    unsigned long currentTime, lastTime;
    double pError, lError, iError, dError;
    double iErrorLimit = 50;  // Adjusted integral windup limit

  public:
    PID(double proportionalGain = 1, double integralGain = 0, double derivativeGain = 0) {
      kp = proportionalGain;
      ki = integralGain;
      kd = derivativeGain;
      lastTime = micros();
    }

    void reset() {
      iError = 0;
    }

    double calculate(double currentTemp, double targetTemp) {
      currentTime = micros();
      double deltaTime = (currentTime - lastTime) / 1000000.0; // Convert to seconds

      if (deltaTime <= 0) return 0; // Prevent division by zero

      lError = pError;
      pError = targetTemp - currentTemp;

      // Prevent integral windup
      iError += pError * deltaTime;
      iError = constrain(iError, -iErrorLimit, iErrorLimit);

      dError = (pError - lError) / deltaTime;

      lastTime = currentTime;
      return kp * pError + ki * iError + kd * dError;
    }

    void setKp(float value) { kp = value; }
    void setKi(float value) { ki = value; }
    void setKd(float value) { kd = value; }
};
