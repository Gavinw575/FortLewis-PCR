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

#include <Arduino.h>
// a stripped down simple PID controller for debugging purposes
struct TuningStruct{
  float kp;
  float ki;
  float kd;
};

class PID{
private:
float kp;
float ki;
float kd;
int highClamp;
int lowClamp;
float error=0, previousError = 0;
float P,I,D;
int output =0;


public:
PID (TuningStruct tuning, int minOutput, int maxOutput){
  highClamp = maxOutput;
  lowClamp = minOutput;
  kp = tuning.kp;
  ki = tuning.ki;
  kd = tuning.kd;
}


// function to calculate the PID stuff
int calculate(double targetTemp, double currentTemp){
  error = (targetTemp)-currentTemp;
  P = error;
  I +=error;
  D = error-previousError;
  previousError=error;
  float rawOutput = kp*P+ki*I+kd*D;
  if (rawOutput > highClamp) {
    rawOutput = highClamp;
  } else if (rawOutput < lowClamp) {
    rawOutput = lowClamp;
  }
    // Anti-windup: only integrate if the output is not at the bounds
  if (rawOutput >= highClamp || rawOutput <= lowClamp) {
      I -= error;
  }
  output = static_cast<int>(rawOutput);
  return output;
}
void reset(){
  P=0;
  I=0;
  D=0;
}

};
