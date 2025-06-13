#include <math.h>
const int inA = 13; // pin connected to INA on VHN5019
const int inB = 12; // pin connected to INB on VHN5019
const int ppwm = 11; // pin connected to PWM on VHN5019
const int fpwm = 10; // pin connected to PWM on fan
const int thermP = A0; // pin connected to thermal resistor neetwork. see elegooThermalResistorSch.png

bool power = false; // software on/off

double peltierPWM = 0; // the PWM signal * curent direction to be sent to curent drivers for peltier

float avgPTemp = 0; // last average for peltier temperature
int avgPTempSampleSize = 3; //Changed from 10 to 3 for faster repsonce time
// sample size for peltier temperature moving average

double targetPeltierTemp = 30; // the tempature the system will try to move to, in degrees C
double currentPeltierTemp; // the tempature curently read from the thermoristor connected to thermP, in degrees C

// class for creating a pid system
class pid {
  private:
  double kp; // higher moves faster
  double ki; // higher fixes ofset and faster
  double kd; // higher settes faster but creastes ofset
  double kAmbiant; // experimental, fixes ofset?, may remove. leave at zero if unshure
  unsigned long currentTime, lastTime; // the time in millisecconds of this timesep and the last timestep
  double pError, lError, iError, dError; // error values for pid calculations. p: porportional, l: last, i: integerl, d: derivitive
    
  public:
  pid (double proportionalGain = 1, double integralGain = 0, double derivativeGain = 0) {
    kp = proportionalGain; // higher moves faster
    ki = integralGain; // higher fixes ofset and faster
    kd = derivativeGain; // higher settes faster but creastes ofset and amplifies noise
  }
  
  double calculate(double currentTemp, double targetTemp){ // gets tempature and performs pid calculation returns error in degrees C
    lastTime = currentTime;
    currentTime = millis();
    lError = pError;
    pError = targetTemp - currentTemp;
    iError = pError * (double)(currentTime - lastTime);
    dError = (pError - lError) / (double)(currentTime - lastTime);
    return kp * pError + ki * iError + kd * dError;
  }
  void setKp(float value) {
    kp = value;
  }
  void setKi(float value) {
    ki = value;
  }
  void setKd(float value) {
    kd = value;
  }
};

// for creating a tempature sensor
// includes noise reduction
// call resetTemp() before a series of getTemp() calls
// see elegooThermalResistorSch.png for wireing
class TempSensor {
  private:
  int pin;
  double temp;
  double lastK;
  
  public:
  TempSensor(int iPin) { 
    pin = iPin; // the pin that conected to the tempature network
  }
  
  void resetTemp() { // resets saved privois tempature, nessasary if it has been a while since last getTemp()
    int tempReading = analogRead(pin);
    double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
    lastK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );
  }

  double getTemp() { // returns the tempature from thermoristor connected to thermP in degrees C, includes noise reduction
    static double spike0u;
    int tempReading = analogRead(pin);
    double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
    tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK ); // kelvin

    //Serial.print(tempK - 273.15);
    //Serial.print(" ");
    
    // noise peak removal
    
    if (tempK - lastK > 0.5 + abs(lastK -299.15) * 0.1) {
      if (spike0u == 0) {
        spike0u = tempK - lastK;
      }
      tempK -= spike0u;
    } else {
      spike0u = 0;
    }

    if (tempK - lastK < -10) {
      tempK += spike0u;
      spike0u = 0;
    }
    
    lastK = tempK;
    
    return (tempK - 273.15); // convert kelvin to celcius
  }
};

// setup pieltier tempature sensor
TempSensor peltierT(thermP);

// setup pieltier PID
pid peltierPID(5, 0.8, 6000);

bool finished = false;
bool start = false;

void setup() {
  // setup serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup pins
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(ppwm, OUTPUT);
  pinMode(fpwm, OUTPUT);
  pinMode(thermP, INPUT);

  // set initial pin state to off
  digitalWrite(inA, LOW);
  digitalWrite(inB, LOW);
  analogWrite(ppwm, 0);
  analogWrite(fpwm, 0);

  peltierT.resetTemp();
}

void loop() {
  currentPeltierTemp = peltierT.getTemp(); // read pieltier temp
  avgPTemp = ((avgPTempSampleSize - 1) * avgPTemp + currentPeltierTemp) / avgPTempSampleSize; // average input with the last 9 inputs
  peltierPWM = peltierPID.calculate(avgPTemp, targetPeltierTemp); // calculate pid and set to output
  peltierPWM = min(255, max(-255, peltierPWM)); // clamp output between -255 and 255

  if (digitalRead(7) == HIGH) {
    start = true;
  }

  if (start) {
    peltierPWM = 63;
  } else {
    peltierT.resetTemp();
  }

  float pI = analogRead(A1) * 0.065168; // for two driver boards
  float pV = peltierPWM * (15 / 255.0);
  
  // for graphing system state
  Serial.print(avgPTemp);
  Serial.print(" ");
  Serial.print(pV);
  Serial.print(" ");
  Serial.print(pI);
  Serial.print("\n");

  if (currentPeltierTemp > 65) {
    finished = true;
  }

  if (finished) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(fpwm, 1024);
    analogWrite(ppwm, 0);
    return;
  }
  
  if (currentPeltierTemp > 150) {// shut off system if over 150 degrees for safety
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(fpwm, 1024);
    analogWrite(ppwm, 0);
    return;
  }

  // convert pieltierDelta to pwm, inA, inB, and fan signals
  analogWrite(ppwm, abs(peltierPWM));
  if (peltierPWM > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(fpwm, 0);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(fpwm, 255);
  }
}
