#include <Arduino.h>  // Required for analogRead()
#include <math.h>     // Required for log()

// TemperatureSensor class for reading temperature from a thermistor
class TemperatureSensor {
  private:
    int pin;

  public:
    // Constructor: initializes the sensor pin
    TemperatureSensor(int iPin) { 
      pin = iPin; // The pin connected to the temperature sensor network
    }

    // Reads the temperature from the thermistor and applies corrections
    double getTemp() { 
      int tempReading = analogRead(pin);
      double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
      tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK ); // Convert to Kelvin
      double tempC = tempK - 273.15; // Convert Kelvin to Celsius
      return applyCorrection(tempC);
    }
    //Corrects the temperature based on the offset, updated on 03/28/25
  private:
    double applyCorrection(double temp) {
      if (temp >= 94) return temp + 5 ;
      else if (temp >= 80) return temp + 4;
      else if (temp >= 70) return temp + 2;
      return temp; // No correction below 70°C
    }
};
