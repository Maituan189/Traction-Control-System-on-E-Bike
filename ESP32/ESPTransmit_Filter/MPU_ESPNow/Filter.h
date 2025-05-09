#ifndef FILTER_h
#define FILTER_h

#include <Arduino.h>
#include <Wire.h>

class Filter
{
  public:
  void Kalman(float &KalmanState, float &KalmanUncertainty, float &KalmanInput, float &KalmanMeasurement);
  float Kalman_Filter(float Kalman_Value, float Rate_Value, float Current_Value);
  void MedianFilter(void);
  float KalmanUncertainty=4*4;
  float Kalman1DOutput[2]; 
  uint32_t LoopTimer;
  private:

};


#endif