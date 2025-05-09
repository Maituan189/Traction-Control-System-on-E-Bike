#include "Filter.h"


void Filter::Kalman(float &KalmanState, float &KalmanUncertainty, float &KalmanInput, float &KalmanMeasurement) 
{
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * (KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

float Filter::Kalman_Filter(float Kalman_Value, float Rate_Value, float Current_Value)
{
  Kalman(Kalman_Value, KalmanUncertainty, Rate_Value, Current_Value);

  Kalman_Value=Kalman1DOutput[0]; 
  KalmanUncertainty=Kalman1DOutput[1];

  while (micros() - LoopTimer < 1000);
  LoopTimer=micros();

  return Kalman_Value;
}

void Filter::MedianFilter() 
{
    
}



