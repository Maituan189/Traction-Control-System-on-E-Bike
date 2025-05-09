#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <float.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#define HEADER 0xAA
#define TERMINATIOR 0xCC
#define MAX_LENGHT_DATA 40
#define UART_PORT UART_NUM_2  
#define BUFFER_LENGHT_MEDIAN_FT 10
#define RX_PIN 16             
#define TX_PIN 17             
#define BUF_SIZE 1024         
#define LED_PIN 2
#define RX2_PIN 16
#define TX2_PIN 17
MPU6050 mpu6050(Wire);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.1);
// Initialize global variables
unsigned char data_package[MAX_LENGHT_DATA]={0};
unsigned char i=0;
bool isReceiving=false, receive_state=false, data_filter_flag = false, check_filter_flag = false;

//  MAC_Address_Receiver
const uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x34, 0x0F, 0x54};
uint32_t Millis_Time1=0, Millis_Time2=0;
float WS1_Speed_Dummy,WS2_Speed_Dummy,WS1_ACC_Dummy,WS2_ACC_Dummy, Current_Dummy, Torque_Dummy, Throttle_In_Dummy, Throttle_Out_Dummy, Ratio_Dummy;

float WS1_Speed_Buffer[BUFFER_LENGHT_MEDIAN_FT],WS2_Speed_Buffer[BUFFER_LENGHT_MEDIAN_FT];
float WS1_ACC_Buffer[BUFFER_LENGHT_MEDIAN_FT],WS2_ACC_Buffer[BUFFER_LENGHT_MEDIAN_FT];
float Current_Buffer[BUFFER_LENGHT_MEDIAN_FT],Torque_Buffer[BUFFER_LENGHT_MEDIAN_FT], Ratio_Buffer[BUFFER_LENGHT_MEDIAN_FT];
float Throttle_In_Buffer[BUFFER_LENGHT_MEDIAN_FT],Throttle_Out_Buffer[BUFFER_LENGHT_MEDIAN_FT];

typedef struct __attribute__((packed, aligned(4))) data
{
  float WS1_Speed;
  float WS2_Speed;
  float WS1_ACC;
  float WS2_ACC;
  float Torque;
  float Padding;
  float Current;
  float Ratio;
  float Throttle_In;
  float Throttle_Out;
  float Roll_Angle;
  float Pitch_Angle;
  float Yaw_Angle;
  float Yaw_Rate;
}struct_data;

typedef struct __attribute__((packed, aligned(4))) data_
{
  float RollAngle_Filter;
  float PitchAngle_Filter;
  float YawAngle_Filter;
  float YawRate_Filter;
}Data_Filter;

Data_Filter data_filter_MPU;
struct_data my_data;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  digitalWrite(2, !digitalRead(2));
}

bool check_overflow(uint8_t *data, size_t len) 
{
  if (*((float*)data) > 500 && *((float*)data) > -500)
  {
    return true;
  }
  else 
  {
    return false;
  }
}

void encode_data(float *WS1_Speed,float *WS2_Speed,float *WS1_Speed_Timer,float *WS2_Speed_Timer,float *WS1_ACC,float *WS2_ACC,float *Current, float *Ratio, float *Throttle_In, float *Throttle_Out, unsigned char array[])
{
  static uint8_t data1[4] = {array[0], array[1], array[2], array[3]};
  static uint8_t data2[4] = {array[4], array[5], array[6], array[7]};
  static uint8_t data3[4] = {array[8], array[9], array[10], array[11]};
  static uint8_t data4[4] = {array[12], array[13], array[14], array[15]};
  static uint8_t data5[4] = {array[16], array[17], array[18], array[19]};
  static uint8_t data6[4] = {array[20], array[21], array[22], array[23]};
  static uint8_t data7[4] = {array[24], array[25], array[26], array[27]};
  static uint8_t data8[4] = {array[28], array[29], array[30], array[31]};
  static uint8_t data9[4] = {array[32], array[33], array[34], array[35]};
  static uint8_t data10[4] = {array[36], array[37], array[38], array[39]};
  if (check_overflow(data1,4) == false)
  {
    *WS1_Speed = *((float*)data1);
  }
  if (check_overflow(data2,4) == false)
  {
    *WS2_Speed = *((float*)data2);
  }
  if (check_overflow(data3,4) == false)
  {
    *WS1_Speed_Timer = *((float*)data3);
  }
  if (check_overflow(data4,4) == false)
  {
    *WS2_Speed_Timer = *((float*)data4);
  }
  if (check_overflow(data5,4) == false)
  {
    *WS1_ACC = *((float*)data5);
  }
  if (check_overflow(data6,4) == false)
  {
    *WS2_ACC = *((float*)data6);
  }
  if (check_overflow(data7,4) == false)
  {
    *Current = *((float*)data7);
  }
  if (check_overflow(data8,4) == false)
  {
    *Ratio = *((float*)data8);
  }
  if (check_overflow(data9,4) == false)
  {
    *Throttle_In = *((float*)data9);
  }
  if (check_overflow(data10,4) == false)
  {
    *Throttle_Out = *((float*)data10);
  }

}

void serial_print(float WS1_Speed,float WS2_Speed, float WS1_Speed_Timer, float WS2_Speed_Timer, float WS1_ACC, float WS2_ACC, float Current, float Ratio, float Throttle_In, float Throttle_Out)
{
    Serial.println("");
    //Serial.print("WS1_Speed: ");
    Serial.println(WS1_Speed);
    //Serial.print("WS2_Speed: ");
    Serial.println(WS2_Speed);
    //Serial.print("WS1_Speed_Timer: ");
    Serial.println(WS1_Speed_Timer);
    //Serial.print("WS2_Speed_Timer: ");
    Serial.println(WS2_Speed_Timer);
    //Serial.print("WS1_ACC: ");
    Serial.println(WS1_ACC);
    //Serial.print("WS2_ACC: ");
    Serial.println(WS2_ACC);
    //Serial.print("Current: ");
    Serial.println(Current);
    Serial.print("Ratio: ");
    Serial.println(Ratio);
    //Serial.print("Throttle_In: ");
    Serial.println(Throttle_In);
    //Serial.print("Throttle_Out: ");
    Serial.println(Throttle_In);
    //delay(10);
}

void setup() {
  Serial.begin(115200);    // Initialize the Serial monitor for debugging
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);   // Initialize Serial2 for sending data
  pinMode(LED_PIN,OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  /*Once ESPNow is successfully Init, we will register for Send CB to*/
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;     
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    return;
  }
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

inline float Median_Filter(float buffer_data[], uint8_t buffer_data_length)
{
  static float result;
  sortArray(buffer_data, buffer_data_length);
  result = (buffer_data[(buffer_data_length/2)-1]+buffer_data[(buffer_data_length/2)])/2;
  memset(buffer_data, 0, buffer_data_length * sizeof(float));
  return result;
}

void loop() {
  static uint8_t receivedChar;
  static uint8_t counter_data=0;
  if (millis() - Millis_Time2 > 20)
  {
    Read_MPU();
    Millis_Time2 = millis();
  }
  if (millis() - Millis_Time1 > 10)
  {
    if (data_filter_flag == true)
    {
      my_data.WS1_Speed   = Median_Filter(WS1_Speed_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.WS2_Speed   = Median_Filter(WS2_Speed_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.WS1_ACC     = Median_Filter(WS1_ACC_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.WS2_ACC     = Median_Filter(WS2_ACC_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.Torque      = Median_Filter(Torque_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.Current     = Median_Filter(Current_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.Ratio       = Median_Filter(Ratio_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.Throttle_In = Median_Filter(Throttle_In_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      my_data.Throttle_Out= Median_Filter(Throttle_Out_Buffer, BUFFER_LENGHT_MEDIAN_FT);
      data_filter_flag = false;
    }
    Send_ESPNow();
    Millis_Time1 = millis();
  }

  if (Serial2.available() > 0) 
  {
    receivedChar = Serial2.read();
  }
  // // Check header character and new data package
  if (receivedChar == HEADER && !isReceiving) 
  {
    isReceiving=true;
    i=0;
  }
  // Receive new data package
  if (isReceiving)
  {
      if (receivedChar == TERMINATIOR) // Receive new data package until ESP detect terminator character
      {
        // Reset receive flag 
        isReceiving=false;
        // Encode Data Package
        encode_data(&WS1_Speed_Dummy, &WS2_Speed_Dummy, &WS1_ACC_Dummy,&WS2_ACC_Dummy, &Torque_Dummy, &my_data.Padding, &Current_Dummy, &Ratio_Dummy, &Throttle_In_Dummy, &Throttle_Out_Dummy, data_package); 
        if ( counter_data < BUFFER_LENGHT_MEDIAN_FT)
        {
          WS1_Speed_Buffer[counter_data] = WS1_Speed_Dummy;  
          WS2_Speed_Buffer[counter_data] = WS2_Speed_Dummy;  
          WS1_ACC_Buffer[counter_data] = WS1_ACC_Dummy;
          WS2_ACC_Buffer[counter_data] = WS2_ACC_Dummy;
          Current_Buffer[counter_data] = Current_Dummy;
          Torque_Buffer[counter_data] = Torque_Dummy;
          Ratio_Buffer[counter_data] = Ratio_Dummy;
          Throttle_In_Buffer[counter_data] = Throttle_In_Dummy;
          Throttle_Out_Buffer[counter_data] = Throttle_Out_Dummy;
          counter_data++;
        }
        else 
        {
          counter_data = 0;
          data_filter_flag = true;
        }
      }
      else 
      {
        // Assign enough value and fit with fomat to data package array, except for header character
          if (i<MAX_LENGHT_DATA && receivedChar != HEADER)
          {
              data_package[i] = receivedChar;
              i+=1;
          }
      }
  }
}

void Read_MPU()
{
  mpu6050.update();
  my_data.Roll_Angle = mpu6050.getAngleY();
  my_data.Pitch_Angle = simpleKalmanFilter.updateEstimate(my_data.Roll_Angle);

  my_data.Yaw_Angle = -mpu6050.getAngleZ();
  my_data.Yaw_Rate = simpleKalmanFilter.updateEstimate(my_data.Yaw_Rate);

  //my_data.Pitch_Angle = -mpu6050.getAngleX();
  // my_data.Yaw_Angle = -mpu6050.getAngleZ();
  // my_data.Yaw_Rate = -mpu6050.getGyroZ();
}

void Filter_Data_MPU()
{
  data_filter_MPU.RollAngle_Filter = simpleKalmanFilter.updateEstimate(my_data.Roll_Angle);
  data_filter_MPU.PitchAngle_Filter = simpleKalmanFilter.updateEstimate(my_data.Pitch_Angle);
  // data_filter_MPU.YawAngle_Filter = simpleKalmanFilter.updateEstimate(my_data.Yaw_Angle);
  // data_filter_MPU.YawRate_Filter = simpleKalmanFilter.updateEstimate(my_data.Yaw_Rate);
}

void Print_Data_MPU()
{
  Serial.print("RollAngle : ");
  Serial.println(my_data.Roll_Angle);
  Serial.print("|RollAngle_Filter : ");
  Serial.println( data_filter_MPU.RollAngle_Filter);
  // Serial.print("|PitchAngle: ");
  // Serial.print(my_data.Pitch_Angle);
  // Serial.print("|YawAngle : ");
  // Serial.print(my_data.Yaw_Angle);
  // Serial.print("|YawRate : ");
  // Serial.println(my_data.Yaw_Rate);
}

void Send_ESPNow()
{
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &my_data, sizeof(my_data));
  if (result == ESP_OK) 
  {
    digitalWrite(2, !digitalRead(2));
  }
}

void sortArray(float arr[], int n) 
{
  for (int i = 0; i < n - 1; i++) 
  {
      for (int j = 0; j < n - i - 1; j++) 
      {
          if (arr[j] > arr[j + 1]) 
          {
            float temp = arr[j];
            arr[j] = arr[j + 1];
            arr[j + 1] = temp;
          }
      }
  }
}

