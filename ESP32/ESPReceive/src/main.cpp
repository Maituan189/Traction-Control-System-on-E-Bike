#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
uint8_t startMarker = 0xAA; // Đánh dấu byte bắt đầu
uint8_t endMarker = 0xFF;   // Đánh dấu byte kết thúc
typedef struct data
{
  float WS1_Speed;
  float WS2_Speed;
  float WS1_Speed_Timer;
  float WS2_Speed_Timer;
  float WS1_ACC;
  float WS2_ACC;
  float Current;
  float Ratio;
  float Throttle_In;
  float Throttle_Out;
  float Roll_Angle;
  float Pitch_Angle;
  float Yaw_Angle;
  float Yaw_Rate;
}struct_data;
// Create a struct_message called myData
struct_data my_Data;

unsigned long currentMillis = 0;

bool check_values(const struct_data *data);


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  digitalWrite(2, !digitalRead(2));
  memcpy(&my_Data, incomingData, sizeof(my_Data));
}

bool check_values(const struct_data *data) 
{
  const float *values = (const float *)data;
  size_t count = sizeof(struct_data) / sizeof(float);
  for (size_t i = 0; i < count; i++)
  {
    if (values[i] < -200.0f || values[i] > 200.0f) 
    {
      return false;
    }
  }
  return true;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(2,OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

}
 
void loop() 
{
  // Serial.print("WS1_Speed: ");
  // Serial.print(my_Data.WS1_Speed);
  // Serial.print("|WS2_Speed: ");
  // Serial.print(my_Data.WS2_Speed);
  // Serial.print("|WS1_Speed_Timer: ");
  // Serial.print(my_Data.WS1_Speed_Timer);
  // Serial.print("|WS2_Speed_Timer: ");
  // Serial.print(my_Data.WS2_Speed_Timer);
  // Serial.print("|WS1_ACC: ");
  // Serial.print(my_Data.WS1_ACC);
  // Serial.print("|WS2_ACC: ");
  // Serial.print(my_Data.WS2_ACC);
  // Serial.print("|Current: ");
  // Serial.print(my_Data.Current);
  // Serial.print("|Ratio: ");
  // Serial.print(my_Data.Ratio);
  // Serial.print("|Throttle_In: ");
  // Serial.print(my_Data.Throttle_In);
  // Serial.print("|Throttle_Out: ");
  // Serial.print(my_Data.Throttle_Out);
  // Serial.print("|Roll_Angle: ");
  // Serial.print(my_Data.Roll_Angle);
  // Serial.print("|Pitch_Angle: ");
  // Serial.print(my_Data.Pitch_Angle);
  // Serial.print("|Yaw_Angle: ");
  // Serial.print(my_Data.Yaw_Angle);
  // Serial.print("|Yaw_Rate: ");
  // Serial.println(my_Data.Yaw_Rate);
  // delay(20);
  if (millis()- currentMillis> 6)
  {
    Serial.write(startMarker);
    Serial.write((uint8_t*)&my_Data, sizeof(my_Data));
    Serial.write(endMarker);
    currentMillis = millis();
  }
}
