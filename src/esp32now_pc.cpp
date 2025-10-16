#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Mac address of the messages sent
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x1A, 0x68, 0x14}; //Change this line with the address of the imu



esp_now_peer_info_t peerInfo;

// Structures of the messages sent
typedef struct cal_values{
  float offsets_sent[16];
} cal_values;

typedef struct raw_message {
  int id;
  int accelT[3];
  int gyroT[3];
  int magT[3];

} raw_message;

typedef struct cal1_message
{
  int id;
  double accelT[3];
  double gyroT[3];
  double magT[4];
} cal1_message;

typedef struct cal2_message
{
  int id;
  double softiron[9];
} cal2_message;

// Names of the structures
raw_message myData;
cal1_message cal1Msg;
cal2_message cal2Msg;
cal_values calVal;

//Provided funcitons for receiving data from MotionCal
byte calData[68];
byte calCount = 0;

uint16_t crc16Update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}

void receiveCalibration() {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
      b = Serial.read();
      if (calCount == 0 && b != 117) return;
      if (calCount == 1 && b != 84) {
          calCount = 0;
          return;
      }

      calData[calCount++] = b;
      if (calCount < 68) return;

      crc = 0xFFFF;
      for (i=0; i < 68; i++) 
          crc = crc16Update(crc, calData[i]);
      if (!crc) {
          float offsets[16];
          memcpy(offsets, calData + 2, 16 * 4);
          for(int i=0;i<16;i++)
          {
            calVal.offsets_sent[i]=offsets[i];
          }

          

          calCount = 0;
          return;
      }

      for (i=2; i < 67; i++) {
          if (calData[i] == 117 && calData[i+1] == 84) {
              calCount = 68 - i;
              memmove(calData, calData + i, calCount);
              return;
          }
      }

      if (calData[67] == 117) {
          calData[0] = 117;
          calCount = 1;
      } else calCount = 0;
  }
}



// Data reciving function
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  memcpy(&cal1Msg, incomingData, sizeof(cal1Msg));
  memcpy(&cal2Msg, incomingData, sizeof(cal2Msg));
  if(myData.id==1) 
  {
    Serial.print("Raw:");

    for(int i=0;i<3;i++)
    {
      Serial.print(myData.accelT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(myData.gyroT[i]);
      Serial.print(",");

    }

    for(int i=0;i<2;i++)
    {
      Serial.print(myData.magT[i]);
      Serial.print(",");
    }
    Serial.print(myData.magT[2]);
    Serial.println();
  }

  else if (cal1Msg.id==2)
  {
    Serial.print("Cal1:");

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.accelT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.gyroT[i]);
      Serial.print(",");
    }

    for(int i=0;i<3;i++)
    {
      Serial.print(cal1Msg.magT[i]);
      Serial.print(",");
    }
    Serial.print(cal1Msg.magT[3]);
    Serial.println();
  }

  else if(cal2Msg.id==3)
  {
    Serial.print("Cal2:");
    for(int i=0;i<8;i++)
    {
      Serial.print(cal2Msg.softiron[i]);
      Serial.print(",");
    }
    Serial.print(cal2Msg.softiron[8]);
    Serial.println();
  }
  
}
 
// Data sending function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  calVal.offsets_sent[0]=0;
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //Seting up sending and recv functions
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);
  
  pinMode(22,OUTPUT);
}
 
void loop() {
  // Reciving the data
  receiveCalibration();

  //If a new value is set, than the new calibration is sent
  if(calVal.offsets_sent[9]!=0)
  {
    esp_now_send(broadcastAddress, (uint8_t *) &calVal, sizeof(calVal));
    calVal.offsets_sent[9]=0;

    digitalWrite(22,HIGH);
    delay(2000);
    digitalWrite(22,LOW);
  }

}

//kb