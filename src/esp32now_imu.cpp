//#include <Wire.h> //Probably not needed
#include <Adafruit_LSM9DS1.h>
//#include <iostream> //Probably not needed
//#include <string> //Probably not needed
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h> 
#include <Adafruit_Sensor_Calibration.h>
#include <SailtrackModule.h>

#define I2C_SDA_PIN 27
#define I2C_SCL_PIN 25

// Create the LSM9DS1 object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Sensor_Calibration_EEPROM cal;

// I2C address definitions (default for LSM9DS1 breakout)
#define LSM9DS1_SCK  0x6B  // Accelerometer + gyroscope
#define LSM9DS1_MCK  0x1E  // Magnetometer


//Mac address of the messages sent
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x1A, 0x4D, 0x60}; //Change this line with the address of the second esp32 board.

//Used for the type of message being sent
int loopcount = 0;

SailtrackModule stm;

//Connection to Sail Track Core
class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onStatusPublish(JsonObject status) {
		JsonObject info = status.createNestedObject("info");
		info["status"] = "connected";
	}
};

// message structures
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

typedef struct cal_values{
  float offsets_sent[16];
} cal_values;

//message names
raw_message myData;
cal1_message cal1Msg;
cal2_message cal2Msg;
cal_values calVal;

//calibration function
void receiveCalibration() {
  cal.accel_zerog[0] = calVal.offsets_sent[0];
  cal.accel_zerog[1] = calVal.offsets_sent[1];
  cal.accel_zerog[2] = calVal.offsets_sent[2];

  cal.gyro_zerorate[0] = calVal.offsets_sent[3];
  cal.gyro_zerorate[1] = calVal.offsets_sent[4];
  cal.gyro_zerorate[2] = calVal.offsets_sent[5];

  cal.mag_hardiron[0] = calVal.offsets_sent[6];
  cal.mag_hardiron[1] = calVal.offsets_sent[7];
  cal.mag_hardiron[2] = calVal.offsets_sent[8];

  cal.mag_field = calVal.offsets_sent[9];

  cal.mag_softiron[0] = calVal.offsets_sent[10];
  cal.mag_softiron[1] = calVal.offsets_sent[13];
  cal.mag_softiron[2] = calVal.offsets_sent[14];
  cal.mag_softiron[3] = calVal.offsets_sent[13];
  cal.mag_softiron[4] = calVal.offsets_sent[11];
  cal.mag_softiron[5] = calVal.offsets_sent[15];
  cal.mag_softiron[6] = calVal.offsets_sent[14];
  cal.mag_softiron[7] = calVal.offsets_sent[15];
  cal.mag_softiron[8] = calVal.offsets_sent[12];

  cal.saveCalibration();

  StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
  JsonObject CalResult = doc.createNestedObject("CalResult");
  CalResult["Cal Result"] = "Sucsessful";
  stm.publish("sensor/imu0/CalResult", doc.as<JsonObjectConst>());

  pinMode(22,OUTPUT);
  digitalWrite(22,HIGH);
  delay(2000);
  digitalWrite(22,LOW);
}


esp_now_peer_info_t peerInfo;

// Data sending function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Data reciving function
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&calVal, incomingData, sizeof(calVal));
  receiveCalibration();
}



void setup() {
  Serial.begin(115200);


  stm.begin("imu_cal", IPAddress(192, 168, 42, 102), new ModuleCallbacks());

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);

  cal.begin(); 
  cal.loadCalibration();

  // Initialize I2C communication with the LSM9DS1
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1);
  }
  Serial.println("LSM9DS1 initialized!");

  // Set ranges and sampling rates
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  // ±2g
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // ±245 degrees per second
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   // ±4 gauss

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  //Seting up sending and recv functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Wire.setClock(400000);

}

void loop() {
  esp_err_t result;
  // Read accelerometer data
  sensors_event_t accelEvent, gyroEvent, magEvent, tempEvent;
  lsm.read();
  lsm.getEvent(&accelEvent, &magEvent, &gyroEvent, &tempEvent);

  
  loopcount++;


  //The ID of messages
  myData.id=0;
  cal1Msg.id=0;
  cal2Msg.id=0;

  // Function for the types of data being sent
  // Normaly the data type is RAW but every 50 messages CAL1 is bening sent and every 100 messages CAL2 is being sent 
  //(This is the syntax of MotitonCal)
  if (loopcount == 50 || loopcount > 100) {
        cal1Msg.id=2;
        for (int i = 0; i < 3; i++) {
            cal1Msg.accelT[i]=cal.accel_zerog[i];
        }
        for (int i = 0; i < 3; i++) {
            cal1Msg.gyroT[i]=cal.gyro_zerorate[i];
        }  
        for (int i = 0; i < 3; i++) {
            cal1Msg.magT[i]=cal.mag_hardiron[i];
        }  
        
        cal1Msg.magT[3]=cal.mag_field;

        esp_now_send(broadcastAddress, (uint8_t *) &cal1Msg, sizeof(cal1Msg));

        loopcount++;
    }
  else if (loopcount >= 100) {
        cal2Msg.id=3;
        for (int i = 0; i < 9; i++) {
          cal2Msg.softiron[i]=cal.mag_softiron[i];
        }

        esp_now_send(broadcastAddress, (uint8_t *) &cal2Msg, sizeof(cal2Msg));
        
        loopcount = 0;
    }
  else{
      myData.id=1;
      myData.accelT[0]=accelEvent.acceleration.x * 8192 / SENSORS_GRAVITY_STANDARD;
      myData.accelT[1]=accelEvent.acceleration.y * 8192 / SENSORS_GRAVITY_STANDARD;
      myData.accelT[2]=accelEvent.acceleration.z * 8192 / SENSORS_GRAVITY_STANDARD;

      myData.gyroT[0]=gyroEvent.gyro.x * SENSORS_RADS_TO_DPS * 16;
      myData.gyroT[1]=gyroEvent.gyro.y * SENSORS_RADS_TO_DPS * 16;
      myData.gyroT[2]=gyroEvent.gyro.z * SENSORS_RADS_TO_DPS * 16;

      myData.magT[0]=magEvent.magnetic.x * 10;
      myData.magT[1]=magEvent.magnetic.y * 10;
      myData.magT[2]=magEvent.magnetic.z * 10;

      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    }

  delay(10);
}

//kb

 


