
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SailtrackModule.h>

#include <Adafruit_AHRS_NXPFusion.h>

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_MULTIPLIER 1.7
#define CONNECTION_LED_PIN 5

#define I2C_SDA 27
#define I2C_SCL 25

#define PUBLISH_RATE 500
#define PUBLISH_PERIOD_MS 1000 / PUBLISH_RATE
#define G_CONSTANT 9.8

Adafruit_BNO055 IMU;
SailtrackModule STM;
Adafruit_NXPSensorFusion IMU_KF;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onWifiConnectionBegin() {
		// TODO: Notify user
	}
	
	void onWifiConnectionResult(wl_status_t status) {
		// TODO: Notify user
	}

	DynamicJsonDocument * getStatus() {
		DynamicJsonDocument * payload = new DynamicJsonDocument(300);
		JsonObject battery = payload->createNestedObject("battery");
		JsonObject cpu = payload->createNestedObject("cpu");
		battery["voltage"] = analogRead(BATTERY_ADC_PIN) * BATTERY_ADC_MULTIPLIER / 1000;
		cpu["temperature"] = temperatureRead();
		return payload;
	}
};

void beginIMU() {
	Wire.begin(I2C_SDA, I2C_SCL);
	IMU = Adafruit_BNO055(55, 0x29, &Wire);
	IMU.begin();
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	STM.setNotificationLed(LED_BUILTIN);
	beginIMU();
  //Kalman Filter INIT
  IMU_KF.begin(PUBLISH_RATE);
}

void loop() {

	sensors_event_t eulerData, gyroData, linearAccelData, magnetometerData, accelerometerData, gravityData;
	IMU.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER);
	IMU.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
	IMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
	IMU.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
	IMU.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	IMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  //sensor fusion on IMU with Kalman Filter from Adafruit AHRS
  
  //geting raw data components
  float gyro_x = gyroData.gyro.x;
  float gyro_y = gyroData.gyro.y;
  float gyro_z = gyroData.gyro.z;
  float ax = accelerometerData.acceleration.x/G_CONSTANT;
  float ay = accelerometerData.acceleration.y/G_CONSTANT;
  float az = accelerometerData.acceleration.z/G_CONSTANT;
  float mx = magnetometerData.magnetic.x;
  float my = magnetometerData.magnetic.y;
  float mz = magnetometerData.magnetic.z;

  //kalman filter update
  IMU_KF.update(gyro_x, gyro_y, gyro_z, ax, ay, az, mx, my, mz);
  
  //extracting measurments from filter
  float filt_accx, filt_accy, filt_accz;
  float filt_gx, filt_gy, filt_gz;
  float filt_roll, filt_pitch, filt_yaw;
  float w, x, y, z;
  IMU_KF.getQuaternion(&w, &x, &y, &z);
  IMU_KF.getLinearAcceleration(&filt_accx, &filt_accy, &filt_accz);
  IMU_KF.getGravityVector(&filt_gx, &filt_gy, &filt_gz);
  filt_roll = IMU_KF.getRoll();
  filt_pitch = IMU_KF.getPitch();
  filt_yaw = IMU_KF.getYaw();
  
  //subracting gravity acceleration vector
/*filt_accx = filt_accx-filt_gx;
  filt_accy = filt_accy-filt_gy;
  filt_accz = filt_accz-filt_gz;
*/
	DynamicJsonDocument payload(500);
  JsonObject euler = payload.createNestedObject("euler");
	euler["x"] = filt_roll;
	euler["y"] = filt_pitch;
	euler["z"] = filt_yaw;
/*
	JsonObject euler = payload.createNestedObject("euler");
	euler["x"] = eulerData.orientation.x;
	euler["y"] = eulerData.orientation.y;
	euler["z"] = eulerData.orientation.z;
*/
	JsonObject gyro = payload.createNestedObject("gyro");
	gyro["x"] = gyroData.gyro.x;
	gyro["y"] = gyroData.gyro.y;
	gyro["z"] = gyroData.gyro.z;
/*
	JsonObject linearAccel = payload.createNestedObject("linear_accel");
	linearAccel["x"] = linearAccelData.acceleration.x;
	linearAccel["y"] = linearAccelData.acceleration.y;
	linearAccel["z"] = linearAccelData.acceleration.z;
*/
	JsonObject linearAccel = payload.createNestedObject("linear_accel");
	linearAccel["x"] = filt_accx;
	linearAccel["y"] = filt_accy;
	linearAccel["z"] = filt_accz;

  JsonObject gravity = payload.createNestedObject("gravity");
	gravity["x"] = filt_gx;
	gravity["y"] = filt_gy;
	gravity["z"] = filt_gz;

 	JsonObject quaternion = payload.createNestedObject("quaternion");
	quaternion["w"] = w;
  quaternion["x"] = x;
	quaternion["y"] = y;
	quaternion["z"] = z; 
/*
	JsonObject magnetometer = payload.createNestedObject("magnetometer");
	magnetometer["x"] = magnetometerData.magnetic.x;
	magnetometer["y"] = magnetometerData.magnetic.y;
	magnetometer["z"] = magnetometerData.magnetic.z;
*/

	JsonObject accelerometer = payload.createNestedObject("accelerometer");
	accelerometer["x"] = accelerometerData.acceleration.x;
	accelerometer["y"] = accelerometerData.acceleration.y;
	accelerometer["z"] = accelerometerData.acceleration.z;

	uint8_t calSystem, calGyro, calAccel, calMag = 0;
	IMU.getCalibration(&calSystem, &calGyro, &calAccel, &calMag);
	JsonObject calibration = payload.createNestedObject("calibration");
	//fare uno store dei dati di calibrazione buoni per evitare future calibrazioni.
	calibration["system"] = calSystem;
	calibration["gyro"] = calGyro;
	calibration["accel"] = calAccel;
	calibration["mag"] = calMag;

	payload["temperature"] = IMU.getTemp();

	STM.publish("sensor/imu0", &payload);

	delay(PUBLISH_PERIOD_MS);
}


