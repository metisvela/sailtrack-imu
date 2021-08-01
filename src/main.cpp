#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "SailtrackModule.h"

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_MULTIPLIER 1.7
#define CONNECTION_LED_PIN 5

#define I2C_SDA 27
#define I2C_SCL 25

#define PUBLISH_RATE 10
#define PUBLISH_PERIOD_MS 1000 / PUBLISH_RATE

Adafruit_BNO055 IMU;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onWifiConnectionBegin() {
		// TODO: Notify user
	}
	
	void onWifiConnectionResult(wl_status_t status) {
		// TODO: Notify user
	}

	DynamicJsonDocument getStatus() {
		DynamicJsonDocument payload(300);
		JsonObject battery = payload.createNestedObject("battery");
		JsonObject cpu = payload.createNestedObject("cpu");
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
	STModule.begin("imu", "sailtrack-imu", IPAddress(192,168,42,102));
	STModule.setCallbacks(new ModuleCallbacks());
	beginIMU();
}

void loop() {
	sensors_event_t eulerData, gyroData, linearAccelData, magnetometerData, accelerometerData, gravityData;
	IMU.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER);
	IMU.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
	IMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
	IMU.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
	IMU.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	IMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

	DynamicJsonDocument payload(300);

	JsonObject euler = payload.createNestedObject("euler");
	euler["x"] = eulerData.orientation.x;
	euler["y"] = eulerData.orientation.y;
	euler["z"] = eulerData.orientation.z;

	JsonObject gyro = payload.createNestedObject("gyro");
	gyro["x"] = gyroData.gyro.x;
	gyro["y"] = gyroData.gyro.y;
	gyro["z"] = gyroData.gyro.z;

	JsonObject linearAccel = payload.createNestedObject("linear_accel");
	linearAccel["x"] = linearAccelData.acceleration.x;
	linearAccel["y"] = linearAccelData.acceleration.y;
	linearAccel["z"] = linearAccelData.acceleration.z;

	JsonObject magnetometer = payload.createNestedObject("magnetometer");
	magnetometer["x"] = magnetometerData.magnetic.x;
	magnetometer["y"] = magnetometerData.magnetic.y;
	magnetometer["z"] = magnetometerData.magnetic.z;

	JsonObject accelerometer = payload.createNestedObject("accelerometer");
	accelerometer["x"] = accelerometerData.acceleration.x;
	accelerometer["y"] = accelerometerData.acceleration.y;
	accelerometer["z"] = accelerometerData.acceleration.z;
	
	payload["temperature"] = IMU.getTemp();

	uint8_t calSystem, calGyro, calAccel, calMag = 0;
	IMU.getCalibration(&calSystem, &calGyro, &calAccel, &calMag);
	JsonObject calibration = payload.createNestedObject("calibration");
	calibration["system"] = calSystem;
	calibration["gyro"] = calGyro;
	calibration["accel"] = calAccel;
	calibration["mag"] = calMag;

	STModule.publish("sensor/imu0", "imu0", payload);

	delay(PUBLISH_PERIOD_MS);
}
