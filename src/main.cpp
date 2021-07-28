#include <Arduino.h>

#include <ArduinoJson.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "SailtrackModule.h"

#define I2C_SDA 27
#define I2C_SCL 25

Adafruit_BNO055 bno;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

void setup()
{
	// Enable serial communication
	Serial.begin(115200);

	STModule.begin("imu", "sailtrack-imu", IPAddress(192,168,42,102));

	Wire.begin(I2C_SDA, I2C_SCL);
	bno = Adafruit_BNO055(55, 0x29, &Wire);

	while (!bno.begin())
	{
		Serial.print(".");
	}
}

void loop()
{
	//could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
	sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
	bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
	bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
	bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
	bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
	bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

	int8_t boardTemp = bno.getTemp();

	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);

	DynamicJsonDocument doc(300);

	JsonArray Orient = doc.createNestedArray("Orient");
	Orient.add(orientationData.orientation.x);
	Orient.add(orientationData.orientation.y);
	Orient.add(orientationData.orientation.z);

	JsonArray Rot = doc.createNestedArray("Rot");
	Rot.add(angVelocityData.gyro.x);
	Rot.add(angVelocityData.gyro.y);
	Rot.add(angVelocityData.gyro.z);

	JsonArray Linear = doc.createNestedArray("Linear");
	Linear.add(linearAccelData.acceleration.x);
	Linear.add(linearAccelData.acceleration.y);
	Linear.add(linearAccelData.acceleration.z);

	JsonArray Mag = doc.createNestedArray("Mag");
	Mag.add(magnetometerData.magnetic.x);
	Mag.add(magnetometerData.magnetic.y);
	Mag.add(magnetometerData.magnetic.z);

	JsonArray Accl = doc.createNestedArray("Accl");
	Accl.add(accelerometerData.acceleration.x);
	Accl.add(accelerometerData.acceleration.y);
	Accl.add(accelerometerData.acceleration.z);
	
	doc["temperature"] = boardTemp;

	JsonObject Calibration = doc.createNestedObject("Calibration");
	Calibration["Sys"] = system;
	Calibration["Gyro"] = gyro;
	Calibration["Accel"] = accel;
	Calibration["Mag"] = mag;

	STModule.publish("sensor/imu0", "imu0", doc);

	delay(BNO055_SAMPLERATE_DELAY_MS);
}
