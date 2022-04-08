#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <SailtrackModule.h>

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_MULTIPLIER 1.7
#define CONNECTION_LED_PIN 5

#define I2C_SDA 27
#define I2C_SCL 25

#define AHRS_SAMPLE_RATE 50
#define AHRS_UPDATE_RATE 50

#define PUBLISH_RATE 5
#define G_CONSTANT 9.8

Adafruit_BNO055 IMU;
SailtrackModule STM;
Adafruit_NXPSensorFusion AHRS;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	DynamicJsonDocument * getStatus() {
		DynamicJsonDocument * payload = new DynamicJsonDocument(300);
		JsonObject battery = payload->createNestedObject("battery");
		JsonObject cpu = payload->createNestedObject("cpu");
		battery["voltage"] = analogRead(BATTERY_ADC_PIN) * BATTERY_ADC_MULTIPLIER / 1000;
		cpu["temperature"] = temperatureRead();
		return payload;
	}
};

void ahrsTask(void * pvArguments) {
	while(true) {
		
		sensors_event_t eulerData, gyroData, linearAccelData, magnetometerData, accelerometerData, gravityData;
		
		IMU.getEvent(&eulerData, Adafruit_BNO055::VECTOR_EULER);
		IMU.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
		IMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
		IMU.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
		IMU.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
		IMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

		float gyroX = gyroData.gyro.x * RAD_TO_DEG;
		float gyroY = gyroData.gyro.y * RAD_TO_DEG;
		float gyroZ = gyroData.gyro.z * RAD_TO_DEG;
		float accelX = accelerometerData.acceleration.x / G_CONSTANT;
		float accelY = accelerometerData.acceleration.y / G_CONSTANT;
		float accelZ = accelerometerData.acceleration.z / G_CONSTANT;
		float magX = magnetometerData.magnetic.x;
		float magY = magnetometerData.magnetic.y;
		float magZ = magnetometerData.magnetic.z;

		AHRS.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);

		delay(1000 / AHRS_UPDATE_RATE);
	}
}

void beginIMU() {
	Wire.begin(I2C_SDA, I2C_SCL);
	IMU = Adafruit_BNO055(55, 0x29, &Wire);
	IMU.begin();
}

void setup() {
	STM.setNotificationLed(LED_BUILTIN);
	STM.begin("imu", IPAddress(192, 168, 42, 102), new ModuleCallbacks());
	beginIMU();
	AHRS.begin(AHRS_SAMPLE_RATE);
	xTaskCreate(ahrsTask, "ahrsTask", TASK_MEDIUM_STACK_SIZE, NULL, TASK_MEDIUM_PRIORITY, NULL);
}

void loop() {
	float linearAccelX, linearAccelY, linearAccelZ;
  	AHRS.getLinearAcceleration(&linearAccelX, &linearAccelY, &linearAccelZ);
	//convert accelerations in m/s2
	linearAccelX = linearAccelX*G_CONSTANT;
	linearAccelY = linearAccelY*G_CONSTANT;
	linearAccelZ = linearAccelZ*G_CONSTANT;

	DynamicJsonDocument payload(500);
  	JsonObject euler = payload.createNestedObject("euler");
	euler["roll"] = AHRS.getRoll();
	euler["pitch"] = AHRS.getPitch();
	euler["yaw"] = AHRS.getYaw();

	JsonObject linearAccel = payload.createNestedObject("linear_accel");
	linearAccel["x"] = linearAccelX;
	linearAccel["y"] = linearAccelY;
	linearAccel["z"] = linearAccelZ;

	payload["temperature"] = IMU.getTemp();

	STM.publish("sensor/imu0", &payload);
	Serial.println("ho pubblicato un cazzo di messaggio");

	delay(1000 / PUBLISH_RATE);
}