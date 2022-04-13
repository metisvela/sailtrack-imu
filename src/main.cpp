#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_MULTIPLIER 1.7
#define CONNECTION_LED_PIN 5

#define I2C_SDA_PIN 25
#define I2C_SCL_PIN 27

#define FILTER_UPDATE_RATE_HZ 100
#define MQTT_PUBLISH_RATE_HZ 2

SailtrackModule stm;
Adafruit_BNO055 bno;
Adafruit_NXPSensorFusion filter;
Adafruit_Sensor_Calibration_EEPROM cal;

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

void publishTask(void * pvArguments) {
	while (true) {
		DynamicJsonDocument payload(500);

		JsonObject euler = payload.createNestedObject("euler");
		euler["x"] = filter.getRoll();
		euler["y"] = filter.getPitch();
		euler["z"] = filter.getYaw();

		JsonObject orientation = payload.createNestedObject("orientation");
		orientation["heading"] = 360 - filter.getYaw();
		orientation["pitch"] = - filter.getPitch();
		orientation["roll"] = filter.getRoll();

		payload["temperature"] = bno.getTemp();

		stm.publish("sensor/imu0", &payload);

		delay(1000 / MQTT_PUBLISH_RATE_HZ);
	}
}

void beginIMU() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	bno.begin(Adafruit_BNO055::OPERATION_MODE_AMG);
	bno.setExtCrystalUse(true);
	Wire.setClock(400000);
}

void beginAHRS() {
	cal.begin();
	cal.loadCalibration();
	filter.begin();
}

void setup() {
	stm.setNotificationLed(LED_BUILTIN);
	stm.begin("imu", IPAddress(192, 168, 42, 102), new ModuleCallbacks());
	beginIMU();
	beginAHRS();
	xTaskCreate(publishTask, "publishTask", TASK_MEDIUM_STACK_SIZE, NULL, TASK_MEDIUM_PRIORITY, NULL);
}

void loop() {
	sensors_event_t accelEvent, gyroEvent, magEvent;

	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
	bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

	cal.calibrate(accelEvent);
	cal.calibrate(gyroEvent);
	cal.calibrate(magEvent);

	float gx, gy, gz;
	gx = gyroEvent.gyro.x * SENSORS_RADS_TO_DPS;
	gy = gyroEvent.gyro.y * SENSORS_RADS_TO_DPS;
	gz = gyroEvent.gyro.z * SENSORS_RADS_TO_DPS;

	float ax, ay, az;
	ax = accelEvent.acceleration.x / SENSORS_GRAVITY_STANDARD;
	ay = accelEvent.acceleration.y / SENSORS_GRAVITY_STANDARD;
	az = accelEvent.acceleration.z / SENSORS_GRAVITY_STANDARD;

	filter.update(gx, gy, gz, ax, ay, az, magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z);

	delay(1000 / FILTER_UPDATE_RATE_HZ);
}
