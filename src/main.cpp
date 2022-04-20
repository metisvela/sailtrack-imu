#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_RESOLUTION 4095
#define BATTERY_ADC_REF_VOLTAGE 1.1
#define BATTERY_ESP32_REF_VOLTAGE 3.3
#define BATTERY_NUM_READINGS 32

#define I2C_SDA_PIN 25
#define I2C_SCL_PIN 27

#define FILTER_UPDATE_RATE_HZ 100
#define MQTT_DATA_PUBLISH_RATE_HZ 10

SailtrackModule stm;
Adafruit_BNO055 bno;
Adafruit_NXPSensorFusion filter;
Adafruit_Sensor_Calibration_EEPROM cal;

float eulerX, eulerY, eulerZ;
float linearAccelX, linearAccelY, linearAccelZ;
int8_t temp;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	DynamicJsonDocument * getStatus() {
		DynamicJsonDocument * payload = new DynamicJsonDocument(300);
		JsonObject battery = payload->createNestedObject("battery");
		JsonObject cpu = payload->createNestedObject("cpu");
		float avg = 0;
		for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
			avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
			delay(20);
		}
		battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
		cpu["temperature"] = temperatureRead();
		return payload;
	}
};

void publishTask(void * pvArguments) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true) {
		DynamicJsonDocument payload(500);

		JsonObject euler = payload.createNestedObject("euler");
		euler["x"] = eulerX;
		euler["y"] = eulerY;
		euler["z"] = eulerZ;

		JsonObject orientation = payload.createNestedObject("orientation");
		orientation["heading"] = 360 - eulerZ;
		orientation["pitch"] = - eulerY;
		orientation["roll"] = eulerX;

		JsonObject linearAccel = payload.createNestedObject("linearAccel");
		linearAccel["x"] = linearAccelX;
		linearAccel["y"] = linearAccelY;
		linearAccel["z"] = linearAccelZ;

		payload["temperature"] = temp;

		stm.publish("sensor/imu0", &payload);

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000 / MQTT_DATA_PUBLISH_RATE_HZ));
	}
}

void beginModule() {
	stm.setNotificationLed(LED_BUILTIN);
	stm.begin("imu", IPAddress(192, 168, 42, 102), new ModuleCallbacks());
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
	beginModule();
	beginIMU();
	beginAHRS();
	xTaskCreate(publishTask, "publishTask", TASK_MEDIUM_STACK_SIZE, NULL, TASK_MEDIUM_PRIORITY, NULL);
}

TickType_t lastWakeTime = xTaskGetTickCount();
void loop() {
	sensors_event_t accelEvent, gyroEvent, magEvent;

	bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
	bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
	bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
	temp = bno.getTemp();

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

	eulerX = filter.getRoll();
	eulerY = filter.getPitch();
	eulerZ = filter.getYaw();

	filter.getLinearAcceleration(&linearAccelX, &linearAccelY, &linearAccelZ); 

	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000 / FILTER_UPDATE_RATE_HZ));
}
