#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

// -------------------------- Configuration -------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		100

#define BATTERY_ADC_PIN 			35
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 		32
#define BATTERY_READING_DELAY_MS	20

#define I2C_SDA_PIN 				27
#define I2C_SCL_PIN 				25
#define STM_NOTIFICATION_LED_PIN    19

#define LOOP_TASK_INTERVAL_MS		1000 / AHRS_UPDATE_FREQ_HZ
#define MQTT_TASK_INTERVAL_MS	 	1000 / MQTT_PUBLISH_FREQ_HZ

// ------------------------------------------------------------------- //

SailtrackModule stm;
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);
Adafruit_Sensor_Calibration_EEPROM cal;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

class ModuleCallbacks: public SailtrackModuleCallbacks {
	void onStatusPublish(JsonObject status) {
		JsonObject battery = status.createNestedObject("battery");
		float avg = 0;
		for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
			avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
			delay(BATTERY_READING_DELAY_MS);
		}
		battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
	}
};

void mqttTask(void * pvArguments) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true) {
		StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;

		JsonObject raw = doc.createNestedObject("rawMeasure");
		raw["ax"] = ax;
		raw["ay"] = ay;
		raw["az"] = az;
        raw["gx"] = gx;
		raw["gy"] = gy;
		raw["gz"] = gz;
        raw["mx"] = mx;
		raw["my"] = my;
		raw["mz"] = mz;

		stm.publish("sensor/imu0", doc.as<JsonObjectConst>());

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
	}
}

void calTask(){}

void beginIMU() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	fxos.begin();
	fxas.begin();
	accelerometer = fxos.getAccelerometerSensor();
	gyroscope = &fxas;
  	magnetometer = fxos.getMagnetometerSensor();
	Wire.setClock(400000);
}

void setup() {
	stm.begin("imu", IPAddress(192, 168, 42, 102), new ModuleCallbacks());
	beginIMU();
	xTaskCreate(mqttTask, "mqttTask", STM_TASK_MEDIUM_STACK_SIZE, NULL, STM_TASK_MEDIUM_PRIORITY, NULL);
}

void loop() {
	TickType_t lastWakeTime = xTaskGetTickCount();
	sensors_event_t accelEvent, gyroEvent, magEvent;

	accelerometer->getEvent(&accelEvent);
	gyroscope->getEvent(&gyroEvent);
	magnetometer->getEvent(&magEvent);

	
	gx = gyroEvent.gyro.x * SENSORS_RADS_TO_DPS * 16;
	gy = gyroEvent.gyro.y * SENSORS_RADS_TO_DPS * 16;
	gz = gyroEvent.gyro.z * SENSORS_RADS_TO_DPS * 16;


	ax = accelEvent.acceleration.x * 8192 / SENSORS_GRAVITY_STANDARD;
	ay = accelEvent.acceleration.y * 8192 / SENSORS_GRAVITY_STANDARD;
	az = accelEvent.acceleration.z * 8192 / SENSORS_GRAVITY_STANDARD;


	mx = magEvent.magnetic.x * 10;
	my = magEvent.magnetic.y * 10;
	mz = magEvent.magnetic.z * 10;
    delay(10);
}
