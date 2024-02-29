#include <Arduino.h>
#include <SPI.h>
#include <SailtrackModule.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>



// ------------------------------------------------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		100
#define MQTT_TASK_INTERVAL_MS	 	1000 / MQTT_PUBLISH_FREQ_HZ
#define BATTERY_ADC_PIN 			35
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 		32
#define BATTERY_READING_DELAY_MS	20

#define I2C_SDA_PIN 				27
#define I2C_SCL_PIN 				25
#define STM_NOTIFICATION_LED_PIN    19

Adafruit_Sensor_Calibration_EEPROM cal;
SailtrackModule stm;
int loopcount = 0;

byte calData[68];
byte calCount = 0;
float cal_array[16];
float cal[8];

uint16_t crc16Update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA00a1;
        else crc = (crc >> 1);
    }
    return crc;
}

void receiveMotionCalCalibration(float *cal_array) {
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
            cal_array = offsets;  
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

		JsonObject calibration = doc.createNestedObject("motioncal");
		calibration["array"] = cal_array;

		stm.publish("calibration", doc.as<JsonObjectConst>());

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
	}
}

void setup(void) {
    stm.begin("serial2mqtt", IPAddress(192, 168, 42, 108), new ModuleCallbacks());
    Serial.begin(115200);
	Wire.setClock(400000);
	xTaskCreate(mqttTask, "mqttTask", STM_TASK_MEDIUM_STACK_SIZE, NULL, STM_TASK_MEDIUM_PRIORITY, NULL);
}

void loop() {
    
    float rawData[16];
    Serial.print("Raw:");
    Serial.print(int(rawData[0]) * 8192 / SENSORS_GRAVITY_STANDARD); Serial.print(",");
    Serial.print(int(rawData[1]) * 8192 / SENSORS_GRAVITY_STANDARD); Serial.print(",");
    Serial.print(int(rawData[2]) * 8192 / SENSORS_GRAVITY_STANDARD); Serial.print(",");
    Serial.print(int(rawData[3]) * SENSORS_RADS_TO_DPS * 16); Serial.print(",");
    Serial.print(int(rawData[4]) * SENSORS_RADS_TO_DPS * 16); Serial.print(",");
    Serial.print(int(rawData[5]) * SENSORS_RADS_TO_DPS * 16); Serial.print(",");
    Serial.print(int(rawData[5]) * 10); Serial.print(",");
    Serial.print(int(rawData[6]) * 10); Serial.print(",");
    Serial.print(int(rawData[7]) * 10); Serial.println();

    loopcount++;

    receiveMotionCalCalibration(&cal_array);

    if (loopcount == 50 || loopcount > 100) {
        Serial.print("Cal1:");
        for (int i = 0; i < 3; i++) {
            Serial.print(cal.accel_zerog[i], 3); 
            Serial.print(",");
        }
        for (int i = 0; i < 3; i++) {
            Serial.print(cal.gyro_zerorate[i], 3);
            Serial.print(",");
        }  
        for (int i = 0; i < 3; i++) {
            Serial.print(cal.mag_hardiron[i], 3); 
            Serial.print(",");
        }  
        Serial.println(cal.mag_field, 3);
        loopcount++;
    }
    if (loopcount >= 100) {
        Serial.print("Cal2:");
        for (int i = 0; i < 9; i++) {
            Serial.print(cal.mag_softiron[i], 4); 
            if (i < 8) Serial.print(',');
        }
        Serial.println();
        loopcount = 0;
    }

    delay(10); 
}
