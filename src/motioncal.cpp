#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor_Calibration.h>

// -------------------------- Configuration -------------------------- //

#define I2C_SDA_PIN                 27
#define I2C_SCL_PIN                 25

// ------------------------------------------------------------------- //


Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);
Adafruit_Sensor_Calibration_EEPROM cal;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
int loopcount = 0;

byte calData[68];
byte calCount = 0;

uint16_t crc16Update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else crc = (crc >> 1);
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
            cal.accel_zerog[0] = offsets[0];
            cal.accel_zerog[1] = offsets[1];
            cal.accel_zerog[2] = offsets[2];
            
            cal.gyro_zerorate[0] = offsets[3];
            cal.gyro_zerorate[1] = offsets[4];
            cal.gyro_zerorate[2] = offsets[5];
            
            cal.mag_hardiron[0] = offsets[6];
            cal.mag_hardiron[1] = offsets[7];
            cal.mag_hardiron[2] = offsets[8];

            cal.mag_field = offsets[9];
            
            cal.mag_softiron[0] = offsets[10];
            cal.mag_softiron[1] = offsets[13];
            cal.mag_softiron[2] = offsets[14];
            cal.mag_softiron[3] = offsets[13];
            cal.mag_softiron[4] = offsets[11];
            cal.mag_softiron[5] = offsets[15];
            cal.mag_softiron[6] = offsets[14];
            cal.mag_softiron[7] = offsets[15];
            cal.mag_softiron[8] = offsets[12];

            cal.saveCalibration();

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

void setup(void) {
    Serial.begin(115200);
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    cal.begin();
    cal.loadCalibration();
    fxos.begin();
	fxas.begin();
	accelerometer = fxos.getAccelerometerSensor();
	gyroscope = &fxas;
  	magnetometer = fxos.getMagnetometerSensor();
	Wire.setClock(400000);
}

void loop() {
    sensors_event_t accelEvent, gyroEvent, magEvent;

    accelerometer->getEvent(&accelEvent);
	gyroscope->getEvent(&gyroEvent);
	magnetometer->getEvent(&magEvent);
    
    Serial.print("Raw:");
    Serial.print(int(accelEvent.acceleration.x * 8192 / SENSORS_GRAVITY_STANDARD)); Serial.print(",");
    Serial.print(int(accelEvent.acceleration.y * 8192 / SENSORS_GRAVITY_STANDARD)); Serial.print(",");
    Serial.print(int(accelEvent.acceleration.z * 8192 / SENSORS_GRAVITY_STANDARD)); Serial.print(",");
    Serial.print(int(gyroEvent.gyro.x * SENSORS_RADS_TO_DPS * 16)); Serial.print(",");
    Serial.print(int(gyroEvent.gyro.y * SENSORS_RADS_TO_DPS * 16)); Serial.print(",");
    Serial.print(int(gyroEvent.gyro.z * SENSORS_RADS_TO_DPS * 16)); Serial.print(",");
    Serial.print(int(magEvent.magnetic.x * 10)); Serial.print(",");
    Serial.print(int(magEvent.magnetic.y * 10)); Serial.print(",");
    Serial.print(int(magEvent.magnetic.z * 10)); Serial.println();

    loopcount++;

    receiveCalibration();

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
