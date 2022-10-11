<p align="center">
  <img src="https://raw.githubusercontent.com/metis-vela-unipd/sailtrack-docs/main/Assets/SailTrack%20Logo.svg" width="180">
</p>

<p align="center">
  <img src="https://img.shields.io/github/license/metis-vela-unipd/sailtrack-imu" />
  <img src="https://img.shields.io/github/v/release/metis-vela-unipd/sailtrack-imu" />
</p>

# SailTrack IMU

SailTrack IMU is a component of the SailTrack system, it manages IMU data, providing attitude information of the boat. To learn more about the SailTrack project, please visit the [documentation repository](https://github.com/metis-vela-unipd/sailtrack-docs).

The SailTrack IMU module is based on a battery powered LilyGo TTGO T7, consisting of an [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller, connected to an IMU breakout board. For a more detailed hardware description of the module, please refer to the [Bill Of Materials](hardware/BOM.csv). The 3D-printable enclosure con be found [here](hardware/STL).

The module performs the following tasks:

* It gets the IMU data coming from the accelerometer, gyrocope and magnetometer, and fuses them, using a Kalman filter, to obtain accurate and clean attitude infromation of the boat. The filtered metrics are then sent to the SailTrack Network.

<p align="center">
  <br/>
  <img src="hardware/Connection Diagram.svg">
</p>

![module-image](hardware/Module%20Image.jpg)

## Installation

Follow the instructions below to get the SailTrack IMU firmware correctly installed. If you encounter any problem, please [open an issue](https://github.com/metis-vela-unipd/sailtrack-imu/issues/new).

1. [Install PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html).
2. Clone the SailTrack IMU repository:
   ```
   git clone https://github.com/metis-vela-unipd/sailtrack-imu.git 
   ``` 
3. Cd into the directory:
   ```
   cd sailtrack-imu
   ```
4. **(macOS ONLY)** Uncomment the commented lines after "Patch for macOS" in the `platformio.ini` file.
5. Connect the module with an USB cable.
6. **(OPTIONAL BUT HIGHLY RECCOMENDED)** Calibrate the IMU:
   1. [Install MotionCal](https://www.pjrc.com/store/prop_shield.html).
   2. Flash the calibration firmware:
      ```
      pio run -e motioncal.cpp
      ```
   3. Open MotionCal and select the serial port the module is connected to.
   4. Start rotating the module doing circular movements in all possible directions until the `Gaps` value gets below 1%.
   5. Press the `Send Cal` button and wait until a green check appears. *Note: if the check doesn't appear after some seconds, press the `Send Cal` button again.*
7. Finally, flash the firmware:
   ```
   pio run
   ```

## Usage

Once the firmware is uploaded the module can work with the SailTrack system. When SailTrack IMU is turned on, a LED start to blink to notify the user about the connection status with SailTrack Core. Then, if the connection is successful, the LED stays on, otherwise the module will put itself to sleep, and it will try to connect later. Once the module is connected it will automatically start sending measurements.

## Contributing

Contributors are welcome. If you are a student of the University of Padua, please apply for the Métis Vela Unipd team in the [website](http://metisvela.dii.unipd.it), specifying in the appliaction form that you are interested in contributing to the SailTrack Project. If you are not a student of the University of Padua, feel free to open Pull Requests and Issues to contribute to the project.

## License

Copyright © 2022, [Métis Vela Unipd](https://github.com/metis-vela-unipd). SailTrack IMU is available under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). See the LICENSE file for more info.
