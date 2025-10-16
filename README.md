# SailTrack IMU
Module of the SailTrack system for getting combined orientation and acceleration data of the boat.
For a better understanding of the whole system, please check [SailTrack Docs](https://github.com/metis-vela-unipd/sailtrack-docs).

## Installation
 1. Download and install [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html). 
 2. Download and install [MotionCal](https://www.pjrc.com/store/prop_shield.html). 
 3. Clone the [SailTrack IMU](https://github.com/metis-vela-unipd/sailtrack-imu) repository.

## Usage
 1. Flash MotionCal calibration firmware:
 ```bash
  pio run -e motioncall.cpp 
 ```
 2. Open MotionCal and select the port on the top-left menu.
 3. Move the sensor doing circular movement until the "Send Cal" button enables.
 4. Press that button and close MotionCal.
 5. Flash main.cpp
 ```bash
  pio run -e main.cpp 
 ```

 ### Wireless Calibration
 1. Upload the espMacAdress.cpp to the board with the IMU and also to another Esp32 board. (Second board will stay pluged in the computer)
 2. Change line 24 on the esp32now_imu.cpp file with the address of the other board.
 3. Change line 6 on the esp32now_pc.cpp file with the address of the board with the IMU.
 4. Upload the files respectively.
 5. Open MotionCal and select the port on the top-left menu.
 6. Move the sensor doing circular movement until the "Send Cal" button enables.
 7. Press that button and close MotionCal.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
Copyright © 2022, [Métis Vela Unipd](https://github.com/metis-vela-unipd). SailTrack Core is available under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). See the LICENSE file for more info.
