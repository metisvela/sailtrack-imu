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

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
Copyright © 2022, [Métis Vela Unipd](https://github.com/metis-vela-unipd). SailTrack Core is available under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). See the LICENSE file for more info.
