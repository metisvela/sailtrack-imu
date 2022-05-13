# SailTrack IMU
Module of the SailTrack system for getting combined orientation and acceleration data of the boat.
For a better understanding of the whole system, please check [SailTrack Docs](https://github.com/metis-vela-unipd/sailtrack-docs).

## Installation
 1. Download and install [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html). 
 2. Download [MotionCal](https://www.pjrc.com/store/prop_shield.html). 
 3. Clone the [SailTrack IMU](https://github.com/metis-vela-unipd/sailtrack-imu) repository.
 4. Flash the calibration firmware:
  ```bash
  pio run -e motioncall.cpp 
  ```
 5. motioncall configuration
 6. Flash the firmware

## Usage

explain led blinking


pio run on platformio core
flash motioncall.cpp
calibrate following the instructions
close motioncall 
flash main.cpp
led blinking

```bash
pio run -e motioncall.cpp 

pio run


```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
