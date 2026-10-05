# ICM-20948 component for ESP-IDF

[![Examples build](https://github.com/cybergear-robotics/icm20948/actions/workflows/build_example.yml/badge.svg)](https://github.com/cybergear-robotics/icm20948/actions/workflows/build_example.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/cybergear-robotics/icm20948/commit-activity)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-orange.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-purple.svg)](https://shields.io/)

This is a modified copy of the SparkFun Arduino library for the TDK InvenSense ICM-20948 Inertial Measurement Unit 9-Degree Of Freedom sensor from [SparkFun_ICM-20948_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary). It bases on the Portable C99 code and uses the I2C and SPI driver of ESP-IDF. The code bases on Version 1.3 of the SparkFun Arduino library and includes support for the InvenSense Digital Motion Processor (DMP™). You can find further details in [DMP.md](docs/DMP.md).

## Supported Features

- I2C
- SPI
- DMP
- Interrupts

## Using component

```bash
idf.py add-dependency "cybergear-robotics/icm20948"
```

## Example

Not all examples are ported. For further examples, please look at the original project.

1. create example project

```bash
idf.py create-project-from-example "cybergear-robotics/icm20948:i2c_agmt"
```

2. Go to to example directory (for example `spi_agmt`)
   `cd spi_agmt`
3. Set ESP chip
   `idf.py set-target esp32`
4. Configure ICM-20948 settings
   `idf.py menuconfig` and go to `ICM-20948 Example`
5. Build, flash
   `idf.py build flash monitor`

`examples/i2c_interrupt` and `examples/spi_interrupt` configure the data-ready
interrupt and log each INT1 edge. Their INT1 GPIO defaults to GPIO 4 and can be
changed under `ICM-20948 ... Interrupt Example` in `menuconfig`.

`examples/i2c_dmp_interrupt` and `examples/spi_dmp_interrupt` configure the
DMP INT1 source and log Quat6 packets read from the DMP FIFO.

## DMP Support

DMP support can be enabled in menuconfig "ICM-20948 Settings". An example is provided in `examples/spi_dmp_quad9_orientation`.

## Related projects

|                                                        Project                                                        | Framework |   I2C   |   SPI   |   DMP   |
| :-------------------------------------------------------------------------------------------------------------------: | :-------: | :-----: | :-----: | :-----: |
|          [SparkFun_ICM-20948_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)           |  Arduino  | &check; | &check; | &check; |
|                         [hellvesper/icm20948-i2c](https://github.com/hellvesper/icm20948-i2c)                         |  ESP-IDF  | &check; |         | &check; |
|                           [wollewald/ICM20948_WE](https://github.com/wollewald/ICM20948_WE)                           |  Arduino  | &check; | &check; |         |
| [isouriadakis/Arduino_ICM20948_DMP_Full-Function](https://github.com/isouriadakis/Arduino_ICM20948_DMP_Full-Function) |  Arduino  | &check; | &check; | &check; |
