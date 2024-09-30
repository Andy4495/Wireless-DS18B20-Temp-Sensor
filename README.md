# Wireless DS18B20 Temperature Sensor

[![Arduino Compile Sketches](https://github.com/Andy4495/Wireless-DS18B20-Temp-Sensor/actions/workflows/arduino-compile-sketches.yml/badge.svg)](https://github.com/Andy4495/Wireless-DS18B20-Temp-Sensor/actions/workflows/arduino-compile-sketches.yml)
[![Check Markdown Links](https://github.com/Andy4495/Wireless-DS18B20-Temp-Sensor/actions/workflows/check-links.yml/badge.svg)](https://github.com/Andy4495/Wireless-DS18B20-Temp-Sensor/actions/workflows/check-links.yml)

The wireless temperature sensor is designed to use an MSP430 LauchPad, CC110L BoosterPack [430BOOST-CC110L][4], and SparkFun's waterproof version of the [DS18B20 temperature sensor][1]. It shows how to use the built-in OneWire library to get temperature readings from the [DS18B20 Digital Thermometer][2].

The sketch and hardware is designed for low-power operation and can operate on a set of 3 alkaline AA cells for approximately one year. Be sure to remove jumpers from the isolation block and LEDs to minimize power usage of the LaunchPad.

The sketch uses a specific data structure for sending the temperature readings to my [Wireless Sensor Receiver Hub][3], but could easily be adapted to other applications and data formats.

## Hardware

Although the MSP430 will work with an operating voltage as low as 1.8 V (depending on the system frequency and specific MSP430 variant), the [DS18B20][2] has a minimum operating voltage of 3.0 V. So I choose to use three alkaline AA cells to create a nominal voltage of 4.5 V along with a [TPS715A33][5] regulator to drop the voltage to 3.3 V which is then used to power the MSP430 and DS18B20. The TPS715A33 is an efficient regulator which, once the supply voltage drops below the minimum dropout level, will then track the input voltage of the cells, meaning that it will be possible to discharge the cells all the way down to the 3.0 V minimum supply voltage and maximize the life of the batteries.

The sketch currently defines the following pin configuration for the temp sensor. These can be changed by updating the associated `#define` in the sketch:

    DS18B20_SIGNAL_PIN  13   // DQ pin, with a 4.7 K Ohm pullup to Vcc
    DS18B20_POWER_PIN   11   // VDD pin

## External Libraries

* [OneWire][7]
* [MspTandV][6] - To read the MSP430 die temperature and supply voltage.

## References

* [DS18B20][2] Digital Thermometer
* SparkFun [Waterproof Temperature Sensor with DS18B20][1]
  * Adafruit sells a [similar sensor][8] that should work the same.
* CC110L BoosterPack [Quick Start Guide][4]
* CC110L-based Sensor [Receiver Hub][3]
* [TPS715A33][5] low dropout regulator

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: https://www.sparkfun.com/products/11050
[2]: https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
[3]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub
[4]: https://www.ti.com/lit/ml/swru312b/swru312b.pdf
[5]: http://www.ti.com/lit/ug/slvu122/slvu122.pdf
[6]: https://github.com/Andy4495/mspTandV
[7]: https://github.com/PaulStoffregen/OneWire
[8]: https://www.adafruit.com/product/381
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
[//]: # ([200]: https://github.com/Andy4495/Wireless-DS18B20-Temp-Sensor)
