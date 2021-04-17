# Connect to an IMU on Linux through IIO

This code is written for the `mpu6050` IMU sensor but changing the name of the device from `mpu6050` to any of the devices hat are listed as compatible should work...

Run `init.sh` as root to setup the i2c bus to register the mpu6050 sensor, but should be easily modifiable to support spi and / or other sensors.

Compile the c code linking against libiio `-liio` flag and run as root to read data from the sensor.
