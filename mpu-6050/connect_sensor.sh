#!/bin/sh

modprobe industrialio
modprobe industrialio-configfs
modprobe industrialio-sw-device
modprobe industrialio-sw-trigger
modprobe iio-trig-hrtimer
modprobe inv-mpu6050
modprobe inv-mpu6050-i2c


echo mpu6050 0x68 > /sys/bus/i2c/devices/i2c-0/new_device

