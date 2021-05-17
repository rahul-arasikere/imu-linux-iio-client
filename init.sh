#!/bin/sh

modprobe industrialio
modprobe industrialio-configfs
modprobe industrialio-sw-device
modprobe industrialio-sw-trigger
modprobe industrialio_triggered_event
modprobe iio-trig-hrtimer
modprobe inv-mpu6050
modprobe inv-mpu6050-i2c


echo mpu6050 0x68 > /sys/bus/i2c/devices/i2c-0/new_device

echo 500 > /sys/bus/iio/devices/iio:device0/sampling_frequency

# cd -

# echo "Setting up configfs entries"
# mkdir /sys/kernel/config/iio/triggers/hrtimer/trigger0
# echo 5000 > /sys/bus/iio/devices/trigger0/sampling_frequency


# echo "Setting up triggers for IIO device"
# cd /sys/bus/iio/devices/iio:device0
# echo trigger0 > trigger/current_trigger
#cd /sys/bus/iio/devices/iio:device1
#echo trigger0 > trigger/current_trigger

#echo "Enabling scan elements"
#echo 1 > scan_elements/in_accel_x_en
#echo 1 > scan_elements/in_accel_y_en
#echo 1 > scan_elements/in_accel_z_en
#echo 1 > buffer/enable
