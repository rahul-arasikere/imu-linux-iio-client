/*
 * readsensors - read the imu and the mag
 *
 * Derived from examples/ad9361-iiostream.c
 * Copyright (C) 2017
 * Author: Bandan Das <bsd@makefile.in>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <assert.h>
#include <unistd.h>

#define ANGLVEL_X "anglvel_x"
#define ANGLVEL_Y "anglvel_y"
#define ANGLVEL_Z "anglvel_z"

#define ACCEL_X "accel_x"
#define ACCEL_Y "accel_y"
#define ACCEL_Z "accel_z"

static bool stop = false;

static struct iio_context *context;

static struct iio_device *accgyro_device;
static struct iio_buffer *accgyro_buffer;

static struct iio_channel *gyro_anglvel_x;
static struct iio_channel *gyro_anglvel_y;
static struct iio_channel *gyro_anglvel_z;

static struct iio_channel *accel_x;
static struct iio_channel *accel_y;
static struct iio_channel *accel_z;

static int16_t raw_gyro_x;
static int16_t raw_gyro_y;
static int16_t raw_gyro_z;

static int16_t raw_accel_x;
static int16_t raw_accel_y;
static int16_t raw_accel_z;

static void handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    stop = true;
}

int main(int argc, char **argv)
{
    // Listen to ctrl+c and ASSERT
    signal(SIGINT, handle_sig);
    context = iio_create_default_context();
    if (context == NULL)
    {
        perror("Failed to acquire default context!\n");
        goto end;
    }
    accgyro_device = iio_context_find_device(context, "mpu6050");
    if (accgyro_device == NULL)
    {
        perror("Failed find gyro device!\n");
        goto clean_up_context;
    }
    gyro_anglvel_x = iio_device_find_channel(accgyro_device, ANGLVEL_X, false);
    if (gyro_anglvel_x == NULL)
    {
        perror("Failed to get anglvel_x channel...\n");
        goto clean_up_context;
    }
    gyro_anglvel_y = iio_device_find_channel(accgyro_device, ANGLVEL_Y, false);
    if (gyro_anglvel_y == NULL)
    {
        perror("Failed to get anglvel_y channel...\n");
        goto clean_up_context;
    }
    gyro_anglvel_z = iio_device_find_channel(accgyro_device, ANGLVEL_Z, false);
    if (gyro_anglvel_z == NULL)
    {
        perror("Failed to get anglvel_z channel...\n");
        goto clean_up_context;
    }
    accel_x = iio_device_find_channel(accgyro_device, ACCEL_X, false);
    if (accel_x == NULL)
    {
        perror("Failed to get accel_x channel...\n");
        goto clean_up_context;
    }
    accel_y = iio_device_find_channel(accgyro_device, ACCEL_Y, false);
    if (accel_x == NULL)
    {
        perror("Failed to get accel_y channel...\n");
        goto clean_up_context;
    }
    accel_z = iio_device_find_channel(accgyro_device, ACCEL_Z, false);
    if (accel_x == NULL)
    {
        perror("Failed to get accel_z channel...\n");
        goto clean_up_context;
    }
    iio_channel_enable(gyro_anglvel_x);
    iio_channel_enable(gyro_anglvel_y);
    iio_channel_enable(gyro_anglvel_z);
    iio_channel_enable(accel_x);
    iio_channel_enable(accel_y);
    iio_channel_enable(accel_z);
    accgyro_buffer = iio_device_create_buffer(accgyro_device, 1, false);
    if (accgyro_buffer == NULL)
    {
        perror("Failed to create gyro buffer!\n");
        goto disable_channels;
    }
    while (!stop)
    {
        if (iio_buffer_refill(accgyro_buffer) > 0)
        {
            iio_channel_read_raw(accel_x, accgyro_buffer, &raw_accel_x, sizeof(int16_t));
            iio_channel_read_raw(accel_y, accgyro_buffer, &raw_accel_y, sizeof(int16_t));
            iio_channel_read_raw(accel_z, accgyro_buffer, &raw_accel_z, sizeof(int16_t));
            iio_channel_read_raw(gyro_anglvel_x, accgyro_buffer, &raw_gyro_x, sizeof(int16_t));
            iio_channel_read_raw(gyro_anglvel_y, accgyro_buffer, &raw_gyro_y, sizeof(int16_t));
            iio_channel_read_raw(gyro_anglvel_z, accgyro_buffer, &raw_gyro_z, sizeof(int16_t));
            printf("gyro_x: %x\tgyro_y:%s\tgyro_z:%x\n", raw_gyro_x, raw_gyro_y, raw_gyro_z);
            printf("accel_x: %x\taccel_y:%s\taccel_z:%x\n", raw_accel_x, raw_accel_y, raw_accel_z);
        }
    }
clean_up_buffer:
    if (accgyro_buffer)
        iio_buffer_destroy(accgyro_buffer);
disable_channels:
    iio_channel_disable(gyro_anglvel_x);
    iio_channel_disable(gyro_anglvel_y);
    iio_channel_disable(gyro_anglvel_z);
    iio_channel_disable(accel_x);
    iio_channel_disable(accel_y);
    iio_channel_disable(accel_z);
clean_up_context:
    if (context)
        iio_context_destroy(context);
end:
    return 0;
}