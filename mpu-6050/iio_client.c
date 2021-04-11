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
#include "udplink.h"
#include <sys/time.h>
#include <unistd.h>

#define DEBUG

#ifdef DEBUG
#define pr_dbg(fmt, ...)                                   \
    do                                                     \
    {                                                      \
        fprintf(stderr, "iio_debug: " fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define pr_dbg(fmt, ...) \
    do                   \
    {                    \
    } while (0)
#endif

/* BMI 160 definitions */
#define IMU "mpu6050"
#define ACCEL_X "accel_x"
#define ACCEL_Y "accel_y"
#define ACCEL_Z "accel_z"
#define ANGVL_X "anglvel_x"
#define ANGVL_Y "anglvel_y"
#define ANGVL_Z "anglvel_z"

/* BMM 150 definitions */
#define MAG "bmc150_magn"
#define MAG_X "magn_x"
#define MAG_Y "magn_y"
#define MAG_Z "magn_z"

typedef struct
{
    double timestamp;
    double imu_angular_velocity_rpy[3];
    double imu_linear_acceleration_xyz[3];
    double imu_orientation_quat[4];
    double velocity_xyz[3];
    double position_xyz[3];
} fdm_packet;

static udpLink_t stateLink;

static bool enable_failed = false;

static struct iio_context *ctx = NULL;
static struct iio_buffer *imubuf = NULL;
static struct iio_buffer *magbuf = NULL;
bool stop = false;

static double accel_x, accel_y, accel_z;
static double angvl_x, angvl_y, angvl_z;
static double mag_x, mag_y, mag_z;

#define GRAVITY 9.805f
#define SCALE 10.0f

static bool get_imu_dev(struct iio_context *ctx, struct iio_device **dev)
{
    *dev = iio_context_find_device(ctx, IMU);
    if (!*dev)
    {
        printf("Could not find MPU6050\n");
        return false;
    }

    return true;
}

static void enable_dev_channel(struct iio_device *dev, char *name)
{
    struct iio_channel *ch;

    if (enable_failed)
        return;

    ch = iio_device_find_channel(dev, name, false);
    if (ch == NULL)
    {
        enable_failed = true;
        printf("Enabling channel %s failed!\n", name);
        return;
    }
    pr_dbg("Enabling channel %s\n", name);
    iio_channel_enable(ch);
}

static void disable_imu_channel(struct iio_context *ctx, char *channel)
{
    struct iio_device *dev = NULL;
    struct iio_channel *ch = NULL;

    get_imu_dev(ctx, &dev);

    if (!dev || !channel)
    {
        pr_dbg("Disabling IMU channel failed\n");
        return;
    }

    ch = iio_device_find_channel(dev, channel, false);
    if (!ch)
    {
        pr_dbg("Disabling IMU channel, could not find channel\n");
        return;
    }
    iio_channel_disable(ch);
}

static void process_imu_buffer(struct iio_device *dev, struct iio_buffer *buf)
{
    ssize_t rxn;
    char *data;
    struct iio_channel *ch;
    ptrdiff_t inc;

    assert(buf != NULL);
    rxn = iio_buffer_refill(buf);
    if (rxn < 0)
    {
        printf("Error filling up IMU buffer\n");
        return;
    }

    ch = iio_device_find_channel(dev, ANGVL_X, false);
    data = iio_buffer_first(buf, ch);
    inc = iio_buffer_step(buf);
    angvl_x = ((int16_t *)data)[0];

    data += inc;
    angvl_y = ((int16_t *)data)[1];

    data += inc;
    angvl_z = ((int16_t *)data)[2];

    data += inc;
    accel_x = ((int16_t *)data)[3];

    data += inc;
    accel_y = ((int16_t *)data)[4];

    data += inc;
    accel_z = ((int16_t *)data)[5];

#if 0	
	ch = iio_device_find_channel(dev, ANGVL_Y, false);
	data = iio_buffer_first(buf, ch);
	angvl_y = ((int16_t *)data)[0];
	
	ch = iio_device_find_channel(dev, ANGVL_Z, false);
	data = iio_buffer_first(buf, ch);
	angvl_z = ((int16_t *)data)[2];
	
	ch = iio_device_find_channel(dev, ACCEL_X, false);
	data = iio_buffer_first(buf, ch);
	accel_x = ((int16_t *)data)[3];

	ch = iio_device_find_channel(dev, ACCEL_Y, false);
	data = iio_buffer_first(buf, ch);
	accel_y = ((int16_t *)data)[5];

	ch = iio_device_find_channel(dev, ACCEL_Z, false);
	data = iio_buffer_first(buf, ch);
	accel_z = ((int16_t *)data)[3];
#endif

    pr_dbg("Accel: x is %f, y is %f, z is %f\n", accel_x, accel_y, accel_z);
    pr_dbg("Angular: x is %d, y is %d, z is %d\n", angvl_x, angvl_y, angvl_z);
    accel_x *= GRAVITY / 4096.f;
    accel_y *= GRAVITY / 4096.f;
    accel_z *= GRAVITY / 4096.f;
    angvl_x *= 1.f / 16.4f;
    angvl_y *= 1.f / 16.4f;
    angvl_z *= 1.f / 16.4f;
    pr_dbg("Accel: x is %f, y is %f, z is %f\n", accel_x, accel_y, accel_z);
    pr_dbg("Angular: x is %f, y is %f, z is %f\n", angvl_x, angvl_y, angvl_z);
}

static void handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    stop = true;
}

int main(int argc, char **argv)
{

    struct iio_device *imu = NULL;
    struct iio_device *mag = NULL;
    struct timeval tv;
    fdm_packet pkt;

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, handle_sig);

    pr_dbg("Acquiring IIO context\n");
    ctx = iio_create_default_context();

    if (ctx == NULL)
    {
        printf("Could not acquire IIO context\n");
        goto done;
    }
    if (!iio_context_get_devices_count(ctx))
    {
        printf("No IIO devices found!\n");
        goto ctx_destroy;
    }

    pr_dbg("Finding IMU device\n");
    if (!get_imu_dev(ctx, &imu))
    {
        printf("Could not find IMU device!\n");
        goto ctx_destroy;
    }

    pr_dbg("Configuring IMU channels\n");
    enable_dev_channel(imu, ACCEL_X);
    enable_dev_channel(imu, ACCEL_Y);
    enable_dev_channel(imu, ACCEL_Z);
    enable_dev_channel(imu, ANGVL_X);
    enable_dev_channel(imu, ANGVL_Y);
    enable_dev_channel(imu, ANGVL_Z);

    if (enable_failed)
    {
        pr_dbg("Exiting since enabling one of the IMU channels failed\n");
        goto ctx_destroy;
    }
    enable_failed = false;

    imubuf = iio_device_create_buffer(imu, 16, false);
    if (!imubuf)
    {
        pr_dbg("Enabling IMU buffers failed!\n");
        goto disable_imu_channel;
    }

    printf("Sensors Ready!\n");
    udpInit(&stateLink, "127.0.0.1", 9003, false);
    pr_dbg("start state link...%d\n", 0);

    while (!stop)
    {
        usleep(1000 * 6);
        process_imu_buffer(imu, imubuf);
        if (gettimeofday(&tv, NULL) < 0)
        {
            pr_dbg("gettimeofday failed\n");
            goto buffer_destroy;
        }

        pkt.timestamp = tv.tv_sec;
        /* set imu_angular_velocity_rpy, raw sensor data */
        pkt.imu_angular_velocity_rpy[0] = angvl_x;
        pkt.imu_angular_velocity_rpy[1] = angvl_y;
        pkt.imu_angular_velocity_rpy[2] = angvl_z;

        /* set imu_linear_acceleration_xyz, raw sensor data */
        pkt.imu_linear_acceleration_xyz[0] = accel_x;
        pkt.imu_linear_acceleration_xyz[1] = accel_y;
        pkt.imu_linear_acceleration_xyz[2] = accel_z;
        /* send udp datagram */
        udpSend(&stateLink, &pkt, sizeof(pkt));
    }

buffer_destroy:
    if (imubuf)
        iio_buffer_destroy(imubuf);
disable_imu_channel:
    disable_imu_channel(ctx, ACCEL_X);
    disable_imu_channel(ctx, ACCEL_Y);
    disable_imu_channel(ctx, ACCEL_Z);
    disable_imu_channel(ctx, ANGVL_X);
    disable_imu_channel(ctx, ANGVL_Y);
    disable_imu_channel(ctx, ANGVL_Z);
ctx_destroy:
    if (ctx)
        iio_context_destroy(ctx);
done:
    return 0;
}