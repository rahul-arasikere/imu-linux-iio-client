#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <iio.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr))                                                                 \
        {                                                                            \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }

static char *name = "mpu6050";
static char *trigger_str = "inv_mpu_6050";
static int buffer_length = 2;
static int count = -1;

// Streaming devices
static struct iio_device *dev;

/* IIO structs required for streaming */
static struct iio_context *ctx;
static struct iio_buffer *rxbuf;
static struct iio_channel **channels;
static unsigned int channel_count;

static bool stop;
static bool has_repeat;

/* cleanup and exit */
static void shutdown()
{
    int ret;

    if (channels)
    {
        free(channels);
    }

    printf("* Destroying buffers\n");
    if (rxbuf)
    {
        iio_buffer_destroy(rxbuf);
    }

    printf("* Disassociate trigger\n");
    if (dev)
    {
        ret = iio_device_set_trigger(dev, NULL);
        if (ret < 0)
        {
            char buf[256];
            iio_strerror(-ret, buf, sizeof(buf));
            fprintf(stderr, "%s while Disassociate trigger\n", buf);
        }
    }

    printf("* Destroying context\n");
    if (ctx)
    {
        iio_context_destroy(ctx);
    }
    exit(0);
}

static void handle_sig(int sig)
{
    printf("Waiting for process to finish... got signal : %d\n", sig);
    stop = true;
}

static ssize_t sample_cb(const struct iio_channel *chn, void *src, size_t bytes, __notused void *d)
{
    const struct iio_data_format *fmt = iio_channel_get_data_format(chn);
    unsigned int j, repeat = has_repeat ? fmt->repeat : 1;

    printf("%s ", iio_channel_get_id(chn));
    for (j = 0; j < repeat; ++j)
    {
        if (bytes == sizeof(int16_t))
            printf("%" PRIi16 " ", ((int16_t *)src)[j]);
        else if (bytes == sizeof(int64_t))
            printf("%" PRId64 " ", ((int64_t *)src)[j]);
    }

    return bytes * repeat;
}

/* simple configuration and streaming */
int main(int argc, char **argv)
{
    // Hardware trigger
    struct iio_device *trigger;
    // Listen to ctrl+c and assert
    signal(SIGINT, handle_sig);

    unsigned int i, j, major, minor;
    char git_tag[8];
    iio_library_get_version(&major, &minor, git_tag);
    printf("Library version: %u.%u (git tag: %s)\n", major, minor, git_tag);

    /* check for struct iio_data_format.repeat support
	 * 0.8 has repeat support, so anything greater than that */
    has_repeat = ((major * 10000) + minor) >= 8 ? true : false;

    printf("* Acquiring IIO context\n");
    IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
    IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

    printf("* Acquiring device %s\n", name);
    dev = iio_context_find_device(ctx, name);
    if (!dev)
    {
        perror("No device found");
        shutdown();
    }

    printf("* Initializing IIO streaming channels:\n");
    for (i = 0; i < iio_device_get_channels_count(dev); ++i)
    {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn))
        {
            printf("%s\n", iio_channel_get_id(chn));
            channel_count++;
        }
    }
    if (channel_count == 0)
    {
        printf("No scan elements found (make sure the driver built with 'CONFIG_IIO_SIMPLE_DUMMY_BUFFER=y')\n");
        shutdown();
    }
    channels = calloc(channel_count, sizeof *channels);
    if (!channels)
    {
        perror("Channel array allocation failed");
        shutdown();
    }
    for (i = 0; i < channel_count; ++i)
    {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn))
            channels[i] = chn;
    }

    printf("* Acquiring trigger %s\n", trigger_str);
    trigger = iio_context_find_device(ctx, trigger_str);
    if (!trigger || !iio_device_is_trigger(trigger))
    {
        perror("No trigger found (try setting up the iio-trig-hrtimer module)");
        shutdown();
    }

    printf("* Enabling IIO streaming channels for buffered capture\n");
    for (i = 0; i < channel_count; ++i)
        iio_channel_enable(channels[i]);

    printf("* Enabling IIO buffer trigger\n");
    if (iio_device_set_trigger(dev, trigger))
    {
        perror("Could not set trigger\n");
        shutdown();
    }

    printf("* Creating non-cyclic IIO buffers with %d samples\n", buffer_length);
    rxbuf = iio_device_create_buffer(dev, buffer_length, false);
    if (!rxbuf)
    {
        perror("Could not create buffer");
        shutdown();
    }

    printf("* Starting IO streaming (press CTRL+C to cancel)\n");
    bool has_ts = strcmp(iio_channel_get_id(channels[channel_count - 1]), "timestamp") == 0;
    int64_t last_ts = 0;
    while (!stop)
    {
        ssize_t nbytes_rx;
        /* we use a char pointer, rather than a void pointer, for p_dat & p_end
		 * to ensure the compiler understands the size is a byte, and then we
		 * can do math on it.
		 */
        char *p_dat, *p_end;
        ptrdiff_t p_inc;
        int64_t now_ts;

        // Refill RX buffer
        nbytes_rx = iio_buffer_refill(rxbuf);
        if (nbytes_rx < 0)
        {
            printf("Error refilling buf: %d\n", (int)nbytes_rx);
            shutdown();
        }

        p_inc = iio_buffer_step(rxbuf);
        p_end = iio_buffer_end(rxbuf);

        // Print timestamp delta in ms
        if (has_ts)
            for (p_dat = iio_buffer_first(rxbuf, channels[channel_count - 1]); p_dat < p_end; p_dat += p_inc)
            {
                now_ts = (((int64_t *)p_dat)[0]);
                printf("[%04" PRId64 "] ", last_ts > 0 ? (now_ts - last_ts) / 1000 / 1000 : 0);
                last_ts = now_ts;
            }

        // Print each captured sample
        switch (buffer_read_method)
        {
        case BUFFER_POINTER:
            for (i = 0; i < channel_count; ++i)
            {
                const struct iio_data_format *fmt = iio_channel_get_data_format(channels[i]);
                unsigned int repeat = has_repeat ? fmt->repeat : 1;

                printf("%s ", iio_channel_get_id(channels[i]));
                for (p_dat = iio_buffer_first(rxbuf, channels[i]); p_dat < p_end; p_dat += p_inc)
                {
                    for (j = 0; j < repeat; ++j)
                    {
                        if (fmt->length / 8 == sizeof(int16_t))
                            printf("%" PRIi16 " ", ((int16_t *)p_dat)[j]);
                        else if (fmt->length / 8 == sizeof(int64_t))
                            printf("%" PRId64 " ", ((int64_t *)p_dat)[j]);
                    }
                }
            }
            printf("\n");
            break;

        case SAMPLE_CALLBACK:
        {
            int ret;
            ret = iio_buffer_foreach_sample(rxbuf, sample_cb, NULL);
            if (ret < 0)
            {
                char buf[256];
                iio_strerror(-ret, buf, sizeof(buf));
                fprintf(stderr, "%s while processing buffer\n", buf);
            }
            printf("\n");
            break;
        }
        case CHANNEL_READ_RAW:
        case CHANNEL_READ:
            for (i = 0; i < channel_count; ++i)
            {
                uint8_t *buf;
                size_t sample, bytes;
                const struct iio_data_format *fmt = iio_channel_get_data_format(channels[i]);
                unsigned int repeat = has_repeat ? fmt->repeat : 1;
                size_t sample_size = fmt->length / 8 * repeat;

                buf = malloc(sample_size * buffer_length);
                if (!buf)
                {
                    perror("trying to allocate memory for buffer\n");
                    shutdown();
                }

                if (buffer_read_method == CHANNEL_READ_RAW)
                    bytes = iio_channel_read_raw(channels[i], rxbuf, buf, sample_size * buffer_length);
                else
                    bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size * buffer_length);

                printf("%s ", iio_channel_get_id(channels[i]));
                for (sample = 0; sample < bytes / sample_size; ++sample)
                {
                    for (j = 0; j < repeat; ++j)
                    {
                        if (fmt->length / 8 == sizeof(int16_t))
                            printf("%" PRIi16 " ", ((int16_t *)buf)[sample + j]);
                        else if (fmt->length / 8 == sizeof(int64_t))
                            printf("%" PRId64 " ", ((int64_t *)buf)[sample + j]);
                    }
                }

                free(buf);
            }
            printf("\n");
            break;
        }

        if (count != -1 && --count == 0)
            break;
    }

    shutdown();

    return 0;
}
