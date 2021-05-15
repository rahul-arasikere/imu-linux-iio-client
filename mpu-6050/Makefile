CC := gcc
CFLAGS := -Wall -g
LD_FLAGS := -liio
OBJS := iio_client.o
PROGS := iio_client

all: $(PROGS);

iio_client: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LD_FLAGS)

%: %.c
	$(CC) $(CFLAGS) -o $@ -c $^ $(LD_FLAGS)

clean:
	$(RM) $(PROGS) $(OBJS)
