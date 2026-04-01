# Smart Home Monitor - Makefile
#
# Cross-compile for ARM (DE10-Nano HPS):
#   make
#
# Compile natively on the board:
#   make CROSS=
#
# Clean:
#   make clean

CROSS    ?= arm-linux-gnueabihf-
CC        = $(CROSS)gcc
CFLAGS    = -std=c99 -Wall -Wextra -O2 -D_GNU_SOURCE
LDFLAGS   = -lm

TARGET    = smarthome
ERASE_TGT = ufm_erase
READ_TGT  = ufm_read
SRCS      = main_supervisor.c esp32_handler.c sim800l_handler.c ufm_storage.c
OBJS      = $(SRCS:.c=.o)

all: $(TARGET) $(ERASE_TGT) $(READ_TGT)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Built: $(TARGET)"

$(ERASE_TGT): ufm_erase.c
	$(CC) $(CFLAGS) -o $@ $<
	@echo "Built: $(ERASE_TGT)"

$(READ_TGT): ufm_read.c
	$(CC) $(CFLAGS) -o $@ $<
	@echo "Built: $(READ_TGT)"

%.o: %.c smart_home.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) $(ERASE_TGT) $(READ_TGT)

deploy: $(TARGET) $(ERASE_TGT) $(READ_TGT)
	@echo "Copy to board: scp $(TARGET) $(ERASE_TGT) $(READ_TGT) root@<board-ip>:/home/root/"

.PHONY: all clean deploy
