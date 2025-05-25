# Makefile for cross-compiling ble_scanner.c for NUC980

# Path to your cross-compiler
CROSS_COMPILE ?= arm-linux-gnueabi-
CC := $(CROSS_COMPILE)gcc

# Paths to Nordic pc-ble-driver headers and library (adjust as needed)
# NRF_BLE_DRIVER_INC := /home/joseph/project/nuc980/pc-ble-driver/include
NRF_BLE_DRIVER_LIB := /home/joseph/project/nuc980/pc-ble-driver/build_v5_arm_full

NRF_BLE_DRIVER_INC := /home/joseph/project/nuc980/pc-ble-driver/sr/include/pc-ble-driver:/home/joseph/project/nuc980/pc-ble-driver/include/common

#CFLAGS := $(foreach d,$(subst :, ,$(NRF_BLE_DRIVER_INC)),-I$d) -Wall
# CFLAGS := -I/home/joseph/project/nuc980/pc-ble-driver/sr/include/pc-ble-driver \
#           -I/home/joseph/project/nuc980/pc-ble-driver/include/common -Wall

# Output binary name
TARGET := ble_scanner

# Source file
SRCS := ble_scanner.c

# Libraries to link
LIBS := -lnrf-ble-driver-sd_api_v5 -lpthread

# Compiler and linker flags
# CFLAGS := -I$(NRF_BLE_DRIVER_INC) -Wall
CFLAGS := -I/home/joseph/project/nuc980/pc-ble-driver/sr/include/pc-ble-driver \
		  -I/home/joseph/project/nuc980/pc-ble-driver/include/common/config \
		  -I/home/joseph/project/nuc980/pc-ble-driver/include/sd_api_v5 \
		  -I/home/joseph/project/nuc980/pc-ble-driver/build/output/target/usr/include/pc-ble-driver/sdk_compat \
          -I/home/joseph/project/nuc980/pc-ble-driver/include/common -Wall

LDFLAGS := -L$(NRF_BLE_DRIVER_LIB) $(LIBS)

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)