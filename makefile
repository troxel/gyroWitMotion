
TARGET_EXEC = gyroWitMotion
BUILD_DIR = ./bin
SRC_DIR = ./src
CC = gcc

VPATH = src

# C Flags
CFLAGS			+= -Wall 
CFLAGS			+= -g


# define the C source files
SRCS				+= gyroWitMotion.c
SRCS				+= shmcmn.c

OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o)
#DEPS := $(OBJS:.o=.d)

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

#-include $(DEPS)

MKDIR_P ?= mkdir -p