# Compiler and Linker
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld

# Compiler Flags
CFLAGS = -mcpu=cortex-m4 -mthumb -std=gnu11 -Wall
LDFLAGS = -T LinkerScript.ld

# Source files
SRC = main.c Startup_STM32F446RE.c

# Object files (compiled from source files)
OBJ = $(SRC:.c=.o)

# Output target
TARGET = final

all: $(TARGET)

$(TARGET): $(OBJ)
	$(LD) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
