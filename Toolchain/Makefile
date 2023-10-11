CC=arm-none-eabi-
MARCH=cortex-m4
CFLASS= -c -mcpu=$(MARCH) -mthumb -sdt=gnu11 -O0 -Wall
LDFLAGS= -T LinkerScript.ld -nostdlibf -Map=app.map

all: app.elf

%.o: %.c 
	$(CC)gcc $(CFLASS) $^ -o $@

app.elf: main.o Startup_STM32F446RE.o
	$(CC)ld $(LDFLAGS) $^ -o $@

clean:
	rm -rf *.o *.elf
