#Declaractions
TARGET = STM32F446RE
BUILD_DIR = build
FLASHADR = 0x8000000
# Useful Directories
BINARY = Emapp
CODEDIRS = . "./../Src"
INCDIRS = . "-I./../Inc"
TCPATH = /home/itsmeayoub/Documents/gcc-arm-none-eabi
Inclibc = $(TCPATH)/arm-none-eabi/lib
Inclibgcc = $(TCPATH)/lib/gcc/arm-none-eabi/10.3.1
LScript = LinkerScript.ld

# Compiler, Architectue and Optimization Options
CC = arm-none-eabi-
MARCH = cortex-m4
OPT = -O0
MCU = -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE 
DEBUG = -g3 -DDEBUG
DEPFLAGS = -MMD -MP -MF $(BUILD_DIR)/$*.d
FPU = -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Compiler and Linker Flags
CFLAGS = -mcpu=$(MARCH) -std=gnu11 $(DEBUG) $(MCU) -c $(INCDIRS) $(OPT) -Wall $(DEPFLAGS) $(FPU) -mthumb
CFILES += -Wno-unused-variable
LDFLAGS = -T $(LScript) -L$(Inclibc) -L$(Inclibgcc) -Map=$(BUILD_DIR)/$(BINARY).map 

# Include The c files existing in other Directories inside the PROJECT
CFILES = $(foreach D, $(CODEDIRS), $(wildcard $(D)/*.c))
# regular expression replacement
#OBJECTS = $(patsubst %.c, %.o, $(CFILES))
#DEPFILES = $(patsubst %.c, %.d, $(CFILES))
# List of object files with the build directory
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CFILES:.c=.o)))

# List of dependency files with the build directory
DEPFILES = $(addprefix $(BUILD_DIR)/,$(notdir $(CFILES:.c=.d)))

#Specify the actions
.PHONY: all
all: $(BUILD_DIR)/$(BINARY).elf $(BUILD_DIR)/$(BINARY).hex $(BUILD_DIR)/$(BINARY).bin

$(BUILD_DIR)/%.o: $(CFILES) | $(BUILD_DIR)
	$(CC)gcc $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(BINARY).elf: $(OBJECTS)
	$(CC)ld $(LDFLAGS) $^ -o $@

$(BUILD_DIR)/$(BINARY).hex: $(BINARY).elf
	$(CC)objcopy -O ihex $< $@

$(BUILD_DIR)/$(BINARY).bin: $(BINARY).elf
	$(CC)objcopy -O binary $< $@

$(BUILD_DIR):
	mkdir $@

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: diff
diff:
	$(info The status of the repository, and the volume of per-file changes:)
	@git status
	@git diff --stat

# Include the dependencies
-include $(DEPFILES)

#Flash the binary 
stflash: $(BUILD_DIR)/$(BINARY).bin
	st-flash --reset write $< $(FLASHADR)
