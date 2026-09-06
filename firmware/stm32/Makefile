PROJECT := EncoderTest
BUILD := build
FW_ROOT := ../../..

CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
PROGRAMMER := /home/idp2/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI

CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES := -DSTM32F401xE
INCLUDES := \
	-IInc \
	-I$(FW_ROOT)/Drivers/CMSIS/Include \
	-I$(FW_ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	-I$(FW_ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc
CFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) -std=c11 -Og -g3 \
	-ffunction-sections -fdata-sections -Wall -Wextra
ASFLAGS := $(CPU_FLAGS) -x assembler-with-cpp
LDFLAGS := $(CPU_FLAGS) \
	-TSTM32CubeIDE/STM32F401RETX_FLASH.ld \
	-Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map \
	--specs=nano.specs --specs=nosys.specs

C_SOURCES := Src/main.c Src/system_stm32f4xx.c Src/stm32f4xx_it.c
ASM_SOURCE := STM32CubeIDE/Example/Startup/startup_stm32f401ceux.s
OBJECTS := $(BUILD)/main.o $(BUILD)/system_stm32f4xx.o \
	$(BUILD)/stm32f4xx_it.o $(BUILD)/startup_stm32f401ceux.o

.PHONY: all clean flash

all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).hex $(BUILD)/$(PROJECT).bin

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/main.o: Src/main.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/system_stm32f4xx.o: Src/system_stm32f4xx.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/stm32f4xx_it.o: Src/stm32f4xx_it.c | $(BUILD)
	$(CC) $(CFLAGS) -DUSE_HAL_DRIVER -c $< -o $@

$(BUILD)/startup_stm32f401ceux.o: $(ASM_SOURCE) | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

flash: all
	$(PROGRAMMER) -c port=SWD -w $(BUILD)/$(PROJECT).hex -v -rst

clean:
	$(RM) -r $(BUILD)
