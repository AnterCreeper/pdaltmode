################################################################################
# MRS Version: 2.2.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
pdaltmode.hex \

SECONDARY_LIST += \
pdaltmode.lst \

SECONDARY_SIZE += \
pdaltmode.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: pdaltmode.elf secondary-outputs

# Tool invocations
pdaltmode.elf: $(OBJS) $(USER_OBJS)
	@	riscv-wch-elf-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -mstrict-align -msave-restore -fmax-errors=20 -Ofast -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -flto -Werror -Wunused -Wall -Wno-format -g -T "/home/hp/pdaltmode/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"pdaltmode.map" --specs=nosys.specs -o "pdaltmode.elf" $(OBJS)  $(LIBS) -lprintf
pdaltmode.hex: pdaltmode.elf
	@	riscv-wch-elf-objcopy -O ihex "pdaltmode.elf" "pdaltmode.hex"
pdaltmode.lst: pdaltmode.elf
	@	riscv-wch-elf-objdump --all-headers --demangle --disassemble -M xw "pdaltmode.elf" > "pdaltmode.lst"
pdaltmode.siz: pdaltmode.elf
	riscv-wch-elf-size --format=berkeley "pdaltmode.elf"

# Other Targets
clean:
	-$(RM) $(OBJS) $(EXPANDS) $(CALLGRAPH_DOT) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(S_DEPS)$(S_UPPER_DEPS)$(ASM_DEPS)$(ASM_UPPER_DEPS)$(C_DEPS) pdaltmode.elf

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets