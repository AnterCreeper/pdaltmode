################################################################################
# MRS Version: 2.1.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32x035_it.c \
../User/gpio.c \
../User/iic.c \
../User/main.c \
../User/mpd.c \
../User/sys.c \
../User/system_ch32x035.c \
../User/timer.c \
../User/usbbc.c 

C_DEPS += \
./User/ch32x035_it.d \
./User/gpio.d \
./User/iic.d \
./User/main.d \
./User/mpd.d \
./User/sys.d \
./User/system_ch32x035.d \
./User/timer.d \
./User/usbbc.d 

OBJS += \
./User/ch32x035_it.o \
./User/gpio.o \
./User/iic.o \
./User/main.o \
./User/mpd.o \
./User/sys.o \
./User/system_ch32x035.o \
./User/timer.o \
./User/usbbc.o 



# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv-wch-elf-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -mstrict-align -msave-restore -fmax-errors=20 -Ofast -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -flto -Werror -Wunused -Wall -Wno-format -g -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Debug" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Core" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/User" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@
