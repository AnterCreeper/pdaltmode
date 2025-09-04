################################################################################
# MRS Version: 2.2.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32x035_it.c \
../User/gpio.c \
../User/iic.c \
../User/iic_dev.c \
../User/main.c \
../User/mpd.c \
../User/sys.c \
../User/system_ch32x035.c \
../User/timer.c \
../User/usbbc.c \
../User/ws2812.c 

C_DEPS += \
./User/ch32x035_it.d \
./User/gpio.d \
./User/iic.d \
./User/iic_dev.d \
./User/main.d \
./User/mpd.d \
./User/sys.d \
./User/system_ch32x035.d \
./User/timer.d \
./User/usbbc.d \
./User/ws2812.d 

OBJS += \
./User/ch32x035_it.o \
./User/gpio.o \
./User/iic.o \
./User/iic_dev.o \
./User/main.o \
./User/mpd.o \
./User/sys.o \
./User/system_ch32x035.o \
./User/timer.o \
./User/usbbc.o \
./User/ws2812.o 


EXPANDS += \
./User/ch32x035_it.c.253r.expand \
./User/gpio.c.253r.expand \
./User/iic.c.253r.expand \
./User/iic_dev.c.253r.expand \
./User/main.c.253r.expand \
./User/mpd.c.253r.expand \
./User/sys.c.253r.expand \
./User/system_ch32x035.c.253r.expand \
./User/timer.c.253r.expand \
./User/usbbc.c.253r.expand \
./User/ws2812.c.253r.expand 



# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-wch-elf-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -mstrict-align -msave-restore -fmax-errors=20 -Ofast -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -flto -Werror -Wunused -Wall -Wno-format -g -I"/home/hp/pdaltmode/Debug" -I"/home/hp/pdaltmode/Core" -I"/home/hp/pdaltmode/User" -I"/home/hp/pdaltmode/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

