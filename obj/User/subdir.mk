################################################################################
# MRS Version: 2.1.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32x035_it.c \
../User/iic.c \
../User/main.c \
../User/system_ch32x035.c 

C_DEPS += \
./User/ch32x035_it.d \
./User/iic.d \
./User/main.d \
./User/system_ch32x035.d 

OBJS += \
./User/ch32x035_it.o \
./User/iic.o \
./User/main.o \
./User/system_ch32x035.o 



# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-wch-elf-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -mstrict-align -msave-restore -fmax-errors=20 -Ofast -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -flto -Werror -Wunused -Wuninitialized -g -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Debug" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Core" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/User" -I"/home/lenovo/mounriver-studio-projects/pdaltmode/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
