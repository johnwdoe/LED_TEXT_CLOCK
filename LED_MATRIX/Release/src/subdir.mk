################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/bcd.c \
../src/i2c.c \
../src/ir.c \
../src/mxcontrol.c \
../src/uart.c 

OBJS += \
./src/bcd.o \
./src/i2c.o \
./src/ir.o \
./src/mxcontrol.o \
./src/uart.o 

C_DEPS += \
./src/bcd.d \
./src/i2c.d \
./src/ir.d \
./src/mxcontrol.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega8 -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


