################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/examples/basic.c \
../Core/Src/examples/pressure.c \
../Core/Src/examples/temperature.c 

OBJS += \
./Core/Src/examples/basic.o \
./Core/Src/examples/pressure.o \
./Core/Src/examples/temperature.o 

C_DEPS += \
./Core/Src/examples/basic.d \
./Core/Src/examples/pressure.d \
./Core/Src/examples/temperature.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/examples/%.o Core/Src/examples/%.su Core/Src/examples/%.cyclo: ../Core/Src/examples/%.c Core/Src/examples/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-examples

clean-Core-2f-Src-2f-examples:
	-$(RM) ./Core/Src/examples/basic.cyclo ./Core/Src/examples/basic.d ./Core/Src/examples/basic.o ./Core/Src/examples/basic.su ./Core/Src/examples/pressure.cyclo ./Core/Src/examples/pressure.d ./Core/Src/examples/pressure.o ./Core/Src/examples/pressure.su ./Core/Src/examples/temperature.cyclo ./Core/Src/examples/temperature.d ./Core/Src/examples/temperature.o ./Core/Src/examples/temperature.su

.PHONY: clean-Core-2f-Src-2f-examples

