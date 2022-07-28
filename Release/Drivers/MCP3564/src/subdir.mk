################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP3564/src/mcp3564.c 

OBJS += \
./Drivers/MCP3564/src/mcp3564.o 

C_DEPS += \
./Drivers/MCP3564/src/mcp3564.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP3564/src/%.o Drivers/MCP3564/src/%.su: ../Drivers/MCP3564/src/%.c Drivers/MCP3564/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F373xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MCP3564/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MCP3564-2f-src

clean-Drivers-2f-MCP3564-2f-src:
	-$(RM) ./Drivers/MCP3564/src/mcp3564.d ./Drivers/MCP3564/src/mcp3564.o ./Drivers/MCP3564/src/mcp3564.su

.PHONY: clean-Drivers-2f-MCP3564-2f-src

