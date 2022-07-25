################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP3564/example/Src/main.c \
../Drivers/MCP3564/example/Src/mcp3564.c \
../Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.c \
../Drivers/MCP3564/example/Src/stm32f3xx_it.c \
../Drivers/MCP3564/example/Src/syscalls.c \
../Drivers/MCP3564/example/Src/sysmem.c \
../Drivers/MCP3564/example/Src/system_stm32f3xx.c 

OBJS += \
./Drivers/MCP3564/example/Src/main.o \
./Drivers/MCP3564/example/Src/mcp3564.o \
./Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.o \
./Drivers/MCP3564/example/Src/stm32f3xx_it.o \
./Drivers/MCP3564/example/Src/syscalls.o \
./Drivers/MCP3564/example/Src/sysmem.o \
./Drivers/MCP3564/example/Src/system_stm32f3xx.o 

C_DEPS += \
./Drivers/MCP3564/example/Src/main.d \
./Drivers/MCP3564/example/Src/mcp3564.d \
./Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.d \
./Drivers/MCP3564/example/Src/stm32f3xx_it.d \
./Drivers/MCP3564/example/Src/syscalls.d \
./Drivers/MCP3564/example/Src/sysmem.d \
./Drivers/MCP3564/example/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP3564/example/Src/%.o Drivers/MCP3564/example/Src/%.su: ../Drivers/MCP3564/example/Src/%.c Drivers/MCP3564/example/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F373xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MCP3564-2f-example-2f-Src

clean-Drivers-2f-MCP3564-2f-example-2f-Src:
	-$(RM) ./Drivers/MCP3564/example/Src/main.d ./Drivers/MCP3564/example/Src/main.o ./Drivers/MCP3564/example/Src/main.su ./Drivers/MCP3564/example/Src/mcp3564.d ./Drivers/MCP3564/example/Src/mcp3564.o ./Drivers/MCP3564/example/Src/mcp3564.su ./Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.d ./Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.o ./Drivers/MCP3564/example/Src/stm32f3xx_hal_msp.su ./Drivers/MCP3564/example/Src/stm32f3xx_it.d ./Drivers/MCP3564/example/Src/stm32f3xx_it.o ./Drivers/MCP3564/example/Src/stm32f3xx_it.su ./Drivers/MCP3564/example/Src/syscalls.d ./Drivers/MCP3564/example/Src/syscalls.o ./Drivers/MCP3564/example/Src/syscalls.su ./Drivers/MCP3564/example/Src/sysmem.d ./Drivers/MCP3564/example/Src/sysmem.o ./Drivers/MCP3564/example/Src/sysmem.su ./Drivers/MCP3564/example/Src/system_stm32f3xx.d ./Drivers/MCP3564/example/Src/system_stm32f3xx.o ./Drivers/MCP3564/example/Src/system_stm32f3xx.su

.PHONY: clean-Drivers-2f-MCP3564-2f-example-2f-Src

