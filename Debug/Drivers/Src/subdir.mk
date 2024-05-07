################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/Src/stm32f103xx_gpio_driver.cpp 

OBJS += \
./Drivers/Src/stm32f103xx_gpio_driver.o 

CPP_DEPS += \
./Drivers/Src/stm32f103xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.cpp Drivers/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"D:/Dell/Documents/Projects/STM32_Course/stm32f103_drivers_oop/Drivers/Src" -I"D:/Dell/Documents/Projects/STM32_Course/stm32f103_drivers_oop/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f103xx_gpio_driver.cyclo ./Drivers/Src/stm32f103xx_gpio_driver.d ./Drivers/Src/stm32f103xx_gpio_driver.o ./Drivers/Src/stm32f103xx_gpio_driver.su

.PHONY: clean-Drivers-2f-Src

