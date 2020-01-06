################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/stm32f407xx_gpio_driver.c \
../Drivers/src/stm32f407xx_spi_driver.c 

OBJS += \
./Drivers/src/stm32f407xx_gpio_driver.o \
./Drivers/src/stm32f407xx_spi_driver.o 

C_DEPS += \
./Drivers/src/stm32f407xx_gpio_driver.d \
./Drivers/src/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o: ../Drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"C:/Users/vkaiser/Documents/_STM32LocalRepository/STM32F407_Drivers/Drivers/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


