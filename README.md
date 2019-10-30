# STM32F407_MCUDriverDevelopment
STM32F407 MCU - Discovery Development Board 
Follow along with the development from scratch and other projects at: https://chicgeek.blogspot.com/

Creating Device header file, peripheral header files (GPIO, I2C, SPI, UART) and an application from scratch to understand MCU driver development

# stm32f407xx.h/.c: Device driver  
Contains register addresses and macros - derived from the User Manual

# stm32f407xx_gpio_driver.h/.c: GPIO Peripheral driver 
Contains macros and definitions to use the GPIO - derived from the User Manual.

# 001_LED_Toggle.c
Test Project to validate drivers by flashing the onboard LED.
