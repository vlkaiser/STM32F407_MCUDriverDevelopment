# STM32F407_MCUDriverDevelopment
STM32F407 MCU - Discovery Development Board 
Follow along with the development from scratch and other projects at: https://chicgeek.blogspot.com/

Creating Device header file, peripheral header files (GPIO, I2C, SPI, UART) and an application from scratch to understand MCU driver development

### stm32f407xx.h/.c: Device driver  
Contains register addresses and macros - derived from the User Manual

### stm32f407xx_gpio_driver.h/.c: GPIO Peripheral driver 
Contains macros and definitions to use the GPIO - derived from the User Manual.

### 001_LED_Toggle.c
Test Project to validate drivers by flashing the onboard LED via Pullup/Pulldown Output Type
Test Project to validate drivers by flashing the onboard LED via Open Drain Output Type and external Pullup Resistor

### 002_LED_Btn.c
Test Project to validate drivers by reading input pin button state, and toggling LED accordingly.

### 003_Ext_LED_Btn.c
Test Project to validate drivers by reading input pin button state, and toggling LED accordingly from External components

### 004_Interrupt_Btn.c
Test Project to validate drivers reading input state of a button, generating an interrupt, and handling it to toggle an LED.

### 005_Interrupt_Btn2.c
Test Project to validate drivers generating an interrupt on button press, and handling it to toggle an LED - using configured GPIO settings.

### 006_SPI_TxData.c
Test Project to validate drivers for SPI communications.  Send 'Hello World'

### 007_SPI_Arduino_SL
Test project using the STM32407 as the SPI Master, and an Arduino as the SPI Slave.  And snickering at the 007/SPI puns.
SPI in full duplex, DFF=0, SSM=0, SCLK = 2MHz, fclk = 16MHz.  Arduino does not return data -> MISO not required.
**In progress**
