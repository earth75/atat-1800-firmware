# atat-1800-firmware
A C firmware for the ATAT-1800 based on STM32CubeMX config files.
See earth75/atat-1800 for details on the associated hardware.

## Structure
This firmware implements the standard USB HID 1.11 interface and does not need a specific driver.
The default key layout is ISO US with keypad, but it is meant to be easy enough to adapt to another layout if needed.

The matrix scanning and backlight updates are done on timer interrupts (TIM7 and TIM14)

Support for macros and more complex behaviour is planned but not implemented yet.

The pin settings, clock and peripherals configuration is described in the STM32CubeMX project (.ioc file).

The project is set up for STM32CubeIDE with no debug probe, so there is a custom step in the build settings.
STM32_Programmer_CLI.exe is called at the end of the build process and tries to find an STM32F072 in DFU mode to load the hex into it.
