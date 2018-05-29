# Overview

This is firmware for a 'STEAMGal' board (STM32 dev board with 6 buttons, 96x64 pixel SSD1331 OLED display, an LED, and a speaker). It is intended to communicate with an ESP8266 '01' module. You know, the cheap ones with 8 pins on a 2x4 connector that doesn't fit in a breadboard.

Pins A0-A5 are unused, and broken out on two headers (one each for A0-A3 and A4-A5, both with +3.3V/GND pins). The USART1 peripheral's TX/RX pins are available on pin A2 and A3, so that's convenient. Pin A1 is used for the `CH_PD` pin which will be used to shut down the ESP8266 unless it is needed; it's a gluttonous chip.

Note that the default baud rate for most ESP8266 modules is 115200; I just clocked it down to 9600 with `AT+CIOBAUD=9600` to make it easier to debug on an oscilloscope.

# Current Status

Not really working, but it's getting there. Three 'basic' AT commands can be transmitted to the ESP8266, and the responses (up to a set buffer length) are displayed on the small OLED display. The `AT` and `AT+RST` commands respond with `OK`, and the `AT+GMR` command prints some basic firmware information.

I ran into a few small issues configuring the USART peripheral, having never used UART or USART before. What I learned:

* Make sure that the `TX` line has a pull-up resistor; it sounds like the transmitter is responsible for doing that in the UART protocol.

* Double-check the baud rate; you can try a bunch of different values quickly from a normal computer with an FTDI cable. 

* While many STM32F1 references said to configure the `RX` pin as floating input, I got 'frame format' errors when doing that on STM32F0 chips. Try configuring the `RX` pin as 'Alternate Function Output', even though it will be receiving data and acting as an input.

* If you only set the `TE` and `RE` bits once when you enable the peripheral, it looks like the STM32 sets some `IDLE` bits after an extended period of no communication. I haven't figured out how to handle those yet and they appear to block the chip from receiving data once they are set, so consider setting/resetting them before/after individual communications. I think it's also good to wait for the `TEACK` and `REACK` flags in the `ISR` register after you set them, but I'm not totally clear on that.

# Supported Chips

Currently only the STM32F031K6 and STM32F051K8 are supported, and I'm only testing on the STM32F031K6. But the STM32F303K8 would also be nice to add, as well as some chips from the L0/L4 lines, since this is sort of a starting point for future ESP8266 projects.
