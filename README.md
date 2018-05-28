# Overview

This is firmware for a 'STEAMGal' board (STM32 dev board with 6 buttons, 96x64 pixel SSD1331 OLED display, an LED, and a speaker). It is intended to communicate with an ESP8266 '01' module. You know, the cheap ones with 8 pins on a 2x4 connector that doesn't fit in a breadboard.

Pins A0-A5 are unused, and broken out on two headers (one each for A0-A3 and A4-A5, both with +3.3V/GND pins). The USART1 peripheral's TX/RX pins are available on pin A2 and A3, so that's convenient. Pin A1 is used for the `CH_PD` pin which will be used to shut down the ESP8266 unless it is needed; it's a gluttonous chip.

# Current Status

Not really working, but it's getting there. Three 'basic' AT commands can be transmitted to the ESP8266, and they seem to work based on how the LEDs blink on the ESP8266 board. The 'reset' command makes it blink twice, and the general 'are you there?' commands make it blink once as data is transmitted back.

But I haven't gotten the STM32 receiving valid responses yet; the `ISR` register shows a 'frame format error' flag the first time that the `RXNE` bit is set, and it seems like only `0`s are received in the `RDR` register. So...hm. Work continues.

Currently only the STM32F031K6 and STM32F051K8 are supported, but the STM32F303K8 would be nice to add.
