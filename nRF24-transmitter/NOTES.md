# Tested on Ubuntu 18.04

# Flash command using stlink firmware:

	$ sudo st-flash write nRF24-transmitter.bin 0x08000000

# USART2 can be accessed through virtual serial port.

	$ sudo screen /dev/ttyACM0 115200


