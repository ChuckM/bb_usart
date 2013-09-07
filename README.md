README
------

This example program sets up USART6 on the [STMF32F4DISC-BB][bb] base 
board for use as a console port. Note it is designed to work with the
existing [libopencm3][lib] build setup. Normally it would live in
libopencm3-examples/examples/stm32/f4/stm32f4-discovery/bb_usart

Some things to note before this will work. 

The pins in the DB9 shell connect to a level shifter on the board but
that level shifter is not connected to the actual UART pins (PC6, and
PC7) unless you install jumpers on the end of connector for (labelled
CON4 on the base board). 

The DB9 connector is a 'male' connector, so you cannot use a regular
PC cable to connect it to your PC, instead you can either connect a 
NULL modem cable between the board and your PC, or you can wire the
pins on CON4 from 1->4 and from 2->3 (which crosses over transmit and
recieve) and put a 'gender changer' on the DB9 and connect a regular
cable. In all cases the only signals that are used are receive, transmit
and ground so you have to be sure your terminal program is ignoring
hardware flow control.

The terminal settings for the receiving device/PC are 115,200 8n1 by
default if you call 'setup_bb()' with a different baudrate that works
fine as well. If you are using the [BlackMagic Debugger][bmd] then either
set the baudrate below 38,400 or make sure you have the 'high baudrate'
version of the firmware.

[bb]: http://www.element14.com/community/community/knode/dev_platforms_kits/element14_dev_kits/stm32f4-discovery-expansion-boards

[bmd]: http://www.blacksphere.co.nz/main/blackmagic

[lib]: https://github.com/libopencm3/libopencm3-examples
