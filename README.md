See [Serial-Mini](https://github.com/chrsbell/Serial-Mini) for the in-development updated and refined serial version!

# PS-2-Mini
Low latency PS/2 keyboard for osu!, inspired and heavily influenced by [David Bern's PS/2 Keyer](http://www.arrl.org/files/file/QEX_Next_Issue/May-Jun_2010/Bern.pdf). The PS/2 protocol implemented is based on [this article](https://www.avrfreaks.net/sites/default/files/PS2%20Keyboard.pdf). It also uses a [hardware based debouncing approach](https://hackaday.com/2015/12/09/embed-with-elliot-debounce-your-noisy-buttons-part-i/) to greatly reduce the required software debounce time.

Use Saleae Logic to view .logicdata files

To do:
* Add USB-to-serial port for firmware updates
* Remove hot swap sockets
* Make version with force-sensing resistor based switch(?)
