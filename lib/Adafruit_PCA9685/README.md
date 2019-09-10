## Adafruit 16-channel PWM & Servo driver

This library is for the [Adafruit 16-channel PWM & Servo board](http://www.adafruit.com/products/815) when being used with a Spark Core, and only Spark Core (sorry Arduino)


### Connections

These boards use I2C to communicate. 2 pins are required for the interface:

    SCL -> D1
    SDA -> D0

### Things to Note

### Default I2C Slave Address 
With no address jumpers, the default I2C slave address of the board is `100 0000` or `0x40`. This default is set in the header file. If you are altering the address with jumpers, make sure to set it in the software to match.

### setVal vs setPWM
`setVal()` provides a some abstractions and is recommended over using `setPWM()`.

### Examples
The examples included in this library are outdated, they don't work with the renaming that has occurred

## Credits
Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried (Ladyada) for Adafruit Industries.  
Adapted by Paul Kourany to work with Spark Core  
Extended by Chad Bean & Louis DeScioli for Grove Labs  

## License
BSD license, check LICENSE for more information
All text above must be included in any redistribution
