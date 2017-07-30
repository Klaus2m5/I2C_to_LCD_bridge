I2C to HD44780 compatible LCD bridge

An AtTiny26 serves as a serial to parallel converter connecting a HD44780 LCD
to an I2C bus. Compared to a PCF8574 it provides

  •  Full 8-bit bus read & write operation. No need to switch the LCD to
     4-bit mode.
  •  Tags (RS, R/W, E) are automatically generated for each byte based on the
     I2C address. Tags need not be set in multiple write operations.
  •  Automatic clock stretching and write buffering on LCD busy. No need for
     timed operations.
  •  Pre-initializes the LCD on power-up with custom reset values and an
     optional splash message.
  •  Automatic PWM backlight control.
  
The converter operates at 400 kHz maximum I2C clock. 100kHz operation avoids
clock stretching completely to overcome the raspberry pi I2C bug.
