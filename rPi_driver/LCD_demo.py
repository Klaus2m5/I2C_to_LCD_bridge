#!/usr/bin/env python
#
# 16 x 2   L C D   d e m o
#
import LCD_bridge
import time
sleep = time.sleep

lcd = LCD_bridge.lcd(0x10)

lcd.display_string("    LCD Demo     L C D  D e m o ", 1)
sleep(1)
lcd.display_string("scroll display   forward/backw. ", 2)
sleep(1)
for x in range(16):
   lcd.write_cmd(0x18)  # display shift right
   sleep(.1)
sleep(1)
for x in range(16):
   lcd.write_cmd(0x1c)  # display shift left
   sleep(.1)
sleep(1)
lcd.write_cmd(0x04)  # move cursor backward during write
lcd.display_string("  drawkcab etirw", 2, 15)   # write backward
sleep(1)
lcd.display_string("                ", 2, 15)
lcd.write_cmd(0xc0 + 15)  # set cursor to end of line 2 
for x in range(65, 81):
   lcd.write_char(x)
   sleep(.2)
sleep(.8)
lcd.write_cmd(0x06)  # move cursor forward during write
lcd.display_string("                ", 2)
lcd.write_cmd(0x0f)  # set cursor on (underscore & blinking)
lcd.display_string("cursor on", 2)
sleep(1)
lcd.display_string("move cursor", 2)
sleep(.5)
for x in range(11):
   lcd.write_cmd(0x10)  # move cursor left
   sleep(.2)
for x in range(11):
   lcd.write_cmd(0x14)  # move cursor right
   sleep(.2)
sleep(.8)
lcd.display_string(" scroll forward while writing   ", 1)
lcd.display_string(" while writing                  ", 2)
sleep(1)
lcd.write_cmd(0xc0)  # set cursor to line 2 
lcd.write_cmd(0x07)  # shift display forward during write
for x in range(8):
   lcd.write_cmd(0x1c)  # display shift left
   sleep(.1)
for x in range(32, 72):
   lcd.write_char(x)
   sleep(.2)
lcd.write_cmd(0xc0)  # set cursor to line 2 
for x in range(72, 112):
   lcd.write_char(x)
   sleep(.2)
sleep(1)
lcd.write_cmd(0x06)  # move cursor forward during write
lcd.write_cmd(0x0c)  # cursor off
lcd.write_cmd(0x01)  # clear display, home cursor
lcd.display_string("switch display", 1)
lcd.display_string("on/off command", 2)
for x in range(5):
   lcd.write_cmd(0x08)  # display off
   sleep(.3)
   lcd.write_cmd(0x0c)  # display on (with cursor off)
   sleep(.3)
sleep(.7)
for x in range(5):
    lcd.display_string("Time: %s" %time.strftime("%H:%M:%S"), 1)
    lcd.display_string("Date: %s" %time.strftime("%m/%d/%Y"), 2)
    sleep(1)

lcd.write_cmd(0x01)  # clear display, home cursor
lcd.display_string("read cursor char", 1)
lcd.write_cmd(0x80)  # set cursor home (line 1, pos 0)
sleep(1)
lcd.write_cmd(0x0e)  # cursor underscore
for x in range(16):
   cur = lcd.read_curpos()
   char = lcd.read_char()
   lcd.display_string("{:3d} @ {:2d}".format(char, cur), 2)
   lcd.write_cmd(0x80 + cur)  # restore cursor
   sleep(.8)
   lcd.write_cmd(0x14)  # move cursor right
lcd.write_cmd(0x0c)  # cursor off

lcd.display_string("  user defined  ", 1)
lcd.display_string("   characters   ", 2)
face = [
   [0b00000, 0b00000, 0b00111, 0b01000, 0b10000, 0b10010, 0b10101, 0b10010],
   [0b00000, 0b00000, 0b11100 ,0b00010, 0b00001, 0b01001, 0b10101, 0b01001],
   [0b10001, 0b10001, 0b10000, 0b10100, 0b10011, 0b01000, 0b00111, 0b00000],
   [0b10001, 0b10001, 0b00001, 0b00101, 0b11001, 0b00010, 0b11100, 0b00000],
   [0b00000, 0b00000, 0b00111, 0b01000, 0b10000, 0b10000, 0b10111, 0b10000],
   [0b00000, 0b00000, 0b11100, 0b00010, 0b00001, 0b00001, 0b11101, 0b00001],
   [0b10001, 0b10001, 0b10000, 0b10011, 0b10100, 0b01000, 0b00111, 0b00000],
   [0b10001, 0b10001, 0b00001, 0b11001, 0b00101, 0b00010, 0b11100, 0b00000],
]
lcd.load_custom_chars(face)
sleep(1)
lcd.write_cmd(0x01)  # clear display, home cursor
lcd.write_cmd(0x80 + 7)  # set cursor to center-1 of line 1 
lcd.write_char(0)
lcd.write_char(1)
lcd.write_cmd(0xc0 + 7)  # set cursor to center-1 of line 2 
lcd.write_char(2)
lcd.write_char(3)
for x in range(7):
   sleep(4)
   lcd.write_cmd(0x80 + 7)  # set cursor to center-1 of line 1 
   lcd.write_char(4)
   lcd.write_char(5)
   sleep(.2)
   lcd.write_cmd(0x80 + 7)  # set cursor to center-1 of line 1 
   lcd.write_char(0)
   lcd.write_char(1)
lcd.write_cmd(0xc0 + 7)  # set cursor to center-1 of line 2 
lcd.write_char(6)
lcd.write_char(7)
lcd.display_string("the", 1)
lcd.display_string("end", 2, 13)

