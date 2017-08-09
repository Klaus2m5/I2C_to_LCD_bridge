#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# LCD driver for the I2C to LCD bridge
#
#
import smbus
from time import sleep

class lcd:
   #initializes objects and lcd
   def __init__(self, addr, port=1):
      self.addr = addr
      self.bus = smbus.SMBus(port)
      self.write_cmd(0x01)


   # write a command to lcd
   def write_cmd(self, cmd):
      for retry in range(10):
         try:
            self.bus.write_byte(self.addr, cmd)
         except IOError:
            sleep(.0002)   # retry after 200탎
         else:
            if 0 < cmd < 4:
               sleep(.002)    #delay clear or home
            break    # no exception - done
      else:
         raise    # max failed retries

   # write a character to lcd (or character generator RAM)
   def write_char(self, data):
      for retry in range(10):
         try:
            self.bus.write_byte(self.addr+1, data)
         except IOError:
            sleep(.0002)   # retry after 200탎
         else:
            break    # no exception - done
      else:
         raise    # max failed retries
      
   # Read current cursor position
   def read_curpos(self):
      for retry in range(10):
         try:
            return self.bus.read_byte(self.addr)
         except IOError:
            sleep(.0002)   # retry after 200탎
         else:
            break    # no exception - done
      else:
         raise    # max failed retries

   # Read a byte from display or character generator RAM
   def read_char(self):
      for retry in range(10):
         try:
            return self.bus.read_byte(self.addr+1)
         except IOError:
            sleep(.0002)   # retry after 200탎
         else:
            break    # no exception - done
      else:
         raise    # max failed retries

   # put string function
   def display_string(self, string, line=1, pos=0):
      if 0 < line <= 4:
         pos += [0, 0, 64, 20, 84][line]
         self.write_cmd(0x80 + pos)
      for char in string:
         self.write_char(ord(char))

   # add custom characters (0 - 7)
   def load_custom_chars(self, fontdata, charnum=0):
      pos = self.read_curpos()
      self.write_cmd(charnum * 8 + 0x40);
      for char in fontdata:
         for line in char:
            self.write_char(line)         
      self.write_cmd(0x80 + pos)    # restore access to DDRAM
