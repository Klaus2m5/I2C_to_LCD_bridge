;
;
;   I 2 C   t o   L C D   bridge
;
; an I2C slave to HD44780 compatible LCD adapter
;
; Copyright (C) 2017  Klaus Dormann
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program.  If not, see <http://www.gnu.org/licenses/>.
;
; contact info at http://2m5.de or email K@2m5.de
;
; Version: 1.0    - 29.07.2017
;
; ATTINY26 fuse settings (if other then default):
;   8 MHz internal oscillator
;   Startup: 14 CK + 65ms (to allow LCD to power up)
;   BOD enabled, 4.0V                "
;
; related docs:
;   I2C_to_HD44780_compatible_LCD_bridge.pdf
;
; description of hardware:
;   porta      8bit databus to LCD
;   portb0     SDA i/o - data read/write (MOSI)
;   portb2     SCL i/o - clock from master / -wait from slave (SCK)
;   portb4     read/-write to LCD
;   portb5     register select to LCD, 0 = command, 1 = data
;   portb3     enable to 1st LCD controller
;   portb6     enable to 2nd LCD controller
;   portb1     PWM to backlight, 0 = on, 1 = off
;
; Changes:
;   10.01.2017 start of development
;   29.07.2017 initial release 1.0
 
            .NOLIST
            .INCLUDE "tn26def.inc"
            .INCLUDE "sam.inc"
            .LIST
                   
.macro      addi
            subi  @0,-@1      ; subtract the negative of an immediate value
.endmacro
.macro      ldiz              ; load Z with flash address
            ldi   zl,low(@0)
            ldi   zh,high(@0)
.endmacro
;
; reserved registers
;
; (I): used during interrupt, (C): interrupt to program communication
; 
; 0x00 non immediate
.DEF  unused_r0   =r0         ; unused
.DEF  s           =r1         ; (I) status save during interrupt
.DEF  timout1     =r2         ; backlight on seconds remaining
.DEF  timout2     =r3         ; backlight dimmed seconds remaining
; 0x04 - used as constants
.DEF  i2c_idle    =r4         ; start condition interrupt enabled,
                              ; overflow interrupt disabled, wire mode: TWI
.DEF  i2c_actv    =r5         ; +overflow rupt enabled, +hold SCL on overflow
.DEF  adr_ack     =r6         ; (C) send ack (0x00) or nak (0xff) for address match 
.DEF  allon       =r7         ; 0xff
; 0x08
.DEF  i2c_clr     =r8         ; clear all flags, clock 8 bits
.DEF  i2c_clr8    =r9         ; clear flags -start, clock 8 bits     
.DEF  i2c_clr1    =r10        ; clear flags -start, clock 1 bit
.DEF  wrt_ack     =r11        ; next write acknoledge
; 0x0c
.DEF  c           =r12        ; GPR
.DEF  d           =r13        ; GPR
.DEF  k           =r14        ; (I) interrupt GPR
.DEF  i2c_adr     =r15        ; (C) I2C slave address
; 0x10 immediate
.DEF  a           =r16        ; general purpose register
.DEF  i           =r17        ; (I) interrupt GPR
.DEF  i2c_buf     =r18        ; (I) i2c load at write (3) to store at wack (4)
.DEF  lcd_adr     =r19        ; (C) LCD address bits as follows:
   .EQU  lcd_sel     =7       ;   currently selected LCD - 0 = LCD1, 1 = LCD2
                              ;   bit 7 is unused at port b (reset)
                              ;   RS & RW same as port b (lcd_ctrl)
; 0x14
.DEF  tick        =r20        ; 8 ms ticks remaining in second
.DEF  b           =r21        ; GPR
.DEF  unused_r22  =r22        ; unused
.DEF  unused_r23  =r23        ; unused
; 0x18 immediate word
.DEF  unused_r24  =r24        ; unused
.DEF  unused_r25  =r25        ; unused
;     x                       ; (C) write buffer in from i2c
; 0x1c
;     y                       ; (C) write buffer out to lcd
;     z                       ; I2C state (overflow interrupt address)

; register alias
#define  zero     yh
#define  i2c_off  yh

;
; Configuration
;
;   internal
.EQU  cf_osccal   =0xff       ;0xff  oscillator calibration - 0xff = don't set
;   I2C
.EQU  cf_lcd_nmbr =1          ;1     1         or 2 LCD controllers 
.EQU  cf_i2c_adr  =0x20       ;0x20  0x20-0x23 or 0x20-027 (0x24-0x27 = enable 2)
;     I2C configuration sanity check
.IF   cf_lcd_nmbr == 1
   .IF   cf_i2c_adr & 3
      .ERROR   "For 1 LCD controller the i2c address must be dividable by 4"
   .ENDIF
.ELSE
   .IF   cf_lcd_nmbr == 2
      .IF   cf_i2c_adr & 7
         .ERROR   "For 2 LCD controllers the i2c address must be dividable by 8"
      .ENDIF
   .ELSE
      .ERROR   "The numder of LCD controllers must be 1 or 2"
   .ENDIF
.ENDIF 
;   LCD reset (refer to LCD datasheet)
.EQU  cf_lcd_rs_sf=0b00111000 ; 001 set function: 8 bit, 2 lines, font, x, x
.EQU  cf_lcd_rs_do=0b00001100 ; 00001 on/off: display, cursor, blink
.EQU  cf_lcd_rs_em=0b00000110 ; 000001 entry mode: cursor +/-, disp. shift
; splash message to be displayed by reset
; replace with your message or comment out for no message
#define  splash   " - I2C to LCD - "
;   LCD backlight
.EQU  cf_lcd_on   =0xff       ;0xff  backlight full on after data access
.EQU  cf_lcd_dim  =0x20       ;0x20  backlight dimmed after 1st timeout
.EQU  cf_lcd_off  =0x00       ;0x00  backlight off after 2nd timeout
.EQU  cf_lcd_to1  =10         ;10    timeout 1 in seconds (max 255)
.EQU  cf_lcd_to2  =50         ;50    timeout 2 in seconds (max 255)

;
; IO registers
;
; 0x2d USICR
.EQU  con_idle    =0b10101000 ; start condition interrupt enabled,
                              ; overflow interrupt disabled, wire mode: TWI
.EQU  con_actv    =0b11111000 ; +overflow rupt enabled, +hold SCL on overflow
; 0x2e USISR
.EQU  con_clr     =0b11100000 ; clear all flags, clock 8 bits
.EQU  con_clr8    =0b01100000 ; clear flags -start, clock 8 bits     
.EQU  con_clr1    =0b01101110 ; clear flags -start, clock 1 bit

;0x36-0x38 PortB
; I2C signals
.EQU  i2c_in      =pinb       ; I2C input
.EQU  i2c_ddr     =ddrb       ; I2C open collector output inverted 1=0
.EQU  i2c_out     =portb      ; I2C output
.EQU  i2c_sda     =0          ;   data i/o
.EQU  i2c_scl     =2          ;   clock i/o
.EQU  i2c_lcd     =0b101      ;   SCL & SDA always on for LCD control port
; LCD control signals
.EQU  lcd_ctrl    =portb      ; LCD control port
.EQU  lcd_cddr    =ddrb       ; LCD control DDR
.EQU  lcd_rs      =5          ;   register select command/status = 0, data = 1 
.EQU  lcd_rw      =4          ;   read/-write
.EQU  lcd_e1      =3          ;   enable LCD 1
.EQU  lcd_e2      =6          ;   enable LCD 2
.EQU  lcd_pwm     =1          ;   backlight PWM
.EQU  lcd_cddr_on =0b1111010  ; LCD control output

;0x39-0x3b PortA
; LCD data bus
.EQU  lcd_out     =porta      ; LCD bus output
.EQU  lcd_in      =pina       ; LCD bus input
.EQU  lcd_ddr     =ddra       ; LCD bus DDR

; LCD read macro
.macro         read_lcd       ;@0 =  input register
   .if   cf_lcd_nmbr == 2
      sbrc  lcd_adr,lcd_sel      ;which LCD?
      rjmp  read_lcd2
   .endif
   sbi   lcd_ctrl,lcd_e1      ;pulse LCD 1 enable 250 ns
   cbi   lcd_ctrl,lcd_e1
   in    @0,lcd_in            ;read lcd bus 187.5 ns after enable
   .if   cf_lcd_nmbr == 2
      rjmp  read_lcd1
read_lcd2:
      sbi   lcd_ctrl,lcd_e2      ;pulse LCD 2 enable 250 ns
      cbi   lcd_ctrl,lcd_e2
      in    @0,lcd_in            ;read lcd bus 187.5 ns after enable
read_lcd1:
   .endif
.endmacro

;
; SRAM usage
;
.DSEG
wrt_buf:       .byte 96       ;write buffer
wrt_bufe:                     ;end of buffer
               ;stack: 32 bytes

; end of SRAM

.CSEG

;
; reset and interrupt vectors
;
   rjmp  reset       ; Reset handler
   rjmp  illegal     ; INT0 handler
   rjmp  illegal     ; Pin Change Interrupt
   rjmp  illegal     ; Timer1 compare match A 
   rjmp  illegal     ; Timer1 compare match B
   rjmp  illegal     ; Timer1 overflow handler
   rjmp  illegal     ; Timer0 overflow handler
   rjmp  i2c_start   ; USI start
   in    s,sreg      ; USI overflow - save flags
   ijmp              ;    - dispatch according to state
                     ; EEPROM ready vector not available
   rjmp  illegal     ; Analog comparator
   rjmp  illegal     ; ADC conversion complete

;
; ******************* Interrupt Routines ****************
;

;
; an illegal interrupt has occured
;
illegal:
   pop   a                 ;remove return address
   pop   a
   ldi   a,1               ;flash backlight once

; error flasher
;
;   RESET REQUIRED!
;
;   flashes backlight, a = count
;   1 = illegal interrupt, 2 = LCD busy timeout on write,
;   3 = LCD busy timeout on read
;
err_flash:
   cli
   ldi   b,lcd_cddr_on     ;release i2c pins
   out   lcd_cddr,b
   out   usicr,i2c_off     ;set i2c off
   out   usisr,i2c_clr     ;clear flags & counter
   do    flash
      mov   c,a               ;# of flashes
      do       flash_times
         out   ocr1a,allon       ;flash on
         ldi   tick,20
         rcall flash_wait
         out   ocr1a,zero        ;flash off
         ldi   tick,20
         rcall flash_wait
         dec   c
      loopne   flash_times
      ldi   tick,100          ;flash pause
      rcall flash_wait
   loop  flash

flash_wait:
   do    flash_wait_loop
      in    b,tifr
      andi  b,(1<<tov1)
      ifne  flash_tick
         out   tifr,b
         dec   tick
         exiteq   flash_wait_loop
      end   flash_tick
   loop  flash_wait_loop
   ret

;
; I2C start condition interrupt
;
i2c_start:
   in    s,sreg            ;save flags
   do    i_sta_wait_scl
      sbic  i2c_in,i2c_scl    ;wait for start completed (SCL low)
      ifs   i_sta_scl
         out   usicr,i2c_actv    ;allow overflow rupts, hold SCL on overflow
         out   usisr,i2c_clr     ;clear flags & counter
         ldiz  i2c_adm           ;state: check address
         mov   adr_ack,allon     ;default nak, ~160 cycles for ack by main
         out   sreg,s            ;restore flags
         reti
      end   i_sta_scl
      sbis  i2c_in,i2c_sda    ;stop while waiting
      ifs   i_sta_stop
         out   usicr,i2c_idle    ;stop condition while waiting (SDA high)
         out   usisr,i2c_clr     ;clear flags & counter
         cbi   i2c_ddr,i2c_sda   ;release sda control
         ldiz  illegal           ;state: idle - no overflow interrupts
         out   sreg,s            ;restore flags
         reti
      end   i_sta_stop
   loop  i_sta_wait_scl

;
; I2C overflow interrupt states
;

;
; state 1 = address compare
;
i2c_adm:    
   in    i,usidr           ;address match?
   eor   i,i2c_adr
   cpi   i,(cf_lcd_nmbr<<2);number of LCD controllers * 4
   brsh  i_ovidl           ;not my address
      out   usidr,adr_ack     ;send ack/nak
      out   usisr,i2c_clr1    ;clear flags -start, count 1 bit
      sbi   i2c_ddr,i2c_sda   ;sda is output
      tst   adr_ack           ;test lcd_busy
      brne  i_ovidl_sda       ;drop to idle with nak
      .if   cf_lcd_nmbr == 2
         bst   i,2               ;LCD select
         bld   lcd_adr,lcd_sel
      .endif
      bst   i,1               ;RS (command or data)
      bld   lcd_adr,lcd_rs
      bst   i,0               ;R/W (read or write)
      bld   lcd_adr,lcd_rw
      ldiz  i2c_rack          ;state: read acknowledge
      iftc  i_adm_read
         ldiz  i2c_aack          ;state: write address acknowledge
      end   i_adm_read
      out   sreg,s
      reti
;
; state 2 = ack for write address
;
i2c_aack:
   cbi   i2c_ddr,i2c_sda   ;sda is input for a master write
   out   usisr,i2c_clr8    ;clear flags -start, clock 8 bits
   ldiz  i2c_write         ;state: write data or command
   mov   wrt_ack,zero      ;send ack after write
   ldi   xl,wrt_buf        ;reset buffer pointers
   ldi   yl,wrt_buf
   out   sreg,s
   reti

;common exit to drop to idle
i_ovidl_sda:
   cbi   i2c_ddr,i2c_sda   ;sda is input when idle
i_ovidl:
   out   usicr,i2c_idle    ;drop to idle
   out   usisr,i2c_clr8    ;clear flags -start
   ldiz  illegal           ;state: waiting for start
   out   sreg,s
   reti

;
; state 3 = write data or command
;
i2c_write:
   in    i,usidr           ;save data received
   out   usidr,wrt_ack     ;send ACK or NAK
   sbi   i2c_ddr,i2c_sda   ;sda is output
   out   usisr,i2c_clr1    ;clear flags -start, count 1 bit
   ldiz  i2c_wack          ;state: acknowledge (write data)
   out   tcnt1,zero        ;start timeout
   st    x+,i              ;buffer output to LCD
   out   sreg,s
   reti
;
; state 4 = ack for write data or command sent
;
i2c_wack:
   cbi   i2c_ddr,i2c_sda   ;sda is input for a master write
   out   usisr,i2c_clr8    ;clear flags -start, clock 8 bits
   ldiz  i2c_write         ;state: write data
   cpi   xl,wrt_bufe-1     ;buffer limit?
   ifeq  i_wack_limit
      mov   wrt_ack,allon     ;send nak after next write
   end   i_wack_limit
   brsh  i_ovidl           ;over limit, force idle
   out   sreg,s
   reti
;
; state 5 = read data sent
;
i2c_read:
   cbi   i2c_ddr,i2c_sda   ;sda is input to receive ack
   out   usidr,zero
   out   usisr,i2c_clr1    ;clear flags -start, count 1 bit
   ldiz   i2c_rack         ;state: read ack
   out   sreg,s
   reti
;
; state 6 = ack/nak for read received (ack sent after address) LCD 1 or 2
;
i2c_rack:
   in    i,usidr           ;test previous ack
   tst   i
   brne  i_ovidl           ;nak - drop to idle
   sbrs  lcd_adr,lcd_rs    ;data?
   ifs   i_rack_data
      out   tcnt1,zero
      do    i_rack_busy
         read_lcd i              ;wait on busy
         tst   i
      exitpl i_rack_busy
         in    i,tcnt1
         cpi   i,3               ;>64 탎?
         ifsh  i_rack_timout
            pop   a                 ;purge return address from stack
            pop   a
            ldi   a,3               ;flash backlight 3 times
            rjmp  err_flash
         end   i_rack_timout
      loop  i_rack_busy
      out   lcd_ctrl,lcd_adr
   end   i_rack_data
   read_lcd i
   out   usidr,i
   sbi   i2c_ddr,i2c_sda   ;sda is output
   out   usisr,i2c_clr8    ;clear flags -start, count 8 bit
   ldi   i,(1<<lcd_rw)|i2c_lcd   ;default to read status
   out   lcd_ctrl,i
   ldiz  i2c_read          ;state: read sent
   out   sreg,s
   reti

;
; ******************* RESET ****************
;

; timed write to LCD during reset
write_lcd:
   out   tcnt1,zero        ;timed write
   out   lcd_out,a
   out   lcd_ctrl,i        ;strobe LCD enable
   nop                     ;250 ns
   out   lcd_ctrl,d
   do       w_lcd_wait
      in    c,tcnt1
      cp    c,b
   looplo   w_lcd_wait
   ret
   
reset:
   ldi   a,ramend          ;set Stack-Pointer
   out   sp,a
   ldi   a,con_idle        ;initialize constants
   mov   i2c_idle,a
   ldi   a,con_actv
   mov   i2c_actv,a
   ldi   a,con_clr
   mov   i2c_clr,a
   ldi   a,con_clr8
   mov   i2c_clr8,a
   ldi   a,con_clr1
   mov   i2c_clr1,a
   clr   allon             ;0xff constant
   dec   allon
   clr   yh                ;set write buffer pointers
   clr   xh
   ldi   yl,wrt_buf
   ldi   xl,wrt_buf

   .if   cf_osccal < 0xff
      ldi   a,cf_osccal       ;set osccal if configured
      out   osccal,a
   .endif
   ldi   a,cf_i2c_adr      ;set i2c address
   mov   i2c_adr,a
   
   ldi   a,0xff            ;set 122 Hz backlight and 8 ms tick timer
   out   ocr1c,a
   ldi   a,cf_lcd_off      ;begin with backlight off
   out   ocr1a,a
   ldi   a,(3<<com1a0)|(1<<pwm1a)   ;pwm mode
   out   tccr1a,a
   ldi   a,9               ;prescaler ck/256
   out   tccr1b,a
   sbi   ddrb,1            ;ocr1a output as backlight pwm

   ldi   a,lcd_cddr_on     ;initialize LCD bus
   out   lcd_cddr,a
   out   lcd_ctrl,zero
   in    d,lcd_ctrl
   out   lcd_ddr,allon     ;write mode
   ldi   i,(1<<lcd_e1)|(1<<lcd_e2) ;strobe both
   ldi   b,130             ;4.1 ms + prescaler sync
   ldi   a,0x30            ;force 8 bit mode
   rcall write_lcd
   rcall write_lcd
   ldi   b,4               ;128 탎
   rcall write_lcd
   ldi   b,2               ;64 탎
   ldi   a,cf_lcd_rs_sf & 0x3f | 0x30  ;set function, forced 8 bit mode
   rcall write_lcd
   ldi   a,cf_lcd_rs_do & 0xf | 8      ;display/cursor on/off
   rcall write_lcd
   ldi   b,57              ;1.8 ms
   ldi   a,1               ;clear & home
   rcall write_lcd
   ldi   b,2               ;64 탎
   ldi   a,cf_lcd_rs_em & 0x7 | 4      ;entry mode
   rcall write_lcd
   out   lcd_ddr,zero      ;write mode off

   #ifdef   splash
   ldiz  (init_msg<<1)
      do    load_msg
         lpm   a,z
         tst   a                 ;end of message?
         exiteq load_msg
         st    x+,a
         adiw  z,1
      loop  load_msg
   #endif

   ldi   a,(1<<lcd_rw)|i2c_lcd   ;read status (wait on busy)
   out   lcd_ctrl,a
                           ;write buffer (splash message) to LCD1
   ldi   lcd_adr,(1<<lcd_rs)|i2c_lcd
   mov   adr_ack,zero

   ;init i2c
   out   usisr,i2c_clr     ;clear flags & counter
   out   usicr,i2c_idle    ;start condition interrupt enabled
   sbi   i2c_ddr,i2c_scl   ;SCL out (clock stretching), LED out
   ldiz  illegal           ;state: idle
   sei

;
; ****************** Main Loop ************
;
; handles the following items:
;   writes buffer to LCD with busy and timeout
;   times backlight
;
   do    main
      cp    xl,yl             ;write buffer pending?
      ifne  m_wrt_pnd
         read_lcd a
         tst   a                 ;busy (bit 7)?
         ifpl  m_busy_end
            out   lcd_ctrl,lcd_adr  ;write command or data to lcd
            ld    a,y
            sbrc  lcd_adr,lcd_rs    ;is command?
            ifs   m_wrt_cmd
               mov   b,a
               andi  b,0xe0            ;is set function?
               cpi   b,0x20
               ifeq  m_wrt_sf
                  sbr   a,0x10            ;force 8 bit mode
               end   m_wrt_sf
            end   m_wrt_cmd
            out   lcd_out,a
            out   lcd_ddr,allon
            .if   cf_lcd_nmbr == 2
               sbrc  lcd_adr,lcd_sel     ;LCD 1 or 2?
               ifs   m_wrt_lcd
                  sbi   lcd_ctrl,lcd_e1   ;pulse enable 1 for 250 ns
                  cbi   lcd_ctrl,lcd_e1
               else  m_wrt_lcd
                  sbi   lcd_ctrl,lcd_e2   ;pulse enable 2 for 250 ns
                  cbi   lcd_ctrl,lcd_e2
               end   m_wrt_lcd
            .else
               sbi   lcd_ctrl,lcd_e1   ;pulse enable 1 for 250 ns
               cbi   lcd_ctrl,lcd_e1
            .endif
            out   lcd_ddr,zero      ;back to read busy
            ldi   a,(1<<lcd_rw)|i2c_lcd   ;default to read status
            out   lcd_ctrl,a
            inc   yl
            ldi   a,cf_lcd_on       ;turn backlight on
            out   ocr1a,a
            ldi   tick,122          ;load timer values
            ldi   a,cf_lcd_to1
            mov   timout1,a
            ldi   a,cf_lcd_to2
            mov   timout2,a
            out   tcnt1,zero        ;reset busy timeout
         else  m_busy_end
            in    a,tcnt1           ;check timeout
            cpi   a,58              ;>1.8 ms?
            ifsh  m_bsy_to
               ldi   a,2               ;flash backlight LED twice
               rjmp  err_flash
             end   m_bsy_to
         end   m_busy_end
      else  m_wrt_pnd
         tst   adr_ack           ;clr nak on address if lcd not busy
         ifne  m_adr_ack_test
            read_lcd a
            sbrs  a,7               ;is not busy?
               mov   adr_ack,zero      ;set ack
         end   m_adr_ack_test
      end   m_wrt_pnd

      in    a,tifr            ;8 ms timer tick?
      andi  a,(1<<tov1)
      ifne  m_tick
         out   tifr,a            ;clear tick
         dec   tick
         ifeq  m_1s
            ldi   tick,122          ;8 ms * 122
            tst   timout1           ;backlight is on?
            ifne  m_to1
               dec   timout1           ;dim backlight?
               ifeq  m_to1_dim
                  ldi   a,cf_lcd_dim
                  out   ocr1a,a
               end   m_to1_dim
            else  m_to1
               tst   timout2           ;backlight is dimmed?
               ifne  m_to2
                  dec   timout2           ;turn off backlight?
                  ifeq  m_to2_off
                     ldi   a,cf_lcd_off
                     out   ocr1a,a
                  end   m_to2_off
               end   m_to2
            end   m_to1
         end   m_1s
      end   m_tick

   loop  main

   #ifdef   splash
init_msg:   .db   splash,0,0
   #endif
