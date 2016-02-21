/*
SoftwareSerial.h (formerly NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.

Exported from Arudino Version: 1.6.7
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <stdio.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#define _SS_MAX_RX_CMD_BUFF 100 // RX Command buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define HIGH 1
#define LOW 0

#define _in(bit,port)	port &= ~(1 << bit)
#define _out(bit,port)	port |= (1 << bit)
#define _on(bit,port)	port |= (1 << bit)
#define _off(bit,port)	port &= ~(1 << bit)
#define _check(bit, port) (1 << bit) & port

#define SOFT_SERIAL_PREFIX "\nSoftSerial>"

class SoftwareSerial
{
private:
  // per object data
  uint8_t _receivePin;
  uint8_t _transmitPinMask;
  uint8_t _extIntPinMask;

  volatile uint8_t * portDataDirectionRegister;
  volatile uint8_t * portInDataRegister;
  volatile uint8_t * portOutDataRegister;
  volatile uint8_t * portExtIntMaskRegister;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  volatile uint8_t cmd_buffer[_SS_MAX_RX_CMD_BUFF];
  volatile uint8_t cmd_buffer_count;
  void (*cmd_handler)(char *cmd, char *arg);

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  inline void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();

  void setTXPin(uint8_t transmitPin);
  void setRXPin(uint8_t receivePin);

  void parseCMD();
  inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  SoftwareSerial(uint8_t _receivePin,
		  uint8_t _transmitPin,
		  volatile uint8_t * portDataDirectionRegister,
		  volatile uint8_t * portInDataRegister,
		  volatile uint8_t * portOutDataRegister,
		  uint8_t _extIntPin,
		  volatile uint8_t * portExtIntMaskRegister,
		  bool inverse_logic = false);
  ~SoftwareSerial();
  void begin(long speed);
  void registerAsStdOut();
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();
  void consoleLoop();
  void registerConsoleListener(void (*fptr)(char *cmd, char *arg));

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }

  // public only for easy access by interrupt handlers
  static void handle_interrupt();
  static int putchar_printf(char var, FILE *stream);
};

#endif
