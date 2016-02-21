/**
 * Microcontroller - ATmega328P
 * Frequency - 8Mhz external quarts
 */


/********************************************************************************
Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../serial/SoftwareSerial.h"

/********************************************************************************
	Macros and Defines
********************************************************************************/


/********************************************************************************
	Function Prototypes
********************************************************************************/
void handle_usart_cmd(char *cmd, char *arg);

/********************************************************************************
	Global Variables
********************************************************************************/
SoftwareSerial mySerial(
		PD2, // RX Port
		PD4, // TX Port
		&DDRD, // Port Data Direction Register
		&PIND, // Port Input Data Register
		&PORTD, // Port Output Data Register
		INT0, // External Interrupt Request Flag
		&EIMSK // External Interrupt Mask Register
	);

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(INT0_vect)
{
	SoftwareSerial::handle_interrupt();
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {

    // enable interrupts
    sei();

    // Start Software Serial Port with given baud rate
    mySerial.begin(38400);

    // Direct the standard output to this Software Serial Port instance
    mySerial.registerAsStdOut();

    // Print Something to stdout
    printf("\nHello World!");

    // Register Console Command Listener
    mySerial.registerConsoleListener(&handle_usart_cmd);

	// main loop
    while (1) {
    	mySerial.consoleLoop();
    }
}

/********************************************************************************
	Functions
********************************************************************************/
void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\ntest command with [%s] argument", args);
	}

	else {
		printf("\nunknown [%s][%s]", cmd, args);
	}
}

