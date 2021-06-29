/*
 * main.h
 *
 *  Created on: May 25, 2020
 *      Author: Florian Hofer
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "Arduino.h"

// Serial connection definition
#define debugSerial SerialUSB		// USB Serial
#define loraSerial SerialLoRa		// Hardware serial

#define LORA_DEBUG 		debugSerial
#define MICROVER		"MKRWAN_1.0V"

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _LoRaWanTest_H_ */
