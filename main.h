// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _LoRaWanTest_H_
#define _LoRaWanTest_H_
#include "Arduino.h"
//add your includes for the project LoRaWanTest here

//end of add your includes here
#define debugSerial SerialUSB		// USB Serial
#define loraSerial SerialLoRa		// Hardware serial

#define LORA_DEBUG 		debugSerial
#define MICROVER		"MKRWAN_1.0V"

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _LoRaWanTest_H_ */
