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
#define LORA_DEVADDR	"01234567"
#define LORA_NWSKEY		"01234567890abcdef01234567890abcd"
#define LORA_APSKEY		"01234567890abcdef01234567890abcd"
#define LORA_APPEUI		"01234567890abcde"
#define LORA_APPKEY 	"01234567890abcdef01234567890abcd"

#define MICROVER		"MKRWAN_1.0V"

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _LoRaWanTest_H_ */
