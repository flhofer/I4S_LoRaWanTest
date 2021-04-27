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

//add your function definitions for the project LoRaWanTest here
// global types
typedef struct sLoRaResutls {
	uint32_t timeTx;
	uint32_t timeRx;		// TODO: one of the two may be removed
	uint32_t timeToRx;
	uint32_t txFrq;			// current used frequency
	uint16_t chnMsk;		// Concluding channel mask
	uint8_t lastCR;			// Coding rate 4/x
	uint8_t txDR;			// Tx data rate
	int8_t txPwr;			// Tx power index used TODO: check if we store index or power dBm
	int8_t rxRssi;			// last rx RSSI, default -128
	uint8_t rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;

//global variable declarations
extern int debug;				// Global flag Debug console attached (PC)

//Do not add code below this line
#endif /* _LoRaWanTest_H_ */
