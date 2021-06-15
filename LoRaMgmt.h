/*
 * LoRaMgmt.h
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#ifndef LORAMGMT_H_
#define LORAMGMT_H_

#include <stdint.h>
#include "main.h"

// Select frequency plan between TTN_FP_EU868 or TTN_FP_US915
#define MAXLORALEN	51			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise
#define LORACHNMAX	16
#define LORABUSY	-4			// error code for busy channel

#define CM_OTAA			1		// LORAWAN use OTAA join instead of ABP
#define CM_DTYCL		2		// LORAWAN enable duty cycle
#define CM_RJN	 		4		// LORAWAN rejoin if failed
#define CM_UCNF			8		// LORAWAN use unconfirmed messages
#define CM_NPBLK		16		// LORAWAN use not public network

/**
  * Lora Configuration
  */
typedef struct
{
	uint8_t	mode  = 2;			// test mode = 0 off, 1 LoRa, 2 LoRaWan, 3 LoRaWan + Remote, 4 LoRaWan Force Join
	uint8_t confMsk;			// configuation mask bits

	// Common all Modes
	uint8_t txPowerTst = 0;		// txPower setting for the low power test
	uint8_t dataLen = 1;		// data length to send over LoRa for a test
	uint8_t repeatSend = 5;		// number of send repeats
	union {
		uint16_t frequency;		// LoRa / FSK frequency in 10KHz steps
		uint16_t chnMsk;		// ChannelMask for LoRaWan EU868 (1-16)
	};
	union {
		uint8_t dataRate = 255;	// data rate starting value for LoRaWan, 255 = leave unchanged
		uint8_t spreadFactor;	// spread factor for Chirp signals FSK/LORA
	};

	// LoRaWan (OTAA vs ABP) or LoRa/FSK settings
	union {
		uint8_t bandWidth;		// Bandwidth setting for LoRa & FSK TODO: create steps
		char * devEui = NULL;   // Device EUI OTAA
		char * devAddr;         // Device Address ABP
	};
	union {
		uint8_t	codeRate;		// Code rate for LoRa & FSK, in 4/
		char * appEui = NULL;   // App EUI OTAA
		char * nwkSKey;         // Nw Session key ABP
	};
	union {
		char * appKey = NULL;   // App KEY OTAA
		char * appSKey;         // App Session key ABP
	};
//	uint32_t NetworkID;          /*< Network ID */

} sLoRaConfiguration_t;

//add your function definitions for the project LoRaWanTest here
// global types
typedef struct {
	uint32_t timeTx;
	uint32_t timeRx;		// one of the two may be removed
	uint32_t timeToRx;
	uint32_t txFrq;			// current used frequency
	uint16_t chnMsk;		// Concluding channel mask
	uint8_t lastCR;			// Coding rate 4/x
	uint8_t txDR;			// Tx data rate
	int8_t txPwr;			// Tx power index used
	int8_t rxRssi;			// last rx RSSI, default -128
	uint8_t rxSnr;			// last rx SNR, default -128
} sLoRaResutls_t;

int LoRaMgmtSetup(sLoRaConfiguration_t * conf);
int LoRaMgmtJoin();

int LoRaMgmtSend();
int LoRaMgmtPoll();
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();
int LoRaMgmtRemote();

int LoRaMgmtGetResults(sLoRaResutls_t * res);
char* LoRaMgmtGetEUI();

#endif /* LORAMGMT_H_ */
