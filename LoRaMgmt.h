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

// TODO: Remove
extern char *appEui;
extern char *appKey;
extern char *devAddr;
extern char *nwkSKey;
extern char *appSKey;

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
	uint8_t repeatSend = 5;					// number of send repeats
	union {
		uint16_t frequency;					// LoRa / FSK frequency in 10KHz steps
		uint16_t chnMsk;					// ChannelMask for LoRaWan EU868 (1-16)
	};
	union {
		uint8_t dataRate = 255;				// data rate starting value for LoRaWan, 255 = leave unchanged
		uint8_t bandWidth;					// Bandwidth setting for LoRa & FSK TODO: create steps
	};

	// LoRaWan Only, OTAA vs ABP
	union {
		char * DevEui = NULL;    /*< Device EUI */
		char * DevAddr;          /*< Device Address */ // WAS uint32_t
	};
	union {
		char * AppEui = NULL;   /*< Application EUI */
		char * NwkSKey;         /*< Network Session Key */
	};
	union {
		char * AppKey = NULL;   /*< Application Key */
		char * AppSKey;         /*< Application Session Key */
	};
//	uint32_t NetworkID;          /*< Network ID */

} loraConfiguration_t;

int LoRaMgmtSetup(loraConfiguration_t * conf);
int LoRaMgmtJoin();

int LoRaMgmtSend();
int LoRaMgmtPoll();
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();
int LoRaMgmtTxPwr(uint8_t txPwr);
int LoRaMgmtRemote();

int LoRaMgmtGetResults(sLoRaResutls_t * res);
char* LoRaMgmtGetEUI();

#endif /* LORAMGMT_H_ */
