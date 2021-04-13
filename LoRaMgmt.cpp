/*
 * LoRaMgmt.cpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#include "LoRaMgmt.h"

#include "main.h"				// Global includes/definitions, i.e. address, key, debug mode
#include "MKRWAN.h"
#include <LoRa.h>
//#include <stdlib.h>				// ARM standard library

LoRaModem modem(loraSerial);

// DevAddr, NwkSKey, AppSKey and the frequency plan
static const char *devAddr = LORA_DEVADDR;
static const char *nwkSKey = LORA_NWSKEY;
static const char *appSKey = LORA_APSKEY;

static bool conf = false;			// use confirmed messages
static int dataLen = 1; 			// TX data length for tests
static unsigned rnd_contex;			// pseudo-random generator context (for reentrant)

static byte genbuf[MAXLORALEN];			// buffer for generated message

/********************** HELPERS ************************/

/*
 * generatePayload: fills a buffer with dataLen random bytes
 *
 * Arguments: - Byte vector for payload
 *
 * Return:	  - next open position (end of buffer)
 */
static byte *
generatePayload(byte *payload){
	// TODO: unprotected memory
	for (int i=0; i < dataLen; i++, payload++)
		*payload=(byte)(rand_r(&rnd_contex) % 255);

	return payload;
}

/*************** TEST SEND FUNCTIONS ********************/


/*
 * LoRaMgmtSendConf: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, >0 ok (no of bytes), <0 error
 */
int LoRaMgmtSend(){
	modem.beginPacket();
	modem.write(genbuf, dataLen);
	return modem.endPacket(conf);
}


// TODO: check for POLL = send ok, ack signal? How does that work?
/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX, check Receive
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 no message, -1 error, >0 No of bytes
 */
int LoRaMgmtPoll(){
	delay(1000);
	if (!modem.available()) {
		// No downlink message received at this time.
		return 0;
	}
	char rcv[MAXLORALEN];
	unsigned int i = 0;
	while (modem.available() && i < MAXLORALEN) {
		rcv[i++] = (char)modem.read();
	}
	debugSerial.print("Received: ");
	for (unsigned int j = 0; j < i; j++) {
		debugSerial.print(rcv[j] >> 4, HEX);
		debugSerial.print(rcv[j] & 0xF, HEX);
		debugSerial.print(" ");
	}
	debugSerial.println();

	return i;
}

/*************** MANAGEMENT FUNCTIONS ********************/

/*
 * LoRaGetChannels:
 *
 * Arguments: - pointer to channel enable bit mask to fill, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
LoRaGetChannels(uint16_t * chnMsk){

	*chnMsk = 0;

	for (int i=0; i<LORACHNMAX; i++)
	  *chnMsk |= (uint16_t)modem.isChannelEnabled((uint8_t)i) << i;

	return (0 == *chnMsk) * -1; // error if mask is empty!
}

/*
 * LoRaMgmtGetResults: getter for last experiment results
 *
 * Arguments: - pointer to Structure for the result data
 *
 * Return:	  - 0 if ok, <0 error
 */
int
LoRaMgmtGetResults(sLoRaResutls_t * res){

}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  -
 */
void LoRaMgmtSetup(){

	if (!modem.begin(EU868)) {
		debugSerial.println("Failed to start module");
		while (1) {}
	};

	modem.dutyCycle(false); // switch off

	debugSerial.print("Your module version is: ");
	debugSerial.println(modem.version());
	debugSerial.print("Your device EUI is: ");
	debugSerial.println(modem.deviceEUI());

	debugSerial.println("-- PERSONALIZE");
	int connected = modem.joinABP(devAddr, nwkSKey, appSKey);
	if (!connected) {
		// Something went wrong; are you indoor? Move near a window and retry
		while (1) {}
	}

	// Set poll interval to 60 secs.
	modem.minPollInterval(60);
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  -
 */
void LoRaMgmtSetupDumb(long FRQ){

	modem.dumb();

	// Configure LoRa module to transmit and receive at 915MHz (915*10^6)
	// Replace 915E6 with the frequency you need (eg. 433E6 for 433MHz)
	if (!LoRa.begin(FRQ)) {
	Serial.println("Starting LoRa failed!");
	while (1);
	}
}

/*
 * LoRaSetGblParam: set generic parameters, re-init random seed
 *
 * Arguments: - confirmed send yes/no
 * 			  - simulated payload length
 *
 * Return:	  -
 */
void LoRaSetGblParam(bool confirm, int datalen){
	conf = confirm;
	// set boundaries for len value
	dataLen = max(min(datalen, MAXLORALEN), 1);

	// initialize random seed with datalen as value
	// keep consistency among tests, but differs with diff len
	rnd_contex = dataLen;
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);
}

/*
 * LoRaSetChannels:
 *
 * Arguments: - channel enable bit mask, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaSetChannels(uint16_t chnMsk, uint8_t drMin, uint8_t drMax){

	bool retVal = true;

	for (int i=0; i<LORACHNMAX; i++, chnMsk >>=1)
		if ((bool)chnMsk & 0x01) {
			retVal &= modem.enableChannel((uint8_t)i);
			retVal &= modem.dataRate(drMin);
		}
		else
			retVal &= modem.disableChannel((uint8_t)i);

	return !retVal * -1;
}

/*
 * LoRaMgmtUpdt: update LoRa message buffer
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtUpdt(){
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);

	return 0;
}

/*
 * LoRaMgmtRcnf: reset modem and reconf
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtRcnf(){
	if (conf)
		return modem.restart() ? 0 : -1;\
	return 0;
}

/*
 * LoRaMgmtTxPwr: set power index on modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtTxPwr(uint8_t txPwr){
	return modem.power(RFO, txPwr) ? 0 : -1;
}


