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

#define freqPlan EU868

// DevAddr, NwkSKey, AppSKey and the frequency plan
static const char *devAddr = LORA_DEVADDR;
static const char *nwkSKey = LORA_NWSKEY;
static const char *appSKey = LORA_APSKEY;

static bool conf = false;			// use confirmed messages
static bool otaa = false;			// use otaa join
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

	for (int i=0; i < dataLen; i++, payload++)
		*payload=(byte)(rand_r(&rnd_contex) % 255);

	return payload;
}

/*************** TEST SEND FUNCTIONS ********************/


/*
 * LoRaMgmtSend: send a message with the defined mode
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

/*
 * LoRaMgmtSendDumb: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, >0 ok (no of bytes), <0 error
 */
int LoRaMgmtSendDumb(){
	while (LoRa.beginPacket() == 0) {
	  delay(1);
	}
	LoRa.write(genbuf, dataLen);
	return LoRa.endPacket(true); // true = async / non-blocking mode
}

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
	// TODO: use getchannelmask (string) instead of bit by bit
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
//	res->timeTx = timeTx;
//	res->timeRx = timeRx;
//	res->timeToRx = timeToRx;
//	res->txFrq = ttn.getFrequency();
	(void)LoRaGetChannels(&res->chnMsk);
//	res->lastCR = ttn.getCR();
	res->txDR = modem.getDataRate();
	res->txPwr = modem.getPower();
	res->rxRssi = modem.getRSSI();
	res->rxSnr = modem.getSNR();
	return 0;
}

#define SECRET_APP_EUI "BE010000000000DF"
#define SECRET_APP_KEY "9ADE44A4AEF1CD77AEB44387BD976928"

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  returns 0 if successful, else -1
 */
int LoRaMgmtSetup(){

	if (!modem.begin(freqPlan)) {
		debugSerial.println("Failed to start module");
		return -1;
	};

	String appEui = SECRET_APP_EUI;
	String appKey = SECRET_APP_KEY;

	int ret = 0;
	ret |= !modem.dutyCycle(false); // switch off the duty cycle
	ret |= !modem.setADR(true);		// enable ADR

	debugSerial.print("Your module version is: ");
	debugSerial.println(modem.version());
	debugSerial.print("Your device EUI is: ");
	debugSerial.println(modem.deviceEUI());

	debugSerial.println("-- PERSONALIZE");
	int connected;
	if (otaa)
		connected = modem.joinOTAA(appEui, appKey);
	else
		connected = modem.joinABP(devAddr, nwkSKey, appSKey);
	if (!connected) {
		// Something went wrong; are you indoor? Move near a window and retry
		debugSerial.println("Network join failed");
		return -1;
	}

	// Set poll interval to 60 secs.
	modem.minPollInterval(60);

	// set to LorIoT standard RX, DR
	ret |= !modem.setRX2Freq(869525000);
	ret |= !modem.setRX2DR(0);

	return ret *-1;
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtSetupDumb(long FRQ){

	modem.dumb();

	// Configure LoRa module to transmit and receive at 915MHz (915*10^6)
	// Replace 915E6 with the frequency you need (eg. 433E6 for 433MHz)
	if (!LoRa.begin(FRQ)) {
		debugSerial.println("Starting LoRa failed!");
		return 1;
	}

	// TODO: defaults for now
	LoRa.setSpreadingFactor(12);
	LoRa.setSignalBandwidth(125000);
	LoRa.setCodingRate4(8);

	LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN); // MAX RFO level
	return 0;
}

/*
 * LoRaSetGblParam: set generic parameters, re-init random seed
 *
 * Arguments: - confirmed send yes/no
 * 			  - simulated payload length
 * 			  - Use OTAA?
 *
 * Return:	  -
 */
void LoRaSetGblParam(bool confirm, int datalen, int OTAA){
	conf = confirm;
	otaa = (OTAA);
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
 * 			  - data rate min (ignored)
 * 			  - data rate max
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaSetChannels(uint16_t chnMsk, uint8_t drMin, uint8_t drMax) {

	bool retVal = true;

	for (int i=0; i<LORACHNMAX; i++, chnMsk >>=1)
		if ((bool)chnMsk & 0x01) {
			retVal &= modem.enableChannel((uint8_t)i);
			retVal &= modem.dataRate(drMax);
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
		return modem.restart() ? 0 : -1;
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


