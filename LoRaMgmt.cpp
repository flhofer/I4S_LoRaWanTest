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
#include <stdlib.h>				// ARM standard library

LoRaModem modem(loraSerial); // @suppress("Abstract class cannot be instantiated")

#define freqPlan EU868

static unsigned rnd_contex;			// pseudo-random generator context (for reentrant)
static byte genbuf[MAXLORALEN];			// buffer for generated message

static loraConfiguration_t * conf;

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

	for (int i=0; i < conf->dataLen; i++, payload++)
		*payload=(byte)(rand_r(&rnd_contex) % 255);

	return payload;
}


static int
xtoInt(char nChar) {
	switch (nChar)
	{
		case '0' ... '9': // Digits
			return (nChar - '0');
		case 'a' ... 'f': // small letters a-f
			return (nChar - 87); // 87 = a - 10
		case 'A' ... 'F': // capital letters A-F
			return (nChar - 55); // 55 = A - 10
		default:	// Not a number or HEX char/terminator
			return 0;
	}
}

static void
printMessage(char* rcv, uint8_t len){
	debugSerial.print("Received: ");
	for (unsigned int j = 0; j < len; j++) {
		debugSerial.print(rcv[j] >> 4, HEX);
		debugSerial.print(rcv[j] & 0xF, HEX);
		debugSerial.print(" ");
	}
	debugSerial.println();
}

/*************** TEST SEND FUNCTIONS ********************/


/*
 * LoRaMgmtSend: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, >0 ok (no of bytes), <0 error
 */
int
LoRaMgmtSend(){
	modem.beginPacket();
	modem.write(genbuf, conf->dataLen);
	return modem.endPacket(conf);
}

/*
 * LoRaMgmtSendDumb: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, >0 ok (no of bytes), <0 error
 */
int
LoRaMgmtSendDumb(){
	while (LoRa.beginPacket() == 0) {
	  delay(1);
	}
	LoRa.write(genbuf, conf->dataLen);
	return LoRa.endPacket(true); // true = async / non-blocking mode
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX, check Receive
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 no message, -1 error, >0 No of bytes
 */
int
LoRaMgmtPoll(){
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
	printMessage(rcv, i);

	return i;
}

/*
 * LoRaMgmtRemote: poll modem for go/stop commands
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 no message, -1 error, >0 Msg code
 * 				1 = Start
 * 				2 = Stop
 */
int
LoRaMgmtRemote(){
	delay(1000);
	if (!modem.available()) {
		// No down-link message received at this time.
		return 0;
	}
	char rcv[MAXLORALEN];
	unsigned int i = 0;
	while (modem.available() && i < MAXLORALEN) {
		rcv[i++] = (char)modem.read();
	}

	if (i == 1){ // one letter
		switch(rcv[0]){
		case 'R':
			return 1;

		case 'S':
			return 2;
		}
	}

	debugSerial.print("Invalid message, ");
	printMessage(rcv, i);
	return -1;
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
LoRaGetChannels(uint16_t * chnMsk){ // TODO: for now only EU868

	*chnMsk = 0;
	int length = modem.getChannelMaskSize(freqPlan);
	const char * mask = modem.getChannelMask().c_str();

	for (int i=0; i < min(length * 4, LORACHNMAX / 4); i++)
	  *chnMsk |= (uint16_t)xtoInt(mask[i]) << (4*(3-i));

	return (0 == *chnMsk) * -1; // error if mask is empty!
}

/*
 * LoRaMgmtGetResults: getter for last experiment results
 *
 * Arguments: - pointer to Structure for the result data
 *
 * Return:	  - 0 if OK, <0 error
 */
int
LoRaMgmtGetResults(sLoRaResutls_t * res){
	int ret = 0;
//	res->timeTx = timeTx;
//	res->timeRx = timeRx;
//	res->timeToRx = timeToRx;
//	res->txFrq = ttn.getFrequency();
	ret |= LoRaGetChannels(&res->chnMsk);
//	res->lastCR = ttn.getCR();
	res->txDR = modem.getDataRate();
	res->txPwr = modem.getPower();
	res->rxRssi = modem.getRSSI();
	res->rxSnr = modem.getSNR();
	return ret * -1;
}

/*
 * LoRaMgmtJoin: Join a LoRaWan network
 *
 * Arguments: -
 *
 * Return:	  returns 0 if successful, else -1
 */
int
LoRaMgmtJoin(){
	if (conf->confMsk & CM_OTAA)
		return !modem.joinOTAA(appEui, appKey) * -1;
	else
		return !modem.joinABP(devAddr, nwkSKey, appSKey) * -1;
}

/*
 * setupLoRaWan: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
setupLoRaWan(){

	if (!modem.begin(freqPlan)) {
		debugSerial.println("Failed to start module");
		return -1;
	};

	int ret = 0;
	ret |= !modem.dutyCycle(false); // switch off the duty cycle
	ret |= !modem.setADR(false);	// disable ADR

	debugSerial.print("Your module version is: ");
	debugSerial.println(modem.version());
	debugSerial.print("Your device EUI is: ");
	if (!conf->DevEui){
		conf->DevEui = strdup(modem.deviceEUI().c_str());
	}
	debugSerial.println(conf->DevEui);

	modem.publicNetwork(!(conf->confMsk & CM_NPBLK));

	if (conf->confMsk & CM_RJN)
		return ret *-1;

	debugSerial.println("-- PERSONALIZE");
	if (LoRaMgmtJoin()) {
		// Something went wrong; are you indoor? Move near a window and retry
		debugSerial.println("Network join failed");
		return -1;
	}

	// Set poll interval to 60 secs.
	modem.minPollInterval(60);

	if (!(conf->confMsk & CM_OTAA)){
		// set to LorIoT standard RX, DR
		ret |= !modem.setRX2Freq(869525000);
		ret |= !modem.setRX2DR(0);
	}

	return ret *-1;
}

/*
 * setChannels:
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setChannels() {

	bool ret = true;
	uint16_t channelsMask[6] = {0};

	channelsMask[0] = chnMsk;

	modem.setMask(channelsMask);
	ret &= modem.sendMask();
	if (dr == 255){
		ret &= modem.dataRate(5);
		ret &= modem.setADR(true);
	}
	else {
		ret &= modem.setADR(false);
		ret &= modem.dataRate((uint8_t)dr);
	}

	return !ret * -1;
}

/*
 * setupDumb: setup LoRa communication with modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int
setupDumb(){

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
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: - noJoin, i.e. do not join the network
 *
 * Return:	  returns 0 if successful, else -1
 */
int
LoRaMgmtSetup(loraConfiguration_t * conf){
	setupLoRaWan();
	setChannels();

	//	// set boundaries for len value
	//	dataLen = max(min(datalen, MAXLORALEN), 1);
	//
	//	// initialize random seed with datalen as value
	//	// keep consistency among tests, but differs with diff len
	//	rnd_contex = dataLen;
	//	// Prepare PayLoad of x bytes
	//	(void)generatePayload(genbuf);
}





/*
 * LoRaMgmtUpdt: update LoRa message buffer
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int
LoRaMgmtUpdt(){
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
int
LoRaMgmtRcnf(){
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
int
LoRaMgmtTxPwr(uint8_t txPwr){
	return modem.power(RFO, txPwr) ? 0 : -1;
}

char* LoRaMgmtGetEUI(){
	if (!conf->DevEui){
		LoRaMgmtSetup(NULL);
	}
	return conf->DevEui;
}
