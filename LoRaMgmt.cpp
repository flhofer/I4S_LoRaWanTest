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

static sLoRaConfiguration_t * conf;

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

/*
 * setTxPwr: set power index on modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setTxPwr(uint8_t txPwr){
	if (conf->mode == 1){
		// Transform the powerIndex to power in dBm
		int npwr = 0;
		switch (txPwr) {
		case 1 : npwr = 14;
				break;
		case 2 : npwr = 11;
				break;
		case 3 : npwr = 8;
				break;
		case 4 : npwr = 5;
				break;
		case 5 : npwr = 2;
				break;
		case 0 : LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN); // MAX level
				return 0;
		}
		LoRa.setTxPower(npwr, PA_OUTPUT_RFO_PIN); // MAX RFO level
		return 0;
	}
	else if (conf->mode > 1)
		return modem.power((txPwr == 0)? PABOOST : RFO, txPwr) ? 0 : -1;
	return 0;
}

/*
 * getChannels:
 *
 * Arguments: - pointer to channel enable bit mask to fill, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
getChannels(uint16_t * chnMsk){ // TODO: for now only EU868

	*chnMsk = 0;
	int length = modem.getChannelMaskSize(freqPlan);
	const char * mask = modem.getChannelMask().c_str();

	for (int i=0; i < min(length * 4, LORACHNMAX / 4); i++)
	  *chnMsk |= (uint16_t)xtoInt(mask[i]) << (4*(3-i));

	return (0 == *chnMsk) * -1; // error if mask is empty!
}

/*
 * setChannels:
 *
 * Arguments: - pointer to channel enable bit mask to use, 0 off, 1 on
 * 			  - dataRate for test start, (disables ADR)
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setChannels(uint16_t chnMsk, uint8_t dataRate) {

	bool ret = true;
	uint16_t channelsMask[6] = {0};

	channelsMask[0] = chnMsk;

	modem.setMask(channelsMask);
	ret &= modem.sendMask();
	if (dataRate == 255){
		ret &= modem.dataRate(5);
		ret &= modem.setADR(true);
	}
	else {
		ret &= modem.setADR(false);
		ret &= modem.dataRate(dataRate);
	}

	return !ret * -1;
}

/*
 * setupLoRaWan: setup LoRaWan communication with modem
 *
 * Arguments: -
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
setupLoRaWan(sLoRaConfiguration_t * newConf){

	if (!modem.begin(freqPlan)) {
		debugSerial.println("Failed to start module");
		return -1;
	};

	int ret = 0;
	ret |= !modem.dutyCycle(newConf->chnMsk & CM_DTYCL); // switch off the duty cycle
	ret |= !modem.setADR(false);	// disable ADR by default

	debugSerial.print("Your module version is: ");
	debugSerial.println(modem.version());
	debugSerial.print("Your device EUI is: ");
	if (!newConf->devEui){
		newConf->devEui = strdup(modem.deviceEUI().c_str());
	}
	debugSerial.println(newConf->devEui);

	modem.publicNetwork(!(newConf->confMsk & CM_NPBLK));

	if (newConf->confMsk & CM_RJN)
		return ret *-1;

	debugSerial.println("-- PERSONALIZE");
	if (LoRaMgmtJoin()) {
		// Something went wrong; are you indoor? Move near a window and retry
		debugSerial.println("Network join failed");
		return -1;
	}

	// Set poll interval to 60 secs.
	modem.minPollInterval(60);

	if (!(newConf->confMsk & CM_OTAA)){
		// set to LorIoT standard RX, DR
		ret |= !modem.setRX2Freq(869525000);
		ret |= !modem.setRX2DR(0);
	}

	return ret *-1;
}

/*
 * setupDumb: setup LoRa communication with modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int
setupDumb(sLoRaConfiguration_t * newConf){

	modem.dumb();

	// Configure LoRa module to transmit and receive at 915MHz (915*10^6)
	// Replace 915E6 with the frequency you need (eg. 433E6 for 433MHz)
	if (!LoRa.begin((long)newConf->frequency * 100000)) {
		debugSerial.println("Starting LoRa failed!");
		return -1;
	}

	LoRa.setSpreadingFactor(newConf->spreadFactor);
	LoRa.setSignalBandwidth(newConf->bandWidth*1000);
	LoRa.setCodingRate4(newConf->codeRate);

	setTxPwr(newConf->txPowerTst);

	return 0;
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
	if (conf->mode == 1) {
		while (LoRa.beginPacket() == 0) {
		  delay(1);
		}
		LoRa.write(genbuf, conf->dataLen);
		return LoRa.endPacket(true); // true = async / non-blocking mode
	}
	else
	{
		modem.beginPacket();
		modem.write(genbuf, conf->dataLen);
		return modem.endPacket(conf);
	}
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
	ret |= getChannels(&res->chnMsk);
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
		return !modem.joinOTAA(conf->appEui, conf->appKey) * -1;
	else
		return !modem.joinABP(conf->devAddr, conf->nwkSKey, conf->appSKey) * -1;
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: - noJoin, i.e. do not join the network
 *
 * Return:	  returns 0 if successful, else -1
 */
int
LoRaMgmtSetup(sLoRaConfiguration_t * newConf){
	int ret = 0;
	switch (newConf->mode){
	case 0: ;
			break;
	case 1: ret = setupDumb(newConf);
			break;
	default:
	case 2:
	case 3:
	case 4:
			ret = setupLoRaWan(newConf);
			ret |= setChannels(newConf->chnMsk, newConf->dataRate);
	}
	ret |= setTxPwr(newConf->txPowerTst);

	// set boundaries for len value
	newConf->dataLen = max(min(newConf->dataLen, MAXLORALEN), 1);

	// initialize random seed with dataLen as value
	// keep consistency among tests, but differs with diff len
	rnd_contex = newConf->dataLen;
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);

	if (ret == 0)
		conf = newConf;
	return ret;
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
	if (!(conf->confMsk & CM_UCNF))
		return modem.restart() ? 0 : -1;
	return 0;
}


char* LoRaMgmtGetEUI(){ // TODO: This does not work
	if (!conf || !conf->devEui){
		setupLoRaWan(conf);
	}
	return conf->devEui;
}
