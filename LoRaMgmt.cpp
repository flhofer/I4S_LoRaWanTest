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
#define UNCF_POLL	5			// How many times to poll
#define MAXLORALEN	252			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise
#define LORACHNMAX	16
#define LORABUSY	-4			// error code for busy channel
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

static uint8_t actChan = 16;	// active channels
static int	pollcnt;			// un-conf poll retries

static unsigned rnd_contex;			// pseudo-random generator context (for reentrant)
static byte genbuf[MAXLORALEN];			// buffer for generated message

static uint32_t timeTx;				// Last TX
static uint32_t timeRx;				// Last RX
static uint32_t timeToRx;			// Last Total time
static uint32_t	txCount;			// Transmission counter
static uint32_t startSleepTS;		// relative MC time of Sleep begin
static uint32_t timerMillisTS;		// relative MC time for timers
static uint32_t startTestTS;		// relative MC time for test start
static uint32_t sleepMillis;		// Time to remain in sleep
static unsigned long rxWindow1 = 1000; // pause duration in ms between tx and rx TODO: get parameter
static unsigned long rxWindow2 = 2000; // pause duration in ms between tx and rx2 TODO: get parameter

static const sLoRaConfiguration_t * conf;
static enum {	iIdle,
				iSend,
				iPoll,
				iBusy,
				iSleep,
			} internalState;


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

/*
 * xtoInt: Transform character to number
 *
 * Arguments: - Read Chanracter
 *
 * Return:	  - Read hex number 0-16
 */
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

/*
 * printMessage: print a binary value as Hex characters
 *
 * Arguments: - byte buffer
 * 			  - length of byte buffer
 *
 * Return:	  -
 */
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
 * setActiveChannels: set number of active bits in channelmsk
 *
 * Arguments: - active channel mask
 *
 * Return:	  -
 */
static void
setActiveChannels(uint16_t chnMsk){
	actChan = 0; // get number of active channels in mask
	for (; (chnMsk); chnMsk >>=1)
		actChan += (uint8_t)(chnMsk & 0x1);
}

/*************** CALLBACK FUNCTIONS ********************/


/*
 * onMessage: Callback function for message received
 * Arguments: -
 *
 * Return:	  -
 */
static void
onMessage(size_t length, bool binary){
	char rcv[length+1];
	unsigned int i = 0;
	while (modem.available() && i < length) {
		rcv[i++] = (char)modem.read();
	}
	rcv[length] = '\0';
	if (binary)
		printMessage(rcv, i);
	else
		debugSerial.println(rcv);
}

/*
 * onBeforeTx: Callback function for before LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onBeforeTx(){
	timerMillisTS = millis();
	timeTx = 0;
	timeRx = 0;
	timeToRx = 0;
}

/*
 * onAfterTx: Callback function for after LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onAfterTx(){
	timeTx = millis() - timerMillisTS;
}

/*
 * onAfterTx: Callback function for after LoRa RX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onAfterRx(){
	timeToRx = millis() - timerMillisTS;
	timeRx = timeToRx - timeTx - rxWindow1;
	if (timeRx > 1000)
		timeRx -= rxWindow2;
}

/*
 * setTxPwr: set power index on modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setTxPwr(uint8_t mode, uint8_t txPwr){
	if (mode == 1){
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
	else if (mode > 1)
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
 * loRaJoin: Join a LoRaWan network
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
loRaJoin(const sLoRaConfiguration_t * newConf){
	if (newConf->confMsk & CM_OTAA)
		return !modem.joinOTAA(newConf->appEui, newConf->appKey) * -1;
	else
		return !modem.joinABP(newConf->devAddr, newConf->nwkSKey, newConf->appSKey) * -1;
}

/*
 * setupLoRaWan: setup LoRaWan communication with modem
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  returns 0 if successful, else -1
 */
static int
setupLoRaWan(const sLoRaConfiguration_t * newConf){

	if (!modem.begin(freqPlan)) {
		debugSerial.println("Failed to start module");
		return -1;
	};

	int ret = 0;
	ret |= !modem.dutyCycle(newConf->confMsk & CM_DTYCL); // switch off the duty cycle
	ret |= !modem.setADR(false);	// disable ADR by default

	modem.publicNetwork(!(newConf->confMsk & CM_NPBLK));

	if (!(newConf->confMsk & CM_RJN) && loRaJoin(newConf))
	{
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

	modem.onMessage(&onMessage);
	modem.onBeforeTx(&onBeforeTx);
	modem.onAfterTx(&onAfterTx);
	modem.onAfterRx(&onAfterRx);

	return ret *-1;
}

/*
 * setupDumb: setup LoRa communication with modem
 *
 * Arguments: - pointer to test configuration to use
 *
 * Return:	  - return 0 if OK, -1 if error
 */
static int
setupDumb(const sLoRaConfiguration_t * newConf){

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

	return 0;
}

/*************** TEST SEND FUNCTIONS ********************/

/*
 * LoRaMgmtSendDumb: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, < 0 = error, 0 = busy, 1 = done, 2 = stop
 * 			--- ALWAYS RETURNS 0
 */
int
LoRaMgmtSendDumb(){
	if (internalState != iSleep){	// Does never wait!
		txCount++;
		while (LoRa.beginPacket() == 0) {
		  delay(1);
		}
		LoRa.write(genbuf, conf->dataLen);
		//return
		LoRa.endPacket(true); // true = async / non-blocking mode
	}
	return 0;
}

/*
 * LoRaMgmtSend: send a message with the defined mode
 *
 * Arguments: -
 *
 * Return:	  status of sending, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtSend(){
	if (internalState == iIdle){
		internalState = iSend;

		modem.beginPacket();
		modem.write(genbuf, conf->dataLen);
		int ret = modem.endPacket(!(conf->confMsk & CM_UCNF));
		if (ret < 0){
			if (LORABUSY == ret ) // no chn -> pause for free-delay / active channels
				internalState = iBusy;
			return 0;
		}

		pollcnt = 0;
		txCount++;

		// sent but no response from confirmed, or not confirmed msg, continue to next step
		if (ret == 1 || !(conf->confMsk & CM_UCNF))
			return 1;
		return 2;
	}
	return 0;	// else busy
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX, check Receive
 *
 * Arguments: -
 *
 * Return:	  status of polling, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtPoll(){
	if (internalState == iIdle){

		if (!(conf->confMsk & CM_UCNF)){
			internalState = iSend;
			return modem.getMsgConfirmed();
		}
		else{
			internalState = iPoll;
			int ret = modem.poll();
			if (ret <= 0){
				if (pollcnt < UNCF_POLL){
					if (LORABUSY == ret ) // no chn -> pause for free-delay / active channels
						internalState = iBusy;
					return 0;	// return 0 until count
				}
				return -1;
			}
			pollcnt++;
			txCount++;
			return 0;
		}
	}
	return 0;
}

/*
 * LoRaMgmtRemote: poll modem for go/stop commands
 *
 * Arguments: -
 *
 * Return:	  status of polling, < 0 = error, 0 = busy, 1 = done, 2 = stop
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
 * Return:	  - 0 if OK, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtGetResults(sLoRaResutls_t * const res){
	int ret = 0;
	res->testTime = millis() - startTestTS;
	res->txCount = txCount;
	res->timeTx = timeTx;
	res->timeRx = timeRx;
	res->timeToRx = timeToRx;
	if (conf->mode == 1){
		res->txFrq = conf->frequency*100000;
		res->lastCR = conf->codeRate;
		res->txDR = conf->spreadFactor;
		res->txPwr = conf->txPowerTst;
	}
	else{
		//	res->txFrq = modem.getFrequency();
		ret |= getChannels(&res->chnMsk);
		//	res->lastCR = modem.getCR();
		res->txDR = modem.getDataRate();
		res->txPwr = modem.getPower();
		res->rxRssi = modem.getRSSI();
		res->rxSnr = modem.getSNR();
	}
	return (ret == 0) ? 1 : -1;
}

/*
 * LoRaMgmtJoin: Join a LoRaWan network
 *
 * Arguments: -
 *
 * Return:	  returns < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtJoin(){
	return !(loRaJoin(conf));
}

/*
 * LoRaMgmtSetup: setup LoRaWan communication with modem
 *
 * Arguments: - noJoin, i.e. do not join the network
 *
 * Return:	  returns 0 if successful, else -1
 */
int
LoRaMgmtSetup(const sLoRaConfiguration_t * newConf){
	int ret = 0;
	switch (newConf->mode){
	default:
	case 0: ;
			break;
	case 1: ret = setupDumb(newConf);
			break;
	case 2 ... 4:
			ret = setupLoRaWan(newConf);
			ret |= setChannels(newConf->chnMsk, newConf->dataRate);
			setActiveChannels(newConf->chnMsk);
	}
	ret |= setTxPwr(newConf->mode, newConf->txPowerTst);

	// initialize random seed with dataLen as value
	// keep consistency among tests, but differs with diff len
	rnd_contex = newConf->dataLen;
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);

	pollcnt = 0;
	txCount = 0;

	if (ret == 0)
		conf = newConf;

	startTestTS = millis();
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

	if (internalState == iIdle){
		// Prepare PayLoad of x bytes
		(void)generatePayload(genbuf);

		return 1;
	}

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


const char*
LoRaMgmtGetEUI(){
	if (!conf || !conf->devEui){
		if (!modem.begin(freqPlan)) {
			debugSerial.println("Failed to start module");
			return NULL;
		};
		return modem.deviceEUI().c_str();
	}
	return conf->devEui;
}

/*
 * LoRaMgmtMain: state machine for the LoRa Control
 *
 * Arguments: -
 *
 * Return:	  -
 */
void
LoRaMgmtMain (){
	switch (internalState){

	case iIdle:
		break;
	case iSend:
		startSleepTS = millis();
		sleepMillis = 100;	// simple retry timer 100ms, e.g. busy
		internalState = iSleep;
		break;
	case iPoll:
		startSleepTS = millis();
		sleepMillis = 1000;	// simple retry timer 1000ms, e.g. busy
		internalState = iSleep;
		break;
	case iBusy:
		startSleepTS = millis();
		sleepMillis = RESFREEDEL/actChan;
		internalState = iSleep;
		break;
	case iSleep:
		if (millis() - startSleepTS > sleepMillis)
			internalState = iIdle;
	}
}
