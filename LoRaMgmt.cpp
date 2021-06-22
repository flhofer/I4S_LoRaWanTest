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
#define POLL_NO		5			// How many times to poll
#define MAXLORALEN	242			// maximum payload length 0-51 for DR0-2, 115 for DR3, 242 otherwise
#define LORACHNMAX	16
#define LORABUSY	-4			// error code for busy channel
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

static uint8_t actBands = 2;	// active channels
static int	pollcnt;			// un-conf poll retries
static int32_t	fcu;			// frame counter

static unsigned rnd_contex;			// pseudo-random generator context (for reentrant)
static byte genbuf[MAXLORALEN];			// buffer for generated message

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
				iRetry,
				iBusy,
				iSleep,
			} internalState;


static sLoRaResutls_t **resP;

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
 * setActiveBands: set number of active bands to determine pause time
 *
 * Arguments: - active channel mask
 *
 * Return:	  -
 */
static void
setActiveBands(uint16_t chnMsk){
	actBands = 0;
	if (chnMsk & 0x07) // base band [8680 - 8686)
		actBands++;
	if (chnMsk & 0xf8) // band 0 [8650 - 8670)
		actBands++;
	// Others are excluded

	if (!actBands)	// avoid DIV 0
		actBands++;
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
	(*resP)->timeTx = 0;
	(*resP)->timeRx = 0;
	(*resP)->timeToRx = 0;
}

/*
 * onAfterTx: Callback function for after LoRa TX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onAfterTx(){
	(*resP)->timeTx = timerMillisTS = millis();
}

/*
 * onAfterTx: Callback function for after LoRa RX
 * Arguments: -
 *
 * Return:	  -
 */
static void
onAfterRx(){
	(*resP)->timeToRx = millis() - timerMillisTS;
	(*resP)->timeRx = (*resP)->timeToRx - (*resP)->timeTx - rxWindow1;
	if ((*resP)->timeRx > 1000)
		(*resP)->timeRx -= rxWindow2;
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

	// Set poll interval to 1 sec.
	modem.minPollInterval(1); // for testing only

	if (!(newConf->confMsk & CM_OTAA)){
		// set to LorIoT standard RX, DR
		ret |= !modem.setRX2Freq(869525000);
		ret |= !modem.setRX2DR(0);
	}

//	modem.onMessage(&onMessage);
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
		(*resP)->txCount++;
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

		fcu = modem.getFCU();
		modem.beginPacket();
		modem.write(genbuf, conf->dataLen);
		int ret = modem.endPacket(!(conf->confMsk & CM_UCNF));
		if (ret < 0){
			if (LORABUSY == ret ){ // no channel available -> pause for free-delay / active channels
				internalState = iBusy;
				return 0;
			}
			return ret;
		}

		internalState = iPoll;
		pollcnt = 0;
		(*resP)->txCount++;

		return 1;
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
		internalState = iPoll;

		int32_t nfcu = modem.getFCU();

		// Confirmed packages trigger a retry after a polling retry delay.
		if (!(conf->confMsk & CM_UCNF)){
			if (nfcu != fcu){
				internalState = iRetry;
				return 0;
			}
			return modem.getMsgConfirmed() ? 1 : -1 ;
		}
		else{
			// Not yet sent?
			if (nfcu != fcu){
				return 0;
			}
			int ret = modem.poll();
			if (ret <= 0){
				if (pollcnt < POLL_NO){
					if (LORABUSY == ret ) // no channel available -> pause for duty cycle-delay / active channels)
						internalState = iBusy;
					return 0;	// return 0 until count
				}
				return ret;
			}
			pollcnt++;
			(*resP)->txCount++;
			// print received telegram
			if (modem.available()){
				char rcv[MAXLORALEN];
				int len = modem.readBytesUntil('\r', rcv, MAXLORALEN);
				printMessage(rcv, len);
			}
			return 1;
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
LoRaMgmtRemote(){ // TODO: fix remote wait
	if (internalState == iIdle){
		internalState = iPoll;

		int ret = modem.poll();
		if (ret < 0 && ret != LORABUSY)
			return ret;

		if (!modem.available()) {
			// No down-link message received at this time.
			return 0;
		}

		char rcv[MAXLORALEN];
		int len = modem.readBytesUntil('\r', rcv, MAXLORALEN);

		if (len == 1){ // one letter
			switch(rcv[0]){
			case 'R':
				return 1;

			case 'S':
				return 2;
			}
		}

		debugSerial.print("Invalid message, ");
		printMessage(rcv, len);
	}
	return 0;
}

/*************** MANAGEMENT FUNCTIONS ********************/

/*
 * LoRaMgmtGetResults: getter for last experiment results
 *
 * Arguments: -
 *
 * Return:	  - 0 if OK, < 0 = error, 0 = busy, 1 = done, 2 = stop
 */
int
LoRaMgmtGetResults(){
	if (!resP)
		return -1;
	int ret = 0;
	(*resP)->testTime = millis() - startTestTS;
	if (conf->mode == 1){
		(*resP)->txFrq = conf->frequency*100000;
		(*resP)->lastCR = conf->codeRate;
		(*resP)->txDR = conf->spreadFactor;
		(*resP)->txPwr = conf->txPowerTst;
	}
	else{
		//	res->txFrq = modem.getFrequency();
		ret |= getChannels(&(*resP)->chnMsk);
		//	res->lastCR = modem.getCR();
		(*resP)->txDR = modem.getDataRate();
		(*resP)->txPwr = modem.getPower();
		(*resP)->rxRssi = modem.getRSSI();
		(*resP)->rxSnr = modem.getSNR();
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
LoRaMgmtSetup(const sLoRaConfiguration_t * newConf, sLoRaResutls_t ** const res){
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
			setActiveBands(newConf->chnMsk);
	}
	ret |= setTxPwr(newConf->mode, newConf->txPowerTst);

	// initialize random seed with dataLen as value
	// keep consistency among tests, but differs with diff len
	rnd_contex = newConf->dataLen;
	// Prepare PayLoad of x bytes
	(void)generatePayload(genbuf);

	resP = res;

	pollcnt = 0;
	(*resP)->txCount = 0;

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
		sleepMillis = 100;	// Sleep timer after send, minimum wait
		internalState = iSleep;
		break;
	case iPoll:
		startSleepTS = millis();
		sleepMillis = 1000;	// simple retry timer 1000ms
		internalState = iSleep;
		break;
	case iRetry:
		startSleepTS = millis();
		sleepMillis =  rxWindow1 + rxWindow2 + 1000; // e.g. ACK lost, = 2+-1s (random)
		internalState = iSleep;
		break;
	case iBusy:	// Duty cycle = 1% chn [1-3], 0.1% chn [4-8]  pause = T/dc - T
		startSleepTS = millis();
		if (false) // Duty cycle is off -> shouln't happen to be in busy
			sleepMillis = RESFREEDEL/actBands;	// More bands, less wait TODO: use airtime based on DR
		else
			sleepMillis = rxWindow1 + rxWindow2 + 1000;
		internalState = iSleep;
		break;
	case iSleep:
		if (millis() - startSleepTS > sleepMillis)
			internalState = iIdle;
	}
}
