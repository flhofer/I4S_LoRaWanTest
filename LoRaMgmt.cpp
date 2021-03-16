/*
 * LoRaMgmt.cpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#include "LoRaMgmt.h"

#include "main.h"				// Global includes/definitions, i.e. address, key, debug mode
#include <MKRWAN.h>
#include <LoRa.h>
//#include <stdlib.h>				// ARM standard library

LoRaModem modem(loraSerial);

// DevAddr, NwkSKey, AppSKey and the frequency plan
static const char *devAddr = LORA_DEVADDR;
static const char *nwkSKey = LORA_NWSKEY;
static const char *appSKey = LORA_APSKEY;

/*
 * LoRaMgmtSendConf: send a confirmed message. If no response arrives
 * 		within timeout, return 1 (busy)
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtSend(){
	  modem.beginPacket();
	  modem.print(msg);
	  err = modem.endPacket(true);
}

/*
 * LoRaMgmtPoll: poll function for confirmed and delayed TX
 *
 * Arguments: -
 *
 * Return:	  status of polling, 0 ok, -1 error, 1 busy
 */
int LoRaMgmtPoll(){


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

	debugSerial.print("Your module version is: ");
	debugSerial.println(modem.version());
	debugSerial.print("Your device EUI is: ");
	debugSerial.println(modem.deviceEUI());

	int connected = modem.joinABP(devAddr, nwkSKey, appSKey);
	if (!connected) {
		debugSerial.println("Something went wrong; are you indoor? Move near a window and retry");
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

}

/*
 * LoRaSetChannels:
 *
 * Arguments: - channel enable bit mask, 0 off, 1 on
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaSetChannels(uint16_t chnMsk, uint8_t drMin, uint8_t drMax){

}

/*
 * LoRaMgmtUpdt: update LoRa message buffer
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtUpdt(){

}

/*
 * LoRaMgmtRcnf: reset modem and reconf
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtRcnf(){

}

/*
 * LoRaMgmtTxPwr: set power index on modem
 *
 * Arguments: -
 *
 * Return:	  - return 0 if OK, -1 if error
 */
int LoRaMgmtTxPwr(uint8_t txPwr){

}


