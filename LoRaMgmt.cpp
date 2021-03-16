/*
 * LoRaMgmt.cpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Florian Hofer
 */

#include "LoRaMgmt.h"
#include "main.h"				// Global includes/definitions, i.e. address, key, debug mode
// #include <stdlib.h>				// AVR standard library

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


