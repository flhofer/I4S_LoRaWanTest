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
#define LORACHNMAX 16
#define LORABUSY	-4			// error code for busy channel

extern char *appEui;
extern char *appKey;
extern char *devAddr;
extern char *nwkSKey;
extern char *appSKey;

int LoRaMgmtSetup();
int LoRaMgmtSetupDumb(long FRQ);
void LoRaSetGblParam(bool confirm, int datalen, int OTAA);

int LoRaSetChannels(uint16_t chnMsk, uint8_t drMin, uint8_t drMax);

int LoRaMgmtSend();
int LoRaMgmtSendDumb();
int LoRaMgmtPoll();
int LoRaMgmtUpdt();
int LoRaMgmtRcnf();
int LoRaMgmtTxPwr(uint8_t txPwr);

int LoRaMgmtGetResults(sLoRaResutls_t * res);

#endif /* LORAMGMT_H_ */
