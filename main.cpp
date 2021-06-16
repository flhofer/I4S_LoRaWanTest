// Do not remove the include below
#include "main.h"
#include "LoRaMgmt.h"			// LoRaWan modem management

#define UNCF_POLL	5			// How many times to poll
#define TST_MXRSLT	30			// What's the max number of test results we allow?
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction
#define LEDBUILDIN	PORT_PA20	// MKRWan1300 build in led position
#define KEYBUFF		73			// Max total usage of key buffers = 32 + 32 + 16 + 3*\0
#define KEYSIZE		32			// 32
/* Strings 		*/

const char prtSttStart[] PROGMEM = "Start test\n";
const char prtSttPoll[] PROGMEM = "Poll for answer\n";
const char prtSttStop[] PROGMEM = "Stop test\n";
const char prtSttRetry[] PROGMEM = "Retry\n";
const char prtSttEvaluate[] PROGMEM = "Evaluate\n";
const char prtSttAddMeas[] PROGMEM = " - add measurement\n";
const char prtSttReset[] PROGMEM = "Reset\n";
const char prtSttRestart[] PROGMEM = "Restart - Init\n";
const char prtSttEnd[] PROGMEM = "End test\n";
const char prtSttPollErr[] PROGMEM = "Poll - No response from server.\n";
const char prtSttDone[] PROGMEM = "done\n";
const char prtSttErrExec[] PROGMEM = "ERROR: during state execution\n";
const char prtSttErrText[] PROGMEM = "ERROR: test malfunction\n";
const char prtSttWrnConf[] PROGMEM = "WARN: Invalid test configuration\n";
const char prtSttSelect[] PROGMEM = "Select Test:\n";
const char prtSttResults[] PROGMEM = "Results:\n";

const char prtTblCR[] PROGMEM = " CR 4/";
const char prtTblDR[] PROGMEM = " DR ";
const char prtTblChnMsk[] PROGMEM = " MSK [EN] ";
const char prtTblFrq[] PROGMEM = " Freq [hz] ";
const char prtTblPwr[] PROGMEM = " pwr [dBm] ";
const char prtTblRssi[] PROGMEM = " rssi ";
const char prtTblSnr[] PROGMEM = " snr ";
const char prtTblTTx[] PROGMEM = " Time TX: ";
const char prtTblTRx[] PROGMEM = " Time RX: ";
const char prtTblTTl[] PROGMEM = " Time Total: ";
const char prtTblTms[] PROGMEM = " ms";

/* Locals 		*/

// Working variables
static sLoRaResutls_t testResults[TST_MXRSLT];	// Storage for test results
static sLoRaResutls_t * trn;					// Pointer to actual entry
static uint8_t actChan = 16;					// active channels
static long txCnt;								// transmission counter
static long durationTest;						// test duration in ms

static sLoRaConfiguration_t newConf;			// test Configuration
static char keyArray[KEYBUFF];					// static array containing init keys

/* 	Globals		*/

int debug = 1;			// print debug
int store = 0;			// store on SD, not terminal output (TODO)

/*************** MIXED STUFF ********************/

/*
 * printScaled(): Print integer as scaled value
 *
 * Arguments:	- value to print
 * 				- scale in x-ths, default (1/1000th)
 *
 * Return:		-
 */
static void
printScaled(uint32_t value, uint32_t Scale = 1000){
	debugSerial.print(value / Scale);
	debugSerial.print(".");
	value %= Scale;
	Scale /=10;
	while (value < Scale){
		debugSerial.print("0");
		Scale /=10;
	}
	if (value != 0)
		debugSerial.print(value);
}

/*
 * printTestResults(): Print LoRaWan communication test
 *
 * Arguments:	-
 *
 * Return:		-
 */
static void
printTestResults(){
	// use all local, do not change global
	sLoRaResutls_t * trn = &testResults[0]; // Initialize results pointer

	// for printing
	char buf[128];

	debugSerial.print(prtSttResults);
	for (int i = 1; i<= TST_MXRSLT; i++, trn++){
		sprintf(buf, "%02d;%07lu;%07lu;0x%02X;%lu;%02u;%02d;%03d;%03d",
				i, trn->timeTx, trn->timeRx,
				trn->chnMsk, trn->txFrq, trn->txDR, trn->txPwr,
				trn->rxRssi, trn->rxSnr);
		debugSerial.println(buf);
	}
}

/*
 * printTestResultsDumb(): print result from LoRa interference tests
 *
 * Arguments:	-
 *
 * Return:		-
 */
static void
printTestResultsDumb(){
	// for printing
	char buf[128];

	debugSerial.print(prtSttResults);
	sprintf(buf, "%07lu;%07lu;%lu;",
			durationTest, txCnt, newConf.frequency);
	debugSerial.println(buf);
}

/*
 * readSerialS(): parsing hex input strings
 *
 * Arguments:	- pointer to char* to put string
 *
 * Return:		-
 */
static void
readSerialS(char * retVal, int len){
	char nChar;
	int pos = 0;
	retVal[0] = '\0';
	while (1){
		if (pos > len ){
			debugSerial.println("Error: too long value! Remember using h to terminate hex");
			break;
		}

		nChar = debugSerial.peek();
		switch (nChar)
		{
			case '0' ... '9': // Digits
			case 'A' ... 'F': // capital letters A-F
			case 'a' ... 'f': // small letters a-f
				debugSerial.read();
				retVal[pos] = nChar;
				pos++;
				break;
			case 'h':	// hex termination
				debugSerial.read();
				// fall-through
				// @suppress("No break at end of case")
			default:	// Not a number or HEX char/terminator
				retVal[pos] = '\0';
				return;
		}
	}
	retVal[pos-1] = '\0';
	return;
}

/*
 * readSerialH(): parsing hex input strings
 *
 * Arguments:	-
 *
 * Return:		- Hex number read
 */
static uint16_t
readSerialH(){
	char nChar;
	uint16_t retVal = 0;
	while (1){
		nChar = debugSerial.peek();
		if ((UINT16_MAX >> 4) < retVal && nChar != 'h'){
			debugSerial.println("Error: too long value! Remember using h to terminate hex");
			break;
		}
		switch (nChar)
		{
			case '0' ... '9': // Digits
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - '0');
				break;
			case 'A' ... 'F': // capital letters A-F
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - 55); // 55 = A - 10
				break;
			case 'a' ... 'f': // small letters a-f
				debugSerial.read();
				retVal = retVal<<4 | (int16_t)(nChar - 87); // 87 = a - 10
				break;
			case 'h':	// hex termination
				debugSerial.read();
				// fall-through
				// @suppress("No break at end of case")
			default:	// Not a number or HEX char/terminator
				return retVal;
		}

	}
	return retVal;
}

/*
 * readSerialD(): parsing numeric input strings
 *
 * Arguments:	-
 *
 * Return:		- number read
 */
static uint16_t
readSerialD(){
	char A;
	uint16_t retVal = 0;
	while ((A = debugSerial.peek())
			&& (A < 58 && A >= 48)){
		if (log10((double)retVal) >= 4.0f ){
			debugSerial.println("Error: too long value!");
			break;
		}
		debugSerial.read();
		retVal = retVal*10 + (int16_t)(A - 48);
	}
	return retVal;
}

static void
resetKeyBuffer(){
	memset(keyArray, '\0', KEYBUFF );

	// key slots are the same for OTAA and ABP (union)
	newConf.appEui = &keyArray[0];				// Hex 32
	newConf.appKey = &keyArray[KEYSIZE+1];		// Hex 32
	newConf.devEui = &keyArray[KEYSIZE*2+2];	// Hex 8 or 16

}

/*************** TEST MANAGEMENT FUNCTIONS*****************/

static unsigned long startTs = 0; // loop timer

// Enumeration for test status
typedef enum { 	rError = -1,
				rInit = 0,
				rOff,
				rDumb,

				rJoin =5, //	 Join-Bomb
				rWaitLoRa,
				rStart,
				rRun,
				rStop,


				rEvaluate = 15,
				rReset,
				rPrint,

				rEnd = 20
			} testRun_t;

typedef enum {	qIdle = 0,
				qRun,
				qStop
			} testReq_t;

static testRun_t tstate = rInit;// test run status
static testReq_t testReq= qIdle;// test request status
static int	pollcnt;			// un-conf poll retries
static int	retries; 			// un-conf send retries

/*
 * runTest(): test runner, state machine
 *
 * Arguments: -
 *
 * Return:	  - test run enumeration status
 */
static testRun_t
runTest(){

	int failed = 0;
	int ret = 0;

	switch(tstate){

	case rInit:
		txCnt = 0;

		// reset status on next test
		memset(testResults,0, sizeof(testResults));
		trn = &testResults[0];	// Init results pointer

		{	// enter new locality

			// Allocate on stack a new configuration

			switch (newConf.mode)
			{
			default:
			case 0 : // off- mute
				tstate = rOff;
				break;
			case 1 : // dumb LoRa
				tstate = rDumb;
				break;
			case 2 :  // LoRaWan
				tstate = rStart;
				break;
			case 3 : // LoRaRemote
				tstate = rWaitLoRa;
				break;
			case 4 : // LoRaWan force Join
				tstate = rJoin;
				break;
			}

			if (LoRaMgmtSetup(&newConf))
			{
				tstate = rError;
				debugSerial.print(prtSttErrExec);
				break;
			}
		}
		debugSerial.print(prtSttStart);
		startTs = millis();
		break;

	/*
	 * Dumb Modem States
	 */

	case rDumb:
		(void)LoRaMgmtSend(); // TODO: this is now a single call
		txCnt++;
		// fall-through
		// @suppress("No break at end of case")

	case rOff:
		if (testReq >= qStop ){
			tstate = rPrint;
			durationTest = millis() - startTs;
		}
		break;

	/*
	 * LoRaWan execution states
	 */

	case rJoin:
		LoRaMgmtJoin();
		txCnt++;
		if (testReq >= qStop ){
			tstate = rPrint;
			durationTest = millis() - startTs;
		}
		break;

	case rWaitLoRa:
		// delay in mgmt - 1000ms
		{
			int cmd = 0;
			if ((cmd = LoRaMgmtRemote()) == 1)
				tstate = rStart;
			else if (cmd != 0)
				tstate = rError;
		}
		// fall-through
		// @suppress("No break at end of case")

	case rStart:

		if (testReq >= qStop ){
			tstate = rStop;
			durationTest = millis() - startTs;
		}

		if ((ret = LoRaMgmtSend()) && ret < 0){
			if (LORABUSY == ret) // no chn -> pause for free-delay / active channels
				delay(RESFREEDEL/actChan);
			else
				delay(100); // simple retry timer 100ms, e.g. busy
			break;
		}

		// sent but no response from confirmed, or not confirmed msg, goto poll
		if (ret == 1 || !(newConf.confMsk & CM_UCNF)){
			tstate = rRun;
			debugSerial.print(prtSttPoll);
		}
		else {
			tstate = rStop;
			debugSerial.print(prtSttStop);
			break;
		}
		// fall-through
		// @suppress("No break at end of case")

	case rRun:

		if (testReq >= qStop ){
			tstate = rStop;
			durationTest = millis() - startTs;
		}

		if ((ret = LoRaMgmtPoll()) && (!(newConf.confMsk & CM_UCNF) || (pollcnt < UNCF_POLL))){
			if (-9 == ret) // no chn -> pause for free-delay / active channels
				delay(RESFREEDEL/actChan);
			else if (1 == ret)
				pollcnt++;
			else
				delay(100); // simple retry timer 100ms, e.g. busy
			break;
		}

		// Unconf polling ended and still no response, or confirmed and error message (end of retries)
		if ((failed = (0 != ret)))
			debugSerial.print( prtSttPollErr);

		tstate = rStop;
		debugSerial.print(prtSttStop);
		// fall-through
		// @suppress("No break at end of case")

	case rStop:

		// unsuccessful and retries left?
		if (failed && (newConf.repeatSend > retries) && testReq < qStop){
			tstate = rStart;
			debugSerial.print(prtSttRetry);
			delay(RESFREEDEL/actChan); // delay for modem resource free
			(void)LoRaMgmtUpdt();
			break;
		}

		tstate = rEvaluate;
		debugSerial.print(prtSttEvaluate);
		// fall-through
		// @suppress("No break at end of case")

	case rEvaluate:
		debugSerial.print(prtSttAddMeas);

		ret = LoRaMgmtGetResults(trn);

		if (debug) {
			debugSerial.print(prtTblCR);
			debugSerial.print(trn->lastCR);
			debugSerial.print(prtTblDR);
			debugSerial.print(trn->txDR);
			debugSerial.print(prtTblChnMsk);
			debugSerial.println(trn->chnMsk);
			debugSerial.print(prtTblFrq);
			debugSerial.print(trn->txFrq);
			debugSerial.print(prtTblPwr);
			debugSerial.print(trn->txPwr);
			debugSerial.print(prtTblRssi);
			debugSerial.print(trn->rxRssi);
			debugSerial.print(prtTblSnr);
			debugSerial.println(trn->rxSnr);

			debugSerial.print(prtTblTTx);
			printScaled(trn->timeTx);
			debugSerial.print(prtTblTms);
			debugSerial.print(prtTblTRx);
			printScaled(trn->timeRx);
			debugSerial.print(prtTblTms);
			debugSerial.print(prtTblTTl);
			printScaled(trn->timeToRx);
			debugSerial.print(prtTblTms);
			debugSerial.println();
		}

		// End of tests?
		if (trn >= &testResults[TST_MXRSLT-1] || testReq >= qStop){
			debugSerial.print(prtSttEnd);
			tstate = rPrint;
			break;
		}

		trn++;
		tstate = rReset;
		debugSerial.print(prtSttReset);

		// fall-through
		// @suppress("No break at end of case")

	case rReset:
		debugSerial.print(prtSttRestart);
		tstate = rStart;
		break;

	/*
	 * Common states ending
	 */
	case rPrint:
		switch (newConf.mode){
		case 0:
			break;

		default:
		case 1:
		case 4:
			printTestResultsDumb();
			break;

		case 2:
		case 3:
			printTestResults();
			break;
		}
		tstate = rEnd;
		// fall-through
		// @suppress("No break at end of case")

	default:
	case rEnd:
		if (testReq != qIdle){
			debugSerial.print(prtSttDone);
			debugSerial.print(prtSttSelect);
			testReq = qIdle;
		}
	}

	return tstate;
}
/*
 * readInput(): read input string
 *
 * Arguments:	-
 *
 * Return:		-
 */
void readInput() {

	signed char A;
	while (debugSerial.available()){
		A = debugSerial.read();

		// work always
		switch (A){

		case 'm': // read test mode
			newConf.mode = (uint8_t)readSerialD();
			if (newConf.mode > 4){
				debugSerial.println("Invalid mode [0-4]");
				newConf.mode = 0; // set to default
			}
			resetKeyBuffer();
			// Defaults for LoRa mode
			if (newConf.mode == 1){
				newConf.frequency = 8683;
				newConf.bandWidth = 250;
				newConf.codeRate = 8;
				newConf.spreadFactor = 12;
			}
			break;

		case 'p': // read Tx power index
			newConf.txPowerTst = (uint8_t)readSerialD();
			if (newConf.txPowerTst > 5 ){ //TODO: this is limiting to EU868
				debugSerial.println("Invalid power level [0-5]");
				newConf.txPowerTst = 0; // set to default
			}
			break;

		case 'l': // read data length
			newConf.dataLen = (uint8_t)readSerialD();
			if (newConf.dataLen > 250 || newConf.dataLen == 0 ){
				debugSerial.println("Invalid data length [1-250]");
				newConf.dataLen = 1; // set to default
			}
			break;

		case 'r': // read repeat count for LoRaWan packets
			newConf.repeatSend = (uint8_t)readSerialD();
			if (newConf.repeatSend == 0 || newConf.repeatSend > TST_MXRSLT){
				debugSerial.print("Invalid repeat count [1-");
				debugSerial.print(TST_MXRSLT);
				debugSerial.println("]");
				newConf.repeatSend = 5; // set to default
			}

			break;
		case 'R': // set to run
			testReq = qRun;
			tstate = rInit;
			break;

		case 'S': // stop test
			testReq = max(qStop, testReq);
			debugSerial.println("Test stop!");
			break;

		case 'T': // Print type of micro-controller
			debugSerial.println(MICROVER);
			break;

		case 'I': // Print type of micro-controller
			debugSerial.println(LoRaMgmtGetEUI());
			break;

		case 'n': // disable debug print
			debug = 0;
			break;

		}
		if (newConf.mode == 1)
			switch (A){
			case 'f': //TODO: this is limiting to EU868
				newConf.frequency = (long)readSerialD(); // TODO: 10 vs 100kHz
				if (newConf.frequency < 8630 || newConf.frequency > 8700 ){
					debugSerial.println("Invalid frequency [8630-8700] * 100 kHz");
					newConf.frequency = 8683; // set to default
				}
				break;

			case 'b': // read bandwidth
				newConf.bandWidth = (uint8_t)readSerialD();
				if (!(newConf.bandWidth == 250 ||
						newConf.bandWidth == 125 ||
						newConf.bandWidth == 62 ||
						newConf.bandWidth == 41 ||
						newConf.bandWidth == 31 ||
						newConf.bandWidth == 20 ||
						newConf.bandWidth == 15 ||
						newConf.bandWidth == 10 )){
					debugSerial.println("Invalid bandwidth [ 250 | 125 | 62 | 41 | 31 | 20 | 15 | 10 ]");
					newConf.bandWidth = 250; // set to default
				}
				break;

			case 'c': // read code rate
				newConf.codeRate = (uint8_t)readSerialD();
				if (newConf.codeRate < 5 || newConf.codeRate > 8){
					debugSerial.println("Invalid code rate 4/[5-8]");
					newConf.codeRate = 8; // set to default
				}
				break;

			case 's': // read spread factor
				newConf.spreadFactor = (uint8_t)readSerialD();
				if (newConf.spreadFactor < 7 || newConf.spreadFactor > 12){
					debugSerial.println("Invalid spread factor [7-12]");
					newConf.spreadFactor = 12; // set to default
				}
				break;
			}

		if (newConf.mode >= 2)
			switch (A){

			case 'c': // set to confirmed
				newConf.chnMsk &= ~CM_UCNF;
				break;
			case 'u': // set to unconfirmed
				newConf.chnMsk |= CM_UCNF;
				break;

			case 'o': // set to otaa
				newConf.chnMsk |= CM_OTAA;
				resetKeyBuffer();
				break;
			case 'a': // set to ABP
				newConf.chnMsk &= ~CM_OTAA;
				resetKeyBuffer();
				break;

			case 'N': // Network session key for ABP
				readSerialS(newConf.nwkSKey, KEYSIZE);
				if (strlen(newConf.nwkSKey) < KEYSIZE){
					debugSerial.println("Invalid network session key");
					newConf.nwkSKey[0] = '\0';
				}
				break;

			case 'A': // Application session key for ABP
				readSerialS(newConf.appSKey, KEYSIZE);
				if (strlen(newConf.appSKey) < KEYSIZE){
					debugSerial.println("Invalid application session key");
					newConf.appSKey[0] = '\0';
				}
				break;

			case 'D': // Device address for ABP
				readSerialS(newConf.devAddr, KEYSIZE/4);
				if (strlen(newConf.devAddr) < KEYSIZE/4){
					debugSerial.println("Invalid device address key");
					newConf.devAddr[0] = '\0';
				}
				break;

			case 'K': // Application Key for OTAA
				readSerialS(newConf.appKey, KEYSIZE);
				if (strlen(newConf.appKey) < KEYSIZE){
					debugSerial.println("Invalid application key");
					newConf.appKey[0] = '\0';
				}
				break;

			case 'E': // EUI address for OTAA
				readSerialS(newConf.appEui, KEYSIZE/2);
				if (strlen(newConf.appEui) < KEYSIZE/2){
					debugSerial.println("Invalid EUI address");
					newConf.appEui[0] = '\0';
				}
				break;

			case 'C':
				newConf.chnMsk = readSerialH();
				if (newConf.chnMsk < 0x01 || newConf.chnMsk > 0xFF ){ //TODO: this is limiting to EU868
					debugSerial.println("Invalid channel mask [0x07-0xFFh]");
					newConf.chnMsk = 0xFF; // set to default
				}
				break;

			case 'd': // read data rate
				newConf.dataRate = (uint8_t)readSerialD();
				if (newConf.dataRate > 5){ // we exclude 6 and 7 for now
					debugSerial.println("Invalid data rate [0-5]");
					newConf.dataRate = 5; // set to default
				}
				break;

			}
	}

}

/*
 *	Setup(): system start
 *
 *	The setup function is called once at startup of the sketch
 */
void setup()
{
	REG_PORT_DIRSET0 = LEDBUILDIN; // Set led to output

	// Initialize Serial ports 0
	// setup serial for debug, disable if no connection after 10 seconds
	debugSerial.begin(9600);
	int waitSE = 999;	// wait for 10 seconds, -10 ms for computation
	while (!debugSerial && waitSE) {
	  delay(10);// -- Included in Serial_::operator()
	  waitSE--;
	}
	debug = ((waitSE));	// reset debug flag if time is elapsed

	// Blink once PIN20 to show program start
	REG_PORT_OUTSET0 = LEDBUILDIN;
	delay(500);
	REG_PORT_OUTCLR0 = LEDBUILDIN;

	debugSerial.print(prtSttSelect);
	debugSerial.flush();
}

/*
 *  loop(): main call
 *
 *  The loop function is called in an endless loop
 */
void loop()
{
	if (testReq == qIdle)
		readInput();
	else{
		runTest();
		// received something?
		if (debugSerial.peek())
			//stop?
			readInput();
	}
}
