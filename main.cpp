// Do not remove the include below
#include "main.h"
#include "LoRaMgmt.h"			// LoRaWan modem management

#define UNCF_POLL	5			// How many times to poll
#define TST_MXRSLT	30			// What's the max number of test results we allow?
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction
#define LEDBUILDIN	PORT_PA20	// MKRWan1300 build in led position

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

// Generic Settings
static uint8_t mode = 1;						// test mode = 0 LoRa, 1 LoRaWan, 2 TODO tests..
static long Frequency = 8683000;				// Frequency of dumb LoRa mode *100 in kHz
static long txCnt;								// transmission counter
static long durationTest;						// test duration in ms

// LoRaWan settings
static bool confirmed = true;					// confirmed messaging?
static bool otaa = false;						// use OTAA join instead of ABP
static uint16_t chnEnabled = 0xFF;				// Channels enabled mask for LoRaWan mode tests
static uint8_t txPowerTst = 4;					// txPower setting for the low power test
static uint8_t dataLen = 1;						// data length to send over LoRa for a test
static uint8_t dataRate = 5;					// data rate starting value
static uint8_t repeatSend = 5;					// number of send repeats

/* 	Globals		*/

int debug = 1;

/*************** MIXED STUFF ********************/

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

static void
printTestResults(){
	// use all local, do not change global
	sLoRaResutls_t * trn = &testResults[0]; // Init results pointer

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

static void
printTestResultsDumb(){
	// for printing
	char buf[128];

	debugSerial.print(prtSttResults);
	sprintf(buf, "%07lu;%07lu;%lu;",
			durationTest, txCnt, Frequency*100);
	debugSerial.println(buf);
}

uint16_t readSerialH(){
	char nChar;
	uint16_t retVal = 0;
	while (1){
		nChar = debugSerial.peek();
		switch (nChar)
		{
			case '0' ... '9': // Digits
				debugSerial.read();
				retVal = retVal*16 + (int16_t)(nChar - '0');
				break;
			case 'a' ... 'f': // small letters a-f
				debugSerial.read();
				retVal = retVal*16 + (int16_t)(nChar - 55); // 55 = A - 10
				break;
			case 'A' ... 'F': // capital letters A-F
				debugSerial.read();
				retVal = retVal*16 + (int16_t)(nChar - 87); // 87 = a - 10
				break;
			case 'h':	// hex termination
				debugSerial.read();
				// fall-through
				// @suppress("No break at end of case")
			default:
				return retVal;
		}

		if (log10((double)retVal) >= 4.0f ){
			debugSerial.println("Error: too long value! Remember using h to terminate hex");
			break;
		}
	}
	return retVal;
}

uint16_t readSerialD(){
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

/*************** TEST MANAGEMENT FUNCTIONS*****************/

static unsigned long startTs = 0; // loop timer

// Enumeration for test status
typedef enum { 	rError = -1,
				rInit = 0,
				rDumb,
				rStart,
				rRun,
				rStop,
				rEvaluate,
				rReset,
				rPrint,
				rEnd = 10
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
 * runTest: test runner
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


		// Set global test parameters
		LoRaSetGblParam(confirmed, dataLen, otaa);

		switch (mode)
		{
		default:
		case 0 : // dumb LoRa
			ret |= LoRaMgmtSetupDumb(Frequency);	// set frequency

			txCnt = 0;
			tstate = rDumb;
			break;

		case 1 : // LoRaWan
			// reset status on next test
			memset(testResults,0, sizeof(testResults));
			trn = &testResults[0];	// Init results pointer

			LoRaMgmtSetup();
			ret |= LoRaSetChannels(chnEnabled, 0, dataRate);	// set channels
			ret |= LoRaMgmtTxPwr(txPowerTst);	// set power index;
			tstate = rStart;
			break;
		// placeholder future modes.. test ecc
		}

		if (ret)
		{
			tstate = rError;
			debugSerial.print(prtSttErrExec);
			break;
		}

		debugSerial.print(prtSttStart);
		startTs = millis();
		break;

	/*
	 * Dumb Modem States
	 */

	case rDumb:
		(void)LoRaMgmtSendDumb();
		txCnt++;
		if (testReq >= qStop ){
			tstate = rPrint;
			durationTest = millis() - startTs;
		}
		break;

	/*
	 * LoRaWan execution states
	 */

	case rStart:

		if ((ret = LoRaMgmtSend()) && ret != 1){
			if (-9 == ret) // no chn -> pause for free-delay / active channels
				delay(RESFREEDEL/actChan);
			else
				delay(100); // simple retry timer 100ms, e.g. busy
			break;
		}

		// sent but no response from confirmed, or not confirmed msg, goto poll
		if (ret == 1 || !confirmed){
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

		if ((ret = LoRaMgmtPoll()) && (confirmed || (pollcnt < UNCF_POLL))){
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
		if (failed && (repeatSend > retries)){
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

		(void)LoRaMgmtGetResults(trn); // TODO: implement and use return value
		// pgm_read_word = read char pointer address from PROGMEM pos PRTTBLCR of the string array
		// strcpy_P = copy char[] from PRROGMEM at that address of PRROGMEM to buf
		// *.print = print that char to serial

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
		if (trn >= &testResults[TST_MXRSLT-1]){
			debugSerial.print(prtSttEnd);
			tstate = rEnd;
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
		switch (mode){
		default:
		case 0:
			printTestResultsDumb();
			break;

		case 1:
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

void readInput() {

	signed char A;
	while (debugSerial.available()){
		A = debugSerial.read();
		switch (A){

		case 'm': // read test mode
			mode = (uint8_t)readSerialD();
			if (mode > 1){
				debugSerial.println("Invalid mode [0-1]");
				mode = 1; // set to default
			}
			break;

		case 'f':
			Frequency = (long)readSerialD() * 100000;
			if (Frequency < 8630 || Frequency > 8700 ){
				debugSerial.println("Invalid frequency [8630-8700] * 100 kHz");
				Frequency = 868300000; // set to default
			}
			break;

		case 'c': // set to confirmed
			confirmed = true;
			break;
		case 'u': // set to unconfirmed
			confirmed = false;
			break;

		case 'o': // set to otaa
			otaa = true;
			break;
		case 'a': // set to abp
			otaa = false;
			break;

		case 'C':
			chnEnabled = readSerialH();
			if (chnEnabled == 0 || chnEnabled > 255 ){
				debugSerial.println("Invalid channel mask [1-16]");
				chnEnabled = 0xFF; // set to default
			}
			break;

		case 'p': // read tx power index
			txPowerTst = (uint8_t)readSerialD();
			if (txPowerTst > 5 ){
				debugSerial.println("Invalid power level [0-5]");
				txPowerTst = 4; // set to default
			}
			break;


		case 'l': // read data length
			dataLen = (uint8_t)readSerialD();
			if (dataLen > 250 || dataLen == 0 ){
				debugSerial.println("Invalid data length [1-250]");
				dataLen = 1; // set to default
			}
			break;

		case 'd': // read data length
			dataRate = (uint8_t)readSerialD();
			if (dataRate > 5){ // we exclude 6 and 7 for now
				debugSerial.println("Invalid data rate [0-5]");
				dataRate = 5; // set to default
			}
			break;

		case 'r': // read repeat count for LoRaWan packets
			repeatSend = (uint8_t)readSerialD();
			if (repeatSend == 0 || repeatSend > TST_MXRSLT){
				debugSerial.print("Invalid repeat count [1-"); // TODO: simplify
				debugSerial.print(TST_MXRSLT);
				debugSerial.println("]");
				repeatSend = 5; // set to default
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

		case 'n': // disable debug print
			debug = 0;
			break;
		}
	}

}

//The setup function is called once at startup of the sketch
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


// The loop function is called in an endless loop
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
