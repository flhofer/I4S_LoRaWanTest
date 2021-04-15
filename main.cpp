// Do not remove the include below
#include "main.h"
#include "LoRaMgmt.h"			// LoRaWan modem management

#define UNCF_POLL	5			// How many times to poll
#define TST_RETRY	5			// How many times retry to send message
#define TST_MXRSLT	30			// What's the max number of test results we allow?
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

/* Strings 		*/

const char prtSttStart[] = "Start test\n";
const char prtSttPoll[] = "Poll for answer\n";
const char prtSttStop[] = "Stop test\n";
const char prtSttRetry[] = "Retry\n";
const char prtSttEvaluate[] = "Evaluate\n";
const char prtSttAddMeas[] = " - add measurement\n";
const char prtSttReset[] = "Reset\n";
const char prtSttRestart[] = "Restart - Init\n";
const char prtSttEnd[] = "End test\n";
const char prtSttPollErr[] = "Poll - No response from server.\n";
const char prtSttDone[] = "done\n";
const char prtSttErrExec[] = "ERROR: during state execution\n";
const char prtSttErrText[] = "ERROR: test malfunction\n";
const char prtSttWrnConf[] = "WARN: Invalid test configuration\n";
const char prtSttSelect[] = "Select Test:\n";
const char prtSttResults[] = "Results:\n";

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

static sLoRaResutls_t testResults[TST_MXRSLT];	// Storage for test results
static sLoRaResutls_t * trn;					// Pointer to actual entry
static char prntGrp;							// Actual executing group
static int prntTno;								// actual executing testno
static uint8_t actChan = 16;					// active channels
static int testend = 1;							// is test terminated?
static uint8_t dataLen = 1;						// data length to send over LoRa for a test
static uint8_t dataLenMin = 1;					// Min data length to send over LoRa
static uint8_t dataLenMax = 255;				// Max data length to send over LoRa
static uint8_t txPowerTst = 4;					// txPower setting for the low power test
static bool confirmed = true;					// TODO: implement menu and switch, BUT should it be changed?
static long Frequency;							// Frequency of dumb lora mode


static uint8_t testGrp = 1;						// Running variables number
static uint8_t testNo = 1;						// "	"	number

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	uint16_t chnEnabled;	// Channels enabled for this test, OR 0.1 MHz res of dumb channel
	uint8_t txPowerIdx;		// Initial TX power index
	uint8_t dr;				// data rate starting value
	uint8_t Mode;			// mode = 0 LoRa, 1 LoRaWan, 2 TODO tests..
	// RX1 Window
	// RX1 delay
	// RX1 DR offset
	// RX2 Window
	// RX2 delay
} testParam_t;

/*************** TEST CONFIGURATIONS ********************/

// Test definition
static testParam_t testA1 = {
	0x01,	// channels
	1,		// TX
	5,		// DR max
	0		// DR min
};

// Test definition
static testParam_t testB1 = {
	0x01,	// channels
	4,		// TX
	5,		// DR max
	0		// DR min
};

static testParam_t testC1 = {
	0xFF,	// channels
	1,		// TX
	5,		// DR max
	0		// DR min
};

// Test group definition - A all remain the same
static testParam_t * testGrpA[] = {
		&testA1,
		&testC1,
		&testA1,
		&testA1,
		&testA1,
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpB[] = {
		&testA1,
		&testB1,
		&testA1,
		NULL // Terminator for automatic sizing
};

static testParam_t * testGrpC[] = {
		&testA1,
		&testA1,
		&testC1,
		&testC1,
		&testC1,
		NULL // Terminator for automatic sizing
};

// All tests grouped
static testParam_t **testConfig[] = { // array of testParam_t**
		testGrpA, // array of testParam_t* (by reference), pointer to first testParam_t* in array
		testGrpB,
		testGrpC,
		NULL // Terminator for automatic sizing
};

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
		sprintf(buf, "%c;%02d;%02d;%07lu;%07lu;0x%02X;%lu;%02u;%02d;%03d;%03d",
				prntGrp, prntTno, i, trn->timeTx, trn->timeRx,
				trn->chnMsk, trn->txFrq, trn->txDR, trn->txPwr,
				trn->rxRssi, trn->rxSnr);
		debugSerial.println(buf);
	}
}

void readEEPromSettings () {

}

void writeEEPromDefaults() { // for defaults

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

static testParam_t ** tno = NULL;
static testParam_t *** tgrp = NULL;

static unsigned long startTs = 0; // loop timer

// Enumeration for test status
enum testRun { 	rError = -1,
				rInit = 0,
				rStart,
				rRun,
				rStop,
				rEvaluate,
				rReset,
				rEnd = 10
			};

static enum testRun tstate = rInit;
static int	pollcnt;			// un-conf poll retries
static int	retries; 			// un-conf send retries

/*
 * runTest: test runner
 *
 * Arguments: - pointer to test structure of actual test
 *
 * Return:	  - test run enumeration status
 */
static enum testRun
runTest(testParam_t * testNow){

	int failed = 0;

	if (!testNow){
		debugSerial.println(prtSttWrnConf);
		return rError;
	}

	int ret = 0;
	switch(tstate){

	case rInit:

		// reset status on next test
		memset(testResults,0, sizeof(testResults));
		trn = &testResults[0];	// Init results pointer

		// Set global test parameters
		LoRaSetGblParam(confirmed, dataLen);

		// Setup channels as configured
		if ((LoRaSetChannels(testNow->chnEnabled, testNow->dr, testNow->Mode)) // set channels
			|| (LoRaMgmtTxPwr(testNow->txPowerIdx))) { // set power index;
			tstate = rError;
			break; // TODO: error
		}

		tstate = rStart;
		debugSerial.print(prtSttStart);
		// fall-through
		// @suppress("No break at end of case")

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
		if (failed && (TST_RETRY > retries)){
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

		// End of tests?
		if (trn >= &testResults[TST_MXRSLT-1]){
			debugSerial.print(prtSttEnd);
			printTestResults();
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

	default:
	case rEnd:
		if (!testend){
			debugSerial.print(prtSttDone);
			debugSerial.print(prtSttSelect);
		}
		testend = 1;
	}

	if (-1 == ret && (rStart != tstate) && (rRun != tstate) ){
		debugSerial.print(prtSttErrExec);
		tstate = rEnd;
	}

	return tstate;
}

void readInput() {

	signed char A;
	while (debugSerial.available()){
		A = debugSerial.read();
		switch (A){

		// read parameter, they come together
		case 'G':
		case 'g': // read test group
			A = debugSerial.read();
			A = A - 65;
			if (A < 5 && A >= 0)
				testGrp = A;
			break;

		case 'T':
		case 't': // read test number to go
			testNo = (uint8_t)readSerialD();
			debugSerial.println(testNo);
			break;

		case 'U':
		case 'u': // set to unconfirmed
			confirmed = false;
//			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//				eeprom_update_byte(&ee_confirmed, confirmed);
//			}
			break;

		case 'C':
		case 'c': // set to confirmed
			confirmed = true;
//			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//				eeprom_update_byte(&ee_confirmed, confirmed);
//			}
			break;

		case 'P':
		case 'p': // read tx power index
			txPowerTst = (uint8_t)readSerialD();
			debugSerial.println(txPowerTst);
//			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//				eeprom_update_byte(&ee_txPowerTst, txPowerTst);
//			}
			break;
		case 'F':
		case 'f':
			Frequency = readSerialD();
			debugSerial.println(Frequency);
			break;
		case 'R':
		case 'r': // set to run
			testend = false;
			prntGrp='A'+testGrp;
			prntTno=testNo;
			// run through tests to pick the right test
			while (testGrp && *tgrp){
				tgrp++; // next test group
				testGrp--;
			}
			tno = *tgrp;
			while (testNo && *tno){
				tgrp++; // next test group
				testNo--;
			}
			startTs = millis();
			tstate = rInit;
			break;
		case 'd':
		case 'D': // reset to defaults
			writeEEPromDefaults();
			readEEPromSettings();
			break;
		}
	}

}

void initVariant () {

	PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN; // Input and Pullup, Port A, pin 15
	REG_PORT_OUTSET0 = PORT_PA15; // PULL-UP resistor rather than pull-down

	REG_PORT_DIRSET0 = PORT_PA20; // Set led to output
}


//The setup function is called once at startup of the sketch
void setup()
{
	// Initialize Serial ports 0
	if (debug) // don't even try if not set
	{
		// setup serial for debug, disable if no connection after 10 seconds
		debugSerial.begin(9600);
		int waitSE = 999;	// wait for 10 seconds, -10 ms for computation
		while (!debugSerial && waitSE) {
		  //delay(10);// -- Included in Serial_::operator()
		  waitSE--;
		}
		debug = ((waitSE));	// reset debug flag if time is elapsed
	}

	// Blink once PIN20 to show program start
	REG_PORT_OUTSET0 = PORT_PA20;
	delay(500);
	REG_PORT_OUTCLR0 = PORT_PA20;
	LoRaMgmtSetup();

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1
	trn = &testResults[0];	// Init results pointer

	startTs = millis();		// snapshot starting time

	debugSerial.print(prtSttSelect);
	debugSerial.flush();
}


// The loop function is called in an endless loop
void loop()
{
	if (testend)
		readInput();
	else{
		if (*tno)
			runTest(*tno);
		else
			debugSerial.print(prtSttErrText);
	}
	// received something
	// debugSerial.read();
}
