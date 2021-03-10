// Do not remove the include below
#include "main.h"

#define UNCF_POLL	5			// How many times to poll
#define TST_RETRY	5			// How many times retry to send message
#define TST_MXRSLT	30			// What's the max number of test results we allow?
#define RESFREEDEL	40000		// ~resource freeing delay ETSI requirement air-time reduction

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

static uint8_t testGrp = 1;						// Running variables number
static uint8_t testNo = 1;						// "	"	number

/* 	Globals		*/

int debug = 1;

// Test data structure
typedef struct _testParam{
	uint16_t chnEnabled;	// Channels enabled for this test
	uint8_t txPowerIdx;		// Initial TX power index
	uint8_t drMax;			// data rate Maximum for the test
	uint8_t drMin;			// data rate Minimum for the test
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

void readEEPromSettings () {

}

void writeEEPromDefaults() { // for defaults

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
			A = debugSerial.read();
			A = A - 48;
			if (A < 10 && A >= 0)
				testNo = A;
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
			A = debugSerial.read();
			A = A - 48;
			if (A < 10 && A >= 0){
				txPowerTst = A;
//				ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//					eeprom_update_byte(&ee_txPowerTst, txPowerTst);
//				}
			}
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
		// TODO: add data length menu
		}
	}

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

	// Blink once PIN13 to show program start
//	PORTC |= (0x01 << PINC7);
//	delay (500);
//	PORTC &= ~(0x01 << PINC7);

//	LoRaMgmtSetup();

	tgrp = &testConfig[0]; 	// assign pointer to pointer to TestgroupA
	tno = *tgrp; 			// assign pointer to pointer to test 1
	trn = &testResults[0];	// Init results pointer

	startTs = millis();		// snapshot starting time

//	printPrgMem(PRTSTTTBL, PRTSTTSELECT);
}


// The loop function is called in an endless loop
void loop()
{
	if (testend)
		readInput();
	else{
//		if (*tno)
//			runTest(*tno);
//		else
//			printPrgMem(PRTSTTTBL, PRTSTTERRTEXT);
		testend=0;
	}
	// received something
	// debugSerial.read();
}
