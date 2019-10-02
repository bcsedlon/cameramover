#include "Arduino.h"

#define TEXT_ID0 "CAMERA MOVER"
#define TEXT_ID1 "GROWMAT.CZ"

//#include "libraries/Timer3/TimerThree.h"
//#include <TimerOne.h>

#include <AccelStepper.h>
//#include <MultiStepper.h>

const byte KPD_ROWS = 4;
const byte KPD_COLS = 4;

#define KPD_I2CADDR 0x20 //0x38
char keys[KPD_ROWS][KPD_COLS] = {
		  {'1','2','3','A'},
		  {'4','5','6','B'},
		  {'7','8','9','C'},
		  {'*','0','#','D'}
};

#define OK_DELAY 500

#define LCD_I2CADDR 0x27 //0x3F
const byte LCD_ROWS = 2;
const byte LCD_COLS = 16;

#define LED_PIN 	13	//13

#define PRESHOT_PIN	3
#define SHOT_PIN	4	//6	//12 //5

#define R_DIR_PIN	12	//2//10 //6
#define R_PUL_PIN	11	//3//11 //7
#define R_ENA_PIN	10

#define L_DIR_PIN	8	//4//8
#define L_PUL_PIN	7	//5//9
#define L_ENA_PIN	6

AccelStepper stepperL(AccelStepper::DRIVER, L_PUL_PIN, L_DIR_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, R_PUL_PIN, R_DIR_PIN);

#define SHOTMODE_ADDR		4
#define SPEED_ADDR			8
#define ACCELERATION_ADDR	12
#define L0_ADDR				16
#define R0_ADDR				20
#define L1_ADDR				24
#define R1_ADDR				28
#define PRESHOTTIME_ADDR	32
#define SHOTTIME_ADDR		36
#define POSTSHOTTIME_ADDR	40
#define LDIRINV_ADDR		44
#define RDIRINV_ADDR		48
#define SHOTINV_ADDR		52
#define STOPS_ADDR			56
#define LMIN_ADDR			60
#define RMIN_ADDR			64
#define LMAX_ADDR			68
#define RMAX_ADDR			72
#define LSCALE_ADDR			76
#define RSCALE_ADDR			80
//#define LANGLE_ADDR			84
//#define RANGLE_ADDR			88
#define LENAINV_ADDR		92
#define RENAINV_ADDR		96
#define STOPS2_ADDR			84
//#define OVERLAYMODE_ADDR	104

#define MESSAGE_CMD_REQUEST  		"?"

#define MESSAGE_CMD_PARREADINT 		"#PRI"
#define MESSAGE_CMD_PARREADFLOAT 	"#PRF"
#define MESSAGE_CMD_PARWRITEINT 	"#PWI"
#define MESSAGE_CMD_PARWRITEFLOAT 	"#PWF"
#define MESSAGE_CMD_PARRELOAD 		"#PLD"

#define MESSAGE_CMD_SETZ 		"Z"
#define MESSAGE_CMD_SETL 		"L"
#define MESSAGE_CMD_SETR 		"R"
#define MESSAGE_CMD_SCROLL 		"S"

//#define MESSAGE_CMD_GO 		"G"
//#define MESSAGE_CMD_SETX 		"X"
//#define MESSAGE_CMD_SETY 		"Y"
/*
//https://github.com/euphy/polargraph/wiki/Polargraph-machine-commands-and-responses
#define MESSAGE_PG_C01			"C01" //C01,<l>,<r>,END
//#define MESSAGE_PG_C09			"C09" //C09,<l>,<r>,END
#define MESSAGE_PG_C17			"C17" //C17,<l>,<r>,<line segment length>,END
#define MESSAGE_PG_C13			"C13" //C13,[<servo position>,]END //drawing
#define MESSAGE_PG_C14			"C14" //C14,[<servo position>,]END //not drawing
#define MESSAGE_PG_C17			"C17" //C17,<l>,<r>,<line segment length>,END
#define MESSAGE_PG_END			"END" //C01,<l>,<r>,END
// response
#define MESSAGE_PG_READY		"READY"

#define MC_MEGA 2
#define MICROCONTROLLER MC_MEGA
#define READY_STR "READY_100"

*/

#define UISTATE_MAIN 		0
//#define UISTATE_FILELIST 	1
//#define UISTATE_SETCLOCK 	2
#define UISTATE_INFO 		3
//#define UISTATE_EDITTEXT 	4
#define UISTATE_CONTROL 	5

#define STATE_STOPPED 		0
#define STATE_RUNNING 		1
#define STATE_PAUSED 		2
#define STATE_DONE	 		3

#define KPD_UP		'A'
#define KPD_DOWN 	'B'
#define KPD_LEFT 	'#'
#define KPD_RIGHT 	'D'
#define KPD_ENTER 	'*'
#define KPD_ESC 	'0'

//#include <avr/wdt.h>

//////////////////////////////////
// variables
//////////////////////////////////

byte state = 0;
bool stop;

unsigned long lastMillis;
byte secondsCounter;
bool secToggle = false;

char uiKeyPressed = 0;
int uiState, uiPage;
unsigned long uiKeyMillis;

//unsigned long cycles, cyclesPrev = 0;

unsigned long preShotMillis, shotMillis, postShotMillis;
//long leftPuls, rightPuls;
//long leftPulsPos, rightPulsPos;
long lComm, rComm, zComm;
long stops, stopsCounter, stops2, stopsCounter2;

// control
byte preShotMode, shotMode, rightDirMode, rightPulMode, leftPulMode, leftDirMode;
bool preShotControl, shotControl, rightPulControl, rightDirControl, leftPulControl, leftDirControl;
bool preShotAuto, shotAuto, rightPulAuto, rightDirAuto, leftPulAuto, leftDirAuto;
bool leftDirInv, rightDirInv, preShotInv, shotInv, leftEnaInv, rightEnaInv;

//#define STEPS_ACC 16
//int rightAcc, rightAccSkip, leftAcc, leftAccSkip;

// parameters
unsigned int pulSpeed, pulAcceleration, preShotTime, shotTime, postShotTime;
long l0, r0, l1, r1, lMax, lMin, rMax, rMin;
unsigned int pulSpeedPrev;

long lScale, rScale;
//long lAngle, rAngle, overlay;
//byte overlayMode;

int shotState;

//int xComm, yComm, zComm;

/*
unsigned int fileIndex = 2;
unsigned int distance = 3000;
unsigned int leftInitLength = 2500;
*/


// inputs

// parameters
/*
unsigned int rightDirOnHour, rightDirOnMin, rightDirOffHour, rightDirOffMin;
unsigned int leftPulOnMin, leftPulOnSec, leftPulOffMin, leftPulOffSec;
float rightDirOnTemp, rightDirOffTemp, rightDirOnTempNight, rightDirOffTempNight;

bool parseCmd(String &text, const char* cmd, byte &mode, const int address=0) {
	int pos = text.indexOf(cmd);
	if(pos > -1) {
		char ch = text.charAt(pos + strlen(cmd));
		if(ch=='A') mode = 0;
		else if(ch=='0') mode = 1;
		else if(ch=='1') mode = 2;
		if(address)
			OMEEPROM::write(address, mode);
		return true;
	}
	return false;
}
*/

//#include <Wire.h>
//#include <SoftwareSerial.h>

bool stepperRunL() {
	if(leftEnaInv)
		digitalWrite(L_ENA_PIN, !stepperL.isRunning());
	else
		digitalWrite(L_ENA_PIN, stepperL.isRunning());

	if(stepperL.targetPosition() > lMax)
			stepperL.moveTo(lMax);
	if(stepperL.targetPosition() < lMin)
		stepperL.moveTo(lMin);
	return stepperL.run();
}

bool stepperRunR() {
	if(rightEnaInv)
		digitalWrite(R_ENA_PIN, !stepperR.isRunning());
	else
		digitalWrite(R_ENA_PIN, stepperR.isRunning());

	if(stepperR.targetPosition() > rMax)
		stepperR.moveTo(rMax);
	if(stepperR.targetPosition() < rMin)
		stepperR.moveTo(rMin);
	return 	stepperR.run();;
}

void steppersRun() {
	stepperRunL();
	stepperRunR();
}

void delayWithSteppersRun(unsigned long delay) {
	unsigned long delayMillis = millis();
	while(millis() - delayMillis < delay) {
		steppersRun();
	}
}

// LCD i2c
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(LCD_I2CADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Keypad 4x4 i2c
#include "Keypad_I2C.h"
#include "Keypad.h"

class Keypad_I2C2 : public Keypad_I2C {
	unsigned long keyMillis;

public:
    Keypad_I2C2(char *userKeymap, byte *row, byte *col, byte numRows, byte numCols, byte address, byte width = 1) : Keypad_I2C(userKeymap, row, col, numRows, numCols, address, width) {
    };

    char Keypad_I2C2::getRawKey() {
    	getKeys();

    	/*
      	if(bitMap[3] == 1) return '*';
        if(bitMap[3] == 2) return '0';
        if(bitMap[3] == 4) return '#';
        if(bitMap[3] == 8) return 'D';

        if(bitMap[2] == 1) return '7';
        if(bitMap[2] == 2) return '8';
        if(bitMap[2] == 4) return '9';
        if(bitMap[2] == 8) return 'C';

        if(bitMap[1] == 1) return '4';
        if(bitMap[1] == 2) return '5';
        if(bitMap[1] == 4) return '6';
        if(bitMap[1] == 8) return 'B';

        if(bitMap[0] == 1) return '1';
        if(bitMap[0] == 2) return '2';
        if(bitMap[0] == 4) return '3';
        if(bitMap[0] == 8) return 'A';

        return NO_KEY;
        */

    	if(bitMap[3] == 1) return '*';
    	if(bitMap[3] == 2) return '7';
    	if(bitMap[3] == 4) return '4';
    	if(bitMap[3] == 8) return '1';

    	if(bitMap[2] == 1) return '0';
    	if(bitMap[2] == 2) return '8';
    	if(bitMap[2] == 4) return '5';
    	if(bitMap[2] == 8) return '2';

    	if(bitMap[1] == 1) return '#';
    	if(bitMap[1] == 2) return '9';
    	if(bitMap[1] == 4) return '6';
    	if(bitMap[1] == 8) return '3';

    	if(bitMap[0] == 1) return 'D';
    	if(bitMap[0] == 2) return 'C';
    	if(bitMap[0] == 4) return 'B';
    	if(bitMap[0] == 8) return 'A';

    	return NO_KEY;
    };

    char Keypad_I2C2::getKey2() {
    	getKeys();
    	if(bitMap[0] & 1) {
    	    		if(bitMap[1] & 1) {
    	    			stop = true;
    	    			//lcd.noBacklight();
    	    			lcd.clear();
    	    			lcd.print(F("EMERGENCY STOP"));
    	    			uiState = UISTATE_MAIN;
    	    			//delay(1000);
    	    			delayWithSteppersRun(OK_DELAY);
    	    			//1 + A
    	    		}

    	    		if(bitMap[3] & 8) {
    	    			stop = false;
    	    			lcd.clear();
    	    			lcd.print(F("EM STOP RESET"));
    	    			//TODO watchdog test
    	    			//lcd.clear();
    	    			//lcd.print(F("WATCHDOG TEST"));
    	    			//while(true) {};
    	    			//1 + 3
    	    		}
    	}
    	/*
    	//TODO !!! Dirty trick !!!
    	if(bitMap[1] & 1) {
    		if(bitMap[0] == 2) rightDirMode=1;
    		if(bitMap[0] == 4) rightDirMode=2;
    		if(bitMap[0] == 8) rightDirMode=0;

    		if(bitMap[1] & 2) rightDirMode=1;
    		if(bitMap[1] & 4) rightDirMode=2;
    		if(bitMap[1] & 8) rightDirMode=0;

    		if(bitMap[2] == 2) {
    			leftPulMode=1;
    			return NO_KEY;
    		}
    		if(bitMap[2] == 4) leftPulMode=2;
    		if(bitMap[2] == 8) {
    			leftPulMode=0;
    			return NO_KEY;
    		}

    		//return NO_KEY;
    	}
    	*/

    	//TODO: again?
    	getKeys();

		if(bitMap[3] == 4) {
			if(bitMap[2] == 8) uiMovePause();
			if(bitMap[1] == 8) uiMoveStart();
			if(bitMap[0] == 8) uiMoveStop();

			if(bitMap[2] == 4) shotMode=1;
			if(bitMap[1] == 4) shotMode=2;
			if(bitMap[0] == 4) shotMode=0;

			/*
			if(bitMap[2] == 2) rightPulMode=1;
			if(bitMap[1] == 2) rightPulMode=2;
			if(bitMap[0] == 2) rightPulMode=0;

			if(bitMap[2] == 1) rightDirMode=1;
			if(bitMap[1] == 1) rightDirMode=2;
			if(bitMap[0] == 1) rightDirMode=0;
			*/
		}
		if((bitMap[3] == 2 || bitMap[3] == 2 + 4) || uiState == UISTATE_CONTROL) {
			//7 +
			//TODO: move calibration
			//if(bitMap[3] == 8 || bitMap[3] == 8 + 2) {
			if(bitMap[2] == 4) {
				//stepperL.move(10000);
				stepperL.moveTo(min(lMax, stepperL.currentPosition() + 1000));
			}
			else if(bitMap[2] == 8) {
				//stepperL.move(-10000);
				stepperL.moveTo(max(lMin, stepperL.currentPosition() - 1000));
			}
			else {
				if(stepperL.isRunning()) {
					stepperL.setAcceleration(100000000.0);
					stepperL.stop();
					while(stepperRunL());
					stepperL.setAcceleration(pulAcceleration);
					//stepperL.moveTo(stepperL.currentPosition());
				}
			}

			//if(bitMap[1] == 8) {
			if(bitMap[3] == 4 || bitMap[3] == 4 + 2) {
				//stepperR.move(10000);
				stepperR.moveTo(min(rMax, stepperR.currentPosition() + 1000));
			}
			//else if(bitMap[0] == 8) {
			else if(bitMap[1] == 4) {
				//stepperR.move(-10000);
				stepperR.moveTo(max(rMin, stepperR.currentPosition() - 1000));
			}
			else {
				if(stepperR.isRunning()) {
					stepperR.setAcceleration(100000000.0);
					stepperR.stop();
					while(stepperRunR());
					stepperR.setAcceleration(pulAcceleration);
					//stepperR.moveTo(stepperR.currentPosition());
				}
			}

	    	//TODO:
	    	if(bitMap[2] == 1) {
	    		shotMode = 2;
	    	}
	    	else {
	    		shotMode = 0;
	    	}
		}
    	if(bitMap[0] || bitMap[1] || bitMap[2] || bitMap[3]) {
    		if(!keyMillis) {
    			keyMillis = millis();
    		}
    		if((keyMillis + 500) > millis()){
    			if((keyMillis + 200) < millis()) {
    				return NO_KEY;
    			}
    		}
    	}
        else
        	keyMillis = 0;

    	return getRawKey();
    }
};

byte rowPins[KPD_ROWS] = {0, 1, 2, 3}; //connect to the row pinouts of the keypad
byte colPins[KPD_COLS] = {4, 5, 6, 7}; //connect to the column pinouts of the keypad
Keypad_I2C2 kpd( makeKeymap(keys), rowPins, colPins, KPD_ROWS, KPD_COLS, KPD_I2CADDR, PCF8574 );

// Menu
//#include "libraries/OMEEPROM/OMEEPROM.h"
//#include "libraries/OMMenuMgr/OMMenuMgr.h"
#include "OMEEPROM.h"
#include "OMMenuMgr2.h"

class OMMenuMgr2 : public OMMenuMgr {
public:
    OMMenuMgr2(const OMMenuItem* c_first, uint8_t c_type, Keypad_I2C2* c_kpd) :OMMenuMgr( c_first, c_type) {
      kpd = c_kpd;
    };
    int OMMenuMgr2::_checkDigital() {
    	char k = kpd->getKey2();
    	if(k == 'A') return BUTTON_INCREASE;
    	if(k == 'B') return BUTTON_DECREASE;
    	if(k == 'D') return BUTTON_FORWARD;
    	if(k == '#') return BUTTON_BACK;
    	if(k == '*') return BUTTON_SELECT;
    	return k;
    	return BUTTON_NONE;
    }
private:
    Keypad_I2C2* kpd;
};

// Create a list of states and values for a select input
MENU_SELECT_ITEM  sel_auto =	{0, {"AUTO!"}};
MENU_SELECT_ITEM  sel_off =		{1, {"OFF!"}};
MENU_SELECT_ITEM  sel_on  = 	{2, {"ON!"}};

MENU_SELECT_LIST  const state_list[] = { &sel_auto, &sel_off, &sel_on};
MENU_SELECT_LIST  const state_listOffOn[] = { &sel_off, &sel_on };
/*
MENU_SELECT rightDirMode_select = { &rightDirMode,           MENU_SELECT_SIZE(state_list),   MENU_TARGET(&state_list) };
MENU_VALUE rightDirMode_value =   { TYPE_SELECT,     0,     0,     MENU_TARGET(&rightDirMode_select), LIGHTMODE_ADDR };
MENU_ITEM rightDirMode_item =     { {"RIGHT DIR"}, ITEM_VALUE,  0,        MENU_TARGET(&rightDirMode_value) };
//                               TYPE             MAX    MIN    TARGET
MENU_SELECT rightPulMode_select =   { &rightDirMode,           MENU_SELECT_SIZE(state_list),   MENU_TARGET(&state_list) };
MENU_VALUE rightPulMode_value =     { TYPE_SELECT,     0,     0,     MENU_TARGET(&rightPulMode_select) , FANMODE_ADDR};
MENU_ITEM rightPulMode_item    =    { {"RIGHT PUL"}, ITEM_VALUE,  0,        MENU_TARGET(&rightPulMode_value) };

MENU_SELECT leftPulMode_select ={ &leftPulMode,           MENU_SELECT_SIZE(state_list),   MENU_TARGET(&state_list) };
MENU_VALUE leftPulMode_value =  { TYPE_SELECT,     0,     0,     MENU_TARGET(&leftPulMode_select), CYCLERMODE_ADDR };
MENU_ITEM leftPulMode_item    = { {"LEFT PUL"}, ITEM_VALUE,  0,        MENU_TARGET(&leftPulMode_value) };

MENU_SELECT leftDirMode_select ={ &leftDirMode,           MENU_SELECT_SIZE(state_list),   MENU_TARGET(&state_list) };
MENU_VALUE leftDirMode_value =  { TYPE_SELECT,     0,     0,     MENU_TARGET(&leftDirMode_select), CYCLERMODE_ADDR };
MENU_ITEM leftDirMode_item    = { {"LEFT DIR"}, ITEM_VALUE,  0,        MENU_TARGET(&leftDirMode_value) };
*/
MENU_SELECT shotMode_select = {&shotMode, MENU_SELECT_SIZE(state_list), MENU_TARGET(&state_list)};
MENU_VALUE shotMode_value = {TYPE_SELECT, 0, 0, MENU_TARGET(&shotMode_select), SHOTMODE_ADDR};
MENU_ITEM shotMode_item = 		{{"SHOT OUTPUT"}, ITEM_VALUE, 0, MENU_TARGET(&shotMode_value)};

//MENU_LIST const submenu_list1[] = { &printMode_item, &leftPulMode_item, &leftDirMode_item, &rightPulMode_item, &rightDirMode_item,};
//MENU_ITEM menu_submenu1 =      { {"OUTPUTS->"},  ITEM_MENU,  MENU_SIZE(submenu_list1),  MENU_TARGET(&submenu_list1) };

MENU_VALUE stops_value = {TYPE_ULONG, 0, 0, MENU_TARGET(&stops), STOPS_ADDR};
MENU_ITEM stops_item = 			{{"STOPS[-]"}, ITEM_VALUE, 0, MENU_TARGET(&stops_value)};
MENU_VALUE stops2_value = {TYPE_LONG, 0, 0, MENU_TARGET(&stops2), STOPS2_ADDR};
MENU_ITEM stops2_item = 			{{"STOPS 2[-]"}, ITEM_VALUE, 0, MENU_TARGET(&stops2_value)};

MENU_VALUE l0_value = {TYPE_LONG, 0, 0, MENU_TARGET(&l0), L0_ADDR};
MENU_ITEM l0_item = 			{{"L0[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&l0_value)};
MENU_VALUE r0_value = {TYPE_LONG, 0, 0, MENU_TARGET(&r0),R0_ADDR};
MENU_ITEM r0_item =				{{"R0[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&r0_value)};
MENU_VALUE l1_value = {TYPE_LONG, 0, 0, MENU_TARGET(&l1), L1_ADDR};
MENU_ITEM l1_item = 			{{"L1[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&l1_value) };
MENU_VALUE r1_value = {TYPE_LONG, 0, 0, MENU_TARGET(&r1),R1_ADDR };
MENU_ITEM r1_item =				{{"R1[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&r1_value) };

MENU_VALUE lmin_value = {TYPE_LONG, 0, 0, MENU_TARGET(&lMin), LMIN_ADDR};
MENU_ITEM lmin_item = 			{{"LMIN[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&lmin_value)};
MENU_VALUE rmin_value = {TYPE_LONG, 0, 0, MENU_TARGET(&rMin),RMIN_ADDR};
MENU_ITEM rmin_item =				{{"RMIN[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&rmin_value)};
MENU_VALUE lmax_value = {TYPE_LONG, 0, 0, MENU_TARGET(&lMax), LMAX_ADDR};
MENU_ITEM lmax_item = 			{{"LMAX[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&lmax_value) };
MENU_VALUE rmax_value = {TYPE_LONG, 0, 0, MENU_TARGET(&rMax),RMAX_ADDR };
MENU_ITEM rmax_item =				{{"RMAX[steps]"}, ITEM_VALUE, 0, MENU_TARGET(&rmax_value) };

MENU_VALUE speed_value = {TYPE_UINT, 0, 0, MENU_TARGET(&pulSpeed), SPEED_ADDR};
MENU_ITEM speed_item = 			{{"MAX SPEED[st/s]"}, ITEM_VALUE, 0, MENU_TARGET(&speed_value)};
MENU_VALUE acceleration_value = {TYPE_UINT, 0, 0, MENU_TARGET(&pulAcceleration), ACCELERATION_ADDR};
MENU_ITEM acceleration_item = 	{{"ACCEL[st/s2]"}, ITEM_VALUE, 0, MENU_TARGET(&acceleration_value)};

MENU_VALUE preShotTime_value = {TYPE_UINT, 0, 0, MENU_TARGET(&preShotTime), PRESHOTTIME_ADDR};
MENU_ITEM preShotTime_item = 	{{"PRE SHOT T[ms]"}, ITEM_VALUE, 0, MENU_TARGET(&preShotTime_value)};
MENU_VALUE shotTime_value = {TYPE_UINT, 0, 0, MENU_TARGET(&shotTime), SHOTTIME_ADDR};
MENU_ITEM shotTime_item = 		{{"SHOT TIME[ms]"}, ITEM_VALUE, 0, MENU_TARGET(&shotTime_value)};
MENU_VALUE postShotTime_value = {TYPE_UINT, 0, 0, MENU_TARGET(&postShotTime), POSTSHOTTIME_ADDR};
MENU_ITEM postShotTime_item = 	{{"POST SHOT T[ms]"}, ITEM_VALUE, 0, MENU_TARGET(&postShotTime_value)};

MENU_VALUE rightDirInv_value = {TYPE_BYTE, 1, 0, MENU_TARGET(&rightDirInv), RDIRINV_ADDR};
MENU_ITEM rightDirInv_item = 	{{"RIGHT DIR INV"}, ITEM_VALUE, 0, MENU_TARGET(&rightDirInv_value)};
MENU_VALUE leftDirInv_value = {TYPE_BYTE, 1, 0, MENU_TARGET(&leftDirInv), LDIRINV_ADDR};
MENU_ITEM leftDirInv_item  = 	{{"LEFT DIR INV"}, ITEM_VALUE, 0, MENU_TARGET(&leftDirInv_value)};
MENU_VALUE shotInv_value = {TYPE_BYTE, 1, 0, MENU_TARGET(&shotInv), SHOTINV_ADDR};
MENU_ITEM shotInv_item = 		{{"SHOT INVERSION"}, ITEM_VALUE, 0, MENU_TARGET(&shotInv_value) };

MENU_VALUE leftEnaInv_value = {TYPE_BYTE, 1, 0, MENU_TARGET(&leftEnaInv), LENAINV_ADDR};
MENU_ITEM leftEnaInv_item = 	{{"LEFT ENA INV"}, ITEM_VALUE, 0, MENU_TARGET(&leftEnaInv_value) };
MENU_VALUE rightEnaInv_value = {TYPE_BYTE, 1, 0, MENU_TARGET(&rightEnaInv), RENAINV_ADDR};
MENU_ITEM rightEnaInv_item = 	{{"RIGHT ENA INV"}, ITEM_VALUE, 0, MENU_TARGET(&rightEnaInv_value) };

MENU_VALUE lscale_value = {TYPE_LONG, 0, 0, MENU_TARGET(&lScale), LSCALE_ADDR};
MENU_ITEM lscale_item = 		{{"LSCALE[st/deg]"}, ITEM_VALUE, 0, MENU_TARGET(&lscale_value)};
MENU_VALUE rscale_value = {TYPE_LONG, 0, 0, MENU_TARGET(&rScale),RSCALE_ADDR};
MENU_ITEM rscale_item =			{{"RSCALE[st/deg]"}, ITEM_VALUE, 0, MENU_TARGET(&rscale_value)};

/*
MENU_VALUE langle_value = {TYPE_LONG, 0, 0, MENU_TARGET(&lAngle), LANGLE_ADDR};
MENU_ITEM langle_item = 		{{"LANGEL[deg]"}, ITEM_VALUE, 0, MENU_TARGET(&langle_value) };
MENU_VALUE rangle_value = {TYPE_LONG, 0, 0, MENU_TARGET(&rAngle),RANGLE_ADDR };
MENU_ITEM rangle_item =			{{"RANGEL[deg]"}, ITEM_VALUE, 0, MENU_TARGET(&rangle_value) };
MENU_VALUE overlayMode_value = {TYPE_UINT, 0, 0, MENU_TARGET(&overlayMode), OVERLAYMODE_ADDR};
MENU_ITEM overlayMode_item = 	{{"OVERLAY MODE"}, ITEM_VALUE, 0, MENU_TARGET(&overlayMode_value)};
*/

MENU_ITEM item_reset = 			{{"RESET DEFAULTS!"}, ITEM_ACTION, 0, MENU_TARGET(&uiResetAction) };
//MENU_ITEM item_info   = { {"INFO->"},  ITEM_ACTION, 0,        MENU_TARGET(&uiInfo) };

MENU_LIST const submenu_list5[] = {&stops_item, &stops2_item, &speed_item, &acceleration_item, &l0_item, &r0_item, &l1_item, &r1_item, &preShotTime_item, &shotTime_item, &postShotTime_item, &leftDirInv_item, &leftEnaInv_item, &rightDirInv_item, &rightEnaInv_item, &shotInv_item, &shotMode_item, &lmin_item, &rmin_item, &lmax_item, &rmax_item, &lscale_item, &rscale_item, &item_reset}; //&langle_item, &rangle_item, &overlayMode_item,
MENU_ITEM menu_submenu5 = 		{{"SETTINGS->"}, ITEM_MENU, MENU_SIZE(submenu_list5), MENU_TARGET(&submenu_list5)};

MENU_ITEM item_control = 		{{"CALIBRATION"}, ITEM_ACTION, 0, MENU_TARGET(&uiControl)};

MENU_ITEM movePause_control = 	{{"MOVE PAUSE!"}, ITEM_ACTION, 0, MENU_TARGET(&uiMovePause)};
MENU_ITEM moveStart_control = 	{{"MOVE START!"}, ITEM_ACTION, 0, MENU_TARGET(&uiMoveStart)};
MENU_ITEM moveStop_control = 	{{"MOVE STOP!"}, ITEM_ACTION, 0, MENU_TARGET(&uiMoveStop)};

MENU_ITEM gotoPosLR0_control = 	{{"GOTO START POS!"}, ITEM_ACTION, 0, MENU_TARGET(&uiGotoPosLR0)};
MENU_ITEM gotoPosLR1_control = 	{{"GOTO FINAL POS!"},	ITEM_ACTION, 0, MENU_TARGET(&uiGotoPosLR1)};
MENU_ITEM gotoPosInit_control = {{"GOTO ZERO POS!"}, ITEM_ACTION, 0,  MENU_TARGET(&uiGotoPosInit)};

//        List of items in menu level
MENU_LIST const root_list[] = {&movePause_control, &moveStart_control, &moveStop_control, &gotoPosInit_control, &item_control, &menu_submenu5, &gotoPosLR0_control, &gotoPosLR1_control};

// Root item is always created last, so we can add all other items to it
MENU_ITEM menu_root = {{"ROOT"}, ITEM_MENU, MENU_SIZE(root_list), MENU_TARGET(&root_list)};

OMMenuMgr2 Menu(&menu_root, MENU_DIGITAL, &kpd);

void serialPrintParInt(int address)
{
	int val;
	OMEEPROM::read(address, val);
	Serial.print(val);
	Serial.println();
	Serial.println();
}
void serialPrintParFloat(int address)
{
	float val;
	OMEEPROM::read(address, val);
	Serial.println(val);
	Serial.println();
	Serial.println();
}

void loadEEPROM() {
    using namespace OMEEPROM;
    read(SHOTMODE_ADDR, shotMode);
    read(SPEED_ADDR, pulSpeed);
    read(ACCELERATION_ADDR, pulAcceleration);
    read(STOPS_ADDR, stops);
    read(PRESHOTTIME_ADDR, preShotTime);
    read(SHOTTIME_ADDR, shotTime);
    read(POSTSHOTTIME_ADDR, postShotTime);
    read(L0_ADDR, l0);
    read(R0_ADDR, r0);
    read(L1_ADDR, l1);
    read(R1_ADDR, r1);
    read(LDIRINV_ADDR, leftDirInv);
    read(RDIRINV_ADDR, rightDirInv);
    read(SHOTINV_ADDR, shotInv);
    read(LMIN_ADDR, lMin);
    read(RMIN_ADDR, rMin);
    read(LMAX_ADDR, lMax);
    read(RMAX_ADDR, rMax);
    read(LSCALE_ADDR, lScale);
    read(RSCALE_ADDR, rScale);
    //read(LANGLE_ADDR, lAngle);
    //read(RANGLE_ADDR, rAngle);
    read(LENAINV_ADDR, leftEnaInv);
    read(RENAINV_ADDR, rightEnaInv);
    //read(OVERLAYMODE_ADDR, overlayMode);
    read(STOPS2_ADDR, stops2);
}

void saveDefaultEEPROM() {
	shotMode = 0;
	pulSpeed = 4000;
	pulAcceleration = 100;
	stops = 2;
	preShotTime = 3000;
	shotTime = 500;
	postShotTime = 1500;
	l0 = 1000;
	r0 = 1000;
	l1 = -1000;
	r1 = -1000;
	leftDirInv = 0;
	rightDirInv = 1;
	shotInv = 0;
	lMin = -10000;
	rMin = -10000;
	lMax = 10000;
	rMax = 10000;
	lScale = 10;
	rScale = 10;
	//lAngle = 30;
	//rAngle = 30;
	//overlayMode = 1;
	stops2 = 0;

    using namespace OMEEPROM;
    write(SHOTMODE_ADDR, shotMode);
    write(SPEED_ADDR, pulSpeed);
    write(ACCELERATION_ADDR, pulAcceleration);
    write(STOPS_ADDR, stops);
    write(PRESHOTTIME_ADDR, preShotTime);
    write(SHOTTIME_ADDR, shotTime);
    write(POSTSHOTTIME_ADDR, postShotTime);
    write(L0_ADDR, l0);
    write(R0_ADDR, r0);
    write(L1_ADDR, l1);
    write(R1_ADDR, r1);
    write(LDIRINV_ADDR, leftDirInv);
    write(RDIRINV_ADDR, rightDirInv);
    write(SHOTINV_ADDR, shotInv);
    write(LMIN_ADDR, lMin);
    write(RMIN_ADDR, rMin);
    write(LMAX_ADDR, lMax);
    write(RMAX_ADDR, rMax);
    write(LSCALE_ADDR, lScale);
    write(RSCALE_ADDR, rScale);
    //write(LANGLE_ADDR, lAngle);
    //write(RANGLE_ADDR, rAngle);
    write(LENAINV_ADDR, leftEnaInv);
    write(RENAINV_ADDR, rightEnaInv);
    //write(OVERLAYMODE_ADDR, overlayMode);
    write(STOPS2_ADDR, stops2);
}

void setSteppers() {
	stepperL.setPinsInverted(leftDirInv);
	stepperR.setPinsInverted(rightDirInv);

    stepperL.setMaxSpeed(pulSpeed);
    stepperL.setAcceleration(pulAcceleration);
    stepperR.setMaxSpeed(pulSpeed);
    stepperR.setAcceleration(pulAcceleration);
}

//////////////////////////////////
// setup
//////////////////////////////////

void setup() {

	//digitalWrite(SHOT_PIN, false);
	//pinMode(SHOT_PIN, OUTPUT);
	//digitalWrite(SHOT_PIN, true);
	pinMode(LED_PIN, OUTPUT);
	pinMode(SHOT_PIN, OUTPUT);
	pinMode(PRESHOT_PIN, OUTPUT);
	pinMode(L_ENA_PIN, OUTPUT);
	pinMode(R_ENA_PIN, OUTPUT);

	if( OMEEPROM::saved() )
		loadEEPROM();
	else
		saveDefaultEEPROM();
	//digitalWrite(SHOT_PIN, !shotInv);

	Serial.begin(57600);
	//Serial.begin(115200);
	while(!Serial);

	//Serial.println("POLARGRAPH ON!");
	//Serial.print("Hardware: ");
	//Serial.println(MICROCONTROLLER);
	//Serial.println("MC_MEGA");
	//Serial.println(READY_STR);

	//Serial1.begin(115200);
	//Serial1.begin(9600);
	//while(!Serial1);

	Serial.setTimeout(20);
	//Serial1.setTimeout(20);

	Serial.println(TEXT_ID0);
	Serial.println(TEXT_ID1);
	//Serial1.println(TEXT_ID0);
	//Serial1.println(TEXT_ID1);

	//setup2();
	Wire.begin( );
	kpd.begin( makeKeymap(keys) );
	lcd.begin(LCD_COLS, LCD_ROWS);

	lcd.print(TEXT_ID0);
	lcd.setCursor(0, 1);
	lcd.print(TEXT_ID1);

	//pulSpeed= max(200, pulSpeed);
	//Timer3.initialize(pulSpeed); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
	//Timer3.attachInterrupt( timerIsr ); // attach the service routine here

	//Timer1.setPeriod();

	//TODO: refresh display
	//uiMain();
	Menu.setDrawHandler(uiDraw);
	Menu.setExitHandler(uiMain);
	Menu.enable(true);

	//pinMode(R_PUL_PIN, OUTPUT);
	//pinMode(R_DIR_PIN, OUTPUT);
	//pinMode(L_PUL_PIN, OUTPUT);
	//pinMode(L_DIR_PIN, OUTPUT);
	setSteppers();

	//wdt_enable(WDTO_8S);
}

bool getInstrumentControl(bool a, byte mode) {
	mode = mode & 3;
	if(mode == 0) return a;
	if(mode == 1) return false;
	if(mode == 2) return true;
	return false;
}

/*
double analogRead(int pin, int samples){
  int result = 0;
  for(int i=0; i<samples; i++){
    result += analogRead(pin);
  }
  return (double)(result / samples);
}
*/

void setPosInit() {
	//leftPuls = 0;
	//rightPuls = 0;
	//leftPulsPos = 0;
	//rightPulsPos = 0;
	stepperL.setCurrentPosition(0);
	stepperR.setCurrentPosition(0);
}

void setPosLR0() {
	//l0 = leftPulsPos;
	//r0 = rightPulsPos;
	l0 = stepperL.currentPosition();
	r0 = stepperR.currentPosition();
	OMEEPROM::write(L0_ADDR, l0);
	OMEEPROM::write(R0_ADDR, r0);
}

void setPosLR1() {
	//l1 = leftPulsPos;
	//r1 = rightPulsPos;
	l1 = stepperL.currentPosition();
	r1 = stepperR.currentPosition();
	OMEEPROM::write(L1_ADDR, l1);
	OMEEPROM::write(R1_ADDR, r1);
}

void gotoPosLR(long l, long r) {
	//TODO: directions
	//leftDirAuto = vp.getDirection(l, vp.getLength(leftPulsPos));
	//rightDirAuto = vp.getDirection(r, vp.getLength(rightPulsPos));

	//leftAccSkip = pulAcceleration;
	//rightAccSkip = pulAcceleration;
	//leftPuls = l - leftPulsPos;
	//rightPuls = r - rightPulsPos;

	//leftGo(l - leftPulsPos > 0, l - leftPulsPos);
	//rightGo(r - rightPulsPos > 0, r - rightPulsPos);
	stepperL.moveTo(l);
	stepperR.moveTo(r);

	/*
	Serial.print(vp.getLength(leftPulsPos));
	Serial.print('\t');
	//Serial.print(l - vp.getLength(leftPulsPos));
	//Serial.print('\t');
	Serial.print(vp.getDirection(l, vp.getLength(leftPulsPos)));
	Serial.print('\t');
	Serial.print(vp.stepDistance);
	Serial.print('\t');
	Serial.println(leftPuls);
	 */
}

void gotoPosInit() {
	gotoPosLR(0, 0);
}


//////////////////////////////////
// main loop
//////////////////////////////////

void loop() {
	//wdt_reset();

	/*
	if(stepperL.currentPosition() >= lMax || stepperL.currentPosition() <= lMin) {
		stepperL.setAcceleration(100000000.0);
		stepperL.stop();
		while(stepperL.runSpeed());
		stepperL.setAcceleration(pulAcceleration);
	}
	if(stepperR.currentPosition() >= rMax || stepperR.currentPosition() >= rMin) {
		stepperR.setAcceleration(100000000.0);
		stepperR.stop();
		while(stepperR.runSpeed());
		stepperR.setAcceleration(pulAcceleration);
	}
	*/

	steppersRun();

	if(millis() - lastMillis > 1000) {
		lastMillis = millis();
		secondsCounter++;
		secToggle ? secToggle = false : secToggle = true;

		//if(!stepperL.run() && !stepperR.run()) {
		//		Serial.println(READY_STR);
		//}

		//TODO:
		setSteppers();
	}

	if (!Menu.shown()) {
		if(!uiState) {
			//TODO: refresh display
			//uiMain();
		}
	}

	char key = kpd.getKey2();
	if(key == '#') {
		uiState = 0;
		uiPage = 0;
		Menu.enable(true);
		if(!Menu.shown()) {
			uiMain();
		}
	}
	else if(!Menu.shown() || !Menu.enable()) {
		if(key == '8') {
			uiControl();
		}
		else if(key == '9') {
			uiInfo();
		}
		else {
			uiScreen();
		}
	}
	Menu.checkInput();

	if(state == STATE_RUNNING) {
		if(stopsCounter == 0 && stopsCounter2 == 0) {
			gotoPosLR(l0, r0);
		}
		if(!stepperL.isRunning() && !stepperR.isRunning()) {
			if(shotState == 0) {
				preShotMillis = millis();
				shotState = 1;

				Serial.print("L: ");
				Serial.println(stepperL.currentPosition());
				Serial.print("R: ");
				Serial.println(stepperR.currentPosition());
				Serial.println("preShot");
			}
			if(millis() - preShotMillis >= preShotTime) {
				if(shotState == 1) {
					shotMillis = millis();
					shotState = 2;

					Serial.print("shot: ");
					Serial.print(stopsCounter);
					Serial.print(" x ");
					Serial.println(stopsCounter2);
				}
				if(millis() - shotMillis >= shotTime) {
					if(shotState == 2) {
						postShotMillis = millis();
						shotState = 3;

						Serial.println("postShot");
					}
					if(millis() - postShotMillis >= postShotTime) {
						shotState = 0;

						Serial.println("nextShot");
						//Serial.println(stopsCounter);
						long nextLeftStop, nextRightStop;

						//if(overlayMode == 0) {
						if(abs(stops2) < 1) {
							stopsCounter++;
							//if(stopsCounter > stops + 1) {
							if(stopsCounter >= stops) {
								state = STATE_DONE;
								stopsCounter = 0;
								nextLeftStop = 0;
								nextRightStop = 0;
							}
							else {
								nextLeftStop = ((l1 - l0) / (stops - 1)) * stopsCounter + l0;
								nextRightStop = ((r1 - r0) / (stops - 1)) * stopsCounter + r0;
							}
						}

						//if(overlayMode == 1) {
						if(abs(stops2) > 0) {
							stopsCounter++;

							//if(stopsCounter > stops + 1) {
							if(stopsCounter >= stops) {
									stopsCounter = 0;
									stopsCounter2++;
							}
							if(stops2 > 0 ) {
								nextLeftStop = ((l1 - l0) / (stops - 1)) * stopsCounter + l0;
								nextRightStop = ((r1 - r0) / (stops2 - 1)) * stopsCounter2 + r0;
							}
							else {
								nextLeftStop = ((l1 - l0) / (-stops2 - 1)) * stopsCounter2 + l0;
								nextRightStop = ((r1 - r0) / (stops - 1)) * stopsCounter + r0;
							}
						}

						//if(stopsCounter2 > stops2 + 1) {
						if(stopsCounter2 >= abs(stops2)) {
							state = STATE_DONE;
							stopsCounter = 0;
							stopsCounter2 = 0;
							nextLeftStop = 0;
							nextRightStop = 0;
						}
/*
						if(stops2 < -1) {
							stopsCounter++;

							if(stopsCounter > stops) {
									stopsCounter = 0;
									stopsCounter2++;
							}
							nextLeftStop = ((l1 - l0) / (-stops2 - 1)) * stopsCounter2 + l0;
							nextRightStop = ((r1 - r0) / (stops - 1)) * stopsCounter + r0;
						}

						//if(stopsCounter2 > stops2 + 1) {
						if(stopsCounter2 > abs(stops2)) {
							state = STATE_DONE;
							stopsCounter = 0;
							stopsCounter2 = 0;
							nextLeftStop = 0;
							nextRightStop = 0;
						}
*/
						//Serial.print("leftStep: ");
						//Serial.println((l1 - l0) / (stops + 1));
						//Serial.print("leftOffset: ");
						//Serial.println(l0);
						//Serial.print("rightStep: ");
						//Serial.println((r1 - r0) / (stops2 + 1));
						//Serial.print("rightOffset: ");
						//Serial.println(r0);

						//Serial.println(stops);
						gotoPosLR(nextLeftStop, nextRightStop);
						Serial.print("nextLeftShot: ");
						Serial.println(nextLeftStop);
						Serial.print("nextRightShot: ");
						Serial.println(nextRightStop);
					}
				}
			}
		}
	}
	else {
		shotState = 0;
	}
	preShotAuto = shotState == 1;
	shotAuto = shotState == 2;

	//////////////////////////////////
	// outputs
	//////////////////////////////////

	preShotControl = getInstrumentControl(preShotAuto, preShotMode);
	digitalWrite(PRESHOT_PIN, preShotInv? preShotControl : !preShotControl);

	shotControl = getInstrumentControl(shotAuto, shotMode);
	digitalWrite(SHOT_PIN, shotInv? shotControl : !shotControl);

	/*
	//TODO: Interrupt driven
	//if(cycles >= cyclesPrev + pulSpeed) {
	if(microseconds >= microsecondsPrev + pulSpeed) {
		//cyclesPrev = cycles;
		microsecondsPrev = microseconds;
		if(rightPuls) {
			rightPuls--;
			rightPulAuto = !rightPulAuto;

			if(rightDirAuto)
				rightPulsPos++;
			else
				rightPulsPos--;
		}
		if(leftPuls) {
			leftPuls--;
			leftPulAuto = !leftPulAuto;

			if(leftDirAuto)
				leftPulsPos++;
			else
				leftPulsPos--;
		}

		if(printDuration) {
			printDuration--;
			printAuto = true;
		}
		else
			printAuto = false;

	}
	//cycles++;
	*/


	/*
	printControl = getInstrumentControl(printAuto, printMode);
	rightPulControl = getInstrumentControl(rightPulAuto, rightPulMode);
    rightDirControl = getInstrumentControl(rightDirAuto, rightDirMode);
	leftPulControl = getInstrumentControl(leftPulAuto, leftPulMode);
	leftDirControl = getInstrumentControl(leftDirAuto, leftDirMode);
	*/
	/*
	if(kpd.getRawKey()) {
		// key pressed
	}
	*/

	/*
	//digitalWrite(LED_PIN, !printControl);
	digitalWrite(PRINT_PIN, !printControl);
	digitalWrite(R_DIR_PIN, !rightDirControl);
	digitalWrite(R_PUL_PIN, !rightPulControl);
	digitalWrite(L_DIR_PIN, !leftDirControl);
	digitalWrite(L_PUL_PIN, !leftPulControl);
	 */


	/*
	pinMode(LED_PIN, OUTPUT);
	pinMode(PRINT_PIN, OUTPUT);
	pinMode(R_PUL_PIN, OUTPUT);
	pinMode(R_DIR_PIN, OUTPUT);
	pinMode(L_PUL_PIN, OUTPUT);
	pinMode(L_DIR_PIN, OUTPUT);
	*/

	//////////////////////////////////
	// communication
	//////////////////////////////////

	String text;
  	if (Serial.available() > 0) {
  		text = Serial.readString();
  	}
  	//if (Serial1.available() > 0)  {
  	//  	text = Serial1.readString();
  	//}

  	if(text) {
  		int pos;
  		//Serial.println(text);
  		pos = text.indexOf(MESSAGE_CMD_PARREADINT);
  		if (pos >= 0) {
  			serialPrintParInt(text.substring(pos + strlen(MESSAGE_CMD_PARREADFLOAT)).toInt());
  		}
  		pos = text.indexOf(MESSAGE_CMD_PARREADFLOAT);
  		if (pos >= 0) {
  			serialPrintParFloat(text.substring(pos + strlen(MESSAGE_CMD_PARREADFLOAT)).toFloat());
  		}
  		pos = text.indexOf(MESSAGE_CMD_PARWRITEINT);
  		if (pos >= 0) {
  			int address = text.substring(pos + strlen(MESSAGE_CMD_PARWRITEINT)).toInt();
  			//#PWI0125:25
  			int value = text.substring(pos + strlen(MESSAGE_CMD_PARWRITEINT) + 5).toInt();
  			OMEEPROM::write(address, value);
  		}
  		pos = text.indexOf(MESSAGE_CMD_PARWRITEFLOAT);
  		if (pos >= 0) {
  			int address = text.substring(pos + strlen(MESSAGE_CMD_PARWRITEINT)).toInt();
  			//#PWI0125:25
  			float value = text.substring(pos + strlen(MESSAGE_CMD_PARWRITEINT) + 5).toFloat();
  			OMEEPROM::write(address, value);
  		}
  		pos = text.indexOf(MESSAGE_CMD_PARRELOAD);
  		if (pos >= 0) {
  			loadEEPROM();
  		}

   		pos = text.indexOf(MESSAGE_CMD_SETZ);
		if (pos >= 0) {
			zComm = text.substring(pos + strlen(MESSAGE_CMD_SETZ)).toInt();
		}
		pos = text.indexOf(MESSAGE_CMD_SETL);
		if (pos >= 0) {
			lComm = (unsigned long)text.substring(pos + strlen(MESSAGE_CMD_SETL)).toInt();
		}
		pos = text.indexOf(MESSAGE_CMD_SETR);
		if (pos >= 0) {
			rComm = (unsigned long)text.substring(pos + strlen(MESSAGE_CMD_SETR)).toInt();
		}
		pos = text.indexOf(MESSAGE_CMD_SCROLL);
		if (pos >= 0) {
			shotMode = zComm;
			gotoPosLR(lComm, rComm);
			Serial.println(rComm);
			Serial.println(lComm);
			Serial.println(zComm);
		}
		if (text.indexOf(MESSAGE_CMD_REQUEST)!=-1 ) {
			//Serial.println(max(leftPuls, rightPuls));
			Serial.println(max(stepperL.distanceToGo(), stepperR.distanceToGo()));
			//Serial1.println(max(leftPuls, rightPuls));
  			//Serial.println();
  			//Serial.println();
   		}
	}
}

void uiOK(){
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("OK"));
	//delay(OK_DELAY);
	delayWithSteppersRun(OK_DELAY);
}

void uiResetAction() {
	saveDefaultEEPROM();
	uiOK();
}

void uiDraw(char* p_text, int p_row, int p_col, int len) {
	lcd.backlight();
	lcd.setCursor(p_col, p_row);
	for( int i = 0; i < len; i++ ) {
		if( p_text[i] < '!' || p_text[i] > '~' )
			lcd.write(' ');
		else
			lcd.write(p_text[i]);
	}
}

void uiInstrument(bool instrument, byte mode) {
	lcd.print(instrument);
	if(mode == 0)
		lcd.print('A');
	else if(mode < 3)
		lcd.print('M');
	else
		lcd.print('X');
}

void uiMovePause() {
	uiOK();
	state = STATE_PAUSED;
	return;
	//Menu.enable(false);
	//uiState = UISTATE_INFO;
	//uiPage = 0;
	//uiKeyMillis = 0;
	//uiKeyPressed = 0;
	//state = STATE_PAUSED;
}

void uiMoveStop() {
	uiOK();
	shotAuto = false;
	stopsCounter = 0;
	stopsCounter2 = 0;
	state = STATE_STOPPED;
	return;
	//Menu.enable(false);
	//uiState = UISTATE_INFO;
	//uiPage = 0;
	//uiKeyMillis = 0;
	//uiKeyPressed = 0;
}

void uiMoveStart() {
	uiOK();
	state = STATE_RUNNING;
	return;
	//Menu.enable(false);
	//uiPage = 0;
	//uiKeyMillis = 0;
	//uiKeyPressed = 0;
	//uiState = UISTATE_INFO;
	//uiPage = 0;
	//state = STATE_RUNNING;
}

void uiControl() {
	lcd.backlight();
	Menu.enable(false);
	uiState = UISTATE_CONTROL;
	uiPage = 1;
	uiKeyMillis = 0;
	uiKeyPressed = 0;

	lcd.clear();
	lcd.setCursor(0, 0);
			  //"0123456789ABCDEF"
	lcd.print(F("Z[C] S[A] F[B] #ESC"));
	lcd.setCursor(0, 1);
			  //"0123456789ABCDEF"
	lcd.print(F("^2-5_ <4-6>  0-S"));
}

void uiInfo() {
	lcd.backlight();
	Menu.enable(false);
	uiState = UISTATE_INFO;
	uiPage=0;
	uiKeyMillis = 0;
	uiKeyPressed = 0;
	lcd.clear();
}

void uiGotoPosLR0() {
	uiOK();
	gotoPosLR(l0, r0);
}

void uiGotoPosLR1() {
	uiOK();
	gotoPosLR(l1, r1);
}

void uiGotoPosInit() {
	uiOK();
	gotoPosInit();
}

void uiScreen() {
	//Menu.enable(false);
	//lcd.backlight();

	char key = kpd.getRawKey();
	// First key stroke after delay, then pause and then continuously
	if(key) {
		 if(!uiKeyMillis) {
			 uiKeyMillis = millis();
		 }
		 if((uiKeyMillis + 120) > millis()) {
			 key = 0;
		 }
		 else {
			 if(uiKeyPressed) {
				 key = 0;
			 }
			 else {
				 uiKeyPressed = key;
			 }
		 }
		 if((uiKeyMillis + 600) < millis()){
			 uiKeyPressed = 0;
		 }
	}
	else {
		uiKeyMillis = 0;
		uiKeyPressed = 0;
	}

	if(uiState == UISTATE_CONTROL) {
		//if(key == KPD_ENTER) {
		if(key == 'C') {
			uiOK();
			setPosInit();
			uiState = UISTATE_MAIN;
			Menu.enable(true);
		}
		if(key == 'A') {
			uiOK();
			setPosLR0();
			uiState = UISTATE_MAIN;
			Menu.enable(true);
		}
		if(key == 'B') {
			uiOK();
			setPosLR1();
			uiState = UISTATE_MAIN;
			Menu.enable(true);
		}
	}

	if(uiState == UISTATE_INFO) {
		if(key == KPD_UP) {
			uiPage--;
		}
		if(key == KPD_DOWN) {
			uiPage++;
		}
		uiPage = max(0, uiPage);
		uiPage = min(5, uiPage);

		if(uiPage == 0) {
			//TODO: once per ?
			lcd.setCursor(0, 0);
			if(state == STATE_STOPPED ) {
			    lcd.print(F("MOVING STOPPED"));
			    uiLcdPrintSpaces8();
			    lcd.setCursor(0, 1);
			    uiLcdPrintSpaces8();
			    uiLcdPrintSpaces8();
			}
			else if(state == STATE_RUNNING || state == STATE_PAUSED) {
				if(state == STATE_PAUSED )
				    lcd.print(F("MOVING PAUSED"));
				else
					lcd.print(F("MOVING RUNNING"));
			    uiLcdPrintSpaces8();
			    lcd.setCursor(0, 1);
				lcd.print(stopsCounter);
				lcd.print('/');
				lcd.print(stops);
			    uiLcdPrintSpaces8();
			    lcd.setCursor(8, 1);
				lcd.print(stopsCounter2);
				lcd.print('/');
				lcd.print(stops2);
			    uiLcdPrintSpaces8();
			}
			else if(state == STATE_DONE ) {
			    lcd.print(F("MOVING DONE"));
			    uiLcdPrintSpaces8();
			    lcd.setCursor(0, 1);
			    uiLcdPrintSpaces8();
			    uiLcdPrintSpaces8();
			}
		}
		if(uiPage == 1) {
			lcd.setCursor(0, 0);
			//lcd.print(leftPuls);
			lcd.print(F("L:"));
			lcd.print(stepperL.currentPosition());
		    uiLcdPrintSpaces8();
		    //lcd.setCursor(10, 0);
		    //lcd.print(F("/"));
		    //lcd.print(rightPuls);
		    //lcd.print(stepperL.distanceToGo());
		    //uiLcdPrintSpaces8();
		    uiLcdPrintSpaces8();
		    lcd.setCursor(0, 1);
		    //lcd.print(leftPulsPos);
		    lcd.print(F("R:"));
		    lcd.print(stepperR.currentPosition());
		    uiLcdPrintSpaces8();
		    //lcd.setCursor(10, 1);
		    //lcd.print(F("/"));
		    //lcd.print(rightPulsPos);
		    //lcd.print(stepperR.distanceToGo());
		    //uiLcdPrintSpaces8();
		    uiLcdPrintSpaces8();
		}
		if(uiPage == 2) {
			lcd.setCursor(0, 0);
			//lcd.print(leftPuls);
			lcd.print(F("L[deg]:"));
			lcd.print(stepperL.currentPosition() / lScale);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			//lcd.print(leftPulsPos);
			lcd.print(F("R[deg]:"));
			lcd.print(stepperR.currentPosition() / rScale);
			uiLcdPrintSpaces8();
			//lcd.setCursor(10, 1);
			//lcd.print(F("/"));
			//lcd.print(rightPulsPos);
			//lcd.print(stepperR.distanceToGo());
			//uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();

			//Serial.println(stepperL.currentPosition());
			//Serial.println(lScale);
			//Serial.println(stepperL.currentPosition() / lScale);
			//Serial.println();

		}
		if(uiPage == 3) {
			if(false) {
				secToggle ? lcd.backlight() : lcd.noBacklight();
			}
			else {
				lcd.backlight();
			}

			lcd.setCursor(0, 0);
			lcd.print("SHOT");
			uiInstrument(shotControl, shotMode);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();

			lcd.setCursor(0, 1);
			lcd.print("LP");
			uiInstrument(leftPulControl, leftPulMode);
			lcd.print("LD");
			uiInstrument(leftDirControl, leftDirMode);
			lcd.print("RP");
			uiInstrument(rightPulControl, rightPulMode);
			lcd.print("RD");
			uiInstrument(rightDirControl, rightDirMode);
		}
		if(uiPage == 4) {
		    lcd.setCursor(0, 0);
			lcd.print(F("L0[st]:"));
			lcd.print(l0);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			lcd.print(F("R0[st]:"));
			lcd.print(r0);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
		}
		if(uiPage == 5) {
			lcd.setCursor(0, 0);
			lcd.print(F("L1[st]:"));
			lcd.print(l1);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			lcd.print(F("R1[st]:"));
			lcd.print(r1);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
		}
		/*
		if(uiPage == 6) {
		    lcd.setCursor(0, 0);
			lcd.print(F("SPEED[st/s]:"));
			lcd.print(pulSpeed);
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			lcd.print(F("ACCEL[st/s2]:"));
			lcd.print(pulAcceleration);
			uiLcdPrintSpaces8();
		}
		if(uiPage == 7) {
		    lcd.setCursor(0, 0);
			lcd.print(F("PRE[ms]:"));
			lcd.print(preShotTime);
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			lcd.print(F("POST[ms]:"));
			lcd.print(postShotTime);
			uiLcdPrintSpaces8();
		}
		if(uiPage == 8) {
		    lcd.setCursor(0, 0);
			lcd.print(F("SHOT[ms]:"));
			lcd.print(shotTime);
			uiLcdPrintSpaces8();
			lcd.setCursor(0, 1);
			uiLcdPrintSpaces8();
			uiLcdPrintSpaces8();
		}
		*/
	}
	/*
	if(uiState == UISTATE_EDITTEXT) {

		lcd.setCursor(0, 0);
		//"0123456789ABCDEF"
		if(key == 'C')
			uiPage++;
		if(key == 'D')
			uiPage--;
		uiPage = max(0, uiPage);
		uiPage = min(15, uiPage);
		//text[0] = 64;
		uint8_t i;
		//strncpy(text2, text, 16);
		i = tmp_text[uiPage];
		if(key == KPD_UP) i++;
		if(key == KPD_DOWN) i--;
		i = max(32, i);
		i = min(126, i);
		tmp_text[uiPage] = (char)i;
		lcd.setCursor(0, 1);
		lcd.print(tmp_text);
		uiLcdPrintSpaces8();
		uiLcdPrintSpaces8();
		if(secToggle) {
			lcd.setCursor(uiPage, 1);
			lcd.print('_');
		}
		if(key == KPD_ENTER) {
			tmp_text[uiPage+1] = 0;
			for(int i=uiPage+2; i<16; i++)
				tmp_text[i]=255;

			//TODO: 15 or 16 chars?
			tmp_text[16] = 0;
			//Serial.println(tmp_text);
			strncpy(text, tmp_text, 16);

			Menu.enable(true);
		}
	}
	*/
}

void uiLcdPrintSpaces8() {
	lcd.print(F("        "));
	steppersRun();
}

void uiMain() {
	//TODO:
	//if(pulSpeed != pulSpeedPrev) {
	//	pulSpeed= max(100, pulSpeed);
	//	pulSpeedPrev = pulSpeed;
	//	Timer3.setPeriod(pulSpeed);
	//}

	//lcd.noBacklight();
	lcd.clear();
	lcd.print(TEXT_ID0);
	lcd.setCursor(0, 1);
	lcd.print(TEXT_ID1);
}

/*
//void leftGo(bool dir, unsigned int puls) {
void leftGo(bool dir, long puls) {
	//leftDirAuto = dir;
	//leftPuls = puls;
	//leftAccSkip = 0;//STEPS_ACC;
	if(!dir)
		stepperL.move(-puls);
	else
		stepperL.move(puls);
}

//void rightGo(bool dir, unsigned int puls) {
void rightGo(bool dir, long puls) {
	//rightDirAuto = dir;
	//rightPuls = puls;
	//rightAccSkip = 0;//STEPS_ACC;
	if(!dir)
		stepperR.move(-puls);
	else
		stepperR.move(puls);
}

void shotGo(unsigned int duration) {

	shotMode = 2;

	//shotDuration = duration;
	//millisPrintPrev = millis();
}
*/

/*
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------

void timerIsr()
{
	if(stop) {
		digitalWrite(SHOT_PIN, !shotAuto);
		return;
	}

	//if(shotting)
	//	return;

	//rightPulAuto ^= bool(rightPuls);
	//rightPuls -= bool(rightPuls);
	//leftPulAuto ^= bool(leftPuls);
	//leftPuls -= bool(leftPuls);

	if(rightAcc < rightAccSkip) {
		rightAcc++;
	}
	else {
		rightAcc = 0;
		rightAccSkip = max(0, rightAccSkip - 1);
		//Serial.print("rightAcc: ");
		//Serial.println(rightAcc);
		//Serial.print("rightAccSkip: ");
		//Serial.println(rightAccSkip);
		rightPulAuto ^= bool(rightPuls);
		rightPuls -= bool(rightPuls);
		if(rightPuls) {
			if(rightDirAuto)
				rightPulsPos++;
			else
				rightPulsPos--;
		}
	}

	if(leftAcc < leftAccSkip) {
		leftAcc++;
	}
	else {
		leftAcc = 0;
		leftAccSkip = max(0, leftAccSkip - 1);
		leftPulAuto ^= bool(leftPuls);
		leftPuls -= bool(leftPuls);
		if(leftPuls) {
			if(leftDirAuto)
				leftPulsPos++;
			else
				leftPulsPos--;
		}
	}

	digitalWrite(R_DIR_PIN, rightDirInv? rightDirAuto : !rightDirAuto);
	digitalWrite(R_PUL_PIN, !rightPulAuto);
	digitalWrite(L_DIR_PIN, leftDirInv? leftDirAuto: !leftDirAuto);
	digitalWrite(L_PUL_PIN, !leftPulAuto);

	if(rightPuls < pulAcceleration) {
		rightAccSkip = min(pulAcceleration - rightPuls, pulAcceleration);
	}
	if(leftPuls < pulAcceleration) {
		leftAccSkip = min(pulAcceleration - leftPuls, pulAcceleration);
	}

	shotControl = getInstrumentControl(shotAuto, shotMode);
	digitalWrite(SHOT_PIN, shotInv? shotControl : !shotControl);
}
*/
