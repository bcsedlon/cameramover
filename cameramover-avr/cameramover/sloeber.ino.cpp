#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2019-10-16 14:05:23

#include "Arduino.h"
#include "Arduino.h"
#define TEXT_ID0 "CAMERA MOVER"
#define TEXT_ID1 "GROWMAT.CZ"
#include <AccelStepper.h>
extern const byte KPD_ROWS;
extern const byte KPD_COLS;
#define KPD_I2CADDR 0x20
#define OK_DELAY 500
#define LCD_I2CADDR 0x27
extern const byte LCD_ROWS;
extern const byte LCD_COLS;
#define LED_PIN 	13
#define PRESHOT_PIN	3
#define SHOT_PIN	4
#define R_DIR_PIN	12
#define R_PUL_PIN	11
#define R_ENA_PIN	10
#define L_DIR_PIN	8
#define L_PUL_PIN	7
#define L_ENA_PIN	6
extern AccelStepper stepperL;
extern AccelStepper stepperR;
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
#define STOPS2_ADDR			84
#define LNEXT_ADDR			88
#define RNEXT_ADDR			92
#define LENAINV_ADDR		96
#define RENAINV_ADDR		97
#define PRESHOTINV_ADDR		98
#define MESSAGE_CMD_REQUEST  		"?"
#define MESSAGE_CMD_PARREADINT 		"#PRI"
#define MESSAGE_CMD_PARREADFLOAT 	"#PRF"
#define MESSAGE_CMD_PARWRITEINT 	"#PWI"
#define MESSAGE_CMD_PARWRITEFLOAT 	"#PWF"
#define MESSAGE_CMD_PARRELOAD 		"#PLD"
#define MESSAGE_CMD_SETZ 		"Z"
#define MESSAGE_CMD_SETL 		"L"
#define MESSAGE_CMD_SETR 		"R"
#define MESSAGE_CMD_GO	 		"G"
#define UISTATE_MAIN 		0
#define UISTATE_INFO 		3
#define UISTATE_CONTROL 	5
#define UISTATE_CONTROL2 	6
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
extern byte state;
extern byte shotState;
extern bool stop;
extern unsigned long lastMillis;
extern byte secondsCounter;
extern bool secToggle;
extern char uiKeyPressed;
extern int uiState;
extern unsigned long uiKeyMillis;
extern unsigned long uiScreenMillis;
extern unsigned long preShotMillis;
extern long lComm;
extern long lStops;
extern byte preShotMode;
extern bool preShotControl;
extern bool preShotAuto;
extern bool leftDirInv;
extern unsigned int pulSpeed;
extern long l0;
extern long stops;
#include "LiquidCrystal_I2C.h"
extern LiquidCrystal_I2C lcd;
#include "Keypad_I2C.h"
#include "Keypad.h"
extern byte rowPins[];
extern byte colPins[];

#include "OMEEPROM.h"
#include "OMMenuMgr2.h"

bool stepperRunL() ;
bool stepperRunR() ;
void steppersRun() ;
void delayWithSteppersRun(unsigned long delay) ;
void serialPrintParInt(int address) ;
void serialPrintParFloat(int address) ;
void loadEEPROM() ;
void saveDefaultEEPROM() ;
void setSteppers() ;
void setup() ;
bool getInstrumentControl(bool a, byte mode) ;
void setPosInit() ;
void setPosLR0() ;
void setPosLR1() ;
void gotoPosLR(long l, long r) ;
void calcStops() ;
void loop() ;
void uiOK();
void uiResetAction() ;
void uiDraw(char* p_text, int p_row, int p_col, int len) ;
void uiInstrument(bool instrument, byte mode) ;
void uiMovePause() ;
void uiMoveStop() ;
void uiMoveStart() ;
void uiControl() ;
void uiControl2() ;
void uiInfo() ;
void uiGotoPosLR0() ;
void uiGotoPosLR1() ;
void uiGotoPosInit() ;
void uiScreen() ;
void uiLcdPrintSpaces8() ;
void uiMain() ;

#include "cameramover.ino"


#endif
