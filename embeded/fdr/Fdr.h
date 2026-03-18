/****************************************
Put the Fdr.h version number at line 20.
******************************************


Fdr runs on the Flight Data Recorder hardware and is able to collect up to 4 voltages plus internal temperature for up to 4.55 hours plus it records the battery voltage. It also can run a RunCam6 camera,
GPS, and an Iridium Modem with an added board.

******************************************/

#ifndef Fdr_h
#define Fdr_h

#include "Arduino.h"


class Fdr
{
  public:
  byte Fdr_h_version[3] = {1,0,9};//header version. It will print as Fdr_h_version[0].Fdr_h_version[1].Fdr_h_version[2]
/*************************************************
                 SET UP LIBRARY ENVIRONMENT
**************************************************/
    Fdr(); //this is the Constructor and sets up the environment for other memembers of its class.
	void FDRoneTimeRun();
	bool areWeDoneWithStartUpDiagQ();//used by setup1()
/*************************************************
                 VARIABLES AND CONSTANTS
**************************************************/ 
	void Pointers(intptr_t *PointerArray);//changed int to intptr_t to be consistent with Fdr.cpp 	
/*************************************************
    PUT ALL FDR FUNTIONALITY INTO THE USER'S LOOP
**************************************************/
    void PutInLoop();
    void Port6Peak();
/*************************************************
                 SEEPROM AND EEPROM
***************************************************/
    int WriteSEEPROM(long eeAddress, byte data);
    int ReadSEEPROM(long eeaddress);
	byte ReadEEPROM(long eeaddress);
	bool dataBlockDumpSEEPROM(long startingAddress,byte numberOfBytes);//It returns true for success and false if failure
/*************************************************
				USER CONTROL OF EXTERNAL LED
***************************************************/		void seizeExternalLEDcontrol();
		void releaseExternalLEDcontrol();
		void externalLED(byte intensityByte);//accepts onBool and offBool 				
/***************************************************/				
	//low level public
	//void diagLED(bool stateBool);
	bool enablePort6HighSpeedRead = false;
	const bool onBool = true;
	const bool offBool = false;
	unsigned long Loop1CycleTimeMSULong = 0;
	unsigned int Loop1CycleTimeLimitMS_UInt = 1000;
	unsigned int Loop1CycleTimeMS_ULong = 0;
	void Loop1CycleTimeMonitor(bool verboseQ);//is public so Iridium.ino can reach it
	void WriteControlBlock(byte eeAddressByte, byte data);
	void UnprotectedWriteEEPROM(long eeAddress, byte data);
	byte ReadControlBlock(byte eeAddressByte);
	bool isLoop1TimerActiveQ();
	//void enableLoop1TimerWarning();
	//void UserDefinedControlOfLED(byte patternByte);//a pattern of 0 returns LED control back to the system. All other pattern values control the LED as predefined.
	int ExternalLED = 5;
	byte ExternalLED_OnByte = 255;
	byte ExternalLED_OffByte = 0;
	byte ExternalLED_dimByte = 32;//emperically found
	byte diagnosticLEDpatternByte = 0;
	byte LEDcontrolReflectsSystemStatusByte = 0;

/***************************************************
				Analog To Digital Converters
Port values 0-3 access ADC03. Port values 4-7 access ADC47. Returned value is in 2's complement format. See function for more details.				
****************************************************/
	int ADCread(byte portByte);
	float ADCconvertToVoltage(int ADCcountInt);
/*************************************************
                 GNSS
***************************************************/
    bool GlobalNavigationSatelliteSystem(bool waitQ);
/*************************************************
                 MODEM
***************************************************/
   unsigned int Iridium(unsigned int ModemCommandInputUInt);
   unsigned int _ModemCommandUInt = 16;
/*************************************************    
				I2C bus access
***************************************************/
void I2C(byte sensorNumberByte,unsigned long delayBeforeAskingForResultULong);
bool I2CbusBusyWithDataDumpQ();//used by core 1 running heartbeat to see if a data dump is running. It is used to force core 0 OK flag.
bool hasFDRoneTimeRunNotRunYetQ = true;
bool simulatorModeQ = false;//if I2C bus 1 fails, use simulated EEPROM and SEEPROM so students can run with on PM.
#define logFileDepth 16 //compiler needs a real number to set size of arrays
byte logFileDepthByte = logFileDepth;//log functons needs this number
unsigned long busUseTimeULong[logFileDepth];//log file to monitor bus 1 usage
byte busUseStateByte[logFileDepth];
byte logRunOnCoreByte[logFileDepth];
byte logUserByte[logFileDepth];
byte logFileIndexByte = 0;
unsigned long logStartTimeULong = millis();
void dumpBus1LogFile();
void logBusState(byte busOwnerByte, byte userByte);
bool disableLogFunctionBool = true;//change to false to run this diag

/**************dualVariableULong************************************
				UTILITIES
***************************************************/
unsigned long changeBytesToUnsignedLong(byte b3, byte b2, byte b1, byte b0);
/**************************************************	
Return Code Translator
************************************************/
void ReturnCodeToPrintedText(unsigned int code);
/************************************************/			

/*********************************************
test variables for core0 core1 test
**********************************************/
byte core1count = 0;
bool resetCount = false;
/******boxes to draw**************************/
	void topOfBox();
	void bottomOfBox();
/*********************************************/
	float readBattery(bool modeBool);
/**********************************************
Average Port 6 sampling period
*****************************************************/
 unsigned long lastPort6SampleTimeUsULong = micros();
 unsigned long port6PeriodLogUsULong[32];
 byte port6LogIndexByte = 0;
 void calculateAndPrintPort6AverageSamplePeriod();

 void testDualVariable();
 unsigned long dualVariableULong[2];
	
 /*****************************************************/
  private:

 /****************************************************
				GPS CONTROL
*****************************************************/			
void sendCFGmessage(byte *CFGmessage, byte length);
byte readACK();
byte UBX_CFG_VALSET_AIR1byte[17];

/****************************************************

  A R D U I N O  O U T P U T  C O N N E C T I O N S
******************************************************
Arduino
Physical logical  	description	
	8		D5	   	external LED
**********************************************************/

/********************************************************
  A R D U I N O   U S A R T   C O N N E C T I O N S
*********************************************************
Arduino
Physical logical  	description	
	1		GPIO0	   	TX0     used by GNSS
	2		GPIO1		RX0     used by GNSS
	11		GPIO8		TX1	hope to use for modem
	12		GPIO9		RX1 	hope to use for modem
**********************************************************/

/******************************************************
  C O N T R O L  B L O C K  IN  E E P R O M
*********************************************************
In the EEPROM the first 16 bytes are the control block.

byte	description
====	===========
  0		Program State
  1		data Dump Status 
  2		spare 
  3		StartOfNextMemoryBlockBaseAddressByte (base address so byte 0)
  4		address byte 1
  5		address byte 2
  6		address byte 3
  7		spare
  8 	spare
  9		spare
  10	hardware and software error codes
11-15 	spare
**********************************************************
Program States:
Value	State
0		In-Flight
1		In-Flight with power disruption
2		MemoryLocked
3		Pre-flight

Given these choices in values, I only need to test if byte is <2 to know I'm in flight.
**********************************************************/
boolean *GNSS_Bool;//declare GNSS_Bool to be a pointer to a boolean data type
boolean *Modem_Bool;
boolean *AutomaticVideoRecordAtPowerUpBool ;
boolean IridiumModemPresentBool;
boolean *I2CsensorsBool;
boolean *EEPROMjustOutputtedBool;
boolean *EEPROMjustClearedBool;
boolean *recordingDataBool;

char *filePathChar;


const byte InFlightByte = 0;
const byte InFlightWithPowerDisruptionByte = 1;
const byte TestForInFlightByte = 2;
const byte MemoryLockedByte = 2;
const byte PreFlightByte = 3;

unsigned long AltitudeULong = 0;

const byte ProgramStateAddressByte = 0;
const byte dataDumpStatusAddressByte = 1;
//2 is spare
const byte StartOfNextMemoryBlockPointerBaseAddressByte = 3;//4, 5, 6 are rest of address with 6 being MSB
//7, 8, and 9 are spare
const byte ErrorCodeAddressByte = 10;
//11-15 are spare
const byte dataBeingDumpedByte = 0;
const byte dataNotBeingDumpedByte = 1;

unsigned int MPM_Busy_TransmitCommandRejectedUInt = 274;
unsigned int TransmissionProcessHasBegunUInt = 322;//matches MPM values
bool ReadEEPROMfailureQBool = false;

/************************************************
M O D E M  S E R I A L  N U M B E R S
*************************************************/
byte *nearFarModemSNbyte;//contains near and far modem User RB sn.
/************************************************						  			 M O D E M  S U C C E S S  R E T U R N S
************************************************/
//unsigned int PingThroughMPM_AndModemSuccessUInt = 400;
//unsigned int ModemReadyForUseUInt = 401;
//unsigned int TransmitSuccessfulAndNoReceiveUInt = 402;
//unsigned int TransmitAndReceiveSuccessfulUInt = 403;
//unsigned int TransmitAndReceiveSuccessfulPlusReceivePendingUInt = 404;
/************************************************
M O D E M  F A I L U R E  R E T U R N S
************************************************/
unsigned int DuplicateTransmitOfDataAttemptedUInt = 255;
/***********************************************************
 R E F E R E N C E  I N F O R M A T I O N
**********************************************************
to interface with laptop: www.arduino.cc\en\Reference\Serial.html
 
 https://learn.sparkfun.com/tutorials/reading-and-writing-serial-eeproms
 
Based on: https://playground.arduino.cc/Code/I2CEEPROM

 **********************************************************/

/***********************************************************
C O N S T A N T S
*********************************************************/
String SourceFile = __FILE__;
byte SamplingRateSecondsByte = 2; //rate data is collected and output. I'm doing oversampling on output to generate data every second so do not change this ant.
byte LogicalAnalogPinByte[8]; //we have ADC03 and ADC47
float voltsPerCountFloat = 187.5E-6;//  6.144V/2^15 = 187.5 uV/step
float batteryVoltageDividerCompensationFloat = 4.3;//100K and 330K in divider
float PMadcStepSizeFloat = 805.66E-6;//12 bit ADC with 3.3V ref so 3.3V/4096
float metersToFeetFloat = 3.28;
int noACKreceivedInt = 32000;//corresponds to 6V
int outOfrangePortNumberInt = 29333;//corresponds to 5.5V
bool asAVoltageBool = true;
//unsigned int FailureRangeMinUInt = 1;//used to assess modem responses
//unsigned int FailureRangeMaxUInt = 299;
//unsigned int StatusRangeMinUInt = 300;
//unsigned int StatusRangeMaxUInt = 399;
//unsigned int SuccessRangeMinUInt = 400;//includes ping and modem set up
//unsigned int TransmitSuccessRangeMinUInt = 402;//only includes transmit related successes
//unsigned int SuccessRangeMaxUInt = 499;
//unsigned int MinimumTransmitCycleTimeSecondsUInt = 60;
#define MinimumTransmitCycleTimeSecondsUInt 60U //it is defined because they are used to define other variables
#define InitialReturnCodeValueUInt  328U //it is defined because they are used to define other variables
#define InitialMPM_ResponseValueUInt 329U //it is defined because they are used to define other variables
/***********************************************
 M O D E M  C O M M A N D S 
 ***********************************************/
unsigned int PingUInt = 0;
unsigned int SetUpModemUInt = 1;
unsigned long PerformTransmitUInt   = 2;
unsigned long getReceivedDataUInt  = 3;
unsigned int setLoopAroundUInt  = 6;
unsigned int clearLoopAroundUint = 7;
unsigned int statusUInt = 8;
//***********************************************
//unsigned int SentPerformTransmitUInt = 323;
//unsigned int SentgetReceivedDataUInt = 324;
//unsigned int AboutToStartTransmitProcessUInt = 326;
//unsigned int InitialReturnCodeValueUInt  = 328;
//unsigned int InitialMPM_ResponseValueUInt = 329;
//unsigned int BusySettingUpModemUInt = 330;
//unsigned int PerformingPingUInt = 331;
//unsigned int MPM_Busy_TransmitCommandPendingUInt = 334;
//unsigned int ModemSetupProceedingUInt = 335;
//unsigned int ModemDefaultsSetUInt = 336;
//unsigned int SentPingUInt = 337;
//unsigned int SBD_MessageSuccessfullyWrittenUInt = 338;
//unsigned int MT_MessagePendingUInt = 339;
//unsigned int MT_MessagesPendingUInt = 340;

//unsigned int InvalidCommandUInt = 120;
//unsigned int PingToMPM_SuccessButToModemFailedUInt = 276;
//unsigned int PingToMPM_TimedOutUInt = 275;
//unsigned int TimeOutWaitingForgetReceivedDataUInt = 279;
//unsigned int NoPingMPM_BusyUInt = 280;
//unsigned int ModemFailedAtSetupUInt = 281;
//unsigned int UnexpectedResponseFromModemDuringSetupUInt = 282;
//unsigned int ModemSetupFailedBecauseMPM_BusyUInt = 283;
//unsigned int ModemFailedAtSetupTimeOutUInt = 284;
//unsigned int MPM_BusyWhenFDR_AskedForSetupUInt = 285;
//unsigned int MPM_DidNotRespondToRequestForDataUInt = 286;
//unsigned int SoftwareError1UInt = 287;
//unsigned int MPM_BusyUInt = 288;
//unsigned int  AskedForPingResultTooSoon_DoPingAgainUInt = 289;
//unsigned int PingToMPM_DidNotRespondUInt = 290;
//unsigned int WrongModemConnectedCheckSerialNumberUInt = 291;
//unsigned int RequestedTransmitTooSoonUInt = 292;
//unsigned int NoFunctioningModemPresentUInt = 293;
//unsigned int TimeOutAfterSendingMessageUInt = 294;
//unsigned int SBD_MessageTimeOutByModemUInt = 295;
//unsigned int SBD_MessageChecksumWrongUInt = 296;
//unsigned int SBD_MessageSizeWrongUInt = 297;
//unsigned int UnexpectedResponseAfterWritingDataToMobileOriginatedBufferUInt = 298;
//unsigned int SBD_MessageSizeTooBigOrTooSmallUInt = 299;
/********************************************************************************
H A R D W A R E  A N D  S O F T W A R E  E R R O R  C O D E S 
*********************************************************************************/
const byte SystemNormalFlagByte = 0;
const byte OutOfRangeReadFlagByte = 1;
const byte OutOfRangeWriteFlagByte = 2;
const byte ControlTheFlightParametersReadyForLaunchReadBackFailureFlagByte = 3;
const byte ControlTheFlightParametersInFlightFlagReadBackFailureFlagByte = 4;
const byte JustWaitReadBackFailureFlagByte = 5;
const byte PrepareForLaunchReadBackFailureFlagByte = 6;
const byte WriteEEPROM_AttemptMadeToWriteToLockedMemoryFlagByte = 7;
const byte SerialAvailableReturnCodeOutOfRangeByte = 9;
const byte ReadBackFromEEPROM_MismatchByte = 10;
const byte AttemptMadeToWriteToControlBlockByte = 11;
const byte AttemptMadeToWriteToDataBlockByte = 12;
const byte equippedGPShardwareFaultByte = 13;
const byte equippedMPMhardwareFaultByte = 14;
const byte equippedModemHardwareFaultByte = 15;
const byte equippedModemSetupFaultByte = 16;
const byte EEPROMdataFailureByte = 17;

const byte loopTimeMoreThanLimitByte = 18;
const byte loop1TimeMoreThanLimitByte = 19;
const byte SEEPROMdataFailureByte = 20;
/**************************************************************************
C A M E R A  R E T U R N  V A L U E S
***************************************************************************/
byte NoCameraErrorByte = 0;//note that this is the same as SystemNormalFlagByte
byte CameraHasNoPowerErrorByte	= 1;
byte IllegalCameraStateValueErrorByte = 2;
byte VideoRecordStateByte = 0;
byte VideoStandbyStateByte = 1;
byte StillStandbyStateByte = 2;
byte CameraHasNoPowerStateByte = 3;
byte MaximumCameraStateValueByte = 3;
/*************************************************************************/
const unsigned int LoopCycleTimeLimitMS_UInt = 1000;
unsigned int  maximumLoopCycleTimeMS_UInt = 0;
unsigned int  maximumLoop1CycleTimeMS_UInt = 0;
/***************************************************************************
V A R I A B L E S  and C O N S T A N T S
***************************************************************************/

const byte numberOfBlocksOffsetByte = 0;//Move the CB steps of 16 to keep data lined up with LOW and HIGH BLOCKs in EEPROM. We move the CB as we wear out the EEPROM.
const byte DataBlockSizeByte = 16;
const long StartOfUsableEEPROM_Long = numberOfBlocksOffsetByte*long(DataBlockSizeByte);//the control block starts at this address. 
long startOfNextMemoryBlockLong = StartOfUsableEEPROM_Long+16L; //this is the pointer to the next available memory location; it is initialized at power up to just after control block.
unsigned long StartTimeMSULong = 0; //will be set by millis() to the value of the real time clock shortly after power up.   
int TimeNowSecondsInt = 0; //used to set seconds flag 
int Last_TimeNow_SecondsInt = 0; //used to set seconds flag 
boolean TimeToTakeSampleBool = false; //is set true at the start of each second and cleared by last user CR3.2
boolean TimeToTransmitBool = false;
boolean TransmitSuccessfulBool = false;
char Command = 'N'; //stores single character sent from laptop; initilized to "N" = null.
boolean ReadyForLaunchFlag = false;
boolean InFlightFlag = false;
byte I2Cbus1PermissionByte = 0;//used to time multiplex the start of I2Cbus1Mitigation() 
//unsigned long StartOfPreFlightCadenceMSULong = millis();
//unsigned long StartOfInFlightCadenceMSULong = millis();
//unsigned long StartOfDisruptedPowerCadenceMSULong = millis();
//unsigned long StartOfMemoryLockedLED_CadenceMSULong = millis();
//unsigned long StartOfFaultLED_CadenceMSULong = millis();
//boolean PreFlightLED_CadenceFlag = false;
//boolean InFlightLED_CadenceFlag = false;
//boolean DisruptedPowerLED_CadenceFlag = false;
//boolean MemoryLockedLED_CadenceFlag = false;
//boolean FaultLED_CadenceFlag = false; 
boolean JustPoweredUpFlag = true; //flag used by ControlTheFlightParameters(). At every power up, this flag is set true.
boolean oddCycleBool = true;//used for normal recording LED
boolean JustPoweredUpToBeUsedByRunCamFlag = true;//flage used only by the RunCam2 code so record command is sent after power up. RGS1.4
boolean JustPoweredUpForTimeStampFlag = true; //flag used by time stamp function. At every power up, this flag is set true. CR3.0
boolean *justClearedEEPROM_Bool;//available to students to clear their SEEPROM
//unsigned long InFlightCadenceIntervalMSULong = 0;
//unsigned long PreFlightCadenceIntervalMSULong= 0;
//unsigned long MemoryLockedCadenceIntervalMSULong= 0;
//unsigned long DisruptedPowerCadenceIntervalMSULong= 0;
//unsigned long FaultCadenceIntervalMSULong= 0;
long eeAddressLong= 0;//local variable used for all addressing of EEPROM 
boolean ContinuousDisplayDataFlag = false; //flag set by ContinuousDisplayDataQ() and cleared at power cycle  CR1.7
//long DurationMSULong= 0 ;
#define IdealCalibrationTimeSecondsULong  10800UL //3 hours=180 minutes=10800 seconds (10800UL) so get number of internal ms counts per second
//unsigned long StartOfCalibrationTimeMSULong = 0;
//long CalibrationTimeMS_Long = 0;
//float OneSecondMillisecondsFloat = 0; //CR2.0
//float ErrorPercentageFloat = 0;
boolean FoundCommandQ = false;
boolean DeepReturnBool = false;//passes a return from a subroutine up to the calling subroutines
//float LimitTestFloat = 0;
int SecondsInt = 0;
byte IncomingByte = 0; //used by RockBlock modem 4.4
volatile unsigned int ReturnCodeUInt = InitialReturnCodeValueUInt;//used by Fdr.cpp and FDR.ino
unsigned int MPM_ResponseUInt = InitialMPM_ResponseValueUInt; 
unsigned int LastMPM_ResponseUInt = 1000;//far outside of valid responses   
volatile byte (*IridiumTransmitDataByte)[45];//a pointer to an array of bytes
byte PreviousIridiumTransmitDataByte[45];//a locally defined array
volatile byte (*IridiumReceivedDataByte)[45];//a pointer to an array of bytes used by modem and I2C drivers. It is sent from MPM
byte _IridiumTransmitDataByte[45];//used by Iridium.ino
byte _IridiumReceivedDataByte[45];//used by Iridium.ino
byte MPM_RejectedDataByte[45];
byte (*NavigationDataByte)[16];
//byte (*I2CdataByte)[17];//first 16 bytes are data. Last byte is status
byte successByte = 2;
byte failedByte = 3;
byte timedOutByte = 4;
byte illegalSensorNumberByte = 1;
byte notReadyYetByte = 5;
byte i = 0;
unsigned long SystemTestStartTimeULong = millis();
bool CycleIdleBool = true;
//byte CommandLowByte = 0;
//byte CommandHighByte = 0;
byte ResponseLowByte = 0;
byte ResponseHighByte = 0;
unsigned int LastReturnCodeUInt = 1000;//beyond all valid return codes 
unsigned int LastSuccessfulTransmitTimeSecondsUInt = (millis()/1000) - MinimumTransmitCycleTimeSecondsUInt - 10;//this ensures we run on first pass.
unsigned long ReadbackStartTimeULong = 0;
unsigned long StartTimeULong = 0;
unsigned long StartForTimeStampULong = millis();
unsigned long StartForLoopTimerMsULong = millis();
unsigned long StartForLoop1TimerULong = millis();
boolean IridiumModemOperationalBool = false;//set true if we can initialize it
unsigned long LastTransmissionCheckTimeStampULong = millis() - 6000;//ensures first request is accepted
boolean PerformDataTransmissionBool = true;//this will let me transmit once 
boolean DataTransmissionSessionActiveBool = false;
byte ModemCommandArrayByte[2];   
unsigned long TimeSinceLastSuccessfulTransmissionSecondsULong = 0;
boolean RequestTransmitRestartBool = false;
char character;
String BufferText = "";
byte count = 0;	
byte EndOfField = 0;
byte CountPlusOne = 0;
byte CountPlusTwo = 0;
boolean AtEndOfNeededSentenceBool = false;
byte pointer = 0;
byte FieldCountByte = 0;
unsigned long GNSS_WaitingForDataTimeLimitMsULong = 1000;
unsigned long GNSS_StartOfWaitingForDataMsULong = millis();
float FractionalMinutesFloat = 0;
byte StartOfAltitudeByte = 0;
byte EndOfFieldByte = 0;
boolean DiagPrintBool = true;
boolean GNSS_TimeToRetryReadingDataQBool = false;//init to don't retry access to GNSS
boolean GNSS_TimedOutWaitingForDataBool = false;//init is that GPS has been detected
unsigned long TimeNowMsULong = millis();//time since power up.
unsigned int GNSS_DelayBeforeRetryOfReadingDataMsUInt = 60000;//Retry talking to GNSS after 60000 ms.   RGS1.5
unsigned long LastTimeWeDidA_RetryOfReadingDataMsULong = TimeNowMsULong - GNSS_DelayBeforeRetryOfReadingDataMsUInt - 10;//initialized so GNSS_Timing() will think it has been more than GNSS_DelayBeforeRetryOfReadingDataMsUInt since last retry
boolean IncompleteSentenceBool = false;
int PortReadingInInt = 0;//make these variables global so I can use them for peak detector RGS1.1
byte PortReadingInHighByte = 0;
byte PortReadingInLowByte = 0;
byte PortReadingOutByte = 0;
long PortDataAddressLong = startOfNextMemoryBlockLong; //port 0
float VoltageFloat = 0; //used when sending port voltages to display
boolean ASCII_Bool = false;
boolean HEX_Bool = true;
byte OldErrorCodeByte = 20;
byte ErrorCodeByte = 21;
boolean NoBatteryWasConnectedBool = true;
byte GNSS_RetryCounterByte = 1;//counter advanced each time we try to access the GNSS and it fails. RGS1.5
byte GNSS_RetryCountLimitByte = 2;//number of times we will retry the GNSS when it has not sent any data
byte LocalErrorByte = 99;
//unsigned long LastLoopTimeStampMSULong = millis()+29999;//offset compensates for not monitoring for the first 30 seconds
unsigned long LastLoopTimeStampMSULong = millis();

//unsigned long LastLoop1TimeStampMS_ULong = millis()+29999;//offset compensates for not monitoring for the first 30 seconds
unsigned long LastLoop1TimeStampMS_ULong = millis();
unsigned long LoopCycleTimeMS_ULong = 0;
bool resetLoopTimerBool = false;//I use this flag to prevent loop time warning after user enters a function abort.
bool resetLoop1TimerBool = false;//this flag enables the library to tell Iridium.ino to reset the loop1() timer. Iridium.ino can read this flag using fdr.isLoop1TimerActiveQ() and clear this flag using fdr.enableLoop1TimerWarning()
bool enableLoopAroundQbool;
bool oddCycleQ = true;//used to toggle LED as function of loop() cycle
byte noACKbyte = 0x02;
byte ACK_ACKbyte = 0x01;
byte ACK_NAKbyte = 0x00;
byte UBX_CFG_VALSET_HEADERbyte[2];// = {0xb5,0x62};
byte UBX_CFG_VALSET_CLASSIDbyte[2];//= {0x06,0x8a};
byte UBX_CFG_VALSET_PAYLOAD_FRONTbyte[4];// = {0x00,0x01, 0x00, 0x00};
byte CFG_NAVSPG_UTCSTANDARDbyte[4];// = {0x20,0x11,0x00,0x1c};
const byte AIR1byte = 0x06;
byte FletcherChecksumByte[2];// = {0,0};
byte lengthOfPayloadByte[2];
int Port6PeakInt;
byte (*LEDflashInfoByte)[3];
const byte I2CbusReleasedByte = 0;
byte I2Cbus1OwnerByte = I2CbusReleasedByte;
float portVoltagesFloat[8];//speeds up output of data
byte  portDataByte[16];//temp storage of GNSS data after read from EEPROM during odd seconds. This saves time. 
byte  GNSSdataByte[16];//temp storage of port data after read from SEEPROM during odd seconds. This saves time. 
long GNSSaltitudeLong;//temp storage of GNSS altitude after reading from SEEPROM during odd seconds. This saves time.
const bool outputAsVoltageBool = true;
const bool outputAsCountBool = false;
const bool haveUSBbool = false;
bool newDTR_Bool = true;
byte simEEPROM[1600];
byte simSEEPROM[1600];
const float PMadcMvPerStepFloat = 0.80566E-3;
const byte teamNumberReadPortByte = 4;
const bool team2Bool = true;//false means team 1
const bool yesBool = true;
const bool noBool = false;
bool printTheUserPromptQ = false;//after a function services a command, they use this flag to tell system they should print the prompt again.
bool externalLEDcontrolBool = false;//lets students control the external LED
/********************************************************************************
U S E R  I D          - USE WITH ReadEEPROM_Pointer()
*********************************************************************************/
byte CalledBySetupByte = 1;
byte CalledByIncrementEEPROM_PointerByte = 2;
byte CalledByDiagnosticOutputControlBlockByte = 4;
byte CalledBySanityCheckControlBlockByte = 5;
byte CalledByRecordDataByte = 6;
byte CalledByOutputDataToFileByte = 7;
byte CalledByOutputRunTimeEstimateByte = 8;
byte CalledByEEPROMdataDumpByte = 9;
byte CalledBySEEPROMdataDumpByte = 10;
/********************************************************************************
S O F T W I R E  S E T U P   (O P T I O N A L) 
*********************************************************************************/
byte sdaPin = 8; //Replace "8" with the logical pin you want to use for the Serial Data line
byte sclPin = 7; //Replace "7" with the logical pin you want to use for the Serial Clock line 
byte cmdAmbient = 6; 
byte cmdObject1 = 7; 
byte cmdObject2 = 8; 
byte cmdFlags = 0xf0; 
byte cmdSleep = 0xff;
const byte neitherByte = 2;//owners are bus 0, 1, or this
byte I2Cbus1LinkedToCoreByte = neitherByte;
const byte unprotectedWriteEEPROMuserByte = 1;
const byte readEEPROMuserByte = 2;
const byte writeSEEPROMuserByte = 3;
const byte readSEEPROMuserByte = 4;
const byte ADCreadUserByte = 5;
const byte dumpEEPROMuserByte = 6;
const byte dumpSEEPROMuserByte = 7;
const byte readEEPROMdataBlockUserByte = 8;
const byte readSEEPROMdataBlockUserByte = 9;
const byte unprotectedWriteSEEPROMuserByte = 10;
unsigned long timeStudyStartTimeULong = millis();
String FdrHeaderDateString = String(__DATE__);   
String FdrHeaderTimeString = String(__TIME__); 
byte dataBlockByte[16];//stores one data block's bytes

const byte writeCycleDelayByte = 6;//EEPROM delaybyte
const byte waitForEEPROMresponseMsByte = 2;//delay after giving the EEPROM an address before we can check for return byte

bool timeSlot0Bool = false;
bool timeSlot1Bool = false;

			
    void Time();//
    void OnTheGround();//
    void InTheAir();//
	void ReadEEPROM_Pointer(byte CalledBySetupByte);//
	void RecordPortAndGNSSReadingsAndOptionallyPrintsToScreen();
	void IncrementEEPROM_Pointer();//
	void SetNominalTimerValue();//
	void LoopCycleTimeMonitor(bool verboseQ);
	void LapseTimerSeconds();//
	void ModemTransmitTiming();//
	void GNSS_Timing();//
	void LED_Cadences();//
	void ControlTheFlightParameters();//
	void ScanForCommand();//
	void OutputMenuQ();//
	void RebootQ();//R command
	void SingleDisplayDataQ(); //D command
	void PrepareForLaunchQ(); //P command
	void ContinuousDisplayDataQ(); //A command  CR1.7
	void OutputDataFileQ(); //O command
	void StopCollectingDataQ(); //S command
	void CalibrateClockQ();//
	void DiagnosticOutputControlBlockQ(); //H command
	void GenerateTestPatternQ(); //T command
	void GenerateBinaryTestPattern();
	//void PingModemDiag(); //moved to Iridium.ino
	//unsigned int PingModem();
	//unsigned int getMPMversion();
	//unsigned int modemStatus();
	//unsigned int loopback();
	void FlashLED();
	void DataIn();//
	void PrintOutGNSS_Array();//
	void IridiumModemSatelliteSystem();//
	void InFlightCadence();//
	void DisruptedPowerCadence();//
	void MemoryLockedCadence();//
	void PreFlightCadence();//
	void FaultCadence();//
	void PrintLine(bool preceedWithBlankLineQ);//
	void C_CommandQ();//
	void FlickerLEDforCalibration();//
	void SerialAvailableResponseErrorCheck();//
	void FirstAbortCommandQ();//
	void G_CommandQ();//
	void PrintCalibrationStartedText();//
	void EndOfCalibration();//
	void PrintCalibrationIntroText();//
	void A_CommandText();//
	void CalibrationUserStatus();
	void CalibrationLEDFlicker();
	void I_CommandText();
	void InvalidCommandText();
	void A_CommandQ(); 
 	void Y_CommandQ();
	void EndOfCalibrationCommandValidityCheck();
	void IandY_CommandQ();
	void E_Command();
	void SetIllegalTimerValue();
	void SaveTimerCalibrationValueToEEPROM();
	void TellUserAboutTimerCalibrationValue();
	void CalculateAndFormatCalibrationValue();
 	void OutputDataToLogFile();
	void DiagnosticOutputControlBlock();
	void OutputSingleLiveScan();
	void printHeadingsForLiveOutput();
	void OutputMenuOfCommandsAndLED_Cadences(bool longQBool);
	void SendPrepareForLaunchWarning();
	void ActOnResponseToPrepareForLaunch();
	void GenerateTestPatternWarning(bool typeBool);
	void ActOnResponseToGenerateTestPattern();
	void ActOnResponseToGenerateBinaryTestPattern();
 	void RecordData();
	void WriteEEPROM_Pointer();
	float LivePortVoltageReadingFloat(byte PortNumberByte);//
	void DisplayErrorState();
	void JustWait();
	//byte ReadEEPROM(long eeaddress); MOVED TO PUBLIC
	void PrepareForLaunch();
	void NonFaultStopProgram();
	void FaultStopProgram();
	void GenerateTestPattern();
	void WriteEEPROM_Data(long eeAddress, byte data);
	unsigned int SetUpModem();
	unsigned int PerformTransmit(); 
	unsigned int getReceivedData();
	void FlushWire1Buffer();
	//void FlushWire2Buffer(); //probably need in future
	void ChangeCameraMode();
	void PushShutter();
	void OnePulse();
	void PrintOutInterpretationOfSingleResponseFromTransmissionCommand();
	void RequestDataTransmissionSession();
	void CheckTransmissionProgress();
	bool I_Two_C_BusAvailableQ();
	void DirectPath(bool HexQ);
	void InitializeNavigationDataByteArray();
	bool FindNeededSentence(bool waitQ);
	void ParceTime();
	void ParceLatitude();
	void ParceLongitude();
	void SkipFourFields();
	void ParceAltitude();
	void PrepareTransmitArrayForModem();
	void ProcessReturnCodes();
	void ReadHeader();
	void ReadSentence();
	void ZeroControlBlock();
	void ZeroJustTimeCalibration();
	bool dollarSignInBufferQ();
	void outputPortVoltages(long pointer,bool oddSecondsQ);
	void RawOutputGPSdata(long EEPROM_PointerLong, bool oddSecondsQ);
	void formattedOutputGPSdata(bool oddSecondQ);
	void printOneSpace();
	void printComma();
	void printTwoSpaces();
	void printColon();
	void ControlBlockEEPROMtest();
	void fullEEPROMtest();
	void partialEEPROMtest();
	void echoSerialTimeout(unsigned long timeout);
	void echoSerialTerminatorTimeout(byte terminator, unsigned long timeout);
	void nmeaWriteByteUpdateChecksum(byte *csum, byte b);
	void nmeaWriteBytesUpdateChecksum(byte *csum, const char *b);
	void nmeaWriteChecksumCRLF(byte csum);
	void sendNMEA(const char *message);
	void emptySerial1Buffer();
	void printUTCTime();
	bool checkForACKheader();
	bool checkForACKresult();
	void printMPMversion();
	void LEDflashInfo();
	void emptyUSBreceiveBufferQ(bool waitQBool);
	bool pingI2C();
	void setupGPS();
	bool I2Cbus1Mitigation(byte userByte);
	void welcomeSerialMonitor();
	void clearRecordedErrorsQ();
	void controlBlockSanityTest();
	void printTeamsName();
	void processAnyActiveCommand();
	void printAmountOfDataStoredAsHMS();
	void printTotalRunTime();
	void printControlBlockStateAndError();
	void printFileInfo();
	bool dumpEEPROM();
	bool dumpSEEPROM();
	bool readEEPROMdataBlock(long eeAddressLong);
	bool readSEEPROMdataBlock(long eeAddressLong);
	void populateSimulatedControlBlock();
	void emptySerialReceiveBuffer();
	void waitForOneTimeRunToFinish();
	void releaseI2Cbus();
	//void inhibitLoop1TimerWarning();
	//void setLoop1TimerToTrue();
	void printGNSSstatusInEnglish();
	void userNumberToEnglish(byte userNumber);
	void printArizonaTimeOrGNSSstatus();
	void timeSlotGenerator(unsigned long intervalULong);
	void sequentialFillMemory(bool PROMBool);
};
#endif