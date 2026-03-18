char IridiumVersionchar[] = "5.1.1";	


//#define mastercontrol
//#define pingTest
//#define pingTest5
#ifdef mastercontrol
	#define nearFarTest
	#define looparoundTest
	#define nearfasrtest //this controls a lot more than near far logic. I used it to debug interrupts interfering with various functions.
	#define modemNGtest //this controls prints showing modem interactions at setup
	#define DiagPrint1
#endif

#define ADRIAN Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);Serial.print(F("(): "));Serial.print(__LINE__);Serial.print(F(".    Time Stamp  "));	Serial.print (millis() - StartForTimeStampULong);Serial.println(F(" ms"));

//#define frozen11
//#define frozen1
//#define modemNGtest
//#define pingMystery
//#define flash
//#define heartbeatTest
//#define deadInterface
//#define noKBf
//#define menuQ
//#define snx
//#define frozen
//#define failedModem
//#define uartTest
//#define setupMystery
//#define mysteryCall
//#define modemsetupTrace
//#define comVar
//#define testTranslate
//#define I2C_conflictTest
//#define pingRunTimeTest
//#define OKtimeTest
//#define noBatteryTest
/******************************************************
Diagnostic Tools

fdr.UserDefinedControlOfLED(byte patternByte) where
patternByte can have the following values 

0	we are no longer using the LED for diagnostic 
1	LED is on continuously
2	LED is off continuously
3	LED flickers for 1 second and is then off
4	LED flickers for 10 seconds and is then off
X   any other value causes LED to flicker for 30 seconds and is then off
******************************************
Available Modems

UserRBsn	RBsn			IMEI

00 			 13301		300234066438070		
01			218642		300534065390120	ME
10			 13298		300234066436090		
11			218641		300534065396130		

*******************************************/

#include <WString.h> // for FlashString   
#include <Stream.h> // for Stream   
//#include <Arduino.h> //was "Arduino.h"  set in FDR.ino
//#include <Wire.h> //for I2C
//#include <SoftWire.h>
//#include <AsyncDelay.h> //used by SoftWire

//#define  MPM__ADDRESS 0xA  //MPM_ address,any number from 0x01 to 0x7F

//MPMinoVersionByte = 5;	

/*******************************************************
F L A G S
*******************************************************/
bool MPM_BusyBool = false;
bool ModemSetupResultWasPickedUpByFDR_Bool = false;
bool NewFDR_CommandRejectedBool = false;
bool SendFirstBlockOfReceivedArrayBool = false;
bool SendSecondBlockOfReceivedArrayBool = false;
bool OneTimeDelayBool = true;//diag
bool PingActiveBool = false;
bool enable = false; //gates diagnostic print statements
//bool NewIridiumCycleBool = false;
//bool CycleActiveBool = false;
//bool FoundOhBool = false;
bool modemLooparoundEnabledQbool = false;
bool MPMonlyLooparoundEnabledQbool = false;
bool secondBlockNotFilledBool = false;
bool firstBlockNotFilledBool= false;
//bool MPMoneTimeRunBool = true;  defined in FDR.ino
bool IridiumModemPresentBool = false;
boolean IridiumModemOperationalBool = false;//set true if we can initialize it
bool core0AliveQBool = true;//used by heartbeat. It will show no problems for the first 4 seconds after power up.
bool sentDataBool = false;//used by heartbeat
byte ProgramStateAddressByte = 0;//was byte ProgramStateAddressByte = 0;
bool preventFloodBool = false;//diag  let it print once
bool resetLoop1TimerBool = false;//set true if we want to inhibit loop1 timer

byte InFlightByte = 0;
byte InFlightWithPowerDisruptionByte = 1;
byte TestForInFlightByte = 2;
byte MemoryLockedByte = 2;
byte PreFlightByte = 3;

unsigned long LEDflashTimerStartULong = millis();
int ExternalLED = 5;
//byte ExternalLED_OnByte = 1;
//byte ExternalLED_OffByte = 0;

//************************************************************
/*************************************************************
C O N S T A N T S  
*************************************************************
All return codes have unique numbers.
************************************************************/
const unsigned int DelayBeforeDoingReadFromModemUInt = 20;//I get a solid failure when reading the serial number when delay is 1 ms and works at 2 ms. So cliff is between 1 and 2 ms. be 10x away from it.
const unsigned int DelayAfterDoingPrintToModemUInt = 20;
 
const long MoreThantheMaxNumberCharactersByteWasProcessedExtractFieldLong = -1;
const long NonNumericCharacterWasFoundBeforeTheCommaExtractFieldLong = -2;
const long NoNumbersFoundBeforeTheCommaExtractFieldLong = -3;
const byte ErrorCodeAddressByte = 10;

byte SystemNormalFlagByte = 0;
byte OutOfRangeReadFlagByte = 1;
byte OutOfRangeWriteFlagByte = 2;
byte ControlTheFlightParametersReadyForLaunchReadBackFailureFlagByte = 3;
byte ControlTheFlightParametersInFlightFlagReadBackFailureFlagByte = 4;
byte JustWaitReadBackFailureFlagByte = 5;
byte PrepareForLaunchReadBackFailureFlagByte = 6;
byte WriteEEPROM_AttemptMadeToWriteToLockedMemoryFlagByte = 7;
byte SerialAvailableReturnCodeOutOfRangeByte = 9;
byte ReadBackFromEEPROM_MismatchByte = 10;
byte AttemptMadeToWriteToControlBlockByte = 11;
byte AttemptMadeToWriteToDataBlockByte = 12;
byte equippedGPShardwareFaultByte = 13;
byte equippedMPMhardwareFaultByte = 14;
byte equippedModemHardwareFaultByte = 15;
byte equippedModemSetupFaultByte = 16;
byte EEPROMdataFailureByte = 17;
unsigned int maximumLoop1CycleTimeMS_UInt = 0;

#define EEPROM_ADR_LOW_BLOCK 0x50 //FDR's EEPROM 0
#define EEPROM_ADR_HIGH_BLOCK 0x54 //FDR's EEPROM 0

const byte dataDumpStatusAddressByte = 1;//is also defined in Fdr.h but we can't get it from here so must define it again
const byte dataBeingDumpedByte = 0;//is also defined in Fdr.h but we can't get it from here so must define it again

/*******************************************************
Modem Serial Numbers
*******************************************************/
const unsigned long RBsnULong[4]={13301,218642,13298,218641};//these are the four RockBLOCK serial numbers in decimal. Each one can take up to 3 bytes
const String nearIMEIstring[4]={"300234066438070","300534065390120","300234066436090","300534065396130"};//these are the IMEI printed on the modems
//in both arrays, the User serial number, written on the modem in binary, identifies the modem
/***********************************************
M 0  S T A T U S  R E T U R N  C O D E S 
************************************************
0 MO message, if any, transferred successfully.  
1 MO message, if any, transferred successfully, but the MT message in the queue was too big to be transferred.  
2 MO message, if any, transferred successfully, but the requested Location Update was not accepted.  
3..4 Reserved, but indicate MO session success if used.  
5..8 Reserved, but indicate MO session failure if used.  
10 GSS reported that the call did not complete in the allowed time.  
11 MO message queue at the GSS is full.  
12 MO message has too many segments.  
13 GSS reported that the session did not complete.  
14 Invalid segment size.  
15 Access is denied.  ISU-reported values:  
16 ISU has been locked and may not make SBD calls (see +CULK command).  
17 Gateway not responding (local session timeout).  
18 Connection lost (RF drop).  
19 Link failure (A protocol error caused termination of the call).  
20..31 Reserved, but indicate failure if used.  
32 No network service, unable to initiate call.  
33 Antenna fault, unable to initiate call.  
34 Radio is disabled, unable to initiate call (see *Rn command).  
35 ISU is busy, unable to initiate call.  
36 Try later, must wait 3 minutes since last registration.  
37 SBD service is temporarily disabled.  
38 Try later, traffic management period (see +SBDLOE command)  
39..63 Reserved, but indicate failure if used.  
64 Band violation (attempt to transmit outside permitted frequency band).  
65 PLL lock failure hardware error during attempted transmit.
*****************************************************
S U B R O U T I N E  R E T U R N  C O D E S 
************************************************
My failure returns are 100 to 299

My status returns are   300 to 399

My success returns are 400 to 499
************************************************/
/*
const unsigned long FailureRangeMinUInt = 1;
const unsigned long FailureRangeMaxUInt = 299;
const unsigned long StatusRangeMinUInt = 300;
const unsigned long StatusRangeMaxUInt = 399;
const unsigned long SuccessRangeMinUInt = 400;
const unsigned long SuccessRangeMaxUInt = 499;
*/
/************************************************
F A I L U R E  R E T U R N S
************************************************/
const unsigned int FailureAfterSBDIX_UInt = 100;
const unsigned int ModemStatus_TimedOutUInt = 101;
const unsigned int UnexpectedMO_StatusValueUInt = 104;
const unsigned int UnexpectedMOMSN_ValueUInt = 105;
const unsigned int UnexpectedMT_StatusValueUInt = 106;
const unsigned int UnexpectedMTMSN_StatusValueUInt = 107;
const unsigned int UnexpectedMT_SBD_MessageLengthUInt = 108;
const unsigned int UnexpectedMT_SBD_MessageQueuedValueUInt = 109;
const unsigned int ModemFailureAfterSBDIX_UInt = 112;
const unsigned int UnexpectedResponseToSBDIXUInt = 113;
const unsigned int TimeOutAfterSBDIX_UInt = 114;
const unsigned int TimeOutAfterSendingMessageSizeUInt = 116;
const unsigned int MO_BufferClearedErrorResponseUInt = 118;
const unsigned int MO_BufferClearedTimeOutUInt = 119;
const unsigned int InvalidCommandUInt = 120;  
const unsigned int MO_BufferClearedUnexpectedResponseUInt = 200;
const unsigned int MT_BufferClearedErrorResponseUInt = 202;
const unsigned int MT_BufferClearedTimeOutUInt = 203;  
const unsigned int MT_BufferClearedUnexpectedResponseUInt = 204;
const unsigned int DisableFlowControlRequestTimedOutUInt = 206;
const unsigned int DisableFlowControlRequestUnexpectedResponseUInt = 207;
const unsigned int DisableSBD_RingSetupFailedDueToTimeOutUInt = 208;
const unsigned int DisableSBD_RingSetupFailedDueToUnexpectedResponseUInt = 209;
const unsigned int RingIndicationErrononiouslyEnabledUInt = 231;
const unsigned int TimeOutAfterGetResponseFromVerifyDisableMT_AlertUInt = 232;  //no response within TimeLimitULong ms
const unsigned int UnexpectedResponseToSBDMTAUInt = 233;
const unsigned int UnexpectedResponseAfterSendingMessageSizeUInt=234;
const unsigned int SetupFailedDueToTimeOutUInt = 236;
const unsigned int SetupFailedDueToUnexpectedResponseUInt = 237;
const unsigned int NetworkStatusUnexpectedResponseUInt = 238;
const unsigned int TimeOutNetworkStatusUInt = 239;//no valid response within TimeLimitULong ms
const unsigned int NetworkNotAvailableUInt = 240;
const unsigned int MT_MessageUnexpectedResponseUInt = 242;
const unsigned int MT_MessageTimeOutUInt = 243;  //no valid response within TimeLimitULong ms
const unsigned int MT_MessageFailedCheckSumUInt = 244;
const unsigned int MT_MessageTooLongUInt = 245;
const unsigned int ModemSetupFailedDueToTimeOutUInt = 249;
const unsigned int NoModemConnectedUInt = 250;
const unsigned int UnexpectedModemConnectedUInt = 251;
const unsigned int DuplicateTransmitOfDataAttemptedUInt = 255;
const unsigned int YouAreAskingToTransmitTooSoonSoTryLaterUInt = 256;
const unsigned int FlowControlSetupFailedDueToTimeOutUInt = 257;
const unsigned int FlowControlSetupFailedDueToUnexpectedResponseUInt = 258;
const unsigned int StoreConfigurationFailedDueToTimeOutUInt = 259;
const unsigned int StoreConfigurationFailedDueToUnexpectedResponseUInt = 260;
const unsigned int SelectProfileFailedDueToTimeOutUInt = 261;
const unsigned int SelectProfileFailedDueToUnexpectedResponseUInt = 262;
const unsigned int RingIndicationUnexpectedResponseUInt = 263;
const unsigned int OK_SearchTimedOutUInt = 264;
const unsigned int OK_UnexpectedResponseUInt = 265;
const unsigned int SignalStrengthTooLowUInt = 269;
const unsigned int TransmitSuccessfulButReceiveFailedUInt = 272;
const unsigned int UnexpectedModemCommandUInt = 273;
const unsigned int MPM_Busy_TransmitCommandRejectedUInt = 274;
const unsigned int PingToMPM_TimedOutUInt = 275;
const unsigned int PingToMPM_SuccessButToModemFailedUInt = 276;
const unsigned int InitialSetupModemStatusValueUInt = 278;
const unsigned int TimeOutWaitingForgetReceivedDataUInt = 279;
const unsigned int NoPingMPM_BusyUInt = 280;
//const unsigned int ModemFailedAtSetupUInt = 281;
const unsigned int UnexpectedResponseFromModemDuringSetupUInt = 282;
const unsigned int ModemSetupFailedBecauseMPM_BusyUInt = 283;
const unsigned int ModemFailedAtSetupTimeOutUInt = 284;//added for completeness. This error only detected in FDR
const unsigned int MPM_BusyWhenFDR_AskedForSetupUInt = 285;
const unsigned int MPM_DidNotRespondToRequestForDataUInt = 286;
const unsigned int SoftwareError1UInt = 287;
const unsigned int MPM_BusyUInt = 288;
const unsigned int  AskedForPingResultTooSoon_DoPingAgainUInt = 289;
const unsigned int PingToMPM_DidNotRespondUInt = 290;
const unsigned int WrongModemConnectedCheckSerialNumberUInt = 291;
const unsigned int RequestedTransmitTooSoonUInt = 292;
const unsigned int NoFunctioningModemPresentUInt = 293;
const unsigned int TimeOutAfterSendingMessageUInt = 294;
const unsigned int SBD_MessageTimeOutByModemUInt = 295;
const unsigned int SBD_MessageChecksumWrongUInt = 296;
const unsigned int SBD_MessageSizeWrongUInt = 297;
const unsigned int UnexpectedResponseAfterWritingDataToMobileOriginatedBufferUInt = 298;
const unsigned int SBD_MessageSizeTooBigOrTooSmallUInt = 299;
/************************************************
S T A T U S  R E T U R N S
************************************************/
const unsigned int SuccessByteAfterSBDIX_UInt = 300;
const unsigned int MessageSizeAcceptedUInt = 301;
const unsigned int MO_BufferClearedSuccessfullyUInt = 302;
const unsigned int MT_BufferClearedSuccessfullyUInt = 303;
const unsigned int OK_FoundUInt = 304;
const unsigned int RingIndicationDisabledUInt = 305;
const unsigned int SetupSuccessfulUInt = 306;
const unsigned int MT_MessageRetrievedCorrectlyUInt = 307;
const unsigned int MT_MessageIsNullUInt = 308;
const unsigned int ModemSetupSuccessfulUInt = 309;
const unsigned int CorrectModemConnectedUInt = 310;
//const unsigned int idleUInt = 311;
const unsigned int  NetworkAvailableWithAcceptableSignalStrengthUInt = 313;
const unsigned int VerifyDisableMT_AlertUInt = 314;
const unsigned int FlushedUART_BufferUInt = 315;
const unsigned int ArraySentToModemUInt = 316;
const unsigned int InitiateTransmitAndReceiveUInt = 317;
const unsigned int TellModemToClearMO_BufferUInt = 318;
const unsigned int TellModemToClearMT_BufferUInt = 319;
const unsigned int ToldModemToGiveUsTheReceivedMessageUInt = 320;
const unsigned int TransmissionProcessHasBegunUInt = 322;
const unsigned int SentPerformTransmitUInt = 323;
const unsigned int SentgetReceivedDataUInt = 324;
const unsigned int AboutToStartTransmitProcessUInt = 326;
const unsigned int WaitingForOK_FromModemUInt  = 327;
//const unsigned int InitialReturnCodeValueUInt  = 328;
//const unsigned int InitialMPM_ResponseValueUInt = 329;
const unsigned int BusySettingUpModemUInt = 330;
const unsigned int PerformingPingUInt = 331;
const unsigned long PingNotRunningUInt = 333;
const unsigned int MPM_Busy_TransmitCommandPendingUInt = 334;
const unsigned int ModemSetupProceedingUInt = 335;
const unsigned int ModemDefaultsSetUInt = 336;
const unsigned int SentPingUInt = 337;
const unsigned int SBD_MessageSuccessfullyWrittenUInt = 338;
const unsigned int MT_MessagePendingUInt = 339;
const unsigned int MT_MessagesPendingUInt = 340;
//const unsigned int NetworkStatusAndSignalStrengthUnavailableDueToModemBugUInt = 341;
/************************************************
S U C C E S S  R E T U R N S
************************************************/
/*
const unsigned int PingThroughMPM_AndModemSuccessUInt = 400;
const unsigned int ModemReadyForUseUInt = 401;
const unsigned int TransmitSuccessfulAndNoReceiveUInt = 402;
const unsigned int TransmitAndReceiveSuccessfulUInt = 403;
const unsigned int TransmitAndReceiveSuccessfulPlusReceivePendingUInt = 404;
const unsigned int dataLoopAroundEnabledUInt = 405;
const unsigned int dataLooopAroundDisabledUInt = 406;
const unsigned int receiveDataPlacedInReceiveArrayUint = 407;
*/
/************************************************
COMMANDS FROM MPM
************************************************
************************************************
MODEM COMMANDS
*************************************************/
//const unsigned int PingUInt = 0;//defined in FDR.ino
const unsigned int SetUpModemUInt = 1;//upper byte holds modem User RB serial numbers
//const unsigned long PerformTransmitUInt   = 2;
//const unsigned long getReceivedDataUInt  = 3;
const unsigned int SendBackFirstBlockOfReceivedArrayUInt = 4;
const unsigned int SendBackSecondBlockOfReceivedArrayUInt = 5;
//const unsigned int setLoopAroundUInt = 6;
//const unsigned int clearLoopAroundUint = 7;
const unsigned int statusUInt =	8;

/**********************************************************
V A R I A B L E S    
***********************************************************/
//byte _IridiumTransmitDataByte[45] = {4,5};//used by Iridium.ino   IridiumTransmitDataByte[45] is already defined
byte PreviousIridiumTransmitDataByte[45] = {6,7};
byte _IridiumReceivedDataByte[45] = {8,9};//used by Iridium.ino
byte i;
byte IndexByte;
byte MOstatusByte = 0;
unsigned int MOMSNUInt = 0;
byte MTstatusByte = 0;
unsigned int MTMSNUInt = 0;
byte MTlengthByte = 0;
byte MTqueuedByte = 0;
byte TextIndexByte = 0;
String BufferText = "";
unsigned int SetupModemStatusUInt = InitialSetupModemStatusValueUInt;
volatile unsigned int ReturnCodeUInt = InitialReturnCodeValueUInt;//viewable by FDR; had unsigned int in front it is defined in Fdr.h 
unsigned int LocalReturnCodeUInt = InitialReturnCodeValueUInt;//not viewable by FDR
unsigned int ModemSetUpReturnCodeUInt = InitialReturnCodeValueUInt;
const unsigned int NoActiveModemCommandUInt = 10;//only Iridium.ino uses it.
unsigned int StatusForFDR_UInt = 0;
unsigned long TransmissionRecycleTimeULong = 0;//FDR can transmit data via Iridium no faster than once per minute in order to limit the cost. was 60000 but for test code changed to 0

unsigned long core1StartForTimeStampULong = millis();//I reset this clock when ping arrives so MPM and FDR have their time stamps in sync
unsigned long StartTimeULong = 0;
char character;
unsigned long TimeLimitULong = 0;
unsigned long CycleStartTimeULong = 0;
unsigned long PingStateUInt = PingNotRunningUInt;
byte BusTestCount = 0;
//unsigned int noActiveModemCommandUInt = 888;   duplicate
//unsigned int NewModemCommandUInt = noActiveModemCommandUInt;
byte nearFarmodemSNbyte;//value set in FDR.ino and passed here hidden in the high byte of SetUpModemUInt 
String IMEIstring;//will hold the near modem's IMEI number which is checked against the one sent back from the modem as part of modem setup
unsigned long farRBsnULong;//will hold the far modem's RockBLOCK serial number printed on the modem. This number takes up to 3 bytes.
/***********************************************************/
unsigned long StartTime = millis(); //test code 
/***********************************************************/
unsigned long heartbeatTimeULong = millis();//used to display heartbeat on TX LED built into Pro Micro 
//unsigned long LastLoop1TimeStampMSULong = millis();
bool toggleBool = true;
bool core1_separate_stack = true;//gives each core its own 8K stack
//unsigned long Loop1CycleTimeMSULong = 0;

void setup1()
{
	Serial2.setFIFOSize(128);//increase serial buffer to prevent overflow
	Serial2.begin(19200); // 19200  Baud rate set to match that of the RockBLOCK.
	
	//while(1)Serial2.print("A");//used for analog testing of TX1
	
	//set up I2C
	pinMode(ExternalLED, OUTPUT); 	
	
	//DirectPath(false);//true means I want HEX output. Once entered, we stay in it so no need to comment out rest of code.

					// diagnostic code
					//unsigned long TimeNow = (millis()-StartTime)/1000;
					#ifdef DiagPrint1
					unsigned long TimeNow = (millis()-StartTime)/1000;
					Serial.println(__LINE__);
					Serial.print(F(" lapse time is "));
					Serial.println( TimeNow );
					#endif
	
	StartTime = millis();			
						
					#ifdef pingTest1
					delay(20000);
					Serial.println(__LINE__);// diag
					Serial.println(F("******start of core1******"));
					#endif
	

	long startTimeForWaitLong = millis();
	while((!fdr.areWeDoneWithStartUpDiagQ()) && (millis() - startTimeForWaitLong < 10000));//hold up loop1() starting until either the Start up diag is done (measured 7 seconds) or it has been 10 second. The start up diag determines if we should write to EEPROM or to the simulator. The 10 second timeout is so we don't hang if core 0 has a problem and the startup doesn't complete.
	
}//end of setup1()


void loop1()
{
	
						#ifdef I2C_conflictTest
						while(1)
						{
							I2Cbus1OwnerByte = 1;
							delay(1000);//this should trigger a real time error message saying that user 1 held the bus too long.
						}
						#endif
											
						#ifdef testTranslate
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCode is "));
						Serial.print(ReturnCodeUInt);
						Serial.println(" which translates to: ");
						fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
						#endif
											
						#ifdef modemsetupTrace1
						//if(ModemCommandUInt == versionQUInt)
						//{
							Serial.print("core ");
							Serial.print(rp2040.cpuid()); 
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							Serial.print("ModemCommandUInt is ");
							Serial.println(ModemCommandUInt);
							delay(1000);
						//}
						#endif
	/*
	unsigned long core1StartForTimeStampULong = millis();
	while(1)
	{		
		delay(100);
		
		Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
		Serial.print(F("(): "));
		Serial.print(__LINE__);
		Serial.print(F(". TS  "));	
		Serial.println(millis() - core1StartForTimeStampULong);
	}									
	*/								
					
						#ifdef noKB1
						Serial.print(F("line number "));
						Serial.println(__LINE__);
						Serial.print(F("BufferText = "));
						Serial.println( BufferText );
						Serial.print(F("IMEIstring = "));
						Serial.println( IMEIstring );
						#endif	
	
						#ifdef uartTest	
							unsigned int count = 0;
							bool failure = false;
							while(!failure)
							{//diag stress test for hardware to modem
						
								Serial.println(F("about to send AT\r"));
								Serial2.print(F("AT\r")); 
								delay(20);
								if(TestForOK() == OK_FoundUInt)
								{
									Serial.print("ATP for ");
									count++;
									Serial.print(count);
									Serial.println(" times in a row.");
								}else
								{
									count = 0;
									Serial.println("Encountered a failure so exit test.");
									failure = true;
								}
								delay(1000);							
							}
						#endif	


				#ifdef frozen11
				fdr.ReadControlBlock(1);//dummmy to get core 1 to own bus
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif


	flashLED();//flashes external LED to say loop1() is cycling. If core0 freezes, send unique pattern.
  
				
				#ifdef frozen1
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
	
	core1Heartbeat();//if core0 is not involved in a data dump, core1Heartbeat() checks that core1 is responsive. If not, it fast flashes external LED via flashLED(). 
	
				
				#ifdef frozen1
				Serial.print("core ");
				Serial.print(rp2040.cpuid()); 
				Serial.print(" ");
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print("ModemCommandUInt is ");
				Serial.println(ModemCommandUInt);
				#endif	
				
	modemCommands();
	
						#ifdef frozen1
						
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis() - core1StartForTimeStampULong);
						
					#endif
			
				
				#ifdef frozen1
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif						
	
	fdr.Loop1CycleTimeMonitor(false);//Any user can inhibit this monitor for one cycle of loop1() by calling fdr.inhibitLoop1TimerWarning(). They can enable the monitor by calling fdr.enableLoop1TimerWarning() and can see if the monitor is enabled by calling fdr.isLoop1TimerActiveQ(). When it takes more than maximumLoop1CycleTimeMS_UInt (1 second) to complete one cycle of loop1(), we print a warning and log the problem in the control block.
	
	I2CstateMachines();//students put their I2C drivers here so they do not disturb realtime. Function is defined in FDR.ino.
	
			#ifdef CBprintTestScript
			Serial.println();
			byte testCBreadByte = fdr.ReadControlBlock(1);
			delay(5);//prevent jumble
			Serial.print("core ");
			Serial.print(rp2040.cpuid()); 
			Serial.print("    ");
			Serial.println();
			Serial.print("fdr.ReadControlBlock(1) returned ");
			Serial.println(testCBreadByte);//qaz diag
			Serial.println();
			#endif
			
			fdr.Port6Peak();//read the high speed port every cycle to supplement reads from core 0
		
}//end of loop1()

void TellModemWeWillBeSending45BytesPlusOverhead() 
/*tells modem to expect a total of 50 bytes of data to be sent from the Pro Micro. Follow this subroutine with GetResponseFromModemAfterSendingMessageSize()
reference: page 95
*/
{
  FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
  Serial2.print("AT+SBDWB=50\r"); //tell modem we are sending 45 bytes of data plus 5 bytes of overhead. Overhead includes "RB" and a 3 byte serial number of the termination modem
  delay(DelayAfterDoingPrintToModemUInt);
}


void WriteDataToMobileOriginatedBuffer()
{//IridiumTransmitDataByte[] was defined in setup and this is the array that is passed as a global array. After being used, it is saved as PreviousIridiumTransmitDataByte[45]. Before doing a transmit, we check for this pattern. If seen, we reject command in order to prevent sending the same data twice in a row. farRBsnULong is the 
	int checkSum = 0;
	//start sending SBD message as bytes to modem starting with the letters "RB" for RockBLOCK...
	Serial2.write(0x52);//ASCII for "R"
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.write(0x42);//ASCI for "B"
	delay(DelayAfterDoingPrintToModemUInt);
	// and followed with serial number in binary.
	
	
	//Serial number for the ground based modem is 3 bytes. For example, far modem could be marked RockBLOCK edc (= 0x35611).
	delay(DelayAfterDoingPrintToModemUInt);	
	//farRBsnULong contains the top, middle and low bytes of the RBsn.
	byte topByteSNbyte = (farRBsnULong & 0x00FF0000)>>16;//farRBsnULong is 4 bytes but SN is only lower 3 bytes. I mask off all but byte 2 and then shift it down 16 bits which is 2 bytes. This puts the top byte into the MSB position.
	byte middleByteSNbyte = (farRBsnULong & 0x0000FF00)>>8;//this time I want byte 1 so mask off all but byte 1 and shift result over 8 bits
	byte lowByteSNbyte = (farRBsnULong & 0x000000FF);//this time I want byte 0 so no shift after masking off lowest byte 
	
	
			#ifdef nearFarTest
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print(F("farRBsnULong = "));
			Serial.println(farRBsnULong,HEX);
			Serial.print(F("topByteSNbyte = "));
			Serial.println(topByteSNbyte,HEX);
			Serial.print(F("middleByteSNbyte = "));
			Serial.println(middleByteSNbyte,HEX);
			Serial.print(F("lowByteSNbyte = "));
			Serial.println(lowByteSNbyte,HEX);
			#endif
	
	//populate MO buffer with 3 byte RB s/n 
	Serial2.write(topByteSNbyte);
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.write(middleByteSNbyte);
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.write(lowByteSNbyte);
	delay(DelayAfterDoingPrintToModemUInt);	
	
	
	checkSum += 0x52 + 0x42 + topByteSNbyte + middleByteSNbyte + lowByteSNbyte;//RB + 3 byte RB s/n 
	
			#ifdef nearFarTest
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			#endif
			
	
	for(int i = 0; i < 45; i++)
	{

			#ifdef nearFarTest22
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			#endif
		
		Serial2.write(IridiumTransmitDataByte[i]); //send next byte of data to modems Mobile Origination buffer
		delay(DelayAfterDoingPrintToModemUInt);
		checkSum += IridiumTransmitDataByte[i]; //add the data to the running sum. Note that a byte is being added to an integer						
	}	
		
	// Writing the check sum to the MO register 	
	Serial2.write(highByte(checkSum));
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.write(lowByte(checkSum));
	delay(DelayAfterDoingPrintToModemUInt);
	//all 50 bytes should be in the MO buffer now	
	ReturnCodeUInt = ArraySentToModemUInt;
	
				#ifdef nearFarTest
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(". TS  "));	
				Serial.println(millis() - core1StartForTimeStampULong);
				Serial.print(F(" checkSum = "));
				Serial.println( checkSum );
				#endif

}//end of WriteDataToMobileOriginatedBuffer()


void SendMessageInMobileOriginatedBuffer()//initiate transmit and receive exchange with satellite
{ //use this when not responding to a ring 
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	Serial2.print("AT+SBDIX\r");
	//Serial.println("AT+SBDIX\r");
	delay(DelayAfterDoingPrintToModemUInt);
	ReturnCodeUInt = InitiateTransmitAndReceiveUInt;
	
				#ifdef DiagPrint1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(". TS  "));	
				Serial.println(millis() - core1StartForTimeStampULong);
				#endif

}


void GetResponseFromModemAfterSendingMessage()
{//it handles the response after AT+SBDIX has been sent to the modem
/***************************************************
R e t u r n   c o d e  d e s c
----------------------------------------------------
	UnexpectedMT_SBD_MessageLengthUInt						
	MO and/or MT messages, if any, transferred successfully
	TimeOutAfterSBDIX_UInt give up after waiting TimeLimitULong ms
	ModemFailureAfterSBDIX_UInt something went wrong with the modem; see MOstatus and MTstatus for details
	UnexpectedResponseToSBDIXUInt response from modem not related to SBDIX command
	
****************************************************/
//Command Response: +SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>
//See 5.144 of the ISU AT Command Reference, version 2
//if MO status has a value other than 0, we have a failure.
	
	//byte number; //used when reading numbers
	// "+SBDIX:10,23456,7,45678,50,2" is test string
	//max number of characters is 29 plus ends with NULL
	BufferText="";//initialize string to zero length
	long FieldValueLong = 0;
	TimeLimitULong = 40000; //in milliseconds
	StartTimeULong = millis();
	
	while(1)
	{//keep looking until start of response found or we run out of time
		
		Serial.println(__LINE__);
					
		while(Serial2.available() == 0)
		//look for bytes in buffer until either we find some or run out of time
		{			
			if((millis() - StartTimeULong) > TimeLimitULong) 
			{
				ReturnCodeUInt = TimeOutAfterSBDIX_UInt;//no bytes Received within TimeLimitULong ms
				return;
			}
		}
		
						#ifdef DiagPrint1
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
					
		while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
		{		
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response

			noInterrupts();//diag
			character = Serial2.read(); //read back one character of the response from SBDWB command
			interrupts();//diag

			
			//character = Serial2.read(); //read back one character of the response from SBDIX command
			
						#ifdef DiagPrint1
						Serial.println(__LINE__);
						Serial.print(F(" character in hex is "));
						Serial.println( character, HEX);
						Serial.print(F(" BufferText string is "));
						Serial.println( BufferText );
						#endif
			
			BufferText.concat(character);//build up BufferText string  for testing
		}//have recorded entire string but must add "," to the end
		
					#ifdef DiagPrint1
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.println(millis() - core1StartForTimeStampULong);
					#endif
					
		BufferText.concat(","); //ExtractField() needs a "," at the end of each field. I add "," at the end of the string because the last field didn't have one
	
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.println(F(" BufferText string is:"));
					Serial.println( BufferText );
					#endif
	
		//see Iridium docs, page 120 for command response
		if (BufferText.indexOf("+SBDIX:") >= 0) 
		{//we have found "+SBDIX:" which is the start of the command response for SDBIX. I assume entire response is now in buffer
		//+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.println(F(" +SBDIX: found "));
					#endif
			
			TextIndexByte = BufferText.indexOf("+SBDIX:") + 7; //first character position after +SBDIX:
			//note that +SBDIX: does not have to be at the start of the string but the fields must follow it although embedded blanks will be ignored. The indexOf is the start of the substring and "+SBDIX:" is 7 characters
			
			
					#ifdef DiagPrint1
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.println(millis() - core1StartForTimeStampULong);
					#endif
			
			//Mobile Origination Status
			
						
			FieldValueLong = ExtractField(2); //max number of digits is 2
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MO status raw field value = "));
					Serial.println( FieldValueLong );
					#endif
					
			if ( FieldValueLong < 0)
			{
				ReturnCodeUInt = UnexpectedMO_StatusValueUInt;
				return;
			}
			
			MOstatusByte = lowByte(FieldValueLong);
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MOstatusByte is "));
					Serial.println( MOstatusByte );
					#endif
			
			if(MOstatusByte > 0)
			{//See 5.144 in IRDM docs. This is the newer version of the response
				ReturnCodeUInt = MOstatusByte;
				return;
			}
			
			//Mobile Originated Message Sequence Number (MOMSN)
			
					#ifdef DiagPrint1					
					Serial.println(__LINE__);
					Serial.print(F(" TextIndexByte = "));
					Serial.println( TextIndexByte );
					#endif
			
			FieldValueLong = ExtractField(5);//max number of digits is 5
			if (FieldValueLong < 0)
			{
				ReturnCodeUInt = UnexpectedMOMSN_ValueUInt;
				return;
			}
			
			MOMSNUInt = FieldValueLong; //implicit conversion from a long to an unsigned integer
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MOMSNUInt is "));
					Serial.println( MOMSNUInt );
					#endif
						
			//Mobile Terminated (MT) status

			Serial.println(__LINE__);
	
						
			FieldValueLong = ExtractField(1);//max number of digits is 1
			if (FieldValueLong < 0)
			{
				ReturnCodeUInt = UnexpectedMT_StatusValueUInt;
				return;
			}
			
			MTstatusByte = lowByte(FieldValueLong);
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MTstatusByte is "));
					Serial.println( MTstatusByte );
					#endif
			
			//Mobile Terminated Message Sequence Number (MTMSN)
			
			FieldValueLong = ExtractField(5);//max number of digits is 5
			if (FieldValueLong < 0)
			{
				
				ReturnCodeUInt = UnexpectedMTMSN_StatusValueUInt;
				return;
			}
			
			MTMSNUInt = FieldValueLong; //implicit conversion from long to unsigned integer
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MTMSNUInt is "));
					Serial.println( MTMSNUInt );
					#endif
			
			//Mobile Terminated SBD message length (in bytes)
						
			FieldValueLong = ExtractField(2);//max number of digits is 2
			if (FieldValueLong < 0)
			{
				ReturnCodeUInt = UnexpectedMT_SBD_MessageLengthUInt;
				return;
			}
			
			MTlengthByte = lowByte(FieldValueLong); 
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MTlengthByte is "));
					Serial.println( MTlengthByte );
					#endif			
			
			//Mobile Terminated SBD messages queued in server.

						
			FieldValueLong = ExtractField(1);//max number of digits is 1
						
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" FieldValueLong = "));
					Serial.println( FieldValueLong );
					#endif
			
			if (FieldValueLong < 0)
			{
				ReturnCodeUInt = UnexpectedMT_SBD_MessageQueuedValueUInt;
				return;
			}
			
			MTqueuedByte = lowByte(FieldValueLong); 
			
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" MTqueuedByte is "));
					Serial.println( MTqueuedByte );
					#endif
			
			if(MTqueuedByte == 1)ReturnCodeUInt = MT_MessagePendingUInt;
			
			if(MTqueuedByte > 1)ReturnCodeUInt = MT_MessagesPendingUInt;
						
		}else{
			//unexpected response
			ReturnCodeUInt =  UnexpectedResponseToSBDIXUInt;
			return;	
		}
		
		//Now have parced response so ready to determine overall status
		
//MOstatusByte = 0 means MO message, if any, transferred successfully. 

				#ifdef DiagPrint1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(". TS  "));	
				Serial.println(millis() - core1StartForTimeStampULong);
				Serial.print(F(" MOstatusByte = "));
				Serial.println( MOstatusByte );
				Serial.print(F(" MTstatusByte = "));
				Serial.println( MTstatusByte );
				Serial.print(F(" MTqueuedByte = "));
				Serial.println( MTqueuedByte );
				#endif
		
		if((MOstatusByte == 0) && (MTstatusByte == 0))
		{
			ReturnCodeUInt =  TransmitSuccessfulAndNoReceiveUInt;
			return;
		}
		
		if((MOstatusByte == 0) && (MTstatusByte == 1) && (MTqueuedByte == 0))
		{//MOstatusByte of 0 means SBD message successfully received from the server. MTstatusByte of 1 means an SBD message was successfully received from the server. MTqueuedByte = 0 means no messages queued in server 
			ReturnCodeUInt =  TransmitAndReceiveSuccessfulUInt;
			return;
		}
		
		
		if((MOstatusByte == 0) && (MTstatusByte == 1) && (MTqueuedByte > 0))
		{//MOstatusByte of 0 means SBD message successfully received from the server. MTstatusByte of 1 means an SBD message was successfully received from the server. MTqueueByte > 0 means one or more messages are waiting to be sent up to the Payload.
			ReturnCodeUInt =  TransmitAndReceiveSuccessfulPlusReceivePendingUInt;
			return;
		}
	
		
		if(MOstatusByte > 0)
				{
			ReturnCodeUInt =  MOstatusByte; //MTstatusByte > 0 is an error code and goes as high as 65.
			return;
		}
	
	ReturnCodeUInt =  FailureAfterSBDIX_UInt;//I think I cover all possible values for MOstatusByte and MTstatusByte but just in case I missed something, we will return with this. 
	return;
	} //bottom of while(1) so return to top and check timer first
}//end of GetResponseFromModemAfterSendingMessage()

long ExtractField(byte MaxNumberCharactersByte)
{
/*The string "BufferText" contains the characters from the modem. Using TextIndexByte to find the first character and "," to mark the end of the field, this subroutine converts each numeric ASCII character into its decimal equivalent and then combines them into an integer which is returned. TextIndexByte is advanced to start of next field. Any blanks found within a field are ignored but the character index is advanced.

The subroutine returns a long because two of the fields return unsigned integers and that won't permit me to pass values <0 as error flags. A long can carry the largests value stored in an unsigned int yet also have negative numbers. 
************************************************************
R E T U R N   C O D E S
************************************************************
			Description							value
MoreThantheMaxNumberCharactersByteWasProcessedExtractFieldLong   -1
NonNumericCharacterWasFoundBeforeTheCommaExtractFieldLong   -2
NoNumbersFoundBeforeTheCommaExtractFieldLong    -3
integer		decimal value of the field (always >= 0)
************************************************************/

	byte CharacterCountByte = 0;
	long FieldContentLong = 0;
	while(BufferText[TextIndexByte] != 0x2C)
	{ //while we don't have a "," we collect ASCII characters
			
			#ifdef DiagPrint1
			if (TextIndexByte > 10){
			Serial.println(__LINE__);
			Serial.print(F(" TextIndexByte = "));
			Serial.println( TextIndexByte );
			Serial.print(F("BufferText[TextIndexByte] in hex = "));
			Serial.println( BufferText[TextIndexByte], HEX );
			}
			#endif
			
		if((BufferText[TextIndexByte] >= 0x30) && (BufferText[TextIndexByte] <= 0x39))
		{//we have an ASCII numeric so convert it to a decimal and add to total.
		
			#ifdef DiagPrint1
			if( TextIndexByte > 21)
			{
			Serial.println(__LINE__);
			Serial.print(F(" FieldContentLong = "));
			Serial.println( FieldContentLong );
			}
			#endif
				
			FieldContentLong = (FieldContentLong*10) + (long(BufferText[TextIndexByte] - 0x30)); //move last digit to left and add new digit; The ASCII charcter for zero is 0x30 so by subtracting 0x30 we get the decimal equiv. for the corresponding number.
			
			#ifdef DiagPrint1	
			if( TextIndexByte >21){
			Serial.println(__LINE__);
			Serial.print(F(" FieldContentLong = "));
			Serial.println( FieldContentLong );
			}
			#endif
				
			CharacterCountByte = CharacterCountByte +1; //increment the count of characters being summed but if a blank is found, do not count it in the Character Count
		}
		TextIndexByte = TextIndexByte + 1; //prepare to access next character position 
		
		if(CharacterCountByte > MaxNumberCharactersByte) return MoreThantheMaxNumberCharactersByteWasProcessedExtractFieldLong;//max field size searched yet no "," found 
		//ready to process next character
	}
	if (CharacterCountByte <1)
	{
		return NoNumbersFoundBeforeTheCommaExtractFieldLong;
	}else{
		TextIndexByte = TextIndexByte + 1; //at end of field so move pointer to first digit in next field 
		return FieldContentLong;
	}
}

void GetResponseFromModemAfterSendingMessageSize()
{//it handles the response after AT+SBDWB has been sent to the modem
/***************************************************
R e t u r n   c o d e   
----------------------------------------------------
MessageSizeAcceptedUInt
TimeOutAfterSendingMessageSizeUInt	  no response within TimeLimitULong ms
SBD_MessageSizeTooBigOrTooSmallUInt
UnexpectedResponseAfterSendingMessageSizeUInt   
****************************************************/
	TimeLimitULong = 20000; //in milliseconds
	//See pg 95
	BufferText="";//initialize string to zero length
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	while (Serial2.available() == 0)
	{
		if((millis() - StartTimeULong) > TimeLimitULong)
		{
			ReturnCodeUInt = TimeOutAfterSendingMessageSizeUInt;//no response Received within TimeLimitULong ms
			return;
		}
	}
	
	while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
	{		
		delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
		
		noInterrupts();//diag
		character = Serial2.read(); //read back one character of the response from SBDWB command
		interrupts();//diag
		
		
		//character = Serial2.read(); //read back one character of the response from SBDWB command
					
		#ifdef DiagPrint1
		Serial.println(__LINE__);
		Serial.print(F(" character in hex is "));
		Serial.println( character, HEX);
		#endif
								
		BufferText.concat(character); //build up BufferText string
	}//have recorded entire string
					
	#ifdef DiagPrint11
	Serial.println(__LINE__);
	Serial.print(F(" entire BufferText string is "));
	Serial.println( BufferText );
	#endif
									
	if (BufferText.indexOf("READY") >= 0)
		{
			ReturnCodeUInt = MessageSizeAcceptedUInt;
			return;
		}

	if (BufferText.indexOf("3") >= 0)
	{
		ReturnCodeUInt = SBD_MessageSizeTooBigOrTooSmallUInt;
		return;
	}
	//if we got here, response is unexpected so drop an error code 
	ReturnCodeUInt = UnexpectedResponseAfterWritingDataToMobileOriginatedBufferUInt;
	return;
}//end of GetResponseFromModemAfterSendingMessageSize()

void ClearMO_MessageBuffer()
{
/*
After collecting string from Mobile Originated buffer, we clear buffer. pages 82 and 123
*/

  FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
  SetToNumericResponses();//in order to get the complete response, I must be in Numeric Response mode. I'll return to Textural Response mode when done reading result.
  Serial2.print("AT+SBDD0\r"); //tell modem to clear the MO buffer. Follow this subroutine with GetResponseFromClearMO_MessageBuffer()
  delay(DelayAfterDoingPrintToModemUInt);
  ReturnCodeUInt = TellModemToClearMO_BufferUInt;
} 


void GetResponseFromClearMO_MessageBuffer()
{//it handles the response after AT+SBDD0 has been sent to the modem. On pg 82 it says the response should be a 0 for success or a 1 for failure. This is true only in numeric mode which is set by "AT V0\r". I then get <cr><lf>OK<cr><lf> 0 <cr> which must mean  buffer cleared successfully. <cr><lf>OK<cr><lf> 1 <cr> must mean failure. When done, I change back to Textual Responses mode with AT V1\r. 
/***************************************************
R e t u r n   c o d e s    		 
----------------------------------------------------
MO_BufferClearedSuccessfullyUInt	
MO_BufferClearedErrorResponseUInt 	
MO_BufferClearedTimeOutUInt			  
MO_BufferClearedUnexpectedResponseUInt	
****************************************************/
	TimeLimitULong = 20000; //in milliseconds	
	//byte number; //used when reading numbers
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem
	BufferText = "";
	while (Serial2.available() == 0)
	{
		if((millis() - StartTimeULong) > TimeLimitULong) 
		{
			ReturnCodeUInt = MO_BufferClearedTimeOutUInt;//no response Received within TimeLimitULong ms
			return;
		}
	}
	
	while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
	{		
		delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
		
			noInterrupts();//diag
			character = Serial2.read(); //read back one character of the response from SBDWB command
			interrupts();//diag
		
		
		//character = Serial2.read(); //read back one character of the response from SBDWB command
		
		#ifdef DiagPrint1
		Serial.println(__LINE__);
		Serial.print(F(" character in hex is "));
		Serial.println( character, HEX);
		#endif
					
		BufferText.concat(character); //build up BufferText string			
	}//have recorded entire string in BufferText.
	SetToTextualResponses();//switch back to Textual Responses. Was set to numeric response in ClearMO_MessageBuffer().	
		
	if (BufferText.indexOf("1") >= 0) 
	{
		ReturnCodeUInt = MO_BufferClearedErrorResponseUInt;//MO buffer could not be cleared. This would be a modem fault.
		return;
	}
	
	if (BufferText.indexOf("0") >= 0) 
		{
			ReturnCodeUInt = MO_BufferClearedSuccessfullyUInt;
			return;
		}
	//zero means MO buffer was cleared successfully. 
	//to get here, I did not see 0 or 1
	
	ReturnCodeUInt = MO_BufferClearedUnexpectedResponseUInt;
	return;
}//end of GetResponseFromClearMO_MessageBuffer()

void ClearMT_MessageBuffer()
{
/*
After collecting string from Mobile Terminated buffer, we clear buffer. pages 82 and 123
*/

  FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
  SetToNumericResponses();//in order to get the complete response, I must be in Numeric Response mode. I'll return to Textural Response mode when done reading result.
  Serial2.print("AT+SBDD1\r"); //tell modem to clear the MT buffer. Follow this subroutine with GetResponseFromClearMT_MessageBuffer()
  delay(DelayAfterDoingPrintToModemUInt);
  ReturnCodeUInt = TellModemToClearMT_BufferUInt;
} 

unsigned int GetResponseFromClearMT_MessageBuffer()
{//it only handles the response after AT+SBDD1 has been sent to the modem. On pg 82 it says the response should be a 0 for success or a 1 for failure. This is true only in numeric mode which is set by "AT V0\r". I then get <cr><lf>OK<cr><lf> 0 <cr> which must mean  buffer cleared successfully. <cr><lf>OK<cr><lf> 1 <cr> must mean failure. When done, I change back to Textual Responses mode with AT V1\r. 
/***************************************************
R e t u r n   c o d e    		 
----------------------------------------------------
MT_BufferClearedSuccessfullyUInt	
MT_BufferClearedErrorResponseUInt 	
MT_BufferClearedTimeOutUInt			  
MT_BufferClearedUnexpectedResponseUInt	
****************************************************/
	TimeLimitULong = 20000; //in milliseconds
	//byte number; //used when reading numbers
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem
	BufferText = "";
	while (Serial2.available() == 0)
	{
		if((millis() - StartTimeULong) > TimeLimitULong) return MT_BufferClearedTimeOutUInt;//no response Received within TimeLimitULong ms
	}
	while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
	{		
		delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response

		noInterrupts();//diag
		character = Serial2.read(); //read back one character of the response from SBDWB command
		interrupts();//diag
		
		//character = Serial2.read(); //read back one character of the response from SBDWB command
		
		#ifdef DiagPrint1
		Serial.println(__LINE__);
		Serial.print(F(" character in hex is "));
		Serial.println( character, HEX);
		#endif
					
		BufferText.concat(character); //build up BufferText string			
	}//have recorded entire string in BufferText.
	SetToTextualResponses();//switch back to Textual Responses.
			
	#ifdef DiagPrint1
	Serial.println(__LINE__);
	Serial.print(F("BufferText is  "));
	Serial.println( BufferText );
	Serial.print(F(" BufferText.indexOf(0) = "));
	Serial.println( BufferText.indexOf("0") );
	#endif

	/*******************************************************************/	
	//As of 3/21/2024, the newer modems have a bug that returns AT+SBDD1 rather than 0 or 1. Until it is fixed, I will return MT_BufferClearedSuccessfullyUInt when I get this text.
	//if (BufferText.indexOf("AT+SBDD1") >= 0) return MT_BufferClearedSuccessfullyUInt;
	/*******************************************************************/
			
	if (BufferText.indexOf("1") >= 0) return MT_BufferClearedErrorResponseUInt;//MT buffer could not be cleared. This would be a modem fault. Test is of getting both OK and 1.
		
	if (BufferText.indexOf("0") >= 0) return  MT_BufferClearedSuccessfullyUInt;
		//zero means MT buffer was cleared successfully. Test is of getting both OK and 0.
	
		//to get here, I did not see 0 or 1
	return MT_BufferClearedUnexpectedResponseUInt;
}
	
void IdentifyModem()
{
/************************************************
R E T U R N   C O D E S
************************************************
NoModemConnectedUInt	
WrongModemConnectedCheckSerialNumberUInt		
CorrectModemConnectedUInt	
************************************************
The correct modem for the payload has an International Mobile Equipment Identity (IMEI) of written on modem. Only with the correct modem connected will the code talk to it.
************************************************/

	TimeLimitULong = 4000; //in milliseconds
	//IMEIstring should be the IMEI printed on the near modem. example: for modem 00 it is "300234066438070";
	BufferText = "";

	byte IMEI_DigitCountByte = 0; //limits read to length of IMEI because after IMEI we get cr lf zero cr.
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 

	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer

						#ifdef modemNGtest1
						Serial.println(__LINE__);
						Serial.print(F(" timestamp = "));
						Serial.println( millis() );
						#endif			
	//noInterrupts();//diag
	
	Serial2.print("AT+CGSN\r"); //Ask modem for its International Mobile Equipment Identity (IMEI). It will respond with 15 digit number followed by cr lf zero cr.
	delay(DelayAfterDoingPrintToModemUInt);

					
					#ifdef noKB1
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
			

	while (Serial2.available() == 0)
		{
			if((millis() - StartTimeULong) > TimeLimitULong)
			{
				
				Serial.println(__LINE__);
				
					#ifdef noKB1
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
				ReturnCodeUInt = NoModemConnectedUInt;
				return; //no response Received within TimeLimitULong ms means no modem connected
			}
					Serial.println(__LINE__);
					//see if we ever find no bytes in buffer
		}
		
	unsigned int readTimerStartUInt = millis();
	while((Serial2.available() > 0) && (IMEI_DigitCountByte < 15) && (millis() - readTimerStartUInt) < 2000) //keep reading stream from modem until all characters collected or we time out. 2 seconds is a swag
		{
					
						#ifdef modemNGtest
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif			
					
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response
			//delay(2);//diag
			
			noInterrupts();//diag
			character = Serial2.read();
			interrupts();//diag
		
						#ifdef modemNGtest
						 Serial.println(__LINE__);
						Serial.print(F(" character in hex = "));
						Serial.println( character, HEX );
						#endif	
								
			if((character >= 0x30) && (character <= 0x39))
			{	
			BufferText.concat(character);//read back one numeric character from the modem in ASCII
			IMEI_DigitCountByte++;
				
				/*
				Serial.println("IdentifyModem() has received this numeric charcter: ");
				Serial.println(character);
				*/
				
			}else
			{
				/*
				Serial.println("IdentifyModem() has received this nonnumeric charcter: ");
				Serial.println(character);
				Serial.print("and in HEX it is ");
				Serial.println(character,HEX);
				Serial.println();
				*/
				
						#ifdef modemNGtest1
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
			}
					
		} //IMEI now collected, non-numberics removed, and assembled into one number. It may be < 15 characters if we timed out or if some characters don't represent numbers in ASCII
		//interrupts();//diag
		
					#ifdef modemNGtest1
					Serial.print(F("line number "));
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.println(millis() - core1StartForTimeStampULong);
					#endif
				
	FlushUART_Buffer(); // toss cr lf zero cr
	
					#ifdef noKB1
					//while(1)
					//{
						Serial.print(F("line number "));
						Serial.println(__LINE__);
						Serial.print(F("BufferText = "));
						Serial.println( BufferText );
						//Serial.print(F("IMEIstring = "));
						//Serial.println( IMEIstring );
						delay(100);
					//}
					#endif			
				
	if (BufferText == IMEIstring)
		{
			ReturnCodeUInt = CorrectModemConnectedUInt;
			return; 
		}
	
					#ifdef sn	
						Serial.print(F("core0 "));
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(F("line number "));
						Serial.println(__LINE__);
						Serial.print(F("BufferText = "));
						Serial.println( BufferText );
						//Serial.print(F("IMEIstring = "));
						//Serial.println( IMEIstring );
						delay(100);
					#endif
				
	ReturnCodeUInt = WrongModemConnectedCheckSerialNumberUInt;	
	Serial.println("The wrong International Mobile Equipment Identity, ");
	Serial.println(BufferText);
	Serial.println("was read back from the modem.");
	Serial.println("We were expecting ");
	Serial.println(IMEIstring);
	return; 
}//end of IdentifyModem()

void IdentifyModemRevision()
{
/************************************************
This function asks the modem for its software Revision number and prints it to the MPM's serial monitor.
************************************************/

	TimeLimitULong = 20000; //in milliseconds
	BufferText = "";
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer								
	Serial2.print("AT+CGMR\r"); //Ask modem for its Revision number
	delay(DelayAfterDoingPrintToModemUInt);
	Serial.println();
	delay(3000);//give time for TT to come up
	Serial.println();
	Serial.print(F("Modem Revision "));
	while (Serial2.available() == 0)
		{
			if((millis() - StartTimeULong) > TimeLimitULong)
			{
				Serial.println(F("timed out"));							
				return;
			}
		}
	while(Serial2.available() > 0) //keep reading stream from modem until all characters collected and printed
		{			
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
			character = Serial2.read();
			Serial.print(character);		
		} 
	return;	
}//end of IdentifyModemRevision()

void IdentifyCurrentIndicatorEventReportingSettings()
{
/************************************************
This function asks the modem for its  current indicator event reporting settings and prints it to the MPM's serial monitor.
************************************************/

	TimeLimitULong = 20000; //in milliseconds
	BufferText = "";
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer								
	Serial2.print("AT+CIER?\r"); //Ask modem for its  current indicator event reporting settings
	delay(DelayAfterDoingPrintToModemUInt);
	Serial.println();
	Serial.print(F("current indicator event reporting settings are "));
	while (Serial2.available() == 0)
		{
			if((millis() - StartTimeULong) > TimeLimitULong)
			{
				Serial.println(F("timed out"));							
				return;
			}
		}
	while(Serial2.available() > 0) //keep reading stream from modem until all characters collected and printed
		{			
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
			character = Serial2.read();
			Serial.print(character);	
		} 
	return;	
}//end of IdentifyCurrentIndicatorEventReportingSettings()

void IdentifySupportedSettings()
{
/************************************************
This function asks the modem for its  List the supported settings.  and prints it to the MPM's serial monitor.
************************************************/

	TimeLimitULong = 20000; //in milliseconds
	BufferText = "";
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer								
	Serial2.print("AT+CIER=?\r"); //Ask modem for its list of supported settings 
	delay(DelayAfterDoingPrintToModemUInt);
	Serial.println();
	Serial.print(F("List the supported settings "));
	while (Serial2.available() == 0)
		{
			if((millis() - StartTimeULong) > TimeLimitULong)
			{
				Serial.println(F("timed out"));							
				return;
			}
		}
	while(Serial2.available() > 0) //keep reading stream from modem until all characters collected and printed
		{			
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
			character = Serial2.read();
			Serial.print(character);		
		} 
	return;	
}//end of IdentifySupportedSettings()

unsigned int  SetModemDefaults()
{
/************************************************
R E T U R N   C O D E S
************************************************
ModemReadyForUseUInt
RingIndicationErrononiouslyEnabledUInt
TimeOutAfterGetResponseFromVerifyDisableMT_AlertUInt		
************************************************
See page 122, section 8.2. Set default configuration to no flow control, SBD automatic notifications enabled. If any commands do not get a response of OK within TimeLimitULong ms, we return a failure.
************************************************/

	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.print("AT&K0\r"); //Disable RTS/CTS flow control
	delay(DelayAfterDoingPrintToModemUInt);
	
					#ifdef mysteryCall
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
		
	ReturnCodeUInt  = TestForOK();
	
						#ifdef mysteryCall
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCodeUInt is "));
						Serial.println( ReturnCodeUInt );
						#endif
	
	if (ReturnCodeUInt  == OK_SearchTimedOutUInt)return DisableFlowControlRequestTimedOutUInt;
	if (ReturnCodeUInt == OK_UnexpectedResponseUInt) return DisableFlowControlRequestUnexpectedResponseUInt;
//got "OK"
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.print("AT+SBDMTA=0\r"); //Disable SBD ring indication because it generates an autonomous response
	delay(DelayAfterDoingPrintToModemUInt);
	ReturnCodeUInt = TestForOK();
	
						#ifdef mysteryCall
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.print (millis() - core1StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print("return code is ");
						Serial.println(ReturnCodeUInt);
						#endif
			
	
	if (ReturnCodeUInt == OK_SearchTimedOutUInt)return DisableSBD_RingSetupFailedDueToTimeOutUInt; 
	
	if (ReturnCodeUInt == OK_UnexpectedResponseUInt)return DisableSBD_RingSetupFailedDueToUnexpectedResponseUInt;
	
					#ifdef mysteryCall
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
	
			
	VerifyDisableMT_Alert();//send "AT+SBDMTA?<cr>" and then can read UART buffer for answer			
	ReturnCodeUInt = GetResponseFromVerifyDisableMT_Alert();//there is no response to AT+SBDMTA=0 but I can send AT+SBDMTA? to read back state. Response is +SBDMTA:0 for ring disabled or +SBDMTA:1 for enabled which is the default.
	if (ReturnCodeUInt != RingIndicationDisabledUInt) return ReturnCodeUInt;
	
			#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print("return code is ");
			Serial.println(ReturnCodeUInt);
			#endif

	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.print("AT&W0\r"); //Store the configuration as profile 0
	delay(DelayAfterDoingPrintToModemUInt);

			#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print("return code is ");
			Serial.println(ReturnCodeUInt);
			#endif
			
	ReturnCodeUInt = TestForOK();
	
				#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print("return code is ");
			Serial.println(ReturnCodeUInt);
			#endif
	
	if (ReturnCodeUInt == 1)return StoreConfigurationFailedDueToTimeOutUInt; 
	if (ReturnCodeUInt == 2)return StoreConfigurationFailedDueToUnexpectedResponseUInt;
	//got "OK"
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.print("AT&Y0\r"); //Select profile 0 as the power-up default
	delay(DelayAfterDoingPrintToModemUInt);

			#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			#endif
	
	ReturnCodeUInt = TestForOK();
	
			#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print("return code is ");
			Serial.println(ReturnCodeUInt);
			#endif
	
	if (ReturnCodeUInt == OK_SearchTimedOutUInt)return SelectProfileFailedDueToTimeOutUInt; 
	if (ReturnCodeUInt == OK_UnexpectedResponseUInt)return SelectProfileFailedDueToUnexpectedResponseUInt;
//all commands successful
	return ModemReadyForUseUInt;
}//end of SetModemDefaults()


unsigned int  TestForOK()
{//state machine implementation
	byte state = 0;
	unsigned int OKstartTime = millis();	
	bool FoundOhBool = false;
							#ifdef snx 
							unsigned int diagTimerUInt = millis();
							#endif
	while(1)
	{
		switch(state)
		{
			case 0:
			//wait for a character or until we time out
				while (Serial2.available() == 0)
					{//while buffer empty, keep looking
						if((millis() - OKstartTime) > 500) 
						{
					
										#ifdef snx1 
										Serial.print(" core1: ");
										Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
										Serial.print(F("(): "));
										Serial.print(__LINE__);
										Serial.print(F(". TS  "));	
										Serial.println(millis() - core1StartForTimeStampULong);
										#endif
										
								#ifdef OKtimeTest
								Serial.println("Waited for OK for 500 ms and gave up.");
								#endif
								
							return OK_SearchTimedOutUInt;//no response Received within TimeLimitULong ms means no modem connected
						}
					}
				//1 or more characters have arrived before we timed out
				state = 1;//advance to looking for "O"
				break; //stay within the while(1)
			
			case 1:
				character = Serial2.read(); //read back one character of the response
				state = 2;
				
										#ifdef snx1 
										Serial.print(" core1: ");
										Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
										Serial.print(F("(): "));
										Serial.print(__LINE__);
										Serial.print(F(". TS  "));	
										Serial.println(millis() - core1StartForTimeStampULong);
										Serial.println("in state 1");
										Serial.print("character is ");
										Serial.println(character);
										Serial.print("and in HEX is ");
										Serial.println(character, HEX);
										#endif
				
				break; //stay within the while(1)
			
			case 2:
			
									#ifdef nsnx1 
									Serial.print(" core1: ");
									Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
									Serial.print(F("(): "));
									Serial.print(__LINE__);
									Serial.print(F(". TS  "));	
									Serial.println(millis() - core1StartForTimeStampULong);
									Serial.println("in state 2");
									#endif			
			
				if (character == 'O')
				{
					FoundOhBool = true;
					state = 0;//we go back to state 0 but it should not take 500 ms for second character to arrive.
					
										#ifdef snx1 
										Serial.print(" core1: ");
										Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
										Serial.print(F("(): "));
										Serial.print(__LINE__);
										Serial.print(F(". TS  "));	
										Serial.println(millis() - core1StartForTimeStampULong);
										Serial.println("found oh");
										#endif	
					
					break;
				}
				
				if(FoundOhBool && character == 'K')
				{
					//note that "O" and "K" must be present in this order but can have characters between them			
					FlushUART_Buffer();//don't leave trash in buffer
					
										#ifdef snx 
										Serial.print(" core1: ");
										Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
										Serial.print(F("(): "));
										Serial.print(__LINE__);
										Serial.print(F(". TS  "));	
										Serial.println(millis() - core1StartForTimeStampULong);
										Serial.print("found OK in ");
										unsigned int lapseTime = millis() - diagTimerUInt;
										Serial.print(lapseTime);
										Serial.println(" ms.");
										#endif					
					
								#ifdef OKtimeTest
								Serial.print("waited for OK for ");
								Serial.print(millis() - OKstartTime);
								Serial.println(" ms.");
								#endif
								
					return OK_FoundUInt;
				}
				//K without "O" first or some other character is ignored
				state = 0;//start over looking for "OK"
				break;
			
		}//end of state machine			
	}//end of while loop
}//end of TestForOK()	

void DirectPath(bool HexQ)
{
	//directly connects terminal emulator to modem
	while(1)
	{
		while(Serial2.available() > 0)
		{
			delay(DelayBeforeDoingReadFromModemUInt);
			character = Serial2.read();
			if (HexQ)
			{
				Serial.println(character, HEX);
			}else{
				Serial.print(character);
			}
		}
		while(Serial.available() > 0)
		{
			character = Serial.read();
			Serial2.print(character);
			delay(DelayAfterDoingPrintToModemUInt);
		}
	}
}


unsigned int  GetResponseFromVerifyDisableMT_Alert()
{//it handles the response after AT+SBDMTA? has been sent to the modem
/***************************************************
R e t u r n   c o d e    		d e s c
----------------------------------------------------
	RingIndicationDisabledUInt				
	RingIndicationErrononiouslyEnabledUInt				
	TimeOutAfterGetResponseFromVerifyDisableMT_AlertUInt					  no valid response within TimeLimitULong ms
****************************************************/

	//Command Response: +SBDMTA:<mode> where mode should be either 0 for disable ring indication or 1 for Enable ring indication (default)
	//See pg 90
	TimeLimitULong = 20000; //in milliseconds
	
	BufferText="";//initialize string to zero length
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	//We will look for the expected string until time is up and will tolerate the buffer temporarily going empty.
		while(Serial2.available() == 0)
		//look for any bytes in buffer until either we find some or run out of time
		{			
			if((millis() - StartTimeULong) > TimeLimitULong)return TimeOutAfterGetResponseFromVerifyDisableMT_AlertUInt;//no bytes Received within TimeLimitULong ms
			
			#ifdef mysteryCall
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.println(millis() - core1StartForTimeStampULong);
			#endif
												
		}
		//at least one byte came in
		StartTimeULong = millis(); 
		while((Serial2.available() > 0) && (millis() - StartTimeULong < 2000))//keep reading stream from modem until all characters collected or we timeout
		{		
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response
			noInterrupts();//diag
			character = Serial2.read(); //read back one character of the response from SBDWB command
			interrupts();//diag
			
			#ifdef mysteryCall //it floods printer so I turned it off
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print(millis() - core1StartForTimeStampULong);
			Serial.println(" ms");
			Serial.print(F(" character is "));
			Serial.println( character);
			Serial.print(F(" character in hex is "));
			Serial.println( character, HEX);
			#endif
						
			BufferText.concat(character); //build up BufferText string			
		}//have recorded entire string in buffer
		

			
		if (BufferText.indexOf("+SBDMTA:0") >=0)return RingIndicationDisabledUInt;
		if (BufferText.indexOf("+SBDMTA:1") >=0)return RingIndicationErrononiouslyEnabledUInt;
		
		#ifdef mysteryCall
		Serial.println(__LINE__);
		Serial.print(F(" entire BufferText string is "));
		Serial.println( BufferText );
		#endif
		
		return RingIndicationUnexpectedResponseUInt;
}//end of GetResponseFromVerifyDisableMT_Alert()

unsigned int NetworkStatus()
{//spontaineous reporting is turned on, read, and then turned off.
/***************************************************
R e t u r n   c o d e    					d e s c
----------------------------------------------------
	SignalStrengthTooLowUInt
	NetworkAvailableWithAcceptableSignalStrengthUInt
	NetworkStatusUnexpectedResponseUInt  
	TimeOutNetworkStatusUInt
		no valid response within TimeLimitULong ms
	NetworkNotAvailableUInt 						
****************************************************/
//See pg 35
	TimeLimitULong = 20000; //in milliseconds
	
	BufferText="";//initialize string to zero length
	byte SignalStrengthByte = 0;
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	//We will look for the expected string until time is up and will tolerate the buffer temporarily going empty.
	TextIndexByte = 0;//used to index through string
	Serial2.print(F("AT+CIER=1,1,1\r"));//enable spontaneous event reporting, signal quality and network status
	//Serial.println(F("AT+CIER=1,1,1\r"));
	delay(DelayAfterDoingPrintToModemUInt);
	while(1)
	{
		while(Serial2.available() == 0)
		{//look for bytes in buffer until either we find some or run out of time
			if((millis() - StartTimeULong) > TimeLimitULong)
			{//ran out of time
				
							#ifdef DiagPrint1
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis() - core1StartForTimeStampULong);
							#endif
								
				Serial2.print(F("AT+CIER=0,0,0\r"));//disable spontaneous event reporting, signal quality and network status
				//Serial.println(F("AT+CIER=0,0,0\r"));
				delay(DelayAfterDoingPrintToModemUInt);		
				return TimeOutNetworkStatusUInt;//no response Received within TimeLimitULong ms
			}
		}
		//at least one byte has arrived
		while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
		{
			delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response

			noInterrupts();//diag
			character = Serial2.read(); //read back one character of the response from SBDWB command
			interrupts();//diag

			
			//character = Serial2.read(); //read back one character of the response from SBDWB command
			
						#ifdef DiagPrint1
						Serial.println(__LINE__);
						Serial.print(F(" character in hex is "));
						Serial.println( character, HEX);
						#endif
			
			BufferText.concat(character); //build up BufferText string
		}//have recorded entire string in buffer 

						#ifdef DiagPrint1
						Serial.println(__LINE__);
						Serial.print(F(" entire BufferText string is "));
						Serial.println( BufferText );
						#endif		
		
		//As of 3/21/2024, the newer two modems have a bug that causes them to return +CIER=0,0,0 rather than +CIEV:. As a workaround, I will assume all is well and return NetworkAvailableWithAcceptableSignalStrengthUInt
	#ifdef bugPatchEnabled
		if (BufferText.indexOf("+CIER=0,0,0") >=0)
		{//this is a command that should be sent TO the modem so is a bug
			Serial2.print(F("AT+CIER=0,0,0\r"));//disable spontaneous event reporting, signal quality and network status
			Serial.println(F("Network Status AndSignal Strength Unavailable Due To Modem Bug"));
			//Serial.println(F("AT+CIER=0,0,0\r"));
			delay(DelayAfterDoingPrintToModemUInt);
			
			
						#ifdef DiagPrint1
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
						
			return NetworkAvailableWithAcceptableSignalStrengthUInt;
		}
	#endif
	
		if (BufferText.indexOf("+CIEV:1,0") >=0)
		{//this means network service currently unavailable
			Serial2.print(F("AT+CIER=0,0,0\r"));//disable spontaneous event reporting, signal quality and network status
			//Serial.println(F("AT+CIER=0,0,0\r"));
			delay(DelayAfterDoingPrintToModemUInt);
			
			
						#ifdef DiagPrint1
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
						
			return NetworkNotAvailableUInt;
		}
		if (BufferText.indexOf("+CIEV:1,1") >=0)
		{//this means network service is available so look for signal strength
			TextIndexByte = BufferText.indexOf("+CIEV:0,") +8; //following ASCII character is signal strength
			
						#ifdef DiagPrint1
						Serial.println(__LINE__);
						Serial.print(F(" TextIndexByte = "));
						Serial.println( TextIndexByte );
						Serial.print(F(" BufferText[TextIndexByte] = "));
						Serial.println( BufferText[TextIndexByte] );
						#endif
					
			Serial2.print(F("AT+CIER=0,0,0\r"));//disable spontaneous event reporting, signal quality and network status
			//Serial.println(F("AT+CIER=0,0,0\r"));
			delay(DelayAfterDoingPrintToModemUInt);
			SignalStrengthByte = BufferText[TextIndexByte] - 0x30;//convert ASCII to number
			
						#ifdef DiagPrint1
						Serial.println(__LINE__);
						Serial.print(F("                Signal Strength = "));
						Serial.println( SignalStrengthByte );
						#endif						
									
			if(SignalStrengthByte < 3)
			{
				
							Serial.println(__LINE__);
							
				return SignalStrengthTooLowUInt;//we will only transmit if we have at least 3 bars out of 5
			}else{
				return NetworkAvailableWithAcceptableSignalStrengthUInt;
			}
					#ifdef DiagPrint1
					Serial.println(__LINE__);
					Serial.print(F(" SignalStrengthByte = "));
					Serial.println( SignalStrengthByte );
					#endif
			
						
		}else{ //didn't find +CIEV:1 network available response so 
			Serial2.print(F("AT+CIER=0,0,0\r"));//disable spontaneous event reporting, signal quality and network status
			
					#ifdef DiagPrint1
					Serial.println(F("AT+CIER=0,0,0\r"));
					#endif
			
			delay(DelayAfterDoingPrintToModemUInt);
			
			
					#ifdef DiagPrint1
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.println(millis() - core1StartForTimeStampULong);
					#endif
						
			return NetworkStatusUnexpectedResponseUInt;
		}
	}
}

unsigned int  GetMT_Message()
{
//_IridiumReceivedDataByte[45] will be populated after returning
/***************************************************
R e t u r n   c o d e    d e s c	          
----------------------------------------------------
	MT_MessageIsNullUInt
	MT_MessageRetrievedCorrectlyUInt			
	MT_MessageUnexpectedResponseUInt			
	MT_MessageTimeOutUInt						  no response within TimeLimitULong ms
	MT_MessageFailedCheckSumUInt				
	MT_MessageTooLongUInt						
****************************************************/

//GetResponseFromModemAfterSendingMessage() will define MTlengthByte which can be used to read the proper number of bytes. However, I will use the message length included in the MT message. See page 90 for +SBDRB - Short Burst Data: Read Binary Data from ISU.

	TimeLimitULong = 20000; //in milliseconds
	BufferText="";//initialize string to zero length
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem
	unsigned int LocalCheckSumUInt = 0;
	unsigned int ReceivedCheckSumUInt = 0;

	if (MTlengthByte <1)return MT_MessageIsNullUInt;
	Serial2.print(F("AT+SBDRB\r"));//tells modem to transfer a single binary SBD message to the UART. There is no response from the modem beyond sending the SBD message.
	Serial.println(F("AT+SBDRB\r"));
	delay(DelayAfterDoingPrintToModemUInt);
	ReturnCodeUInt = ToldModemToGiveUsTheReceivedMessageUInt;

	while(Serial2.available() == 0)
	{
		if((millis() - StartTimeULong) > TimeLimitULong)
		{
			
			#ifdef DiagPrint1
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.println(millis() - core1StartForTimeStampULong);
			#endif
							
			return MT_MessageTimeOutUInt;//no response Received within TimeLimitULong ms
		}
	}
	//if buffer has bytes before time runs out, collect them
	//format is {2-byte message length} + {binary SBD message} + {2-byte checksum}
	while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
	{		
		delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response	
		
		noInterrupts();//diag
		character = Serial2.read(); //read back one character of the response from SBDWB command
		interrupts();//diag

		//character = Serial2.read(); //read back one character of the MT message
		
		#ifdef looparoundTest
		Serial.println(__LINE__);
		Serial.print(F(" char in hex: "));
		Serial.println( character, HEX);
		#endif
					
					
		if ((character != '\r') && (character != '\n'))BufferText.concat(character); //build up BufferText string but exclude <cr>, and <lf>			
	}//have recorded entire string in buffer. It should contain {2-byte message length} + {binary SBD message} + {2-byte checksum}. {binary SBD message} seems to be just the 45 byte array.
	
				#ifdef looparoundTest
				Serial.print(F("line # "));	
				Serial.println(__LINE__);
				Serial.print(F(" BufferText[0] in hex = "));
				Serial.println( BufferText[0], HEX );
				Serial.print(F(" BufferText[1] in hex = "));
				Serial.println( BufferText[1], HEX );
				#endif
	
	//the first two bytes are the 2-byte message length
	unsigned int MessageLengthUInt = word(BufferText[0],BufferText[1]); //build decimal value of message length from the first two hex bytes.
						
				#ifdef looparoundTest
				Serial.print(F("line # "));					
				Serial.println(__LINE__);
				Serial.print(F(" MessageLengthUInt = "));
				Serial.println( MessageLengthUInt );
				#endif
						
	if (MessageLengthUInt > 50)return MT_MessageTooLongUInt;
	//MT Message length is within limits. This means that the array can't be larger than 45 bytes because we have 5 bytes of overhead.

	LocalCheckSumUInt = 0;//unsigned integer is 2 bytes 
	for (i=2; i < MessageLengthUInt+2;i++)//checksum is over the array. Skip over first two bytes which are the message length.
	{
		LocalCheckSumUInt = LocalCheckSumUInt + BufferText[i];

		#ifdef looparoundTest
		Serial.print(F("line # "));			
		Serial.print(__LINE__);
		Serial.print(F(". TS  "));	
		Serial.println(millis() - core1StartForTimeStampULong);
		Serial.print(F(" LocalCheckSumUInt = "));
		Serial.println( LocalCheckSumUInt );
		#endif

	}

		#ifdef looparoundTest
		Serial.print(F("line # "));			
		Serial.print(__LINE__);
		Serial.print(F(". TS  "));	
		Serial.println(millis() - core1StartForTimeStampULong);	
		Serial.print( "Iridium Received Data. Message Length is ");//diag
		Serial.println(MessageLengthUInt);
		#endif
	
	for (i = 2; i < MessageLengthUInt+2; i++)//normally this means going from 2 to 47 but could be fewer if user sent fewer bytes in MT message. Skip over first 2 bytes of BufferText because they are the message length.
	{
		IndexByte = i-2;//0 to typically 44
		_IridiumReceivedDataByte[IndexByte] = BufferText[i];

		#ifdef looparoundTest
		Serial.print( _IridiumReceivedDataByte[IndexByte] );
		Serial.print(",");
		#endif
	//_IridiumReceivedDataByte[] can also be filled elseware during MPM looparound
	}
	
	/************************************************************
Layout of MT buffer contents:
index	contents
0,1		message length
2-46	array bytes 0-44
47		MSB of checksum  (45+2)
48		LSM of checksum (45+3)  
*************************************************************/
	unsigned int CheckSumUpperBytePointerUInt = MessageLengthUInt+2;
	unsigned int CheckSumLowerBytePointerUInt = MessageLengthUInt+3;
	//Verify message wasn't corrupted.
	ReceivedCheckSumUInt = word(BufferText[CheckSumUpperBytePointerUInt],BufferText[CheckSumLowerBytePointerUInt]);

	#ifdef looparoundTest
	Serial.println(__LINE__);
	Serial.print(F(" ReceivedCheckSumUInt = "));
	Serial.println( ReceivedCheckSumUInt );
	
	Serial.println(__LINE__);
	Serial.print(F(" LocalCheckSumUInt = "));
	Serial.println( LocalCheckSumUInt );
	#endif		
						
	if(LocalCheckSumUInt != ReceivedCheckSumUInt)return MT_MessageFailedCheckSumUInt;
	//_IridiumTransmitDataByte was not corrupted so can return sucessfully
	return MT_MessageRetrievedCorrectlyUInt;	
}

void LoopAround()//I could not loop around through modem to work
{
	if(modemLooparoundEnabledQbool)//looparound through modem
	{
		//This is a user invoked diagnostic tool that will take what is in the MO buffer and put it into the MT buffer. See page 95.
		FlushUART_Buffer();//so I am only reading back new info
		Serial2.print(F("AT+SBDTC\r"));//tells modem to transfer the contents of the MO buffer to the MT buffer
		delay(DelayAfterDoingPrintToModemUInt);
		LocalReturnCodeUInt = TestForOK();//It will wait up to 500 ms for "OK"
		if (LocalReturnCodeUInt != OK_FoundUInt)
			{
				Serial.print(F("Loop around request to modem failed"));
			}	
	}
	if(MPMonlyLooparoundEnabledQbool)//looparound only through MPM
	{
		
			#ifdef looparoundTest
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(". TS  "));	
			Serial.print (millis() - core1StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.println(F("_IridiumReceivedDataByte[]:"));
			#endif
			
		for(byte index = 0; index < 45;index++)
		{
			_IridiumReceivedDataByte[index] = IridiumTransmitDataByte[index];
			
					#ifdef looparoundTest
					Serial.print(_IridiumReceivedDataByte[index]);
					Serial.print(F(" "));
					#endif

		}
	}
}//end of LoopAround()

void IridiumModemSetup()
{
/***********************************************
Possible Status Return Codes
************************************************
ModemSetupProceedingUInt
CorrectModemConnectedUInt
************************************************
Possible final Return Codes
************************************************
WrongModemConnectedCheckSerialNumberUInt
ModemFailedAtSetupUInt
ModemReadyForUseUInt
***********************************************/

//byte i=0;//used in data counter

					#ifdef setupMystery
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
	
	
	Serial2.print("ATE0");//disable echo mode
	
	ReturnCodeUInt = ModemSetupProceedingUInt;
	
	delay(2);//diag
		
	SetToTextualResponses(); //all subroutines depend on seeing Textual Responses responses. It is the default but this is defensive
	
					#ifdef mysteryCall
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
	delay(2);//diag
	
	IdentifyModem();//ReturnCode is updated
	
					#ifdef mysteryCall
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
							
	//failures are returned; success advances us to next step in setup 						
	if(ReturnCodeUInt != CorrectModemConnectedUInt)
	{
		
					#ifdef mysteryCall
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(" ReturnCodeUInt is "));
					Serial.println( ReturnCodeUInt );
					#endif
		
		return;
	}

	ReturnCodeUInt = SetModemDefaults();
	
						#ifdef mysteryCall
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCode is "));
						fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
						#endif
				
	/*
	if(ReturnCodeUInt != ModemDefaultsSetUInt)
	{
		ReturnCodeUInt = ModemFailedAtSetupUInt;
		return;
	}
	ReturnCodeUInt = ModemReadyForUseUInt;
	*/
	
	ModemCommandUInt = NoActiveModemCommandUInt;//this causes first read of status to be ModemReadyForUseUInt and subsequent ones to be idle.
	return; //if ATP, return code is modem ready for use
}//end of IridiumModemSetup()

void PerformTransmitAndReceiveSequence()
{	
	
						#ifdef modemNGtest
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif

/************************************************
input is IridiumTransmitDataByte[45]. It was initialized to a decending count starting at 255 and ending at 211. I test for this sequence and if I see it, will return an error code of DuplicateTransmitOfDataAttemptedUInt. After sending valid data, IridiumTransmitDataByte[45] is initialized to this data.

output is return code and, if all ok, _IridiumReceivedDataByte[45]

We only return upon success or failure. FDR can read ReturnCodeUInt at any time for status.

If Modem loop around has been enabled via the modemLooparoundEnabledQbool flag, I echo data via the modem regardless of sateline state and return success as return code. However, I could not get it to work so will loop around only as far as MPM with the MPMonlyLooparoundEnabledQbool flag.
*************************************************/
		delay(20);//this delay provides a time window for interrupts that receive data from FDR to work. These interrupts deliver receive data necessary for PerformTransmitAndReceiveSequence() to work.
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif	

//check for duplicate data being transmitted if not in looparound
byte arrayIndexOneByte = 0;
byte arrayIndexTwoByte = 0;
bool identicalArrayQbool = false;
//we expect a valid IridiumTransmitDataByte[] coming in
	if((!modemLooparoundEnabledQbool) && (!MPMonlyLooparoundEnabledQbool))//neither loop around active 
	{
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif
		
		for (arrayIndexOneByte = 0;arrayIndexOneByte < 45;)//check each byte in array compared to previous byte. If all the same, generate errors.
		{

						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif	
			
			if(IridiumTransmitDataByte[arrayIndexOneByte] == PreviousIridiumTransmitDataByte[arrayIndexOneByte])
			{
				arrayIndexOneByte++;//advance to next value of arrayIndexOneByte and check next element
				identicalArrayQbool = true;//identicalArrayQbool is true so far in the comparison
				
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif				
				
			}else
			{//one byte is different so update previous array with current array
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif		
		
				for (arrayIndexTwoByte = 0; arrayIndexTwoByte < 45;arrayIndexTwoByte++)//since array was successfully transmitted, save it as previous array 
					{				
						PreviousIridiumTransmitDataByte[arrayIndexTwoByte] = IridiumTransmitDataByte[arrayIndexTwoByte];//save a copy of what was just transmitted. We use it to prevent sending the same data twice in a row
					}
				identicalArrayQbool = false;	
				arrayIndexOneByte = 255;//forces loop to terminate early
				
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif				
				
			}					
		}//check next byte
		//all bytes now checked until we see a difference

						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif
		
		if(identicalArrayQbool)//if all elements were equal so set a failure return code.	
		{
				ReturnCodeUInt = DuplicateTransmitOfDataAttemptedUInt;
				
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif				
				
				return;
		}
	}//array not a duplicate so continue transmit process; I only check when not in looparound
	
	
	if((!modemLooparoundEnabledQbool) && (!MPMonlyLooparoundEnabledQbool))//Do check of satelite only if not in loop around
	{
		ReturnCodeUInt = NetworkStatus();//we will only transmit if we have at least 3 bars
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif		
		
	}else
	{//in looparound so fake sateline is OK
		ReturnCodeUInt = NetworkAvailableWithAcceptableSignalStrengthUInt;
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif		
		
	}

				
						#ifdef modemNGtest
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif
									
	if(ReturnCodeUInt != NetworkAvailableWithAcceptableSignalStrengthUInt)
	{
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif		
		
		return;
	}

	//Otherwise, proceed with transmission sequence if MPMonlyLooparoundEnabledQbool is false. That means we can be operational or modem looparound.
	if(!MPMonlyLooparoundEnabledQbool)
	{
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif		
		
		ClearMO_MessageBuffer();//defensive since buffer gets overwritten
		GetResponseFromClearMO_MessageBuffer();	
		if (ReturnCodeUInt != MO_BufferClearedSuccessfullyUInt)
		{
			
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif			
			
			return;
		}
		ClearMT_MessageBuffer();//no response from modem at this point 
		ReturnCodeUInt = GetResponseFromClearMT_MessageBuffer();	
		if (ReturnCodeUInt != MT_BufferClearedSuccessfullyUInt)		
		{
			
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif			
			
			return;
		}
		TellModemWeWillBeSending45BytesPlusOverhead();
		GetResponseFromModemAfterSendingMessageSize();//if size of array is within limits, we get READY which translates to MessageSizeAcceptedUInt. If beyond these limits, we get "3" which translates to SBD_MessageSizeTooBigOrTooSmallUInt.
		if (ReturnCodeUInt != MessageSizeAcceptedUInt)
		{
			
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif			
			
			return;
		}
	
							#ifdef modemNGtest
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);							
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.println(F("data to modem:"));
							for(i=1;i<45;i++)
							{
								Serial.print(IridiumTransmitDataByte[i]);
								Serial.print(",");
							}
							Serial.println();
							#endif
	
		WriteDataToMobileOriginatedBuffer();//data comes from IridiumTransmitDataByte[45]. RockBlock address information plus checksum are added. 
	  
							#ifdef looparoundTest
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));			
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.print(F(" ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif
	
		ReturnCodeUInt = GetResponseFromModemAfterWritingDataToMobileOriginatedBuffer();
		if(ReturnCodeUInt != SBD_MessageSuccessfullyWrittenUInt)
		{
		
					#ifdef looparoundTest
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
		
			return;
		}
	}
	//data is now in the Mobile Originated Buffer unless I'm doing MPM looparound, and I can either loop it back or transmit it
	if(modemLooparoundEnabledQbool || MPMonlyLooparoundEnabledQbool)
	{
		
						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif			
		
		LoopAround();
		/************************************************
		if modemLooparoundEnabledQbool true: Contents of MO buffer are moved to MT buffer
		if MPMonlyLooparoundEnabledQbool true: _IridiumReceivedDataByte[] is set equal to IridiumTransmitDataByte[]
		*************************************************/
		delay(20);//this delay provides a time window for interrupts that receive data from FDR to work. These interrupts deliver receive data necessary for LoopAround() to work.
		
						#ifdef looparoundTest
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): MPM "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.print (millis() - core1StartForTimeStampULong);
						Serial.println(F(" ms"));	
						Serial.print(F("responds from GetMT_Message(): "));		Serial.println(GetMT_Message());//it tries to populates _IridiumReceivedDataByte[45] but will report faults. I'l print out the local return code
						Serial.println(F("data received from modem (MT buffer) or from MPM looparound: "));
						for(i=0;i<45;i++)
						{
							Serial.print(_IridiumReceivedDataByte[i]);
							Serial.print(",");	
						}
						Serial.println();
						#endif
					
	}else
	{//no looparound active
		SendMessageInMobileOriginatedBuffer();//initiate transmit and receive exchange with satellite

						#ifdef DiagPrint1		
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis());
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif		
		
	}

						#ifdef DiagPrint1		
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis());
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif
						
	if(!MPMonlyLooparoundEnabledQbool)
	{//if in normal mode or looparound through modem, do the following

						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif	

		GetResponseFromModemAfterSendingMessage();

						#ifdef DiagPrint1
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));						
						Serial.println(__LINE__);
						#endif			
		
		
	}//if MPMonlyLooparoundEnabledQbool true, we do nothing with the modem
	
	/********************************
			populated variables are
			MOstatusByte
			MOMSNUInt
			MTstatusByte
			MTMSNUInt
			MTlengthByte
			MTqueuedByte
	*********************************/
	if((modemLooparoundEnabledQbool)||(MPMonlyLooparoundEnabledQbool))
	{
		ReturnCodeUInt = TransmitAndReceiveSuccessfulUInt;
	
						#ifdef DiagPrint1		
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis());
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif	
	}
	
	//there won't be a modem response because no transmit/receive took place but we will still say transmission was success 
	if ((ReturnCodeUInt == TransmitAndReceiveSuccessfulUInt)||(ReturnCodeUInt == TransmitAndReceiveSuccessfulPlusReceivePendingUInt))
	{//if we did Receive a message, get it 
		
		
						#ifdef DiagPrint1		
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis());
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif	
						
		if(!MPMonlyLooparoundEnabledQbool)
		{
			ReturnCodeUInt = GetMT_Message(); //it tries to populates _IridiumReceivedDataByte[45] but will report faults
			
						#ifdef DiagPrint1		
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis());
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif				
			
		}else
		{//if MPMonlyLooparoundEnabledQbool is true, we don't deal with modem but must say Transmit and Receive Successful so looped back data is sent back to FDR
			ReturnCodeUInt = TransmitAndReceiveSuccessfulUInt;
		}//if we are doing MPM loop around, force return code to so no data received from ground

							#ifdef DiagPrint1
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));			
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.print(F(" ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif
				
	}	
	//translating from internal jagon to external simple terminology.
	if(ReturnCodeUInt == MT_MessageIsNullUInt)
	{		
		ReturnCodeUInt = TransmitSuccessfulAndNoReceiveUInt;
		
							#ifdef DiagPrint1
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));			
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.print(F(" ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif
					
	}
	
	if (ReturnCodeUInt == MT_MessageRetrievedCorrectlyUInt)
	{
							#ifdef DiagPrint1	
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));			
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.print(F(" ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif
		
		if(MTqueuedByte == 0)ReturnCodeUInt = TransmitAndReceiveSuccessfulUInt;
		if(MTqueuedByte > 0)ReturnCodeUInt = TransmitAndReceiveSuccessfulPlusReceivePendingUInt;
		//I don't expect this case but it appears that data has been left in MT queue. I'm not sure how we get it out with current architecture. I had assumed the data was on the server. 
							#ifdef DiagPrint1		
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis());
							Serial.print(F(" ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif				
	}
	//all other return codes are faults or loopback so pass them through
	ModemCommandUInt = NoActiveModemCommandUInt;//this causes first read of status to be ModemReadyForUseUInt and subsequent ones to be idle.
	return;
}//end of PerformTransmitAndReceiveSequence()


bool IridiumTransmitDataByteArrayIsDuplicate()
{
	for (i = 0; i<45; i++)
	{					
		if (IridiumTransmitDataByte[i] != PreviousIridiumTransmitDataByte[i])
		{
			for (i = 0; i<45; i++)
			{
			PreviousIridiumTransmitDataByte[i] = IridiumTransmitDataByte[i]; 
			}
			return false;
		}
	}
	//all 45 bytes match so array user wants to transmitted matches the array just transmitted. It will not be sent.
	return true;
}

void VerifyDisableMT_Alert() 
//ask modem to return the ring function state. reference: page 90.
{
	FlushUART_Buffer(); //defensive action in case last subroutine did not empty buffer
	delay(DelayAfterDoingPrintToModemUInt);
	Serial2.print("AT+SBDMTA?\r");
	delay(DelayAfterDoingPrintToModemUInt);
	ReturnCodeUInt = VerifyDisableMT_AlertUInt;				
}

void FlushUART_Buffer()
{
	unsigned long startTimeForFlushULong = millis();
	
	delay(2);//diag
	
	while((Serial2.available() > 0) && (millis()- startTimeForFlushULong < 2000))
	{//keep reading stream from modem until all characters collected or it has been 2 seconds	
	delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response

	noInterrupts();//diag
	Serial2.read(); //read back one character of the response from SBDWB command
	interrupts();//diag

	
	//Serial2.read();
	
	delay(2);//diag
		
	}
	delay(DelayAfterDoingPrintToModemUInt); //emperically found that I need to wait before sending anything to UART
	//ReturnCodeUInt = FlushedUART_BufferUInt;//not useful status
	
	if(Serial2.available() > 0)
	{//if we flushed for 2 seconds and there is more, print it out:
		Serial.print("&"); 
		noInterrupts();//diag
		Serial.print(Serial2.read(),HEX);
		interrupts();//diag
		
		delay(2);//diag
			
	}
		
}

void SetToTextualResponses()
{
	Serial2.print("AT V1\r");//switch to Textual Responses
	delay(DelayAfterDoingPrintToModemUInt);
	FlushUART_Buffer();//dump response	
}

void SetToNumericResponses()
{
	Serial2.print("AT V0\r");//switch to Numeric
	delay(DelayAfterDoingPrintToModemUInt);	
	FlushUART_Buffer();//dump response
}

void InitializeIridiumTransmitDataByteArray()
{
	for (i = 0; i < 45; i++)
	{
		IridiumTransmitDataByte[i] = 255 - i;
	}
}


bool CycleActiveQ()
{//if we are still reporting status, we are not done yet.
	if ((ReturnCodeUInt >= StatusRangeMinUInt) || (ReturnCodeUInt >= StatusRangeMaxUInt))return true;
	return false;//if not reporting status, then we either failed or succeeded so are done.
}


void PingModem()
{
/****************************************************
R E T U R N  C O D E S 
*****************************************************
PingThroughMPM_AndModemSuccessUInt
PingToMPM_SuccessButToModemFailedUInt 

If the battery is too low, the modem is marginal and so I will not ping it and instead return PingToMPM_SuccessButToModemFailedUInt

This is a low level function so have hidden its internal return codes from FDR 
*****************************************************/

						#ifdef modemsetupTrace 
						
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif

	bool asAVoltageBool = true;
	if(fdr.readBattery(asAVoltageBool) < 6)
	{//battery to low to trust modem
		ReturnCodeUInt = PingToMPM_SuccessButToModemFailedUInt; 
		return;
	}
	
	ReturnCodeUInt = PerformingPingUInt;

	FlushUART_Buffer();//so I am only reading back new info
	Serial2.print(F("AT\r"));//modem should respond to "AT" with "OK"
				
	LocalReturnCodeUInt = TestForOK();//It will wait up to 500 second for "OK"
	if (LocalReturnCodeUInt == OK_FoundUInt)
	{
		
					#ifdef modemsetupTrace 			
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
					#endif
		
		ReturnCodeUInt = PingThroughMPM_AndModemSuccessUInt;
	
	}else{
		
					#ifdef modemsetupTrace 	
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F("LocalReturnCodeUInt is "));
						Serial.println(LocalReturnCodeUInt);
					#endif

		ReturnCodeUInt = PingToMPM_SuccessButToModemFailedUInt;			
	}
	return;
}//end of PingModem()

unsigned int GetResponseFromModemAfterWritingDataToMobileOriginatedBuffer()
{
//it handles the response after "READY" which is a response to AT+SBDWB=
/***************************************************
R e t u r n   c o d e   
----------------------------------------------------
SBD_MessageSuccessfullyWrittenUInt
TimeOutAfterSendingMessageUInt   no response within TimeLimitULong ms
SBD_MessageTimeOutByModemUInt    timing done by modem
SBD_MessageChecksumWrongUInt
SBD_MessageSizeWrongUInt					  
UnexpectedResponseAfterSendingMessageSizeUInt   
****************************************************/
	TimeLimitULong = 100000; //in milliseconds  was 20 seconds but for diag changed to 100 seconds
	//See pg 95
	BufferText="";//initialize string to zero length
	StartTimeULong = millis(); //prevents subroutine hanging up if it doesn't get a response from the modem 
	while (Serial2.available() == 0)
	{
		if((millis() - StartTimeULong) > TimeLimitULong) return TimeOutAfterSendingMessageUInt;//no response Received within TimeLimitULong ms
	}
	while(Serial2.available() > 0)//keep reading stream from modem until all characters collected
	{		
		delay(DelayBeforeDoingReadFromModemUInt); //emperically found that waiting more than 0.5ms is needed for reliable response

		noInterrupts();//diag
		character = Serial2.read(); //read back one character of the response from SBDWB command
		interrupts();//diag

		
		//character = Serial2.read(); //read back one character of the response from SBDWB/READY response
					
		
							#ifdef charCheck
							Serial.println(__LINE__);
							Serial.print(F(" character in hex is "));
							Serial.println( character, HEX);
							#endif
								
		BufferText.concat(character); //build up BufferText string
	}//have recorded entire string
					
							#ifdef charCheck
							Serial.println(__LINE__);
							Serial.print(F(" entire BufferText string is "));
							Serial.println( BufferText );
							#endif
									
	if (BufferText.indexOf("0") >= 0)return SBD_MessageSuccessfullyWrittenUInt;
	if (BufferText.indexOf("1") >= 0)return SBD_MessageTimeOutByModemUInt;
	if (BufferText.indexOf("2") >= 0)return SBD_MessageChecksumWrongUInt;
	if (BufferText.indexOf("3") >= 0)return SBD_MessageSizeWrongUInt;
	return UnexpectedResponseAfterSendingMessageSizeUInt;
}//end of GetResponseFromModemAfterWritingDataTo MobileOriginatedBuffer()

void getModemSerialNumbers()
{
	/**************************************************
	Input is nearURBsnByte and farURBsnByte defined in FDR.ino. 
	Outputs are the IMEI of the near modem (IMEIstring) and the far RockBLOCK sn (farRBsnULong) which is part of the transmitted array. **************************************************/
		
	//byte nearURBsnByte = (0b1100 & nearFarmodemSNbyte) >> 2; //mask off the two top bits and then shift them down two places.
	//byte farURBsnByte = 0b11 & nearFarmodemSNbyte; //mask off upper two bits to leave lower two bits which are the far User's modem s
	IMEIstring = nearIMEIstring[nearURBsnByte];//select the IMEI for the near modem
	farRBsnULong = RBsnULong[farURBsnByte];//select the far modem's RockBLOCK serial number written on modem 
	
					#ifdef nearFarTest   
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("() MPM: "));		
					Serial.print(__LINE__);
					Serial.print(F(". TS  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print(F("IMEIstring = "));
					Serial.println(IMEIstring);
					Serial.print(F("nearURBsnByte = "));
					Serial.println(nearURBsnByte);
					WriteDataToMobileOriginatedBuffer();//should print far RB sn by byte
					#endif 		
	return;	
}

void initialCountdown()
{
	if (OneTimeDelayBool)
	{
		for(i=0; i < 5;i++)
		{
		Serial.println(5 - i);
		delay(100);
		}
		Serial.println(F("This is the MPM."));
		Serial.println();
		ReturnCodeUInt = idleUInt;//I'm now cycling so set return code to idle
		OneTimeDelayBool = false;
	}
}

void setupModemQ()
{
	
			
			#ifdef mysteryCall 
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print(F("SetUpModemUInt = "));
			Serial.println(SetUpModemUInt);
			Serial.print(F("ModemCommandUInt = "));
			Serial.println(ModemCommandUInt);
			#endif 
							
	if(ModemCommandUInt == SetUpModemUInt)
	{
		setupModem();	
	}
}//end of setupModemQ()

void setupModem()
{	
							#ifdef mysteryCall 
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));						
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - core1StartForTimeStampULong);	
							Serial.println(F(" ms"));
							#endif		
				
	
		ReturnCodeUInt = ModemSetupProceedingUInt;
		//nearFarmodemSNbyte = highByte(ModemCommandUInt);//since low byte matches SetUpModemUInt, we know that the high byte is the nearFarmodemSNbyte sent from FDR.
		
							#ifdef pingMystery 
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - core1StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif 	


		getModemSerialNumbers();//translate nearURBsnByte and farURBsnByte into its IMEIstring and  farRBsnULong
		
							#ifdef mysteryCall 
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - core1StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif 	
		
		IridiumModemSetup();//it verifies correct modem connected to payload via the IMEIstring and fills ReturnCodeUInt	
		
		//IdentifyCurrentIndicatorEventReportingSettings();//prints to TerraTerm
		
							#ifdef pingMystery
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));
							Serial.print(__LINE__);
							Serial.print(F(". TS  "));	
							Serial.println(millis() - core1StartForTimeStampULong);
							Serial.print(F("ReturnCodeUInt = "));
							Serial.println( ReturnCodeUInt );
							#endif
		
		ModemCommandUInt = NoActiveModemCommandUInt;
	
	//return code was set by IridiumModemSetup()
}//end of setupModem()


void pingQ()
{	
	
			#ifdef pingTest 
			if (ModemCommandUInt != NoActiveModemCommandUInt)
			{			
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - core1StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print(F("ModemCommandUInt = "));
				Serial.println(ModemCommandUInt);
			}
			#endif
	
	if (ModemCommandUInt == PingUInt)
	{
		core1StartForTimeStampULong = millis();//start diag timer when ping command received.
		
						#ifdef modemsetupTrace 
						
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
		
		PingModem();//ReturnCodeUInt now contains the result.	
		
						#ifdef modemsetupTrace 
									
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCodeUInt is "));
						fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
						#endif
		
		ModemCommandUInt = NoActiveModemCommandUInt;

	}//end of ping function 
}//end of pingQ()

void versionQ()
{	
	if (ModemCommandUInt == versionQUInt)
	{
		printVersionOfIridium();
		return;
	}
}//end of version function


void setLoopAroundQ()
{	
	if (ModemCommandUInt == setLoopAroundUInt)
	{

						#ifdef nearfasrtest 
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
		
		MPMonlyLooparoundEnabledQbool = true;
		//modemLooparoundEnabledQbool = true;//was modemLooparoundEnabledQbool . flag used to enable LoopAround()
		ReturnCodeUInt = dataLoopAroundEnabledUInt;//ReturnCodeUInt now contains ack. A status request will return this value if nothing else has run.
		
						#ifdef nearfasrtest 
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif
		
		ModemCommandUInt = NoActiveModemCommandUInt;

	}//end of loop around enable
}

void clearLoopAroundQ()
{	
	if (ModemCommandUInt == clearLoopAroundUint)
	{

						#ifdef nearfasrtest1 
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						#endif
		
		MPMonlyLooparoundEnabledQbool = false;//was modemLooparoundEnabledQbool. flag used to enable LoopAround()
		ReturnCodeUInt = dataLooopAroundDisabledUInt;//ReturnCodeUInt now contains ack. A status request will return this value if nothing else has run.
		
						#ifdef nearfasrtest 
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif
		
		ModemCommandUInt = NoActiveModemCommandUInt;

	}//end of loop around disabled
}

void PerformTransmitQ()
{	
	if (ModemCommandUInt == PerformTransmitUInt)
	{
		
		ReturnCodeUInt = AboutToStartTransmitProcessUInt;								
		PerformTransmitAndReceiveSequence();//ReturnCodeUInt constantly being updated and will be sent back to FDR when requested within an interrupt. We only return upon conclusion: success or failure. If modem loop around has been enabled, we echo data via modem regardless of satellite state. If MPM loop around is enabled, we echo back data regardless of modem with a return code of dataLoopAroundEnabledUInt. If not in looparound and transmitting data is the same as the last session, return code is DuplicateTransmitOfDataAttemptedUInt to prevent wasting satellite credits.
	
				#ifdef DiagPrint1
					if ((ReturnCodeUInt == TransmitAndReceiveSuccessfulUInt) ||(ReturnCodeUInt == TransmitSuccessfulAndNoReceiveUInt)||(ReturnCodeUInt == TransmitAndReceiveSuccessfulPlusReceivePendingUInt))
					{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						
			
						//delay(55000);//test code because we also have test of command to ensure we do not get too high a rate of transmit commands. diag
						//while(1);//stop program after single success
					}
				#endif
	
		ModemCommandUInt = NoActiveModemCommandUInt;
		
						#ifdef DiagPrint1
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(". TS  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F(" ReturnCodeUInt = "));
						Serial.println( ReturnCodeUInt );
						#endif
		
	}
}


void modemCommands()
{		
	if(ModemCommandUInt  == NoActiveModemCommandUInt)return;

			#ifdef modemsetupTrace1
			Serial.print("core ");
			Serial.print(rp2040.cpuid()); 
			Serial.print(" ");
			Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print("ModemCommandUInt is ");
			Serial.println(ModemCommandUInt);
			#endif
			
			
	setupModemQ();//all of these functions will change ModemCommandUInt  to NoActiveModemCommandUInt when they are done
	
			#ifdef mysteryCall
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.println(F(" ms"));
			#endif
			
	pingQ();
	versionQ();
	setLoopAroundQ();
	clearLoopAroundQ();
	PerformTransmitQ();
	if(ModemCommandUInt  != NoActiveModemCommandUInt)
	{//this means none of the above functions were activated so command was not valid
		ReturnCodeUInt = UnexpectedModemCommandUInt;
	}
}

unsigned int Iridium(unsigned int _ModemCommandUInt)
{	//the argument is asigned to ModemCommandUInt and loop1() will process it. This function does nothing else.
	if((_ModemCommandUInt != versionQUInt) && (!ModemBool))return InvalidCommandUInt;//if Iridium() is called when the modem is not equipped, return this error unless the command is for the version of the code. In this case, accept the command.

	ModemCommandUInt = _ModemCommandUInt;//ModemCommandUInt is defined in FDR.ino while _ModemCommandUInt is local

					#ifdef modemsetupTrace			
					Serial.print("core ");
					Serial.print(rp2040.cpuid()); 
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - core1StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print(F("ModemCommandUInt = "));
					Serial.println(ModemCommandUInt);
					#endif

/********************************************************
M O D E M  C O M M A N D S 
********************************************************
PingUInt
SetUpModemUInt
versionQUInt   (prints version of Iridium.ino)
PerformTransmitUInt
getReceivedDataUInt <-- not needed. Just read array;
dataLoopAroundEnabledUInt
dataLoopAroundDisabledUInt
statusUInt <--  toss? Use to transfer ReturnCodeUInt in Irdium.ino to ReturnCodeUInt in Fdr
********************************************************
D A T A  T O  M O D E M 
********************************************************
IridiumTransmitDataByte[] of 45 bytes 
********************************************************
R E T U R N  C 0 D E S
********************************************************
ModemReadyForUseUInt

BusySettingUpModemUIntSetUpModem();
ModemFailedAtSetupUInt
ModemFailedAtSetupTimeOutUInt

SentPerformTransmitUInt
SentgetReceivedDataUInt

SentPingUInt

PingThroughMPM_AndModemSuccessUInt
PingToMPM_SuccessButToModemFailedUInt
PingToMPM_TimedOutUInt
MPM_Busy_TransmitCommandRejectedUInt
InvalidCommandUInt

MPM_DidNotRespondToRequestForDataUInt

dataLoopAroundEnabledUInt
dataLoopAroundDisabledUInt


********************************************************
R E S P O N S E  F R O M  M O D E M  A N D  D A T A
********************************************************
MPM_ResponseUInt
IridiumReceivedDataByte[45]

MPM_RejectedDataByte[45] - accessable with TeraTerm connected to MPM. Uncomment the #define rejectedData line to turn on the print statements
********************************************************
FDR can send Modem Pro Micro (MPM) only PerformTransmitUInt plus 45 bytes of data to transmit, Ping, and Abort. Any other command is flagged as an error.

To get status and, optionally, the received array, this subroutine requests up to 47 bytes. The first two bytes hold the status and the remaining 45 bytes are the received data.

FDR can request that MPM give it status and, optionally, received data. Possible responses to FDR are:
•	ReturnCodeUInt
•	InvalidCommandUInt 
•	MPM_Busy_TransmitCommandRejectedUInt

If ReturnCodeUInt is TransmitAndReceiveSuccessfulUInt, student can see what was sent from the ground in IridiumReceivedDataByte[45].
*******************************************************/	

						#ifdef runOnceTest
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - core1StartForTimeStampULong);
						Serial.print(F("ModemCommand = "));
						Serial.println(ModemCommand);
						#endif
						
	return ReturnCodeUInt;	
}// end of Iridium()


void MPMoneTimeRun()
//The first time it is called, it sees if a modem is provisioned. If so, it tests the modem and then prints the Iridium.ino version number to the terminal emulator.
//Subsequent calls to this function have no effect.
{
	//unsigned int modemStatusUInt;
	
	if (MPMoneTimeRunBool)
	{

		MPMoneTimeRunBool = false;	
		
			#ifdef modemsetupTrace
			Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.println(F(" ms"));
			#endif
			
//Next, optionally test MPM and modem	
		if (ModemBool)
		{//verify modem can be used		
			//initialize previous transmit array that is used to prevent two equal arrays from being transmitted.
			for(i=0;i<45;i++)
			{
				PreviousIridiumTransmitDataByte[i]=0x00;
			}

							#ifdef modemsetupTrace
							
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);	
							Serial.print(F(".    Time Stamp  "));	
							Serial.print(millis() - StartForTimeStampULong);
							Serial.println(" ms");
							Serial.print(F("ReturnCodeUInt is "));
							fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
							#endif
			
			PingModemDiag();//function calls Iridium(PingUInt) and translates return code into printed text. It then prints out Iridium.ino version number. ReturnCodeUInt is available for evalutation. ItThe Modem hardware is operationalThe Modem hardware is operational will time out if no response	
							
							#ifdef modemsetupTrace
								
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.println(millis() - StartForTimeStampULong);
							Serial.println(" ms");	
							Serial.print(F("ReturnCodeUInt is "));
							fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
							delay(2); //print buffer problem fix?	
							#endif
							
			delay(5);//give time for ping function to start
			//PingModemDiag() doesn't return until ping completes so return code will be the final one. Intermediate return codes will have passed. So following code unnecessary
			if(ReturnCodeUInt == PerformingPingUInt)
			{
				while(ReturnCodeUInt == PerformingPingUInt)
				{
					Serial.println("ping is running");//wait for ping to terminate diag
				}
			}
			
						#ifdef modemsetupTrace
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F("ReturnCodeUInt is "));
						fdr.ReturnCodeToPrintedText(ReturnCodeUInt);

						#endif
			
			
			if(ReturnCodeUInt == PingThroughMPM_AndModemSuccessUInt)
			{
				IridiumModemPresentBool = true;//auto detect a modem but it may not be the correct serial number or be fully functional. If all OK, IridiumModemOperationalBool will next be set during modem setup.
			}else
			{//failed ping so return early
				fdr.topOfBox();
				Serial.println("   *     Modem failed ping.       *");
			    fdr.bottomOfBox();
				fdr.WriteControlBlock(ErrorCodeAddressByte, equippedModemHardwareFaultByte);
				IridiumModemPresentBool = false;//defensive
				Serial.println();
				Serial.println("		****** Sad End of Start Up diagnostics. ******");
				Serial.println();				
				
				#ifdef menuQ
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif					
				
				return;//failure returned and left as return code but it is easier for user to just use return value since timing is not known to them.
				
			}
			
			//to get here, modem must have passed ping		
			//ModemCommandUInt = SetUpModemUInt;//set up for setupModemQ() by placing nearFarModemSNbyte as MSB and SetUpModemUInt which equals 1 as the LSB
			
					#ifdef modemsetupTrace
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif	
					
			setupModem();
			
					#ifdef modemsetupTrace
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif			
			
			
				#ifdef mysteryCall
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(". TS  "));	
				Serial.print (millis() - core1StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print("ModemCommandUInt is ");
				Serial.println(ModemCommandUInt);
				#endif
			

			if(ReturnCodeUInt == ModemSetupProceedingUInt)
			{
				while(ReturnCodeUInt == ModemSetupProceedingUInt)
				{//wait for modem setup to complete
					Serial.println("Modem setup in process.");//diag
					delay(500);//diag
				}
			}
			
			
			if(ReturnCodeUInt == CorrectModemConnectedUInt)
			{
				while(ReturnCodeUInt == CorrectModemConnectedUInt)
				{//this is a momentary intermediate return code so wait until it changes to final value 
					Serial.println("Correct modem is connected.");//diag
					delay(500);//diag
				}
			}
			
			
			if(ReturnCodeUInt == ModemReadyForUseUInt) 
			{
				IridiumModemOperationalBool = true;
				Serial.println("The modem's S/N is correct and has been configured so is ready for use.");
			}else
			{
				IridiumModemOperationalBool = false;
				Serial.print(F("Modem failed setup with Return Code of "));
				fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
				fdr.WriteControlBlock(ErrorCodeAddressByte, equippedModemSetupFaultByte);
			}
		}//end of modem test
		
		//printMPMversion();//print the Irdium.ino version
		
		Serial.println();
		
		Serial.println("     ***** End of Start Up diagnostics. *****");
		Serial.println();
		Serial.println();
		
		
				#ifdef modemsetupTrace
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif		
	
	}//end of one time run, execute this tasks if this is the first time we ran through loop
	
	return;//after first time executed, just return
}//end of MPMoneTimeRun()		

void PingModemDiag()
{
/************************************************************************
function calls Iridium(PingUInt) and translates return code into printed text.
*************************************************************************/
	bool returnCodeDecodedBool = false;
	
					#ifdef pingRunTimeTest
					unsigned long pingTimeStartULong = millis();
					#endif
	
	PingModem();

					#ifdef modemsetupTrace
					
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print("(): ");		
					Serial.print(__LINE__);
					//StartForTimeStampULong = millis();
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));	
					Serial.print("initial value returned by PingModem() is ");
					fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
					#endif
	
	unsigned int pingStartTimeUInt = millis();
	while((ReturnCodeUInt == PerformingPingUInt) && (millis() - pingStartTimeUInt < 500)) //wait up to 500 ms for ping test to be over; ReturnCodeUInt is updated by core1 asynchronously
	{
		delay(100);
	}
	//to get here, ping has completed
	
					#ifdef pingRunTimeTest
					Serial.print("ping run time was ");
					Serial.print(millis() - pingTimeStartULong);
					Serial.println(" ms.");
					#endif
		
	
					#ifdef pingTest
										
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);					
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));	
					Serial.print("final return code is ");//diag
					Serial.println(ReturnCodeUInt);//diag
					#endif
		
	if(ReturnCodeUInt == PingThroughMPM_AndModemSuccessUInt)
	{
			Serial.println(F("The Modem hardware is operational."));
			returnCodeDecodedBool = true;			
	}
	
	if(ReturnCodeUInt == PingToMPM_SuccessButToModemFailedUInt)
		{
				fdr.topOfBox();
				Serial.println("   * Processor is operational but *");
				Serial.println("   *      the modem is not.       *");
				fdr.bottomOfBox();
				Serial.println();
			returnCodeDecodedBool = true;			
		}	
	
	if(ReturnCodeUInt == PingToMPM_TimedOutUInt)
		{
				fdr.topOfBox();
				Serial.println("   * Processor is not operational *");
				Serial.println("   * so we can't reach the modem. *");
				fdr.bottomOfBox();
				Serial.println();
			returnCodeDecodedBool = true;			
		}
		
	if(!returnCodeDecodedBool)
	{
		Serial.print(F(" "));	
		
				fdr.topOfBox();
				Serial.println("   * Processor has another error  *");
				Serial.println("   * or status:                   *");
				Serial.println();
				fdr.ReturnCodeToPrintedText(ReturnCodeUInt);
				Serial.println();
				fdr.bottomOfBox();
				Serial.println();
	}
	
	Serial.println();				
}//end of PingModemDiag()

void flashLED()
{
	/******************************************
LEDflashInfoByte[0] is the error code
LEDflashInfoByte[1] is the state
core0AliveQBool overrides sequence to say core0 is not running.

If diagLEDmode is enabled in the library, we set LEDflashInfoByte[2] to diagLEDmodeIsEnabledByte. This causes flashLED() to not control the external LEd so diagnostics can use it
**********************************************/
	byte diagLEDmodeIsEnabledByte = 1;
	if(LEDflashInfoByte[2] == diagLEDmodeIsEnabledByte)return;//diagnostics will control the external LED.

					#ifdef noKB1
					if(LEDflashInfoByte[1] != 0)
					{
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F("LEDflashInfoByte[0] = "));
						Serial.println(LEDflashInfoByte[0]);
						Serial.print(F("LEDflashInfoByte[1] = "));
						Serial.println(LEDflashInfoByte[1]);
					}
					#endif 

	bool noErrorsBool = false;
	bool off = false;
	bool on = true;
	unsigned long timeIntervalULong = millis() - LEDflashTimerStartULong;//LEDflashTimerStartULong was initialized to millis(). timeIntervalULong is the number of ms since LEDflashTimerStartULong was last initialized.
		
	if(LEDflashInfoByte[0] == 0) noErrorsBool = true;
	//LEDflashInfoByte[0] is the error code
	
					//LEDflashInfoByte[0] = 0;//for testing
					//LEDflashInfoByte[1] = 0;//for testing
	
	if (!core0AliveQBool)  
	{//top priority: if core 0 is dead

					#ifdef noKB1
					if(!preventFloodBool)
					{
						preventFloodBool = true;
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
					}
					#endif
		
		
		betweenLimits(timeIntervalULong,0,50,on);
		betweenLimits(timeIntervalULong,50,1000,off);
		return;
	}
	
	//if both cores are running, we can still dump memory. The next most important thing is battery.
	
	bool asAVoltageBool = true;
	if(fdr.readBattery(asAVoltageBool) < 6)//if no battery power,  turn off LED. Loss of battery also print out warnings.
	{
		digitalWrite(ExternalLED, ExternalLED_OffByte);
					
									#ifdef noBatteryTest
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif
		/*		good stuff so save		
		betweenLimits(timeIntervalULong,0,200,on);
		betweenLimits(timeIntervalULong,200,400,off);
		betweenLimits(timeIntervalULong,400,600,on);
		betweenLimits(timeIntervalULong,600,800,off);
		betweenLimits(timeIntervalULong,800,1000,on);
		*/
		
		return;
	}
			
	switch (LEDflashInfoByte[1])
		{//LEDflashInfoByte[1] is the state
			case 0: //InFlightByte 
					
							//Serial.println("InFlightByte flash pattern");
							//Serial.println();
							//delay(500);
				
				betweenLimits(timeIntervalULong,0,500,off);

				if(noErrorsBool)
				{
					betweenLimits(timeIntervalULong,500,1000,on);
				}else
				{//there is an error, flicker LED rather than have it on for 1000 ms
				
					//Serial.println("InFlightByte flash pattern with error");
					//Serial.println();
					//delay(500);
						
					betweenLimits(timeIntervalULong,500,700,on);
					betweenLimits(timeIntervalULong,700,800,off);
					betweenLimits(timeIntervalULong,800,1000,on);
				}
				break;
				
			case 1: //InFlightWithPowerDisruptionByte
				betweenLimits(timeIntervalULong,0,500,off);
				
				if(noErrorsBool)
				{
					betweenLimits(timeIntervalULong,500,1000,on);
				}else
				{//with errors:
					betweenLimits(timeIntervalULong,500,700,on);
					betweenLimits(timeIntervalULong,700,800,off);
					betweenLimits(timeIntervalULong,800,1000,on);
				}
				break;		
				
			case 2: //MemoryLockedByte
				betweenLimits(timeIntervalULong,0,200,off);
				betweenLimits(timeIntervalULong,200,1000,on);
				break;
			
			case 3: //PreFlightByte
			
									#ifdef preFlightTest
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif
									
						
				betweenLimits(timeIntervalULong,0,200,on);
				if(noErrorsBool)
				{
				betweenLimits(timeIntervalULong,200,1000,off);
				}else
				{
									#ifdef preFlightTest
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif
					
					
					betweenLimits(timeIntervalULong,200,400,off);
					betweenLimits(timeIntervalULong,400,600,on);
					betweenLimits(timeIntervalULong,600,800,off);
					betweenLimits(timeIntervalULong,800,1000,on);
				break;
				}
		}
}//end of flashLED()

void betweenLimits(unsigned long timeIntervalULong, unsigned long lowerLimitULong, unsigned long upperLimitUlong, bool onQ)
{//if valueUlong is between these limits, onQ being true turns LED on and false turns it off. If timeIntervalULong is > 1000, it sets it back to 0 by setting timeIntervalULong to millis()

	if((timeIntervalULong >= lowerLimitULong) && (timeIntervalULong <= upperLimitUlong))
	{
		if(onQ)
		{
			digitalWrite(ExternalLED, ExternalLED_OnByte);
		}else
		{
			digitalWrite(ExternalLED, ExternalLED_OffByte);	
		}	
	}
	
	if(timeIntervalULong > 1000)
	{
		LEDflashTimerStartULong = millis();//reset timer
	}
}

void core1Heartbeat()
{//If core 0 is not involved in a data dump, core1Heartbeat() performs this task: every 4 seconds, core1 sends the lowest byte of millis() to core0 and detects that core0 returns the same value after waiting 2 seconds. If sucessful, global variable core0AliveQBool is updated to true. sentDataBool is initialized to false. heartbeatTimeULong is initialized to millis(). If core 0 is involved in a data dump, core1Heartbeat() forces core0AliveQBool true.

/*
	if(fdr.ReadControlBlock(dataDumpStatusAddressByte) == dataBeingDumpedByte)
	{
		core0AliveQBool = true;//if core0 is dumping data, force success and return
		return;
	}
*/	
	unsigned long intervalULong = millis() - heartbeatTimeULong;
	
	if((intervalULong >= 4000) && !sentDataBool)
	{//it is time to assign a number to core1toCore0Byte and we have not sent the data yet
		core1toCore0Byte = millis();//pick a semi-random number
		
				#ifdef comVar 
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print("(): ");		
				Serial.print(__LINE__);
				Serial.print(".    Time Stamp  ");	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(" ms");
				Serial.print("core1toCore0Byte = ");
				Serial.println(core1toCore0Byte);
				#endif
		
		sentDataBool = true;
		
					#ifdef comVar1
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("about to send heartbeat data");
					#endif
					
	}
	if((intervalULong >= 8000) && sentDataBool)
	{ //4 seconds after sending data, check if same value returned by core0
		sentDataBool = false;//reset test
		
					#ifdef comVar
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("about to read heartbeat data");
					#endif
					
		
				#ifdef comVar 
				Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print(F("core0toCore1Byte = "));
				Serial.println(core0toCore1Byte);
				Serial.print(F("core1toCore0Byte = "));
				Serial.println(core1toCore0Byte);
				#endif 
		
		
		if(core1toCore0Byte == core0toCore1Byte)
		{ 
			core0AliveQBool = true;
			
					#ifdef comVar
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("data matches so core0 ATP");					
					#endif
			
		}else
		{
			core0AliveQBool = false;
								
					#ifdef comVar
					Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("data didn't match so core0 failed");					
					#endif
					
		}	
		heartbeatTimeULong = millis();//reset timer	
	}
}//end of core1Heartbeat()


void printVersionOfIridium()
{
		Serial.print("\tIridium.ino\t");
		Serial.print(IridiumVersionchar);//print the Irdium.ino version
		Serial.print("\t");
		Serial.print(__DATE__);//date and time Iridium.ino was downloaded from Canvas
		Serial.print(" at ");
		Serial.println(__TIME__);
		ModemCommandUInt = NoActiveModemCommandUInt;
}