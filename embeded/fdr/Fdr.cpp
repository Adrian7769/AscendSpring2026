char Fdr_cpp_versionChar[] = "1.0.16";

// **************Team Information***************************
char team1NameChar[] = "State Penitentiary";
char team1MembersChar[]= "Aso, David*, Ethan, and Jesse";

char team2NameChar[] = "Spacejunkies";
char team2MembersChar[]= "Adrian, Caden, Irwin*, James, and Justin";
/************************************************************

For first semester, remove // from in front the #define.


For second semester, put // in front of it

************************************************************/

//#define firstSemester

/**************************************
During the First Semester, I will record analog data in EEPROM plus all GPS data in the SEEPROM. I will output it when requested. 

During the Second Semester, the modem is added. I will continue to record analog voltages in EEPROM but not GPS and students must record GPS plus any I2C outputs in SEEPROM.
**************************************/
#ifndef firstSemester
	#define secondSemester  //I did this so I wouldn't get confused
#endif
/**************************************/

//test tools:
//#define fillPROM
//#define speedOfPrintTest
//#define dgw
//#define ADCbug
//#define diagLEDmode //enable fdr.externalLED(bool state) to take over control of the external LED.
//#define faultaddressPointer
//#define releaseDiag
//#define logRawDump
//#define busTrace
//#define logTest
//#define bus1_occupancy
//#define externalLEDverbose
//#define LcT
//#define rcb
//#define loop1Timer
//#define busState
//#define I2Cstat
//#define readDebug
//#define CBfatal
//#define CBprint
//#define DiagPrint123
//#define readMys
//#define dumpLook1
//#define dumpLook
//#define loop1TimerTes
//#define ADCtimeCheck
//#define OTMlapse
//#define ControlBlockTiming
//#define lockup
//#define releaseTrap
//#define forceToMaxData
//#define ADCdebug
//#define dumpDebug
//#define emptyBuffer
//#define loopTimerDetails
//#define DiagPrint8
//#define DiagPrint12
//#define gpstestPrint
//#define nearFarTest 
//#define pingTest
//#define rejectedData
//#define print1
//#define runOnceTest
//#define newGPScodeDiag
//#define GNSSprint22
//#define DiagPrint47
//#define DiagPrint477
//#define GNSSprint
//#define pointerTest
//#define debugParceTime
//#define ebugLoopAround
//#define ModemStatusTesting
//#define modemNGtest
//#define gpsRunTimeStudy
//#define correctHeaderTest
//#define GPSdataStore
//#define receiveTest
//#define correctHeaderTest1
//#define DiagPrint3
//#define debugParceTime1
//#define rawGPSoutput
//#define GPSout
//#define gpsTiming
//#define PaulGPScode
//#define GNSSchangeToAir1
//#define gpsLapseTime
//#define gpsNoWait
//#define errorPrint
//#define GNSSchangeToAir1SecondTry
//#define pollingTimingTest
//#define air1Test
//#define GNSS_M6
//#define GNSS_M10
//#define ADCtest
//#define serial1Monitor
//#define motemFlagTest
//#define errorTrap
//#define pingPrints
//#define controlBlockTest_dataByte
//#define versionPrint
//#define menuQ
//#define frozen
//#define modemsetupTrace
//#define bus1Status
//#define printTimeStudy
//#define batteryReading

//This is a macro that can be used during debugging at any location in the code.
#define CFLTS Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);Serial.print(F("(): "));Serial.print(__LINE__);Serial.print(F(".    Time Stamp  "));Serial.print (millis() - StartForTimeStampULong);Serial.println(" ms.");

//#define CFLTSwithTS while(SerialBusyQBool);while(1){byte timebyte = byte(micros()) & 0b11;if((timebyte = 0b00)&&(rp2040.cpuid()== 0)break;if(timebyte = 0b10)&&(rp2040.cpuid()== 1)break;}	SerialBusyQBool=true;Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);Serial.print(F("(): "));Serial.print(__LINE__);Serial.print(F(".    Time Stamp  "));Serial.print (millis() - StartForTimeStampULong);Serial.println(" ms.");SerialBusyQBool = false;


//end of test tools

#define _sp Serial.print(//these might be useful but there is a risk of confusing the reader
#define _spn Serial.println(

#include <Arduino.h>
#include "Fdr.h"
#include <Wire.h> //I2C interface library

unsigned long GPSbaudRateULong = 9600;//was 9600 baud rate of GPS defaults to 9600. Change this value if your GPS is at another rate.

#define EEPROM_ADR_LOW_BLOCK 0x50 //FDR's EEPROM 0
#define EEPROM_ADR_HIGH_BLOCK 0x54 //FDR's EEPROM 0

#define SEEPROM_ADR_LOW_BLOCK 0x51 //Student's EEPROM 1
#define SEEPROM_ADR_HIGH_BLOCK 0x55 //Student's EEPROM 1

#define FailureRangeMinUInt 1U
#define FailureRangeMaxUInt 299U
#define StatusRangeMinUInt 300U
#define StatusRangeMaxUInt 399U
#define SuccessRangeMinUInt 400U//includes ping and modem set up
#define TransmitSuccessRangeMinUInt 402U//only includes transmit related successes
#define SuccessRangeMaxUInt 499U
#define MinimumTransmitCycleTimeSecondsUInt 60U


Fdr::Fdr()
{ //Constructor. This is similar to setup() but is in the library and only refers to its own class.
	//ErrorCodeAddressByte = 10; defined in Fdr.h
	
	//PM's analog inputs
	pinMode(26, INPUT);
	pinMode(27, INPUT);
	pinMode(28, INPUT);
	pinMode(29, INPUT);//GPIO 29 will be used to measure BATT MON because ADC47 will be dead if there is no battery and will return unspecified value while RP will be operational since it will get power from the USB via 3.3V.
}//end of Fdr::Fdr()


void Fdr::Pointers(intptr_t *PointerArray)
{
	/******************************************
				LIBRARY VARIABLES TO LINK TO USER'S VARIABLES
	*******************************************/
	IridiumTransmitDataByte = reinterpret_cast< byte(*) [45] > (PointerArray[0]);//IridiumTransmitDataByte points to the same address as the user's IridiumTransmitData[0]
	IridiumReceivedDataByte = reinterpret_cast< byte(*)[45] > (PointerArray[1]);
	NavigationDataByte = reinterpret_cast< byte(*)[16] > (PointerArray[2]);
	GNSS_Bool = reinterpret_cast< bool* > (PointerArray[3]);//GNSS_Bool points to the same address as the user's GNSS
	LEDflashInfoByte = reinterpret_cast< byte(*)[3] > (PointerArray[4]);//PointerArray[4] holds info for the LED flash program that runs on Iridium. element 0 is the error code and element 1 is the program state.

						#ifdef pointerTest 
						delay(2000);//time for Terra Term to stabilize
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));			
						#endif 

	//AutomaticVideoRecordAtPowerUpBool = reinterpret_cast< bool* >(PointerArray[5]);//not used anymore
	filePathChar = reinterpret_cast< char* >(PointerArray[6]); //added 11/7/2023 and later modified
	//I2Cbus1OwnerByte = reinterpret_cast< byte* > (PointerArray[7]);//from Kevin
	EEPROMjustClearedBool = reinterpret_cast< bool* > (PointerArray[8]);//used by students in second semester to signal they should clear their EEPROM
	//unused variable = reinterpret_cast< bool* > (PointerArray[9]);
	EEPROMjustOutputtedBool = reinterpret_cast< bool* > (PointerArray[10]);
	recordingDataBool= reinterpret_cast< bool* > (PointerArray[11]);
	
						#ifdef nearFarTest
						delay(2000);					
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F("nearURBsnByte = "));
						Serial.println(nearURBsnByte );
						Serial.print(F("farURBsnByte = "));
						Serial.println(farURBsnByte );
						#endif 
						
						#ifdef gpstest1 
						delay(2000);
						byte translateBoolToByte = 3;
						if(*GNSS_Bool)

						{
							translateBoolToByte = 1;
						}else
						{
							translateBoolToByte = 0;
						}
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F(" GNSS bool = "));
						Serial.println(translateBoolToByte);
						#endif 
}


void Fdr::PutInLoop()
{

			
				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
	
								//testVariableUInt = millis();//to be read by core1
	
								#ifdef 	receiveTest
								(*IridiumReceivedDataByte)[4] = byte(millis());
								Serial.print("*IridiumReceivedDataByte)[4] = ");
								Serial.println((*IridiumReceivedDataByte)[4]);
								return;
								#endif
								
								#ifdef serial1Monitor
								char character;
								unsigned int now = millis();
								while (now - millis() < 10000)
								{
									if (Serial1.available() > 0)
									{
										character = Serial1.read();
										if(character == '$')Serial.println();
										Serial.print(character);
										//Serial.print(" ");
									}
								}
								#endif
 
	welcomeSerialMonitor();//check if SerialMonitor was just connected. If so, output Menu.
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	LoopCycleTimeMonitor(false);//if set to true, generates one message if it takes more than LoopCycleTimeLimitMS_UInt for one cycle. No monitoring for first 30 seconds to allow for hardware start up delays. A parameter of true means print loop cycle time on each cycle that is longer than the previous ones. If parameter set to false, no max cycle time printed but we still log failure if cycle time exceeds limit.

							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif



				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
	
	LEDflashInfo();//pack LED flash info into LEDflashInfoUInt for use by Iridium.ino 

							#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif	

							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

	if(*GNSS_Bool)
	{
		
				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif
					
					#ifdef gpsLapseTime
								unsigned long gpsTimeULong = millis();
											
								bool GPShasNewDataBool = GlobalNavigationSatelliteSystem(false);//if GPS equipped, run it on every cycle so data is newest when needed without waiting. waitQ is set to false for now.
						
								if(GPShasNewDataBool){
								Serial.print("GPS took ");
								gpsTimeULong = millis() - gpsTimeULong;
								Serial.print(gpsTimeULong);
								Serial.println(" ms to run.");
								Serial.println();
								}
					#endif	

				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
						
		#ifndef gpsLapseTime
			GlobalNavigationSatelliteSystem(false);//if GPS equipped, run it on every cycle so data is newest when needed without waiting. Because waitQ is false, function will check the recieve buffer and if it has a NEMA sentence, it processes it. If not, it just returns. If not equipped, it just returns with false but we don't look at the return value in this instance. i don't check returned logical value because nav array tells all.
		#endif
		
				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif
				
	}

							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif


	
				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
				
	LapseTimerSeconds(); //only advance time if ControlTheFlightParameters() has run due to power up. Otherwise, time will advance before variables that depend on being initialized have been set.
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

	
	
	
						#ifdef frozen
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
				
	OnTheGround(); //tasks performed on the ground or during simulated flight 


							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

	
				#ifdef frozen
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif		
	
	InTheAir(); //tasks mostly done in the air but can be done on the ground during testing
	
						#ifdef frozen
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif		
				
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif		


}//end of PutInLoop()

void Fdr::OnTheGround()
{
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	ControlTheFlightParameters();
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

	ScanForCommand();
	
				#ifdef menuQ1
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	


	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
							
	processAnyActiveCommand();
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
							
}	//end of OnTheGround()

void Fdr::InTheAir()
{
							#ifdef DiagPrint9
							ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
							if (ErrorCodeByte != OldErrorCodeByte){
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.println(millis() - StartForTimeStampULong);
							Serial.print(F(" ErrorCodeByte = "));
							Serial.println( ErrorCodeByte );
							OldErrorCodeByte = ErrorCodeByte;
							}
							#endif	

							#ifdef DiagPrint1
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    TS:  "));	
							Serial.println(millis() - StartForTimeStampULong);
							Serial.print(F(" JustPoweredUpToBeUsedByRunCamFlag = "));
							Serial.println( JustPoweredUpToBeUsedByRunCamFlag );
							#endif
	
	
	if(JustPoweredUpFlag == true)return; //if we just powered up, data recording parameters have not yet been initialized. This is done in ControlTheFlightParameters () which is within OnTheGround() CR2.0
	
							#ifdef DiagPrint9
							ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
							if (ErrorCodeByte != OldErrorCodeByte)
							{
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.println(millis() - StartForTimeStampULong);
							Serial.print(F(" ErrorCodeByte = "));
							Serial.println( ErrorCodeByte );
							OldErrorCodeByte = ErrorCodeByte;
							}
							#endif
	
	DataIn(); //record, process, and save time and sensor data to EEPROM

							#ifdef DiagPrint9
							ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
							if (ErrorCodeByte != OldErrorCodeByte)
							{
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.println(millis() - StartForTimeStampULong);
							Serial.print(F(" ErrorCodeByte = "));
							Serial.println( ErrorCodeByte );
							OldErrorCodeByte = ErrorCodeByte;
							}
							#endif

							#ifdef GNSStest
//I //the call to gps function out because I should be only calling it from PutInLoop() and at initial diag.							
//GlobalNavigationSatelliteSystem();//output is (*NavigationDataByte)[ ]. Results can be stored in SEEPROM. If no GNSS device is connected, (*NavigationDataByte)[ ] will have an error flag.  RGS1.0
							#endif

							#ifdef GNSStest
								PrintOutGNSS_Array();//diag   RGS1.0
							#endif

							#ifdef nearFarTest
							ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
							if (ErrorCodeByte != OldErrorCodeByte)
							{
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.println(millis() - StartForTimeStampULong);
							Serial.print(F(" ErrorCodeByte = "));
							Serial.println( ErrorCodeByte );
							OldErrorCodeByte = ErrorCodeByte;
							}
							#endif

}//end of InTheAir()


/********************************************************************************
L E V E L  2  S U B R O U T I N E S
*********************************************************************************/

void Fdr::ControlTheFlightParameters()
{	

					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
	
  if (JustPoweredUpFlag == true){ //executes only when power first comes up
	  JustPoweredUpFlag = false;
	  StartTimeMSULong = millis();// application of power initializes the Start Time to the current value of the real time clock.
	  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
	  
	  if (ReadControlBlock(ProgramStateAddressByte ) == PreFlightByte)
	  { //if ReadyForLaunch true, set InFlightByte state
			*EEPROMjustClearedBool = true;//When we moved from stopped to Preflight, we clear EEPROM by resetting the pointer to next available address. Then we power cycle so all variables are initialized. After power comes back, if we read EEPROM and see preflight, we know we will move to flight, so EEPROM was erased. We can set the EEPROMjustClearedBool flag true and it will be avaialable to the students. If later we get a power hit, we will not start out in preflight so no chance of this flag going true again. This flag is available to students to trigger the erasing of their SEEPROM. They are free to clear it when done.
			
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
  
		  WriteControlBlock(ProgramStateAddressByte,InFlightByte);
		  //verify writes worked
		  if (ReadControlBlock(ProgramStateAddressByte) != InFlightByte){
			  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
			  
			  WriteControlBlock(ErrorCodeAddressByte, ControlTheFlightParametersReadyForLaunchReadBackFailureFlagByte);	//RGS1.2
		  }

					  #ifdef DiagPrint123
					  ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
					  if (ErrorCodeByte != OldErrorCodeByte)
					  {
					  Serial.print(F("ControlTheFlightParameters(): "));
					  Serial.print(__LINE__);
					  Serial.print(F(".    Time Stamp  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  Serial.print(F(" ErrorCodeByte = "));
					  Serial.println( ErrorCodeByte );
					  OldErrorCodeByte = ErrorCodeByte;
					  }
					  #endif
  
	  return;	
	  }
	  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
	  
		if (ReadControlBlock(ProgramStateAddressByte) == InFlightByte){//if InFlight, set state to DisruptedPower.	
			  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
		  
			WriteControlBlock(ProgramStateAddressByte, InFlightWithPowerDisruptionByte);
		  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
		  
			//verify writes worked
			if (ReadControlBlock(ProgramStateAddressByte) != InFlightWithPowerDisruptionByte){ 
			  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif				
			  
			  WriteControlBlock(ErrorCodeAddressByte, ControlTheFlightParametersInFlightFlagReadBackFailureFlagByte);//RGS1.2
		  }
		  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
	  
		  return;
	  }
	  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
	  
  }
  
					  #ifdef DiagPrint123
					  Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					  Serial.print(F("(): "));		
					  Serial.print(__LINE__);
					  Serial.print(F(".    TS:  "));	
					  Serial.println(millis() - StartForTimeStampULong);
					  #endif
		
}//end of ControlTheFlightParameters()



void Fdr::LoopCycleTimeMonitor(bool verboseQ)
/**************************************************
Alarms once when loop() cycle time is too long which effects code interfacing in real time. Starts monitoring 30 seconds after power up. If loop is held up because we are waiting for a user response, no warning is generated if resetLoopTimerBool is also set true. 

If verboseQ is true, we print loop time on each cycle that is longer than past times.

StartForTimeStampULong is initialized to millis() at power up.  StartForLoopTimerMsULong is initialized to millis() at power upand again after each cycle so we are measuring cycle time.
****************************************************/
{						
	if ((millis() - StartForTimeStampULong) < 30000)//don't monitor cycle time for first 30 seconds to give time for one time time delays to run.
	{//don't time loop() until FDRoneTimeRun() is done
		StartForLoopTimerMsULong = millis();//keep resetting this until 30 seconds is over
		return;
	}else
	{
		if(resetLoopTimerBool)
		{//user has held up the loop so don't generate software warning. Reset loop timer.
			StartForLoopTimerMsULong = millis();//this causes previous loop to be ignored
			resetLoopTimerBool = false;//clear loop timer reset flag
			return;
		}
		
		LoopCycleTimeMS_ULong = millis() -  StartForLoopTimerMsULong;
		
		if(LoopCycleTimeMS_ULong > maximumLoopCycleTimeMS_UInt)
		{					
			maximumLoopCycleTimeMS_UInt = LoopCycleTimeMS_ULong;		
			
			if(verboseQ)
			{				
				Serial.println();
				Serial.print("The maximum core 0 cycle is ");
				Serial.print(maximumLoopCycleTimeMS_UInt);
				Serial.println(" ms.");
			}
		}
		
		if(LoopCycleTimeMS_ULong > LoopCycleTimeLimitMS_UInt)
		{
			WriteControlBlock(ErrorCodeAddressByte,loopTimeMoreThanLimitByte); //log error in control block
			Serial.println();
			Serial.println("	Excessive core 0 loop cycle time ");
			Serial.println();
		}		
		StartForLoopTimerMsULong = millis();//reset loop() timer
	}
}//end of LoopCycleTimeMonitor()

void Fdr::LapseTimerSeconds()
{ 
	if (JustPoweredUpFlag == true)
	{
						#ifdef DiagPrint9
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						#endif
		
		return; //do not advance time if we just powered up because OnTheGround()//ControlTheFlightParameters() has not run yet to initialize related parameters
	}
	
	TimeNowSecondsInt = int((millis() - StartTimeMSULong + 500)/1000); //round to the nearest second
	if ((TimeNowSecondsInt - Last_TimeNow_SecondsInt) > (SamplingRateSecondsByte - 1))
	{ //we have to advance SamplingRateSecondsByte second in order to say we have gone SamplingRateSecondsByte seconds since last reading of ports. CR3.2 
		TimeToTakeSampleBool = true; //set seconds flag if we just started new second
		//TimeToTakeSampleBool set false just before ports scanned 
		Last_TimeNow_SecondsInt = TimeNowSecondsInt;	 
	}
	//if TimeNowSeconds is equal to LastTimeNowSeconds, there has not been an advanced to the next second so do nothing	
}//end of LapseTimerSeconds()


void Fdr::ScanForCommand()
{

					#ifdef menuQ1
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
	//if(!Serial.dtr())return;//detect if laptop connected. If not, just return because their can't be any commands. If a laptop is connected, check for incoming command. If there is one, set flags for requested tasks	qaz
	if (Serial.available() > 0) 
	{ //if there is at least one character in the buffer, process first one

					#ifdef menuQ
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif

		Command = Serial.read();
		
					#ifdef menuQ
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print("Command = ");
					Serial.println(Command);
					#endif
		
		emptyUSBreceiveBufferQ(false);//was true  throw away all other characters in buffer. true means wait 500 ms to collect all characters before starting to dump buffer
		
		switch(Command)
		{
			case 'M':
			break;
			
			case 'm':
			break;

			case 'D':
			break;
			
			case 'd':
			break;				

			case 'A':
			break;
			
			case 'a':
			break;

			case 'S':
			break;
			
			case 's':
			break;				

			case 'P':
			break;
			
			case 'p':
			break;

			case 'O':
			break;
			
			case 'o':
			break;				

			case 'H':
			break;
			
			case 'h':
			break;

			case 'R':
			break;
			
			case 'r':
			break;

			case 'T':
			break;				

			case 't':
			break;
			
			case 'C':
			break;

			case 'c':
			break;				
			
			default:
			Serial.print(Command);
			Command = 'm'; //map any character except M, m, D, A, C, S, P, O, H, T, r, R, C, and c to "M" for display of the short menu. Set to "N" when done processing command.
			
			Serial.println(" is an invalid keystroke interpreted as m.");
			PrintLine(true);
		}	
	}else //no keystroke received so see if we need to print a prompt
	{
		if (printTheUserPromptQ && Command == 'N')
		{ //print a command prompt once after we return to idle
				if(digitalRead(teamNumberReadPortByte) == team2Bool) 
				{//we read back a logic high which is the same as a boolean true and means this is team 2
					Serial.print("What is your bidding, ");
					Serial.print(team2NameChar);
				}else
				{//we read back a logic low which is the same as a boolean false and means this is team 1
					Serial.print(" What is your bidding, ");
					Serial.print(team1NameChar);
				}
				Serial.print("? ");
				printTheUserPromptQ = noBool;//just printed prompt so don't print again until we get a user response		
		}
	}
}//end of ScanForCommand()


void Fdr::StopCollectingDataQ()
{
	if ((Command == 'S')||(Command == 's')){
		Serial.println(" S");
		Serial.println();
		Serial.println();
			Serial.println(F("Data collection has stopped and the EEPROM is locked."));	
		Serial.println();
		WriteControlBlock(ProgramStateAddressByte,MemoryLockedByte);
		*recordingDataBool = false;		
		Command = 'N'; //clear Command
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed
	}
}

	
void Fdr::OutputDataFileQ()
{
	if (Command == 'O')
	{ //Output formatted data to file on laptop
		//unsigned int startTimeOfDataDumpUInt = millis();
		Serial.println(" O");//echo command 
		Serial.println("Printed data is formatted.");		
		Serial.println("Be sure to have your Tera Term log file enabled.");
		#ifdef firstSemester
			Serial.println("It takes about 1 minute to print 30 minutes of data.");
		#endif
		#ifdef secondSemester
			Serial.println("It takes about 1 minute to print 60 minutes of data.");
		#endif
		Serial.println("Press any key to start the data dump.");
		Serial.println("Then you can press S to stop the data dump.");		
		PrintLine(true);
		while(Serial.available() == 0);//wait for user to press any key
		emptyUSBreceiveBufferQ(false);//throw away any response
		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								

								
								#endif 	
		
		
		OutputDataToLogFile();
		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	
		
		
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed
	}
	if (Command == 'o')
	{ 
		WriteControlBlock(dataDumpStatusAddressByte,dataBeingDumpedByte);//tell system we are dumping data. This is used by core1 during heartbeat to know when to not expect response.
		Serial.println(" o");
		Serial.println("Printed data is unformatted.");
		Serial.println("Be sure to have your Tera Term log file enabled.");
		Serial.println("It takes about 1 minute to print 60 minutes of data.");
		Serial.println("Press any key to start the data dump.");	
		Serial.println("Then you can press S to stop the data dump.");		
		PrintLine(true);
		while(Serial.available() == 0);//wait for user to press any key
		emptyUSBreceiveBufferQ(false);//throw away any response
		Serial.println("Unformatted EEPROM data:");
		Serial.println();
		resetLoopTimerBool = true;//prevents loop timer complaint

		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif
								
		unsigned long timeStudyStartTimeULong = millis();//used to measure total lapse time
				
		if(!dumpEEPROM())
		{
			Command = 'N';//retire command because we are done
			printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed
			return;//raw dump of EEPROM or simulated EEPROM. This function does return true for success and false if there is a failure or abort. Return on false so an abort doesn't dumpSEEPROM
		}
		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	
		
		
		Serial.println();
		
		Serial.println("Unformatted SEEPROM data:");
		Serial.println();
		
		dumpSEEPROM();//raw dump of SEEPROM or simulated EEPROM. This function does return true for success and false if there is a failure or abort but we don't care.
		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	
		
		
		float minutesLapsedFloat = float((millis() - timeStudyStartTimeULong))/60000;
		Serial.println();
		Serial.print("  Output took ");
		Serial.print(minutesLapsedFloat,1);
		Serial.println(" minutes to print.");
		Serial.println();

		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint	
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed
	
		WriteControlBlock(dataDumpStatusAddressByte,dataNotBeingDumpedByte);//tell system we are not dumping data anymore. This is used by core1 during heartbeat to know when to expect response.
	}
}//end of OutputDataFileQ()

void Fdr::DiagnosticOutputControlBlockQ(){
	if ((Command == 'H')|| (Command == 'h')){ //Output control block
		Serial.println(" H");
		Serial.println();
		Serial.println("********** System Information **********");
		Serial.println();
		dumpBus1LogFile();//disableLogFunctionBool is set in Fdr.h
		calculateAndPrintPort6AverageSamplePeriod();
		DiagnosticOutputControlBlock();
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint
		//resetLoop1TimerBool = true;	////prevents loop1 timer complaint
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed		
	}
}

	
void Fdr::SingleDisplayDataQ()
{
	if((Command == 'D')||(Command == 'd'))
	{//Display all port voltages
		Serial.println(" D");		
		OutputSingleLiveScan();
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer 
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed		
	}
}

void Fdr::ContinuousDisplayDataQ()
{
	if ((Command == 'A')|| (Command == 'a') )
	{
		if(ContinuousDisplayDataFlag)//then A was previously entered so we are printing data to the screen when we received another A which means stop printing
		{
			ContinuousDisplayDataFlag = false;//stop live printing to screen
			Serial.println();
			Serial.println("A.     Continuous printing stopped.");
			Serial.println();
			Command = 'N'; //clear Command
			printTheUserPromptQ = yesBool;//print command prompt because we are done with continuous print so give user a prompt
			return;
		}else//then A was not previously entered so we are not printing data to the screen when we received A which means start printing
		{//start live printing
			ContinuousDisplayDataFlag = true; //flag used by DataIn; true means we will print collected data to the screen.
			Serial.println(" A.     Enter 'A' again to stop printing to the screen.");//echo back "A" and then inform user how to stop screen prints.		
		}
		resetLoopTimerBool = true;//prevents loop timer complaint
				
		printHeadingsForLiveOutput(); //tell user they will see port readings as they are collected
		Command = 'N'; //clear Command
		printTheUserPromptQ = noBool;//disables command prompt in this case becuase is messes up print of data	
	}
}//end of ContinuousDisplayDataQ()

void Fdr::OutputMenuQ()
	{		
	if(Command == 'M')
	{
		Serial.println(" M");		
		OutputMenuOfCommandsAndLED_Cadences(true);//output long form
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint
		//resetLoop1TimerBool = true;	//prevents loop1 timer complaint	
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed		
		return;
	}
	if(Command == 'm')
	{
		Serial.println(" m");	
		OutputMenuOfCommandsAndLED_Cadences(false);//output short form
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint
			
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed	
		return;
	}		
}

void Fdr::clearRecordedErrorsQ()
	{		
	if((Command == 'C') || (Command == 'c'))
	{//Clear the recorded error, if any
		Serial.println(" C");
		Serial.println();
		if(ReadControlBlock(ErrorCodeAddressByte) != SystemNormalFlagByte)
		{
			DisplayErrorState();
			Serial.println("Error has been cleared.");
			Serial.println();
			WriteControlBlock(ErrorCodeAddressByte,SystemNormalFlagByte); //set error state to system normal
		}else
		{
			Serial.println("There is no error to clear.");
			Serial.println();
		}
		Command = 'N'; //clear Command
		resetLoopTimerBool = true;//prevents loop timer complaint
				
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed		
		return;
	}
}


void Fdr::RebootQ(){		
	if (Command == 'R')
	{
		Serial.println(" R");		
		Serial.println(" Processor is now ready for new software.");
		rp2040.rebootToBootloader();//same as pushing the boot button
		Command = 'N'; //clear Command although the reboot wipes this out anyway		
		return;
	}
	if(Command == 'r')
	{
		Serial.println(" r");	
		Serial.println();		
		Serial.println(" The two processors will now be reset.");
		Serial.println();
		Serial.println();
		Serial.println();
		Serial.println();		
		Command = 'N'; //clear Command although the reboot wipes this
		rp2040.reboot();

	}	
}

void Fdr::PrepareForLaunchQ(){
	if ((Command == 'P') || (Command == 'p')){
		Serial.println(" P");		
		SendPrepareForLaunchWarning();		
		ActOnResponseToPrepareForLaunch();
		Command = 'N'; //clear Command although the reboot wipes this out anyway	
	}
}

void Fdr::GenerateTestPatternQ()
{
	if (Command == 'T')
	{	
		bool for_T_patternBool = true;
		
		Serial.println(" T");
		GenerateTestPatternWarning(for_T_patternBool);
		ActOnResponseToGenerateTestPattern();
		resetLoopTimerBool = true;//prevents loop timer complaint
			
		Command = 'N'; //clear Command
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed				
	}
	
	if(Command == 't')
	{
		bool for_t_patternBool = false;
		
		Serial.println(" t");		
		GenerateTestPatternWarning(for_t_patternBool);
		ActOnResponseToGenerateBinaryTestPattern();
		resetLoopTimerBool = true;//prevents loop timer complaint
			
		Command = 'N'; //clear Command
		printTheUserPromptQ = yesBool;//enables prompt to be printed once after last command completed	
	}		
}

void Fdr::DataIn()
{
	if (ReadControlBlock(ProgramStateAddressByte)== MemoryLockedByte)
	{
		enablePort6HighSpeedRead = false;//stops fdr.Port6Peak() from running
		return;
	}
	
	if (ReadControlBlock(ProgramStateAddressByte) < TestForInFlightByte )
	{
		//if true, we are in flight
		
		enablePort6HighSpeedRead = true;//lets fdr.Port6Peak() run
		
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
		
		if(TimeToTakeSampleBool == true)
		{	
			//if true, it is time to take a sample from all ports except port 6. We will use the peak for port 6.
			
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
			
			RecordData();//read all 8 analog ports and store data in EEPROM.In the first semester,  store GNSS data in the SEEPROM. In second semester, store port data in the EEPROM. Students store GNSS and I2C data in the SEEPROM.
			
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
			
		}else{
			//if not time to take samples, take a reading from port 6 and see if it is a peak
			Port6Peak();
			
			#ifdef DiagPrint9
			ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
			if (ErrorCodeByte != OldErrorCodeByte)
			{
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" ErrorCodeByte = "));
			Serial.println( ErrorCodeByte );
			OldErrorCodeByte = ErrorCodeByte;
			}
			#endif
			
		}
	}

	
					#ifdef DiagPrint1
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif

					#ifdef DiagPrint9
					ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
					if (ErrorCodeByte != OldErrorCodeByte)
					{
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.println(millis() - StartForTimeStampULong);
					Serial.print(F(" ErrorCodeByte = "));
					Serial.println( ErrorCodeByte );
					OldErrorCodeByte = ErrorCodeByte;
					}
					#endif

}//end of DataIn()


/********************************************************************************
L E V E L  3  S U B R O U T I N E S
*********************************************************************************/
void Fdr::RecordData()
{
	
						#ifdef DiagPrint6
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						#endif
	
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
	
	TimeToTakeSampleBool = false;
	ReadEEPROM_Pointer(CalledByRecordDataByte);//updates startOfNextMemoryBlockLong
	
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
		
	RecordPortAndGNSSReadingsAndOptionallyPrintsToScreen();
	
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
		
	IncrementEEPROM_Pointer();//it reads the pointer in the EEPROM and increments it by the block size but does not write it back into the EEPROM
	
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
		
	WriteEEPROM_Pointer(); //writes startOfNextMemoryBlockLong
	
						#ifdef DiagPrint9
						ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
						if (ErrorCodeByte != OldErrorCodeByte)
						{
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" ErrorCodeByte = "));
						Serial.println( ErrorCodeByte );
						OldErrorCodeByte = ErrorCodeByte;
						}
						#endif
			
}//end of RecordData()


void Fdr::OutputMenuOfCommandsAndLED_Cadences(bool longQBool)
{// *menu*
	Serial.println();
	Serial.println(F(" Glendale Community College Flight Data Recorder"));
	printTeamsName();
	Serial.println();
#ifdef firstSemester
	Serial.println(F(" The library is configured for the first semester."));
	Serial.println();
#endif
	
#ifdef secondSemester
	Serial.println(F(" The library is configured for the second semester."));
	Serial.println();
#endif
	
	Serial.println(F("   M    The full menu."));
	Serial.println(F("   m    The short menu."));	
	Serial.println(F("   D    Display all port voltages."));
	Serial.println(F("   P    Prepare for launch!"));
	Serial.println(F("   A    Display all data live. Enter A again to stop. Works only while in flight state."));
	Serial.println(F("   S    While in flight state, stop data collection and prepare to output data."));	
	Serial.println(F("   O    Output formatted data file. Press S key to stop it."));
	Serial.println(F("   o    Output unformatted data file."));
	Serial.println(F("   H    Diagnostic: Output system information."));
	Serial.println(F("   T    Generate a test pattern with voltages and GNSS data usable by Excel."));
	Serial.println(F("   t    Generate a test pattern of sequential numbers usable by Excel."));
	Serial.println(F("   R    Reboot in preparation for download."));
	Serial.println(F("   r    Reset processor."));
	Serial.println(F("   C    Clear all recorded errors."));
	Serial.println(F(" ALT+ES Clear Tera Term screen."));
	Serial.println(F(" ALT+FL Name Tera Term receiving file and start receiving data."));
	Serial.println(F("        Enter file name and press Save."));
	Serial.println(F(" ALT+FQ Close Tera Term receiving file."));
	Serial.println();
	Serial.println();
	if(longQBool)
	{//long form of menu
		Serial.println(F("      LED flash rate                    Description"));
		Serial.println(F(" ===============================  ========================"));
		Serial.println(F(" __________"));
		Serial.println(F("           |"));
		Serial.println(F("           |__________"));
		Serial.println(F(" 0.5 sec on;0.5 sec off            in flight"));
		Serial.println();	
		Serial.println(F("  _   _   _"));
		Serial.println(F(" | | | | | |"));	
		Serial.println(F(" | |_| |_| |_________"));	
		Serial.println(F(" flicker for 0.5 sec;0.5 sec off   in flight with fault(s)"));
		Serial.println();
		Serial.println(F(" _________________"));
		Serial.println(F("                  |"));
		Serial.println(F("                  |____"));
		Serial.println(F(" 0.8 sec on;0.2 sec off            memory locked"));
		Serial.println();
		Serial.println(F(" ____"));
		Serial.println(F("     |"));
		Serial.println(F("     |_______________"));	
		Serial.println(F(" 0.2 sec on;0.8 sec off            pre-flight"));
		/*  when in pre-flight, core 0 is halted so processing not possible. core 1 is running but doesn't get update, I think. No matter because a text warning has been added that is more effective.
		Serial.println();
		Serial.println(F("  _   _   _   _   _   _"));
		Serial.println(F(" | | | | | | | | | | | |"));	
		Serial.println(F(" | |_| |_| |_| |_| |_| |_"));		
		Serial.println(F(" 0.2 sec on;0.2 sec off            pre-flight with fault(s)"));
		*/
		Serial.println();
		Serial.println(F(" _"));
		Serial.println(F("  |"));	
		Serial.println(F("  |__________________"));			
		Serial.println(F(" short flash per sec               core 0 crashed or dumping data"));
		Serial.println();
		Serial.println();
		Serial.println(F(" always on or always off           core 1 crashed"));
		Serial.println();
		Serial.println();
		Serial.println(F(" off                               no battery"));
		Serial.println();
		Serial.println();
		PrintLine(false);
		if(*GNSS_Bool)
		{
			Serial.println();
			Serial.println("          ******GNSS error codes******");
			Serial.println();
			Serial.println("           200	GNSS OK but no data yet");
			Serial.println("           201	GNSS hardware failure");
			Serial.println("           203	correct NMEA header could not be found");
			Serial.println("           205	GNSS has not updated this field");
			Serial.println("          *****************************");
			Serial.println();		
		}
		Serial.println(F(" Fdr Library source file path: "));
		Serial.println(SourceFile);
		Serial.println();
	}
}

void Fdr::OutputSingleLiveScan()
{
	byte PortNumberByte = 0;
	float PortVoltageFloat = 0;
	float BateryVoltageFloat = readBattery(outputAsVoltageBool);
			
			#ifdef ADCtest 
			Serial.print("core ");
			Serial.print(rp2040.cpuid());
			Serial.print(" ");
			Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.println(F(" ms"));
			Serial.print(F("BateryVoltageFloat = "));
			Serial.println(BateryVoltageFloat);
			#endif 
			
	Serial.println();
	Serial.println("System Status, Port Voltages, and GNSS Status");
	PrintLine(false);//print a line of ======
	DisplayErrorState(); //if any error detected, tell user
	PrintLine(true);
	Serial.println();
	Serial.println("Display all Port Voltages");
	Serial.println();
	if (BateryVoltageFloat < 5)
	{ //when battery is off, we see less than 5V and sensors plus camera do not get power
		Serial.println("Battery is disconnected so sensors");
		Serial.println("do not have power.");
		Serial.println("All port voltages set to 5.25 volts.");
		Serial.println();
		if(simulatorModeQ)Serial.println("In simulation mode.");
	}else
	{
		Serial.print("Battery voltage is ");
		Serial.print(BateryVoltageFloat,1); //output format x.xxx
		Serial.println(" V");
	}
	Serial.println();
	Serial.println(F("Port    Voltage"));
	Serial.println(F("====   ======="));
	for (PortNumberByte = 0; PortNumberByte < 7; PortNumberByte++)
	{
			printOneSpace();
			Serial.print(PortNumberByte);
			Serial.print(F("        "));
			PortVoltageFloat = LivePortVoltageReadingFloat(PortNumberByte);
			Serial.println(PortVoltageFloat,3); //output format x.xxx	
	}
	Serial.println();
	//output local time or GNSS status if GPS or GNSS equipped
	
					#ifdef GPSout
					Serial.print("*NavigationDataByte[0] = ");
					Serial.println((*NavigationDataByte)[0]);
					Serial.print("*NavigationDataByte[1] = ");
					Serial.println((*NavigationDataByte)[1]);
					Serial.print("*NavigationDataByte[2] = ");
					Serial.println((*NavigationDataByte)[2]);
					#endif
	
	if(*GNSS_Bool)
	{
		printArizonaTimeOrGNSSstatus();		
		Serial.println();
		Serial.println();
	}
}//end of OutputSingleLiveScan()

void Fdr::printArizonaTimeOrGNSSstatus()
{
	byte hoursByte = 0;
	byte hoursAMPMByte = 0;
	if((*NavigationDataByte)[0] < 24)//it is time not an error code
	{
		hoursByte = ((*NavigationDataByte)[0] + 17) % 24;//UTC-7 is local miliary time. Modulo 24, -7 is the same as +17. Calculate modulo 24 to prevent negative values. Since nav data is a byte, if I subtract 7, I get wrap around to near 256 so would be a bug.
		
		#ifdef gpstestPrint
		Serial.print("hoursByte = ");
		Serial.println(hoursByte);
		#endif
		
		//hoursByte has values from 0 to 23.
		
		if (hoursByte > 12)
		{
			hoursAMPMByte = hoursByte - 12;//convert to AM/PM
		}else
		{
			hoursAMPMByte = hoursByte;
		}
		
		if (hoursByte == 0) //special case where hours is 0 but we write 12 for AM/PM format
		{
			hoursAMPMByte = 12;
		}
		
		Serial.println();
		Serial.print(F("Arizona time: "));
		Serial.print(hoursAMPMByte);
		Serial.print(":");
		if((*NavigationDataByte)[1] < 10)//minutes
		{
			Serial.print("0");//if < 10, print leading 0
		}
		Serial.print((*NavigationDataByte)[1]);
		Serial.print(":");
		if((*NavigationDataByte)[2] < 10)//seconds
		{
			Serial.print("0");
		}
		Serial.print((*NavigationDataByte)[2]);		
		if(hoursByte > 12)
		{			
			Serial.print(" PM");
		}else
		{
			Serial.print(" AM");
		}
	}else
	{
		Serial.print(F(" GNSS status: "));
		printGNSSstatusInEnglish();
	}
}//end of printArizonaTimeOrGNSSstatus()

void Fdr::JustWait()
{
	WriteControlBlock(ProgramStateAddressByte,MemoryLockedByte);
	WriteControlBlock(ErrorCodeAddressByte,SystemNormalFlagByte);
	//verify program state write worked; I don't check write to error code because if it fails, no point in writing error to it. Therefore, I must assume that write worked or there is no point in proceeding.
	if ((ReadControlBlock(ProgramStateAddressByte)!= MemoryLockedByte) || (ReadControlBlock(ErrorCodeAddressByte) != SystemNormalFlagByte))
	{
		WriteControlBlock(ErrorCodeAddressByte,JustWaitReadBackFailureFlagByte);
	}
}

void Fdr::printHeadingsForLiveOutput()
{
	if (ReadControlBlock(ProgramStateAddressByte)== MemoryLockedByte){
		Serial.println();
		Serial.println(F("Memory is locked,so no new data was recorded."));		
		return;
	}
	if (ReadControlBlock(ProgramStateAddressByte ) < TestForInFlightByte )
	{
		Serial.println();
		Serial.println();
		Serial.println(F("WARNING: A battery voltage less than 6V means sensors and camera may not operate correctly."));
		Serial.println(F("                                                  |"));
		Serial.println(F("                                                  ^"));
		//Serial.println();
		//Serial.println();
		
		#ifdef firstSemester
		if(*GNSS_Bool)
		{
			Serial.println(F("Port0  Port1  Port2  Port3  Port4  Port5  Port6  batt   |H:M:S| Lat-D:M:S N/S Lon-D:M:S E/W|Alt, feet"));
		}else
		{
			Serial.println(F("Port0  Port1  Port2  Port3  Port4  Port5  Port6  batt"));
		}
		#endif
		
		#ifdef secondSemester
			Serial.println(F("Port0  Port1  Port2  Port3  Port4  Port5  Port6  batt"));
		#endif
	
	}
}//end of printHeadingsForLiveOutput()

void Fdr::OutputDataToLogFile()
{//data was collected every SamplingRateSecondsByte (set to 2) seconds but we will output data every second by over sampling
//output line format is
//Seconds,Port0,Port1,Port2,Port3,Port4,Port5,Port6 peak,Battery, H:M:S; Lat D:M:S N/S, Lon D:M:S E/W; alititude in feet if first semester
//hit any key to abort data dump

								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

	WriteControlBlock(dataDumpStatusAddressByte,dataBeingDumpedByte);//tell system we are dumping data. This is used by core1 during heartbeat to know when to not expect response.
	timeStudyStartTimeULong = millis();	
	bool oddSecondBool = true;
	bool evenSecondBool = false;
	resetLoopTimerBool = true;//prevents loop timer complaint
	int TimeStampSecondsInt = -1;//causes first print to be at 1
	
	WriteControlBlock(ProgramStateAddressByte,MemoryLockedByte); //lock memory so no data can be overwritten
	

#ifdef firstSemester
	//estimate total print time for first semester
	ReadEEPROM_Pointer(CalledByOutputRunTimeEstimateByte);//result is placed in startOfNextMemoryBlockLong. pPrameter that is passed is used for debugging since 5 fucntions call readEEPROM_Pointer()

	//count lines to print
	long numberofLinesPrintedLong = 2*((startOfNextMemoryBlockLong - 16L - StartOfUsableEEPROM_Long)/16L);//I subtract 16 bytes from startOfNextMemoryBlockLong so I don't count the control block. I subtract StartOfUsableEEPROM_Long because the control block can be moved which leaves fewer addresses for data. The number of lines is 2 x block count because of oversampling.
	float numberOfLinesPrintedPerNumberOfMinutesFloat = 16380/*lines*//6.5669/*minutes*/;
	float estimatedRunTimeMinutesFloat = float(numberofLinesPrintedLong)/numberOfLinesPrintedPerNumberOfMinutesFloat;// I measured 16,380 lines of data printed in 6.5669 minutes so 2494.3 lines of data/minute. So dividing by this number should give me minutes to print the data out.
	Serial.println();
	Serial.print("*** Estimated time to dump data is ");
	Serial.print(estimatedRunTimeMinutesFloat,1);
	Serial.print(" minutes. ***");	
	Serial.println();
	Serial.println();
#endif

#ifdef firstSemester
	if(*GNSS_Bool)
		{
		Serial.println("Run, Time,Port_Voltages,,,,,,,Battery,Az_Time,,,Latitude,,,,Longitude,,,,altitude");
		Serial.println(F("Minutes,Seconds,Port_0,Port_1,Port_2,Port_3,Port_4,Port_5, Port_6,voltage,H,M,S,D,M,S,N/S,D,M,S,E/W,feet"));//output titles of the columns. Port2 is a peak reading found between port scans
		}else
		{
		Serial.println("Run, Time,Port_Voltages,,,,,,,Battery");
		Serial.println(F("Minutes,Seconds,Port_0,Port_1,Port_2,Port_3,Port_4,Port_5, Port_6,voltage"));//output titles of the columns. Port6 is a peak reading found between port scans}
		}
#endif

#ifdef secondSemester
	Serial.println("Run,Time,Port_Voltages,,,,,,,Battery");
	Serial.println(F("Minutes,Seconds,Port_0,Port_1,Port_2,Port_3,Port_4,Port_5, Port_6,voltage"));
	#endif

	ReadEEPROM_Pointer(CalledByOutputDataToFileByte); //get pointer to end of data which is called. CalledByOutputDataToFileByte is the calling subroutine's ID. Output is to the variable startOfNextMemoryBlockLong.
	
	emptyUSBreceiveBufferQ(false);//be sure receive UART buffer is empty. false means we don't wait 500 ms before emptying buffer.


								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

		
	for(long EEPROM_PointerLong = StartOfUsableEEPROM_Long + 16; EEPROM_PointerLong < startOfNextMemoryBlockLong;EEPROM_PointerLong = EEPROM_PointerLong + DataBlockSizeByte) //we start at StartOfUsableEEPROM_Long and right after control block. change this number if control block increases. startOfNextMemoryBlockLong is next empty data block so stop before we get here. jump the pointer by the size of the data block each cycle. Data block is 16 bytes now.
	{
		
								#ifdef speedOfPrintTest 
								CFLTS
								#endif 	

		
		if(Serial.available() > 0)
		{
			//a key has been pressed. Only "S" means to abort the data dump. All others are ignored.
			if(Serial.read() == 'S')
			{
				Serial.println();
				Serial.println("Data Dump has been aborted.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				*EEPROMjustOutputtedBool = false;//tells students in second semester not to output data. Not sure I need to do this.
				return;//leave OutputDataToLogFile() so data dump has terminated
			}else
			{
				Serial.println();
				Serial.println("Invalid command. If you wish to stop the data dump, enter S.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				delay(1000);//give time for user to react.
			}
		}
			
		//move to start of each block of data and sequence through all of the blocks 
		//first output field is the time stamp unless we had power hit	
		//do a fast read of one block of data starting at EEPROM_PointerLong and put result into portDataByte[].
		if(!readEEPROMdataBlock(EEPROM_PointerLong))
		{
			Serial.print("readEEPROMdataBlock() failed at line ");
			Serial.println(__LINE__);
		}

								#ifdef speedOfPrintTest 
								CFLTS
								#endif 	

		
		if(portDataByte[14]+portDataByte[15] == 0)
		{
		
		//if (ReadEEPROM(EEPROM_PointerLong + 14L) + ReadEEPROM(EEPROM_PointerLong + 15L) == 0){ //I'm reading the battery voltage. If I read 0, it means there was  a power hit recorded
		
			Serial.println();
			Serial.println(F("Power was disrupted."));
			TimeStampSecondsInt = -1; //when battery voltage reads 0, it means power hit recorded so restart time stamp count. I add 2 to this value before using it so by starting at -1, my first line of data is marked +1 second. RGS1.3
		}else
		{//power not disrupted
			
									#ifdef printTimeStudy
									timeStudyStartTimeULong = millis();//reset timer
									Serial.print("time stamp at ");
									Serial.print(__LINE__);
									Serial.print(" is ");
									Serial.println(millis() - timeStudyStartTimeULong);
									#endif
			
			//no power hit so print data and continue with time
			TimeStampSecondsInt = TimeStampSecondsInt + int(SamplingRateSecondsByte); //blocks are built every SamplingRateSecondsByte seconds so block count is run time; first block gets time stamp of SamplingRateSecondsByte.
			//building a Comma Separated Volume (CSV) output
			int minutesInt = TimeStampSecondsInt/60;//integer division means we round down in minutes. had to use int because max time is more than 255 minutes
			byte secondsByte = TimeStampSecondsInt - 60*minutesInt;//total seconds minus minutes displayed yields remaining seconds after we display minutes
			Serial.print(minutesInt);
			Serial.print(",");
			Serial.print(secondsByte);
			//after time, in seconds is printed, we dump all analog port data
			
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	
			
			
			outputPortVoltages(EEPROM_PointerLong,oddSecondBool);//output is sent to log file and also stored in portVoltagesFloat[8] for use by even numbered seconds without having to regenerate it.
		
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

					
		#ifdef firstSemester
			if(*GNSS_Bool)
			{
			readSEEPROMdataBlock(EEPROM_PointerLong);//save data in GNSSdataByte[]
			formattedOutputGPSdata(oddSecondBool); //after analog port data printed, we print out the fully formatted GNSS data plus save to GNSSdataByte[] and the formatted altitude is stored in  GNSSaltitudeLong. This data is printed out during even seconds.
			}else
			{
				Serial.println();//line feed would have been supplied by outputGPSdata() but since GNSS not equipped, add line feed here
			}
		#endif
			
		#ifdef secondSemester
				Serial.println(); //first semester, line feed supplied by outputGPSdata(). Second semester, I will add it here. No prints from SEEPROM done by me this semeseter
		#endif

								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

					
			//output same data again but show time stamp plus 1 second. This gives an output file with data every second which will match up with ANSWR's GPS data
			
			//now at an even second. TimeStampSecondsInt is advanced by 2 seconds and is always odd so add 1 to get even seconds		
			minutesInt = (TimeStampSecondsInt+1)/60;//integer division means we round down in minutes. had to use int because max time is more than 255 minutes
			secondsByte = (TimeStampSecondsInt+1) - 60*minutesInt;//total seconds minus minutes displayed yields remaining seconds after we display minutes
			Serial.print(minutesInt);
			Serial.print(",");
			Serial.print(secondsByte);

								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

			
			outputPortVoltages(EEPROM_PointerLong,evenSecondBool);//data was stored in portVoltagesFloat[8]
			
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	
			
			
		#ifdef firstSemester
			if(*GNSS_Bool)
			{
				readSEEPROMdataBlock(EEPROM_PointerLong);//save data in GNSSdataByte[]
				formattedOutputGPSdata(evenSecondBool);//data stored in GNSSdataByte[16]
			}else
			{
				Serial.println();//line feed would have been supplied by outputGPSdata() but since GNSS not equipped, add line feed here
			}	
			#endif
		
		#ifdef secondSemester
			Serial.println(); //first semester, line feed supplied by outputGPSdata(). Second semester, I will add it here
		#endif
				
			*EEPROMjustOutputtedBool = true;//is available for students to know that the EEPROM was just outputted so they should output the SEEPROM		
		}//end of code when power was not disrupted
	}//all data was output

	float minutesLapsedFloat = float((millis() - timeStudyStartTimeULong))/60000;
	Serial.println();
	Serial.print("  Output took ");
	Serial.print(minutesLapsedFloat,1);
	Serial.println(" minutes to print.");
	Serial.println();
	
								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(" ms.");
								#endif 	

						
	WriteControlBlock(dataDumpStatusAddressByte,dataNotBeingDumpedByte);//tell system we are not dumping data. This is used by core1 during heartbeat to know when to expect response.			
}//end of OutputDataToLogFile()

void Fdr::DiagnosticOutputControlBlock()
{
//also see fullEEPROMtest();//diag this is full test of EEPROM and takes about 3 hours.

	partialEEPROMtest();//tests first 64 bytes of data storage
	printControlBlockStateAndError();//output Control Block plus file name 	
	
	Serial.println();
	Serial.println(F("Clear fault with Pre-flight or the C command."));
	
	printAmountOfDataStoredAsHMS();//output amount of memory used for data as hours, minutes, and seconds	
	printTotalRunTime();	
	printFileInfo();
}//end of DiagnosticOutputControlBlock()

void Fdr::SendPrepareForLaunchWarning()
{
	PrintLine(true);//RGS1.2
	Serial.println(F("You asked to prepare for launch.")); 
	//see if there are recorded faults and if so, warn user

	if(ReadControlBlock(ErrorCodeAddressByte) != 0)
	{
			Serial.println();
			topOfBox();			
			Serial.println("   *       Errors detected.       *");
			Serial.println("   *    Launch not recommended.   *");
			bottomOfBox();
			Serial.println();
	}
	Serial.println();
	Serial.println(F("This will erase all of your existing data."));
	Serial.println("	Are you sure? Y/N");//verify user wants to do this
	Serial.println(F("Answer Y or N"));	
}//end of SendPrepareForLaunchWarning()


void Fdr::GenerateTestPatternWarning(bool typeBool)
{ 
		PrintLine(true);//RGS1.2
		Serial.println(F("You asked to generate a test pattern in the data")); 
		if(typeBool)
		{
			Serial.println("that can be turned into graphs in EXCEL.");
		}else
		{
			Serial.println("with acending number in the EEPROM and decending numbers in the SEEPROM.");
		}
		Serial.println();
		Serial.println("This will erase all data. Are you sure? Y/N");
}


void Fdr::ActOnResponseToPrepareForLaunch()
{
	while (true){//wait until a character comes in
		if (Serial.available() > 0)
		{
			Command = Serial.read(); 
			if ((Command == 'Y')|| (Command == 'y'))
			{	
				Serial.println(" Y");
				PrepareForLaunch();
				LEDflashInfo();//update so core1 flashes correctly in the preflight state.
				PrintLine(true);//RGS1.2
				Serial.println(F("Ready to launch: Turn power off"));
				Serial.println();
				//Serial.println();

				NonFaultStopProgram();//we only return if in simulation mode. Command is left at Y or y
				Command = 'N';//retire preflight command and print the short menu
				OutputMenuOfCommandsAndLED_Cadences(false);
				return;
			}
			
			if ((Command == 'N')|| (Command == 'n'))
			{//previously, this would abort if command was not Y or y and that caused an abort when newline was automatically added to carriarge return. Now logic only responds to no. All other characters are ignored.	
				Serial.println(" N");
				Serial.println (F("Flight request has been aborted."));
				resetLoopTimerBool = true;//prevents loop timer complaint			
				printTheUserPromptQ = yesBool;//trigger prompt for a command
				return;
			}
		}	
	}
}//end of ActOnResponseToPrepareForLaunch()


void Fdr::ActOnResponseToGenerateTestPattern()
{	
	while (1)
	{//wait until a character comes in
		if (Serial.available() > 0){
			Command = Serial.read(); 
			if ((Command == 'Y')|| (Command == 'y'))
			{
				PrintLine(true);//RGS1.2
				Serial.println(F("Pattern generation started."));
				Serial.println("....................");	
				GenerateTestPattern();
				Serial.println();
				Serial.println ("Done");
			}else{
				Serial.println (F("Pattern request aborted"));
			}
			resetLoopTimerBool = true;//prevents loop timer complaint
			return;
		}	
	}
}//end of ActOnResponseToGenerateTestPattern()

void Fdr::ActOnResponseToGenerateBinaryTestPattern()
{	
	while (1)
	{//wait until a character comes in
		if (Serial.available() > 0){
			Command = Serial.read(); 
			if ((Command == 'Y')|| (Command == 'y'))
			{
				PrintLine(true);//RGS1.2
					Serial.println(F("Pattern generation started."));
					Serial.println("................");
				GenerateBinaryTestPattern();
				Serial.println();
				Serial.println ("Done");
			}else{
				Serial.println (F("Pattern request aborted"));
			}
			return;
		}	
	}
}

void Fdr::GenerateTestPattern()
{
/*********************************************************************
I need to be able to generate a test pattern in the data. LSB is about 0.005V but I display down to 0.01V. This means I should increment starting at bit 2 which means increment in steps of 2. I don’t want any data to be the same as adjacent data so can start first port at 0*2 so will show 0, the second port at 1*2 and should shown 0.01v, third 2*3 and should show 0.03, etc. On next block, the first port will be at 2+16 = 18. 

GNSS data is also simulated.
***********************************************************************/

	long NumberOfTestBlocksLong = 20L; //CR3.2
	long TopOfMemoryLong = 0;
	int StartingValueInt = 0;
	int PortStepSizeInt = 1; //since bit 2, this value is multiplied by 2
	byte TestValueHighByte = 0;
	byte TestValueLowByte = 0;
	unsigned long AddressUnsignedLong = 0;
	int TestValueInt = StartingValueInt;
	int TestValueBatteryInt = 0;
#ifdef firstSemester
	byte simSecondsByte = 1;//to match time stamp
#endif
	WriteControlBlock(ProgramStateAddressByte,InFlightByte); //change to in-flight so memory can be written
	
	for (TopOfMemoryLong = StartOfUsableEEPROM_Long + 16; TopOfMemoryLong < ((NumberOfTestBlocksLong*DataBlockSizeByte)+StartOfUsableEEPROM_Long+16);TopOfMemoryLong = (TopOfMemoryLong+DataBlockSizeByte))
	{//move across the blocks of data to fill all of the specified blocks of memory. Avoid the control block
		for (byte PortPointerByte = 0; PortPointerByte < 7; PortPointerByte++)
		{//move along current block of data but stop before battery value
			AddressUnsignedLong = TopOfMemoryLong + (2*PortPointerByte);//generate address of next byte to be written
			TestValueHighByte = highByte(TestValueInt);
			TestValueLowByte = lowByte(TestValueInt);//parce test value into its bytes
			WriteEEPROM_Data(AddressUnsignedLong,TestValueHighByte); //write test bytes to EEPROM 
			WriteEEPROM_Data(AddressUnsignedLong+1L,TestValueLowByte);
			TestValueInt = TestValueInt + PortStepSizeInt*2;//advance test value	
		}//end of write of all ports except battery which is on Port 7
		
		TestValueBatteryInt =  (TestValueInt/2) + 1435;//battery reading which must be >5 to avoid battery off warning. 2048 is equiv to 3.07V which is multiplied by 2 in output code
		AddressUnsignedLong = TopOfMemoryLong + (2*7); //battery's address	which is port 7		
		TestValueHighByte = highByte(TestValueBatteryInt);
		TestValueLowByte = lowByte(TestValueBatteryInt);
		WriteEEPROM_Data(AddressUnsignedLong,TestValueHighByte);
		WriteEEPROM_Data(AddressUnsignedLong+1L,TestValueLowByte);
		//all analog ports done

	#ifdef firstSemester
		if(*GNSS_Bool)
		{
			byte simGNSSByte = 1;//matches time stamp
			//H:M:S,D:M:S N, D:M:S E, ALTITITUDE in 4 bytes, meters

			
			for (byte byteIndexByte = 0;byteIndexByte < 16; byteIndexByte++)
			{		//if GNSS equipped, write GPS to SEEPROM using same base address but now just index through bytes
				AddressUnsignedLong = TopOfMemoryLong + byteIndexByte;
				if((byteIndexByte != 6) && (byteIndexByte != 10)&& (byteIndexByte != 15))
				{
					simGNSSByte = random(0, 59);//keep value below 60.
				}else
				{
					simGNSSByte = 0;//legal values
				}
				if((byteIndexByte == 0))//make hours more reasonable
				{
					simGNSSByte = 9;
				}
				if((byteIndexByte == 1))//make minutes more reasonable
				{
					simGNSSByte = 10;
				}
				if((byteIndexByte == 2))//make seconds sequential
				{
					simGNSSByte = simSecondsByte;
					simSecondsByte = simSecondsByte +2;//account for oversampling
				}
				
				WriteSEEPROM(AddressUnsignedLong, simGNSSByte);//move simulated GNSS array into SEEPROM's data block qaz GLOBAL BUG: I FORGOT TO PROCESS RETURN VALUE FROM THE WRITE WHICH CAN CONTAIN FAILURES.
			}
		}
	#endif
	
		TestValueInt = TestValueInt + PortStepSizeInt*2;//so average of adjacent blocks for the same port is not truncated
		Serial.print(F("."));//status to user - one block written
	}//end of block so move to next block
	
	startOfNextMemoryBlockLong = (NumberOfTestBlocksLong*DataBlockSizeByte)+StartOfUsableEEPROM_Long; //update memory pointer so output subroutine knows when to stop
	WriteEEPROM_Pointer(); //writes startOfNextMemoryBlockLong
	WriteControlBlock(ProgramStateAddressByte, MemoryLockedByte); //done generating test data so lock memory
}//end of GenerateTestPattern()

void Fdr::GenerateBinaryTestPattern()
{//Starting just above the control block, write sequential numbers from 0 to 255 in the first 255 bytes of the EEPROM and from 255 down to 0 in the SEEPROM. Update the next available address to reflect the new data's top limit.
	long eeAddressLong;
	byte testDataByte = 0;
	WriteControlBlock(ProgramStateAddressByte,InFlightByte); //change to in-flight so memory can be written
	for(eeAddressLong = StartOfUsableEEPROM_Long +16L ;eeAddressLong < (StartOfUsableEEPROM_Long +16L + 256L); ++eeAddressLong)
	{
		UnprotectedWriteEEPROM(eeAddressLong,testDataByte);
		WriteSEEPROM(eeAddressLong,255 - testDataByte);
		if(eeAddressLong % 16 == 0)Serial.print(".");//status indicator for user
		testDataByte++;
	}
//update next available data block pointer	
	startOfNextMemoryBlockLong = StartOfUsableEEPROM_Long + 16L +256L;
	WriteEEPROM_Pointer();
	WriteControlBlock(ProgramStateAddressByte, MemoryLockedByte); //done generating test data so lock memory
}//end of GenerateBinaryTestPattern()

void Fdr::ReadEEPROM_Pointer(byte ID)
//A long is 4 bytes. Each read is a byte that I first convert to a long. Then I multiply each converted byte by an integer defined as a long and add them up. The sum is a long. In this way we don't mix data types.
{ //output is startOfNextMemoryBlockLong
//ID used to figure out who called this subroutine since it is called by 4 subroutines
	(void) (ID); //Kevin added this but I'm not clear why although I see that ID is passed to this function but I don't use it. 

						#ifdef dumpLook1
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

	byte b0 = ReadControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte);
	
						#ifdef dumpLook1
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
	byte b1 = ReadControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte + 1);
	byte b2 = ReadControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte + 2);
	byte b3 = ReadControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte + 3);
	
	startOfNextMemoryBlockLong = changeBytesToUnsignedLong(b3,b2,b1,b0);	
} //end of ReadEEPROM_Pointer()

void Fdr::WriteEEPROM_Pointer()
{//the variable startOfNextMemoryBlockLong is written to the control block
	WriteControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte,byte(startOfNextMemoryBlockLong)); //this takes the lowest byte of the float. 	
	WriteControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte+1,byte(startOfNextMemoryBlockLong>>8)); //this  takes the second byte 
	WriteControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte+2,byte(startOfNextMemoryBlockLong>>16)); //this takes the third byte 
	WriteControlBlock(StartOfNextMemoryBlockPointerBaseAddressByte+3,byte(startOfNextMemoryBlockLong>>24)); //this  takes the forth byte 
}

void Fdr::RecordPortAndGNSSReadingsAndOptionallyPrintsToScreen()
{ //records one block of data
//Ports 0-6 are analog input voltage ports. Port 7 is the battery monitor. Port 6 is read at a higher rate than other ports. There are always 8 channels for a total of 16 bytes.
 
	*recordingDataBool = true;//set student's flag to say I am recording data into the EEPROM.
	PortDataAddressLong = startOfNextMemoryBlockLong;//begin new block of data

									//if (ContinuousDisplayDataFlag == true)Serial.println(); //used on the ground to test system; force new line before outputting data
	
	//Record and optonally print to screen all port voltages. Then record and otpionally print GNSS data.
	for (byte PortCountByte = 0; PortCountByte < 8; PortCountByte++)
	{//0 to 7		
		if(PortCountByte < 6)
		{//0 to 5
			PortReadingInInt = ADCread(PortCountByte);//conversion to voltage is applied when printed out
			//this value is 2's complement so we could have a negative number
			//28,000 for no battery, 29,333 for failed ADC, 32,000 failed to respond
			
									#ifdef ADCbug
									Serial.println();
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("Port ");
									Serial.print(PortCountByte);
									Serial.print(" has a count of ");
									Serial.print(PortReadingInInt);
									Serial.println(" counts");
									#endif
						
		}
		
		if(PortCountByte == 6)
		{//high speed function that takes peak positive reading
			PortReadingInInt = Port6PeakInt;//port6 was read as often as possible in order to find the peak over the last waiting interval. Rather than read it now, we use the peak value that was found
			Port6PeakInt = 0;//prepare for the next peak detection interval
		}
		
		if(PortCountByte == 7) //battery measured via PM's ADC
		{
			if((JustPoweredUpForTimeStampFlag == true) && (ReadControlBlock(ProgramStateAddressByte) == InFlightWithPowerDisruptionByte))
			{ 
			//this is the battery data. If we just recovered from a power hit, set battery voltage value to 0 to indicate power hit to output function
				PortReadingInInt = 0; 
				JustPoweredUpForTimeStampFlag = false; //is set at power up and this is the only use so can be cleared here

								#ifdef newGPSfeature
									Serial.print(F("line number "));			
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.println(millis() - StartForTimeStampULong);
									#endif
				
			}else
			{//not a power hit so read battery
				//float batteryVoltageFloat = readBattery(outputAsCountBool);
				//PortReadingInInt = int(batteryVoltageFloat);
				
				PortReadingInInt = int(readBattery(outputAsCountBool));//readBattery() can output count or voltage but is defined as a float so if we want the count, I must convert the float to an integer. Volage conversion is performed when printed out.
				

									#ifdef batteryReading
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.println(__LINE__);	
									Serial.print("battery count directly from fcn as an int is ");
									Serial.println(PortReadingInInt);
									#endif
						
			}	
									#ifdef DiagPrint7
									Serial.print(F("line number "));			
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.println(millis() - StartForTimeStampULong);
									#endif
		}				

		//write the current port data
		PortReadingInHighByte = highByte(PortReadingInInt);	
		PortReadingInLowByte = lowByte(PortReadingInInt);
		
		
									#ifdef ADCbug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("Port ");
									Serial.print(PortCountByte);
									Serial.print(" has a count of ");
									Serial.print(PortReadingInInt);
									Serial.println(" counts ");									
									#endif		
		
								#ifdef DiagPrint9
								ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
								if (ErrorCodeByte != OldErrorCodeByte)
								{
									Serial.print(F("line number "));			
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.println(millis() - StartForTimeStampULong);
									Serial.print(F(" ErrorCodeByte = "));
									Serial.println( ErrorCodeByte );
									OldErrorCodeByte = ErrorCodeByte;
								}
								#endif
		
								#ifdef DiagPrint9
								Serial.print(F("line number "));			
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.println(millis() - StartForTimeStampULong);
								Serial.print(F(" PortDataAddressLong = "));
								Serial.println( PortDataAddressLong );
								#endif
		
		WriteEEPROM_Data(PortDataAddressLong,PortReadingInHighByte);//if address is pointing within control block, error is generated 
		WriteEEPROM_Data(PortDataAddressLong+1L,PortReadingInLowByte);

	#ifdef firstSemester
		/*************************************************
		GNSS put here because I want to use the same address as used on EEPROM for SEEPROM	
		If equipped, write GPS to SEEPROM using same address
		PortCountByte goes 0 - 7 but I store 16 bytes by taking two bytes per port so it matches my navigational array
		***************************************************/
		if(*GNSS_Bool)
		{
			WriteSEEPROM(PortDataAddressLong, (*NavigationDataByte)[2*PortCountByte]);//even bytes 0 2 4 6 8 10 12 14  
			WriteSEEPROM(PortDataAddressLong+1L, (*NavigationDataByte)[(2*PortCountByte) + 1]);//odd bytes 1 3 5 7 9 11 13 15 
		}
	#endif	

 				
								#ifdef GPSdataStore
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print("PortDataAddressLong = ");
								Serial.println(PortDataAddressLong);
								Serial.print("PortCountByte = ");
								Serial.println(PortCountByte);
								Serial.print("(*NavigationDataByte)[2*PortCountByte] =");
								Serial.println((*NavigationDataByte)[2*PortCountByte]);
								#endif
					
								#ifdef DiagPrint9
								ErrorCodeByte = ReadControlBlock(ErrorCodeAddressByte);
								if (ErrorCodeByte != OldErrorCodeByte)
								{
									Serial.print(F("line number "));			
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.println(millis() - StartForTimeStampULong);
									Serial.print(F(" ErrorCodeByte = "));
									Serial.println( ErrorCodeByte );
									OldErrorCodeByte = ErrorCodeByte;
								}
								#endif
	
		if (ContinuousDisplayDataFlag == true)
		{//format and output current port reading which is a count
			if (PortCountByte != 7)
			{
				
									#ifdef ADCbug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("Port ");
									Serial.print(PortCountByte);
									Serial.print(" has a count of:: ");
									Serial.println(PortReadingInInt);
									#endif					
				
				VoltageFloat = ADCconvertToVoltage(PortReadingInInt);//for ports 0 - 6
				
				
									#ifdef ADCbug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("Port ");
									Serial.print(PortCountByte);
									Serial.print(" = ");
									Serial.print(VoltageFloat,3);
									Serial.println(" volts");
									Serial.println("port count was ");
									Serial.print(PortReadingInInt);
									#endif				
						
			}
			if (PortCountByte == 7)
			{ //battery monitor so different ADC. It has 12 bits rather than 15 plus we have a voltage divider in front of it.
				VoltageFloat = readBattery(asAVoltageBool);

				
									#ifdef ADCbug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("Port ");
									Serial.print(PortCountByte);
									Serial.print(" = ");
									Serial.print(VoltageFloat,3);
									Serial.println(" volts");
									#endif				
 				
			}
			
			Serial.print(VoltageFloat,3);//used on the ground in Flight state to test system; 3 places past decimal point
			printTwoSpaces(); //put two spaces after each reading
			
		}//end of continuous print code that is within the storing of port and GNSS data to EEPROM and SEEPROM 

		PortDataAddressLong = PortDataAddressLong + 2L; //advance to the next port data address.
	}//done printing all ports so print GNSS data next

	if (ContinuousDisplayDataFlag == true)
	{

#ifdef firstSemester		
	//after printing all analog ports, print GNSS values. The GNSS runs continuous in the background and updates NavigationDataByte[] when it can so I just need to read this array and format it
	//output UTC time if GNSS equipped
		if(*GNSS_Bool)
		{
			printUTCTime();//it ensures all fields have 2 digits by adding a 0 when fields is < 10
			//output Lat
			Serial.print((*NavigationDataByte)[3]);//degrees Lat
			printColon();
			Serial.print((*NavigationDataByte)[4]);//minutes Lat
			printColon();	
			Serial.print((*NavigationDataByte)[5]);//seconds Lat
			printOneSpace();
			if((*NavigationDataByte)[6] == 0)
			{
				Serial.print("N");			
			}

			if((*NavigationDataByte)[6] == 1)
				{
					Serial.print("S");
				}
			//if it isn't 0 or 1, print its value
			if((*NavigationDataByte)[6] > 1)
			{
				Serial.print((*NavigationDataByte)[6]);
			}
		
			printTwoSpaces();
			//output Lon
			Serial.print((*NavigationDataByte)[7]);//degrees Lon
			printColon();
			Serial.print((*NavigationDataByte)[8]);//minutes Lon
			printColon();		
			Serial.print((*NavigationDataByte)[9]);//seconds Lon
			printOneSpace();
			if((*NavigationDataByte)[10] == 0)
			{
				Serial.print("E");			
			}
			if((*NavigationDataByte)[10] == 1)
				{
					Serial.print("W");
				}
			if((*NavigationDataByte)[10] > 1)//if it isn't 0 or 1, print its value
			{
				Serial.print((*NavigationDataByte)[10]);
			}

			printTwoSpaces();
			//output 4 byte altitude in feet if valid. It should arrives in meters.
			//suppress output if not valid

			byte b3 = (*NavigationDataByte)[11];
			byte b2 = (*NavigationDataByte)[12];
			byte b1 = (*NavigationDataByte)[13];
			byte b0 = (*NavigationDataByte)[14];


			if((*NavigationDataByte)[15] == 0)//this means output is in meters as expected so safe to calc altitude
				{//translate the bytes and convert to feet
					float altitudeFloat = float(changeBytesToUnsignedLong(b3,b2,b1,b0))*metersToFeetFloat;//assemble number and covert from meters to feet
					Serial.println(altitudeFloat,0);//do line feed at end
				}else
				{					
					Serial.println((*NavigationDataByte)[15]);
				}
		}else  //end of printing of GNSS data if it has been provisioned
		{//we don't have a GNSS so just output a new line
			Serial.println();//if not printing GNSS data, I still need to have a line feed
		}
	#endif //end of first semester code
	
	#ifdef secondSemester
		Serial.println(); //first semester, line feed supplied by dumping GPS data. Second semester, I will add it here
	#endif
	}//end of continuous output code

		resetLoopTimerBool = true;//prevents loop timer complaint
}//end of RecordPortAndGNSSReadingsAndOptionallyPrintsToScreen()
	
void Fdr::IncrementEEPROM_Pointer(){	
/*****************************************************************
EEPROM is 2^17 = 131,072 bytes going from address 0 to 131071.  A block is DataBlockSizeByte (16 bytes). Addresses 0 through 15 are for the control block.

This first data block goes from address 16 through [(16+16) -1] =  31. The next block starts at (16 + 16 =) 32 and goes through 47.

The max number of data blocks is {[2^17] - control block}/16 = (131072 - 16)/16 = 8191 block of data.

The last data block starts at 16 + (16*8190) = 131,056. If we went one more data block, it would start at 131072 which is beyond available memory.  

DataBlockSizeByte is in a #define so is a substitution within the compiler. Therefore it does not have Byte added to the end.

If the control block's memory is no longer reliable, we will get readback errors. Then we update startOfNextMemoryBlockLong in increments of 16 to move both the control block and the start of data memory that follows it.
*******************************************************************/

	ReadEEPROM_Pointer(CalledByIncrementEEPROM_PointerByte); //it outputs to startOfNextMemoryBlockLong; this is the start of the block that was just written.
	if (startOfNextMemoryBlockLong <= (131072 - (2*DataBlockSizeByte)))
	//if we just wrote the second to last memory block, we can increment but if we just wrote the last memory block or beyond, we have no more room
	{
		startOfNextMemoryBlockLong = startOfNextMemoryBlockLong + DataBlockSizeByte;//there is room so increment to address of the next data block start.
	}else
	{//there is not enough room for this next DataBlock. Lock the EEPROM without incrementing and return.
		WriteControlBlock(ProgramStateAddressByte,MemoryLockedByte);
	}
}

/********************************************************************************
L E V E L  4  S U B R O U T I N E S
*********************************************************************************/

void Fdr::DisplayErrorState()
{		
	Serial.print("Last Recorded error: "); 
	switch(ReadControlBlock(ErrorCodeAddressByte))
	{
		case 0:  //SystemNormalFlagByte
		Serial.println("none - System normal.");
		break;
		
		case 1: //OutOfRangeReadFlagByte
		Serial.println("Out Of Range Read");
		break;	

		case 2: //OutOfRangeWriteFlagByte
		Serial.println("Out Of Range Write");
		break;

		case 3: //ControlTheFlightParametersReadyForLaunchReadBackFailureFlagByte
		Serial.println("Control The Flight Parameters Ready ForLaunch Read Back Failure");
		break;
		
		case 4: //ControlTheFlightParametersInFlightFlagReadBackFailureFlagByte
		Serial.println("Control The Flight Parameters In Flight Flag Read Back Failure");
		break;		
		
		case 5: //JustWaitReadBackFailureFlagByte
		Serial.println("Just Wait Read Back Failure");
		break;
		
		case 6: //PrepareForLaunchReadBackFailureFlagByte
		Serial.println("Prepare For Launch Read Back Failure");
		break;	

		case 7: //WriteEEPROM_AttemptMadeToWriteToLockedMemoryFlagByte
		Serial.println("Write EEPROM Attempt Made To Write To Locked Memory");
		break;

		case 9:  //SerialAvailableReturnCodeOutOfRangeByte
		Serial.println("Serial Available Return Code Out Of Range");
		break;	

		case 10: //ReadBackFromEEPROM_MismatchByte
		Serial.println("Read Back From EEPROM Mismatch");
		break;

		case 11:  //AttemptMadeToWriteToControlBlockByte
		Serial.println("Attempt Made To Write To Control Block");
		break;	

		case 12: //AttemptMadeToWriteToDataBlockByte
		Serial.println("Attempt Made To Write To Data Block");
		break;	

		case 13: //equippedGPShardwareFaultByte
		Serial.println("GPS or GNSS hardware Fault");
		break;	

		case 14: //equippedMPMhardwareFaultByte
		Serial.println("equipped MPM hardware Fault");
		break;	

		case 15:  //equippedModemHardwareFaultByte
		Serial.println("equipped Modem Hardware Fault");
		break;	

		case 16:  //equippedModemSetupFaultByte
		Serial.println("equipped Modem Setup Fault");
		break;	

		case 17:  //EEPROMdataFailureByte
		Serial.println("EEPROM data Failure");
		break;	

		case 18:  //loopTimeMoreThanLimit
		Serial.print("loop() Time More Than limit of ");
		Serial.print(LoopCycleTimeLimitMS_UInt);
		Serial.println(" ms.");
		break;			

		case 19:  //loop1TimeMoreThanLimit
		Serial.print("loop1() Time More Than limit of ");
		Serial.print(LoopCycleTimeLimitMS_UInt);
		Serial.println(" ms.");
		break;

		case 20:  //SEEPROMdataFailureByte
		Serial.println("SEEPROM data Failure");
		break;			
		
		default:
		Serial.println("Unknown system failure");	
		break;
	}					
}//end of DisplayErrorState()

void Fdr::PrepareForLaunch()
{
	WriteControlBlock(ProgramStateAddressByte,PreFlightByte); //set program state to pre-flight
	startOfNextMemoryBlockLong = StartOfUsableEEPROM_Long + 16;//just above the control block
	WriteEEPROM_Pointer(); //the variable startOfNextMemoryBlockLong is written to the control block
	WriteControlBlock(ErrorCodeAddressByte,SystemNormalFlagByte); //set error state to system normal
}

float Fdr::LivePortVoltageReadingFloat(byte PortNumberByte)
{
	//LogicalAnalogPinByte[ ] maps port number to analog logical pin number
	float PortReadingFloat = 0;
	PortReadingFloat = ADCconvertToVoltage(ADCread(PortNumberByte));//read count for this port and convert to a voltage in float. Port number > 7  or no ACK return 5.5V.

	if(PortNumberByte == 7)
	{//we are at port 7 which is the battery monitor
		PortReadingFloat = PortReadingFloat*batteryVoltageDividerCompensationFloat;//battery monitor has a voltage divider using 100K to ground and 220K to battery for a 1:3.2 divider which means multiply by 3.2 to get battery voltage.
	}
		
	
				#ifdef ADCtest
				Serial.print("core ");
				Serial.print(rp2040.cpuid()); 
				Serial.print(" ");
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				Serial.print(F(" ADC port = "));
				Serial.print( PortNumberByte );
				Serial.print(F(". Voltage reading = "));
				Serial.println(PortReadingFloat,3);
				#endif	
				
	return PortReadingFloat;
}//end of LivePortVoltageReadingFloat(byte PortNumberByte)

/********************************************************************************
L E V E L  5  S U B R O U T I N E S
**********************************************************************/

void Fdr::NonFaultStopProgram()
{ //effectively stop program but does look for 'r' command. It is called when preflight invoked and we are waiting for user to remove power.

	emptyUSBreceiveBufferQ(false);//dump buffer but don't wait 500 ms first
	
	Serial.println("No further commands accepted until after power turned");
	Serial.println("off and then back on except for reboot (r) command.");
	Serial.println();
	
	while(1)
	{	
		if(Serial.read() == 'r')
		{
			Serial.println();
			Serial.println();
			Serial.println();
			waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. I measured 1828 ms. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.			
			if(simulatorModeQ)
			{
				PrepareForLaunch();//this sets up control block and sets us to pre-flight. We do not go back to setup(), as we would with real reboot, so will not change from pre-flight to flight. Therefore, we do it next.
				WriteControlBlock(ProgramStateAddressByte,InFlightByte);
				return;//Command is left at Y or y
			}else
			{
				rp2040.reboot();//if we are not in simulator mode, reboot the RP2040. no need for return since we jump to a restart of the processors. A reboot in simulator mode wipes the RAM so we don't get into the flight state.
			}				
		}else
		{			
			Serial.print("%");
			delay(50);	
		}
	}
}//end of NonFaultStopProgram()


void Fdr::FaultStopProgram()
{ //effectively stop program and turns off LED. It is not being used right now
	while(true)
	{ //CR1.9
		digitalWrite(ExternalLED, ExternalLED_OffByte);
	}
}

void Fdr::WriteControlBlock(byte eeAddressByte, byte dataByte) /*************************************************
function accepts a single byte for the address and converts it to a long because this is the control block which will always contain less than 255 bytes

The Control Block is relocatable using StartOfUsableEEPROM_Long. As we wear out the EEPROM, we can move the CB. Data memory always starts after the CB.

eeAddressByte does not change as the CB is moved. 
***************************************************/
{
	if(eeAddressByte > 15)
	{//if true, it is a write beyond the control block to flag error

					#ifdef CBfatal
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
				
		WriteControlBlock(ErrorCodeAddressByte, AttemptMadeToWriteToDataBlockByte);
		return;//we didn't sieze the bus so don't have to release it
		//this is not a recursive call because we don't go back to this place in the function.
	}
	
					#ifdef CBfatal
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.print(millis() - StartForTimeStampULong);
					Serial.print(".  Data is ");
					Serial.println(dataByte);
					#endif
					
	UnprotectedWriteEEPROM(StartOfUsableEEPROM_Long + long(eeAddressByte), dataByte);

					#ifdef CBfatal
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif

				#ifdef errorTrap
				if(eeAddressByte == ErrorCodeAddressByte)
				{
					Serial.println();
					Serial.print("someone wrote ");
					Serial.print(data);
					Serial.println(" to the Error Code Address");	
				}
				#endif
}//end of WriteControlBlock()

byte Fdr::ReadControlBlock(byte eeAddressByte)
{ 
/***************************************************
subroutine accepts a single byte for the address and converts it to a long because this is the control block which will always contain less than 255 bytes.

The CB address is from 0 to 15. Any relocation is handled by this ReadControlBlock() function: The Control Block is relocated using StartOfUsableEEPROM_Long. As we wear out the EEPROM, we move the start of the usable EEPROM with CB at the beginning. Data memory always starts after the CB.
**************************************************/

						#ifdef CBfatal
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print("eeAddressByte = "):
						Serial.println(eeAddressByte);
						#endif
			
			
						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
	byte dataByte = ReadEEPROM(StartOfUsableEEPROM_Long + long(eeAddressByte)); 

						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

	return dataByte;
}//end of ReadControlBlock()

/********************************************************************************
E E P R O M   S U B R O U T I N E S
*********************************************************************************/
//Based on eeprom example from SparkFun Electronics June 11th, 2017

void Fdr::WriteEEPROM_Data(long eeAddressLong, byte data)
{//RGS1.2
	if (eeAddressLong < 16L)
	{//data write attempted into control block
		WriteControlBlock(ErrorCodeAddressByte,AttemptMadeToWriteToControlBlockByte);
		return; //set error value but keep collecting data
	}else{//all other writes pass through
		UnprotectedWriteEEPROM(eeAddressLong,data);
	}			
}

void Fdr::UnprotectedWriteEEPROM(long eeAddressLong, byte dataByte)
{

//Function accepts a 4 byte address. All non-hardware errors are logged to the control block. Readback errors are hardware related so just print to USB. Trying to use the EEPROM to log the error may cause another readback error and we will end up with recursive errors which lock up the system.

//The EEPROM's spec sheet shows a Block Select Bit (B0) and two bytes of address. When eeAddressLong is < 65536, the lower block is accessed so B0 = 0. All of my addresses are this case as verified with a scope.

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.print(millis() - StartForTimeStampULong);
					Serial.print("; EEPROM address is ");
					Serial.print(eeAddressLong);
					Serial.print(" and data is ");
					Serial.println(dataByte);
					#endif

	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	
					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif	
	
	if(simulatorModeQ)
	{//simulated EEPROM is a 1600 byte array
		if(eeAddressLong > 1599)eeAddressLong = 1599;//map all out of range data into last byte
		simEEPROM[int(eeAddressLong)] = dataByte;
		return;//we didn't request the bus so don't have to release it
	}

	if (eeAddressLong > 131071L){ //max address is 2^17 - 1 = 131071
		WriteControlBlock(ErrorCodeAddressByte,OutOfRangeWriteFlagByte);
		return;//we didn't request the bus so don't have to release it
	}

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
					
	I2Cbus1Mitigation(unprotectedWriteEEPROMuserByte);//siezes bus and identify yourself. If other core is using it, we wait.	

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
	if (eeAddressLong < 65536L)
	  {
		Wire1.beginTransmission(EEPROM_ADR_LOW_BLOCK);
		//eeAddressLong &= 0xFFFF; //Erase the upper 16 bits of the long variable qaz
	  }
	  else
	  {
		Wire1.beginTransmission(EEPROM_ADR_HIGH_BLOCK);
	  }

	Wire1.write((byte)(eeAddressLong >> 8)); // queue MSB
	Wire1.write((byte)(eeAddressLong)); //  queue LSB
	Wire1.write(dataByte); // queue single byte
	
	if(Wire1.endTransmission(true) == 2)//transmit address and stop symbol
	{
		Serial.print("Failed to receive ACK at line ");
		Serial.println(__LINE__);
	}
  
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms
	//write has completed
	releaseI2Cbus();
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif
			
//verify write worked
	if (ReadEEPROM(eeAddressLong) != dataByte)
	{//read back of data failed so I can't trust EEPROM to log error. Just print out error.
		Serial.print(F("EEPROM write failed. Error is Read Back From EEPROM Mismatch and is at address "));
		Serial.print(eeAddressLong);
		Serial.print(".   Line ");
		Serial.println(__LINE__);
		return;//bus was released
	}
	//readback was successful.
	
					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
	return;//bus was released	
}//end of UnprotectedWriteEEPROM()

byte Fdr::ReadEEPROM(long eeAddressLong)
{ //subroutine accepts a 4 byte address and returns a single byte of data. Also drives ReadEEPROMfailureQBool flag.

				#ifdef timeRreadEEPROM
				unsigned long timerStartULong = millis();//flag if read takes too long
				#endif

	
						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(" ms   start of ReadEEPROM()");
						#endif
	
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	
						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
	if(simulatorModeQ)
	{//simulated EEPROM is 1600 bytes
		if(eeAddressLong > 1599)eeAddressLong = 1599;//map all out of range data into last byte
		return simEEPROM[int(eeAddressLong)];
	}else
	{
		
						#ifdef lockup
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	}

	if (eeAddressLong > 131071L)
	{ 
		WriteControlBlock(ErrorCodeAddressByte, OutOfRangeReadFlagByte);
		
						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
	return 0;//we return 0 for the data but the error is logged. No need to release the bus because we didn't take it yet
	}
	//emptyUSBreceiveBufferQ(false);//defensively empty receive buffer and don't wait 500 ms first. It will empty buffer for up to 1 second and then silently give up. qaz
	
						#ifdef readDebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
	I2Cbus1Mitigation(readEEPROMuserByte);//siezes bus and identify yourself and your core.	

						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
						
	if (eeAddressLong < 65536L) 
	{ //send block address based on eeAddressLong 
		Wire1.beginTransmission(EEPROM_ADR_LOW_BLOCK);
	}else
	{//eeAddressLong >= 65536L
		Wire1.beginTransmission(EEPROM_ADR_HIGH_BLOCK);
	}
  //then send data address within the block
	Wire1.write((byte)(eeAddressLong >> 8)); //queue MSB
	Wire1.write((byte)(eeAddressLong)); //queue LSB
	
	if(Wire1.endTransmission(true) == 2)//transmit address and send stop symbol
	{//a return value of 2 means ACK failure. Since EEPROM can't be reached, print failure to USB
		Serial.print("Failed to receive ACK at line ");
		Serial.println(__LINE__);
		releaseI2Cbus();//release bus and core
		ReadEEPROMfailureQBool = true;
		return 0;//EEPROM access failed so return 0.
	}
	//to get here, we got ACK so can request data from EEPROM
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms so wait 10 for now qaz.
	
						#ifdef readDebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));	
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
	//then get single data byte from EEPROM
	if (eeAddressLong < 65536L)
	{
		Wire1.requestFrom(EEPROM_ADR_LOW_BLOCK,1,true);//request 1 byte and then apply stop symbol
	}else
	{
		Wire1.requestFrom(EEPROM_ADR_HIGH_BLOCK,1,true);//request 1 byte and then apply stop symbol
	}
	byte readDataByte = 0;
	delay(waitForEEPROMresponseMsByte);//give EEPROM time to respond to the request for data

						#ifdef readDebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print("Wire1.available() = ");
						Serial.println(Wire1.available());
						#endif
			


	switch(Wire1.available())
	{
		case 0://no byte returned
				
							#ifdef lockup
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
							
			//can't log fact that I was unable to read back from EEPROM because EEPROM can't be reached so  just print out warning. 
			Serial.print("EEPROM did not respond to read by ReadEEPROM() while running on core ");
			Serial.print(rp2040.cpuid());
			Serial.print(" Line ");
			Serial.print(__LINE__);
			Serial.print(" using address ");
			Serial.println(eeAddressLong);
			ReadEEPROMfailureQBool = true;
			releaseI2Cbus();
		break;
		
		case 1://one byte returned (sunny day)	
			//24LC1025 spec sheet says data should be in buffer immdiately after I2C command run but I waited waitForEEPROMresponseMsByte just to be safe
			readDataByte = Wire1.read(); //if EEPROM and its I2C is available, return the single byte.
			releaseI2Cbus();
			
						#ifdef readDebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(" ms");
						#endif
		break;
				
		default://more than 1 byte in buffer
			readDataByte = Wire1.read();//take first byte
			FlushWire1Buffer();//toss excess bytes but also print them out as diag
			releaseI2Cbus();
			
						#ifdef readDebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
						
			WriteControlBlock(ErrorCodeAddressByte, EEPROMdataFailureByte);//log fact that I read back more than 1 byte from EEPROM. I assume we did reach EEPROM but it returned more bytes than expected.			
			Serial.print("EEPROM returned more than the expected single byte while in ReadEEPROM() on core ");
			Serial.print(rp2040.cpuid());
			Serial.print(". Line ");
			Serial.print(__LINE__);
			ReadEEPROMfailureQBool = true;
		break;
	}//end of evaluating and reading returned byte(s)

						#ifdef rcb1
						unsigned long runTimeUlong = millis()- timerStartULong;
						if (runTimeUlong > 100)
						{
							Serial.print("ReadEEPROM() took ");
							Serial.print(runTimeUlong);
							Serial.print( " ms.  Line ");
							Serial.println(__LINE__);
						}
						#endif

	
						#ifdef rcb
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms.    end of ReadEEPROM()"));
						Serial.print("  eeAddressLong = ");
						Serial.println(eeAddressLong);
						#endif
	
	return readDataByte;//is valid data with no error logged or 0 with error logged. Bus released 
}//end of ReadEEPROM()

bool Fdr::dumpEEPROM()
{/**************************************************************************
This is a raw dump of all bytes in the simulated or real EEPROM

The function returns true for success and false if a failure is encountered. Some failures will also generate prints to the USB.

output is portDataByte[]
*****************************************************************************/

						#ifdef speedOfPrintTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.

						#ifdef speedOfPrintTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
	ReadEEPROM_Pointer(CalledByEEPROMdataDumpByte);//retrieve startOfNextMemoryBlockLong. We dump data from address (StartOfUsableEEPROM_Long + 16) up to the startOfNextMemoryBlockLong. StartOfUsableEEPROM_Long is the start of the Control Block. (StartOfUsableEEPROM_Long + 16) is the start of user data.
	
	long eeAddressLong = 0;
	
						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

	
	if(simulatorModeQ)
	{//simulated EEPROM is 1600 bytes

								#ifdef dumpDebug
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
			
		if(startOfNextMemoryBlockLong > 1599)startOfNextMemoryBlockLong = 1599;//map an out of range startOfNextMemoryBlockLong into last byte of the simEEPROM[] array
		
		for(eeAddressLong = (StartOfUsableEEPROM_Long + 16);eeAddressLong < startOfNextMemoryBlockLong; eeAddressLong++)
		{
			Serial.print(simEEPROM[int(eeAddressLong)]);
			Serial.print(",");
			if((eeAddressLong + 1) % 16 == 0)Serial.println();//output carriage return, line feed after each block of 16 bytes		
		}
		Serial.println();
		Serial.print("End of data dump of simulated EEPROM");
		return true;
	}

						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif


//otherwise, we are addressing the EEPROM

									#ifdef busState
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("I2Cbus1LinkedToCoreByte = ");
									Serial.println(I2Cbus1LinkedToCoreByte);
									Serial.print("I2Cbus1OwnerByte = ");
									Serial.println(I2Cbus1OwnerByte);
									#endif

								#ifdef dumpDebug
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print(F("StartOfUsableEEPROM_Long = "));
								Serial.println(StartOfUsableEEPROM_Long);
								Serial.print(F("startOfNextMemoryBlockLong = "));
								Serial.println(startOfNextMemoryBlockLong);
								#endif

//print all user data
	for(eeAddressLong = (StartOfUsableEEPROM_Long + 16);eeAddressLong < startOfNextMemoryBlockLong; eeAddressLong = eeAddressLong + 16)
	{
		if(Serial.available() > 0)
		{
			//a key has been pressed. Only "S" means to abort the data dump. All others are ignored.
			if(Serial.read() == 'S')
			{
				Serial.println("S");
				Serial.println("Data Dump has been aborted.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				*EEPROMjustOutputtedBool = false;//tells students in second semester not to output data. Not sure I need to do this.
				return false;//leave OutputDataToLogFile() with false to indicate data dump was terminated
			}else
			{
				Serial.println();
				Serial.println("Invalid command. If you wish to stop the data dump, enter S.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				delay(1000);//give time for user to react.
			}
		}

						#ifdef speedOfPrintTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		if(!readEEPROMdataBlock(eeAddressLong))//results go into portDataByte[]. 
		{
			return false;//If there is a problem, fcn returns false so we abort dumpEEPROM()
		}
		for(byte indexByte = 0; indexByte < 16; indexByte++)
		{
			Serial.print(portDataByte[indexByte]);
			Serial.print(",");//add field delimiter
		}
		Serial.println();//add <cr> at end of line of data	
	}//bottom of the address loop. All data has been read and printed out
	
						#ifdef speedOfPrintTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
								
	return true;//dump was successful
}//end of dumpEEPROM()

bool Fdr::dumpSEEPROM()
{/**************************************************************************
This is a raw dump of all bytes in the simulated or real SEEPROM

The function returns true for success and false if a failure is encountered. Some failures will also generate prints to the USB.

output is GNSSdataByte[]
*****************************************************************************/
	

						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing SEEPROM so failure is a valid response and we will just continue without delay.

						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
	ReadEEPROM_Pointer(CalledBySEEPROMdataDumpByte);//retrieve startOfNextMemoryBlockLong. We dump data from address (StartOfUsableEEPROM_Long + 16) up to the startOfNextMemoryBlockLong. StartOfUsableEEPROM_Long is the start of the Control Block. (StartOfUsableEEPROM_Long + 16) is the start of user data.
	
	long eeAddressLong = 0;
	
						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

	
	if(simulatorModeQ)
	{//simulated SEEPROM is 1600 bytes

								#ifdef dumpDebug
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
			
		if(startOfNextMemoryBlockLong > 1599)startOfNextMemoryBlockLong = 1599;//map an out of range startOfNextMemoryBlockLong into last byte of the simSEEPROM[] array
		
		for(eeAddressLong = (StartOfUsableEEPROM_Long + 16);eeAddressLong < startOfNextMemoryBlockLong; eeAddressLong++)
		{
			Serial.print(simSEEPROM[int(eeAddressLong)]);
			Serial.print(",");
			if((eeAddressLong + 1) % 16 == 0)Serial.println();//output carriage return, line feed after each block of 16 bytes		
		}
		Serial.println();
		Serial.print("End of data dump of simulated SEEPROM");
		return true;
	}

						#ifdef dumpLook
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

//otherwise, we are addressing the SEEPROM

								#ifdef busState
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print("I2Cbus1LinkedToCoreByte = ");
								Serial.println(I2Cbus1LinkedToCoreByte);
								Serial.print("I2Cbus1OwnerByte = ");
								Serial.println(I2Cbus1OwnerByte);
								#endif
	

								#ifdef dumpLook
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif


								#ifdef dumpDebug
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print(F("StartOfUsableEEPROM_Long = "));
								Serial.println(StartOfUsableEEPROM_Long);
								Serial.print(F("startOfNextMemoryBlockLong = "));
								Serial.println(startOfNextMemoryBlockLong);
								#endif

//print all user data
	for(eeAddressLong = (StartOfUsableEEPROM_Long + 16);eeAddressLong < startOfNextMemoryBlockLong; eeAddressLong = eeAddressLong + 16)
	{
		if(Serial.available() > 0)
		{
			//a key has been pressed. Only "S" means to abort the data dump. All others are ignored.
			if(Serial.read() == 'S')
			{
				Serial.println("S");
				Serial.println("Data Dump has been aborted.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				*EEPROMjustOutputtedBool = false;//tells students in second semester not to output data. Not sure I need to do this.
				return false;//leave OutputDataToLogFile() with false because data dump  terminated
			}else
			{
				Serial.println();
				Serial.println("Invalid command. If you wish to stop the data dump, enter S.");
				Serial.println();
				emptyUSBreceiveBufferQ(false);//remove any keystrokes so menu is not triggered. 
				delay(1000);//give time for user to react.
			}
		}
		
		if(!readSEEPROMdataBlock(eeAddressLong))//populates GNSSdataByte[]
		{
			return false;//results go into GNSSdataByte[]. If there is a problem, fcn returns false so we abort dumpSEEPROM()
		}
		
		for(byte indexByte = 0; indexByte < 16; indexByte++)
		{
			Serial.print(GNSSdataByte[indexByte]);
			Serial.print(",");//add field delimiter
		}
		Serial.println();//add <cr> at end of line of data	
	}//bottom of the address loop. All data has been read and printed out
	

	
								#ifdef dumpLook
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
	return true;//dump was successful
}//end of dumpSEEPROM()

int Fdr::WriteSEEPROM(long eeAddressLong, byte dataByte)
{

/******************************************************
Function accepts a 4 byte address. All non-hardware errors are logged to the control block. Readback errors are hardware related so just print to USB. Trying to use the SEEPROM to log the error may cause another readback error and we will end up with recursive errors which lock up the system.

The SEEPROM's spec sheet shows a Block Select Bit (B0) and two bytes of address. When eeAddressLong is < 65536, the lower block is accessed so B0 = 0. All of my addresses are this case as verified with a scope.

Student's EEPROM. subroutine accepts a 4 byte address and returns a single integer. If no error, the upper byte will be 0 and the lower byte will be data. 

If there is an error, 0x01ec is returned, where ec is the error code, ec:

 ec		meaning
 00 	the address is out of range
 01		failed to receive ACK
 02		read back failure
	
 ************************************************************/

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.print(millis() - StartForTimeStampULong);
					Serial.print("; SEEPROM address is ");
					Serial.print(eeAddressLong);
					Serial.print(" and data is ");
					Serial.println(dataByte);
					#endif

	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing SEEPROM so failure is a valid response and we will just continue without delay.
	
					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif	
	
	if(simulatorModeQ)
	{//simulated SEEPROM is a 1600 byte array
		if(eeAddressLong > 1599)eeAddressLong = 1599;//map all out of range data into last byte
		simSEEPROM[int(eeAddressLong)] = dataByte;
		return 0x0000;//success
	}

	if (eeAddressLong > 131071L){ //max address is 2^17 - 1 = 131071
		WriteControlBlock(ErrorCodeAddressByte,OutOfRangeWriteFlagByte);
		return 0x0100;
	}

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
					
	I2Cbus1Mitigation(unprotectedWriteSEEPROMuserByte);//siezes bus and identify yourself. If other core is using it, we wait.	

					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
	if (eeAddressLong < 65536L)
	  {
		Wire1.beginTransmission(SEEPROM_ADR_LOW_BLOCK);
		//eeAddressLong &= 0xFFFF; //Erase the upper 16 bits of the long variable qaz
	  }
	  else
	  {
		Wire1.beginTransmission(SEEPROM_ADR_HIGH_BLOCK);
	  }

	Wire1.write((byte)(eeAddressLong >> 8)); // queue MSB
	Wire1.write((byte)(eeAddressLong)); //  queue LSB
	Wire1.write(dataByte); // queue single byte
	
	if(Wire1.endTransmission(true) == 2)//transmit address and stop symbol
	{
		Serial.print("Failed to receive ACK at line ");
		Serial.println(__LINE__);
		releaseI2Cbus();
		return 0x0101;
	}
  
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms
	//write has completed
	releaseI2Cbus();
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif
			
//verify write worked
	if (ReadSEEPROM(eeAddressLong) != dataByte)
	{//read back of data failed so I can't trust SEEPROM to log error. Just print out error.
		Serial.print(F("SEEPROM write failed. Error is Read Back From SEEPROM Mismatch and is at address "));
		Serial.print(eeAddressLong);
		Serial.print(".   Line ");
		Serial.println(__LINE__);
		return 0x0102;
	}
	//readback was successful.
	
					#ifdef DiagPrint123
					Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    TS:  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
					
	return 0x0000;//success		
}//end of UnprotectedWriteSEEPROM()

int Fdr::ReadSEEPROM(long eeAddressLong)
{ 
/************************************************
Student's EEPROM. subroutine accepts a 4 byte address and returns a single integer. If no error, the upper byte will be 0 and the lower byte will be data. 

If there is an error, 0x01ec is returned, where ec is the error code, ec:

 ec		meaning
 00 	the address is out of range
 01		I2C bus not available
 02		failed to receive ACK
 03		no data read back
 04		more than one byte returned
 
 *************************************************/

					#ifdef timeReadEEPROM
						unsigned long timerStartULong = millis();//flag if read takes too long
					#endif
					
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 3000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	
	if(simulatorModeQ)
	{//simulated SEEPROM is 1600 bytes
		if(eeAddressLong > 1599)eeAddressLong = 1599;//map all out of range data into last byte
		return simSEEPROM[int(eeAddressLong)];
	}
	
	if (eeAddressLong > 131071L)
	{ //address is out of range
		return 0x0100;//we didn't sieze the bus yet so don't release it
	}
	
	I2Cbus1Mitigation(readSEEPROMuserByte);//siezes bus and identify yourself	
		
	if (eeAddressLong < 65536L) 
	{ //send block address based on eeAddressLong 
		Wire1.beginTransmission(SEEPROM_ADR_LOW_BLOCK); //we will be talking to EEPROM 1
	}else
	{
		Wire1.beginTransmission(SEEPROM_ADR_HIGH_BLOCK);//we will be talking to EEPROM 1
	}
	
  //then send data address within the block 
	Wire1.write((byte)(eeAddressLong >> 8)); //queue MSB
	Wire1.write((byte)(eeAddressLong)); //queue LSB
	if(Wire1.endTransmission(true) == 2)//transmit address and stop symbol
	{
		Serial.print("Failed to receive ACK at line ");
		Serial.println(__LINE__);
		
		releaseI2Cbus();
		return 0x0102;
	}
	
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms so wait 10.
 	
	if (eeAddressLong < 65536L)
	{
		Wire1.requestFrom(SEEPROM_ADR_LOW_BLOCK, 1);
	}else
	{
		Wire1.requestFrom(SEEPROM_ADR_HIGH_BLOCK, 1);
	}
	
	int rdataInt = 0;
	delay(waitForEEPROMresponseMsByte);//give time for device to respond
	
	switch(Wire1.available())
	{//evaluate data that was returned from SEEPROM
	
		case 0://no byte returned so give up
		
			FlushWire1Buffer();
			releaseI2Cbus();//since SEEPROM didn't respond, give back I2C bus 1
				
							#ifdef lockup
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
							
			//can't log fact that I was unable to read back from SEEPROM because EEPROM can't be reached so  print out warning. 
			Serial.print("SEEPROM did not respond to read by ReadSEEPROM() while running on core ");
			Serial.print(rp2040.cpuid());
			Serial.print(" Line ");
			Serial.print(__LINE__);
			Serial.print(" using address ");
			Serial.println(eeAddressLong);
			return 0x0103;//no data read back
	
		case 1: //one byte is available to read, return the integer 0x00DD where DD is the single byte of date.
			rdataInt = int(Wire1.read()); //the single byte of data becomes the LSB.
			break;
						
		default: //more than 1 byte read back
			rdataInt = int(Wire1.read());//read first byte
			FlushWire1Buffer();//toss excess bytes but also print them out as diag
			releaseI2Cbus();
			
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
						
			WriteControlBlock(ErrorCodeAddressByte, SEEPROMdataFailureByte);//log fact that I read back more than 1 byte from EEPROM. This may be futile since EEPROM can't be reached so also print out warning. 		
			Serial.print("SEEPROM returned more than the expected single byte while in ReadEEPROM()  on core ");
			Serial.print(rp2040.cpuid());
			Serial.print(". Line ");
			Serial.print(__LINE__);

			return 0x0104;//more than one byte returned.
	}//end of switch() that evaluates returned byte(s)	
	//read was successful if we got here
	
						#ifdef timeRreadEEPROM
						unsigned long runTimeUlong = millis() - timerStartULong;
						if (runTimeUlong > 100)
						{
							Serial.print("ReadEEPROM() took ");
							Serial.print(runTimeUlong);
							Serial.print( "ms.  Line ");
							Serial.println(__LINE__);
						}
						#endif
	
	FlushWire1Buffer();	
	releaseI2Cbus();
		
	return rdataInt;
}//end of ReadSEEPROM()


void Fdr::FlushWire1Buffer()
{//ensure nothing left in buffer. It is a defensive move and nothing should be in buffer so when not empty, I print it out for debugging
	unsigned long startOfFlushULong = millis();
	while(Wire1.available() > 0)
	{
		Wire1.read();
		if(millis() - startOfFlushULong > 500)
		{
			Serial.println("Flush Wire 1 buffer took more than 500 ms so gave up.");
			return;
		}
	}
}

void Fdr::FDRoneTimeRun()
//The first time it is called, it prints the Fdr.cpp version number to the terminal emulator. Then it tests the I2C bus and prints OK or the program stops executing. If the GPS is provisioned, it verifies the hardware is functioning but not that it has found any satellites. If the modem is provisioned, it verifies the hardware is functional and sets it.
//Subsequent calls to this function return the last known return code
{
			#ifdef OTMlapse
			long OneTimeRunTimerLong = millis();
			#endif
			
	if (hasFDRoneTimeRunNotRunYetQ)
	{

						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		pinMode(4,INPUT_PULLUP);//If this pin is tied to ground, it means this is team 2's board. If disconnected it pulls to VCC to indicate this is team 1's board. 	
		Serial.print(F("\tFdr.cpp\t\t"));
		Serial.print(Fdr_cpp_versionChar);
		Serial.print("\t");
		Serial.print(__DATE__);
		Serial.print(" at ");
		Serial.println(__TIME__);
		
		Serial.print(F("\tFdr.h\t\t"));
		Serial.print(Fdr_h_version[0]);Serial.print(".");Serial.print(Fdr_h_version[1]);Serial.print(".");Serial.print(Fdr_h_version[2]);
		Serial.print("\t");
		Serial.print(FdrHeaderDateString);
		Serial.print(" at ");
		Serial.print(FdrHeaderTimeString);
		Serial.println();
		Serial.println();
		Serial.println("See the Software Description document for the meaning of these dates and times.");		
			
		StartForTimeStampULong = millis();//used for diag prints		
		noTone(ExternalLED);//be sure no tone is active
		digitalWrite(ExternalLED, ExternalLED_OffByte);//be sure LED is off
		
		//ping I2C bus 1. If ping fails, it is a fatal fault so abort rest of test.
		Serial.println();
		Serial.println();
		Serial.println("     ******** Start Up Diagnostics ********");
		Serial.println();
					
		analogReadResolution(12);//4096 steps
		delay(2);//give PM time to reconfigure				
					
		
		if(readBattery(asAVoltageBool) < 6)
		{
			topOfBox();
			Serial.println("   *     No Battery detected.     *");
			bottomOfBox();
			Serial.println();
		}

						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

			
		if(pingI2C())
		{
			Serial.println(" I2C bus 1 is operational.");
			Serial.println();
		}else
		{
			topOfBox();
			Serial.println("   * I2C bus 1 has a fatal fault. *");
			bottomOfBox();
			Serial.println();
			Serial.println();
			topOfBox();	
			Serial.println("   *   Switching to a simulated   *");
			Serial.println("   *    Flight Data Controller    *");
			bottomOfBox();
			Serial.println();
			Serial.println();
			
						#ifdef lockup
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
			
			populateSimulatedControlBlock();//sets it to locked memory, next data block at address 16, no errors
			
						#ifdef lockup
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif			
			
			simulatorModeQ = true;
			WriteControlBlock(dataDumpStatusAddressByte,dataNotBeingDumpedByte);//force data dump status to not dumping data 
			hasFDRoneTimeRunNotRunYetQ = false;//we are done running one time run once so now we can mark it as done
			
						#ifdef lockup
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif			
						
			return;//since we are in simulation mode, nothing else to do
		}

						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif


		//verify control block memory is OK
		enablePort6HighSpeedRead = false;//reduce load on bus 1 during test
		ControlBlockEEPROMtest();//outputs status but continues
		
						#ifdef CBfatal
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
						#ifdef faultaddressPointer
						startOfNextMemoryBlockLong = 131040;//top of memory is 131072. Two data blocks from the top is 131040.
						WriteEEPROM_Pointer();
						#endif
		
		controlBlockSanityTest();//see if all values are within the expected range. If not, init all values
		
						#ifdef CBfatal
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		enablePort6HighSpeedRead = true;//restore function which only runs if we are in flight
		
		if (readBattery(outputAsVoltageBool) < 5)
		{ //when battery is off, we see less than 5V and sensors plus camera do not get power
				Serial.println();
				topOfBox();
				Serial.println(F("   *      Warning: No Battery     *"));
				Serial.println(F("   *   Some hardware will fail.   *"));
				bottomOfBox();				
				Serial.println();
		}

		
		//Next, if the GPS has been provisioned, test that it doesn't have a hardware fault.		
		if (*GNSS_Bool)
		{
			
			
							#ifdef ebugLoopAround
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
			//Serial1 connects to the GNSS or GPS
			Serial1.setFIFOSize(128);//increase serial buffer to prevent overflow
			Serial1.begin(GPSbaudRateULong); //define what rate the UART will expect from GPS
			delay(10);//give hardware time to configure			

						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif



							#ifdef GNSSprint			
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif			
			
			//turn off all NEMA sentences except GGA to reduce chatter from GPS
			//sendNMEA("PUBX,40,GGA,0,0,0,0,0"); //let GGA run
			sendNMEA("PUBX,40,GLL,0,0,0,0,0");
			sendNMEA("PUBX,40,GSA,0,0,0,0,0");
			sendNMEA("PUBX,40,GSV,0,0,0,0,0");
			sendNMEA("PUBX,40,RMC,0,0,0,0,0");
			sendNMEA("PUBX,40,VTG,0,0,0,0,0");
			delay(100);//for config messages to be processed


						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

			
			GlobalNavigationSatelliteSystem(true);//test GPS and wait for sentence.
			
						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			
			
			//sendNMEA commands may not work but GPS still may be usable.
			
			if((*NavigationDataByte)[0] == 201)
			{//we have a hardware problem because sentence not found in 1 second
				topOfBox();
				Serial.println("   *  GNSS/GPS has a fatal fault. *");
				bottomOfBox();
				Serial.println();
			
				WriteControlBlock(ErrorCodeAddressByte, equippedGPShardwareFaultByte);//record GPS hardware fault but only during oneTimeRun
			}else
			{
				//assume neo M10 and send command to change to AIR1.
				byte UBX_CFG_VALSET_AIR1byte[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x01,0x00,0x00,0x21,0x00,0x11,0x20,0x06,0xF2,0x4F};
				sendCFGmessage(UBX_CFG_VALSET_AIR1byte, sizeof(UBX_CFG_VALSET_AIR1byte));
				
				byte ACKresultByte = readACK(); //I should see ACK from setting AIR1 within 1 second				
				//returns 0 for ACK-ACK, 1 for ACK-NAK, and 2 for timed out after 1 second.		
				if(ACKresultByte == 0x00)
				{
					Serial.println(F(" GNSS OK and configured to provide location at up to 164,000 feet."));//GNSS responded correctly and we can get to 50,000 meters.
				}else
				{
					Serial.println(F("**GPS OK but unable to configure to provide location above about 30,000 feet.**"));
				}
				Serial.println();
			}								
		}else
		{
			Serial.println("****GNSS is not equipped.****");
			Serial.println();
		}
		

			
						#ifdef modemsetupTrace1
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
//test ADCs
	bool ADCatpBool = true;
	if(ADCread(0) == noACKreceivedInt)
	{
		Serial.println("***Analog to Digital Converter ADC03 failed to respond.***");
		Serial.println();
		ADCatpBool = false;
	}
	
	if(ADCread(4) == noACKreceivedInt)
	{
		Serial.println("***Analog to Digital Converter ADC47 failed to respond.***");
		Serial.println();
		ADCatpBool = false;
	}

	if(ADCatpBool)
	{
		Serial.println(" Both Analog to Digital Converters are OK.");
		Serial.println();
	}		
	
					#ifdef forceToMaxData
					//force next block pointer to 0x7090 which is 1 hour of data plus control block so 28,816 
					WriteControlBlock(3,0x90);
					WriteControlBlock(4,0x70);
					WriteControlBlock(5,0x0);
					WriteControlBlock(6,0x0);
					#endif
	
	WriteControlBlock(dataDumpStatusAddressByte,dataNotBeingDumpedByte);//force data dump status to not dumping data 

						#ifdef OTMlapse
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
	//init the bus 1 log files
	for(byte countByte = 0; countByte < logFileDepthByte; countByte++)
	{
		busUseTimeULong[countByte] = 3;
		busUseStateByte[countByte] = 4;
		logRunOnCoreByte[countByte] = 5;
	}
						#ifdef fillPROM
						sequentialFillMemory(true);//EEPROM
						sequentialFillMemory(false);//SEEPROM
						#endif
	
	hasFDRoneTimeRunNotRunYetQ = false;//we are done running one time run once so now we can mark it as done
	
					#ifdef OTMlapse
					Serial.print("FDR One Time Run took ");
					Serial.print(millis() - OneTimeRunTimerLong);
					Serial.println(" ms");
					#endif
					

  
	}//end of one time run, execute this tasks if this is the first time we ran through loop
	return;//after first time executed, just return
}//end of FDRoneTimeRun()


void Fdr::setupGPS()
{
/*******************************************	
Input: GPSbaudRateULong  defined at top of program
Output: UART set to this rate
*******************************************/
	
	if(*GNSS_Bool)return;//if not equipped with GNSS, don't set it up
	
				#ifdef newGPScodeDiag
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(" T"));
				Serial.println (millis() - StartForTimeStampULong);
				#endif

		Serial1.setTX(0); //for Pro Micro Pico RP2040 	GPIO0 is TX for Serial1
		Serial1.setRX(1);//for Pro Micro Pico RP2040  	GPIO1 is RX for Serial1
		Serial1.begin(GPSbaudRateULong); //define what rate the UART will expect from GPS
		delay(10);//time for hardware to configure

}//end of setupGPS()

bool Fdr::GlobalNavigationSatelliteSystem(bool waitQ)
{
	if(!*GNSS_Bool)return false;//don't run if GNSS not equipped and return a failure 
/*********************************************************
GlobalNavigationSatelliteSystem(bool waitQ)
Inputs: 
•	waitQ – if true, wait up to 1 second for GPGGA or GNGGA. If false, return right away if UART buffer is empty.
•	asynronous stream from GPS that periodically contains GGA sentence.

Outputs: 
*   function returns false if we didn't run or fault 
    detected
•	nav array containing data from GGA.
•	Errors stored in byte 0 of nav array plus various flags

GlobalNavigationSatelliteSystem()calls 
1.	FindNeededSentence(waitQ)
2.	InitializeNavigationDataByteArray()
3.	Parcing functions

1.	FindNeededSentence(waitQ)
This function calls 
a.	ReadHeader()
b.	ReadSentence()
2.	InitializeNavigationDataByteArray() - Nav array filled with 200 to signify init.

3.	Parcing functions - They read BufferText and place results into the nav array 


(*NavigationDataByte)[16] consists of:

BYTE	DESCRIPTION
0		hours (UTC)
1		minutes (UTC)
2		seconds (UTC)
3		degrees (latitude)
4		minutes (latitude)
5		seconds (latitude)
6		0 for north, 1 for south
7		degrees (longitude)
8		minutes (longitude)
9		seconds (longitude)
10		0 for east, 1 for west
11		altitude MSB (byte 3)   See also Alititude()
12		altitude (byte 2)
13		altitude (byte 1)
14		altitude LSB(byte 0)
15		units: 0 for meters, 1 for feet

The user must look at UTC in order to know if this is new data.

Array is returned with 200 for any field that is null. If $GPGGA or $GNGGA sentence not found, all fields set to 200. 

Hours = 201 means we timed out waiting for data from GNSS.
north/south and east/west fields set to 202 means unexpected value read.

Hours = 203 means correct header could not be found

Hours = 205 means the array was never changed from its initial value.

These values were chosen because they are out of range for all bytes and is also not a symptom of a hardware fault which can occur with the value 255.

**********************************************/

								#ifdef gpsRunTimeStudy
								StartForTimeStampULong = millis();//restart time for this study
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
								#ifdef gpsTiming
								Serial.println(__LINE__);
								Serial.print("time: ");
								Serial.println (millis());
								#endif


	BufferText = "";//clear Buffer variable to receive c from UART buffer
	
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
		
								#ifdef gpsRunTimeStudy
								Serial.print(F("line number "));			
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.println(millis() - StartForTimeStampULong);
								#endif

//UART buffer may contain GGA so don't erase it. I erased it when I was polling and GGA was turned off.

								#ifdef pollingTimingTest
								StartForTimeStampULong = millis();
								#endif
	
								
								#ifdef gpsRunTimeStudy
								Serial.print(F("line number "));			
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.println(millis() - StartForTimeStampULong);
								#endif
								
								
	delay(50);//time to fill buffer with sentence
	//echoSerialTimeout(10000);
	//#endif
	
				#ifdef frozen
				Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif	
	
	if(waitQ)
	{
		FindNeededSentence(true); //It returns true if first header found is GGA and false if it is not. When true, it processes the sentence and updates the output array. If false, it does not process the sentence and does not update the array. We must return to the gps function within the time it takes for the wrong sentence to be received, about 30 ms?
	}else 
	{//don't wait
		if(FindNeededSentence(false) == false)return false;//return false if buffer empty or header just found was not correct one. Old output array not updated.
	}
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
	
								#ifdef frozen
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
				
	if((*NavigationDataByte)[0] == 201)
		{//we have a hardware problem
			WriteControlBlock(ErrorCodeAddressByte, equippedGPShardwareFaultByte);//record GPS hardware fault from within GPS function
			return true;//this means GPS did run but we did have a problem
		}

								#ifdef GNSSprint9
								if (GNSS_TimedOutWaitingForDataBool)
								{								
									Serial.print(F("line number "));			
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.println(millis() - StartForTimeStampULong);
								}
								#endif
	//to get here, we must have correct sentence's characters

		
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
	
	InitializeNavigationDataByteArray();//initialize array to 205. Parcing code should be setting each element to either received data, 200, or 201.
	
/************************************************************
D A T A  S T R U C T U R E  O F  S E N T E N C E  
*************************************************************
Sample Characters	Quantity	Count value or variable
	,				delimiter		0
	15  			hours			1 & 2  or "," if UTC is null 
	01				minutes			3 & 4
	16				seconds			5 & 6
	.				decimal			7
	00				fractional seconds	8 thorugh A-1
	,				delimiter 		A
	33				degrees lat		A+1 & A+2			
	17				minutes lat		A+3 & A+4
	.				decimal			A+5
	56410			fractional minutes lat A+6 through B-1
	,				delimiter		B
	N				north/south		B+1
	,				delimiter 		C			
	112				degrees lon		C+1 thorugh C+3
	05 				minutes lon		C+4 & C+5
	.				decimal			C+6
	05327			fractional minutes lon C+7 through D-1		
	,				delimiter		D
	W				east/west		D+1
	,				delimiter		E
	2,09,1 			ignored fields	
	.				decimal		
	02				ignored fields	
	,				delimiter		F
	362 			altitude		F+1 through G-1 or ","
	.				decimal			G   can be "." or ","
	2				fractional (ignore)	H-1
	,				delimiter		H
	M				altitude units	H+1
,-28.1,M,,0000*62	ignored fields

all empty fields will cause 200 to be put in corresponding output array element.
*******************************************************/
								#ifdef gpsTiming
								Serial.println(__LINE__);
								Serial.print("time: ");
								Serial.println (millis());
								#endif

//parce BufferText string	
	IncompleteSentenceBool = false;
	ParceTime();
	if (IncompleteSentenceBool)return true;//GPS ran but had problem
	ParceLatitude();
	if (IncompleteSentenceBool)return true;	
	ParceLongitude();
	if (IncompleteSentenceBool)return true;
	SkipFourFields();

	if (IncompleteSentenceBool)return true;
	ParceAltitude();
	if (IncompleteSentenceBool)return true;
																							#ifdef gpsTiming
								Serial.println(__LINE__);
								Serial.print("time: ");
								Serial.println (millis());
								#endif
								
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
								#ifdef gpsTiming
								Serial.println(__LINE__);
								Serial.print("time: ");
								Serial.println (millis());
								#endif	
	return true;//we are done but must return value
	
}//at end of GlobalNavigationSatelliteSystem()

void Fdr::PrepareTransmitArrayForModem()
{
	byte count;
	//put GNSS data in transmit array
	for(count = 0; count < 16; count++)
	{
		(*IridiumTransmitDataByte)[count] = (*NavigationDataByte)[count];

	}
	bool asAcountBool = false;
	int BatteryReadingInt = int(readBattery(asAcountBool));

	(*IridiumTransmitDataByte)[16] = highByte(BatteryReadingInt);
	(*IridiumTransmitDataByte)[17] = lowByte(BatteryReadingInt);
	
	for (count = 18;count <45;count++)
	{
		(*IridiumTransmitDataByte)[count] = count;
	}
	

	
	for(count = 0;count<45;count++)	
	{
		Serial.print((*IridiumTransmitDataByte)[count]);
		printComma();
	}
	Serial.println();
	
	
	for(count = 0;count<45;count++)	//move all transmit data to Iridium's array
	{
		_IridiumTransmitDataByte[count] = (*IridiumTransmitDataByte)[count];
	}
} //end of PrepareTransmitArrayForModem() diag

void Fdr::ReadHeader()
/**************************************************************
If no data waiting in UART buffer, it just returns.
If data is waiting, it puts the first 5 characters into BufferText and returns.
If there is less than 5 characters in the buffer, it returns with what it found in BufferText.

Output

BufferText -  will be "" if no data was in the UART buffer; will be up to 5 characters if available.
*************************************************************/

{
	count = 0;
	BufferText = "";
	
					#ifdef DiagPrint3
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.println(millis() - StartForTimeStampULong);
					Serial.print(F(" BufferText = "));
					Serial.print( BufferText );
					Serial.println();
					#endif
	
	while(count < 5)
	{//record header which is 5 characters
		while(Serial1.available() < 1)//buffer now empty
		{
			
			if((millis()-GNSS_StartOfWaitingForDataMsULong) > GNSS_WaitingForDataTimeLimitMsULong)
			{
				GNSS_TimedOutWaitingForDataBool = true;//timed out waiting for characters
				(*NavigationDataByte)[0] = 201;//flags that this array should not be trusted because header not found
			
				
						#ifdef DiagPrint3
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						#endif
				
				return;//nothing in buffer so give up and don't wait for data
			}
		}

		
	//buffer has data
		character = Serial1.read();
		
		#ifdef rawGPSoutput
		//Serial.print(character);//prints ASCII
		//Serial.println("hex:");
		Serial.write(character);//prints hex
		#endif
		
		#ifdef DiagPrint3
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F("                 character = "));
						Serial.println( character );
						#endif
		
		BufferText.concat(character);
		count++;
	}
//we now have 5 characters from GPS stored in BufferText and it should be the header 
	
						#ifdef DiagPrint3
						Serial.print(F("line number "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F(" BufferText = "));
						Serial.println( BufferText );
						#endif
		
}//end of ReadHeader()

void Fdr::ReadSentence()
{
/************************************************************
Input: none
Output: BufferText filled with read in characters except the end of sentence delimeter is dropped
************************************************************/

	BufferText = "";//prepare to record all characters starting after header
	count = 0;
	
								#ifdef frozen 
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
	
	while(AtEndOfNeededSentenceBool == false)
	{//record all characters after header until carriage return

		while(Serial1.available() < 1)
		{
			if((millis()-GNSS_StartOfWaitingForDataMsULong) > GNSS_WaitingForDataTimeLimitMsULong)
			{
				GNSS_TimedOutWaitingForDataBool = true;//timed out waiting for characters
				(*NavigationDataByte)[0] = 201;//flags that this array should not be trusted because header not found
				
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
				
				return;
			}
		}
	//only get here when buffer has data
	character = Serial1.read();		
	
		#ifdef rawGPSoutput
		//Serial.print(character);//prints ASCII
		//Serial.println("hex:");
		Serial.write(character);//prints hex
		#endif
		
	
	  if (character == '\r')
	  {
		  AtEndOfNeededSentenceBool = true;//found carriage return so at end of the sentence. Setting this flag releases us from the while()
		  
		  
	  }else{
		  BufferText.concat(character);//if not at end of sentence, keep recording  
	  }

								#ifdef frozen1 //floods
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
		
	}//now have all characters in needed sentence in BufferText
}//end of ReadSentence()

void Fdr::ParceTime()
{
//we are walking through the string BufferText	
				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				#endif
	
	//parce hour UTC
	count = 0;//delimiter position before hours UTC which is the first character in the string BufferText
	
				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				#endif
	
	if(BufferText[1] == ',')//UTC field is null
	{
		
				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				#endif
				
		(*NavigationDataByte)[0] = 200;//null UTC so corresponding output array element for hours is set to 200
		(*NavigationDataByte)[1] = 200;//null UTC so corresponding output array element for minutes is set to 200
		(*NavigationDataByte)[2] = 200;//null UTC so corresponding output array element for minutes is set to 200
		IncompleteSentenceBool = true;
		return;
	}else
	{
		//UTC is not null so process it
		
				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				#endif
		
		(*NavigationDataByte)[0] = (((BufferText[1] - 0x30)*10) + (BufferText[2] - 0x30));//convert each ASCII character to a number and then use it to build the hour value


				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				Serial.print(F("hours = "));
				Serial.println((*NavigationDataByte)[0]);
				#endif
		
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[0] > 23)(*NavigationDataByte)[0] = 200;


				#ifdef debugParceTime
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				#endif
		
		//parce minutes UTC

		(*NavigationDataByte)[1] = ((BufferText[3] - 0x30)*10) + (BufferText[4] - 0x30);
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[1] > 59)(*NavigationDataByte)[1] = 200;
		
		//parce seconds UTC
		(*NavigationDataByte)[2] = ((BufferText[5] - 0x30)*10) + (BufferText[6] - 0x30);
		//do range check to protect against unexpected field values
		
		#ifdef debugParceTime
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" BufferText[5] = "));
		Serial.println( BufferText[5] );
		Serial.print(F(" BufferText[6] = "));
		Serial.println( BufferText[6] );
		Serial.print(F(" (*NavigationDataByte)[2] = "));
		Serial.println( (*NavigationDataByte)[2] );
		#endif
		
		if((*NavigationDataByte)[2] > 59)(*NavigationDataByte)[2] = 200;
	
		count = 7;//puts us at either "." if just before fractional seconds or "," for field delimiter
			
		#ifdef debugParceTime
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" seconds = "));
		Serial.println( (*NavigationDataByte)[2] );
		#endif

		//parce fractions of a second UTC. I don't use it but need to advance count to next delimiter
		
		if(BufferText[count] == ',')
		{//if there is no fractional part, we see "," right away so "A" is 7.
	
		#ifdef debugParceTime
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
	
		}else{
		//there is a fractional part
			
			while(BufferText[count] != ',')
			{
				if(count>30)break;//protect against garbage in bufferText hanging program
				
				#ifdef debugParceTime1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				Serial.print(F("count = "));
				Serial.println( count );
				Serial.print(F(" BufferText[count] = "));
				Serial.println( BufferText[count] );
				#endif
				
				count++;//search for delimiter and advance count along field. Exit just before ',' with count at A-1
			}
			//we normally exit while() at delimter which is count of "A" but are forced out when count is more than 30.
		}
	}
}//end of ParceTime()

void Fdr::ParceLatitude()
{	
	//parce degrees latitude
	CountPlusOne = count+1;	//at A+1 if field not null and B if it is null
	if(BufferText[CountPlusOne] == ',')
	{//latitude field is null so fill each byte with 200
		(*NavigationDataByte)[3] = 200;
		(*NavigationDataByte)[4] = 200;
		(*NavigationDataByte)[5] = 200;
		IncompleteSentenceBool = true;
		return;
	}else{
		//process degrees latitude field
		CountPlusOne = count+1;//most significant digit  "A+1"
		CountPlusTwo = count+2;//least significant digit  "A+2"
		(*NavigationDataByte)[3] = ((BufferText[CountPlusOne] - 0x30)*10) + (BufferText[CountPlusTwo] - 0x30);
		count = count + 3;//now at start of minutes latitude sub-field.
		
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[3] > 90)(*NavigationDataByte)[3] = 200;
		
		#ifdef DiagPrintGNSS
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" (*NavigationDataByte)[3] = "));
		Serial.println( (*NavigationDataByte)[3] );
		#endif
		
		//parce minutes latitude
		//count is most significant digit which is "A+3"
		CountPlusOne = count+1;//least significant digit "A+4"
		(*NavigationDataByte)[4] = ((BufferText[count] - 0x30)*10) + (BufferText[CountPlusOne] - 0x30);
		count = count+2;//now at "A+5" if this is the decimal before fractional minutes latitude or at "B" if no fractional part, the delimiter before North/South field.
		
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[4] > 59)(*NavigationDataByte)[4] = 200;
		

		float FractionalMinutesFloat = 0;		
		if(BufferText[count] == '.')//this means we have a fractional minute. If none, we just use FractionalMinutesFloat = 0
		{ 
			//parce fractional minutes latitude and convert to seconds
			count = count+1;//now at A+6, the most significant digit of fracctional minutes
			EndOfField = count;//prepare to search for end of fractional minutes sub-field which will be "B-1"
		
			//Serial.println(__LINE__);
			while(BufferText[EndOfField] != ',')
			{
				if(EndOfField > 200)
				{//200 is way beyond what could be in this field
					Serial.println(" GNSS parcing time error");
					break;//prevents program from hanging up
				}
			EndOfField++;
			}
			//search for delimiter and advance EndOfField along sub-field. Exit at "," with count at "B". The number we need to process starts at count and ends at EndOfField-1
		
			//Serial.println(__LINE__);
			
			float DividerFloat = 10;//scales each digit
		
			for(pointer = count; pointer < (EndOfField); pointer++)
			{//pointer goes from A+6 to B-1 
				FractionalMinutesFloat = FractionalMinutesFloat + float((BufferText[pointer]- 0x30)/DividerFloat);
				DividerFloat = DividerFloat*10;//prepare to scale next digit 
			}
			
			(*NavigationDataByte)[5] = byte(FractionalMinutesFloat*60);//convert from fractional minutes to integer seconds
			
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[5] > 59)(*NavigationDataByte)[5] = 200;			
		
			count = EndOfField;//update character count to delimiter just before north/south field. Now at B.
		}else{
			//there is no fractional minutes so seconds latitude is simply 0
			// character count is at delimiter just before north/south field. Now at B.
			(*NavigationDataByte)[5] =	0;
		}
	}
	count = count+1;//advance count to N/S field  "B+1"
	
	#ifdef DiagPrintGNSS
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" count = "));
	Serial.println( count );
	#endif

	
	//parce north/south  
	switch(BufferText[count]){
	case ',':
		(*NavigationDataByte)[6] = 200;//means null field
		count = count + 2;//character count set to delimiter just before degrees longitude  B+2
		break;
		
	case 'N':
		(*NavigationDataByte)[6] = 0;//0 means north
		count = count +1;//advance to delimiter just before degrees longitude	"C"
		break;
		
	case 'S':
	(*NavigationDataByte)[6] = 1;//1 means south
	count = count +1;//advance to delimiter just before degrees longitude	"C"
	break;
	
	default:
	(*NavigationDataByte)[6] = 202;//means unexpected value
	count = count +1;//advance to delimiter just before degrees longitude	"C"
	break;
	}
	
}//end of ParceLatitude()

void Fdr::ParceLongitude()
{
	//parce degrees longitude
	count = count+1;//advance to most significant digit  C+1
	CountPlusOne = count+1;//C+2
	CountPlusTwo = count+2;//C+3
	
	#ifdef DiagPrint1			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" count = "));
	Serial.println( count );
	Serial.print(F(" BufferText[count] = "));
	Serial.println( BufferText[count] );
	Serial.print(F(" BufferText[CountPlusOne] = "));
	Serial.println( BufferText[CountPlusOne] );
	Serial.print(F(" BufferText[CountPlusTwo] = "));
	Serial.println( BufferText[CountPlusTwo] );
	#endif
	
	if((BufferText[count] == ',')|| (BufferText[CountPlusOne]==','))
	{
		IncompleteSentenceBool = true;
		return;
	}
	(*NavigationDataByte)[7] = ((BufferText[count]- 0x30)*100) + ((BufferText[CountPlusOne]- 0x30)*10) + (BufferText[CountPlusTwo]- 0x30);
	
	//do range check to protect against unexpected field values
	if((*NavigationDataByte)[7] > 180)(*NavigationDataByte)[7] = 200;
	
	#ifdef DiagPrintGNSS
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" (*NavigationDataByte)[7] = "));
	Serial.println( (*NavigationDataByte)[7] );
	#endif
	
	count = count + 3;//advance to most significant digit of minutes longitude. C+4
	
	//parce minutes longitude
	CountPlusOne = count+1;//C+5
	(*NavigationDataByte)[8] = ((BufferText[count]- 0x30)*10) + (BufferText[CountPlusOne]- 0x30);
	
	//do range check to protect against unexpected field values
	if((*NavigationDataByte)[8] > 59)(*NavigationDataByte)[8] = 200;
	
	//parce seconds longitude
	count = count + 2;//advance to start of fractional minutes longitude which starts with "." if there is any fraction ("C+6") or with "," to signal the end of the field ("D").
	EndOfField = count;
	if(BufferText[EndOfField] == '.')
	{//we have a fractional part of minutes longitude
		while(BufferText[EndOfField] != ',')
			{
				if(EndOfField > 200)
				{
					Serial.println(" GNSS parcing Lon error");
					break;//prevents program from hanging up
				}
			EndOfField++;
			}
//search for delimiter and advance EndOfField along field. Exit at "," with EndOfField at "D". The number we need to process starts at count+1 ("C+7") and ends at EndOfField-1 ("D-1")
		
			//Serial.println(__LINE__);
			
		count = count+1;//at start of fractional part
		float DividerFloat = 10;//scales each number 
		FractionalMinutesFloat = 0;
		
		for(pointer = count; pointer < EndOfField; pointer++)
		{
			#ifdef DiagPrintGNSS
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" pointer = "));
			Serial.println( pointer );
			Serial.print(F(" BufferText[pointer] = "));
			Serial.println( BufferText[pointer] );			
			Serial.print(F(" DividerFloat = "));
			Serial.println( DividerFloat );
			#endif			
			
			FractionalMinutesFloat = FractionalMinutesFloat + float((BufferText[pointer]- 0x30)/DividerFloat);
			DividerFloat = DividerFloat*10;//prepare to scale next digit
			
			#ifdef DiagPrintGNSS
			Serial.print(F(" FractionalMinutesFloat = "));
			Serial.println( FractionalMinutesFloat );
			#endif
			
		}
		
		(*NavigationDataByte)[9] = byte(FractionalMinutesFloat*60);//convert from fractional minutes to integer seconds
		
		//do range check to protect against unexpected field values
		if((*NavigationDataByte)[9] > 59)(*NavigationDataByte)[9] = 200;
	
		count = EndOfField;//advance to delimiter just before East/West field "D"
	}else{
		//we do not have a fractional part of minutes longitude
		(*NavigationDataByte)[9] = 0;//there is no fractional part of minutes longitude so seconds equals 0. Note that this is not null (200)
		//count is C+6 but since fractional minutes sub-field is empty, we are at the delimiter so this count is also "D". 
	}
	
	//parce east/west
	CountPlusOne = count+1;//D+1
	CountPlusTwo = count+2;
	
	#ifdef DiagPrintGNSS
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" count = "));
	Serial.println( count );
	Serial.print(F(" BufferText[CountPlusOne] = "));
	Serial.println( BufferText[CountPlusOne] );
	#endif
	
	switch(BufferText[CountPlusOne]){
	case ',':
		//East/West field is empty
		IncompleteSentenceBool = true;
		return;
	
	case 'E':
		(*NavigationDataByte)[10] = 0;//0 means east
		count = count+2;//at delimeter after normal East/West   ("E")
		break;
		
	case 'W':
	(*NavigationDataByte)[10] = 1;//1 means west
	count = count+2;//at delimeter after normal East/West   ("E")
	break;
	
	default:
	(*NavigationDataByte)[10] = 202;//means unexpected value
	IncompleteSentenceBool = true;
	return;
	}
	
	if(BufferText[CountPlusTwo] != ',')
	{//unexpected character after E/W field. This check was added because I saw this anomoly once.
		IncompleteSentenceBool = true;
		return;
	}
	
}//end of ParceLongitude()

void Fdr::SkipFourFields()
{
	//next, advance past 4 delimiter. Count is now at delimiter just after east/west field which is "E". We will increment count until we get to the 4th delimiter which is just before the altitude field.
	FieldCountByte = 0;
	while(FieldCountByte < 4)
	{
		if(BufferText[count] == ',')FieldCountByte = FieldCountByte +1;//if we find a ',', increment the count until we have traversed 4 fields which we are ignoring
		count=count+1;//advance to next character
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" count = "));
		Serial.println( count );
		Serial.print(F(" BufferText[count] = "));
		Serial.println( BufferText[count] );
		#endif
		
	}
}//end of SkipFourFields()

void Fdr::ParceAltitude()
{
	StartOfAltitudeByte = count;//count is now at the first digit of the altitude field which is "F+1"
	//start including digits until we reach either "." or ","
	EndOfFieldByte = StartOfAltitudeByte;// "F+1"
	
	#ifdef DiagPrint1
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" StartOfAltitudeByte = "));
	Serial.println( StartOfAltitudeByte );
	Serial.print(F(" BufferText[StartOfAltitudeByte] = "));
	Serial.println( BufferText[StartOfAltitudeByte] );	
	#endif
	
	if(BufferText[StartOfAltitudeByte] == ',')//if true, we are at H and altitude field is null
	{
		IncompleteSentenceBool = true;
		return;
	}else{
		//altitude field is not null but may not have a fractinal part 
		while((BufferText[EndOfFieldByte] != '.') && (BufferText[EndOfFieldByte] != ','))
		{	
			EndOfFieldByte++;//search for decimal and comma and if neither seen, advance EndOfField along field. Exit at "G" if decimal found or "H" if "," found. The numbers we need to process starts at count (F+1) and ends at EndOfFieldByte-1.
		
			#ifdef DiagPrint1
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" EndOfFieldByte = "));
			Serial.println( EndOfFieldByte );
			Serial.print(F(" BufferText[EndOfFieldByte] = "));
			Serial.println( BufferText[EndOfFieldByte] );
			#endif
			
		}

		EndOfFieldByte = EndOfFieldByte -1;//must back up one count because the above logic takes us to the decimal or delimiter
		
			#ifdef DiagPrint1
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" EndOfFieldByte = "));
			Serial.println( EndOfFieldByte );
			Serial.print(F(" BufferText[EndOfFieldByte] = "));
			Serial.println( BufferText[EndOfFieldByte] );
			#endif
			
		AltitudeULong = 0;
		long MultiplierLong = 1;//scales each number
		
		for(pointer = EndOfFieldByte; pointer >= StartOfAltitudeByte; pointer--)
		{//move from right (G-1) to left (F+1)
		
			#ifdef DiagPrint1
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" pointer = "));
			Serial.println( pointer );
			Serial.print(F(" BufferText[pointer] = "));
			Serial.println( BufferText[pointer] );
			Serial.print(F(" MultiplierLong = "));
			Serial.println( MultiplierLong );
			#endif
			
			AltitudeULong = AltitudeULong + long((BufferText[pointer] - 0x30) * MultiplierLong);
			MultiplierLong = MultiplierLong * 10;//advance to next highest number
			
			#ifdef DiagPrint1
			Serial.print(F("line number "));			
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			//Serial.println(millis() - StartForTimeStampULong);
			Serial.print(F(" AltitudeULong = "));
			Serial.println( AltitudeULong );
			#endif
			
		}
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" AltitudeULong = "));
		Serial.println( AltitudeULong );
		#endif
		
		/*******************************
		transfer AltitudeULong into array as 4 bytes
		
		byte	description
		11		altitude MSB (byte 3)
		12		altitude (byte 2)
		13		altitude (byte 1)
		14		altitude LSB(byte 0)
		*********************************/
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
		
		(*NavigationDataByte)[11] = byte(AltitudeULong >> 24);//MSB: byte 3
		(*NavigationDataByte)[12] = byte(AltitudeULong >> 16);//byte 2
		(*NavigationDataByte)[13] = byte(AltitudeULong >> 8);//  byte 1
		(*NavigationDataByte)[14] = byte(AltitudeULong);//LSB: byte 0
	}
	
	count = EndOfFieldByte + 1;//advance to character after integer alititude  "G" or "H"
	
	//ignore fractional altitude and advance count to next field
	
	#ifdef DiagPrint1
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" count = "));
	Serial.println( count );
	#endif
	
	GNSS_StartOfWaitingForDataMsULong = millis();
	while(BufferText[count] != ',')
	{
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		Serial.print(F(" count = "));
		Serial.println( count );
		Serial.print(F(" BufferText[count] = "));
		Serial.println( BufferText[count] );
		#endif
		
		count = count +1;
		if((millis()-GNSS_StartOfWaitingForDataMsULong) > GNSS_WaitingForDataTimeLimitMsULong)
			{
				IncompleteSentenceBool = true;
				return;
			}
	}
	
	//now we are at the delimeter preceeding units field: "H"
	
	#ifdef DiagPrint1
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	#endif
		
	CountPlusOne = count+1;//H+1
	
	#ifdef DiagPrint1
	Serial.print(F("line number "));			
	Serial.print(__LINE__);
	Serial.print(F(".    Time Stamp  "));	
	Serial.println(millis() - StartForTimeStampULong);
	Serial.print(F(" CountPlusOne = "));
	Serial.println( CountPlusOne );
	Serial.print(F(" BufferText[CountPlusOne] = "));
	Serial.println( BufferText[CountPlusOne] );
	#endif
		
	switch(BufferText[CountPlusOne]){//units field
		case ',':	
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
		
		IncompleteSentenceBool = true;
		return;

		case 'M':
		(*NavigationDataByte)[15] = 0;//0 means metric
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
		
		break;
		
		case 'f':
		(*NavigationDataByte)[15] = 1;//1 means feet but we this is not in the standards for the GPGGA sentence
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
		
		break;
		
		default:
		(*NavigationDataByte)[15] = 202;//means unexpected value
		
		#ifdef DiagPrint1
		Serial.print(F("line number "));			
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));	
		Serial.println(millis() - StartForTimeStampULong);
		#endif
		
		IncompleteSentenceBool = true;
		return;
	}
	
				#ifdef DiagPrint1
				Serial.print(F("line number "));			
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.println(millis() - StartForTimeStampULong);
				Serial.print(F(" (*NavigationDataByte)[15] = "));
				Serial.println( (*NavigationDataByte)[15] );
				#endif
		
}//end of ParceAltitude()

bool Fdr::FindNeededSentence(bool waitQ)
/****************************************************
Sunny day (state of waitQ doesn’t matter)
Function returns true because GGA header was waiting in the UART buffer. It filled the nav array and returns true.

Rainy Days with waitQ = false
1.	UART buffer empty – return false immediately
2.	UART buffer first character is not $ - Keep looking for $ until 1 second. If no joy, return false.
3.	UART buffer first character is $ but no GGA, return false

Rainy Days with waitQ = true
1.	UART buffer empty – wait up to 1 second. If no joy, set 203 and return true (preserves other logic).
2.	UART buffer first character is not $ - Keep looking for $ until 1 second. If no joy, return false.
3.	UART buffer first character is $ but no GGA, keep looking
*************************************************/
{
	//unsigned long GNSS_StartOfCollectDataMsULong = 0;//set to millis() when used
	
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
								
	GNSS_StartOfWaitingForDataMsULong = millis();

								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
	
	AtEndOfNeededSentenceBool = false;//when true, we exit the following while loop
	
	while(AtEndOfNeededSentenceBool == false)
	{//keep reading characters from GNSS until $GPGGA or $GNGGA is detected or we time out
		 
		if((millis() - GNSS_StartOfWaitingForDataMsULong) > (GNSS_WaitingForDataTimeLimitMsULong)) //1000 ms
		{
			(*NavigationDataByte)[0] = 203;//flags that this array should not be trusted because correct header could not be found.
			
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif		
			
			return true;//I return true to preserve other logic
		}
		
		unsigned long emptyBufferWaitStartTimeULong = millis();
		while(Serial1.available() == 0)
		{//Buffer is empty
			if(!waitQ)
			{
				
				#ifdef frozen
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif				
				
				return false;//nothing in buffer so just return becuase we are not waiting
			}
			
			if((millis()-emptyBufferWaitStartTimeULong) > GNSS_WaitingForDataTimeLimitMsULong) //1000 ms 
			{ //we will wait up to 1000 ms for something to be put in buffer before we give up
				
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
				
				GNSS_TimedOutWaitingForDataBool = true;//timed out waiting for characters
				(*NavigationDataByte)[0] = 201;//flags that this array should not be trusted because $ plus header not found after waiting 4 seconds. Looks like GPS is not working.	
							
							#ifdef GNSSprint
							Serial.print(F("line number "));			
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.println(millis() - StartForTimeStampULong);
							#endif

				
				return true;
			}
		}//when UART buffer has data, we continue.
		
		//GNSS_StartOfCollectDataMsULong = millis();//initialize timer to protect against hanging due to no useful data from GPS		
						
								#ifdef gpsRunTimeStudy1
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif

								#ifdef frozen1//floods
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
								
		character = Serial1.read();//only get here when buffer has at least one character
		
								#ifdef rawGPSoutput
								//Serial.print(character);//prints ASCII
								//Serial.println("hex:");
								Serial.write(character);//prints hex
								#endif
		
		
								#ifdef DiagPrint31	
								Serial.print(F("line number "));			
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.println(millis() - StartForTimeStampULong);
								Serial.print(F("+"));
								Serial.println( character );
								#endif
		
		if (character == '$')
		{//at start of a sentence. Now see if it is the sentence we need

						
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
			
			//GNSS_TimedOutWaitingForDataBool = false;//because we found $ but it will change to true if we can't find right header 
			ReadHeader();//if it reads the wrong header or a partial header, it still returns. If it doesn't see anything, it returns "". Data is in BufferText
			
								
								#ifdef frozen1
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif	
								
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println();
								Serial.print("BufferText = ");
								Serial.print(BufferText);
								Serial.println();
								#endif
			

			if((BufferText.indexOf("GNGGA") == 0) || (BufferText.indexOf("GPGGA") == 0))
			{
			
			//if((BufferText.indexOf("GNGGA") == 0)||(BufferText.indexOf("GPGGA") == 0))
			//{//found needed header 
			
								#ifdef correctHeaderTest1
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
				ReadSentence();//it controls AtEndOfNeededSentenceBool flag so controls the while() above. BufferText is filled with read in characters except the end of sentence delimeter is dropped
							
																								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
								
								#ifdef gpsRunTimeStudy
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
			
				
			}else
			{//we didn't see wanted header after $ so return saying current sentence is not GGA if waitQ is false.	

							#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
		
				if(!waitQ)
				{

							#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

					return false;
				}
			
							#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
			
			}//we didn't see $ as read in character so continue
			
								#ifdef frozen
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
		
		}//bottom of while(AtEndOfNeededSentenceBool) so if we processed entire sentence and put the data in BufferText, we are done. Otherwise we will read next character looking for the $ followed by the header which signals the start of our data.
	
								#ifdef frozen1//floods
								Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								#endif
	}

							#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
							

	return true;//we successfully processed GGA
}//end of FindNeededSentence()

#ifdef GNSStest
void Fdr::PrintOutGNSS_Array()
{
					#ifdef DiagPrint5
					Serial.print(F("line number "));			
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.println(millis() - StartForTimeStampULong);
					#endif
		
		for(i = 0;i<16;i++)
		{
		Serial.print( (*NavigationDataByte)[i] );
		printComma();
		}
		Serial.println();
		
		Initialize(*NavigationDataByte)Array();//done with array so initialize it.
}
#endif

void Fdr::InitializeNavigationDataByteArray()
{
	for(i = 0;i < 16;i++)(*NavigationDataByte)[i] = 205;
}

void Fdr::Port6Peak()
{
	if(!enablePort6HighSpeedRead)return;//flag is set when we read the other ports; is temporarily cleared during eeprom testing to speed it up
	
	//log 32 sample periods when running on core 1 and then start over
if(rp2040.cpuid() == 1)
{	
	if(port6LogIndexByte > 32)port6LogIndexByte = 0;//reset log pointer when it gets larger than the size of port6PeriodLogUsULong[] array
	port6PeriodLogUsULong[port6LogIndexByte] = micros() - lastPort6SampleTimeUsULong;//calculate and store the last sample period
	lastPort6SampleTimeUsULong = micros();//reset time interval
	port6LogIndexByte++;
}	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							Serial.print("lastPort6SampleTimeUsULong = ");
							Serial.println(lastPort6SampleTimeUsULong);
							#endif
	int ADCcountInt = ADCread(6);
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	if(ADCcountInt < Port6PeakInt)return;//this will be most of the time so want it to run as fast as possible
	Port6PeakInt = ADCcountInt;//when we do read a higher value, save it as the positive peak. Spikes will be doublets so catching only positive spikes is valid.
	
							#ifdef loopTimerDetails
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	
						#ifdef DiagPrint7
						Serial.print(F("Port6Peak(): "));			
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.println(millis() - StartForTimeStampULong);
						Serial.print(F("Port6PeakInt = "));	
						Serial.println( Port6PeakInt );
						#endif
	
}//end of Port6Peak()

void Fdr::ZeroControlBlock()
{//diag tool to corrupt control block before it is checked
	for(byte ControlBlockAddressByte = 0; ControlBlockAddressByte < 16; ControlBlockAddressByte++)
	{
		WriteControlBlock(ControlBlockAddressByte, 0);
	}
}

void Fdr::PrintLine(bool preceedWithBlankLineQ)
{
	if(preceedWithBlankLineQ)Serial.println();
	Serial.println("===========================================================");
}


void Fdr::RawOutputGPSdata(long EEPROM_PointerLong, bool oddSecondQ)
{//this version outputs formatted data block
//during odd seconds, it reads the SEEPROM, formats all data in the block except altitude, and sends it to the log file plus stores the data in GNSSdataByte[16]
//during even seconds, it reads GNSSdataByte[16], formats the data, and sends it to the log file

	byte oneGPSbyte = 0;
	if(oddSecondQ)
	{//if we are in an odd second, save SEEPROM data in an array and print it out to the log file
		for(long i = 0; i < 16; i++)
		{
			oneGPSbyte = ReadSEEPROM(EEPROM_PointerLong + i);
			Serial.print(oneGPSbyte);
			printComma();
			GNSSdataByte[i] = oneGPSbyte;//save for use during even numbered seconds
		}
	}else
	{//this is an even second so just output from the array
		for(long i = 0; i < 16; i++)
		{
			Serial.print(GNSSdataByte[i]);
			printComma();
		}
		Serial.println();//end of record
	}
}//end of RawOutputGPSdata()


void Fdr::outputPortVoltages(long EEPROM_PointerLong,bool oddSecondsQ)
{
	//on odd seconds, read EEPROM, format data, and output to log file plus store output in portVoltagesFloat[8]. On even seconds, just ouput portVoltagesFloat[8]. This speeds up the printing
	
	printComma();
	/****************************************************
	If a portByte is out of range, the returned count will be 29,333 (outOfrangePortNumberInt) which translates to 5.5V. 

	If the ADC fails to respond, the return count will be 32,000  ( noACKreceivedInt) which translates to 6.0V.
	
	results also saved in portVoltagesFloat[8] for use by even numbered seconds. This saves time
**********************************************************/
					#ifdef printTimeStudy
					unsigned long timeStudyStartTimeULong = millis();//used to measure total lapse time
					#endif

	for (byte port = 0; port < 8;port++)
	{ // ports 0 - 7
		if(oddSecondsQ)
		{
			
					#ifdef printTimeStudy
					Serial.print("time stamp at ");
					Serial.print(__LINE__);
					Serial.print(" is ");
					Serial.println(millis() - timeStudyStartTimeULong);
				#endif			
			
			int ADCoutputInt = 256*portDataByte[(2*port)] + portDataByte[1+(2*port)];//MSB is first first and then LSB
			
			//int ADCoutputInt = (<< 8 int(portDataByte[(2*port))) & int(portDataByte[(1+(2*port))]);//MSB is first first and then LSB;// qaz when all else works, test out this format which should be faster.
			
			
					#ifdef printTimeStudy
					Serial.print("time stamp at ");
					Serial.print(__LINE__);
					Serial.print(" is ");
					Serial.println(millis() - timeStudyStartTimeULong);
					#endif			
			
			
			//int ADCoutputInt = << 8(ReadEEPROM(EEPROM_PointerLong + (2L*long(port)))) | ReadEEPROM(EEPROM_PointerLong + 1L + (2L*long(port)));//assemble the integer from the 2 bytes in memory  contains a bug
								
			VoltageFloat = ADCconvertToVoltage(ADCoutputInt);//convert count to a voltage
			
					#ifdef printTimeStudy
					Serial.print("time stamp at ");
					Serial.print(__LINE__);
					Serial.print(" is ");
					Serial.println(millis() - timeStudyStartTimeULong);
					#endif				
			
			if(VoltageFloat > 6.1)WriteControlBlock(ErrorCodeAddressByte, EEPROMdataFailureByte);
				//the EEPROM holding the voltage sample is defective or ADC problem so drop error 17.
			
			if (port == 7)
			{
				VoltageFloat = float(ADCoutputInt)* PMadcStepSizeFloat *batteryVoltageDividerCompensationFloat;//This is a special case because the battery is read with the Pro Micro's ADC plus has a voltage divider in front of it
			}
			
			Serial.print(VoltageFloat,3);//3 places past decimal point
			printComma();
			
					#ifdef printTimeStudy
					Serial.print("time stamp at ");
					Serial.print(__LINE__);
					Serial.print(" is ");
					Serial.println(millis() - timeStudyStartTimeULong);
					#endif				
			
			portVoltagesFloat[port] = VoltageFloat;//save output for even numbered seconds
			
		}else//end of output if odd second
		{//this is an even second so print port voltages from the array rather than calculating them again
			Serial.print(portVoltagesFloat[port],3);
			printComma();
		}	
	}//all port voltages have been output
}//end of outputPortVoltages()


void Fdr::printComma()
{
	Serial.print(F(","));
}

void Fdr::printOneSpace()
{
	Serial.print(" ");
}

void Fdr::printTwoSpaces()
{
	Serial.print("  ");
}

void Fdr::printColon()
{
	Serial.print(":");
}

void Fdr::ControlBlockEEPROMtest()
{//output is either Cntrl Bk: OK or Cntrl Bk: NG
//Before any relocation of the control block by startOfNextMemoryBlockLong, the control block starts at 0 and goes for 15 bytes. ReadControlBlock() takes care of any shifting by startOfNextMemoryBlockLong

							#ifdef CBfatal
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

			
	byte ControlBlockAddressByte = 0;
	byte dataByte = 0;
	bool memoryOK_Bool = true;
	bool testAbortedBool = false;
//read each byte of the control block, write 0 and verify, write 0xFF and read back, and then restore original data.
 	
	byte toggleByte = 0; 
	for(ControlBlockAddressByte = 0; ControlBlockAddressByte < 16; ControlBlockAddressByte++)
	{

							#ifdef CBfatal
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif

		
		//print a spinning line to tell user program is running
		if(toggleByte == 0)
		{
			Serial.print('\b');//backspace
			Serial.print("-");
		}
		
		if(toggleByte == 1)
		{
			Serial.print('\b');//backspace
			Serial.print('\\');// the first \ tells compiler that the next \ is taken as a characters and not a command
		}
		if(toggleByte == 2)
		{
			Serial.print('\b');//backspace
			Serial.print("|");
		}
		
		if(toggleByte == 3)
		{
			Serial.print('\b');//backspace
			Serial.print("/");
		}
				
		toggleByte++;
		if(toggleByte == 4)toggleByte = 0; 

						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

					#ifdef releaseTrap
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
		//save existing data from this address
		dataByte = ReadControlBlock(ControlBlockAddressByte);//ReadControlBlock() adds to this address to adjust for a relocation of the control block by startOfNextMemoryBlockLong
		
					#ifdef releaseTrap
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif		
		
						#ifdef controlBlockTest_dataByte
						Serial.print("control block data first read: ");
						Serial.println(dataByte);
						#endif
		
		//exercise address with writes of 0 and all ones. Errors are logged in the Control Block so should not be trusted yet.
		
						#ifdef CBfatal
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif		

						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif		
		
		WriteControlBlock(ControlBlockAddressByte, 0);
	// user assumes the control block in in addresses 0-15 but function will account for any shift by startOfNextMemoryBlockLong
	
						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
						
						#ifdef CBfatal
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif							
	
		if(ReadControlBlock(ControlBlockAddressByte) != 0)
		{
			memoryOK_Bool = false;
			goto outputResult;
		}
	//test all ones 
	
						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
		WriteControlBlock(ControlBlockAddressByte, 0xFF);
	// test that all ones was written
	
						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
		if(ReadControlBlock(ControlBlockAddressByte) != 0xFF)
		{
			memoryOK_Bool = false;
			goto outputResult;
		}
	//restore data, verify with read back
	
						#ifdef controlBlockTest_dataByte
						Serial.print("control block data restored with: ");
						Serial.println(dataByte);
						#endif
						
						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif						
						

		WriteControlBlock(ControlBlockAddressByte, dataByte);

						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		if(ReadControlBlock(ControlBlockAddressByte) != dataByte) //read back test
		{
			memoryOK_Bool = false;
		}
		//advance to next byte in control block

						#ifdef ControlBlockTiming
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		if(Serial.available() > 0)
		{//if any character entered from keyboard, abort control block memory test and empty receive buffer.
			ControlBlockAddressByte = 16;//abort test
			memoryOK_Bool = true;//dummy up success
			emptyUSBreceiveBufferQ(false);//dump buffer but don't wait 500 ms for more characters
			Serial.print('\b');//backspace over /, -, \, or |
			Serial.println(" aborted");
			testAbortedBool = true;

		}				
	}
	
							#ifdef CBfatal
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	
	outputResult:
		Serial.print('\b');//erase last spinner character
		Serial.print(" ");
		
		
	if(memoryOK_Bool == false)
		{
		topOfBox();
		Serial.println("   * Control Block: fatal fault.  *");
		bottomOfBox();
		Serial.println();
		}else{
			if(!testAbortedBool)Serial.println(F("The Control Block memory is OK"));
		}
		Serial.println();
		return;
}//end of ControlBlockEEPROMtest()

void Fdr::fullEEPROMtest()
{//output is either EEPROM: OK or EEPROM: NG
//this test will take about 3 hours to run.
	long EEPROMAddressLong = 0;
	byte dataByte = 0;
	bool memoryOK_Bool = true;
	byte statusCounterByte = 0;

	Serial.print(F("EEPROM:  "));//set up for results printing out
	
	for(EEPROMAddressLong = 0; EEPROMAddressLong < 131072; EEPROMAddressLong++)
	{
		if(statusCounterByte > 0xFE)Serial.print(F("."));//print dot every 256 bytes tested and let the count wrap
		statusCounterByte++;
		
		//save existing data from this address
		
		dataByte = ReadEEPROM(EEPROMAddressLong);
		Serial.print(F("dataByte: "));//diag
		Serial.print(dataByte);//diag
		Serial.print(F(" at loc "));//diag
		Serial.println(EEPROMAddressLong);//diag
		//exercise address with writes of 0 and all ones. Errors are logged in the Control Block so should not be trusted yet.
		UnprotectedWriteEEPROM(EEPROMAddressLong,0);
		
	// test that zero was written
		if(ReadEEPROM(EEPROMAddressLong) != 0){
			memoryOK_Bool = false;
			Serial.println(F("readback of 0x00 failed"));
		}else{
		memoryOK_Bool = true;
			Serial.println(F("readback of 0x00 passed"));
		}
	//test all ones 
		UnprotectedWriteEEPROM(EEPROMAddressLong,0xFF);
	// test that all ones was written
		if(ReadEEPROM(EEPROMAddressLong) != 0xFF){
			memoryOK_Bool = false;
			Serial.println(F("readback of 0xFF failed"));
		}else{
			memoryOK_Bool = true;
			Serial.println(F("readback of 0xFF passed"));
		}
	//restore data and advance to next byte in control block
		UnprotectedWriteEEPROM(EEPROMAddressLong, dataByte);
		if(ReadEEPROM(EEPROMAddressLong) != dataByte){
			memoryOK_Bool = false;
			Serial.println(F("readback of original data failed"));
		}else{
		memoryOK_Bool = true;
			Serial.println(F("readback of original data passed"));
		}
		if(memoryOK_Bool == false)
			{
				Serial.print(F("NG at "));
				Serial.println(EEPROMAddressLong);
				memoryOK_Bool = true;//reset flag
			}else
			{
				//Serial.println(F("OK"));  diag
			}
		if(Serial.available() > 0)
		{	
			EEPROMAddressLong = 131072; //abort test with a pass
			emptyUSBreceiveBufferQ(false);//remove keystroke but don't wait for more to come in
			Serial.println("Test aborted.");
		}
	}
}//end of fullEEPROMtest()

void Fdr::partialEEPROMtest()
{
/****************************************************
Output is either EEPROM: OK or failure data

This test check the first numberOfBytesTested of usable EEPROM out of the device's 131,071 bytes.

Press any key to abort the test
******************************************************/

	long EEPROMAddressLong = 0;
	byte dataByte = 0;
	byte testByte = 0;
	byte numberOfBytesTested = 64;
	byte toggleByte = 1;
	Serial.println();
	Serial.print("Test the first ");
	Serial.print(numberOfBytesTested);
	Serial.print(" bytes of EEPROM data storage: ");//set up for results printing out
	
	emptyUSBreceiveBufferQ(false);//defensive
	enablePort6HighSpeedRead = false;//reduce load on bus 1 by turning off Port6Peak() 
	for(EEPROMAddressLong = StartOfUsableEEPROM_Long + 16; EEPROMAddressLong < StartOfUsableEEPROM_Long + 16 + numberOfBytesTested; EEPROMAddressLong++)
	{
		//spin icon to entertain user while we test the EEPROM 
		if(toggleByte == 1)
		{
			Serial.print('\b');//backspace
			Serial.print("-");
		}
		
		if(toggleByte == 2)
		{
			Serial.print('\b');
			Serial.print('\\');// first \ says to use the following character so I print the backslash
		}
		
		if(toggleByte == 3)
		{
			Serial.print('\b');//backspace 
			Serial.print("|");
		}
		
		if(toggleByte == 4)
		{
			Serial.print('\b');//backspace
			Serial.print("/");
		}
				
		toggleByte++;
		if(toggleByte > 4 )toggleByte = 0;

	//test a data memory location	
		dataByte = ReadEEPROM(EEPROMAddressLong);		
		
	//exercise address with writes of 0 and all ones. Errors are printed since we can't trust EEPROM in this case
		UnprotectedWriteEEPROM(EEPROMAddressLong,0);		
		// test that zero was written
		testByte = ReadEEPROM(EEPROMAddressLong);
		if(testByte != 0)
		{
			Serial.print("At address "); 
			Serial.print(EEPROMAddressLong);
			Serial.print(" - readback of 0x00 failed. Got "); 
			Serial.println(testByte);
			return;
		}
		
		//write 0xFF
		UnprotectedWriteEEPROM(EEPROMAddressLong,0xFF);
		// test that all ones was written
		testByte = ReadEEPROM(EEPROMAddressLong);
		if(testByte != 0xFF)
		{
			Serial.print("At address "); 
			Serial.print(EEPROMAddressLong);
			Serial.print(" - readback of 0xFF failed. Got ");
			Serial.println(testByte); 
			return;
		}
		
		//restore data
		UnprotectedWriteEEPROM(EEPROMAddressLong, dataByte);
		testByte = ReadEEPROM(EEPROMAddressLong);
		if( testByte != dataByte)
		{
			Serial.print("At address "); 
			Serial.print(EEPROMAddressLong);
			Serial.println(F(" - readback of original data failed. Got ")); 
			Serial.println(testByte); 
			Serial.print(F(" instead of "));
			Serial.println(dataByte); 
			return;
		}

	//with data restored, it is safe to check if user wants to abort test
		if(Serial.available() > 0)//abort test if any key was pressed
		{
			Serial.print('\b');//backspace over /, -, \, or |
			Serial.println("Test was aborted.");
			emptyUSBreceiveBufferQ(false);//get rid of and other keys that were pressed
			return;
		}

	}// advance to next byte in control block	
	//if all locations tested and we didn't return, ATP
	Serial.print('\b');//backspace over /, -, \, or |
	Serial.println(F(" Result is OK."));
	enablePort6HighSpeedRead = true;//turn fdr.Port6Peak() back on
}//end of partialEEPROMtest()

void Fdr::formattedOutputGPSdata(bool oddSecondQ)
{
/*****************************************************************
Input is GNSSdataByte[]

readSEEPROMdataBlock(EEPROM_PointerLong) fills GNSSdataByte[] from SEEPROM and we use it for odd and even seconds to speed up print.

If GNSSdataByte[0] is > 24, we have some error so print warning and return.

Dump fully formatted gps data. For example: time is AZ time and not UTC, degrees,minutes,seconds,N, is printed rather than D:M:S N

output is printed to log file and GNSSdataByte[ ] is  used during eveb seconds to speed up printing of even seconds output
also save formatted altitude in GNSSaltitudeLong.
***********************************************************************/

						#ifdef dgw
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif


	byte hoursByte = GNSSdataByte[0];
	
	if(hoursByte > 24)//would be 200, 201, or 205
	{
		Serial.println();
		return;
	}
//output local time	by subtracting 7 but this is modulo 24.
	if(hoursByte <= 7)//0 to 7 for the hours. For example, if hoursByte = 0, hoursByte - 7 gives me Az time but this is -7 which means the day before so add 24 to give me 17. if hoursByte = 7, hoursByte - 7 gives me Az time but this is 0 which means the day before so add 24 to give me 24.         
	{
		hoursByte = hoursByte + 24 - 7;
	}else
	{//then hoursByte is greater than 7 so just subtract 7
		hoursByte = hoursByte - 7;
	}
//we now have Az military time	
	if(hoursByte <= 12)//0 to 12 for the hours, this is AM or noon
	{
		Serial.print(hoursByte);//prints 0 to 12 and this is AM or noon
	}
	if((hoursByte > 12) && (hoursByte <= 24))//13 to 24 for the hours
	{
		Serial.print(hoursByte - 12);//prints 1 to 12 and this is PM or midnight
	}

	
	
	printComma();
	byte minutesBytes = GNSSdataByte[1];
	Serial.print(minutesBytes);//minutes
	printComma();		
	Serial.print(GNSSdataByte[2]);//seconds
	printComma();
	
//output Lat
	Serial.print(GNSSdataByte[3]);//degrees
	printComma();
	Serial.print(GNSSdataByte[4]);//minutes
	printComma();		
	Serial.print(GNSSdataByte[5]);//seconds
	printComma();
	byte northSouthByte = GNSSdataByte[6];
	if(northSouthByte == 0)
	{
		Serial.print("N");			
	}
	if(northSouthByte == 1)
	{
		Serial.print("S");
	}
	if(northSouthByte > 1)
	{
		Serial.print(northSouthByte);//print error
		Serial.print(northSouthByte);//This field will show either N, S, or <number>?
	}
	printComma();
	
//output Lon
	Serial.print(GNSSdataByte[7]);//degrees
	printComma();
	Serial.print(GNSSdataByte[8]);//minutes
	printComma();		
	Serial.print(GNSSdataByte[9]);//seconds
	printComma();
	byte eastWestByte = GNSSdataByte[10];
	if(eastWestByte == 0)
	{
		Serial.print("E");			
	}
	if(eastWestByte == 1)
	{
		Serial.print("W");
	}
	if(eastWestByte > 1)
	{
		Serial.print(eastWestByte);
	}
	printComma();
	
//output 4 byte altitude if units are in meters

	if(GNSSdataByte[15] > 0)
	{
		Serial.println(GNSSdataByte[15]);
	}else
	{//units are in meters so safe to convert to feet	
		byte b3 = GNSSdataByte[11];
		byte b2 = GNSSdataByte[12];
		byte b1 = GNSSdataByte[13];
		byte b0 = GNSSdataByte[14];
		float altitudeFloat = float(changeBytesToUnsignedLong(b3,b2,b1,b0))*metersToFeetFloat;//assemble number and covert from meters to feet			
		Serial.println(altitudeFloat,0);
	}
}//end of formattedOutputGPSdata()



void Fdr::echoSerialTimeout(unsigned long timeout)
{
  // Echo serial data from Serial1 to Serial until a timeout occurs
  
  unsigned long startTime = millis();
	Serial.println("Start of monitoring time.");
	while (millis() < (startTime + timeout))
	{
		while (Serial1.available())
		{
			Serial.write(Serial1.read());						
						
						#ifdef pollingTimingTest
						Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif						
		}

	}
	Serial.println("Done monitoring GNSS output [4891]");
}

void Fdr::echoSerialTerminatorTimeout(byte terminator, unsigned long timeout)
{
  // Echo serial data from Serial1 to Serial until terminator
  // is received or a timeout occurs
  
  unsigned long startTime = millis();
  byte c = 0;
  while ((c != terminator) && (millis() < (startTime + timeout)))
  {
    if (Serial1.available())
    {
      c = Serial1.read();
      Serial.write(c);
    }
  }
}

void Fdr::nmeaWriteByteUpdateChecksum(byte *csum, byte b)
{
//he defined *csum which I believe is a pointer to csum. Note that *csum is updated but not returned so by using a pointer, the value, where ever it is, is correctly changed.
//This is a simple XOR checksum
  // Send a single NMEA byte and update the checksum
  
  Serial1.write(b);//write sends a single hex value
  *csum = *csum ^ b;//XOR *csum with new byte
}

void Fdr::nmeaWriteBytesUpdateChecksum(byte *csum, const char *b)
{
//this time, he is also passing a constant character *b. It appears that b is a string
  // Loop through NMEA text, send each byte and update checksum
  
  for (byte i = 0; i < byte(strlen(b)); i++)//sequence through the string from the first to the last element.
  {
    nmeaWriteByteUpdateChecksum(csum, b[i]);
	//here is where I call the function defined on line 37. He passes csum but that field is defined as a pointer to *csum. He then passes b[i] where i is being advanced by the for loop on each pass so only a single character is passed here and it is a single byte. 
  }
}

void Fdr::nmeaWriteChecksumCRLF(byte csum)
{
//note that this time, he is passing the csum which is a single byte
  // Send the NMEA checksum as two ASCII characters, and then tack on CR and LF
  
  byte nibble = csum >> 4; 
  // csum is 8 bits. He is extracting the top 4 bits and shifting them down to be the 4 least significant bits. 
  //Get 4 most significant bits
  nibble += 0x30; // Convert to ASCII 0-9
  if (nibble >= 0x3A) //if the 4 bits are more than 9, it will be a letter so we need more processing
    nibble += 0x41 - 0x3A; // Convert to ASCII A-F
  Serial1.write(nibble);//most significant 4 bits sent to GPS as an ASCII character
  
  nibble = csum & 0xF; // Get 4 least significant bits and mask off the upper nibble.
  nibble += 0x30; // Convert to ASCII 0-9
  if (nibble >= 0x3A)
    nibble += 0x41 - 0x3A; // Convert to ASCII A-F
  Serial1.write(nibble);//least significant 4 bits sent to GPS as an ASCII character

  Serial1.write(0x0D); // CR
  Serial1.write(0x0A); // LF
}

void Fdr::sendNMEA(const char *message)
{
  // Send a NMEA message. Add the $,*,checksum,CR,LF

  byte csum = 0;

  Serial1.write('$'); // Dollar - not in checksum  
  nmeaWriteBytesUpdateChecksum(&csum, message); // Send the message
  Serial1.write('*'); // Asterix - not in checksum
  nmeaWriteChecksumCRLF(csum); // Write 2-byte ASCII checksum and CR, LF  
}
//#endif

unsigned long Fdr::changeBytesToUnsignedLong(byte b3, byte b2, byte b1, byte b0){
	//take four bytes and assemble them into a Long which I return
	unsigned long resultULong = ((unsigned long)b3 << 24) | ((unsigned long)b2 << 16) | ((unsigned long)b1 << 8) | (unsigned long)b0;
	

	
	return resultULong;
}

//#ifdef 	GNSSchangeToAir1SecondTry

void Fdr::sendCFGmessage(byte *CFGmessage, byte length)
{
	for(byte i = 0;i < length;i++)
	{
		Serial1.write(CFGmessage[i]);
		//Serial.print("sending byte ");//diag
		//Serial.print(i);
		//Serial.print(" with value of ");
		//Serial.print(CFGmessage[i],HEX);
		//Serial.println();
	}
}

//#endif

byte Fdr::readACK()
{
	//returns 0 for ACK-ACK, 1 for ACK-NAK, and 2 for timed out after 1 second.
	unsigned long startTimeULong = millis();
	//Serial.println(" monitor ACK response for 1 second:");//diag
	while(millis() - startTimeULong < 1000)
	{
		if(Serial1.available() > 0)
		{
			if(checkForACKheader())//returns true if it finds the ACK header  at 100,000 feet
			{
				if(checkForACKresult()) //ACK-ACK returns true and ACK-NAK false
				{			
					return 0;//we successfully change to AIR1
				}else
				{
					return 1;//we did not change to AIR1 but GNSS did respond
				}
			}
		}
	}		
	return 2;//we waited 1 second for ACK and gave up.
}

void Fdr::emptySerial1Buffer()
{
	while(Serial1.available() > 0)//ensure buffer is empty
		{
			Serial1.read();
		}
}

void Fdr::printUTCTime()
{
	Serial.print((*NavigationDataByte)[0]);//hours
	printColon();
	if((*NavigationDataByte)[1] < 10){
		Serial.print("0");//leading zero in minutes
	}
	Serial.print((*NavigationDataByte)[1]);//minutes
	printColon();
	if((*NavigationDataByte)[2] < 10){
		Serial.print("0");//leading zero in minutes
	}	
	Serial.print((*NavigationDataByte)[2]);//seconds
	printTwoSpaces();
}

bool Fdr::checkForACKheader()
{  //buffer has data. Function returns true if it finds ACK
	byte readByte;
	unsigned int startTimeUInt = millis();
	while((millis() - startTimeUInt) < 1000)
	{
		if(Serial1.available() > 0)
		{
			readByte = Serial1.read();
			//Serial.print("ACK response data is ");//diag
			//Serial.println(readByte,HEX);//diag
			
			if(readByte == 0xb5)//header
			{
				delay(2);
				readByte = Serial1.read();//read second byte
				//Serial.println(readByte,HEX);
				if(readByte == 0x62)//header
				{					
					delay(2);
					readByte = Serial1.read();//read third byte
					//Serial.println(readByte,HEX);					
					if(readByte == 0x05)
					{												
						return true;//ACK ID
					}
				}
			}
		}		
	}
	return false;//ACK header not found within 1 second
}

bool Fdr::checkForACKresult()
{ //buffer has data. Function returns true if next byte is 1 which means ACK-ACK and returns false if next byte is 0 which means ACK-NAK. Any other value returns false.
	if(Serial1.read() == 0x01)//ACK-ACK
		{
			//Serial.println("read ACK-ACK");//diag
			
			return true;
		}
		return false;
}

void Fdr::LEDflashInfo()
{

			#ifdef readMys
			Serial.print("core ");
			Serial.print(rp2040.cpuid());
			Serial.print(" ");
			Serial.print(__FUNCTION__);
			Serial.print(F("(): "));		
			Serial.print(__LINE__);
			Serial.print(F(".    Time Stamp  "));	
			Serial.print (millis() - StartForTimeStampULong);
			Serial.print(F(" ms"));
			byte testByte = ReadControlBlock(ErrorCodeAddressByte);
			Serial.print(".  Error Code = ");
			Serial.println(testByte);
			#endif
			
	(*LEDflashInfoByte)[0] = ReadControlBlock(ErrorCodeAddressByte);
	(*LEDflashInfoByte)[1] = ReadControlBlock(ProgramStateAddressByte);
	//if externalLEDcontrolBool is true, we set a flag to stop flashLED() from controlling the external LED
	
	if(externalLEDcontrolBool)
	{
		byte diagLEDmodeIsEnabledByte = 1;
		(*LEDflashInfoByte)[2] = diagLEDmodeIsEnabledByte;
	}else
	{
		byte diagLEDmodeIsNotEnabledByte = 0;
		(*LEDflashInfoByte)[2] = diagLEDmodeIsNotEnabledByte;
	}	
	
				#ifdef readMys
				Serial.print("core ");
				Serial.print(rp2040.cpuid());
				Serial.print(" ");
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif
				
	
}

int Fdr::ADCread(byte portByte)
{
/************************************************
Input:
If there is no battery, both external ADCs will be dead but the PM's ADC works. Therefore, I can read the battery voltage. If the battery is < 5V, I will force all external ADC readings to be 5.25 volts.

ports 0-3 are from ADC03
ports 4-7 are from ADC47
simulatorModeQ

Output:
integer in 2's complement so we get 15 bits of data

The ADC is a TI ADS1115 device. It has a resolution of 16 bits with 2's complement output. One count equals 6.144V/(2^15 counts) = 187.5 microvolts. With no input, the output is +/- 3 counts or +/- 0.00056 volts. 

The maximum input voltage permitted is 5 volts. 

If there is no battery, the returned count will be 28,000 which translates to 5.25 volts.

If a portByte is out of range, the returned count will be 29,333 (outOfrangePortNumberInt) which translates to 5.5V. 

If the ADC fails to respond, the return count will be 32,000  ( noACKreceivedInt) which translates to 6.0V.

If we are in simulation mode, it returns a count of 13333 which correspnds to 2.5V.

For ADC03, the ADDR pin is tied to SDA so the 7 bit device's address is 0x4A. For ADC47, the ADDR pin is tied to 5V so the 7 bit device's address is 0x49.
 
*********************************************
Low Level Details

address ponter register value to access the config register is
0b000000 01 = 0x1
address ponter register value to access the conversion register is 0b000000 00 = 0x0

Config register is 2 bytes: 

0b 0 xxx - 000 0 - 100 0 - 0 0 11 
0x	 xxx    0       8           3 


where xxx is 100 for port 0, 101 for port 1, 110 for port 2, and 111 for port 3 all single ended

conversion register is 2 bytes big endium (MSB LSB)

***********************************************/
						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			

	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 5000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	
						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			
	if(simulatorModeQ)
	{
		if(portByte < 7)
		{
			return 13333;//translates to 2.5V
		}
		if(portByte == 7)
		{
			return 0;//this is the battery port
		}
		if(portByte > 7)
		{
			return outOfrangePortNumberInt;//our of range so just return a count equal to 5.5V. Don't need to release the I2C bus because we never took it
		}
	}
	
	if(readBattery(true) < 5)
	{//ADCs don't have power so force the port voltages to 5.25V by returning a count of 28000
		int ADCsDontHavePowerIndicationInt = 28000;
		return ADCsDontHavePowerIndicationInt;
	}
	
	byte deviceAddressByte = 0x48;  //default is ADC03
	
	if(portByte > 7)return outOfrangePortNumberInt;//our of range so just return a count equal to 5.5V. Don't need to release the I2C bus because we never took it

	byte readPort[4];//these are the MSB (data byte 1) of the configuration register data 
	readPort[0] = {0x40};//bits 15:12 are 0100;11:8 are 0000
	readPort[1] = {0x50};//bits 15:12 are 0101
	readPort[2] = {0x60};//bits 15:12 are 0110
	readPort[3] = {0x70};//bits 15:12 are 0111	
	
	const byte ADC03AddressByte = 0x4A;
	const byte ADC47AddressByte = 0x4B;
	
	if(portByte < 4) 
	{
		deviceAddressByte = ADC03AddressByte;
	}else
	{ 
		deviceAddressByte = ADC47AddressByte;
		
					//Serial.println(__LINE__);//diag
		
		portByte = portByte - 4;//maps ADC47 port numbers down to between 0 and 3
	}

						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
			
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
	I2Cbus1Mitigation(ADCreadUserByte);//siezes bus and identify yourself
	
						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
	
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif

			#ifdef bus1Status1
			Serial.println("bus occupied 5");
			#endif
		
		
//tell ADC which analog port to read
	//TransmissionInProgress was set false by begin()	
	Wire1.beginTransmission(deviceAddressByte);//7 bit slave address
	Wire1.write(0x01);//address pointer register set for config register is sent to transmit buffer
	//TransmissionInProgress flag is false
	
							#ifdef loopTimerDetails1
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif	
	
	Wire1.write(readPort[portByte]); //data bytes 1. readPort[] is an array of addresses within the ADC for the four ports.
	
							#ifdef loopTimerDetails1
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif		
	
	Wire1.write(0x83);//data byte 2
	//bits 7:5 is 100, 4 is 0, 3 is 0, 2 is 0, 1:0 is 11 so 
	//so we have 0b 10000011 = 0x83 
	//TransmissionInProgress is false
	
							#ifdef loopTimerDetails1
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							#endif
	
	if(Wire1.endTransmission(true) != 0)//send all in buffer and put stop at end. A returned value other than 0 is a failure
	{
		Serial.println("Failure was received from ADC.");//diag
		
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif			

						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif


		FlushWire1Buffer();
		releaseI2Cbus();
		
						#ifdef ADCtimeCheck
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
		
		
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif
				
		return noACKreceivedInt;//value translates to 6V
	}
	//the above writes 1 byte to the address pointer register and  2 bytes to the configuration register
	
	delay(20);//give time for ADC to work. 10 ms causes reading to appear at more than one port. 15 ms causes good reading some of the time. 20 ms works every time. 
//next, ask for results from ADC reading from previously specified analog port	

	Wire1.beginTransmission(deviceAddressByte);   
	//TransmissionInProgress flag is false.
	Wire1.write(0x00);//address pointer register set for conversion register is written to buffer. P1:P0 are 00 which means we want to read the Conversion register
	//TransmissionInProgress flag is false
	if(Wire1.endTransmission(true) != 0)//send all in buffer and put stop at end. A returned value of other than 0 is a failure where I had changed it to false so change it back.
	{
		
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
		FlushWire1Buffer();
		releaseI2Cbus();

						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif

					#ifdef bus1Status
					Serial.println("bus released 5.1");
					#endif		
		
		return noACKreceivedInt;//value translates to 6V
		
	}//sends all bytes in buffer and puts stop at end. TransmissionInProgress flag is false.
	//first half of read sequence is done
	delay(waitForEEPROMresponseMsByte);//give time for ADC return data from conversion reg (not sure I need to wait)
	//next, ask for results from ADC reading from previously specified analog port	
	//Next read back 2 bytes
	//TransmissionInProgress flag is false
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	
	FlushWire1Buffer();//defensive
	Wire1.requestFrom(deviceAddressByte,2,true);//it puts out STOP when bytes received
	//TransmissionInProgress flag is now true

						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif	
	delay(waitForEEPROMresponseMsByte);//give time for device to respond
	// the expected 2 bytes arrived so read them
		byte MSB = Wire1.read();
		byte LSB = Wire1.read();		
		int ADCoutputInt = int(MSB << 8 | LSB);//first read is MSB and second read is LSB
		
	FlushWire1Buffer();// I have found that ADC sends more than 2 bytes so flush the rest
	releaseI2Cbus();
	
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif

	return ADCoutputInt;
}//end of ADCread()

float Fdr::ADCconvertToVoltage(int ADCoutputInt)
{	
	float readVoltageFloat = (187.5E-6 * float(ADCoutputInt));
					//Serial.print("resulting voltage is ");
					//Serial.print(readVoltageFloat,4);//4 places to rt of decimal point
					//Serial.println(" volts");
	return readVoltageFloat;
}//end of ADCconvertToVoltage()

void Fdr::emptyUSBreceiveBufferQ(bool waitQBool)
{//empty USB receieve buffer for up to 1 second; if waitQBool is true, delay 0.5 second before emptying buffer to allow user to enter all characters.

					#ifdef emptyBuffer
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
	
	if(waitQBool)
	{
		
				#ifdef emptyBuffer
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				#endif			
		
		delay(500);//allow time for all characters to be entered from keyboard
		resetLoopTimerBool = true;//prevents loop timer complaint
	}
	
	if(Serial.available() > 0)	
	{			
		
					#ifdef emptyBuffer
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
		
		emptySerialReceiveBuffer();	//it will empty buffer for up to 100 ms and then return. This prevents function from hanging if buffer never empty.	
	}
}//end of emptyUSBreceiveBufferQ()

void Fdr::ReturnCodeToPrintedText(unsigned int code)
{
	//ReturnCodeToPrintedText(ReturnCodeUInt);
	switch(code)
	{	

		case 100:
		Serial.println(" Failure After SBDIX");
		break;

		case 101:
		Serial.println(" Modem Status TimedOut");
		break;
		
		case 104:			 
		Serial.println(" Unexpected MO StatusValue");
		break;
		
		case 105:			  
		Serial.println(" Unexpected MOMSN Value");
		break;			

		case 106:			  
		Serial.println(" Unexpected MT StatusValue");
		break;
		
		case 107:		  
		Serial.println(" Unexpected MTMSN StatusValue");
		break;

		case 108:			  
		Serial.println(" Unexpected MT SBD MessageLength");
		break;

		case 109:
		Serial.println(" Unexpected MT SBD Message Queued Value");
		break;
		
		case 112:
		Serial.println(" Modem Failure After SBDIX");
		break;				


		case 113:
		Serial.println(" Unexpected Response ToSBDIX");
		break;	


		case 114:
		Serial.println(" Time Out After SBDIX ");
		break;	

		case 116:
		Serial.println(" Time Out After Sending Message Size");
		break;	

		case 118:
		Serial.println(" MO Buffer Cleared Error Response");
		break;				

		case 119:
		Serial.println(" MO Buffer Cleared Time Out");
		break;	
		
		case 120:
		Serial.println(" Invalid Command");
		break;	

		case 200:
		Serial.println(" MO Buffer Cleared Unexpected Response");
		break;			

		case 202:
		Serial.println(" MT Buffer Cleared Error Response");
		break;	
		
		 case 203:
		Serial.println(" MT Buffer Cleared Time Out");
		break;

		 case 204:
		Serial.println(" MT BufferClearedUnexpectedResponse");
		break;
		
		case 206:
		Serial.println(" Disable Flow Control Request Timed Out");
		break;

		case 207:
		Serial.println(" Disable Flow Control Request Unexpected Response");
		break;

		case 208:
		Serial.println(" Disable SBD Ring Setup Failed Due To  Time Out ");
		break;

		case 209:
		Serial.println(" Disable SBD RingSetupFailedDueToUnexpectedResponse");
		break;	

		case 231:
		Serial.println(" RingIndicationErrononiouslyEnabled");
		break;				


		case 232:
		Serial.println("  Time Out AfterGetResponseFromVerifyDisable MT Alert");
		break;	
		
		case 233:
		Serial.println(" UnexpectedResponseToSBDMTA");
		break;	
		
		case 234:
		Serial.println(" UnexpectedResponseAfterSendingMessageSize");
		break;	


		case 236:
		Serial.println(" SetupFailedDueTo Time Out ");
		break;

		
		case 237:
		Serial.println(" SetupFailedDueToUnexpectedResponse");
		break;
		
		case 238:
		Serial.println(" NetworkStatusUnexpectedResponse");
		break;

		case 239:
		Serial.println("  Time Out NetworkStatus");
		break;
		
		case 240:
		Serial.println(" NetworkNotAvailable");
		break;
		
		case 242:
		Serial.println(" MT MessageUnexpectedResponse");
		break;	

		case 243:
		Serial.println(" MT Message Time Out ");
		break;				

		case 244:
		Serial.println(" MT MessageFailedCheckSum");
		break;	
		
		case 245:
		Serial.println(" MT MessageTooLong");
		break;
		
		case 249:
		Serial.println(" ModemSetupFailedDueTo Time Out ");
		break;

		case 250:
		Serial.println(" NoModemConnected");
		break;
		
		case 251:
		Serial.println(" UnexpectedModemConnected");
		break;
		
		case 255:
		Serial.println(" DuplicateTransmitOfDataAttempted");
		break;
		
		case 256:
		Serial.println(" YouAreAskingToTransmitTooSoonSoTryLater");
		break;
				
		case 257:
		Serial.println(" FlowControlSetupFailedDueTo Time Out ");
		break;
		
		case 258:
		Serial.println(" FlowControlSetupFailedDueToUnexpectedResponse");
		break;
		
		case 259:
		Serial.println(" StoreConfigurationFailedDueTo Time Out ");
		break;
		
		case 260:
		Serial.println(" StoreConfigurationFailedDueToUnexpectedResponse");
		break;
		
		case 261:
		Serial.println(" SelectProfileFailedDueTo Time Out ");
		break;		
		
		case 262:
		Serial.println(" SelectProfileFailedDueToUnexpectedResponse");
		break;

		case 263:
		Serial.println(" Ring Indication Unexpected Response");
		break;
		
		case 264:
		Serial.println("OK Search TimedOut");
		break;
		
		case 291:
		Serial.println("Wrong modem connected; check Serial Number");
		break;
		
		case 328:
		Serial.println("Initial Return Code value");
		break;
   
		case 401:
		Serial.println("Modem ready for use");
		break;
				
		
		case 265:
		Serial.println("OK Unexpected Response");
		break;

		case 269:
		Serial.println("Signal Strength Too Low");
		break;

		case 272:
		Serial.println("Transmit Successful But Receive Failed");
		break;

		case 273:
		Serial.println("Unexpected Modem Command");
		break;

		case 274:
		Serial.println("MPM Busy TransmitCommand Rejected");
		break;

		case 275:
		Serial.println("Ping To MPM Timed Out");
		break;

		case 276:
		Serial.println("Ping To MPM Success But To Modem Failed");
		break;

		case 278:
		Serial.println("Initial Setup Modem Status Value");
		break;

		case 279:
		Serial.println("Time Out Waiting Forget Received Data");
		break;

		case 280:
		Serial.println("No Ping MPM Busy");
		break;

		case 281:
		Serial.println("Modem Failed At Setup");
		break;	

		case 282:
		Serial.println("Unexpected Response From Modem During Setup");
		break;	

		case 283:
		Serial.println("Modem Setup Failed Because MPM Busy");
		break;	

		case 284:
		Serial.println("Modem Failed At Setup Time Out");
		break;	

		case 285:
		Serial.println("MPM Busy When FDR Asked For Setup");
		break;	

		case 286:
		Serial.println("MPM Did Not Respond To Request For Data");
		break;

		case 287:
		Serial.println("Software Error 1");
		break;

		case 288:
		Serial.println("MPM Busy");
		break;

		case 289:
		Serial.println("Asked For Ping Result Too Soon Do Ping Again");
		break;

		case 290:
		Serial.println("Ping To MPM Did Not Respond");
		break;

		case 292:
		Serial.println("Requested Transmit Too Soon");
		break;

		case 293:
		Serial.println("No Functioning Modem Present");
		break;

		case 294:
		Serial.println("Time Out After Sending Message");
		break;

		case 295:
		Serial.println("SBD Message Time Out By Modem");
		break;

		case 296:
		Serial.println("SBD Message Checksum Wrong");
		break;

		case 297:
		Serial.println("SBD Message Size Wrong");
		break;

		case 298:
		Serial.println("Unexpected Response After Writing Data To Mobile Originated Buffer");
		break;		

		case 299:
		Serial.println("SBD Message Size Too Big Or Too Small");
		break;	

		case 300:
		Serial.println("Success Byte After SBDIX");
		break;	

		case 301:
		Serial.println("Message Size Accepted");
		break;	

		case 302:
		Serial.println("MO Buffer Cleared Successfully");
		break;	

		case 303:
		Serial.println("MT Buffer Cleared Successfully");
		break;	

		case 304:
		Serial.println("OK Found");
		break;	

		case 305:
		Serial.println("Ring Indication Disabled");
		break;	

		case 306:
		Serial.println("Setup Successful");
		break;	

		case 307:
		Serial.println("MT Message Retrieved Correctly");
		break;	

		case 308:
		Serial.println("MT Message Is Null");
		break;	

		case 309:
		Serial.println("Modem Setup Successful");
		break;	

		case 310:
		Serial.println("Correct Modem Connected");
		break;	

		case 311:
		Serial.println("idle");
		break;	

		case 313:
		Serial.println("Network Available With Acceptable SignalStrength");
		break;	

		case 314:
		Serial.println("Verify Disable MT Alert");
		break;	

		case 315:
		Serial.println("Flushed UART Buffer");
		break;	

		case 316:
		Serial.println("Array Sent To Modem");
		break;	

		case 317:
		Serial.println("Initiate Transmit And Receive");
		break;	

		case 318:
		Serial.println("Tell Modem To Clear MO Buffer");
		break;	

		case 319:
		Serial.println("Tell Modem To Clear MT Buffer");
		break;	

		case 320:
		Serial.println("Told Modem To Give Us The Received Message");
		break;	

		case 322:
		Serial.println("Transmission Process Has Begun");
		break;	

		case 323:
		Serial.println("Sent Perform Transmit");
		break;	

		case 324:
		Serial.println("SentgetReceivedData");
		break;	

		case 326:
		Serial.println("About To Start Transmit Process");
		break;	

		case 327:
		Serial.println("Waiting For OK From Modem");
		break;	

		case 330:
		Serial.println("Busy Setting Up Modem");
		break;	

		case 331:
		Serial.println("Performing Ping");
		break;	

		case 333:
		Serial.println("Ping Not Running");
		break;	

		case 334:
		Serial.println("MPM Busy Transmit Command Pending");
		break;	

		case 335:
		Serial.println("Modem Setup Proceeding");
		break;	

		case 336:
		Serial.println("Modem Defaults Set");
		break;	

		case 337:
		Serial.println("Sent Ping");
		break;	

		case 338:
		Serial.println("SBD Message Successfully Written");
		break;	

		case 339:
		Serial.println("MT Message Pending");
		break;	

		case 340:
		Serial.println("MT Messages Pending");
		break;	

		case 400:
		Serial.println("Ping Through MPM And Modem Success");
		break;	


		case 402:
		Serial.println("Transmit Successful And No Receive");
		break;	

		case 403:
		Serial.println("Transmit And Receive Successful");
		break;	

		case 404:
		Serial.println("Transmit And Receive Successful Plus Receive Pending");
		break;	

		case 405:
		Serial.println("data Loop Around Enabled");
		break;	

		case 406:
		Serial.println("data Looop Around Disabled");
		break;	

		case 407:
		Serial.println("receive Data Placed In Receive Array");
		break;

		default:		
		Serial.println(code);//print return code if not translated
		break;
	}
}//end of ReturnCodeToPrintedText()

bool Fdr::pingI2C()
{//we are testing the I2C bus by reading the first byte in the SEEPROM. I chose this function because it returns an error code. ReadEEPROM() does not.

	int pingResultInt = ReadSEEPROM(0);//all failures cause the MSB to equal 0x01. If no failures, it equals 0.
	if(pingResultInt >> 8 != 0)return false;
	return true;
}

bool Fdr::I2CbusBusyWithDataDumpQ()
{//return true if a data dump is active. Otherwise, return false
	if((I2Cbus1OwnerByte == dumpEEPROMuserByte) || (I2Cbus1OwnerByte == dumpSEEPROMuserByte))
	{
		return true;
	}else
	{
		return false;
	}
}


bool Fdr::I2Cbus1Mitigation(byte userByte)
{
/******************************************************
Any library function can run on either core so we must deal with the situation where both cores are running this code at the exactly same or nearly the same time. It has happend and has caused random failures on the bus with both cores owning it. We prevent such collisions by maintaining a time division strategy that counts from 0 to 7 and then rolls backs to 0. Each count takes 1 ms.

When we start I2Cbus1Mitigation() and it is running on core 0, we must wait until we are in timeslot 0 or 1 which is defined as time slot 0. If we start I2Cbus1Mitigation() and we are running on core 1, we must wait until we are in timeslots 4 or 5 which is time slot 1. Durng timeslots 2,3,6, and 7 we can be processing the bus state but can't start it. These are defined as being in the guard band. This gives time for I2Cbus1Mitigation() to stabilize before the other core can request the bus. In this way, we avoid the case of both cores calling I2Cbus1Mitigation() at the same or nearly same time and having a race condition that looks like a hardware failure on the I2C bus.

return true if bus and user transfer successful and false if we timed out. We hang if in data dump.

externalLED() reports bus status if bus1_occupancy is defined

bus1 usage is logged using busUseTimeULong[] and busUseStateByte[]. It is read out as part of the H command at which time the logFileIndexByte is set back to 0. The depth of the log file is set with logFileIndexByte.
*********************************************************/

									#ifdef releaseDiag
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif


	unsigned long startOfMitigationTimeULong = millis();
	
									#ifdef busState
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									Serial.print("I2Cbus1LinkedToCoreByte = ");
									Serial.println(I2Cbus1LinkedToCoreByte);
									Serial.print("I2Cbus1OwnerByte = ");
									Serial.println(I2Cbus1OwnerByte);
									#endif
	
										#ifdef bus1_occupancy 
											seizeExternalLEDcontrol();
										#endif
		
	while(true)
	{//we keep cycling until we hit the right combination of core number and time slot
		
		timeSlotGenerator(1000);//time slot generator tells us if we are time slot 0, 1, or a guardband. 1000 means 1000 microseconds per time slot and guard band. 
	
//core 0 time slot 0	
		if(rp2040.cpuid() == 0 && timeSlot0Bool)
		{
			if(I2Cbus1LinkedToCoreByte == neitherByte)
			{//bus is not linked to either core so give it to core 0
				I2Cbus1LinkedToCoreByte = 0;
				
											#ifdef I2Cstat
											Serial.print(" [ ");
											#endif
				
				I2Cbus1OwnerByte = userByte;//give proposed user ownership
				
						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OnByte );
						#endif
				
				logBusState(0,userByte);
				
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif				
				
				return true;//core 0 gets bus
			}
			
			if(I2Cbus1LinkedToCoreByte == 1)
			{//bus is currently linked to core 1
				while(I2Cbus1LinkedToCoreByte == 1)//wait for core 1 to release bus but don't hang
				{//if we are not doing a data dump, then print an error if we stay in this loop more than 1000 ms and then return false.
					if(((I2Cbus1OwnerByte != dumpEEPROMuserByte) && (I2Cbus1OwnerByte != dumpSEEPROMuserByte)) && (millis() - startOfMitigationTimeULong > 1000))
					{
						Serial.println();
						Serial.print("There is a bug because ");
						userNumberToEnglish(I2Cbus1OwnerByte);
						Serial.println(" waited more than 1 second for the I2C bus1.");
						
						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif						
						logBusState(30,userByte);
						
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif						
						
						return false;
					}
				//otherwise, keep waiting
				}
				//to get to here, core 1 has released so we can link core 0			
				I2Cbus1LinkedToCoreByte = 0;//then link core 0 to the bus and
				I2Cbus1OwnerByte = userByte;//give proposed user ownership
				
						#ifdef I2Cstat
						Serial.print(" [ ");
						#endif

								#ifdef diagLEDmode
								externalLED(onBool);
								#endif
				logBusState(0,userByte);

												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif
				
				return true;//core 0 gets bus
			}
			
			if((I2Cbus1LinkedToCoreByte == 0) && (I2Cbus1OwnerByte != userByte))
			{//This is a software bug caused by a function running on core 0 not releasing the bus before they return and then another function on this core needing the bus.
		
									#ifdef readDebug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif		
				
				userNumberToEnglish(I2Cbus1OwnerByte);
				Serial.println(" didn't release bus 0 before returning.");//I don't trust the bus so can't log the error.			
				I2Cbus1OwnerByte = userByte;//give proposed user ownership	

						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif	
				logBusState(31,userByte);	

												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif
				
				return false;
			}
			
			if((I2Cbus1LinkedToCoreByte == 0) && (I2Cbus1OwnerByte == userByte))
			{//This is a software bug caused by the current owner not releasing the bus and now asking for it.
				userNumberToEnglish(I2Cbus1OwnerByte);
				Serial.println(" didn't release bus 0 before returning and now wants it.");
		
									#ifdef readDebug
									Serial.print("core ");
									Serial.print(rp2040.cpuid());
									Serial.print(" ");
									Serial.print(__FUNCTION__);
									Serial.print(F("(): "));		
									Serial.print(__LINE__);
									Serial.print(F(".    Time Stamp  "));	
									Serial.print (millis() - StartForTimeStampULong);
									Serial.println(F(" ms"));
									#endif		


						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif
				logBusState(32,userByte);	

												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif
				
				return false;
			}
		}//end of core 0 time slot 0	

//core 1 time slot 1			
		if(rp2040.cpuid() == 1 && timeSlot1Bool)
		{//I2Cbus1Mitigation() just started running on core 1 so can become stable before core 0 can run
			if(I2Cbus1LinkedToCoreByte == neitherByte)
			{//bus is not linked to either core give it to core 1 and return.
				I2Cbus1LinkedToCoreByte = 1;
				I2Cbus1OwnerByte = userByte;//give proposed user ownership

														#ifdef I2Cstat
														Serial.print(" ] ");
														#endif

								#ifdef diagLEDmode
								externalLED(onBool);
								#endif

						#ifdef bus1_occupancy 
							externalLED(ExternalLED_dimByte );
						#endif
				logBusState(1,userByte);	
				
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif				
				
				return true;//core 1 gets bus
			}
			
			if(I2Cbus1LinkedToCoreByte == 0)
			{//bus is currently linked to core 0 
				while(I2Cbus1LinkedToCoreByte == 0)//so wait for core 0 to release it but don't hang
				{//if we are not doing a data dump, then print an error if we stay in this loop more than 1000 ms and then return failed.
					if(((I2Cbus1OwnerByte != dumpEEPROMuserByte) && (I2Cbus1OwnerByte != dumpSEEPROMuserByte)) && (millis() - startOfMitigationTimeULong > 1000))
					{
						Serial.println();
						Serial.print("There is a bug because ");
						userNumberToEnglish(I2Cbus1OwnerByte);
						Serial.println(" waited more than 1 second for the I2C bus1.");
						
						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif						
						logBusState(33,userByte);
						
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif						
						
						return false;
					}
				//otherwise, keep waiting
				}
				//to get to here, core 0 has released so we can link core 1
				I2Cbus1LinkedToCoreByte = 1;//then link core 1 to the bus and
				I2Cbus1OwnerByte = userByte;//give proposed user ownership

													#ifdef I2Cstat
													Serial.print(" ] ")						
													#endif

						#ifdef bus1_occupancy 
							externalLED(ExternalLED_dimByte );
						#endif
				logBusState(1,userByte);
				
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif				
				
				return true;//core 1 gets bus
			}
			if((I2Cbus1LinkedToCoreByte == 1) && (I2Cbus1OwnerByte != userByte))
			{//This is a software bug caused by a function running on core 1 not releasing the bus before they return and then another function on this core needing the bus.
		
													#ifdef readDebug
													Serial.print("core ");
													Serial.print(rp2040.cpuid());
													Serial.print(" ");
													Serial.print(__FUNCTION__);
													Serial.print(F("(): "));		
													Serial.print(__LINE__);
													Serial.print(F(".    Time Stamp  "));	
													Serial.print (millis() - StartForTimeStampULong);
													Serial.println(F(" ms"));
													#endif		
				
				userNumberToEnglish(I2Cbus1OwnerByte);
				Serial.println(" didn't release bus 1 before returning.");//I don't trust the bus so can't log the error.										
				I2Cbus1OwnerByte = userByte;//give proposed user ownership	


						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif
						
				logBusState(34,userByte);	

												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif
				
				return false;
			}
			
			if((I2Cbus1LinkedToCoreByte == 1) && (I2Cbus1OwnerByte == userByte))
			{//This is a software bug caused by the current owner not releasing the bus and now asking for it.
				userNumberToEnglish(I2Cbus1OwnerByte);
				Serial.println(" didn't release the bus 1 before returning and now wants it.");
		
													#ifdef readDebug
													Serial.print("core ");
													Serial.print(rp2040.cpuid());
													Serial.print(" ");
													Serial.print(__FUNCTION__);
													Serial.print(F("(): "));		
													Serial.print(__LINE__);
													Serial.print(F(".    Time Stamp  "));	
													Serial.print (millis() - StartForTimeStampULong);
													Serial.println(F(" ms"));
													#endif		


						#ifdef bus1_occupancy 
							externalLED(ExternalLED_OffByte );
						#endif
				
				logBusState(35,userByte);

												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												#endif
				
				return false;
			}
			
		}//end of core 1 time slot 1
		
	}//if we got this far, we are in a guard band so circle back and try again.
}//end of I2Cbus1Mitigation()	

void Fdr::releaseI2Cbus()
{//to avoid collisions between cores, we only permit a core to write to the log file during its timeslot
		
					#ifdef releaseDiag
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
		

				logBusState(neitherByte,99);//99 means no user
				//log first and then relase the bus				
				I2Cbus1LinkedToCoreByte = neitherByte;
				I2Cbus1OwnerByte = I2CbusReleasedByte;

									#ifdef bus1_occupancy 
									externalLED(ExternalLED_OffByte );
									#endif	
												
												#ifdef busTrace
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												//delay(100);//diag
												#endif
												
										
}

float Fdr::readBattery(bool modeBool)
{
	/*****************************************************
	Battery is read from the Pro Micro's ADC and not the I2C based ADC. If mode is false, we output count as a float. If mode is true, we output voltage as a float
	*****************************************************/
	byte GPIO29 = 29;
	byte PM_ADCbatteryPortByte = GPIO29;//The PM has GPIO 29 tied to the voltage divider that monitors the battery.
	int rawCountInt = analogRead(PM_ADCbatteryPortByte);
	if(!modeBool)
	{
		return float(rawCountInt);
	}
	//else, output as a voltage
	float portVoltageFloat = float(rawCountInt)*PMadcMvPerStepFloat;//   3.3V/4096 = 0.80566 mV/step 	
	
						
	float BatteryVoltageFloat = batteryVoltageDividerCompensationFloat * portVoltageFloat;
	
						#ifdef batteryReading1
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.println(__LINE__);
						Serial.print("raw count is ");
						Serial.println(rawCountInt);
						Serial.print("port voltage should be ");
						Serial.println(portVoltageFloat,3);
						Serial.print("battery voltage should be ");
						Serial.println(BatteryVoltageFloat,3);
						#endif
						
	return BatteryVoltageFloat;
}//end of readBattery()

void Fdr::topOfBox()
{
	Serial.println("   ********************************");
	Serial.println("   *                              *");
}

void Fdr::bottomOfBox()
{
	Serial.println("   *                              *");
	Serial.println("   ********************************");
}

void Fdr::welcomeSerialMonitor()
{//when Data Terminal Ready is detected for the first time, it outputs menu to TeraTerm
	if(Serial.dtr() && newDTR_Bool)
	{//we see DTR for first time since power up
		newDTR_Bool = false;
		OutputMenuOfCommandsAndLED_Cadences(false);//output short form of menu
		Command = 'N';//set command to null
		printTheUserPromptQ = yesBool;//print the user prompt. With Command = 'N' and printTheUserPromptQ = yesBool we will trigger the prompt.
	}
	if(Serial.dtr() != 1 && newDTR_Bool == false)newDTR_Bool = true;//if DTR is 0 and newDTR_Bool is false, it means we lost DTR and it just came back so treat it as a new connection
}

void Fdr::controlBlockSanityTest()
{//check that all control block values are within bounds. If any are not, warn user and init control block.

	ReadEEPROM_Pointer(CalledBySanityCheckControlBlockByte);//output is startOfNextMemoryBlockLong
	
			#ifdef CBfatal
			Serial.print("Start of next memory block = ");
			Serial.println(startOfNextMemoryBlockLong);
			Serial.print("Start Of Usable EEPROM = ");
			Serial.println(StartOfUsableEEPROM_Long);
			#endif
	
	bool controlBlockIsOK = true;
	
							//Serial.println("Control Block Test");
	
	if (ReadControlBlock(ProgramStateAddressByte) > 3)
	{
		controlBlockIsOK = false;
		Serial.print("Program state should be < 3 but is ");Serial.println(ReadControlBlock(ProgramStateAddressByte));
		goto initControlBlock; 
	}
	
	if ((ReadControlBlock(dataDumpStatusAddressByte) > 1))
	{
		controlBlockIsOK = false;
		Serial.print("Dump Status should be < 2 but is ");Serial.println(ReadControlBlock(dataDumpStatusAddressByte));
		goto initControlBlock;
	}
	
	if ((ReadControlBlock(2) != 2))
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 2 should be 2 but is ");Serial.println(ReadControlBlock(2));
		goto initControlBlock;
	}
	
	if ((startOfNextMemoryBlockLong < StartOfUsableEEPROM_Long))
	{
		controlBlockIsOK = false;
		Serial.print("The start of the next memory block must be equal to or greater than the start of usable EEPROM but is not.");
		goto initControlBlock;
	}
	
	if (startOfNextMemoryBlockLong > (131072L - long(DataBlockSizeByte)))
	{
		controlBlockIsOK = false;
		Serial.print("The start of the next memory block must be less than or equal to the last block in memory but is ");Serial.println(startOfNextMemoryBlockLong);
		goto initControlBlock;
	}
	
	if ((ReadControlBlock(7) != 7))  
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 7 should contain 7 but is ");Serial.println(ReadControlBlock(7));
		goto initControlBlock;
	}

	if ((ReadControlBlock(8) != 8))
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 8 should contain 8 but is ");Serial.println(ReadControlBlock(8));
		goto initControlBlock;
	}
	
	if ((ReadControlBlock(9) != 9))
	{
		Serial.print("Unused address 9 should contain 9 but is ");Serial.println(ReadControlBlock(9));
		controlBlockIsOK = false;
		goto initControlBlock;
	}	

	if (ReadControlBlock(ErrorCodeAddressByte) > 20) 
	{
		controlBlockIsOK = false;
		Serial.print("The error code must be less than or equal to 20 but is ");Serial.println(ReadControlBlock(ErrorCodeAddressByte));
		goto initControlBlock;
	}
	
	if (ReadControlBlock(11) != 11)
	{
		Serial.print("Unused address 11 should contain 11 but is ");Serial.println(ReadControlBlock(11));
		controlBlockIsOK = false;
		goto initControlBlock;
	}

	if (ReadControlBlock(12) != 12)
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 12 should contain 12 but is ");Serial.println(ReadControlBlock(12));
		goto initControlBlock;
	}
	
	if (ReadControlBlock(13) != 13)
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 13 should contain 13 but is ");Serial.println(ReadControlBlock(13));
		goto initControlBlock;
	}	

	if (ReadControlBlock(14) != 14)
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 14 should contain 14 but is ");Serial.println(ReadControlBlock(14));
		goto initControlBlock;
	}

	if (ReadControlBlock(15) != 15)
	{
		controlBlockIsOK = false;
		Serial.print("Unused address 15 should contain 15 but is ");Serial.println(ReadControlBlock(15));
		goto initControlBlock;
	}
	//Serial.println(__LINE__);//diag	
	return;//control block elements are within expected range

initControlBlock:
	
	if(controlBlockIsOK == false)
	{

		#ifdef OTMlapse
		Serial.println();
		Serial.print("Control block: ");
		for(byte addressByte = 0; addressByte < 16; addressByte++)
		{
			Serial.print(ReadControlBlock(addressByte));
			Serial.print(" ");
		}
		Serial.println();
		#endif

	//initialize the Control Block
		WriteControlBlock(ProgramStateAddressByte,MemoryLockedByte);
		
		WriteControlBlock(dataDumpStatusAddressByte,dataNotBeingDumpedByte);
		
		WriteControlBlock(2,2);//spare
		
		startOfNextMemoryBlockLong = StartOfUsableEEPROM_Long + 16;//just above Control Block
		WriteEEPROM_Pointer(); //the variable startOfNextMemoryBlockLong is written to the control block
		
		WriteControlBlock(7,7);//spare
		WriteControlBlock(8,8);//spare
		WriteControlBlock(9,9);//spare
		
		WriteControlBlock(ErrorCodeAddressByte,SystemNormalFlagByte);
		
		WriteControlBlock(11,11);//spare
		WriteControlBlock(12,12);//spare
		WriteControlBlock(13,13);//spare
		WriteControlBlock(14,14);//spare
		WriteControlBlock(15,15);//spare
	
			Serial.println();
			Serial.println();
			topOfBox();
			Serial.println("   *     EEPROM was initialized   *");
			Serial.println("   *      because at least one    *");
			Serial.println("   * parameter was outside of the *");
			Serial.println("   *         normal limits.       *");
			bottomOfBox();
			Serial.println();
			Serial.println();
	}
}//end of controlBlockSanityTest()

void Fdr::printTeamsName()
{
//print team's name as a function of GPIO4 grounded or not unless we are in diagLEDmode. 
	#ifdef diagLEDmode
	Serial.println();
	Serial.println("*** The external LED is in diagnostic mode. ***");
	Serial.println();
	return;
	#endif
		
		if(!simulatorModeQ)
		{
			Serial.println();	
			if(digitalRead(4)) //GPIO4 disconnected so this is team 2
			{
				Serial.print(" Welcome ");
				Serial.print(team2NameChar);
				Serial.print(" members: ");
				Serial.print(team2MembersChar);
			}else
			{//GPIO4 tied to ground so this is team 1
				Serial.print(" Welcome ");
				Serial.print(team1NameChar);
				Serial.print(" members: ");
				Serial.print(team1MembersChar);
			}
		}else
		{
			Serial.println();
			Serial.println("               *** Simulation Mode ***");
		}
		Serial.println();
		Serial.println();		
}

void Fdr::processAnyActiveCommand()
{//see if Command matches any of these options. Command = 'N' means none.
	OutputMenuQ(); //M command
	RebootQ();//R and r commands
	SingleDisplayDataQ(); //D command
	PrepareForLaunchQ(); //P command
	ContinuousDisplayDataQ(); //A command  CR1.7
	OutputDataFileQ(); //O command
	StopCollectingDataQ(); //S command
	DiagnosticOutputControlBlockQ(); //H command
	GenerateTestPatternQ(); //T command	
	clearRecordedErrorsQ(); //C command
}

void Fdr::printAmountOfDataStoredAsHMS()
{
	ReadEEPROM_Pointer(CalledByDiagnosticOutputControlBlockByte);//retrieve startOfNextMemoryBlockLong
	long timeRecordedSecondsLong = (2*(startOfNextMemoryBlockLong - StartOfUsableEEPROM_Long))/long(DataBlockSizeByte);//times 2 because I take 2 seconds per data block.
	long timeRecordedHoursLong = timeRecordedSecondsLong/3600;//truncated so I get integer hours
	
	long timeRecordedMinutesLong = (timeRecordedSecondsLong/60) - (timeRecordedHoursLong*60);//truncated so I get integer minutes after subtracting hours
	timeRecordedSecondsLong = timeRecordedSecondsLong - (60*timeRecordedMinutesLong) - (3600*timeRecordedHoursLong);//subtract off integer minutes as seconds and integer hours as seconds so the remainder is seconds and I get hours, minutes, and seconds recorded
	Serial.println();
	Serial.print(F("Memory holds "));	

switch(timeRecordedHoursLong)
	{
		case 0://if no hours, don't print anything
			break;
			
		case 1:
			Serial.print("1 hour ");
			break;
			
		default:
				Serial.print(timeRecordedHoursLong);
				Serial.print(" hours ");
	}	

	switch(timeRecordedMinutesLong)
	{
		case 0:
			if(timeRecordedHoursLong == 0)break;//no hours or minues so don't print either of them
			
			Serial.print("0 minutes ");//we have hours but no minutes
			break;
			
		case 1:
			Serial.print("1 minute ");
			break;
			
		default:
				Serial.print(timeRecordedMinutesLong);
				Serial.print(" minutes ");
	}
	
	switch(timeRecordedSecondsLong)
	{
		case 0:
		if((timeRecordedHoursLong == 0) && (timeRecordedMinutesLong == 0))
		{
			Serial.print("no");//no hours, minutes, or seconds so no data is in memory
			break;
		}else
		{
			Serial.print("0 seconds of");
			break;
		}
		
		case 1:
			Serial.print("1 second of");
			break;
			
		default:
			Serial.print(timeRecordedSecondsLong);
			Serial.print(" seconds of");
	}
			
	Serial.print(" data. (");//trailing ( is in front of % of memory
}//end of printAmountOfDataStoredAsHMS()

void Fdr::printTotalRunTime()
{
	long maxAddressLong = 131072L;
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 5000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	if(simulatorModeQ)maxAddressLong = 1599L;		
	int PercentFullInt = int(0.5+((startOfNextMemoryBlockLong-StartOfUsableEEPROM_Long)*100L)/(maxAddressLong-StartOfUsableEEPROM_Long));//round to the nearest percent CR3.2
	Serial.print(PercentFullInt);
	Serial.println(F("% of memory full)"));
	Serial.print(F("Data is recorded every "));
	if(SamplingRateSecondsByte == 1) {
		Serial.print(F("second")); 
	}else
	{
		Serial.print(SamplingRateSecondsByte);	
		Serial.print(F(" seconds"));
	}
	Serial.print(F(" for a total runtime of "));
	float SampleRateFloat = float(SamplingRateSecondsByte);
	float maxDataBlocksFloat = 8191;//The max number of data blocks is {[2^17] - control block}/16 = (131072 - 16)/16 = 8191 block of data.
	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 5000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.	
	if(simulatorModeQ)
	{
		maxDataBlocksFloat = 99;//The max number of data blocks is {1599 - control block}/16 = (1599 - 16)/16 = 99 block of data.
		float TotalRunTimeMinutesFloat = maxDataBlocksFloat*SampleRateFloat/60; //maximum run time in decimal minutes
		Serial.print(TotalRunTimeMinutesFloat);
		Serial.println(F(" minutes because we are in simulation mode."));	
	}else
	{
		float TotalRunTimeHoursFloat = maxDataBlocksFloat*SampleRateFloat/3600; //maximum run time in decimal hours
		unsigned int TotalRunTimeHoursSegmentUInt = int(TotalRunTimeHoursFloat);//strip off hours
		unsigned int TotalRunTimeMinutesSegmentUInt =
		(TotalRunTimeHoursFloat - float(TotalRunTimeHoursSegmentUInt))*60;
		Serial.print(TotalRunTimeHoursSegmentUInt);
		Serial.print(F(" hours "));
		Serial.print(TotalRunTimeMinutesSegmentUInt);
		Serial.println(F(" minutes."));
	}
}//end of printTotalRunTime() 

void Fdr::printControlBlockStateAndError()
{
	Serial.println();
	Serial.println();
	Serial.println(F("Control Block"));
	Serial.println(F("======================"));
	
	switch(ReadControlBlock(ProgramStateAddressByte))
	{
		case 3://PreFlightByte
		Serial.println("In Pre-flight");
		break;
		
		case 0: //InFlightByte
		Serial.println("In flight");
		break;	

		case 1: //InFlightWithPowerDisruptionByte
		Serial.println("In flight with power hit");
		break;

		case 2://MemoryLockedByte
		Serial.println("Memory is locked and in the Data Readout state.");
		break;

		default:
		Serial.println("Error: unknown system state.");	
		break;
	}
	
	DisplayErrorState();
	
}//end of printControlBlockStateAndError()

void Fdr::printFileInfo()
{
	Serial.println();
	Serial.print(F("The FDR.cpp file was last modified or downloaded on "));
	Serial.print(__DATE__);
	Serial.print(" at ");
	Serial.print(__TIME__);
	Serial.println(".");
	Serial.println();
	Serial.println(__FILE__);
	Serial.println();
	Serial.print(F("Fdr.cpp Version "));
	Serial.print(Fdr_cpp_versionChar);
	Serial.println();
	Serial.println();
	Serial.print(F("The calling program came from "));
	//use filePathChar to print the .ino path
	char *programPathChar = filePathChar;  //define a local string that contains my .ino program path
	Serial.println(programPathChar);//the intent is to print the .ino's program path
	Serial.println();
}//end of printFileInfo()

bool Fdr::readEEPROMdataBlock(long eeAddressLong)
{//eeAddressLong is the base address for the data block. We read 16 bytes starting with the base address and put the result into portDataByte[] which is global. Success returns true and any failure returns false.


								#ifdef speedOfPrintTest 
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								//Serial.print(F("enablePort6HighSpeedRead  = "));
								//Serial.println(enablePort6HighSpeedRead );
								#endif 

	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 5000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing EEPROM so failure is a valid response and we will just continue without delay.
	
	
	
	if(simulatorModeQ)
	{
		for(byte i = 0; i < 16; i++)
		{
				portDataByte[i] = simEEPROM[int(eeAddressLong + i)];
		}
		return true;//bus not used so no need to release it
	}

	if(eeAddressLong > 131071L)
	{
		return false;//bus not used so no need to release it
	}
	
	//any address that causes a read over a BLOCK boundry is rejected. The LOW BLOCK ends at 65536 so if I start to read more than 16 counts below this value, I will read beyond the top. 
	if ((eeAddressLong > (65536L - 16)) && (eeAddressLong < (65536L)))
	{
		WriteControlBlock(ErrorCodeAddressByte, EEPROMdataFailureByte);//log fact that block address spans LOW and HIGH BLOCKS
		Serial.print("readEEPROMdataBlock() requested block read spanned LOW and HIGH BLOCKS");
		Serial.println(__LINE__);
		
		return false;//address spans LOW and HIGH BLOCKS so is a bug.
	}
	
//address is valid so we can access EEPROM
	
	I2Cbus1Mitigation(readEEPROMdataBlockUserByte);//sieze bus and identify yourself. As the data block reader, we don't time out as we use the bus	 
	
	if (eeAddressLong < 65536L) 
	{ //send block address based on eeAddressLong 
		Wire1.beginTransmission(EEPROM_ADR_LOW_BLOCK);
	}else
	{
		Wire1.beginTransmission(EEPROM_ADR_HIGH_BLOCK);
	}
	
  //then send data address of first byte within the block 
	Wire1.write((byte)(eeAddressLong >> 8)); //queue MSB
	Wire1.write((byte)(eeAddressLong)); //queue LSB
	Wire1.endTransmission(false);//transmit address message but not a stop symbol so I can use sequential read feature of device
	
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms so wait 6.
	
//request 16 bytes from EEPROM and then send stop symbol
	if (eeAddressLong < 65536L)
	{
		Wire1.requestFrom(EEPROM_ADR_LOW_BLOCK, 16,true);
	}else
	{
		Wire1.requestFrom(EEPROM_ADR_HIGH_BLOCK, 16, true);
	}
	
	delay(waitForEEPROMresponseMsByte);//give time for device to respond
	
	if (Wire1.available() == 16) //expected number of bytes received
	{
		for(byte countByte = 0; countByte < 16; countByte++)
		{
			portDataByte[countByte] = Wire1.read(); //fill output array from buffer	
		}			
	}else
	{
		FlushWire1Buffer();//unexpected number of bytes returned.
		releaseI2Cbus();
		
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif		
		
		#ifdef bus1Status
		Serial.println("bus released 2.1");
		#endif
					
		WriteControlBlock(ErrorCodeAddressByte, EEPROMdataFailureByte);//log fact that I was unable to read back from EEPROM
		Serial.print("EEPROM did not respond as excpect to read by readEEPROMdataBlock(). Line ");
		Serial.println(__LINE__);
		
		return false;
	}
	
				
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif				
				
						#ifdef bus1Status
						Serial.println("bus released 2.1");
						#endif
							
				
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif	

						#ifdef speedOfPrintTest 
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(" ms.");
						#endif 						
				

//all bytes in data block read and stored in portDataByte[]
	releaseI2Cbus();
	return true;
	
}//end of readEEPROMdataBlock()

bool Fdr::readSEEPROMdataBlock(long eeAddressLong)
{//eeAddressLong is the base address for the data block. We read 16 bytes starting with the base address and put the result into portDataByte[] which is global. Success returns true and any failure returns false.
//output is GNSSdataByte[]

						#ifdef speedOfPrintTest 
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F("enablePort6HighSpeedRead  = "));
						Serial.println(enablePort6HighSpeedRead);
						#endif

	waitForOneTimeRunToFinish();//If FDRoneTimeRun() has not run yet and I'm running in core 1, wait up to 5000 ms for it to run. If we are in core 0, then we are testing I2C bus by accessing SEEPROM so failure is a valid response and we will just continue without delay.
	
	if(simulatorModeQ)
	{
		for(byte i = 0; i < 16; i++)
		{
				GNSSdataByte[i] = simSEEPROM[int(eeAddressLong + i)];
		}
		return true;//bus not used so no need to release it
	}

	if(eeAddressLong > 131071L)
	{
		return false;//bus not used so no need to release it
	}
	
	//any address that causes a read over a BLOCK boundry is rejected. The LOW BLOCK ends at 65536 so if I start to read more than 16 counts below this value, I will read beyond the top. 
	if ((eeAddressLong > (65536L - 16)) && (eeAddressLong < (65536L)))
	{
		WriteControlBlock(ErrorCodeAddressByte, SEEPROMdataFailureByte);//log fact that block address spans LOW and HIGH BLOCKS
		Serial.print("readSEEPROMdataBlock() requested block read spanned LOW and HIGH BLOCKS");
		Serial.println(__LINE__);
		
		return false;//address spans LOW and HIGH BLOCKS so is a bug.
	}
	
//address is valid so we can access SEEPROM
	
	I2Cbus1Mitigation(readSEEPROMdataBlockUserByte);//sieze bus and identify yourself. As the data block reader, we don't time out as we use the bus	 
	
	if (eeAddressLong < 65536L) 
	{ //send block address based on eeAddressLong 
		Wire1.beginTransmission(SEEPROM_ADR_LOW_BLOCK);
	}else
	{
		Wire1.beginTransmission(SEEPROM_ADR_HIGH_BLOCK);
	}
	
  //then send data address of first byte within the block 
	Wire1.write((byte)(eeAddressLong >> 8)); //queue MSB
	Wire1.write((byte)(eeAddressLong)); //queue LSB
	Wire1.endTransmission(false);//transmit address message but not a stop symbol so I can use sequential read feature of device
	
	delay(writeCycleDelayByte); //Write cycle time is max of 5 ms so wait 6.
	
//request 16 bytes from SEEPROM and then send stop symbol
	if (eeAddressLong < 65536L)
	{
		Wire1.requestFrom(SEEPROM_ADR_LOW_BLOCK, 16,true);
	}else
	{
		Wire1.requestFrom(SEEPROM_ADR_HIGH_BLOCK, 16, true);
	}
	
	delay(waitForEEPROMresponseMsByte);//give time for device to respond
	
	if (Wire1.available() == 16) //expected number of bytes received
	{
		for(byte countByte = 0; countByte < 16; countByte++)
		{
			GNSSdataByte[countByte] = Wire1.read(); //fill GNSSdataByte{} array from buffer	
		}			
	}else
	{
		FlushWire1Buffer();//unexpected number of bytes returned.
		releaseI2Cbus();
		
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif		
		
		#ifdef bus1Status
		Serial.println("bus released 2.1");
		#endif
					
		WriteControlBlock(ErrorCodeAddressByte, SEEPROMdataFailureByte);//log fact that I was unable to read back from SEEPROM
		Serial.print("SEEPROM did not respond as excpect to read by readSEEPROMdataBlock(). Line ");
		Serial.println(__LINE__);
		
		return false;
	}
	
				
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif				
				
						#ifdef bus1Status
						Serial.println("bus released 2.1");
						#endif
							
				
						#ifdef ADCdebug
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.print(F(" ms"));
						Serial.println("  bus released");
						#endif				
				

//all bytes in data block read and stored in portDataByte[]
	releaseI2Cbus();
	return true;	
}//end of readSEEPROMdataBlock()

void Fdr::populateSimulatedControlBlock()
{//forces control block to locked memory, next available address of 16, and no errors.
	simEEPROM[0] = MemoryLockedByte;
	simEEPROM[1] = dataNotBeingDumpedByte;
	simEEPROM[2] = 2;//spare
	simEEPROM[3] = byte(startOfNextMemoryBlockLong) + 16;//LSB of address of next data block shifted up if CB was moved
	simEEPROM[4] = 0;//I'm assuming the CB shift will never be more than 255 - 16.
	simEEPROM[5] = 0;
	simEEPROM[6] = 0;//MSB
	simEEPROM[7] = 7;//spare
	simEEPROM[8] = 8;//spare
	simEEPROM[9] = 9;//spare
	simEEPROM[10] = SystemNormalFlagByte;//error code
	simEEPROM[11] = 11;//spare
	simEEPROM[12] = 12;//spare
	simEEPROM[13] = 13;//spare
	simEEPROM[14] = 14;//spare
	simEEPROM[15] = 15;//spare	
}

bool Fdr::areWeDoneWithStartUpDiagQ()
{//this function provides setup1() access to the hasFDRoneTimeRunNotRunYetQ flag. hasFDRoneTimeRunNotRunYetQ is initialized to true. When FDRoneTimeRun() is done running, it set this flag to false. I invert the flag and return it so when start up diag is done, we return true.
	return !hasFDRoneTimeRunNotRunYetQ;
}

void Fdr::emptySerialReceiveBuffer()
{
	long startEmptyULong = millis();
	//char receivedChar;
	while((Serial.available() > 0) && (millis()-startEmptyULong < 100)) 
	{//100 ms time limit prevents the program from hanging

		//receivedChar = 
		
		Serial.read();//all characters are read but not echoed back to user
		//Serial.print(receivedChar);//diag
	}
}	

void Fdr::waitForOneTimeRunToFinish()
{
	long oneTimeRunWaitStartTimeLong = millis();
	while(hasFDRoneTimeRunNotRunYetQ && (rp2040.cpuid() == 1) && (millis() - oneTimeRunWaitStartTimeLong < 5000))
	{//I est 3600 ms to finish. It takes 2400 ms to just exercise the control block. I chose 5 seconds so I would not permanently freeze but it should never be that long. This is not an issue if we are running in core 0 because FDRoneTimeRun() and waitForOneTimeRunToFinish() can't run at same time.	
	}
}

/*  no longer needed because loop1() timer moved into library.
bool Fdr::isLoop1TimerActiveQ()
{//if resetLoop1TimerBool was set, provide access to this flag from Iridium.ino
	if(resetLoop1TimerBool)
	{
		
					#ifdef loop1TimerTes1
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
		return true;
	}else
	{
					#ifdef loop1TimerTes1
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
		return false;
	}
}

void Fdr::resetLoop1TimerBool = true
{
		resetLoop1TimerBool = true;
}

void Fdr::enableLoop1TimerWarning()
{//this enables Iridium.ino to clear the resetLoop1TimerBool flag

					#ifdef loop1TimerTes1
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif

	resetLoop1TimerBool = false;
}
*/
void Fdr::printGNSSstatusInEnglish()
{
	
	switch((*NavigationDataByte)[0])
	{
		case 200:
		Serial.println("OK but no data yet");
		return;
		
		case 201:
		Serial.println("hardware failure");		
		return;	
		
		case 203:
		Serial.println("interface failure");		
		return;
		
		case 205:
		Serial.println("try later");		
		return;	
		
		default:
		Serial.println("unknown status");	
		return;
	}
}

void Fdr::userNumberToEnglish(byte userNumber)
{
	switch(userNumber)
	{
		case 0:
		Serial.print("unprotectedWriteEEPROM ");
		return;
		
		case 1:
		Serial.print("unprotectedWriteEEPROM ");
		return;
		
		case 2:
		Serial.print("readEEPROM ");
		return;
		
		case 3:
		Serial.print("writeSEEPROM ");
		return;
		
		case 4:
		Serial.print("readSEEPROM ");
		return;
		
		case 5:
		Serial.print("ADCread ");
		return;
		
		case 6:
		Serial.print("dumpEEPROM ");
		return;
		
		case 7:
		Serial.print("dumpSEEPROM ");
		return;
		
		case 8:
		Serial.print("readEEPROMdataBlock ");
		return;
		
		case 9:
		Serial.print("readSEEPROMdataBlock ");
		return;
		
		case 10:
		Serial.print("unprotectedWriteSEEPROM ");
		return;
		
		default:
		Serial.print(userNumber);
		return;
	}
}//end of userNumberToEnglish()

void Fdr::Loop1CycleTimeMonitor(bool verboseQ)
/**********************************************
The function must only run on core 1.

Alarms when loop1() cycle time exceeds maximumLoop1CycleTimeMS_UInt (1 second) and logs the failure. Starts monitoring 30 seconds after power up to avoid alarming during set up diagnostics. If loop1 is held up because we are waiting for a user response, no warning is generated. This is accomplished by the user setting the resetLoop1TimerBool flag in the Fdr library. The flag is read and then cleared during this loop1() cycle.

If verboseQ is true, we print max cycle time each time it increases.
*************************************************/
{	
	if ((millis() - StartForTimeStampULong)< 30000)
	{
		LastLoop1TimeStampMS_ULong = millis();
		return;//don't monitor cycle time for 30 seconds to give time for initial code to run. StartForTimeStampULong is defined in Fdr.h and is valid for both cores because it starts at 0 at power up when both cores start.
	}
	
	
					#ifdef  LcT
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print("hasFDRoneTimeRunNotRunYetQ = ");
					Serial.println(hasFDRoneTimeRunNotRunYetQ);
					#endif	
	
	if(hasFDRoneTimeRunNotRunYetQ)
	{//if FDRoneTimeRun() has not completed, reset loop1() timer
		LastLoop1TimeStampMS_ULong = millis();//keep resetting this on each cycle of loop1() until 30 seconds
		
					#ifdef  LcT
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif
					
		
		return;
	}else
	{//FDRoneTimeRun() has completed so start monitoring
		if(resetLoop1TimerBool)
		{//Some user has disabled the loop1() timer for this cycle so we don't generate a software warning if the loop1() cycle time is too long. Then we enable loop1() timer.
			LastLoop1TimeStampMS_ULong = millis();//reset loop timer 
			resetLoop1TimerBool = false;//enable loop1() timer that the user inhibited
			
					#ifdef  LcT
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif		
			
			return;
		}
		//then the loop1() timer is active so continue to monitor the loop1() cycle time
		Loop1CycleTimeMS_ULong = millis() -  LastLoop1TimeStampMS_ULong;
		
		if(Loop1CycleTimeMS_ULong > maximumLoop1CycleTimeMS_UInt)
		{
			maximumLoop1CycleTimeMS_UInt = Loop1CycleTimeMS_ULong;	
			
			if(verboseQ)
			{				
				Serial.println();
				Serial.print("Maximum Core 1 cycle time is ");
				Serial.print(maximumLoop1CycleTimeMS_UInt);
				Serial.println(" ms.");
			}
		}
		
		if(Loop1CycleTimeMS_ULong > Loop1CycleTimeLimitMS_UInt)
		{
			Serial.println();
			Serial.println("	Excessive core 1 loop cycle time.");
			Serial.println();
			WriteControlBlock(ErrorCodeAddressByte, 19);//Compiler doesn't like defining loop1TimeMoreThanLimitByte in .h and in Iridium so I just used 19.
		}		
		LastLoop1TimeStampMS_ULong = millis();//reset loop() timer
		
					#ifdef  LcT
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					#endif		
		
	}
}//end of Loop1CycleTimeMonitor()

void Fdr::seizeExternalLEDcontrol() 
{

	externalLEDcontrolBool = onBool;
					
}

void Fdr::releaseExternalLEDcontrol()
{
	externalLEDcontrolBool = offBool;	
}	

void Fdr::externalLED(byte intensityByte)
{
	if(externalLEDcontrolBool == offBool)return;
	
	analogWrite(ExternalLED,intensityByte);
	
						#ifdef externalLEDverbose
						//if(intensityByte == 255)
						//{
							//Serial.println("Core 0 has the bus.");
						//}
						
						if(intensityByte == 32)
						{
							Serial.println("Core 1 has the bus.");
						}	
						#endif	
	
}

void Fdr::timeSlotGenerator(unsigned long intervalULong)
{	//each time slot is intervalULong microseconds long. Just return if the intervals is 0 to prevent division by zero. No error generated.
	if(intervalULong == 0)return;//prevent division by 0
	byte timeByte = byte(micros()/2000) & 0b00000011;//1 ms per TS  was 1000
	
	//define the timeslots and the guard band
	if(timeByte == 0)
	{
		timeSlot0Bool = true;
		timeSlot1Bool = false;
		//Serial.println(" TS0 ");//diag
		return;
	}

	if(timeByte == 2)
	{
		timeSlot0Bool = false;
		timeSlot1Bool = true;
		//Serial.println(" TS1 ");//diag
		return;
	}

	if((timeByte == 1) || (timeByte == 3))//guard band
	{
		timeSlot0Bool = false;
		timeSlot1Bool = false;
		//Serial.println(" GB ");//diag
		return;		
	}
}//end of time slot generator

void Fdr::logBusState(byte busOwnerByte, byte userByte)
{
	//record time and  which core is linked to bus 1 until file is full. 0 is core 0, 1 is core 1, 2 is free
	if(disableLogFunctionBool)return;//set disableLogFunctionBool in Fdr.h
	if(logFileIndexByte > logFileDepthByte -1)logFileIndexByte = 0;//When we have filled the log file, we circle back and start populating from the beginning to form a circular log with first in, first out.
	
												#ifdef logTest 
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												Serial.print(F("millis() = "));
												Serial.println(millis());
												#endif	
	
	busUseTimeULong[logFileIndexByte] = micros()/1000;
	busUseStateByte[logFileIndexByte] = busOwnerByte;
	logRunOnCoreByte[logFileIndexByte] = rp2040.cpuid();
	logUserByte[logFileIndexByte] = userByte;
	logFileIndexByte++;//advance to the next record
}

void Fdr::dumpBus1LogFile()
{//this log consists of four arrays. Hold time is derived from the core time array
	if(disableLogFunctionBool)return;//set disableLogFunctionBool in Fdr.h

												#ifdef logRawDump
												Serial.println("Raw dump of log file:");
												for(byte i = 0;i<16;i++)
												{
													Serial.print(logRunOnCoreByte[i]);
													Serial.print(" ");
													Serial.print(busUseStateByte[i]);
													Serial.print(" ");
													Serial.print(logUserByte[i]);
													Serial.print(" ");
													Serial.println(busUseTimeULong[i]);
												}
												#endif
									

		Serial.println("function     bus        user        core        Hold");
		Serial.println("on core    linked to    name        time,ms	    time,ms");
		Serial.println("--------   ---------	-------		-------		-------");

/*********************************************************************
logFileIndexByte points to the last valid entry in the log,. 
logFileDepthByte is the length of the log. 

We start dumping from (logFileIndexByte - logFileDepthByte) and print for logFileDepthByte entries with the count held in outerLoopCountInt.

outerLoopCountInt which can be a negative number is translated into the index needed for the files using these rules: 

	if outerLoopCountInt is negative, indexByte is the sum of outerLoopCountInt and logFileDepthByte
	if outerLoopCountInt is 0, indexByte is 0
	If outerLoopCountInt is greater than logFileDepthByte minus 1, indexByte equals outerLoopCountInt - logFileDepthByte


For example, if logFileIndexByte = 16 and logFileDepthByte = 16, we start dumping from 16 - 16 =  0 and dump until we get to 15. If logFileIndexByte = 8, 8 - 16 = -8. This means we wrapped so must add 16 to get back into the range of the log. So we start at 8. Then we print from 8 to 15 which is the top of all arrays. The next entry, 16, sends us back at 0. We then count from 0 to 7 and we have printed all 16 entries.
*************************************************************************/

	byte indexByte = 0;//used to index each array
	
	int startIndexInt = int(logFileIndexByte)  - int(logFileDepthByte);
	int endIndexInt = int(logFileIndexByte);
	byte initialIndexValueByte = 0;
	//translate initialIndexValueByte into an index for the arrays: initialIndexValueByte 
	if(startIndexInt < 0) initialIndexValueByte = byte(startIndexInt + int(logFileDepthByte));
	if (startIndexInt == 0) initialIndexValueByte = 0;
	if(startIndexInt > int(logFileDepthByte - 1)) initialIndexValueByte = byte(startIndexInt - int(logFileDepthByte));
	
	
												#ifdef logTest 
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));	
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												Serial.print(F("startIndexInt = "));
												Serial.println(startIndexInt);
												
												Serial.print(F("endIndexInt = "));
												Serial.println(endIndexInt);
												
												Serial.print(F("initialIndexValueByte = "));
												Serial.println(initialIndexValueByte);
												#endif 	
		
	
	for(int outerLoopCountInt = startIndexInt; outerLoopCountInt < endIndexInt; outerLoopCountInt++)
	{
		//convert outerLoopCountInt to indexByte
		if(outerLoopCountInt < 0) indexByte = byte(outerLoopCountInt + int(logFileDepthByte));
		if (outerLoopCountInt == 0) indexByte = 0;
		if(outerLoopCountInt > int(logFileDepthByte - 1)) indexByte = byte(outerLoopCountInt - int(logFileDepthByte));
		if((outerLoopCountInt >= 0) && (outerLoopCountInt < int(logFileDepthByte)))indexByte = byte(outerLoopCountInt);

	
												#ifdef logTest 
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												Serial.print(F("outerLoopCountInt = "));
												Serial.println(outerLoopCountInt);
												Serial.print(F("indexByte = "));
												Serial.println(indexByte);
												#endif 
			

		Serial.print("   ");
		Serial.print(logRunOnCoreByte[indexByte]);//print core that ran the code
		Serial.print("           ");

		if(busUseStateByte[indexByte] == neitherByte)
		{
			Serial.print("X");
		}else
		{
			Serial.print(busUseStateByte[indexByte]);//print core linked to bus
		}
		
		Serial.print("         ");	
		
		if(logUserByte[indexByte] == 99)
		{
			Serial.print("  none   ");
		}else
		{
		userNumberToEnglish(logUserByte[indexByte]);//print user in English
		}
		Serial.print("	");	
												#ifdef logTest 
												Serial.print("core ");
												Serial.print(rp2040.cpuid());
												Serial.print(" ");
												Serial.print(__FUNCTION__);
												Serial.print(F("(): "));		
												Serial.print(__LINE__);
												Serial.print(F(".    Time Stamp  "));
												Serial.print (millis() - StartForTimeStampULong);
												Serial.println(F(" ms"));
												Serial.print(F("busUseTimeULong[indexByte] = "));
												Serial.println(busUseTimeULong[indexByte]);
												Serial.print(F("busUseTimeULong[initialIndexValueByte] = "));
												Serial.println(busUseTimeULong[initialIndexValueByte]);
												#endif 
		
		
		Serial.print(busUseTimeULong[indexByte] - busUseTimeULong[initialIndexValueByte]);//print time since first entry to keep the time a managable size		
		Serial.print("	");		
		if((busUseStateByte[indexByte] != neitherByte) || (outerLoopCountInt == startIndexInt))
		{//lapse time can't start from the bus being occupied or as first entry
			Serial.println("--");
		}else 
		{//we are starting from bus being released so calc lapse time
			if(indexByte != 0)
			{
			Serial.println(busUseTimeULong[indexByte] - busUseTimeULong[indexByte - 1]);//print out the lapse time bus was held
			}else
			{//if indexByte = 0, previous entry is 15
				Serial.println(busUseTimeULong[0] - busUseTimeULong[15]);
			}
		}
	}//bottom of outer loop
}
	
void Fdr::calculateAndPrintPort6AverageSamplePeriod()
	{
		if(ReadControlBlock(ProgramStateAddressByte) > InFlightWithPowerDisruptionByte)return;//if not in flight, just return because the reading doesn't matter
		
		unsigned long sumOfPeriodsUsULong = 0;
		unsigned long maximumPeriodUsULong = 0;
		for(byte i = 0;i < 32;i++)
		{
							//Serial.println(port6PeriodLogUsULong[i]);//diag
			
			if(port6PeriodLogUsULong[i] > maximumPeriodUsULong)maximumPeriodUsULong = port6PeriodLogUsULong[i];//look for max value
			sumOfPeriodsUsULong = sumOfPeriodsUsULong + port6PeriodLogUsULong[i];//add up all values in preparaton for calculating the average
		}
		unsigned long averagePeriodMsULong = (sumOfPeriodsUsULong/32000) + 0.5;
		
					#ifdef loopTimerDetails
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));		
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));	
					Serial.print(millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print("sumOfPeriodsUsULong = ");
					Serial.println(sumOfPeriodsUsULong);
					#endif
					
		
		Serial.println();
		Serial.print("The time between readings of port 6 is ");
		Serial.print(averagePeriodMsULong);// divide by 10 to get average and divided by 1000 to convert from microseconds to milliseconds. Then convert to float and print after rounding.
		Serial.print(" +/- ");
		Serial.print((maximumPeriodUsULong/1000) - averagePeriodMsULong);
		Serial.print(" ms.");
		Serial.println();
	}
	
void Fdr::testDualVariable()
{//this tests the idea of a variable, dualVariableULong[], that has a unique address as a function of which core exexutes the function
	dualVariableULong[rp2040.cpuid()] = micros();
	
	byte otherCoreByte = 0;
	if(rp2040.cpuid() == 0)otherCoreByte = 1;
	float deltaFloatMsFloat = float(dualVariableULong[rp2040.cpuid()] - dualVariableULong[otherCoreByte])/1000;	
	if(deltaFloatMsFloat > 0.00)return;//don't bother to print if more than 0.1 ms apart
	
	//prvent both cores from printing at the same time

	while(1)//wait until we arrive at our timeslot
	{
		timeSlotGenerator(1);//sets timeSlot0Bool and timeSlot1Bool. 1 us per timeslot
		if(timeSlot0Bool && (rp2040.cpuid() == 0))break;		
		if(timeSlot1Bool && (rp2040.cpuid() == 1))break;
	}//must be in the wrong timeslot or in the guardband so try again
	Serial.print("core ");
	Serial.print(rp2040.cpuid());
	Serial.print(" is running and core ");
	Serial.print(otherCoreByte);
	Serial.print(" ran ");
	Serial.print(deltaFloatMsFloat,3);
	Serial.println(" milliseconds ago.");
}

void Fdr::sequentialFillMemory(bool PROMBool)
{//fills all of the data bytes with a number equal to the LSB from its address
	Serial.println("About to fill all of EEPROM and SEEPROM with sequential numbers.");
	for(long eeAddressLong = StartOfUsableEEPROM_Long+16;eeAddressLong < 131072;eeAddressLong++)
	{
		if(PROMBool)
		{//this means EEPROM
			UnprotectedWriteEEPROM(eeAddressLong, byte(eeAddressLong));
		}else
		{
			WriteSEEPROM(eeAddressLong, byte(eeAddressLong));
		}
		if(eeAddressLong % 256 == 0)Serial.print(".");//so we know it is working
	}
}	