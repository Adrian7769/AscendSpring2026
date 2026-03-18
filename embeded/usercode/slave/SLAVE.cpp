char FDRinoVersionChar[] = "4.1.5";//if you modify this code, update the version number.

//#define subus
//#define neoTime
//#define NeoTest
//#define loopTimerTest
//#define extLEDexperiment


#include <Fdr.h>
Fdr fdr; //I am defining Fdr as type fdr (yup, confusing)
#include <Wire.h>
#include <Arduino.h>
 
/**************************************
				Second Semester Modems
***************************************
The near modem is the one in the payload. The far modem is who we are talking with via satellite.
***************************************
	Available functions contained in the library.
*************************************************
          Second Semester SEEPROM AND EEPROM Drivers
***************************************************
    fdr.WriteSEEPROM(long eeAddress, byte data);
    fdr.ReadSEEPROM(long eeaddress);
	fdr.ReadEEPROM(long eeaddress);
**************************************************
				UTILITIES
***************************************************
fdr.changeBytesToUnsignedLong(byte b3, byte b2, byte b1, byte b0)

where b0 is the LSB and function returns an unsigned long formed from the 4 bytes sent to it

Student Control of the external LED:

	* fdr.seizeExternalLEDcontrol()
	* fdr.releaseExternalLEDcontrol()
	* fdr.externalLED(byte intensityByte)  with stateBool  
	  equal to onBool or offBool
	
built in choices for intensityByte are
	
	* ExternalLED_OnByte  gives full brightness
	* ExternalLED_OffByte makes it dark but still under student's control
	* ExternalLED_dimByte makes it dim
*************************************************
	Second Semester EEPROM Flags You Might Find Useful
*************************************************/
bool EEPROMjustOutputtedBool = false;//flag is set true after EEPROM is outputted and can be cleared by students
bool EEPROMjustClearedBool = false;//flag is set true after EEPROM is cleared and can be cleared by students
bool recordingDataBool = false;//flag is set true when EEPROM is being used to record data and false when we are not recording.
/*************************************************
                 Second Semester GNSS
***************************************************
    When you need the GPS data, read the NavigationDataByte[16] array for the newest information. It is updated automatically.
***************************************************
              Second Semester I2C Sensors
**************************************************/		
const byte successByte = 2;
const byte failedByte = 3;
const byte timedOutByte = 4;
const byte notReadyYetByte = 5;	 
const byte idleStateByte = 0;

const byte I2CcodeBlockA_Byte = 1;
const byte I2CcodeBlockB_Byte = 2;
const byte I2CcodeBlockC_Byte = 3;
const byte I2CcodeBlockD_Byte = 4;
const byte I2CcodeBlockE_Byte = 5;
const byte I2CcodeBlockF_Byte = 6;

const byte Si7021codeBlockA_Byte = 1;
const byte Si7021codeBlockB_Byte = 2;
const byte Si7021codeBlockC_Byte = 3;
const byte Si7021codeBlockD_Byte = 4;
const byte Si7021codeBlockE_Byte = 5;
const byte Si7021codeBlockF_Byte = 6;
//define as many codeBlocks as you need
const byte delayForByte = 255;
//model state machine variables
byte I2CstateByte = idleStateByte;
unsigned long I2CdelayStartTimeULong  = millis();
unsigned long I2CdelayULong = 0;
byte I2CReturnToByte = 0;//is set in I2CdelayForAndThenReturnTo()
byte I2CreceivedDataByte[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//Si7021 state machine variables
byte Si7021StateByte = idleStateByte;
unsigned long Si7021startTimeULong = millis();
unsigned long Si7021delayStartTimeULong = millis();
unsigned long Si7021delayULong = 0;  
byte Si7021ReturnToByte = 0;//is set in Si7021delayForAndThenReturnTo()
byte Si7021AddressByte = 0x40;
byte Si7021measureTempByte = 0xF3;
byte Si7021receivedDataByte[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//bytes 0-15 are data while 16 is the error code
byte iq = 0;//diag
bool flipBool = true;//diag
/***************************************************
                 Second Semester MODEM
***************************************************
    Iridium(unsigned int ModemCommandInputUInt)
*************************************************  

***********************************************
			Second Semester MODEM COMMANDS
***********************************************/
#define PerformTransmitUInt 2U
#define getReceivedDataUInt 3U
#define setLoopAroundUInt   6U
#define clearLoopAroundUint  7U
#define PingUInt 0U
#define versionQUInt 9U
/************************************************
S U C C E S S  R E T U R N S
************************************************/
const unsigned int PingThroughMPM_AndModemSuccessUInt = 400;
const unsigned int ModemReadyForUseUInt = 401;
const unsigned int TransmitSuccessfulAndNoReceiveUInt = 402;
const unsigned int TransmitAndReceiveSuccessfulUInt = 403;
const unsigned int TransmitAndReceiveSuccessfulPlusReceivePendingUInt = 404;
const unsigned int dataLoopAroundEnabledUInt = 405;
const unsigned int dataLooopAroundDisabledUInt = 406;


bool equippedBool = true;
bool unequippedBool = false;
								#ifdef gpsTest
								unsigned long gpsLastRanULong = millis();
								#endif
/******************************************
			FEATURE CONTROL
*******************************************
Set any to true to enable its functionality and to false to disable it.
*******************************************/
bool GNSS_Bool = equippedBool; //if a GPS is connected, set to equippedBool. Otherwise, say unequippedBool.
bool ModemBool = unequippedBool; //If an Iridium modem is connected, set to equippedBool. Otherwise, say unequippedBool.
/******************************************
	Modem Provisioning
********************************************
nearURBsn and farURBsn where URBsn means User RockBlock serial number. 

The near modem is the modem in your payload. The far modem is the one you want to pass data. 
***********************************************
	Available Modems

UserRBsn	RBsn			IMEI

	0 		13301		300234066438070		
	1		218642		300534065390120		
	2		13298		300234066436090		
	3		218641		300534065396130		

there is also a spare modem that has not been provisioned. 
These numbers are stored in the Modem's Pro Micro code, Iridium.ino.
************************************************/
byte nearURBsnByte = 0;// was 0 replace number with the UserRBsn you will fly
byte farURBsnByte = 3;//replace number with the UserRBsn you want to send data to

/***************************EEPROM_C***************
			USER LIBRARY VARIABLES
*******************************************/
byte NavigationDataByte[16];
byte IridiumReceivedDataByte[45] = {1};
byte IridiumTransmitDataByte[45] = {8};//initialized to a unique nonzero value had volatile
/*******************************************/

//timers and flags
bool onBool = true;
bool offBool = false;
bool justClearedEEPROM = false;
unsigned long lastLoopDurationULong = 1000;//initial estimate
unsigned long lastLoopTimeULong = millis()-1200;
bool modemTransactionActiveBool = false;
unsigned long loopDelayULong = 500;//starting point
unsigned long currentLoopDurationULong = 0;
unsigned long changeInLoopDurationULong = 0;
unsigned long StartForTimeStampULong = millis();//for diag prints
unsigned long StartForGPSTaskULong = millis();
unsigned long StartForModemTaskULong = millis();
unsigned long powerUpStartTimeULong = millis();
unsigned int modemTimerUInt = millis();

byte LEDflashInfoByte[3];//contains system info from library plus element [2] is 0 if diagLEDmode is not defined in Fdr.cpp and is a 1 if it is
bool AutomaticVideoRecordAtPowerUp = false; //We can reuse this in pointer array

bool printOnceBool = true;//diag
byte ExternalLED_OnByte = 255;
byte ExternalLED_OffByte = 0;
byte ExternalLED_dimByte = 32;//emperically found

//this is how I enable diagnostic code:

//#define btDebug
//#define GNSSTestData
//#define testDataArray
//#define loopTimerDetails
//#define siDriver
//#define SiTest
//#define quiicTest
//#define test 
//#define diagprint
//#define nearFarTest
//#define looparoundTest
//#define pingTest
//#define setupTest
//#define dumbDelayForFourZeroThree
//#define transmitTest
//#define gpsTest
//#define gpsFullArray 
//#define gpsJustTime
//#define timeTest
//#define I2CdiagPrint
//#define stackTest
//#define receiveTest
//#define toneTest
//#define SEEPROMflagTest
//#define OneTimeRunDiag
//#define GPScommandTest
//#define extraLoopTime
//#define GPSrateChange
//#define bytesToLong
//#define testOfReadEEPROM
//#define UARTtxTest
//#define testFunction
//#define ADCtest
//#define heartbeatTest
//#define noKB
//#define menuQ
//#define frozen
//#define comVar
//#define ADCdeviceTest
//#define ADCport0Test
//#define testTranslate

// *******a sample of diagnostic code***************

//Here is a sample of diagnostic code:	
				
						#ifdef enableVariable 
						Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.print(F("variableToPrtintOut = "));
						Serial.println(variableToPrtintOut);
						#endif 
						
//Indent the block of code enough so it is not distracting but leave it in place. You will likely need to turn it on later
/*************************************************/			

char filePathChar[] = __FILE__;//define a string called filePathChar[] that contains the entire path to this .ino program

bool evenQbool = false;
bool MPMoneTimeRunBool = true; //used to gate modem setup  defined in Iridium.ino
//byte I2Cbus1OwnerByte = 0;
const unsigned long idleUInt = 311;
const unsigned long FailureRangeMinUInt = 1;
const unsigned long FailureRangeMaxUInt = 299;
const unsigned long StatusRangeMinUInt = 300;
const unsigned long StatusRangeMaxUInt = 399;
const unsigned long SuccessRangeMinUInt = 400;
const unsigned long SuccessRangeMaxUInt = 499;	
const unsigned int ModemFailedAtSetupUInt = 281;
//unsigned int Iridium(statusUInt); //defined in Iridium.ino
						#ifdef GPScommandTest
							byte setAIR1[13] = {0xb5,0x62,0x06,0x8a,0x05,0x00,0x20,0x11,0x00,0x1c,0x06,0xE8,0x36};
							byte checkAIR1[12] = {0xb5,0x62,0x06,0x8b,0x04,0x00,0x20,0x11,0x00,0x1c,0xe2,0x4F};
							byte alphaTest[3] = {0x61, 0x62, 0x63};
						#endif
const unsigned int statusUInt = 8;	

volatile byte core0toCore1Byte = 0;//used by heartbeat
volatile byte core1toCore0Byte = 0;//used by heartbeat
volatile byte core0DataByte = 0;//used by heartbeat

volatile unsigned int ModemCommandUInt = 100;//10 is the value of NoActiveModemCommandUInt which is set by Iridium.ino when a command has been completed. I initialize to this value to be safe and hope it doesn't cause confusion.

//AHT20 I2C variables/function declarations
const byte AHT20_ADDRESS = 0x38;  // I2C address for AHT20.
const byte AHT20_measurementTrigger = 0xAC;  // Measurement command This is the measurement trigger instruction.
const byte AHT20_measurementSettings = 0x33;  // This configures internal measurement settings.
const byte AHT20_placeholder = 0x00;  // This is a required placeholder byte.

const byte AHT20codeBlockA_Byte = 1;  // this is where we tell the sensor to start measuring the temp and hum.
const byte AHT20codeBlockB_Byte = 2;  // this is when we are waiting for the sensor to take the measurements. (delay 80ms)
const byte AHT20codeBlockC_Byte = 3;  // This is were we request the sensor for its Temp and hum data.
const byte AHT20codeBlockD_Byte = 4;  // This is where if we got all the bytes we will read them and put them in an array to send to seeprom.
const byte AHT20codeBlockE_Byte = 5;  // This is where we wait a little bit longer if we cant read the bytes yet.

byte AHT20StateByte = idleStateByte;  // this is what is going to say what state we are going to be in and going to. we start off in idle because in the beginning before we go to block A-E we arnt doing anything.
unsigned long AHT20startTimeULong = millis();  //This is where we store "millis()" at its current time for a timer.
unsigned long AHT20delayStartTimeULong = millis();
unsigned long AHT20delayULong = 0;
byte AHT20ReturnToByte = 0; // After the delay we would go to the next state we assaigned this varible to be.

byte AHT20data[6];  // Byte 0 → Status, Byte 1 → Humidity MSB, Byte 2 → Humidity middle, Byte 3 → Half humidity + half temperature, Byte 4 → Temperature middle, Byte 5 → Temperature LSB,
unsigned long AHT20rawHum20bit  = 0;  // This is where we will have temp by itself because it orginally was with hum.
unsigned long AHT20rawTemp20bit = 0;  // This is where we will have hum by itself because it orginally was with temp.
byte AHT20humBytes[3];  // This is where we will have temp by itself because it orginally was with hum.
byte AHT20tempBytes[3]; // This is where we will have hum by itself because it orginally was with temp.
byte AHT20record[7];  // [0..2] = hum, [3..5] = temp, [6] = status

// Converted value variables.
float AHT20humidityRH = 0.0f;
float AHT20tempC = 0.0f;
float AHT20tempF = 0.0f;

byte AHT20statusByte = notReadyYetByte;  // The current status we are in. For example somthing was successful of somthing failed.

void AHT20delayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte);  //This starts a non-blocking delay for how long we want and when the delay is finished go to the next place we want.
void AHT20byteSplitter();  // This is where we are going to extract the temp and hum bytes.



/*******************************************/
intptr_t PointerArray[12];
/******************************************
		SET UP LIBRARY ENVIRONMENT
******************************************/		
Fdr Fdr; //the second Fdr is the Constructor for the class Fdr which is the first Fdr on the line 

void setup()
{	
	Serial.begin(57600);//UART connect to the terminal emulator via the USB C cable
	//delay(100);//give time for UART to settle
	Serial2.begin(38400);//UART initialize baud rate
	//set up I2C bus 0 which is Quiic and bus 1
	Wire.setSDA(16);//I2C for Quiic connector
	Wire.setSCL(17);//I2C for Quiic connector
	Wire.begin(); //set up communications beween FDR to Quiic
	Wire.setClock(400000); //set clock rate for EEPROMs
	
	Wire1.setSDA(2);//I2C bus1
	Wire1.setSCL(3);//I2C bus1
	Wire1.begin(); //set up communications between FDR to EEPROM
	Wire1.setClock(40000);

	FillPointerArray();//collect address of all variables needed by the Fdr library
	fdr.Pointers(PointerArray);//send pointer array to the library
		
	introCountdown();//wait up to 10 seconds for TeraTerm to come up
	showVersions();


	/*
  ws2812fx.init();//confirmed
  ws2812fx.setBrightness(128);//confirmed
  //ws2812fx.setSpeed(2000);//confirmed
  ws2812fx.setMode(0);//confirmed for static color
  ws2812fx.setColor(GREEN);//confirmed
  ws2812fx.start();//confirmed
  */
 
}


//This exists so I can pass a byte array between methods, as C++ does not like doing this normally
struct byteArrayReturn {
	byte dataToSend[19] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x7A};
};

//this crc lookup table was generated at https://www.crccalc.com/ using the Bluetooth crc-8 option,
//This specifies a polynomial value of A7, reflected input and output, and an initial crc value and final xor value of 0
byte crcLookup[256] = {
  0x00, 0x6B, 0xD6, 0xBD, 0x67, 0x0C, 0xB1, 0xDA, 0xCE, 0xA5, 0x18, 0x73, 0xA9, 0xC2, 0x7F, 0x14,
  0x57, 0x3C, 0x81, 0xEA, 0x30, 0x5B, 0xE6, 0x8D, 0x99, 0xF2, 0x4F, 0x24, 0xFE, 0x95, 0x28, 0x43,
  0xAE, 0xC5, 0x78, 0x13, 0xC9, 0xA2, 0x1F, 0x74, 0x60, 0x0B, 0xB6, 0xDD, 0x07, 0x6C, 0xD1, 0xBA,
  0xF9, 0x92, 0x2F, 0x44, 0x9E, 0xF5, 0x48, 0x23, 0x37, 0x5C, 0xE1, 0x8A, 0x50, 0x3B, 0x86, 0xED,
  0x97, 0xFC, 0x41, 0x2A, 0xF0, 0x9B, 0x26, 0x4D, 0x59, 0x32, 0x8F, 0xE4, 0x3E, 0x55, 0xE8, 0x83,
  0xC0, 0xAB, 0x16, 0x7D, 0xA7, 0xCC, 0x71, 0x1A, 0x0E, 0x65, 0xD8, 0xB3, 0x69, 0x02, 0xBF, 0xD4,
  0x39, 0x52, 0xEF, 0x84, 0x5E, 0x35, 0x88, 0xE3, 0xF7, 0x9C, 0x21, 0x4A, 0x90, 0xFB, 0x46, 0x2D,
  0x6E, 0x05, 0xB8, 0xD3, 0x09, 0x62, 0xDF, 0xB4, 0xA0, 0xCB, 0x76, 0x1D, 0xC7, 0xAC, 0x11, 0x7A,
  0xE5, 0x8E, 0x33, 0x58, 0x82, 0xE9, 0x54, 0x3F, 0x2B, 0x40, 0xFD, 0x96, 0x4C, 0x27, 0x9A, 0xF1,
  0xB2, 0xD9, 0x64, 0x0F, 0xD5, 0xBE, 0x03, 0x68, 0x7C, 0x17, 0xAA, 0xC1, 0x1B, 0x70, 0xCD, 0xA6,
  0x4B, 0x20, 0x9D, 0xF6, 0x2C, 0x47, 0xFA, 0x91, 0x85, 0xEE, 0x53, 0x38, 0xE2, 0x89, 0x34, 0x5F,
  0x1C, 0x77, 0xCA, 0xA1, 0x7B, 0x10, 0xAD, 0xC6, 0xD2, 0xB9, 0x04, 0x6F, 0xB5, 0xDE, 0x63, 0x08,
  0x72, 0x19, 0xA4, 0xCF, 0x15, 0x7E, 0xC3, 0xA8, 0xBC, 0xD7, 0x6A, 0x01, 0xDB, 0xB0, 0x0D, 0x66,
  0x25, 0x4E, 0xF3, 0x98, 0x42, 0x29, 0x94, 0xFF, 0xEB, 0x80, 0x3D, 0x56, 0x8C, 0xE7, 0x5A, 0x31,
  0xDC, 0xB7, 0x0A, 0x61, 0xBB, 0xD0, 0x6D, 0x06, 0x12, 0x79, 0xC4, 0xAF, 0x75, 0x1E, 0xA3, 0xC8,
  0x8B, 0xE0, 0x5D, 0x36, 0xEC, 0x87, 0x3A, 0x51, 0x45, 0x2E, 0x93, 0xF8, 0x22, 0x49, 0xF4, 0x9F
};


//Parameters are the struct that holds our data array, and length of the data in that array)
//If used to check the full array (including the CRC), it will evaluate to 0, otherwise it will be the same value as the last index
byte generateCRC(struct byteArrayReturn data, int len)
{
	byte poly = 0xA7; // This is the byte representation of the CRC-8 Bluetooth polynomial, provided by Wikipedia, as is the method to get it.
	byte crc = 0x00; // Most CRC algorithms use an initial value of 0

	//This loops through the data array (except for the last value) and sets the crc value to the inverted remainder of the bitwise xor operation of the current crc and the inverse of the current value
	//this is mostly handled through the use of a crc lookup table, in order to minimize the realtime it takes.
	for(int i = 0; i < len; i++){
		crc=crcLookup[crc^data.dataToSend[i]];
	}
	#ifdef btDebug
		Serial.print("Calculated CRC: ");
		Serial.println(crc, HEX);
	#endif
  return crc;
}

byteArrayReturn fillDataArrayToSend(byte command){
	byteArrayReturn ret;
#ifndef testDataArray
	#ifdef btDebug
		Serial.println("Filling Data Array");
	#endif
	//get the current active memory block fromo the control block
	unsigned long currentMemoryBlockULong = fdr.changeBytesToUnsignedLong(fdr.ReadEEPROM(6),fdr.ReadEEPROM(5),fdr.ReadEEPROM(4),fdr.ReadEEPROM(3)) - 16;
	#ifndef GNSSTestData
		//the first 16 bytes of data are the GNSS output
		for(int i = 0; i < sizeof(NavigationDataByte); ++i){
			ret.dataToSend[i] = NavigationDataByte[i];
		}
	#endif
	{
		int battRaw = fdr.ADCread(7);
		ret.dataToSend[16] = (byte)((battRaw >> 8) & 0xFF); // MSB
		ret.dataToSend[17] = (byte)( battRaw       & 0xFF); // LSB
	}
#endif
	//the last byte of the array is the CRC value, so the receiver can make sure all the data came through with the right values in the right order
	#ifdef btDebug
		Serial.println("Data Array Filled");
	#endif
	//generateCRC takes a byteArrayReturn struct (because passing actual arrays is weird) and the number of data bytes in order to generate a CRC value
	//This is the size - 1 because the last byte is for the CRC value itself.
	ret.dataToSend[18] = generateCRC(ret, sizeof(ret.dataToSend)-1);
	return ret;
}

void checkAndSendRequestedBT(){
	if(Serial2.available() > 0){ //check if team 2 has sent a command
		int command = Serial2.read();
			#ifdef btDebug
					String sentString;
					Serial.println("Received " + static_cast<String>(command));
			#endif
		//Upon receiving a command, immediately echo it back so the other payload knows we got it
		byteArrayReturn send;
		switch(command) {
			case -1:
				//This shouldn't be reachable, as this can only be the case if the receive buffer is empty when read
				Serial.println("Empty receive buffer, how did we get here?");
				break;
			case 1: //team 2's command to send available data
				send = fillDataArrayToSend(command);
				for(int i = 0; i < sizeof(send.dataToSend) / sizeof(send.dataToSend[0]); i++){
					Serial2.write(send.dataToSend[i]);
						#ifdef btDebug
							sentString += send.dataToSend[i];
							sentString += " ";
						#endif
				}
					#ifdef btDebug
						Serial.println("Sent " + sentString);
					#endif
				break;
			default:
				Serial.println("Not a valid command");
				break;
		}
	}
}

void updateI2C(){
	if(AHT20StateByte == idleStateByte){
		AHT20StateByte = AHT20codeBlockA_Byte;
	}
	return ;
}

void storeI2C(){
	for(int i = 0; i < 16; ++i){
				fdr.WriteSEEPROM(i, 0x00);
	}
	if(recordingDataBool){
		unsigned long currentMemoryBlockULong = fdr.changeBytesToUnsignedLong(fdr.ReadEEPROM(6),fdr.ReadEEPROM(5),fdr.ReadEEPROM(4),fdr.ReadEEPROM(3));
		if(currentMemoryBlockULong/16 % 2 == 0){
			for(int i = 0; i < sizeof(NavigationDataByte); ++i){
				fdr.WriteSEEPROM(currentMemoryBlockULong + i, NavigationDataByte[i]);
			}
			for(int i = sizeof(NavigationDataByte); i < 16; ++i){
				fdr.WriteSEEPROM(currentMemoryBlockULong + i, 0x00);
			}
		}
		else{
			for(int i = 0; i < sizeof(AHT20record); ++i){
				fdr.WriteSEEPROM(currentMemoryBlockULong + i, AHT20record[i]);
			}
			for(int i = sizeof(AHT20record); i < 16; ++i){
				fdr.WriteSEEPROM(currentMemoryBlockULong + i, 0x00);
			}
		}
	}
}

void loop()
{
	
		fdr.FDRoneTimeRun();//deals with FDR and GPS. When done, it sets MPMoneTimeRunBool true so core1 will run modem setup. It also determines if we are in simulator mode so this must run before any use is made of I2C bus
		
						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
	
	

						#ifdef ADCport0Test
							int countInt = 0;
							byte portByte = 0;
							while(1)
							{
								Serial.print("ADC port ");
								Serial.print(portByte);
								Serial.print(" reads ");
								countInt = fdr.ADCread(portByte);
								Serial.print(countInt);
								Serial.println(" counts");
								Serial.print("Resulting voltage is ");
								Serial.println(fdr.ADCconvertToVoltage(countInt),4); 
								delay(1000);
								portByte++;
								if(portByte > 7)portByte = 0;
							}
						#endif
					
						#ifdef frozen
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
	


						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
	
						#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
		

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
	
		MPMoneTimeRun();//sets up modem, if equipped and runs in core0. All other modem functions run in core1.

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
	
						#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
			
						#ifdef testFunction
							test33Function(42);//second variable was defined outside of any function.
						#endif

						#ifdef gpsTest
							GPStest();
						#endif

						#ifdef looparoundTest
							modemLoooparoundTest();	
						#endif					
			
		fdr.PutInLoop();//contains all Flight Data Recorder functionality

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
	
		//put some of your code hereS
		checkAndSendRequestedBT();
		updateI2C();
		storeI2C();

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
									
						#ifdef ADCdeviceTest
							while(1)
							{	
								Serial.println("read ADC port 0.");
								//Serial.print(fdr.ADCread(0));//it returns an int
								//Serial.println(" counts");
								Serial.print(fdr.ADCconvertToVoltage(fdr.ADCread(0)),4);
								Serial.println(" volts");
								delay(1000);
							}		
						#endif
	
						#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif							
							
						#ifdef receiveTest
						Serial.print("IridiumReceivedDataByte[4] = ");
						Serial.println(IridiumReceivedDataByte[4]);
						#endif

						#ifdef SEEPROMflagTest
							SEEPROMtest();
						#endif	
							
						#ifdef ADCtest
							unsigned int OffsetCount = 25;
							unsigned int ADCcount = 0;
							float portVoltage = 0;
							
							while(1)
							{
							ADCcount = analogRead(29) - OffsetCount;		
								Serial.print("ADCcount = ");
								Serial.println(ADCcount);
								portVoltage = 0.00322*float(ADCcount);
								Serial.print("port voltage = ");
								Serial.println(portVoltage);
								portVoltage = 3.108*portVoltage;
								Serial.print("battery voltage = ");
								Serial.println(portVoltage);	
								delay(1000);
							}

	
						#endif


						#ifdef UARTtxTest
						while(1)
						{
									Serial2.print(F("AT\r"));
									delay(1000);
						}
						#endif	

						#ifdef bytesToLong 
							byte b3 = 0x8F;
							byte b2 = 0xFF;
							byte b1 = 0;
							byte b0 = 0xFF;

							unsigned long resultLong = fdr.changeBytesToUnsignedLong(b3, b2, b1, b0);

							Serial.println("input bytes are ");
							Serial.print(b3);
							Serial.print(",");
							Serial.print(b2);
							Serial.print(",");
							Serial.print(b1);
							Serial.print(",");
							Serial.print(b0);
							Serial.print(" and resulting long is ");
							Serial.println(resultLong);
						#endif

						#ifdef testOfReadEEPROM
							//read the program state and print it out once
							byte programStateByte = fdr.ReadEEPROM(0);//it is at address 0 in the control block
							Serial.print("program state is ");
							Serial.println(programStateByte);
							delay(1000);
						#endif

						#ifdef loopTimerDetails
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
						
		fdr.Port6Peak();//call this function as often as possible to minimize time delay between readings of port 6

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
						
		//put the rest of your code here

						#ifdef loopTimerDetails
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
						
						#ifdef frozen
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print("core ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
						#endif
									
		core0Heartbeat();//works with core1 to monitor core 0
	
						#ifdef subus
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							for(long i = 0;i < 400000;++i)
							{
								digitalWrite(5, 0);
								digitalWrite(5, 1);		
							}
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));		
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));	
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));	
							//while(1);//stop running				
						#endif

						#ifdef neoTime
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
									
						
						#ifdef loopTimerTest
							Serial.print("1 - test loop1 timerQ output is ");
							Serial.println(fdr.resetLoop1TimerQ() );
							delay(2000);
							Serial.println("Clear loop1 timerQ.");
							fdr.clearResetLoop1TimerFlag();
							Serial.print("2 - test loop1 timerQ output is ");
							Serial.println(fdr.resetLoop1TimerQ() );
							Serial.println();
							delay(2000);
						#endif
								

						#ifdef CBtestScript
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
					
						#ifdef extLEDexperiment
							fdr.seizeExternalLEDcontrol();
							fdr.externalLED(ExternalLED_OnByte);
							delay(500);
							fdr.externalLED(ExternalLED_dimByte);
							delay(1000);
							fdr.externalLED(ExternalLED_OnByte);
							delay(500);
							fdr.externalLED(ExternalLED_OffByte);
							delay(500);	
						#endif
	
}//end of loop()




void FillPointerArray()
{
	/******************************************
				LIBRARY VARIABLES
	*******************************************/
	PointerArray[0] = reinterpret_cast< intptr_t >(&IridiumTransmitDataByte);
	PointerArray[1] = reinterpret_cast< intptr_t >(&IridiumReceivedDataByte);
	PointerArray[2] = reinterpret_cast< intptr_t >(&NavigationDataByte);
	PointerArray[3] = reinterpret_cast< intptr_t >(&GNSS_Bool);//element 3 now contains the address of the GNSS variable
	PointerArray[4] = reinterpret_cast< intptr_t >(&LEDflashInfoByte);
	//PointerArray[5] = reinterpret_cast< intptr_t >(&AutomaticVideoRecordAtPowerUp);
	PointerArray[6] = reinterpret_cast< intptr_t >(&filePathChar[0]);//put the starting address of the string filePath into the pointer array
	//PointerArray[7] = reinterpret_cast< intptr_t >(&I2Cbus1OwnerByte);
	PointerArray[8] = reinterpret_cast< intptr_t >(&EEPROMjustClearedBool); //
	//PointerArray[9] = reinterpret_cast< intptr_t >(&unsed variable);
	PointerArray[10] = reinterpret_cast< intptr_t >(&EEPROMjustOutputtedBool);
	PointerArray[11] = reinterpret_cast< intptr_t >(&recordingDataBool);
}

void printOneSpace()
{
	Serial.print(F(" "));
}


					#ifdef GPScommandTest
						void readGPSfor10Seconds()
						{
							unsigned int startUInt = millis();
							while((millis() - startUInt) < 10000)
							{
								while (Serial1.available())
										{
										  Serial.write(Serial1.read());
										}
							}
							Serial.println("done monitoring GNSS");
						}
					#endif

		
					#ifdef gpsFullArray
						Serial.print("time stamp for GPS, seconds: ");
						unsigned long deltaTimeULong = (millis() -powerUpStartTimeULong)/1000;
						Serial.println(deltaTimeULong);
						
						Serial.println(F("GPS Output, bytes 0-15"));
									   
						for(byte i=0;i<16;i++)
							{
								Serial.print(NavigationDataByte[i]);//print out array, one byte on a line. See GPS docs for layout of bytes
									Serial.print("|");
							}
						Serial.println();
						delay(1500);
					#endif

					#ifdef gpsJustTime
						bool PMQbool = true;
						if (NavigationDataByte[0] == 200){
						Serial.println("GPS OK but no lock yet.");	
						return;
						}
						if (NavigationDataByte[0] > 200){
						Serial.print("GPS failed with error ");	
						Serial.println(NavigationDataByte[0]);
						return;
						}
							
						int hoursInt = NavigationDataByte[0] - 7;
						if (hoursInt > 12)
							{
								hoursInt = hoursInt - 12;//change to AM/PM format
							}else{
								PMQbool = false;
							}
							
						Serial.print(hoursInt);
						Serial.print(":");
						Serial.print(NavigationDataByte[1]);
						Serial.print(":");
						Serial.print(NavigationDataByte[2]);
						if(PMQbool == true){
							Serial.println(" PM");
						}else{
							Serial.println(" AM");
						}
					#endif		


					#ifdef SEEPROMflagTest
						void SEEPROMtest()
							{
								if(EEPROMjustClearedBool)
								{
									delay(3000);
									Serial.println();
									Serial.println("************************cleared EEPROM flag is true");
									EEPROMjustClearedBool = false;
								}
								
								if(EEPROMjustOutputtedBool)
								{
									delay(3000);
									Serial.println();
									Serial.println("************************EEPROMjustOutputtedBool is true");
									EEPROMjustOutputtedBool = false;
								}
								
									if(recordingDataBool)
								{
									delay(3000);
									Serial.println();
									Serial.println("************************recordingDataBool is true");
								}else
								{
									delay(3000);
									Serial.println();
									Serial.println("************************recordingDataBool is false");
								}
									
							}
					#endif

					#ifdef looparoundTest
						void modemLoooparoundTest()
						{
							//top of test	
													#ifdef looparoundPrint
														Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
														Serial.print(F("(): "));		
														Serial.print(__LINE__);
														Serial.print(F(".    Time Stamp  "));	
														Serial.print (millis() - StartForTimeStampULong);
														Serial.println(F(" ms"));
													#endif

							//enable looparound	
								Iridium(setLoopAroundUInt);
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								delay(40);//give MPM time to work						
								Serial.print(F("modem command = "));
								Serial.println(setLoopAroundUInt);
								Serial.print(F("return code = "));
								Serial.println(Iridium(statusUInt));//since there are no failures for this command, I ignore return code
								
							//build test array to send to MPM
							
													#ifdef looparoundPrint
														Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
														Serial.print(F("(): "));		
														Serial.print(__LINE__);
														Serial.print(F(".    Time Stamp  "));	
														Serial.println (millis() - StartForTimeStampULong);
													#endif
													
								for(byte count = 0; count < 45;count++)
								{
									IridiumTransmitDataByte[count] = byte(millis());
									delay(1);//testpattern of numbers that should be different from the last time this was executed so we don't fool ourselves with old data looking like things worked
								}

									IridiumTransmitDataByte[0] = 0;
									IridiumTransmitDataByte[31] = 31;
									IridiumTransmitDataByte[44] = 44;//makes it easier to find first and second sub array ends.
								
									Serial.println(F("Transmitted data:"));
									for(byte count = 0; count < 45;count++)
									{
										Serial.print(IridiumTransmitDataByte[count]);
										Serial.print(" ");
									}
									Serial.println();
												
							//ready to transmit	
													#ifdef looparoundPrint
														Serial.println();
														Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
														Serial.print(F("(): "));		
														Serial.print(__LINE__);
														Serial.print(F(".    Time Stamp  "));	
														Serial.println (millis() - StartForTimeStampULong);
													#endif
								
								Iridium(PerformTransmitUInt);
								delay(40);//give MPM time to work
								
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));				
								Serial.print(F("modem command = "));
								Serial.println(PerformTransmitUInt);
								Serial.print(F("return code = "));
								Serial.println(Iridium(statusUInt));	
								
								//I got 323 SentPerformTransmitUInt						
								//I will wait until I see TransmitAndReceiveSuccessfulUInt
								while((Iridium(statusUInt) <= StatusRangeMaxUInt) && (Iridium(statusUInt) >= StatusRangeMinUInt))
								{//we are receiving status messages
										delay(100);//give time for MPM to work
										Serial.print("+");
										if(Iridium(statusUInt) == idleUInt)
										{
											//Iridium(statusUInt) = 450;//just in case we get into this case but is not a good solution. I force Iridium(statusUInt) out of the status range
											Serial.print("=");
										}
								}
								Serial.println();
							//final result now available
								if(Iridium(statusUInt) <= FailureRangeMaxUInt )
								{//we had a failure so try again later
									Serial.println("failure from PerformTransmitUInt");
									Serial.print("Iridium(statusUInt) is ");
									Serial.println(Iridium(statusUInt));
									goto bailOut;
								}
							//to get here, we had a success. Since we are in looparound, we have to get TransmitAndReceiveSuccessfulUInt
							
													#ifdef looparoundPrint
														Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
														Serial.print(F(": "));	
														Serial.print(__LINE__);
														Serial.print(F(".    Time Stamp  "));
														Serial.print(millis() - StartForTimeStampULong);
														Serial.println(F(" ms"));
														Serial.print(F("Iridium(statusUInt) = "));
														Serial.println(Iridium(statusUInt)); 
													#endif
													
								Iridium(getReceivedDataUInt);
								delay(40);//give MPM time to work
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F(": "));	
								Serial.print(__LINE__);
								Serial.print(F("modem command = "));
								Serial.println(getReceivedDataUInt);
								Serial.print(F("Iridium(statusUInt) = "));
								Serial.println(Iridium(statusUInt)); //I got 401
								if(Iridium(statusUInt) <= FailureRangeMaxUInt)
								{
							//we failed to get receive data. Try another session later.
									goto bailOut;
								}
								while((Iridium(statusUInt) <= StatusRangeMaxUInt) && (Iridium(statusUInt) >= StatusRangeMinUInt))
								{//we are receiving status messages
									delay(100);//give time for MPM to work
									Serial.print("&");
									if(Iridium(statusUInt) == idleUInt)
									{
										//Iridium(statusUInt) = 450;//just in case we get into this case but is not a good solution. I force Iridium(statusUInt) out of the status range 
										Serial.print("%");	
									}
								}						
											
								//I got ModemReadyForUseUInt 403 but was expecting data has been placed in receive array so go get it
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F("Iridium(statusUInt) = "));
								Serial.println(Iridium(statusUInt)); //just to confirm I got the success code I expected.	
							//read echoed data
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));	
														
								Serial.println(F("Echoed data:"));
								for(byte count = 0; count < 45;count++)
								{
									Serial.print(IridiumReceivedDataByte[count]);
								//testpattern should print out
									Serial.print(" ");
								}
								Serial.println();

							//turn off looparound
								Iridium(clearLoopAroundUint);
								delay(40);//give MPM time to work						
								//there are no failures for this command so I ignore return code
								Serial.print(F("modem command = "));
								Serial.println(clearLoopAroundUint);
								Serial.print(F(" 2 Iridium(statusUInt) = "));
								Serial.println(Iridium(statusUInt));
								
							//verify we are back to idle
							
							#ifdef looparoundPrint
								Serial.print(F("3 Iridium(statusUInt) = "));
								Serial.println(Iridium(statusUInt));
							#endif
							
							bailOut:;//sorry, I just had to use a goto....


						}//end of modemLoooparoundTest()
					#endif

					#ifdef modemTransmitTest
						void modemTransmitTest()
						{
							if(ModemBool && !modemTransactionActiveBool && (millis() - StartForModemTaskULong > 5000))
							{
								StartForModemTaskULong = millis();//reset modem task timer				
								//build test array to send to MPM
								Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
								Serial.print(F("(): "));		
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));	
								Serial.println (millis() - StartForTimeStampULong);
								byte count;
								if(evenQbool)
								{
									for(count = 0; count < 45;count++)
									{
										IridiumTransmitDataByte[count] = count;
									}
								}else
								{	
									for(count = 0; count < 45;count++)
									{
										IridiumTransmitDataByte[count] = 44 - count;
									}
								}		
								if(evenQbool) //toggle flag
									{
										evenQbool = false;
									}else
									{
										evenQbool = true;
									}
									
								IridiumTransmitDataByte[0] = 0;
								IridiumTransmitDataByte[31] = 31;
								IridiumTransmitDataByte[44] = 44;//makes it easier to find first and second sub array ends.
										
								Serial.println(F("Transmitted data:"));
								for(byte count = 0; count < 22;count++)
								{
									Serial.print(IridiumTransmitDataByte[count]);
									printOneSpace();
								}
								Serial.println();
								for(byte count = 23; count < 45;count++)
								{
									Serial.print(IridiumTransmitDataByte[count]);
									printOneSpace();
								}
								Serial.println();
							//ready to transmit	
								StartForTimeStampULong = millis();//set time to zero
								
								Iridium(PerformTransmitUInt);
								
								Serial.println(F("perform transmit modem command"));
								Serial.print(F("initial transmit return code = "));
								Serial.println(Iridium(statusUInt));
								modemTransactionActiveBool = true;
							}
							if(modemTransactionActiveBool)
							{		
								//Iridium(statusUInt);
								
								if(Iridium(statusUInt) == idleUInt)
								{
									modemTransactionActiveBool = false;//we are done
									Serial.println(F("MPM is now idle"));
								}else
								{
									if((Iridium(statusUInt) <= StatusRangeMaxUInt) && (Iridium(statusUInt) >= StatusRangeMinUInt))
									{//we are receiving status messages
										Serial.print(F("status return code = "));
										Serial.println(Iridium(statusUInt));

									}
								}
								
								if(Iridium(statusUInt) <= FailureRangeMaxUInt)
								{//we are receiving failure messages
									modemTransactionActiveBool = false;//we are done	
									Serial.print(F("failure return code = "));
									Serial.println(Iridium(statusUInt));
									
								}
								
								if((Iridium(statusUInt) <= SuccessRangeMaxUInt) && (Iridium(statusUInt) >= SuccessRangeMinUInt))
								{//we are success status messages
									Serial.print(F("success return code = "));
									Serial.println(Iridium(statusUInt));

								}
							}
						}//end of modemTransmitTest()
					#endif

					#ifdef PaulGPScode
						// Communicate with the u-blox MAX-M10S over Serial (UART)
						// Set the rate of the standard NMEA messages to zero
						// Poll individual NMEA messages


						void F(unsigned long timeout)
						{
							// Echo serial data from Serial1 to Serial until a timeout occurs
							
							unsigned long startTime = millis();
							while (millis() < (startTime + timeout))
							{
							while (Serial1.available())
							{
								Serial.write(Serial1.read());
							}
							}
							Serial.print("done echoing GNSS [924]");
						}

						void echoSerialTerminatorTimeout(byte terminator, unsigned long timeout)
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

						void nmeaWriteByteUpdateChecksum(byte *csum, byte b)
						{
						//he defined *csum which I believe is a pointer to csum. Note that *csum is updated but not returned so by using a pointer, the value, where ever it is, is correctly changed.
						//This is a simple XOR checksum
							// Send a single NMEA byte and update the checksum
							
							Serial1.write(b);//write sends a single hex value
							*csum = *csum ^ b;//XOR *csum with new byte
						}

						void nmeaWriteBytesUpdateChecksum(byte *csum, const char *b)
						{
						//this time, he is also passing a constant character *b. It appears that b is a string
							// Loop through NMEA text, send each byte and update checksum
							
							for (int i = 0; i < strlen(b); i++)//sequence through the string from the first to the last element.
							{
							nmeaWriteByteUpdateChecksum(csum, b[i]);
							//here is where I call the function defined on line 37. He passes csum but that field is defined as a pointer to *csum. He then passes b[i] where i is being advanced by the for loop on each pass so only a single character is passed here and it is a single byte. 
							}
						}

						void nmeaWriteChecksumCRLF(byte csum)
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

						void sendNMEA(const char *message)
						{
							// Send a NMEA message. Add the $,*,checksum,CR,LF

							byte csum = 0;

							Serial1.write('$'); // Dollar - not in checksum  
							nmeaWriteBytesUpdateChecksum(&csum, message); // Send the message
							Serial1.write('*'); // Asterix - not in checksum
							nmeaWriteChecksumCRLF(csum); // Write 2-byte ASCII checksum and CR, LF  
						}
					#endif

					#ifdef testFunction
						byte B = 42;//if I define it here and do not use in passed variables, will it work?
						void test33Function(byte A,byte B)
						{
							Serial.print("A= ");
							Serial.println(A);
								Serial.print("B= ");
							Serial.println(B);
							while(1);//stop
						}
					#endif

void core0Heartbeat()
{//echo a byte from core1 back to it

			#ifdef comVar
			if(printOnceBool)
			{
				printOnceBool = false;
				Serial.print("core ");
				Serial.print(rp2040.cpuid()); 
				Serial.print(" ");
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print(F("core1toCore0Byte = "));
				Serial.println(core1toCore0Byte);
			}
				#endif

	core0toCore1Byte = core1toCore0Byte;
}

void introCountdown()
{
	unsigned long startCountdownTimeULong = millis();
	while(millis() - startCountdownTimeULong < 10000 && !Serial.dtr());//wait up to 10 seconds for Serail Monitor to be recognized.
}
	
void showVersions()
{
	Serial.println();	
	Serial.println("  Glendale Community College Flight Data Recorder");
	Serial.println(F("                  by R.G. Sparber"));
	Serial.println();
	Serial.println("  Version Control:");
	Serial.println("\tComponent\tVersion\tmodified");
	Serial.println("\t=========\t=======\t========");
	
	//Iridium(versionQUInt);//tells core 1 to print version of Iridium.ino plus date and time of last modified
	//delay(50);//give time for Iridium() to finish printing
	Serial.print(F("\tFDR.ino\t\t"));
	Serial.print(FDRinoVersionChar);
	Serial.print("\t");
	Serial.print(__DATE__);//date and time FDR.ino was downloaded from Canvas or modified
	Serial.print(" at ");
	Serial.println(__TIME__);
	printVersionOfIridium();//goes on Version Control list
}

void AHT20byteSplitter() {
	AHT20rawHum20bit = ((unsigned long)AHT20data[1] << 12) | ((unsigned long)AHT20data[2] <<  4) | ((unsigned long)(AHT20data[3] >> 4));

	AHT20rawTemp20bit = ((unsigned long)(AHT20data[3] & 0x0F) << 16) | ((unsigned long)AHT20data[4] << 8) | ((unsigned long)AHT20data[5]);

	AHT20humBytes[0]  = (byte)((AHT20rawHum20bit >> 16) & 0x0F);
	AHT20humBytes[1]  = (byte)((AHT20rawHum20bit >> 8) & 0xFF);
	AHT20humBytes[2]  = (byte)(AHT20rawHum20bit & 0xFF);

	AHT20tempBytes[0] = (byte)((AHT20rawTemp20bit >> 16) & 0x0F);
	AHT20tempBytes[1] = (byte)((AHT20rawTemp20bit >>  8) & 0xFF);
	AHT20tempBytes[2] = (byte)(AHT20rawTemp20bit & 0xFF);

}

void I2CstateMachines()
{
/*
	/************************************************************************
	We have one state machine for each Quiic client. Run only one machine at a time to avoid collisions on the Quiic bus. You can do this by having the last state of one machine point to the first state of the next machine.
	**************************************************************************

	//state controller - it sequences through each state machine. 

	//Each time the Si7021 state machine is idle, print out last data and start new cycle.

						#ifdef quiicTest
							if(iq < 100)for(iq = 0;iq<100;iq++)//stops code from running more than 100 times.
							{
								Wire1.beginTransmission(0);//address is all zeros
								Wire1.write(0);//register address is all zeros
								Wire1.endTransmission(false);//send bytes
							}
						#endif
			//Si7021DriverWire();//diag
			//delay(1000);//diag
		
						#ifdef SiExerciser
							if(Si7021StateByte == idleStateByte)//diag to keep Si7021 running every 2 seconds
							{
								float TempCodeFloat = 256*Si7021receivedDataByte[0]+Si7021receivedDataByte[1];
								float TemperatureC_Float = ((175.72*TempCodeFloat)/65536)-46.85;
								float TemperatureF_Float =  ((9*TemperatureC_Float)/5) + 32;
								Serial.print("temperature is ");
								Serial.print(TemperatureC_Float,1);
								Serial.print("C or ");
								Serial.print(TemperatureF_Float,1);
								Serial.println("F");
								Serial.print("Status code: ");

								switch(Si7021receivedDataByte[16])//process error code into printed text
								{
									case successByte:
									Serial.println("success");
									break;

									case failedByte:
									Serial.println("failure");
									break;

									case timedOutByte:
									Serial.println("timed out");
									break;

									case notReadyYetByte:
									Serial.println("Not ready yet");
									break;

									default:
									Serial.println("Other error");
									break;
								}

								Si7021delayForAndThenReturnTo(2000, Si7021codeBlockA_Byte);//wait 2 seconds and then start next read cycle without being a realtime hog.
								return;//return to base loop. New Si7021StateByte will start that state machine next cycle
							}
						#endif
	//end of state controller

	/********************************************************
							this is a state machine example
			I2CstateByte initialized to idleStateByte so
			this code does not execute beyond first case.
	*******************************************************
	/*
		switch(I2CstateByte)
		{	
			case idleStateByte://so we just return
			break;

			case 0://I2CcodeBlockA_Byte
				// put code block A here
				
				I2CdelayForAndThenReturnTo(2000, I2CcodeBlockB_Byte);//wait 2000 ms and then run code block B
				
				break;

			case 1: // I2CcodeBlockB_Byte
			
				// put code block B here
				
				I2CstateByte = idleStateByte;//by going to the idle state, we exit the code
				break;

			case 2: //delayForByte
				//this is our real-time friendly delay function
				if(millis()- I2CdelayStartTimeULong > I2CdelayULong)
				{
					I2CstateByte = I2CReturnToByte;
				}
				break;
				
			default:
			Serial.println("I2C state machine illegal state");//warns user of a bug
		}
	*
	//end of state machine example

	/*************************************************************************
			This state machine supports an Si7021 temperature sensor on Quiic
	**************************************************************************
*/
/* Si7021 State Machine
	byte writeResultByte = 0;//apparently you can't define a variable from within a switch statement or you get many confusing compiler errors
	byte WireEndTransResultByte = 0;
	#ifdef SiTest
		byte wireRequestResultByte = 0;
	#endif

	switch(Si7021StateByte)
	{
		case idleStateByte:
			break;
		case Si7021codeBlockA_Byte:

							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("block A");
							#endif
			

			Wire.beginTransmission(Si7021AddressByte);//Quiic. Sensor address is 0x40
			writeResultByte = Wire.write(Si7021measureTempByte);

							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print(F("writeResultByte = "));
								Serial.println(writeResultByte);
							#endif

				
			if(writeResultByte != 1)//returns the number of bytes written; 0 for fault and 1 if OK
			{
				Si7021receivedDataByte[16] = failedByte;//populate status byte with failure
				Si7021StateByte = idleStateByte;//done so return to idle
				break;
			}else
			{
				WireEndTransResultByte = Wire.endTransmission(false);
				switch(WireEndTransResultByte)
				{
					case 0://success
						Si7021StateByte = Si7021codeBlockB_Byte;
						break;

					case 1://data too long to fit in transmit buffer.
						Si7021receivedDataByte[16] = 11;
						break;

					case 2://received NACK on transmit of address.buffer.
						Si7021receivedDataByte[16] = 12;
						break;
						
					case 3://received NACK on transmit of data.
						Si7021receivedDataByte[16] = 13;
						break;
						
					case 4://other error.
						Si7021receivedDataByte[16] = 14;
						break;
						
					case 5://timeout
						Si7021receivedDataByte[16] = 15;
						break;
				}
			}
			Si7021StateByte = Si7021codeBlockB_Byte;
			break;

		case Si7021codeBlockB_Byte:

							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("block B");
							#endif
			
			Si7021delayForAndThenReturnTo(20, Si7021codeBlockC_Byte);//wait 20 ms and then run code block C
			break;
				
		case Si7021codeBlockC_Byte:

							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("block C");
								wireRequestResultByte = Wire.requestFrom(Si7021AddressByte,2, true);//command a read of 2 bytes of data. It was false but now true meaning add stop at end. Function returns the number of bytes in the receive buffer.
							#endif
			#ifndef SiTest	
				Wire.requestFrom(Si7021AddressByte,2, true);
			#endif
							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print(F("wireRequestResultByte = "));
								Serial.println(wireRequestResultByte);
							#endif
				
			Si7021startTimeULong = millis();//start timer to prevent hanging on no responds from device
			Si7021StateByte = Si7021codeBlockD_Byte;
			break;
			
		case Si7021codeBlockD_Byte:
							#ifdef SiTest1
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("block D");
							#endif
			
			if(Wire.available() >= 2)
			{//our two bytes are waiting to be read
				Si7021receivedDataByte[0] = Wire.read();//MSB
				Si7021receivedDataByte[1] = Wire.read();//LSB
				Si7021receivedDataByte[16] = successByte;//populate status byte
				Si7021StateByte = idleStateByte;//we are done so return to idle for the Si7021 driver
				//Wire.stop(false);//Wire doesn't have the stop() function but SoftWire does. stop comes from last option on request
							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.print(F("receivedDataByte[0] = "));
								Serial.println(Si7021receivedDataByte[0]);
								Serial.print(F("receivedDataByte[1] = "));
								Serial.println(Si7021receivedDataByte[1]);
								Serial.print(F("status = "));
								Serial.println(Si7021receivedDataByte[16]);
							#endif
						
			}else
			{
				Si7021StateByte = Si7021codeBlockE_Byte;//wait up to 200 ms for data and then give up.
			}
			break;

		case Si7021codeBlockE_Byte:
							#ifdef SiTest1
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("block E");
							#endif

			if((millis() - Si7021startTimeULong) > 200)
			{//we timed out waiting for our temp reading
				Si7021receivedDataByte[16] = timedOutByte;//4
				Si7021StateByte = idleStateByte;//we gave up so return to idle for the Si7021 driver
				//Wire.stop(false);//Wire doesn't have the stop() function but SoftWire does. Not sure how Wire generates that stop signal.

							#ifdef SiTest
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
						
			}else
			{
				Si7021StateByte = Si7021codeBlockD_Byte;//it hasn't been 200 ms yet so look for the data again
			}
			break;
						
		case delayForByte:
		//this is our real-time friendly delay function
		if(millis()- Si7021delayStartTimeULong > Si7021delayULong)
		{
							#ifdef SiTest
								Serial.print("core ");
								Serial.print(rp2040.cpuid());
								Serial.print(" ");
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
								Serial.println("delayFor state");
							#endif
			Si7021StateByte = Si7021ReturnToByte;
		}
		break;//since Si7021StateByte is unmodified when if() is false, we keep coming back here on each cycle until the time is up.
				
		default:
			Serial.println("Si7021 state machine illegal state");//warns user of a bug
		break;
	}//end of Si7021 state machine
*/

	byte AHT20writeResultByte = 0;
	byte AHT20endTransResultByte = 0;

	switch (AHT20StateByte) {

		case idleStateByte:{
			break;
		}

		case AHT20codeBlockA_Byte: { //Start of state machine, tells sensor to start gathering data.

				#ifdef AHTTest
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("block A");
				#endif

			AHT20statusByte = notReadyYetByte;  // we shoudlnt have any data this is the start of the driver.

			Wire.beginTransmission(AHT20_ADDRESS);  // I want to start talking to the device at this I2C address.

			AHT20writeResultByte = 0;  // Counting successful writes.
			AHT20writeResultByte += Wire.write(AHT20_measurementTrigger);  // if the write was good writeResultByte = 0 + 1 = 1, if it was bad writeResultByte = 0 + 0 = 0.
			AHT20writeResultByte += Wire.write(AHT20_measurementSettings);  // if the write was good writeResultByte = 1 + 1 = 2, if it was bad writeResultByte = 0 + 0 = 0
			AHT20writeResultByte += Wire.write(AHT20_placeholder);  // if the write was good writeResultByte = 2 + 1 = 3, if it was bad writeResultByte = 0 + 0 = 0

				#ifdef AHTTest
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.print(F("AHT20writeResultByte = "));
					Serial.println(AHT20writeResultByte);
				#endif


			if (AHT20writeResultByte != 3) { // If we didn't write all 3 bytes we have a failure.
				AHT20statusByte = failedByte;  // set status as failure.
				AHT20record[0] = AHT20humBytes[0];
				AHT20record[1] = AHT20humBytes[1];
				AHT20record[2] = AHT20humBytes[2];
				AHT20record[3] = AHT20tempBytes[0];
				AHT20record[4] = AHT20tempBytes[1];
				AHT20record[5] = AHT20tempBytes[2];
				AHT20record[6] = AHT20statusByte;

				AHT20StateByte = idleStateByte;  // makes us go to the idle state when we break.
				break;  // now that we failed and set the state machine to idle we get out of A and go to idle.
			}

			AHT20endTransResultByte = Wire.endTransmission(true);  // Sends a START condition on the I2C bus, Sends the address (0x38), Sends each byte in the transmit buffer, Waits for ACK (acknowledgment) from the slave after each byte, Optionally sends a STOP condition. False means do NOT send STOP and keeps the bus running. True means Stop and makes bus stop.

			if (AHT20endTransResultByte != 0) {  // 0 is a success (sensor acknowledged) if it isnt 0 go into this statement.

				AHT20statusByte = failedByte;  // set status as failure.
				AHT20record[0] = AHT20humBytes[0];
				AHT20record[1] = AHT20humBytes[1];
				AHT20record[2] = AHT20humBytes[2];
				AHT20record[3] = AHT20tempBytes[0];
				AHT20record[4] = AHT20tempBytes[1];
				AHT20record[5] = AHT20tempBytes[2];
				AHT20record[6] = AHT20statusByte;

				AHT20StateByte = idleStateByte;  // makes us go to the idle state when we break.
				break;  // now that we failed and set the state machine to idle we get out of A and go to idle.
			}

			AHT20StateByte = AHT20codeBlockB_Byte; // We did it everything shouldve worked now we go to state B.
			break;  // if everything went well and we had no errors or hopfully no errors go through we get out of case A and go to case B.
		}

		case AHT20codeBlockB_Byte: { //Waits for data to be gathered

				#ifdef AHTTest
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("block B");
				#endif

			AHT20delayForAndThenReturnTo(80, AHT20codeBlockC_Byte); // We have to wait 80 ms for sensor to get temp and hum measurments.
			break;  // after we are done waiting the next state is C so we get out of B and go to case C.
		}

		case AHT20codeBlockC_Byte: { //After delay from State B, sends request to sensor for data

				#ifdef AHTTest
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("block C");
				#endif

			Wire.requestFrom(AHT20_ADDRESS, (uint8_t)6, (uint8_t)true);  // Ask for 6 bytes and release bus at end. We want data from AHT20_ADDRESS, it should be 6, true is to stay stop when we are done.
			AHT20startTimeULong = millis();  // sets AHT20startTimeULong to the current ms that have passed since the FDR has been on.
			AHT20StateByte = AHT20codeBlockD_Byte;
			break;
		}

		case AHT20codeBlockD_Byte: { //After request is sent, reads the available data
				#ifdef AHTTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.println("block D");
				#endif

			if (Wire.available() >= 6) {  // check if we recived at least 6 bytes.
				for (int i = 0; i < 6; i++) {  // First, put the first 6 bytes received into the data array
					AHT20data[i] = Wire.read();
				}

				if (AHT20data[0] & 0x80) {  // check to see if the sensor is still busy. If it is, go to state E to wait for it to be ready.
					AHT20statusByte = notReadyYetByte;
					AHT20StateByte = AHT20codeBlockE_Byte;
					break;
				}
				AHT20byteSplitter();  // If we get a good reading than we get to split the bytes to have 3 bytes of hum and 3 of temp
				AHT20statusByte = successByte;

					#if AHT20_DEBUG
							float humidity = (AHT20rawHum20bit * 100.0f) / 1048576.0f;
							float tempC = (AHT20rawTemp20bit * 200.0f) / 1048576.0f - 50.0f;
							float tempF = tempC * 9.0f / 5.0f + 32.0f;

							Serial.print("[AHT20] RH=");
							Serial.print(humidity, 1);
							Serial.print("%  T=");
							Serial.print(tempC, 1);
							Serial.print("C ");
							Serial.print(tempF, 1);
							Serial.println("F");
					#endif

				//Record the data after it's been split'
				AHT20record[0] = AHT20humBytes[0];
				AHT20record[1] = AHT20humBytes[1];
				AHT20record[2] = AHT20humBytes[2];
				AHT20record[3] = AHT20tempBytes[0];
				AHT20record[4] = AHT20tempBytes[1];
				AHT20record[5] = AHT20tempBytes[2];
				AHT20record[6] = AHT20statusByte;

					#ifdef AHTTest
							Serial.print("core ");
							Serial.print(rp2040.cpuid());
							Serial.print(" ");
							Serial.print(__FUNCTION__);
							Serial.print(F("(): "));
							Serial.print(__LINE__);
							Serial.print(F(".    Time Stamp  "));
							Serial.print (millis() - StartForTimeStampULong);
							Serial.println(F(" ms"));
							Serial.print(F("AHT20data[0] = "));
							Serial.println(AHT20data[0]);
							Serial.print(F("AHT20data[1] = "));
							Serial.println(AHT20data[1]);
							Serial.print(F("AHT20data[2] = "));
							Serial.println(AHT20data[2]);
							Serial.print(F("AHT20data[3] = "));
							Serial.println(AHT20data[3]);
							Serial.print(F("AHT20data[4] = "));
							Serial.println(AHT20data[4]);
							Serial.print(F("AHT20data[5] = "));
							Serial.println(AHT20data[5]);
							Serial.print(F("status = "));
							Serial.println(AHT20record[0]);
					#endif
				AHT20StateByte = idleStateByte;  // we go to idle to start all over again.
			}
			else {  // if we dont have 6 bytes yet go to Case E
				AHT20StateByte = AHT20codeBlockE_Byte;
			}

			break;
		}

		case AHT20codeBlockE_Byte: { //Delay state if sensor needs more time to gather data or get ready

				#ifdef AHTTest
					Serial.print("core ");
					Serial.print(rp2040.cpuid());
					Serial.print(" ");
					Serial.print(__FUNCTION__);
					Serial.print(F("(): "));
					Serial.print(__LINE__);
					Serial.print(F(".    Time Stamp  "));
					Serial.print (millis() - StartForTimeStampULong);
					Serial.println(F(" ms"));
					Serial.println("block E");
				#endif

			if ((millis() - AHT20startTimeULong) > 200) {  // if we are past 200 ms

				AHT20statusByte = timedOutByte;  // This is to say sensor is in timeout.
				AHT20record[0] = 0;
				AHT20record[1] = 0;
				AHT20record[2] = 0;
				AHT20record[3] = 0;
				AHT20record[4] = 0;
				AHT20record[5] = 0;
				AHT20record[6] = AHT20statusByte;

				AHT20StateByte = idleStateByte;  // didnt work out now we have to restart.

					#ifdef AHTTest
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

			else {
				AHT20StateByte = AHT20codeBlockD_Byte;  // goes back to case D if everything went well do split the data.
			}
			break;
		}

		case delayForByte: {  // this is the temporary waiting state.

			if (millis() - AHT20delayStartTimeULong > AHT20delayULong) {  // when our timer is out go back to the state we wanted to go to.

					#ifdef AHTTest
						Serial.print("core ");
						Serial.print(rp2040.cpuid());
						Serial.print(" ");
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						Serial.println("delayFor state");
					#endif

				AHT20StateByte = AHT20ReturnToByte;
			}
			break;
		}

		default:  {//warns user of an illegal state then returns to idle state
			Serial.println("AHT20 State Machine Illegal State!");
			AHT20StateByte = idleStateByte;
			break;
		}
	}

}

void I2CdelayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte)
{//this prepares us for a delay that is real-time friendly

	#ifdef SiTest
		Serial.print("core ");
		Serial.print(rp2040.cpuid());
		Serial.print(" ");
		Serial.print(__FUNCTION__);
		Serial.print(F("(): "));
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));
		Serial.print (millis() - StartForTimeStampULong);
		Serial.println(F(" ms"));
		Serial.println("I2CdelayForAndThenReturnTo");
	#endif

	I2CstateByte = delayForByte; //we go to the timer state next
	I2CReturnToByte = nextStateByte;
	I2CdelayStartTimeULong = millis();//reset timer
	I2CdelayULong = delayUInt;//this makes the delay global so it will persist as we return to loop1() 
}

void Si7021delayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte)
{//this prepares us for a delay that is real-time friendly for the Si7021 driver

	#ifdef SiTest
		Serial.print("core ");
		Serial.print(rp2040.cpuid());
		Serial.print(" ");
		Serial.print(__FUNCTION__);
		Serial.print(F("(): "));
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));
		Serial.print (millis() - StartForTimeStampULong);
		Serial.println(F(" ms"));
		Serial.println("Si7021delayForAndThenReturnTo");
		Serial.print("delay is ");
		Serial.println(delayUInt);
		Serial.print("next state is ");
		Serial.println(nextStateByte);
	#endif

	Si7021StateByte = delayForByte; //we go to the timer state next
	Si7021ReturnToByte = nextStateByte;
	Si7021delayStartTimeULong = millis();//reset timer
	Si7021delayULong = delayUInt;//this makes the delay global so it will persist as we return to loop1() 
}

void AHT20delayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte) {

	#ifdef AHTTest
		Serial.print("core ");
		Serial.print(rp2040.cpuid());
		Serial.print(" ");
		Serial.print(__FUNCTION__);
		Serial.print(F("(): "));
		Serial.print(__LINE__);
		Serial.print(F(".    Time Stamp  "));
		Serial.print (millis() - StartForTimeStampULong);
		Serial.println(F(" ms"));
		Serial.println("AHT20delayForAndThenReturnTo");
		Serial.print("delay is ");
		Serial.println(delayUInt);
		Serial.print("next state is ");
		Serial.println(nextStateByte);
	#endif


	AHT20StateByte = delayForByte;
	AHT20ReturnToByte = nextStateByte;
	AHT20delayStartTimeULong = millis();
	AHT20delayULong = delayUInt;
}

void Si7021DriverWire()//driver which does not include a state machine
{
	
							#ifdef siDriver
								Serial.print(__FUNCTION__);
								Serial.print(F("(): "));
								Serial.print(__LINE__);
								Serial.print(F(".    Time Stamp  "));
								Serial.print (millis() - StartForTimeStampULong);
								Serial.println(F(" ms"));
							#endif
				
	
	Wire.beginTransmission(Si7021AddressByte);//bus 0. Sensor address is 0x40
	if(Wire.write(Si7021measureTempByte) == 0)//returns 0 for fault and 1 if OK
	{
		Si7021receivedDataByte[16] = failedByte;//populate status byte with failure
			#ifdef siDriver
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
			#endif
		return;
	}
	byte endTransResultByte = Wire.endTransmission(false);//send slave address, write and measurement command
	if(endTransResultByte != 0)
	{
		Si7021receivedDataByte[16] = failedByte;//populate status byte with failure
			#ifdef siDriver
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
				Serial.print("endTransResultByte = ");
				Serial.println(endTransResultByte);
			#endif
		return;
	}

	delay(20);//time for sensor to take a reading plus software overhead
	Wire.requestFrom(Si7021AddressByte,2,true);//command a read of 2 bytes of data it was false but now true because Wire has stop command here
	unsigned long startTimeULong = millis();
		
			#ifdef siDriver
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
			#endif
	while(1)
	{//read the MS Byte and LS Byte
		if(Wire.available() >= 2)
		{//our two bytes are waiting to be read

			#ifdef siDriver
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
			#endif
				
			Si7021receivedDataByte[0] = Wire.read();//MSB
			Si7021receivedDataByte[1] = Wire.read();//LSB
			Si7021receivedDataByte[16] = successByte;//set status byte
			Serial.print(F("Si7021receivedDataByte[0] = "));
			Serial.println(Si7021receivedDataByte[0]);
			Serial.print(F("Si7021receivedDataByte[1] = "));
			Serial.println(Si7021receivedDataByte[1]);
			Serial.print(F("status = "));
			Serial.println(Si7021receivedDataByte[16]);
			return;
		}else
		{
			if((millis() - startTimeULong) > 200)
			{//we timed out waiting for our temp reading
				Si7021receivedDataByte[16] = timedOutByte;//4
				Serial.println("we timed out");
				return;//give up on sensor after waiting 200 ms
			}
		}
	}
		//sw0.stop(false); //there is not Wire stop()
		
			#ifdef siDriver
				Serial.print(__FUNCTION__);
				Serial.print(F("(): "));		
				Serial.print(__LINE__);
				Serial.print(F(".    Time Stamp  "));	
				Serial.print (millis() - StartForTimeStampULong);
				Serial.println(F(" ms"));
			#endif
}
