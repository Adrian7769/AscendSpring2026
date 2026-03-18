// TEAM #2
char FDRinoVersionChar[] = "4.2.3";//if you modify this code, update the version number.

//#define subus
//#define neoTime
//#define NeoTest
//#define loopTimerTest
//#define extLEDexperiment


// TO DO BEFORE NEXT TEST:
// Remove Record Counter in SEEPROM.
// FIX EEPROM DECODER INCORRECT DATA.
// REFINE FLIGHT STATUS AND BUZZER STATUS CODES. 
// FOR BUZZER ONLY BEEP on SUCCESS TX, LANDING, AND BT-FAIL THIS IS WRONG --- >
// [ANALOG] PSI=18.62  Gx=16.07  Gy=16.07  |G|=22.73  IntT=431.6C  ExtT=432.6C  Batt=0.30V
// [PHASE] !! PSI out of range: 18.62 — skipping this cycle
// [SEEPROM-HDR] Updated: addr=624  rec=38  latch=0b0
// [SEEPROM] Rec#38  addr=608  used=0%  (38/8191 recs)  AHT=0  radioIdx=0  sig=2  flight=0b00000001

#define ADRIAN Serial.print("core ");Serial.print(rp2040.cpuid());Serial.print(" ");Serial.print(__FUNCTION__);Serial.print(F("(): "));Serial.print(__LINE__);Serial.print(F(".    Time Stamp  "));	Serial.print (millis() - StartForTimeStampULong);Serial.println(F(" ms"));
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

//=====================================================================================================================================================================

const byte AHT20_ADDRESS = 0x38;  // I2C address for AHT20.
const byte AHT20_measurementTrigger = 0xAC;  // Measurement command This is the measurement trigger instruction.
const byte AHT20_measurementSettings = 0x33;  // This configures internal measurement settings.
const byte AHT20_placeholder = 0x00;  // This is a required placeholder byte.

const byte AHT20codeBlockA_Byte = 1;  // this is where we tell the sensor to start measuring the temp and hum.
const byte AHT20codeBlockB_Byte = 2;  // this is when we are waiting for the sensor to take the measurements. (delay 80ms)
const byte AHT20codeBlockC_Byte = 3;  // This is were we request the sensor for its Temp and hum data.
const byte AHT20codeBlockD_Byte = 4;  // This is where if we got all the bytes we will read them and put them in an array to send to seeprom.
const byte AHT20codeBlockE_Byte = 5;  // This is where we wait a little bit longer if we cant read the bytes yet.

byte AHT20StateByte = idleStateByte;  // this is what is going to say what state we are going to be in and going too. we start off in idle because in the beginning before we go to block A-E we arnt doing anything.
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

// Converted value varibles.
float AHT20humidityRH = 0.0f;
float AHT20tempC = 0.0f;
float AHT20tempF = 0.0f;

byte AHT20statusByte = notReadyYetByte;  // The current status we are in. For example somthing was successful of somthing failed.

void AHT20delayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte);  //This starts a non-blocking delay for how long we want and when the delay is finished go to the next place we want.
void AHT20byteSplitter();  // This is where we are going to extract the temp and hum bytes.

#if AHT20_DEBUG
  void AHT20printState(const char* msg) {
		Serial.print("[AHT20] ");
    Serial.println(msg);
  }
#endif

//=====================================================================================================================================================================


//define as many codeBlocks as you need
const byte delayForByte = 255;

// Generic I2C state machine variables (template/example)
byte I2CstateByte = idleStateByte;
unsigned long I2CdelayStartTimeULong  = millis();
unsigned long I2CdelayULong = 0;
byte I2CReturnToByte = 0;  // set in I2CdelayForAndThenReturnTo()
byte I2CreceivedDataByte[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// ============================================================
//   TASK SCHEDULER — 2-Second SEEPROM Write
// ============================================================
//
//   SEEPROM RECORD FORMAT  (16 bytes per record, written every 2 s)
//   ---------------------------------------------------------------
//   Every record begins at an address that is a multiple of 16.
//   Record number = seepromWriteAddressLong / 16  (0-based)
//
//   Only I2C sensor data is stored here.  GPS data lives on the SLAVE board and
//   is received via Bluetooth — it is not stored in this SEEPROM.  The GPS data
//   is forwarded over Iridium inside the 5-minute TX array (TX[1..16]).
//
//   Byte  Field                          Source
//   ----  ------------------------------ ----------------------------
//   [0]   Record sequence number (0-255, wraps)  — seepromRecordCountByte
//   [1]   AHT20 Humidity  byte 0 (MSB)   AHT20record[0]  (20-bit raw, top nibble)
//   [2]   AHT20 Humidity  byte 1 (mid)   AHT20record[1]
//   [3]   AHT20 Humidity  byte 2 (LSB)   AHT20record[2]
//   [4]   AHT20 Temp      byte 0 (MSB)   AHT20record[3]  (20-bit raw, top nibble)
//   [5]   AHT20 Temp      byte 1 (mid)   AHT20record[4]
//   [6]   AHT20 Temp      byte 2 (LSB)   AHT20record[5]
//   [7]   AHT20 status byte              AHT20record[6]  (2=OK 3=fail 4=timeout)
//   [8]   Last locked radio station MSB  Last_scanned_station_idx >> 8
//   [9]   Last locked radio station LSB  Last_scanned_station_idx & 0xFF
//   [10]  Last locked station signal     Last_scanned_station_signal_strength (0-15)
//   [11]  Radio stereo flag              RadioStereoBool (1=stereo, 0=mono)
//   [12]  Flight status byte             flightStatusByte (see bit definitions below)
//   [13]  Reserved (0x00)
//   [14]  Reserved (0x00)
//   [15]  Reserved (0x00)
//
//   To recover humidity %RH from bytes [1..3]:
//     raw20 = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3
//     RH%   = (raw20 * 100.0) / 1048576.0
//
//   To recover temperature from bytes [4..6]:
//     raw20 = ((uint32_t)byte4 << 16) | ((uint32_t)byte5 << 8) | byte6
//     TempC = (raw20 * 200.0 / 1048576.0) - 50.0
//     TempF = TempC * 9.0 / 5.0 + 32.0
//
//   To recover radio station frequency from bytes [8..9]:
//     idx   = ((uint16_t)byte8 << 8) | byte9
//     MHz   = 89.2 + (idx * 0.1)   [valid range: idx 0-188]
//
//   Flight status byte [12] bit map:
//     Bit 0 = BT RX Success
//     Bit 1 = BT RX Failure
//     Bit 2 = Balloon Burst (accel > 5 g)
//     Bit 3 = Descent
//     Bit 4 = Ascent
//     Bit 5 = Landing
// ---------------------------------------------------------------
byte latestI2Cblock[16]   = {0};  // most recently assembled sensor snapshot
byte previousI2Cblock[16] = {0};  // last snapshot confirmed written to SEEPROM
byte seepromRecordCountByte   = 0; // wrapping sequence counter stored in each record

// SEEPROM capacity: 128 KB = 131072 bytes, max address = 131071
// Each record is 16 bytes → max 8191 records → ~4.55 hours at 2-second intervals
// (minus 1 record because address 0-15 is the header)
const long SEEPROM_MAX_ADDRESS_LONG = 131072L;  // first invalid address (one past last byte)
bool seepromFullBool = false;  // set true when no more room; stops further writes
bool seepromRecoveredBool = false; // one-shot: set true after seepromRecoverOrInit() runs

// Update and Fix stored in block zero in EEprom
// ============================================================
//   SEEPROM PERSISTENT HEADER  (address 0-15, first 16 bytes)
// ============================================================
//   Header format (16 bytes):
//   Byte  Content
//   [0]   Magic byte 0 — 0xFD
//   [1]   Magic byte 1 — 0x52  (together = "FDR" marker)
//   [2]   seepromWriteAddressLong  byte 0 (LSB)
//   [3]   seepromWriteAddressLong  byte 1
//   [4]   seepromWriteAddressLong  byte 2
//   [5]   seepromWriteAddressLong  byte 3 (MSB)
//   [6]   seepromRecordCountByte
//   [7]   Latched flags: bit 0 = burstLatchedBool, bit 1 = landingLatchedBool
//   [8..15] Reserved (0x00)
//
//   Data records begin at address 16 (SEEPROM_DATA_START_LONG).
// ============================================================
const byte  SEEPROM_MAGIC_0           = 0xFD;
const byte  SEEPROM_MAGIC_1           = 0x52;
const long  SEEPROM_DATA_START_LONG   = 16L;   // first data record address

// seepromWriteAddressLong is the NEXT address to write to.
// Initialized to SEEPROM_DATA_START_LONG; overridden by header recovery on boot.
long seepromWriteAddressLong  = SEEPROM_DATA_START_LONG;

const unsigned long TASK_2SEC_INTERVAL_UL = 2000UL;
unsigned long task2SecTimerULong = 0;  // initialised to 0; fires on first loop pass

// ============================================================
//   FLIGHT STATUS BYTE  (packed into IridiumTransmitDataByte[0])
// ============================================================
//   Bit 0 — BT RX Success           (set by btUpdate)
//   Bit 1 — BT RX Failure           (set by btUpdate)
//   Bit 2 — Balloon Burst           (LATCHED — once set, never clears)
//   Bit 3 — Descent                 (live — pressure rising)
//   Bit 4 — Ascent                  (live — pressure dropping)
//   Bit 5 — Landing                 (LATCHED — once set, never clears)
//   Bits 6-7 reserved (0)
#define STATUS_BT_RX_SUCCESS  (1 << 0)
#define STATUS_BT_RX_FAILURE  (1 << 1)
#define STATUS_BALLOON_BURST  (1 << 2)
#define STATUS_DESCENT        (1 << 3)
#define STATUS_ASCENT         (1 << 4)
#define STATUS_LANDING        (1 << 5)
byte flightStatusByte = 0;  // updated by flight logic; written to TX[0] and SEEPROM[12]

// ============================================================
//   MASTER BOARD ANALOG PORT MAP
// ============================================================
//   Port  Sensor                    Wire Color
//   ----  -------------------------  ----------
//    0    Camera trigger (RC6)       —
//    1    (unused)                   —
//    2    Pressure                   Green
//    3    External Temperature       Gold
//    4    Internal Temperature       Silver
//    5    Accelerometer X-axis       Yellow
//    6    Accelerometer Y-axis       Green
//
//   Battery is read via analogRead(29) on the RP2040,
//   NOT through the external ADC (ports 0-7).
// ============================================================
const byte ADC_PORT_PRESSURE    = 2;
const byte ADC_PORT_EXT_TEMP    = 3;
const byte ADC_PORT_INT_TEMP    = 4;
const byte ADC_PORT_ACCEL_X     = 5;
const byte ADC_PORT_ACCEL_Y     = 6;

// ============================================================
//   SENSOR CALIBRATION CONSTANTS
//   (from lab calibration data)
// ============================================================
// Accelerometer:  G = (Vm - offset) / scale
const float ACCEL_X_OFFSET_V  = 1.650f;   // voltage at 0 g
const float ACCEL_X_SCALE_VPG = 0.224f;   // volts per g
const float ACCEL_Y_OFFSET_V  = 1.651f;   // voltage at 0 g
const float ACCEL_Y_SCALE_VPG = 0.224f;   // volts per g (Y uses same scale per cal sheet)

// Pressure:  PSI = 3.745 * Vm - 1.04
const float PRESSURE_SCALE_PSIPV = 3.745f; // PSI per volt
const float PRESSURE_OFFSET_PSI  = -1.04f; // PSI offset

// Temperature (analog):
//   Internal: T°C = (Vm - 0.502) / 0.011
//   External: T°C = (Vm - 0.491) / 0.011
const float INT_TEMP_OFFSET_V  = 0.502f;
const float EXT_TEMP_OFFSET_V  = 0.491f;
const float TEMP_SCALE_VPDC    = 0.011f;   // volts per degree C

// Battery:  Va = Vr / 0.971
const float BATTERY_CAL_DIVISOR = 0.971f;

// ============================================================
//   FLIGHT PHASE DETECTION — thresholds & state
// ============================================================
//   Validated against Fall 2025 Ascend flight data (Team 2).
//
//   Ascent:    PSI drops (altitude rising), |G| ≈ 1.0
//   Burst:     |G| spikes > 5 g  while airborne (PSI < 13.5)
//   Descent:   PSI rises (altitude falling),  burst already latched
//   Landing:   PSI stable near ground + |G| < 1.0 for N readings
//
//   NOTE — parachute descent produces 5-7 g average due to
//   oscillation, so |G| > 5 alone does NOT distinguish burst from
//   descent.  Burst is latched on *first* occurrence while airborne.
// ============================================================

// Detection thresholds
const float BURST_G_THRESHOLD       = 5.0f;   // |G| above this triggers burst (if airborne)
const float GROUND_PSI_THRESHOLD    = 13.5f;   // PSI above this ≈ ground level
const float ASCENT_DELTA_PSI        = 0.02f;   // PSI must drop by this per 2 s to count as ascending
const float DESCENT_DELTA_PSI       = 0.02f;   // PSI must rise by this per 2 s to count as descending
const float LANDING_G_THRESHOLD     = 1.0f;    // |G| below this means payload stopped swinging
const float LANDING_PSI_STABLE      = 0.05f;   // max |dPSI| per 2 s to count as "stable"
const byte  LANDING_STABLE_COUNT    = 5;        // consecutive stable readings → landing (10 seconds)

// Detection state (persists across calls)
float previousPSI_Float             = 0.0f;    // last pressure reading
bool  previousPSI_ValidBool         = false;    // false until first reading taken
bool  burstLatchedBool              = false;    // once set, never clears
bool  landingLatchedBool            = false;    // once set, never clears
byte  landingStableCounterByte      = 0;        // consecutive ground-stable readings

// Latest computed values (available for diag prints)
float latestGx_Float   = 0.0f;
float latestGy_Float   = 0.0f;
float latestGmag_Float = 0.0f;
float latestPSI_Float  = 0.0f;

// FlightPhaseDiag is now controlled by DIAG_PHASE in the Debug Control Panel above

// -------------------- JUSTIN I2C ---------------- //
#define ENABLE_TEA5767  //JT

#ifdef ENABLE_TEA5767
/************************************************************************
 * TEA5767 RADIO SCANNER DECLARATIONS
 * Based on NXP TEA5767HN Product Data Sheet (from datasheet)
 ************************************************************************/
// I2C address is fixed at 110 0000b (0x60) (from datasheet)
const byte RadioAddressByte = 0x60; 

// State Machine States
const byte RadioIdleStateByte     = 0;
const byte RadioWriteDataByte     = 1;
const byte RadioReadStatusByte    = 2;
const byte RadioScanningStateByte = 3;

// Timing Constants (from datasheet)
// Using 32.768 kHz clock crystal for reference (from datasheet)
const unsigned long Crystal_Freq = 32768; 
const unsigned long IF_Freq      = 225000; // 225 kHz Intermediate Frequency (from datasheet)


// Scanning Configuration
const float ScanStartFreq = 89.2; // Your requested 89.2 to 108 MHz cycle
const float ScanEndFreq   = 108.0; // US/Europe Band End (from datasheet)
const float ScanIncrement = 0.1;   // 0.1 MHz increments

// Precomputed Station Frequencies
// Pre-calculated PLL values for TEA5767
// Range: 89.2 to 108.0 MHz in 0.1 MHz steps
// Total Stations: 189

const uint16_t RadioStationPLL[189] = {
    10916, 10928, 10940, 10952, 10964, 10977, 10989, 11001, 11013, 11026, 
    11038, 11050, 11062, 11074, 11087, 11099, 11111, 11123, 11135, 11148, 
    11160, 11172, 11184, 11196, 11209, 11221, 11233, 11245, 11257, 11270, 
    11282, 11294, 11306, 11318, 11331, 11343, 11355, 11367, 11380, 11392, 
    11404, 11416, 11428, 11441, 11453, 11465, 11477, 11489, 11502, 11514, 
    11526, 11538, 11550, 11563, 11575, 11587, 11599, 11611, 11624, 11636, 
    11648, 11660, 11672, 11685, 11697, 11709, 11721, 11734, 11746, 11758, 
    11770, 11782, 11795, 11807, 11819, 11831, 11843, 11856, 11868, 11880, 
    11892, 11904, 11917, 11929, 11941, 11953, 11965, 11978, 11990, 12002, 
    12014, 12026, 12039, 12051, 12063, 12075, 12088, 12100, 12112, 12124, 
    12136, 12149, 12161, 12173, 12185, 12197, 12210, 12222, 12234, 12246, 
    12258, 12271, 12283, 12295, 12307, 12319, 12332, 12344, 12356, 12368, 
    12380, 12393, 12405, 12417, 12429, 12442, 12454, 12466, 12478, 12490, 
    12503, 12515, 12527, 12539, 12551, 12564, 12576, 12588, 12600, 12612, 
    12625, 12637, 12649, 12661, 12673, 12686, 12698, 12710, 12722, 12734, 
    12747, 12759, 12771, 12783, 12796, 12808, 12820, 12832, 12844, 12857, 
    12869, 12881, 12893, 12905, 12918, 12930, 12942, 12954, 12966, 12979, 
    12991, 13003, 13015, 13027, 13040, 13052, 13064, 13076, 13088, 13101, 
    13113, 13125, 13137, 13150, 13162, 13174, 13186, 13198, 13211
};


// Driver Variables
byte RadioStateByte             = RadioIdleStateByte;
unsigned long RadioDelayStartTimeULong = 0;
unsigned long RadioDelayDurationULong  = 0;
byte RadioReturnToByte          = 0; 

// Output Variables
float RadioTargetFrequency = ScanStartFreq; // old unused
byte RadioSignalLevel      = 0;             // 4-bit ADC level (from datasheet)
bool RadioStereoBool       = false;         // Stereo indication flag (from datasheet)
bool RadioAutoScanEnabled  = true;          // Control flag for the loop


uint16_t stationIndex = 0; 
const uint16_t totalStations = sizeof(RadioStationPLL) / sizeof(RadioStationPLL[0]); 


// --- NEW GLOBALS FOR TRACKING ---
uint16_t Last_scanned_station_idx = 0;
byte Last_scanned_station_signal_strength = 0;

// RADIO_DIAG_PRINT is now controlled from the Debug Control Panel above
#endif

// ------------- JUSTIN I2C ------------ // 

// ============================================================
//   BUZZER — GPIO 22
// ============================================================
const byte BUZZER_PIN = 22;

// Pattern tables — {ON_ms, OFF_ms, ON_ms, OFF_ms, ...}

// LANDED: rapid continuous beeping — easy to track on the ground
const unsigned int PAT_LANDED[]   = {120, 120, 0};

// BURST: 3 urgent rapid beeps then pause
const unsigned int PAT_BURST[]    = {80, 80, 80, 80, 80, 2000, 0};

// DESCENT: double beep every 3 seconds
const unsigned int PAT_DESCENT[]  = {150, 150, 150, 2500, 0};

// ASCENT: single beep every 4 seconds 
const unsigned int PAT_ASCENT[]   = {200, 4000, 0};

// BT FAILURE: long-short-long
const unsigned int PAT_BT_FAIL[]  = {400, 200, 100, 200, 400, 4000, 0};

// TX SUCCESS: quick triple chirp
const unsigned int PAT_TX_OK[]    = {50, 50, 50, 50, 50, 0};

// IDLE / PRE-LAUNCH: silence
const unsigned int PAT_IDLE[]     = {0};

// Buzzer state machine variables
const unsigned int* buzzerPattern       = PAT_IDLE;  // active pattern pointer
byte  buzzerPatternIndex                = 0;          // position within the pattern
bool  buzzerToneOn                      = false;      // is the pin HIGH right now?
unsigned long buzzerStepStartULong      = 0;          // when current step began
byte  buzzerPreviousFlightStatus        = 0;          // for transition detection
bool  buzzerTxChirpPending              = false;      // one-shot TX success chirp
byte  buzzerTxChirpStep                 = 0;
unsigned long buzzerTxChirpStartULong   = 0;

#define DIAG_BUZZER  // uncomment to see buzzer state transitions on Serial 
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
bool GNSS_Bool = unequippedBool; //if a GPS is connected, set to equippedBool. Otherwise, say unequippedBool.		
bool ModemBool = equippedBool; //Iridium modem connected — iridiumScheduler() active
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
byte nearURBsnByte = 1;// was 0 replace number with the UserRBsn you will fly
byte farURBsnByte = 3;//replace number with the UserRBsn you want to send data to

/******************************************
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
unsigned long systemDiagTimerULong = 0;          // for DIAG_SYSTEM periodic summary
const unsigned long SYSTEM_DIAG_INTERVAL_UL = 30000UL; // print system summary every 30 s
unsigned int modemTimerUInt = millis();

// ============================================================
//   IRIDIUM 5-MINUTE TASK SCHEDULER
// ============================================================
// States for the scheduler state machine
enum IridiumSchedState {
    ISCHED_IDLE,       // waiting for transmit window timer to expire
    ISCHED_BUILD,      // assemble the 45-byte TX array
    ISCHED_TRANSMIT,   // issue PerformTransmitUInt to the modem
    ISCHED_AWAIT,      // poll ReturnCodeUInt each loop pass (non-blocking)
    ISCHED_COOLDOWN    // wait a retry delay, then return to ISCHED_IDLE
};
IridiumSchedState iridiumSchedState = ISCHED_IDLE;

const unsigned long IRIDIUM_NOMINAL_INTERVAL_UL = 300000UL; // 5 minutes in ms
unsigned long iridiumSchedTimerULong  = 0;   // set to millis() when a wait begins
unsigned long iridiumSchedDelayULong  = IRIDIUM_NOMINAL_INTERVAL_UL; // current wait duration

// Rolling modem status history for TX[42..43]
byte iridiumCurrentStatusByte  = 0;  // last modem return code (low byte), TX[42]
byte iridiumPreviousStatusByte = 0;  // one before that, TX[43]
byte iridiumTxSuccessCountByte = 0;  // cumulative successful transmissions, TX[44]

// Diagnostic: uncomment IridiumSchedDiag in the diagnostics section below to enable prints

byte LEDflashInfoByte[3];//contains system info from library plus element [2] is 0 if diagLEDmode is not defined in Fdr.cpp and is a 1 if it is
bool AutomaticVideoRecordAtPowerUp = false; //We can reuse this in pointer array

bool printOnceBool = true;//diag
byte ExternalLED_OnByte = 255;
byte ExternalLED_OffByte = 0;
byte ExternalLED_dimByte = 32;//emperically found

//#define DIAG_BT             // [BT] polls, retries, CRC results, status bit changes

#define DIAG_SEEPROM        // [SEEPROM] every 2s write: record#, address, full/ok
#define DIAG_SEEPROM_HEADER // [SEEPROM] header reads/writes during recovery & updates

//#define DIAG_IRIDIUM        // [ISCHED] state transitions, return codes, retry delays
//#define DIAG_TX_ARRAY       // [TX] hex dump of all 45 bytes when array is built

#define DIAG_PHASE          // [PHASE] PSI, dPSI, |G|, Gx, Gy, status bits every 2s

#define DIAG_ANALOG         // [ANALOG] pressure PSI, accel G, temps °C, battery V

#define AHTTaskManager      // [AHT20] task manager state transitions
//#define AHT20_DEBUG 1       // [AHT20] RH% and Temp on every successful read (0=off, 1=on)
//#define RADIO_DIAG_PRINT  // [RADIO] TEA5767 station, signal, stereo on every lock

// We probably do not need this here
#define DIAG_SYSTEM         // [SYS] periodic summary every 30s: uptime, status byte, all states

//#define AHTTest           // verbose AHT20 I2C wire-level trace
//#define loopTimerDetails  // loop() execution timing
//#define FlightPhaseDiag   // (replaced by DIAG_PHASE above)
//#define IridiumSchedDiag  // (replaced by DIAG_IRIDIUM above)
//#define SEEPROMflagTest
//#define quiicTest
//#define diagprint
//#define frozen
//#define comVar		

// time
// lat long
// altitude

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
extern volatile unsigned int ReturnCodeUInt; //defined in Iridium.ino
						#ifdef GPScommandTest
						byte setAIR1[13] = {0xb5,0x62,0x06,0x8a,0x05,0x00,0x20,0x11,0x00,0x1c,0x06,0xE8,0x36};
						byte checkAIR1[12] = {0xb5,0x62,0x06,0x8b,0x04,0x00,0x20,0x11,0x00,0x1c,0xe2,0x4F};
						byte alphaTest[3] = {0x61, 0x62, 0x63};
						#endif
	

volatile byte core0toCore1Byte = 0;//used by heartbeat
volatile byte core1toCore0Byte = 0;//used by heartbeat
volatile byte core0DataByte = 0;//used by heartbeat

volatile unsigned int ModemCommandUInt = 100;//10 is the value of NoActiveModemCommandUInt which is set by Iridium.ino when a command has been completed. I initialize to this value to be safe and hope it doesn't cause confusion.

/*******************************************/
intptr_t PointerArray[12];
/******************************************
		SET UP LIBRARY ENVIRONMENT
******************************************/		
Fdr Fdr; //the second Fdr is the Constructor for the class Fdr which is the first Fdr on the line 

// --------------------------------------------- BT MODEM --------------------------------------------- // 
bool BT_EQQUIPPED_BOOL = true; // BT hardware connected — btUpdate() runs in loop()
enum BT_State {
    BT_IDLE,
    BT_WAIT_RESPONSE,
    BT_READING,
    BT_COMPLETE,
    BT_ERROR
};
const byte    BT_SLAVE_REQ    = 0x01;
const uint8_t PACKET_SIZE     = 19;   // Slave sends: + GNSS[0-15] + battery[16-17] + CRC[18]
const long    POLL_INTERVAL   = 2000;   
const long    RX_TIMEOUT      = 2000;  
const byte MAX_RETRIES = 1; 
byte btRetryCount = 0;
BT_State btState=BT_IDLE;
bool btDataValidBool = false;  // true after first successful validated BT RX
// lastValidBtRx holds the most recent GOOD packet from the slave.
// buildIridiumTxArray() reads from here instead of received.rxBuffer directly,
// so a failed poll doesn't wipe out the last known GNSS + slave battery data.
byte lastValidBtRx[20] = {0};  // same size as PACKET_SIZE
unsigned long previousPollMs=0;
unsigned long rxStartMs=0;
uint8_t rxIndex=0;
struct byteArrayRx {
	byte rxBuffer[PACKET_SIZE];
};
byteArrayRx received;
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
byte generateCRC(struct byteArrayRx data, int len)
{
  byte poly = 0xA7; // This is the byte representation of the CRC-8 Bluetooth polynomial, provided by Wikipedia, as is the method to get it.
  byte crc = 0x00; // Most CRC algorithms use an initial value of 0
  //This loops through the data array (except for the last value) and sets the crc value to the inverted remainder of the bitwise xor operation of the current crc and the inverse of the current value
  //this is mostly handled through the use of a crc lookup table, in order to minimize the realtime it takes.
  for(int i = 0; i < len - 1; i++){
    crc=crcLookup[crc^data.rxBuffer[i]];
  }
  return crc;
}
bool validateArray() {
	byte valid = generateCRC(received ,PACKET_SIZE);
    return (valid == received.rxBuffer[PACKET_SIZE - 1]);
}
void printPacket() {
    Serial.println("[BT] Packet received:");
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        Serial.print("  [");
        if (i < 10) Serial.print("0");
        Serial.print(i);
        Serial.print("] 0x");
        if (received.rxBuffer[i] < 0x10) Serial.print("0");
        Serial.println(received.rxBuffer[i], HEX);
    }
}
void flushRx() {
    while (Serial1.available()) Serial1.read();
}
void btUpdate() {
    unsigned long now = millis();
    switch (btState) {
        case BT_IDLE:
            if (now - previousPollMs >= POLL_INTERVAL) {
                previousPollMs = now;
                btRetryCount = 0;
                flushRx();
                rxIndex = 0;
                memset(received.rxBuffer, 0, PACKET_SIZE);
                #ifdef DIAG_BT
                    Serial.print(F("[BT] Polling slave → 0x01  (lastValid="));
                    Serial.print(btDataValidBool ? F("yes") : F("no"));
                    Serial.println(F(")"));
                #endif
                Serial1.write(BT_SLAVE_REQ);
                rxStartMs = now;
                btState = BT_WAIT_RESPONSE;
            }
            break;
        case BT_WAIT_RESPONSE:
            if (Serial1.available() > 0) {
                #ifdef DIAG_BT
                    Serial.print(F("[BT] First byte arrived after "));
                    Serial.print(now - rxStartMs);
                    Serial.println(F("ms → READING"));
                #endif
                btState = BT_READING;
            } else if (now - rxStartMs >= RX_TIMEOUT) {
                #ifdef DIAG_BT
                    Serial.println(F("[BT] !! TIMEOUT: No response from slave"));
                #endif
                btState = BT_ERROR;
            }
            break;
        case BT_READING:
            while (Serial1.available() && rxIndex < PACKET_SIZE) {
                received.rxBuffer[rxIndex++] = Serial1.read();
            }
            if (rxIndex >= PACKET_SIZE) {
                #ifdef DIAG_BT
                    Serial.print(F("[BT] Got all "));
                    Serial.print(PACKET_SIZE);
                    Serial.print(F(" bytes in "));
                    Serial.print(now - rxStartMs);
                    Serial.println(F("ms → COMPLETE"));
                #endif
                btState = BT_COMPLETE;
            } else if (now - rxStartMs >= RX_TIMEOUT) {
                #ifdef DIAG_BT
                    Serial.print(F("[BT] !! TIMEOUT: Only got "));
                    Serial.print(rxIndex);
                    Serial.print(F("/"));
                    Serial.print(PACKET_SIZE);
                    Serial.println(F(" bytes"));
                #endif
                btState = BT_ERROR;
            }
            break;
        case BT_COMPLETE:
            if (validateArray()) {
                #ifdef DIAG_BT
                    printPacket();
                    unsigned long alt = ((unsigned long)received.rxBuffer[11] << 24)
                                      | ((unsigned long)received.rxBuffer[12] << 16)
                                      | ((unsigned long)received.rxBuffer[13] <<  8)
                                      |  (unsigned long)received.rxBuffer[14];
                    Serial.print(F("[BT]   GNSS: "));
                    Serial.print(received.rxBuffer[0]); Serial.print(F(":"));
                    Serial.print(received.rxBuffer[1]); Serial.print(F(":"));
                    Serial.print(received.rxBuffer[2]); Serial.print(F(" UTC  "));
                    Serial.print(received.rxBuffer[3]); Serial.print(F("°"));
                    Serial.print(received.rxBuffer[4]); Serial.print(F("'"));
                    Serial.print(received.rxBuffer[5]); Serial.print(F("\" "));
                    Serial.print(received.rxBuffer[6] == 0 ? F("N") : F("S"));
                    Serial.print(F("  "));
                    Serial.print(received.rxBuffer[7]); Serial.print(F("°"));
                    Serial.print(received.rxBuffer[8]); Serial.print(F("'"));
                    Serial.print(received.rxBuffer[9]); Serial.print(F("\" "));
                    Serial.print(received.rxBuffer[10] == 0 ? F("E") : F("W"));
                    Serial.print(F("  Alt="));  Serial.print(alt);
                    Serial.print(received.rxBuffer[15] == 0 ? F("m") : F("ft"));
                    Serial.print(F("  SlaveBatt=["));
                    Serial.print(received.rxBuffer[16], HEX); Serial.print(F(","));
                    Serial.print(received.rxBuffer[17], HEX); Serial.println(F("]"));
                #endif
                memcpy(lastValidBtRx, received.rxBuffer, PACKET_SIZE);
                btDataValidBool = true;
                flightStatusByte |=  STATUS_BT_RX_SUCCESS;
                flightStatusByte &= ~STATUS_BT_RX_FAILURE;
            } else {
                #ifdef DIAG_BT
                    Serial.println(F("[BT] !! CRC or Echo FAIL — discarding"));
                    printPacket();
                #endif
                flightStatusByte |=  STATUS_BT_RX_FAILURE;
                flightStatusByte &= ~STATUS_BT_RX_SUCCESS;
            }
            btState = BT_IDLE;
            break;
        case BT_ERROR:
            flushRx();
            if (btRetryCount < MAX_RETRIES) {
                btRetryCount++;
                #ifdef DIAG_BT
                    Serial.print(F("[BT] Retry "));
                    Serial.print(btRetryCount + 1);
                    Serial.print(F("/"));
                    Serial.println(MAX_RETRIES + 1);
                #endif
                rxIndex = 0;
                memset(received.rxBuffer, 0, PACKET_SIZE);
                Serial1.write(BT_SLAVE_REQ);
                rxStartMs = now;
                btState = BT_WAIT_RESPONSE; 
            } else {
                #ifdef DIAG_BT
                    Serial.println(F("[BT] !! Slave unresponsive after all retries → IDLE"));
                #endif
                flightStatusByte |=  STATUS_BT_RX_FAILURE;
                flightStatusByte &= ~STATUS_BT_RX_SUCCESS;
                btState = BT_IDLE;            
            }
            break;
    }
}

// --------------------------------------------- BT MODEM --------------------------------------------- //
void setup()
{	
	Serial.begin(57600);//UART connect to the terminal emulator via the USB C cable

	//delay(100);//give time for UART to settle
	//set up I2C bus 0 which is Quiic and bus 1

    // BTMODEM SETUP
	Serial1.begin(38400);
  Serial.println();

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

	// Buzzer output
	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);
		
	introCountdown();//wait up to 10 seconds for TeraTerm to come up
	showVersions();

}

void loop()
{
	
		fdr.FDRoneTimeRun();//deals with FDR and GPS. When done, it sets MPMoneTimeRunBool true so core1 will run modem setup. It also determines if we are in simulator mode so this must run before any use is made of I2C bus
		
	// One-shot SEEPROM recovery — must run AFTER FDRoneTimeRun sets up the I2C bus.
	if (!seepromRecoveredBool) {
		seepromRecoveredBool = true;
		seepromRecoverOrInit();
	}
		
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
	// When the library signals a data dump was requested, output all SEEPROM records
	if (EEPROMjustOutputtedBool) {
		EEPROMjustOutputtedBool = false;
		dumpSEEPROMtoCSV();
	}
	// When the library signals Prepare-for-Launch ("P" command), erase SEEPROM for new flight.
	// The library sets this flag after resetting the EEPROM pointer and power-cycling.
	// Students are expected to use it to clear their own SEEPROM data.
	if (EEPROMjustClearedBool) {
		EEPROMjustClearedBool = false;
		seepromPreFlightErase();
	}

						#ifdef loopTimerDetails
						Serial.print(__FUNCTION__);
						Serial.print(F("(): "));		
						Serial.print(__LINE__);
						Serial.print(F(".    Time Stamp  "));	
						Serial.print (millis() - StartForTimeStampULong);
						Serial.println(F(" ms"));
						#endif
	
	if(BT_EQQUIPPED_BOOL) {
		btUpdate();
	}
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
	I2CstateMachines();     // runs AHT20 + Radio scanner state machines every loop
	task2SecSEEPROMwrite(); // 2-second task: write changed I2C sensor block to SEEPROM
	iridiumScheduler();     // 5-minute task: build TX array and transmit via Iridium
	buzzerUpdate();         // non-blocking buzzer pattern player — every loop pass

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

	// ---- PERIODIC SYSTEM SUMMARY ----
	#ifdef DIAG_SYSTEM
	if ((millis() - systemDiagTimerULong) >= SYSTEM_DIAG_INTERVAL_UL) {
		systemDiagTimerULong = millis();
		unsigned long uptimeSec = millis() / 1000UL;
		Serial.println(F(""));
		Serial.println(F("[SYS] ╔══════════════════════════════════════════════╗"));
		Serial.println(F("[SYS] ║         SYSTEM STATUS SUMMARY               ║"));
		Serial.println(F("[SYS] ╚══════════════════════════════════════════════╝"));
		Serial.print(F("[SYS] Uptime: "));
		Serial.print(uptimeSec / 60UL); Serial.print(F("m "));
		Serial.print(uptimeSec % 60UL); Serial.println(F("s"));
		// Status byte breakdown
		Serial.print(F("[SYS] flightStatus = 0b"));
		for (int b = 7; b >= 0; b--) Serial.print((flightStatusByte >> b) & 1);
		Serial.print(F("  → BT:"));
		Serial.print((flightStatusByte & STATUS_BT_RX_SUCCESS) ? F("OK") : ((flightStatusByte & STATUS_BT_RX_FAILURE) ? F("FAIL") : F("none")));
		Serial.print(F("  Burst:"));  Serial.print(burstLatchedBool ? F("YES") : F("no"));
		Serial.print(F("  Phase:"));
		if (flightStatusByte & STATUS_ASCENT)  Serial.print(F("ASCENT"));
		else if (flightStatusByte & STATUS_DESCENT) Serial.print(F("DESCENT"));
		else if (landingLatchedBool) Serial.print(F("LANDED"));
		else Serial.print(F("GROUND"));
		Serial.println();
		// SEEPROM
		long seepromUsed = seepromWriteAddressLong - SEEPROM_DATA_START_LONG;
		long seepromTotal = SEEPROM_MAX_ADDRESS_LONG - SEEPROM_DATA_START_LONG;
		Serial.print(F("[SYS] SEEPROM: "));
		Serial.print(seepromUsed / 16); Serial.print(F(" records, "));
		Serial.print((seepromUsed * 100L) / seepromTotal); Serial.print(F("% used"));
		if (seepromFullBool) Serial.print(F("  *** FULL ***"));
		Serial.println();
		// BT link
		Serial.print(F("[SYS] BT: lastValid="));
		Serial.print(btDataValidBool ? F("yes") : F("no"));
		Serial.print(F("  state="));
		const char* btNames[] = {"IDLE","WAIT","READ","DONE","ERR"};
		Serial.println(btNames[btState]);
		// Iridium
		Serial.print(F("[SYS] Iridium: state="));
		const char* ischedNames[] = {"IDLE","BUILD","TX","AWAIT","COOL"};
		Serial.print(ischedNames[iridiumSchedState]);
		Serial.print(F("  txOK="));     Serial.print(iridiumTxSuccessCountByte);
		Serial.print(F("  lastCode="));  Serial.print(iridiumCurrentStatusByte);
		Serial.print(F("  nextTx in "));
		if (iridiumSchedState == ISCHED_IDLE) {
			unsigned long remaining = 0;
			unsigned long elapsed = millis() - iridiumSchedTimerULong;
			if (elapsed < iridiumSchedDelayULong) remaining = (iridiumSchedDelayULong - elapsed) / 1000UL;
			Serial.print(remaining); Serial.print(F("s"));
		} else {
			Serial.print(F("(active)"));
		}
		Serial.println();
		// Sensors
		Serial.print(F("[SYS] Sensors: PSI="));
		Serial.print(latestPSI_Float, 2);
		Serial.print(F("  |G|="));       Serial.print(latestGmag_Float, 1);
		Serial.print(F("  AHT20="));     Serial.print(AHT20statusByte == successByte ? F("OK") : F("err"));
		Serial.println();
		Serial.println(F("[SYS] ──────────────────────────────────────────────"));
		Serial.println(F(""));
	}
	#endif
	
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
							Serial.println(ReturnCodeUInt);//since there are no failures for this command, I ignore return code
							
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
							Serial.println(ReturnCodeUInt);	
							
							//I got 323 SentPerformTransmitUInt						
							//I will wait until I see TransmitAndReceiveSuccessfulUInt
							while((ReturnCodeUInt <= StatusRangeMaxUInt) && (ReturnCodeUInt >= StatusRangeMinUInt))
							{//we are receiving status messages
									delay(100);//give time for MPM to work
									Serial.print("+");
									if(ReturnCodeUInt == idleUInt)
									{
										ReturnCodeUInt = 450;//just in case we get into this case but is not a good solution. I force ReturnCodeUInt out of the status range
										Serial.print("=");
									}
							}
							Serial.println();
						//final result now available
							if(ReturnCodeUInt <= FailureRangeMaxUInt )
							{//we had a failure so try again later
								Serial.println("failure from PerformTransmitUInt");
								Serial.print("ReturnCodeUInt is ");
								Serial.println(ReturnCodeUInt);
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
												Serial.print(F("ReturnCodeUInt = "));
												Serial.println(ReturnCodeUInt); 
												#endif
												
							Iridium(getReceivedDataUInt);
							delay(40);//give MPM time to work
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F(": "));	
							Serial.print(__LINE__);
							Serial.print(F("modem command = "));
							Serial.println(getReceivedDataUInt);
							Serial.print(F("ReturnCodeUInt = "));
							Serial.println(ReturnCodeUInt); //I got 401
							if(ReturnCodeUInt <= FailureRangeMaxUInt)
							{
						//we failed to get receive data. Try another session later.
								goto bailOut;
							}
							while((ReturnCodeUInt <= StatusRangeMaxUInt) && (ReturnCodeUInt >= StatusRangeMinUInt))
							{//we are receiving status messages
								delay(100);//give time for MPM to work
								Serial.print("&");
								if(ReturnCodeUInt == idleUInt)
								{
									ReturnCodeUInt = 450;//just in case we get into this case but is not a good solution. I force ReturnCodeUInt out of the status range 
									Serial.print("%");	
								}
							}						
										
							//I got ModemReadyForUseUInt 403 but was expecting data has been placed in receive array so go get it
							Serial.print("core ");Serial.print(rp2040.cpuid()); Serial.print(" ");Serial.print(__FUNCTION__);
							Serial.print(F("(): "));
							Serial.print(__LINE__);
							Serial.print(F("ReturnCodeUInt = "));
							Serial.println(ReturnCodeUInt); //just to confirm I got the success code I expected.	
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
							Serial.print(F(" 2 ReturnCodeUInt = "));
							Serial.println(ReturnCodeUInt);
							
						//verify we are back to idle
						
							#ifdef looparoundPrint
							Serial.print(F("3 ReturnCodeUInt = "));
							Serial.println(ReturnCodeUInt);
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
						Serial.println(ReturnCodeUInt);
						modemTransactionActiveBool = true;
					}
					if(modemTransactionActiveBool)
					{		
						//Iridium(statusUInt);
						
						if(ReturnCodeUInt == idleUInt)
						{
							modemTransactionActiveBool = false;//we are done
							Serial.println(F("MPM is now idle"));
						}else
						{
							if((ReturnCodeUInt <= StatusRangeMaxUInt) && (ReturnCodeUInt >= StatusRangeMinUInt))
							{//we are receiving status messages
								Serial.print(F("status return code = "));
								Serial.println(ReturnCodeUInt);

							}
						}
						
						if(ReturnCodeUInt <= FailureRangeMaxUInt)
						{//we are receiving failure messages
							modemTransactionActiveBool = false;//we are done	
							Serial.print(F("failure return code = "));
							Serial.println(ReturnCodeUInt);
							
						}
						
						if((ReturnCodeUInt <= SuccessRangeMaxUInt) && (ReturnCodeUInt >= SuccessRangeMinUInt))
						{//we are success status messages
							Serial.print(F("success return code = "));
							Serial.println(ReturnCodeUInt);

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
void I2CstateMachines()
{
/************************************************************************
We have one state machine for each Quiic client. Run only one machine at a time to avoid collisions on the Quiic bus. You can do this by having the last state of one machine point to the first state of the next machine.
**************************************************************************/

//state controller - sequences through each I2C state machine.
//AHT20 runs continuously on a 2-second self-restart cycle.
//When AHT20 is idle we kick off a new measurement cycle.


//=====================================================================================================================================================================


#ifdef AHTTaskManager
  if (AHT20StateByte == idleStateByte) {
    
    AHT20delayForAndThenReturnTo(2000, AHT20codeBlockA_Byte);  
    return;
  }
#endif


//=====================================================================================================================================================================

//end of state controller

//=====================================================================================================================================================================


byte AHT20writeResultByte = 0;
byte AHT20endTransResultByte = 0;

switch (AHT20StateByte) {
    
  case idleStateByte:
    break;

  case AHT20codeBlockA_Byte: {
      
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
    AHT20writeResultByte += Wire.write(AHT20_measurementSettings);  // if the write was good writeResultByte = 1 + 1 = 0, if it was bad writeResultByte = 0 + 0 = 0
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

      AHT20StateByte = idleStateByte;
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

      AHT20StateByte = idleStateByte;
      break;  // now that we failed and set the state machine to idle we get out of A and go to idle.
    }

    AHT20StateByte = AHT20codeBlockB_Byte; // We did it everything shouldve worked now we go to state B.
    break;  // if everything went well and we had no errors or hopfully no errors go through we get out of case A and go to case B.
  }

  case AHT20codeBlockB_Byte:
      
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

  case AHT20codeBlockC_Byte: {
      
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

  case AHT20codeBlockD_Byte: {
      
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
			
		if (Wire.available() >= 6) {  // check if we recived all 6 bytes.

      for (int i = 0; i < 6; i++) {  // than we will put all the bytes in the data array.
        
        AHT20data[i] = Wire.read();
      }
  
      if (AHT20data[0] & 0x80) {  // check to see if the sensor is still busy.
          
        AHT20statusByte = notReadyYetByte;
  			AHT20StateByte = AHT20codeBlockE_Byte;  // or do a short delay and then return to D
        break;
      }

      AHT20byteSplitter();  // if we get a Good reading than we get to split the bytes to have 3 bytes of hum and 3 of temp
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
				Serial.print(F("AHT20data[0] (Hum1) = "));
				Serial.println(AHT20data[0]);
				Serial.print(F("AHT20data[1] (Hum2) = "));
				Serial.println(AHT20data[1]);
				Serial.print(F("AHT20data[2] (Hum3) = "));
			  Serial.println(AHT20data[2]);
				Serial.print(F("AHT20data[3] (Temp1) = "));
				Serial.println(AHT20data[3]);
				Serial.print(F("AHT20data[4] (Temp2) = "));
				Serial.println(AHT20data[4]);
				Serial.print(F("AHT20data[5] (Temp3) = "));
				Serial.println(AHT20data[5]);
				Serial.print(F("status = "));
				Serial.println(AHT20record[6]);
			#endif 

      AHT20StateByte = idleStateByte;
    }
    
    else {  // if we dont have 6 bytes yet go to Case E
      
      AHT20StateByte = AHT20codeBlockE_Byte;
    }
      
    break;
  }

  case AHT20codeBlockE_Byte: { // we are in time out and hopefully we get the 6 bytes.
      
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
      AHT20record[0] = AHT20statusByte;  // status byte first
      AHT20record[1] = 0;
      AHT20record[2] = 0;
      AHT20record[3] = 0;
      AHT20record[4] = 0;
      AHT20record[5] = 0;
      AHT20record[6] = 0;

      AHT20StateByte = idleStateByte;

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

    default:
      AHT20StateByte = idleStateByte;
    break;		
}



  // Run the radio scanner state machine
	#ifdef ENABLE_TEA5767
	RadioStateMachine();
	#endif
}  // end of I2CstateMachines()



//=====================================================================================================================================================================
// ============================================================
//   updateFlightStatus()
//
//   Reads the pressure sensor (port 2) and accelerometer (ports 5-6),
//   converts to physical units using calibration equations, and
//   updates the flight phase bits in flightStatusByte.
//
//   DETECTION LOGIC (validated against Fall 2025 flight data):
//
//   ASCENT:   Pressure is dropping (dPSI < -0.02 per 2 s) AND
//             we are airborne (PSI < 13.5).  Clears each cycle.
//
//   BURST:    |G| exceeds 5 g while airborne.  LATCHED — once the
//             balloon pops, this bit stays set for the rest of the
//             flight.  During ascent |G| never exceeds 1.1 g, so
//             5 g is an unambiguous threshold.  Parachute descent
//             also produces 5-7 g, but burst is already latched
//             by then and won't re-trigger.
//
//   DESCENT:  Pressure is rising (dPSI > +0.02 per 2 s) AND burst
//             has been latched.  Clears each cycle.  Requiring the
//             burst latch prevents a false descent flag from being
//             set during pre-launch pressure fluctuations.
//
//   LANDING:  After burst, pressure is near ground level (> 13.5 PSI),
//             pressure is stable (|dPSI| < 0.05), AND |G| < 1.0
//             (payload has stopped swinging under the parachute).
//             Must hold for 5 consecutive 2-second readings (10 s).
//             LATCHED once triggered.  The |G| < 1.0 check is
//             critical: during parachute descent at low altitude,
//             pressure is already near ground level but |G| is 5-7 g.
//
//   BT bits (0-1) are NOT touched here; they are set by btUpdate().
//
//   Call once per 2-second task cycle from task2SecSEEPROMwrite().
// ============================================================
void updateFlightStatus() {

    // ---- Read analog sensors and convert to physical units ----
    float vPressure = fdr.ADCconvertToVoltage(fdr.ADCread(ADC_PORT_PRESSURE));
    float vAccelX   = fdr.ADCconvertToVoltage(fdr.ADCread(ADC_PORT_ACCEL_X));
    float vAccelY   = fdr.ADCconvertToVoltage(fdr.ADCread(ADC_PORT_ACCEL_Y));

    float psiNow = (PRESSURE_SCALE_PSIPV * vPressure) + PRESSURE_OFFSET_PSI;
    float gX     = (vAccelX - ACCEL_X_OFFSET_V) / ACCEL_X_SCALE_VPG;
    float gY     = (vAccelY - ACCEL_Y_OFFSET_V) / ACCEL_Y_SCALE_VPG;
    float gMag   = sqrtf(gX * gX + gY * gY);

    // Store for diag access
    latestGx_Float   = gX;
    latestGy_Float   = gY;
    latestGmag_Float = gMag;
    latestPSI_Float  = psiNow;

    #ifdef DIAG_ANALOG
        // Raw sensor voltages + converted values every 2 seconds
        float vIntTemp  = fdr.ADCconvertToVoltage(fdr.ADCread(ADC_PORT_INT_TEMP));
        float vExtTemp  = fdr.ADCconvertToVoltage(fdr.ADCread(ADC_PORT_EXT_TEMP));
        float intTempC  = (vIntTemp - INT_TEMP_OFFSET_V) / TEMP_SCALE_VPDC;
        float extTempC  = (vExtTemp - EXT_TEMP_OFFSET_V) / TEMP_SCALE_VPDC;
        float battRaw   = analogRead(29) * 0.00322f;
        float battActual = battRaw / BATTERY_CAL_DIVISOR;
        Serial.print(F("[ANALOG] PSI="));  Serial.print(psiNow, 2);
        Serial.print(F("  Gx="));          Serial.print(gX, 2);
        Serial.print(F("  Gy="));          Serial.print(gY, 2);
        Serial.print(F("  |G|="));         Serial.print(gMag, 2);
        Serial.print(F("  IntT="));        Serial.print(intTempC, 1); Serial.print(F("C"));
        Serial.print(F("  ExtT="));        Serial.print(extTempC, 1); Serial.print(F("C"));
        Serial.print(F("  Batt="));        Serial.print(battActual, 2); Serial.println(F("V"));
    #endif

    // ---- Pressure sanity guard ----
    if (psiNow < 0.0f || psiNow > 16.0f) {
        #ifdef DIAG_PHASE
            Serial.print(F("[PHASE] !! PSI out of range: "));
            Serial.print(psiNow, 2);
            Serial.println(F(" — skipping this cycle"));
        #endif
        return;
    }

    // ---- Compute pressure trend ----
    float deltaPSI = 0.0f;
    if (previousPSI_ValidBool) {
        deltaPSI = psiNow - previousPSI_Float;
    }
    previousPSI_Float     = psiNow;
    previousPSI_ValidBool = true;

    bool airborne = (psiNow < GROUND_PSI_THRESHOLD);

    flightStatusByte &= ~(STATUS_ASCENT | STATUS_DESCENT);

    // ---- ASCENT ----
    if (airborne && deltaPSI < -ASCENT_DELTA_PSI) {
        flightStatusByte |= STATUS_ASCENT;
    }

    // ---- BURST ----
    if (!burstLatchedBool && airborne && gMag >= BURST_G_THRESHOLD) {
        burstLatchedBool = true;
        #ifdef DIAG_PHASE
            Serial.println(F("[PHASE] ========================================"));
            Serial.print(F("[PHASE] *** BURST DETECTED ***  |G|="));
            Serial.print(gMag, 1);
            Serial.print(F("g  PSI="));  Serial.print(psiNow, 2);
            Serial.print(F("  Gx="));    Serial.print(gX, 1);
            Serial.print(F("  Gy="));    Serial.println(gY, 1);
            Serial.println(F("[PHASE] ========================================"));
        #endif
    }
    if (burstLatchedBool) {
        flightStatusByte |= STATUS_BALLOON_BURST;
    }

    // ---- DESCENT ----
    if (burstLatchedBool && deltaPSI > DESCENT_DELTA_PSI) {
        flightStatusByte |= STATUS_DESCENT;
    }

    // ---- LANDING ----
    if (burstLatchedBool && !landingLatchedBool) {
        bool nearGround    = (psiNow >= GROUND_PSI_THRESHOLD);
        bool pressureStable = (fabsf(deltaPSI) < LANDING_PSI_STABLE);
        bool stoppedSwinging = (gMag < LANDING_G_THRESHOLD);

        if (nearGround && pressureStable && stoppedSwinging) {
            landingStableCounterByte++;
            if (landingStableCounterByte >= LANDING_STABLE_COUNT) {
                landingLatchedBool = true;
                #ifdef DIAG_PHASE
                    Serial.println(F("[PHASE] ========================================"));
                    Serial.println(F("[PHASE] *** LANDING DETECTED ***"));
                    Serial.println(F("[PHASE] ========================================"));
                #endif
            }
            #ifdef DIAG_PHASE
            else {
                Serial.print(F("[PHASE] Landing stable count: "));
                Serial.print(landingStableCounterByte);
                Serial.print(F("/"));  Serial.println(LANDING_STABLE_COUNT);
            }
            #endif
        } else {
            if (landingStableCounterByte > 0) {
                #ifdef DIAG_PHASE
                    Serial.print(F("[PHASE] Landing counter reset (was "));
                    Serial.print(landingStableCounterByte);
                    Serial.print(F(") — gnd="));    Serial.print(nearGround);
                    Serial.print(F(" stable="));     Serial.print(pressureStable);
                    Serial.print(F(" noSwing="));    Serial.println(stoppedSwinging);
                #endif
            }
            landingStableCounterByte = 0;
        }
    }
    if (landingLatchedBool) {
        flightStatusByte |= STATUS_LANDING;
    }

    #ifdef DIAG_PHASE
        Serial.print(F("[PHASE] PSI="));   Serial.print(psiNow, 2);
        Serial.print(F("  dPSI="));        Serial.print(deltaPSI, 4);
        Serial.print(F("  |G|="));         Serial.print(gMag, 1);
        Serial.print(F("  air="));         Serial.print(airborne);
        Serial.print(F("  burst="));       Serial.print(burstLatchedBool);
        Serial.print(F("  land="));        Serial.print(landingLatchedBool);
        Serial.print(F("  status=0b"));
        for (int b = 7; b >= 0; b--) Serial.print((flightStatusByte >> b) & 1);
        Serial.println();
    #endif
}

//=====================================================================================================================================================================
// ============================================================
//   seepromUpdateHeader()
//
//   Writes the 8-byte persistent header to SEEPROM address 0-7.
//   Called after every data write so that a power-loss event
//   loses at most one 2-second record.
// ============================================================
void seepromUpdateHeader() {
    fdr.WriteSEEPROM(0, SEEPROM_MAGIC_0);
    fdr.WriteSEEPROM(1, SEEPROM_MAGIC_1);
    fdr.WriteSEEPROM(2, (byte)( seepromWriteAddressLong        & 0xFF));
    fdr.WriteSEEPROM(3, (byte)((seepromWriteAddressLong >>  8) & 0xFF));
    fdr.WriteSEEPROM(4, (byte)((seepromWriteAddressLong >> 16) & 0xFF));
    fdr.WriteSEEPROM(5, (byte)((seepromWriteAddressLong >> 24) & 0xFF));
    fdr.WriteSEEPROM(6, seepromRecordCountByte);
    byte latchByte = (burstLatchedBool ? 0x01 : 0x00)
                   | (landingLatchedBool ? 0x02 : 0x00);
    fdr.WriteSEEPROM(7, latchByte);
    #ifdef DIAG_SEEPROM_HEADER
        Serial.print(F("[SEEPROM-HDR] Updated: addr="));
        Serial.print(seepromWriteAddressLong);
        Serial.print(F("  rec="));     Serial.print(seepromRecordCountByte);
        Serial.print(F("  latch=0b")); Serial.println(latchByte, BIN);
    #endif
}

//=====================================================================================================================================================================
void seepromRecoverOrInit() {
    byte m0 = (byte)fdr.ReadSEEPROM(0);
    byte m1 = (byte)fdr.ReadSEEPROM(1);

    #ifdef DIAG_SEEPROM_HEADER
        Serial.print(F("[SEEPROM-HDR] Header read: magic=[0x"));
        if (m0 < 0x10) Serial.print('0');
        Serial.print(m0, HEX);
        Serial.print(F(", 0x"));
        if (m1 < 0x10) Serial.print('0');
        Serial.print(m1, HEX);
        Serial.print(F("]  expected=[0x"));
        Serial.print(SEEPROM_MAGIC_0, HEX);
        Serial.print(F(", 0x"));
        Serial.print(SEEPROM_MAGIC_1, HEX);
        Serial.println(F("]"));
    #endif

    if (m0 == SEEPROM_MAGIC_0 && m1 == SEEPROM_MAGIC_1) {
        long addr = 0;
        addr |= ((long)(byte)fdr.ReadSEEPROM(2));
        addr |= ((long)(byte)fdr.ReadSEEPROM(3)) <<  8;
        addr |= ((long)(byte)fdr.ReadSEEPROM(4)) << 16;
        addr |= ((long)(byte)fdr.ReadSEEPROM(5)) << 24;

        if (addr >= SEEPROM_DATA_START_LONG && addr <= SEEPROM_MAX_ADDRESS_LONG && (addr % 16 == 0)) {
            seepromWriteAddressLong = addr;
        } else {
            seepromWriteAddressLong = SEEPROM_DATA_START_LONG;
            #ifdef DIAG_SEEPROM_HEADER
                Serial.print(F("[SEEPROM-HDR] !! Stored address invalid ("));
                Serial.print(addr);
                Serial.println(F(") — resetting to 16"));
            #endif
        }

        seepromRecordCountByte = (byte)fdr.ReadSEEPROM(6);
        byte latchByte = (byte)fdr.ReadSEEPROM(7);
        burstLatchedBool   = (latchByte & 0x01) != 0;
        landingLatchedBool = (latchByte & 0x02) != 0;
        if (burstLatchedBool)   flightStatusByte |= STATUS_BALLOON_BURST;
        if (landingLatchedBool) flightStatusByte |= STATUS_LANDING;
        if ((seepromWriteAddressLong + 16) > SEEPROM_MAX_ADDRESS_LONG) {
            seepromFullBool = true;
        }

        Serial.println(F("[SEEPROM] *** POWER RECOVERY ***"));
        Serial.print(F("[SEEPROM]   Resuming at addr="));
        Serial.print(seepromWriteAddressLong);
        Serial.print(F("  rec#="));     Serial.print(seepromRecordCountByte);
        long recordsSoFar = (seepromWriteAddressLong - SEEPROM_DATA_START_LONG) / 16;
        Serial.print(F("  records="));  Serial.println(recordsSoFar);
        Serial.print(F("[SEEPROM]   latch=0b"));  Serial.print(latchByte, BIN);
        Serial.print(F("  burst="));   Serial.print(burstLatchedBool);
        Serial.print(F("  land="));    Serial.print(landingLatchedBool);
        Serial.print(F("  full="));    Serial.println(seepromFullBool);
    } else {
        // No valid header — start fresh
        seepromWriteAddressLong = SEEPROM_DATA_START_LONG;
        seepromRecordCountByte  = 0;
        seepromFullBool         = false;
        Serial.println(F("[SEEPROM] No valid header — starting fresh at address 16."));
    }
}

//=====================================================================================================================================================================
// ============================================================
//   seepromPreFlightErase()
//
//   Called when the library signals that the user pressed "P"
//   (Prepare for Launch), which sets EEPROMjustClearedBool.
//
//   This invalidates the SEEPROM header by overwriting the
//   magic bytes with 0xFF, then resets all write-state variables.
//   We do NOT need to erase the entire 128 KB — the old data will
//   be overwritten naturally as new records are written, and
//   dumpSEEPROMtoCSV() only reads up to seepromWriteAddressLong
//   so it will never see stale data from a previous flight.
//
//   This also clears the latched flight status flags because
//   a new flight is beginning.
// ============================================================
void seepromPreFlightErase() {
    // Invalidate header — 2 byte writes, takes ~12 ms
    fdr.WriteSEEPROM(0, 0xFF);
    fdr.WriteSEEPROM(1, 0xFF);

    // Reset all SEEPROM state
    seepromWriteAddressLong = SEEPROM_DATA_START_LONG;
    seepromRecordCountByte  = 0;
    seepromFullBool         = false;
    memset(previousI2Cblock, 0, 16);

    // Reset flight phase latches for the new flight
    burstLatchedBool        = false;
    landingLatchedBool      = false;
    landingStableCounterByte = 0;
    previousPSI_ValidBool   = false;
    flightStatusByte        = 0;

    Serial.println(F("[SEEPROM] Pre-flight erase complete. Ready for new flight."));
}

//=====================================================================================================================================================================
// ============================================================
//   assembleI2Cblock()
//   Fills latestI2Cblock[16] with the current sensor snapshot
//   using the documented record format above.
//   Call this just before comparing/writing to SEEPROM.
// ============================================================
void assembleI2Cblock() {
    latestI2Cblock[0]  = seepromRecordCountByte;       // sequence number (wraps 0-255)
    // AHT20 sensor data (raw 20-bit values split across 3 bytes each)
    latestI2Cblock[1]  = AHT20record[0];               // humidity byte 0 (MSB nibble)
    latestI2Cblock[2]  = AHT20record[1];               // humidity byte 1
    latestI2Cblock[3]  = AHT20record[2];               // humidity byte 2 (LSB)
    latestI2Cblock[4]  = AHT20record[3];               // temp byte 0 (MSB nibble)
    latestI2Cblock[5]  = AHT20record[4];               // temp byte 1
    latestI2Cblock[6]  = AHT20record[5];               // temp byte 2 (LSB)
    latestI2Cblock[7]  = AHT20record[6];               // AHT20 status (2=OK 3=fail 4=timeout)
    // TEA5767 Radio — last station that locked + signal quality
		// Current Station Idx
		// Current Intensity 
		// Approximatly 19 Full Sweeps With 16Bytes * 2min Write 
    latestI2Cblock[8]  = (byte)(Last_scanned_station_idx >> 8);   // station idx MSB
    latestI2Cblock[9]  = (byte)(Last_scanned_station_idx & 0xFF); // station idx LSB
    latestI2Cblock[10] = Last_scanned_station_signal_strength;    // signal level 0-15
    latestI2Cblock[11] = (byte)RadioStereoBool;                   
    latestI2Cblock[12] = flightStatusByte;             // packed flight status bits
    latestI2Cblock[13] = 0x00;                         // reserved
    latestI2Cblock[14] = 0x00;                         // reserved
    latestI2Cblock[15] = 0x00;                         // reserved
}
//=====================================================================================================================================================================
// ============================================================
//   task2SecSEEPROMwrite()
//   Task Scheduler — fires every 2 seconds.
//   Assembles the 16-byte sensor record, compares it with the
//   previously written record.  If data has changed, writes all
//   16 bytes to SEEPROM and advances seepromWriteAddressLong.
//   The sequence counter in byte [0] always increments so even
//   if only the timestamp changed the record will be written.
//   Call once per loop() iteration.
// ============================================================
void task2SecSEEPROMwrite() {
    if ((millis() - task2SecTimerULong) < TASK_2SEC_INTERVAL_UL) {
        return;  // not time yet
    }
    task2SecTimerULong = millis();  // reset 2-second timer
    // --- SEEPROM overflow guard ---
    // Each record is 16 bytes.  The write would touch addresses
    // seepromWriteAddressLong through seepromWriteAddressLong+15.
    // If the last byte would exceed the chip, stop writing and preserve
    // the most recent data that was successfully stored.
    if (seepromFullBool) {
        return;  // already full — preserve last written data
    }
    if ((seepromWriteAddressLong + 16) > SEEPROM_MAX_ADDRESS_LONG) {
        seepromFullBool = true;
        #ifdef DIAG_SEEPROM
            Serial.println(F("[SEEPROM] ========================================"));
            Serial.println(F("[SEEPROM] *** FULL — no more room ***"));
            Serial.print(F("[SEEPROM] Total records: "));
            Serial.println((seepromWriteAddressLong - SEEPROM_DATA_START_LONG) / 16);
            Serial.println(F("[SEEPROM] ========================================"));
        #endif
        return;
    }
    seepromRecordCountByte++;
    updateFlightStatus();
    assembleI2Cblock();
    if (memcmp(latestI2Cblock, previousI2Cblock, 16) == 0) {
        return;
    }
    for (byte i = 0; i < 16; i++) {
        fdr.WriteSEEPROM(seepromWriteAddressLong + i, latestI2Cblock[i]);
    }
    memcpy(previousI2Cblock, latestI2Cblock, 16);
    seepromWriteAddressLong += 16;
    seepromUpdateHeader();
    #ifdef DIAG_SEEPROM
        long usedBytes  = seepromWriteAddressLong - SEEPROM_DATA_START_LONG;
        long totalBytes = SEEPROM_MAX_ADDRESS_LONG - SEEPROM_DATA_START_LONG;
        long recCount   = usedBytes / 16;
        long pctUsed    = (usedBytes * 100L) / totalBytes;
        Serial.print(F("[SEEPROM] Rec#"));
        Serial.print(seepromRecordCountByte);
        Serial.print(F("  addr="));
        Serial.print(seepromWriteAddressLong - 16);
        Serial.print(F("  used="));
        Serial.print(pctUsed);
        Serial.print(F("%  ("));
        Serial.print(recCount);
        Serial.print(F("/"));
        Serial.print(totalBytes / 16);
        Serial.print(F(" recs)  AHT="));
        Serial.print(latestI2Cblock[7]);
        Serial.print(F("  radioIdx="));
        Serial.print((uint16_t)((latestI2Cblock[8] << 8) | latestI2Cblock[9]));
        Serial.print(F("  sig="));
        Serial.print(latestI2Cblock[10]);
        Serial.print(F("  flight=0b"));
        for (int b = 7; b >= 0; b--) Serial.print((latestI2Cblock[12] >> b) & 1);
        Serial.println();
    #endif
}
//=====================================================================================================================================================================
// ============================================================
//   getIridiumRetryDelayMs()
//
//   Given a modem return code, returns how many milliseconds
//   to wait before the next transmit attempt.
//   Returns 0 for any satellite-level success code (0-4) —
//   caller resets to the nominal 5-minute interval instead.
// ============================================================
unsigned long getIridiumRetryDelayMs(unsigned int code) { // Thanks Daniel 
    if (code <= 4) return 0;  // satellite success — use 5-min interval
    switch (code) {
        case 10: return  30000UL;  // call did not complete in allowed time
        case 11: return  60000UL;  // GSS MO queue full
        case 12: return  60000UL;  // too many segments
        case 13: return  30000UL;  // session did not complete
        case 14: return  60000UL;  // invalid segment size
        case 15: return 120000UL;  // access denied
        case 16: return 120000UL;  // ISU locked
        case 17: return  30000UL;  // gateway not responding (local timeout)
        case 18: return  15000UL;  // connection lost / RF drop — retry quickly
        case 19: return  30000UL;  // link failure (protocol error)
        case 32: return  30000UL;  // no network service
        case 33: return  60000UL;  // antenna fault
        case 34: return 120000UL;  // radio disabled
        case 35: return  10000UL;  // ISU busy — retry soon
        case 36: return 180000UL;  // must wait 3 min since last registration
        case 37: return 120000UL;  // SBD service temporarily disabled
        case 38: return  60000UL;  // traffic management period
        case 64: return  60000UL;  // band violation
        case 65: return  60000UL;  // PLL lock failure (hardware)
        default: break;
    }
    // Reserved failure ranges: 5-8, 20-31, 39-63
    if ((code >= 5  && code <= 8)  ||
        (code >= 20 && code <= 31) ||
        (code >= 39 && code <= 63)) return 30000UL;
    // Internal library failure codes 100-299m
    if (code >= 100 && code <= 299) return 15000UL;
    return 30000UL;  // unknown — default 15 s
}

//=====================================================================================================================================================================
// ============================================================
//   buildIridiumTxArray()
//
//   Assembles all 45 bytes of IridiumTransmitDataByte[]
//   from live data sources immediately before transmitting.
//
//   TX Array layout (45 bytes, indices 0-44):
//   [0]      Flight status byte (6 packed bits — see #defines)
//   [1..16]  GNSS data from SLAVE via BT (lastValidBtRx[1..16])
//   [17..32] Master I2C sensors (latestI2Cblock[0..15])
//   [33]     Reserved (0x00)
//   [34]     Master battery MSB  — analogRead(29) raw ADC counts
//   [35]     Master battery LSB
//   [36]     Internal temp MSB   — ADC port 4 (Silver wire)
//   [37]     Internal temp LSB
//   [38]     External temp MSB   — ADC port 3 (Gold wire)
//   [39]     External temp LSB
//   [40]     Slave battery MSB  (lastValidBtRx[17])
//   [41]     Slave battery LSB  (lastValidBtRx[18])
//   [42]     Current modem return code  (iridiumCurrentStatusByte)
//   [43]     Previous modem return code (iridiumPreviousStatusByte)
//   [44]     Cumulative successful TX count (iridiumTxSuccessCountByte)
//
//   Dashboard decoding for analog bytes:
//     Battery voltage:  Vr = raw * 0.00322;  Va = Vr / 0.971
//     Internal temp °C: Vm = fdr.ADCconvertToVoltage(raw); T = (Vm - 0.502) / 0.011
//     External temp °C: Vm = fdr.ADCconvertToVoltage(raw); T = (Vm - 0.491) / 0.011
// ============================================================
void buildIridiumTxArray() {
    // [0] Flight status byte
    IridiumTransmitDataByte[0] = flightStatusByte;

    // [0..15] GNSS data from SLAVE via BT — read from the "last known good" buffer.
    // lastValidBtRx[0..15] = GNSS data block
    // lastValidBtRx[16..17] = slave battery
    // lastValidBtRx[18] = CRC (not transmitted)
    // If BT has never succeeded, lastValidBtRx is all zeros and
    // STATUS_BT_RX_FAILURE will be set in flightStatusByte — the dashboard
    // must check that bit before trusting the GNSS fields.
    for (byte i = 0; i < 16; i++) {
        IridiumTransmitDataByte[1 + i] = lastValidBtRx[i];
    }
    // [17..32] Latest I2C sensor block (AHT20 + Radio) — no SEEPROM readback needed
    for (byte i = 0; i < 16; i++) {
        IridiumTransmitDataByte[17 + i] = latestI2Cblock[i];
    }
    // [33] Reserved padding
    IridiumTransmitDataByte[33] = 0x00;
    // [34..39] Master board analog readings (raw int split MSB/LSB)
    // Battery uses the RP2040's built-in ADC on pin 29, not the external ADC.
    int adcBatt    = analogRead(29);                     // master battery (pin 29)
    int adcIntTemp = fdr.ADCread(ADC_PORT_INT_TEMP);     // internal temp (port 4)
    int adcExtTemp = fdr.ADCread(ADC_PORT_EXT_TEMP);     // external temp (port 3)
    IridiumTransmitDataByte[34] = (byte)((adcBatt    >> 8) & 0xFF);
    IridiumTransmitDataByte[35] = (byte)( adcBatt          & 0xFF);
    IridiumTransmitDataByte[36] = (byte)((adcIntTemp >> 8) & 0xFF);
    IridiumTransmitDataByte[37] = (byte)( adcIntTemp       & 0xFF);
    IridiumTransmitDataByte[38] = (byte)((adcExtTemp >> 8) & 0xFF);
    IridiumTransmitDataByte[39] = (byte)( adcExtTemp       & 0xFF);
    // [40..41] Slave battery bytes from last valid BT receive packet
    IridiumTransmitDataByte[40] = lastValidBtRx[16];
    IridiumTransmitDataByte[41] = lastValidBtRx[17];
    // [42..43] Rolling modem return code history
    IridiumTransmitDataByte[42] = iridiumCurrentStatusByte;
    IridiumTransmitDataByte[43] = iridiumPreviousStatusByte;
    // [44] Cumulative successful transmission count
    IridiumTransmitDataByte[44] = iridiumTxSuccessCountByte;
}

//=====================================================================================================================================================================
// ============================================================
//   iridiumScheduler()
//
//   Non-blocking state machine that manages the Iridium
//   5-minute transmit cycle.  Call every loop().
//
//   Normal flow:
//     ISCHED_IDLE      → timer expires  → ISCHED_BUILD
//     ISCHED_BUILD     → array packed   → ISCHED_TRANSMIT
//     ISCHED_TRANSMIT  → modem kicked   → ISCHED_AWAIT
//     ISCHED_AWAIT     → result ready   → ISCHED_COOLDOWN
//     ISCHED_COOLDOWN  → wait done      → ISCHED_IDLE
//
//   On success the cooldown is the full 5-minute interval.
//   On failure the cooldown is the response-code-driven retry
//   delay from getIridiumRetryDelayMs().
//
//   Enable //#define IridiumSchedDiag for serial diagnostics.
// ============================================================
//Excessive Loop 1 time when 269 (we can ignore this excessive loop time)
void iridiumScheduler() {
    if (!ModemBool) return;  // modem not fitted
    switch (iridiumSchedState) {
        case ISCHED_IDLE:
            if ((millis() - iridiumSchedTimerULong) >= iridiumSchedDelayULong) {
                #ifdef DIAG_IRIDIUM
                    Serial.print(F("[ISCHED] Timer expired after "));
                    Serial.print(iridiumSchedDelayULong / 1000UL);
                    Serial.println(F("s → BUILD"));
                #endif
                iridiumSchedState = ISCHED_BUILD;
            }
            break;

        case ISCHED_BUILD:
            assembleI2Cblock();     // refresh I2C snapshot
            buildIridiumTxArray();  // pack all 45 bytes

            #ifdef DIAG_IRIDIUM
                Serial.println(F("[ISCHED] TX array built → TRANSMIT"));
                Serial.print(F("[ISCHED]   statusByte=0b"));
                for (int b=7;b>=0;b--) Serial.print((IridiumTransmitDataByte[0]>>b)&1);
                Serial.print(F("  TxCount="));
                Serial.println(iridiumTxSuccessCountByte);
            #endif
            #ifdef DIAG_TX_ARRAY
                Serial.print(F("[TX] "));
                for (byte i = 0; i < 45; i++) {
                    if (IridiumTransmitDataByte[i] < 0x10) Serial.print('0');
                    Serial.print(IridiumTransmitDataByte[i], HEX);
                    Serial.print(' ');
                    if (i == 0 || i == 16 || i == 32 || i == 39 || i == 41 || i == 43) Serial.print(F("| "));
                }
                Serial.println();
                Serial.println(F("[TX]  [0]=status | [1-16]=GNSS | [17-32]=I2C | [33]=rsv | [34-39]=ADC | [40-41]=slaveBatt | [42-43]=modem | [44]=txCnt"));
            #endif

            iridiumSchedState = ISCHED_TRANSMIT;
            break;

        case ISCHED_TRANSMIT:
            #ifdef DIAG_IRIDIUM
                Serial.println(F("[ISCHED] Issuing PerformTransmit to modem → AWAIT"));
            #endif
            Iridium(PerformTransmitUInt);
            iridiumSchedState = ISCHED_AWAIT;
            ADRIAN
            break;

        case ISCHED_AWAIT: {
			// -- Cannot read from ReturnCodeUInt Directly!! -- // Read from Iridium(StatusUint) Why causing excessive loop time??
            unsigned int code = Iridium(8);
            // Status codes 300-399 mean the modem is still working — come back next loop
            if ((code >= StatusRangeMinUInt) && (code <= StatusRangeMaxUInt)) break;
            ADRIAN
            // Final result is in — shift status history
            iridiumPreviousStatusByte = iridiumCurrentStatusByte;
            iridiumCurrentStatusByte  = (byte)(code & 0xFF);
            bool txSuccess = (code >= SuccessRangeMinUInt && code <= SuccessRangeMaxUInt)
                             || (getIridiumRetryDelayMs(code) == 0);
            if (txSuccess) {
                iridiumTxSuccessCountByte++;
                buzzerTxChirpPending = true;  // trigger a quick triple chirp
                // NOTE: BT RX status bits are set by btUpdate(), NOT here.
                // The modem result does not affect BT success/failure flags.
                iridiumSchedDelayULong = IRIDIUM_NOMINAL_INTERVAL_UL;  // reset to 5 min
                #ifdef DIAG_IRIDIUM
                    Serial.print(F("[ISCHED] SUCCESS  code="));
                    Serial.print(code);
                    Serial.print(F("  TxCount="));
                    Serial.println(iridiumTxSuccessCountByte);
                #endif
            } else {
                // NOTE: BT RX status bits are set by btUpdate(), NOT here.
                iridiumSchedDelayULong = getIridiumRetryDelayMs(code);

                #ifdef DIAG_IRIDIUM
                    Serial.print(F("[ISCHED] FAILURE  code="));
                    Serial.print(code);
                    Serial.print(F("  retry in "));
                    Serial.print(iridiumSchedDelayULong / 1000UL);
                    Serial.println(F(" s"));
                #endif
            }

            iridiumSchedTimerULong = millis();
            iridiumSchedState = ISCHED_COOLDOWN;
            break;
        }

        case ISCHED_COOLDOWN:
            if ((millis() - iridiumSchedTimerULong) >= iridiumSchedDelayULong) {
                iridiumSchedTimerULong = 0;  // force ISCHED_IDLE to fire immediately
                iridiumSchedDelayULong = 0;
                iridiumSchedState = ISCHED_IDLE;
            }
            break;
    }
}
//=====================================================================================================================================================================
// ============================================================
//   dumpSEEPROMtoCSV()
//
//   Reads every 16-byte record stored in SEEPROM and prints
//   it to Serial as a CSV stream.  Paste the Serial Monitor
//   output directly into Excel (Data → From Text/CSV) for
//   immediate analysis.
//
//   WHEN TO CALL:
//     Triggered by the existing EEPROMjustOutputtedBool flag
//     that the library sets when the user requests a data dump
//     (e.g. keypress 'D' in the terminal menu).
//     Call from loop() after fdr.PutInLoop() like:
//
//       if (EEPROMjustOutputtedBool) {
//           EEPROMjustOutputtedBool = false;
//           dumpSEEPROMtoCSV();
//       }
//
//   OUTPUT FORMAT (one header row + one data row per record):
//
//   RecordNum, GPS_HH, GPS_MM, GPS_SS, GPS_Time,
//   Hum_RH_pct, Temp_C, Temp_F, AHT20_Status,
//   Radio_Station_MHz, Radio_Signal, Radio_Stereo,
//   FlightStatus, BT_RX_OK, BT_RX_FAIL,
//   Balloon_Burst, Descent, Ascent, Landing
//
//   AHT20_Status key:  2 = success, 3 = failed, 4 = timed out
//   Radio_Signal:      0-15  (4-bit ADC from TEA5767)
//   Radio_Stereo:      1 = stereo lock, 0 = mono
// ============================================================

void dumpSEEPROMtoCSV() {
    // Data records start at address 16 (first 16 bytes are the persistent header)
    long totalRecords = (seepromWriteAddressLong - SEEPROM_DATA_START_LONG) / 16;

    if (totalRecords <= 0) {
        Serial.println(F("[DUMP] No records in SEEPROM."));
        return;
    }

    Serial.println(F("[DUMP] Begin SEEPROM CSV export"));
    Serial.println(F("---BEGIN CSV---"));

    // --- Header row ---
    Serial.println(F("RecordNum,"
                     "Hum_RH_pct,Temp_C,Temp_F,AHT20_Status,"
                     "Radio_Station_MHz,Radio_Signal,Radio_Stereo,"
                     "FlightStatus,"
                     "BT_RX_OK,BT_RX_FAIL,Balloon_Burst,Descent,Ascent,Landing"));

    // --- Data rows ---
    byte rec[16];

    for (long recNum = 0; recNum < totalRecords; recNum++) {
        long baseAddr = SEEPROM_DATA_START_LONG + (recNum * 16);
        // Read the 16-byte record from SEEPROM
        for (byte i = 0; i < 16; i++) {
            rec[i] = (byte)fdr.ReadSEEPROM(baseAddr + i);
        }
        // Decode AHT20 humidity (20-bit raw across bytes [1..3])
        unsigned long rawHum = ((unsigned long)rec[1] << 16)
                             | ((unsigned long)rec[2] <<  8)
                             |  (unsigned long)rec[3];
        float humRH = (rawHum * 100.0f) / 1048576.0f;
        // Decode AHT20 temperature (20-bit raw across bytes [4..6])
        unsigned long rawTemp = ((unsigned long)rec[4] << 16)
                              | ((unsigned long)rec[5] <<  8)
                              |  (unsigned long)rec[6];
        float tempC = (rawTemp * 200.0f / 1048576.0f) - 50.0f;
        float tempF = tempC * 9.0f / 5.0f + 32.0f;
        // Decode radio station frequency from index (bytes [8..9])
        uint16_t stIdx = ((uint16_t)rec[8] << 8) | rec[9];
        float stationMHz = 89.2f + (stIdx * 0.1f);
        // Decode flight status bits from byte [12]
        byte status = rec[12];
        // Print CSV row
        Serial.print(rec[0]);             Serial.print(F(","));  // RecordNum (seq byte)
        Serial.print(humRH, 2);           Serial.print(F(","));  // Hum_RH_pct
        Serial.print(tempC, 2);           Serial.print(F(","));  // Temp_C
        Serial.print(tempF, 2);           Serial.print(F(","));  // Temp_F
        Serial.print(rec[7]);             Serial.print(F(","));  // AHT20_Status
        Serial.print(stationMHz, 1);      Serial.print(F(","));  // Radio_Station_MHz
        Serial.print(rec[10]);            Serial.print(F(","));  // Radio_Signal
        Serial.print(rec[11]);            Serial.print(F(","));  // Radio_Stereo
        Serial.print(status);             Serial.print(F(","));  // FlightStatus (raw byte)
        Serial.print((status >> 0) & 1);  Serial.print(F(","));  // BT_RX_OK
        Serial.print((status >> 1) & 1);  Serial.print(F(","));  // BT_RX_FAIL
        Serial.print((status >> 2) & 1);  Serial.print(F(","));  // Balloon_Burst
        Serial.print((status >> 3) & 1);  Serial.print(F(","));  // Descent
        Serial.print((status >> 4) & 1);  Serial.print(F(","));  // Ascent
        Serial.println((status >> 5) & 1);                       // Landing
    }
    Serial.println(F("---END CSV---"));
    Serial.print(F("[DUMP] "));
    Serial.print(totalRecords);
    Serial.println(F(" records exported."));
}

//=====================================================================================================================================================================

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


//=====================================================================================================================================================================

//=====================================================================================================================================================================


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


//=====================================================================================================================================================================

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

// -------- JUSTIN STATE MACHINE -------- //
#ifdef ENABLE_TEA5767
/**
 * Function: RadioDelayForAndThenReturnTo
 * Purpose: A non-blocking timer that allows the FDR to keep running 
 * tasks (like GPS/Modem) while the Radio PLL is locking.
 */
void RadioDelayForAndThenReturnTo(unsigned int delayUInt, byte nextStateByte) {
    RadioStateByte = delayForByte; 
    RadioReturnToByte = nextStateByte;
    RadioDelayStartTimeULong = millis(); 
    RadioDelayDurationULong = delayUInt; 
}

/**
 * Function: RadioStateMachine
 * Purpose: Executes the non-blocking I2C communication and auto-scan logic
 * using precomputed PLL values.
 */
void RadioStateMachine() {
    switch(RadioStateByte) {
        
        case RadioIdleStateByte:
            if(RadioAutoScanEnabled) {
                RadioStateByte = RadioWriteDataByte;
            }
            break;

        case RadioWriteDataByte: {
            uint16_t pllValue = RadioStationPLL[stationIndex];

            byte pllHigh = (pllValue >> 8) & 0x3F;
            byte pllLow  = pllValue & 0xFF;

            Wire.beginTransmission(RadioAddressByte);
            Wire.write(pllHigh); 
            Wire.write(pllLow);  
            Wire.write(0xB0);  
            Wire.write(0x10);  
            Wire.write(0x00); 

            byte error = Wire.endTransmission();
            
            if(error == 0) {
                RadioDelayForAndThenReturnTo(30, RadioReadStatusByte);
            } else {
                #ifdef RADIO_DIAG_PRINT
                Serial.print(F("Write Error: "));
                Serial.println(error);
                #endif
                
                RadioDelayForAndThenReturnTo(15, RadioScanningStateByte); 
            }
            break;
        }

        case RadioReadStatusByte:
            if(Wire.requestFrom((int)RadioAddressByte, 5) >= 5) {
                byte b1 = Wire.read(); 
                byte b2 = Wire.read();
                byte b3 = Wire.read(); 
                byte b4 = Wire.read(); 
                byte b5 = Wire.read();

                if(b1 & 0x80) {
                    RadioSignalLevel = (b4 >> 4) & 0x0F; 
                    RadioStereoBool = (b3 & 0x80);        

                    // --- UPDATE THE GLOBAL VARIABLES ---
                    Last_scanned_station_idx = stationIndex;
                    Last_scanned_station_signal_strength = RadioSignalLevel;

                    #ifdef RADIO_DIAG_PRINT
                    float currentFreq = ScanStartFreq + (stationIndex * ScanIncrement);
                    Serial.print(F("STATION ID: ")); Serial.print(stationIndex);
                    Serial.print(F(" | FREQ: ")); Serial.print(currentFreq, 1);
                    Serial.print(F(" MHz | LVL: ")); Serial.println(RadioSignalLevel);
                    #endif
                    
                    RadioStateByte = RadioScanningStateByte;
                } else {
                    RadioDelayForAndThenReturnTo(20, RadioReadStatusByte);
                }
            } else {
                #ifdef RADIO_DIAG_PRINT
                Serial.println(F("Read Error: No Data"));
                #endif
                
                RadioDelayForAndThenReturnTo(250, RadioScanningStateByte);
            }
            break;

        case RadioScanningStateByte:
            stationIndex++;

            if(stationIndex >= totalStations) {
                stationIndex = 0;
                
                #ifdef RADIO_DIAG_PRINT
                Serial.println(F("--- SCAN RESTART ---"));
                #endif
            }
						RadioDelayForAndThenReturnTo(2000,RadioIdleStateByte);
            break;

        case delayForByte: 
            if(millis() - RadioDelayStartTimeULong > RadioDelayDurationULong) {
                RadioStateByte = RadioReturnToByte;
            }
            break;
    }
}
#endif

// ============================================================
//   buzzerUpdate()
//
//   Non-blocking buzzer pattern player.  Called every loop().
//
//   1. Detects flightStatusByte transitions -> selects new pattern
//   2. Plays the active pattern by toggling BUZZER_PIN
//   3. Handles one-shot TX success chirp (overrides briefly)
//
//   Pattern format: array of unsigned int durations in ms.
//     Even indices = tone ON,  Odd indices = tone OFF.
//     A 0 entry means "loop back to start".
//     PAT_IDLE = {0} means silence.
//
//   Priority (evaluated top-down, first match wins):
//     LANDED > BURST > DESCENT > ASCENT > BT_FAIL > IDLE
// ============================================================
void buzzerUpdate() {

    // ---- 1. DETECT STATE TRANSITIONS ----
    // We only switch patterns when the effective flight state changes,
    // so we compare the current status to what was active last time.
    // This keeps the pattern playing smoothly and restarts on transitions.

    byte currentStatus = flightStatusByte;
    bool transitioned = false;

    // Compute the "effective state byte" that matters for buzzer priority.
    // We mask to just the bits we care about for pattern selection.
    byte buzzerRelevantBits = currentStatus & (STATUS_LANDING | STATUS_BALLOON_BURST | STATUS_DESCENT | STATUS_ASCENT | STATUS_BT_RX_FAILURE);
    byte previousRelevantBits = buzzerPreviousFlightStatus & (STATUS_LANDING | STATUS_BALLOON_BURST | STATUS_DESCENT | STATUS_ASCENT | STATUS_BT_RX_FAILURE);

    if (buzzerRelevantBits != previousRelevantBits) {
        transitioned = true;
        buzzerPreviousFlightStatus = currentStatus;

        // Select new pattern based on priority
        const unsigned int* newPattern = PAT_IDLE;

        if (currentStatus & STATUS_LANDING) {
            newPattern = PAT_LANDED;
        } else if ((currentStatus & STATUS_BALLOON_BURST) && (currentStatus & STATUS_DESCENT)) {
            // Burst happened AND currently descending — descent pattern
            newPattern = PAT_DESCENT;
        } else if (currentStatus & STATUS_BALLOON_BURST) {
            // Burst latched but not yet descending (brief moment after pop)
            newPattern = PAT_BURST;
        } else if (currentStatus & STATUS_DESCENT) {
            newPattern = PAT_DESCENT;
        } else if (currentStatus & STATUS_ASCENT) {
            newPattern = PAT_ASCENT;
        } else if (currentStatus & STATUS_BT_RX_FAILURE) {
            newPattern = PAT_BT_FAIL;
        }

        // Only restart the pattern if it actually changed
        if (newPattern != buzzerPattern) {
            buzzerPattern        = newPattern;
            buzzerPatternIndex   = 0;
            buzzerToneOn         = false;
            buzzerStepStartULong = millis();
            digitalWrite(BUZZER_PIN, LOW);

            #ifdef DIAG_BUZZER
                Serial.print(F("[BUZZER] Transition -> "));
                if      (newPattern == PAT_LANDED)   Serial.println(F("LANDED (rapid beep)"));
                else if (newPattern == PAT_BURST)    Serial.println(F("BURST (urgent triple)"));
                else if (newPattern == PAT_DESCENT)  Serial.println(F("DESCENT (double beep)"));
                else if (newPattern == PAT_ASCENT)   Serial.println(F("ASCENT (heartbeat)"));
                else if (newPattern == PAT_BT_FAIL)  Serial.println(F("BT FAIL (long-short-long)"));
                else                                 Serial.println(F("IDLE (silent)"));
            #endif
        }
    }

    // ---- 2. TX SUCCESS CHIRP (one-shot override) ----
    // When a transmit succeeds we play a quick triple chirp,
    // then return to the current state pattern.
    if (buzzerTxChirpPending) {
        unsigned long now = millis();
        unsigned int dur = PAT_TX_OK[buzzerTxChirpStep];

        if (dur == 0) {
            // Chirp finished — return to state pattern
            buzzerTxChirpPending = false;
            buzzerTxChirpStep    = 0;
            digitalWrite(BUZZER_PIN, LOW);
            buzzerStepStartULong = millis();
            buzzerToneOn         = false;
            buzzerPatternIndex   = 0;
            #ifdef DIAG_BUZZER
                Serial.println(F("[BUZZER] TX chirp done, resuming state pattern"));
            #endif
            return;
        }

        // Kick off the first step of the chirp
        if (buzzerTxChirpStep == 0 && !buzzerToneOn) {
            buzzerToneOn = true;
            buzzerTxChirpStartULong = now;
            digitalWrite(BUZZER_PIN, HIGH);
            #ifdef DIAG_BUZZER
                Serial.println(F("[BUZZER] TX chirp start"));
            #endif
        }

        if ((now - buzzerTxChirpStartULong) >= dur) {
            buzzerTxChirpStep++;
            buzzerTxChirpStartULong = now;
            buzzerToneOn = (buzzerTxChirpStep % 2 == 0);  // even=ON, odd=OFF
            digitalWrite(BUZZER_PIN, buzzerToneOn ? HIGH : LOW);
        }
        return;  // while chirping, skip the main pattern player
    }

    // ---- 3. PLAY THE ACTIVE PATTERN ----
    // PAT_IDLE = {0} means silence — just keep the pin LOW.
    if (buzzerPattern[0] == 0) {
        if (buzzerToneOn) {
            buzzerToneOn = false;
            digitalWrite(BUZZER_PIN, LOW);
        }
        return;
    }

    unsigned long now = millis();
    unsigned int stepDuration = buzzerPattern[buzzerPatternIndex];

    // If we hit the 0 sentinel, loop back to start
    if (stepDuration == 0) {
        buzzerPatternIndex   = 0;
        stepDuration         = buzzerPattern[0];
        buzzerStepStartULong = now;
        buzzerToneOn         = true;
        digitalWrite(BUZZER_PIN, HIGH);
        return;
    }

    // Check if current step's time has elapsed
    if ((now - buzzerStepStartULong) >= stepDuration) {
        buzzerPatternIndex++;

        // Check for loop-back sentinel
        if (buzzerPattern[buzzerPatternIndex] == 0) {
            buzzerPatternIndex = 0;
        }

        buzzerStepStartULong = now;
        // Even index = ON, Odd index = OFF
        buzzerToneOn = (buzzerPatternIndex % 2 == 0);
        digitalWrite(BUZZER_PIN, buzzerToneOn ? HIGH : LOW);
    }

    // First call after pattern assignment — kick off the first tone
    if (buzzerPatternIndex == 0 && !buzzerToneOn && transitioned) {
        buzzerToneOn         = true;
        buzzerStepStartULong = now;
        digitalWrite(BUZZER_PIN, HIGH);
    }
}