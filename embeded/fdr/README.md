# Rick Sparber Flight Data Controller

This collection of drivers runs on a Flight Data Recorder (https://rick.sparber.org/FlightDataRecorderSoftwareDescription.pdf). 

## About Flight Data Recorder ##

The Flight Data Recorder contains a Sparkfun Pro Micro, EEPROM, and linear regulators. It can accept six analog inputs plus monitors its input voltage. Peripheral can be connected to it including GPS, RunCam2, and an Iridium modem. This modem requires an additional board plus a second Pro Micro.


## What is the Fdr Library? ##

The Flight Data Recorder (Fdr) library contains all functions needed to run the Flight Data Recorder:


                 SEEPROM - Student EEPROM
 
    int WriteSEEPROM(long eeAddress, byte data);
    int ReadSEEPROM(long eeaddress);
	
Both functions return error codes for hardware problems and out of range addresses.

	

                 RunCam2 CAMERA

    void VideoRecordAtPowerUp();
    byte StartVideoRecording();
    byte GoToVideoStandby();
    byte GoToStillPictureStandby();
    byte TakeStillPicture();
	
This camera accepts an undocumented signal that moves it between states but does not return any acknowledgement. These functions depend on a software defined state machine that is assumed to be in sync with the camera. We only know the initial state at power up.
	

                 GNSS
				 
    void GlobalNavigationSatelliteSystem();

The sentence from a GPS or Global Navigation Satellite System is parced into bytes for easy access. For example, longitudinal degrees is in one byte while longitudinal minutes is in a different byte. Error codes can also be injected.


                 MODEM

    unsigned int Iridium(unsigned int ModemCommand);

The Iridium modem passes 45 bytes to a ground based server plus can receive 45 bytes from this server. The entire transaction, including all possible errors, is handled through this function. The user can also call this function at any time to receive a status report as the transaction is progressing. The code in Fdr talks to 15242 bytes of code in another Pro Micro which ties directly to the modem. Contact me for more information. It is too much to explain here.   

Written by Rick Sparber.
