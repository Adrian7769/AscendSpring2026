#pragma once
#include <string>
#include <vector>
#include <cstdint>

// ============================================================
//   RECEIVED HEX PAYLOAD — 50 bytes total
// ============================================================
//   [0..1]   "RB" header         (modem manufacturer)
//   [2..4]   Serial number       (modem manufacturer, 3 bytes big-endian)
//   [5..49]  TX array            (our 45 bytes from buildIridiumTxArray)
//
//   TX ARRAY v4.2 layout at offset +5:
//   [5]      Flight status byte (6 packed bits)
//   [6..21]  GNSS from BT slave (lastValidBtRx[0..15])
//              [6] UTC Hours  [7] UTC Min  [8] UTC Sec
//              [9] Lat°  [10] Lat'  [11] Lat"  [12] Lat hemi (0=N 1=S)
//              [13] Lon° [14] Lon'  [15] Lon"  [16] Lon hemi (0=E 1=W)
//              [17..20] Altitude (4-byte big-endian)
//              [21] Altitude units (0=m 1=ft)
//   [22..37] I2C sensor block (latestI2Cblock[0..15])
//              [22] Record seq  [23..25] AHT20 Hum (20-bit)
//              [26..28] AHT20 Temp (20-bit)  [29] AHT20 status
//              [30..31] Radio station idx  [32] signal  [33] stereo
//              [34] flight status (SEEPROM copy)  [35..37] reserved
//   [38]     Reserved
//   [39..40] Master battery (ADC MSB/LSB)
//   [41..42] Internal temp  (ADC MSB/LSB)
//   [43..44] External temp  (ADC MSB/LSB)
//   [45..46] Slave battery  (MSB/LSB)
//   [47]     Current modem return code
//   [48]     Previous modem return code
//   [49]     Cumulative TX success count
// ============================================================

namespace PayloadIndex {
    // Manufacturer bytes
    constexpr int HEADER_BYTE_0     = 0;
    constexpr int HEADER_BYTE_1     = 1;
    constexpr int SERIAL_BYTE_0     = 2;
    constexpr int SERIAL_BYTE_1     = 3;
    constexpr int SERIAL_BYTE_2     = 4;

    // TX offset
    constexpr int TX_OFFSET         = 5;

    constexpr int FLIGHT_STATUS     = 5;

    constexpr int UTC_HOURS         = 6;
    constexpr int UTC_MINUTES       = 7;
    constexpr int UTC_SECONDS       = 8;
    constexpr int LAT_DEGREES       = 9;
    constexpr int LAT_MINUTES       = 10;
    constexpr int LAT_SECONDS       = 11;
    constexpr int LAT_HEMISPHERE    = 12;
    constexpr int LON_DEGREES       = 13;
    constexpr int LON_MINUTES       = 14;
    constexpr int LON_SECONDS       = 15;
    constexpr int LON_HEMISPHERE    = 16;
    constexpr int ALT_BYTE_0        = 17;
    constexpr int ALT_BYTE_1        = 18;
    constexpr int ALT_BYTE_2        = 19;
    constexpr int ALT_BYTE_3        = 20;
    constexpr int ALT_UNITS         = 21;

    constexpr int I2C_RECORD_SEQ    = 22;
    constexpr int AHT20_HUM_0      = 23;
    constexpr int AHT20_HUM_1      = 24;
    constexpr int AHT20_HUM_2      = 25;
    constexpr int AHT20_TEMP_0     = 26;
    constexpr int AHT20_TEMP_1     = 27;
    constexpr int AHT20_TEMP_2     = 28;
    constexpr int AHT20_STATUS     = 29;
    constexpr int RADIO_STATION_MSB = 30;
    constexpr int RADIO_STATION_LSB = 31;
    constexpr int RADIO_SIGNAL     = 32;
    constexpr int RADIO_STEREO     = 33;
    constexpr int SEEPROM_STATUS   = 34;

    constexpr int RESERVED_38      = 38;

    constexpr int MASTER_BATT_MSB  = 39;
    constexpr int MASTER_BATT_LSB  = 40;
    constexpr int INT_TEMP_MSB     = 41;
    constexpr int INT_TEMP_LSB     = 42;
    constexpr int EXT_TEMP_MSB     = 43;
    constexpr int EXT_TEMP_LSB     = 44;
    constexpr int SLAVE_BATT_MSB   = 45;
    constexpr int SLAVE_BATT_LSB   = 46;

    constexpr int MODEM_CURRENT    = 47;
    constexpr int MODEM_PREVIOUS   = 48;
    constexpr int TX_SUCCESS_COUNT = 49;

    constexpr int EXPECTED_SIZE    = 50;
}

namespace SensorCal {
    // ── Master battery ────────────────────────────────────────────────────
    // Source: analogRead(29) on the RP2040 built-in ADC.
    // ADC_TO_VOLTAGE converts raw counts to the pin voltage (3.3V / 1024).
    // MASTER_BATT_CAL corrects for the master board's resistor divider network.
    // Formula: BatteryV = raw * ADC_TO_VOLTAGE / MASTER_BATT_CAL
    //   (divider ratio < 1 means the pin sees a fraction of battery voltage,
    //    so we divide by it to get true battery voltage)
    constexpr float ADC_TO_VOLTAGE    = 0.00322f;  // V/count  — RP2040 analogRead(29) ONLY
    constexpr float MASTER_BATT_CAL   = 0.971f;    // resistor divider ratio for master board

    // ── External fdr ADC (temp sensors + slave battery) ───────────────────
    // Source: fdr.ADCread() on the external FDR ADC chip.
    // This is a completely different ADC from the RP2040 built-in.
    // Back-calculated from MOMSN 77: raw=4284, board serial = 22.2°C
    //   → required V = 22.2 × 0.011 + 0.502 = 0.7462 V
    //   → FDR_ADC_TO_VOLTAGE = 0.7462 / 4284 = 0.00017411 V/count
    // ⚠️  Verify against fdr.ADCconvertToVoltage() in your FDR library if accessible.
    constexpr float FDR_ADC_TO_VOLTAGE = 0.00017411f; // V/count — external fdr ADC ONLY

    // ── Slave battery ─────────────────────────────────────────────────────
    // Source: slave fdr.ADCread(7), sent over BT in lastValidBtRx[16..17].
    // The slave board has its own calibration equation (per team calibration doc):
    //   Battery Voltage = 1.049 × V_measured
    // Formula: BatteryV = raw * FDR_ADC_TO_VOLTAGE * SLAVE_BATT_CAL
    constexpr float SLAVE_BATT_CAL    = 1.049f;    // slave board calibration multiplier

    // ── Temperature calibration ───────────────────────────────────────────
    // tempC = (V_measured - OFFSET) / SCALE
    // Matches INT_TEMP_OFFSET_V, EXT_TEMP_OFFSET_V, TEMP_SCALE_VPDC in the .ino.
    constexpr float INT_TEMP_OFFSET_V = 0.502f;    // V at 0°C, internal sensor
    constexpr float EXT_TEMP_OFFSET_V = 0.491f;    // V at 0°C, external sensor
    constexpr float TEMP_SCALE_VPDC   = 0.011f;    // V per °C

    // ── AHT20 / Radio ─────────────────────────────────────────────────────
    constexpr float AHT20_DIVISOR     = 1048576.0f; // 2^20 — 20-bit raw to RH/Temp
    constexpr float RADIO_BASE_MHZ    = 89.2f;
    constexpr float RADIO_STEP_MHZ    = 0.1f;

    // BATTERY_DIVIDER kept for backward compatibility — prefer MASTER_BATT_CAL
    constexpr float BATTERY_DIVIDER   = MASTER_BATT_CAL;
}

// Flight status bits (TX[0] = byte [5])
struct FlightStatus {
    bool btRxSuccess   = false;
    bool btRxFailure   = false;
    bool balloonBurst  = false;
    bool descent       = false;
    bool ascent        = false;
    bool landing       = false;
    uint8_t raw        = 0;

    void decode(uint8_t statusByte) {
        raw           = statusByte;
        btRxSuccess   = (statusByte >> 0) & 1;
        btRxFailure   = (statusByte >> 1) & 1;
        balloonBurst  = (statusByte >> 2) & 1;
        descent       = (statusByte >> 3) & 1;
        ascent        = (statusByte >> 4) & 1;
        landing       = (statusByte >> 5) & 1;
    }

    std::string getPhaseString() const {
        if (landing)      return "LANDED";
        if (descent)      return "DESCENT";
        if (balloonBurst) return "BURST";
        if (ascent)       return "ASCENT";
        return "PRE-LAUNCH";
    }
};

struct DMS {
    uint8_t degrees = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    char hemisphere = 'N';
};

struct AHT20Data {
    float humidityRH = 0.0f;
    float tempC      = 0.0f;
    float tempF      = 0.0f;
    uint8_t status   = 0;
    bool isValid     = false;

    std::string statusString() const {
        switch (status) {
            case 2: return "OK";
            case 3: return "FAIL";
            case 4: return "TIMEOUT";
            case 5: return "NOT READY";
            default: return "UNKNOWN";
        }
    }
};

struct RadioData {
    uint16_t stationIndex  = 0;
    float frequencyMHz     = 0.0f;
    uint8_t signalStrength = 0;
    bool stereo            = false;
};

struct AnalogSensor {
    std::string name;
    float voltage     = 0.0f;
    float measurement = 0.0f;
    std::string unit;
    bool isValid      = false;
};

struct ModemInfo {
    uint8_t currentCode    = 0;
    uint8_t previousCode   = 0;
    uint8_t txSuccessCount = 0;
    std::string currentDesc;
    std::string previousDesc;
};

struct PayloadData {
    bool isValid = false;

    // Manufacturer header
    std::string header;
    bool headerValid      = false;
    uint32_t serialNumber = 0;

    // Flight status
    FlightStatus flightStatus;

    // GNSS
    bool gnssValid     = false;  // true only if all sanity checks pass
    uint8_t utcHours   = 0;
    uint8_t utcMinutes = 0;
    uint8_t utcSeconds = 0;
    DMS latitudeDMS;
    DMS longitudeDMS;
    double latitude  = 0.0;
    double longitude = 0.0;
    int32_t altitude = 0;
    std::string altitudeUnits = "meters";

    // I2C
    uint8_t recordSequence = 0;
    AHT20Data aht20;
    RadioData radio;

    // Analog
    AnalogSensor masterBattery;
    AnalogSensor slaveBattery;
    AnalogSensor internalTemp;
    AnalogSensor externalTemp;

    // Modem
    ModemInfo modem;

    std::string toString() const;
};

struct EmailContent {
    bool isValid = false;
    std::string imei;
    int momsn = 0;
    std::string transmitTime;
    double iridiumLatitude  = 0.0;
    double iridiumLongitude = 0.0;
    double iridiumCep       = 0.0;
    int sessionStatus       = 0;
    std::string hexData;
    PayloadData payload;

    std::string toString() const;
};

class Decoder {
public:
    EmailContent parseEmail(const std::string& bodyText);
    PayloadData decodeHexPayload(const std::string& hexString);

    static AnalogSensor decodeMasterBattery(uint16_t rawADC);
    static AnalogSensor decodeSlaveBattery(uint8_t msb, uint8_t lsb);
    static AnalogSensor decodeInternalTemp(uint16_t rawADC);
    static AnalogSensor decodeExternalTemp(uint16_t rawADC);
    static std::string getModemStatusDescription(uint8_t code);

private:
    std::string extractField(const std::string& text, const std::string& fieldName);
    std::vector<uint8_t> hexStringToBytes(const std::string& hex);
};