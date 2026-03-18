#include "Decoder.h"
#include "logger.h"
#include <regex>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <cmath>

extern Logger logger;

// ============================================================
//   EMAIL PARSING (unchanged — same Rockblock email format)
// ============================================================

EmailContent Decoder::parseEmail(const std::string& bodyText) {
    EmailContent telemetry;
    try {
        telemetry.imei = extractField(bodyText, "IMEI");

        std::string momsnStr = extractField(bodyText, "MOMSN");
        if (!momsnStr.empty()) telemetry.momsn = std::stoi(momsnStr);

        telemetry.transmitTime = extractField(bodyText, "Transmit Time");

        std::string latStr = extractField(bodyText, "Iridium Latitude");
        if (!latStr.empty()) telemetry.iridiumLatitude = std::stod(latStr);

        std::string lonStr = extractField(bodyText, "Iridium Longitude");
        if (!lonStr.empty()) telemetry.iridiumLongitude = std::stod(lonStr);

        std::string cepStr = extractField(bodyText, "Iridium CEP");
        if (!cepStr.empty()) telemetry.iridiumCep = std::stod(cepStr);

        std::string statusStr = extractField(bodyText, "Iridium Session Status");
        if (!statusStr.empty()) telemetry.sessionStatus = std::stoi(statusStr);

        telemetry.hexData = extractField(bodyText, "Data");

        if (!telemetry.hexData.empty()) {
            telemetry.payload = decodeHexPayload(telemetry.hexData);
        }

        if (!telemetry.imei.empty() && !telemetry.hexData.empty()) {
            telemetry.isValid = true;
            logger.log(LOG_INFO, "Parsed telemetry - MOMSN: " + std::to_string(telemetry.momsn));
        } else {
            logger.log(LOG_WARNING, "Parsed telemetry missing essential fields");
        }
    } catch (const std::exception& e) {
        logger.log(LOG_ERROR, "Error parsing email: " + std::string(e.what()));
        telemetry.isValid = false;
    }
    return telemetry;
}

std::string Decoder::extractField(const std::string& text, const std::string& fieldName) {
    try {
        std::string pattern = fieldName + ":\\s*([^\\n\\r]+)";
        std::regex fieldRegex(pattern);
        std::smatch match;
        if (std::regex_search(text, match, fieldRegex)) {
            std::string value = match[1].str();
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t\r\nUTC") + 1);
            return value;
        }
    } catch (const std::exception& e) {
        logger.log(LOG_ERROR, "Error extracting field '" + fieldName + "': " + std::string(e.what()));
    }
    return "";
}

std::vector<uint8_t> Decoder::hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::string cleanHex;
    for (char c : hex) {
        if (std::isxdigit(c)) cleanHex += c;
    }
    for (size_t i = 0; i + 1 < cleanHex.length(); i += 2) {
        std::string byteString = cleanHex.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

// ============================================================
//   HEX PAYLOAD DECODE — 50-byte frame (5 mfr + 45 TX v4.2)
// ============================================================

PayloadData Decoder::decodeHexPayload(const std::string& hexString) {
    PayloadData payload;
    try {
        std::vector<uint8_t> b = hexStringToBytes(hexString);
        logger.log(LOG_INFO, "Decoding hex payload: " + hexString.substr(0, 50) +
            "... (" + std::to_string(b.size()) + " bytes)");

        if (static_cast<int>(b.size()) < PayloadIndex::EXPECTED_SIZE) {
            logger.log(LOG_WARNING, "Payload too short: expected " +
                std::to_string(PayloadIndex::EXPECTED_SIZE) + " bytes, got " +
                std::to_string(b.size()));
            return payload;
        }

        // ===== [0..1] MANUFACTURER HEADER =====
        char h0 = static_cast<char>(b[PayloadIndex::HEADER_BYTE_0]);
        char h1 = static_cast<char>(b[PayloadIndex::HEADER_BYTE_1]);
        payload.header = std::string(1, h0) + std::string(1, h1);
        payload.headerValid = (payload.header == "RB");
        logger.log(LOG_INFO, "  Header: " + payload.header +
            (payload.headerValid ? " Valid" : " Not Valid (expected 'RB')"));

        // ===== [2..4] SERIAL NUMBER =====
        payload.serialNumber = (b[PayloadIndex::SERIAL_BYTE_0] << 16) |
                               (b[PayloadIndex::SERIAL_BYTE_1] << 8)  |
                                b[PayloadIndex::SERIAL_BYTE_2];
        logger.log(LOG_INFO, "  RockBLOCK Serial: " + std::to_string(payload.serialNumber));

        // ===== [5] FLIGHT STATUS BYTE =====
        payload.flightStatus.decode(b[PayloadIndex::FLIGHT_STATUS]);
        logger.log(LOG_INFO, "  Flight Status: 0x" +
            ([](uint8_t v){ std::stringstream s; s << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0') << (int)v; return s.str(); })
            (payload.flightStatus.raw) + " -> " + payload.flightStatus.getPhaseString() +
            " | BT:" + (payload.flightStatus.btRxSuccess ? "OK" :
            (payload.flightStatus.btRxFailure ? "FAIL" : "NONE")));

        // ===== [6..21] GNSS DATA FROM BT SLAVE =====
        payload.utcHours   = b[PayloadIndex::UTC_HOURS];
        payload.utcMinutes = b[PayloadIndex::UTC_MINUTES];
        payload.utcSeconds = b[PayloadIndex::UTC_SECONDS];
        logger.log(LOG_INFO, "  UTC: " + std::to_string(payload.utcHours) + ":" +
            std::to_string(payload.utcMinutes) + ":" + std::to_string(payload.utcSeconds));

        // Latitude DMS
        payload.latitudeDMS.degrees   = b[PayloadIndex::LAT_DEGREES];
        payload.latitudeDMS.minutes   = b[PayloadIndex::LAT_MINUTES];
        payload.latitudeDMS.seconds   = b[PayloadIndex::LAT_SECONDS];
        payload.latitudeDMS.hemisphere = (b[PayloadIndex::LAT_HEMISPHERE] == 0) ? 'N' : 'S';
        payload.latitude = payload.latitudeDMS.degrees
                         + (payload.latitudeDMS.minutes / 60.0)
                         + (payload.latitudeDMS.seconds / 3600.0);
        if (payload.latitudeDMS.hemisphere == 'S') payload.latitude = -payload.latitude;

        // Longitude DMS
        payload.longitudeDMS.degrees   = b[PayloadIndex::LON_DEGREES];
        payload.longitudeDMS.minutes   = b[PayloadIndex::LON_MINUTES];
        payload.longitudeDMS.seconds   = b[PayloadIndex::LON_SECONDS];
        payload.longitudeDMS.hemisphere = (b[PayloadIndex::LON_HEMISPHERE] == 0) ? 'E' : 'W';
        payload.longitude = payload.longitudeDMS.degrees
                          + (payload.longitudeDMS.minutes / 60.0)
                          + (payload.longitudeDMS.seconds / 3600.0);
        if (payload.longitudeDMS.hemisphere == 'W') payload.longitude = -payload.longitude;

        logger.log(LOG_INFO, "  Lat: " + std::to_string(payload.latitude) +
            "  Lon: " + std::to_string(payload.longitude));

        // Altitude (4 bytes big-endian)
        payload.altitude = (static_cast<int32_t>(b[PayloadIndex::ALT_BYTE_0]) << 24) |
                           (static_cast<int32_t>(b[PayloadIndex::ALT_BYTE_1]) << 16) |
                           (static_cast<int32_t>(b[PayloadIndex::ALT_BYTE_2]) <<  8) |
                            static_cast<int32_t>(b[PayloadIndex::ALT_BYTE_3]);
        payload.altitudeUnits = (b[PayloadIndex::ALT_UNITS] == 0) ? "meters" : "feet";
        logger.log(LOG_INFO, "  Altitude: " + std::to_string(payload.altitude) + " " + payload.altitudeUnits);

        // ===== GNSS SANITY CHECKS =====
        // The BT slave can send junk data. We still log everything
        // but only set gnssValid=true if ALL checks pass.
        {
            bool sane = true;
            // UTC range checks
            if (payload.utcHours > 23) {
                logger.log(LOG_WARNING, "  GNSS SANITY: UTC hours=" + std::to_string(payload.utcHours) + " > 23");
                sane = false;
            }
            if (payload.utcMinutes > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: UTC minutes=" + std::to_string(payload.utcMinutes) + " > 59");
                sane = false;
            }
            if (payload.utcSeconds > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: UTC seconds=" + std::to_string(payload.utcSeconds) + " > 59");
                sane = false;
            }
            // Latitude: degrees 0-90, minutes 0-59, seconds 0-59
            if (payload.latitudeDMS.degrees > 90) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lat degrees=" + std::to_string(payload.latitudeDMS.degrees) + " > 90");
                sane = false;
            }
            if (payload.latitudeDMS.minutes > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lat minutes=" + std::to_string(payload.latitudeDMS.minutes) + " > 59");
                sane = false;
            }
            if (payload.latitudeDMS.seconds > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lat seconds=" + std::to_string(payload.latitudeDMS.seconds) + " > 59");
                sane = false;
            }
            // Longitude: degrees 0-180, minutes 0-59, seconds 0-59
            if (payload.longitudeDMS.degrees > 180) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lon degrees=" + std::to_string(payload.longitudeDMS.degrees) + " > 180");
                sane = false;
            }
            if (payload.longitudeDMS.minutes > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lon minutes=" + std::to_string(payload.longitudeDMS.minutes) + " > 59");
                sane = false;
            }
            if (payload.longitudeDMS.seconds > 59) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lon seconds=" + std::to_string(payload.longitudeDMS.seconds) + " > 59");
                sane = false;
            }
            // Hemisphere bytes must be 0 or 1
            if (b[PayloadIndex::LAT_HEMISPHERE] > 1) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lat hemisphere byte=" + std::to_string(b[PayloadIndex::LAT_HEMISPHERE]) + " (expected 0 or 1)");
                sane = false;
            }
            if (b[PayloadIndex::LON_HEMISPHERE] > 1) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Lon hemisphere byte=" + std::to_string(b[PayloadIndex::LON_HEMISPHERE]) + " (expected 0 or 1)");
                sane = false;
            }
            // Altitude units byte must be 0 or 1
            if (b[PayloadIndex::ALT_UNITS] > 1) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Alt units byte=" + std::to_string(b[PayloadIndex::ALT_UNITS]) + " (expected 0 or 1)");
                sane = false;
            }
            // Altitude sanity: reject obviously absurd values (> 50,000 m or < -1000 m)
            if (payload.altitude > 50000 || payload.altitude < -1000) {
                logger.log(LOG_WARNING, "  GNSS SANITY: Altitude=" + std::to_string(payload.altitude) + " out of range");
                sane = false;
            }
            // All-zero GNSS block means no lock
            if (payload.utcHours == 0 && payload.utcMinutes == 0 && payload.utcSeconds == 0 &&
                payload.latitudeDMS.degrees == 0 && payload.latitudeDMS.minutes == 0 &&
                payload.longitudeDMS.degrees == 0 && payload.longitudeDMS.minutes == 0) {
                logger.log(LOG_WARNING, "  GNSS SANITY: All-zero GNSS — no satellite lock");
                sane = false;
            }

            payload.gnssValid = sane;
            logger.log(LOG_INFO, "  GNSS Valid: " + std::string(sane ? "YES" : "NO — junk data, will not plot"));
        }

        // ===== [22..37] I2C SENSOR BLOCK =====
        payload.recordSequence = b[PayloadIndex::I2C_RECORD_SEQ];

        // AHT20 Humidity: 20-bit raw -> RH%
        uint32_t rawHum = (static_cast<uint32_t>(b[PayloadIndex::AHT20_HUM_0]) << 16) |
                          (static_cast<uint32_t>(b[PayloadIndex::AHT20_HUM_1]) <<  8) |
                           static_cast<uint32_t>(b[PayloadIndex::AHT20_HUM_2]);
        payload.aht20.humidityRH = (rawHum * 100.0f) / SensorCal::AHT20_DIVISOR;

        // AHT20 Temperature: 20-bit raw -> C
        uint32_t rawTemp = (static_cast<uint32_t>(b[PayloadIndex::AHT20_TEMP_0]) << 16) |
                           (static_cast<uint32_t>(b[PayloadIndex::AHT20_TEMP_1]) <<  8) |
                            static_cast<uint32_t>(b[PayloadIndex::AHT20_TEMP_2]);
        payload.aht20.tempC = (rawTemp * 200.0f / SensorCal::AHT20_DIVISOR) - 50.0f;
        payload.aht20.tempF = payload.aht20.tempC * 9.0f / 5.0f + 32.0f;
        payload.aht20.status = b[PayloadIndex::AHT20_STATUS];
        payload.aht20.isValid = (payload.aht20.status == 2);

        logger.log(LOG_INFO, "  AHT20: " + std::to_string(payload.aht20.humidityRH) + "% RH, " +
            std::to_string(payload.aht20.tempC) + " C (" + std::to_string(payload.aht20.tempF) +
            " F) [" + payload.aht20.statusString() + "]");

        // Radio scanner
        payload.radio.stationIndex = (static_cast<uint16_t>(b[PayloadIndex::RADIO_STATION_MSB]) << 8) |
                                      static_cast<uint16_t>(b[PayloadIndex::RADIO_STATION_LSB]);
        payload.radio.frequencyMHz = SensorCal::RADIO_BASE_MHZ +
                                     (payload.radio.stationIndex * SensorCal::RADIO_STEP_MHZ);
        payload.radio.signalStrength = b[PayloadIndex::RADIO_SIGNAL];
        payload.radio.stereo = (b[PayloadIndex::RADIO_STEREO] != 0);

        logger.log(LOG_INFO, "  Radio: " + std::to_string(payload.radio.frequencyMHz) +
            " MHz, signal=" + std::to_string(payload.radio.signalStrength) + "/15" +
            (payload.radio.stereo ? " STEREO" : " MONO"));

        // ===== [39..46] ANALOG SENSORS =====
        uint16_t rawMBatt = (static_cast<uint16_t>(b[PayloadIndex::MASTER_BATT_MSB]) << 8) |
                             static_cast<uint16_t>(b[PayloadIndex::MASTER_BATT_LSB]);
        payload.masterBattery = decodeMasterBattery(rawMBatt);

        uint16_t rawIntT = (static_cast<uint16_t>(b[PayloadIndex::INT_TEMP_MSB]) << 8) |
                            static_cast<uint16_t>(b[PayloadIndex::INT_TEMP_LSB]);
        payload.internalTemp = decodeInternalTemp(rawIntT);

        uint16_t rawExtT = (static_cast<uint16_t>(b[PayloadIndex::EXT_TEMP_MSB]) << 8) |
                            static_cast<uint16_t>(b[PayloadIndex::EXT_TEMP_LSB]);
        payload.externalTemp = decodeExternalTemp(rawExtT);

        payload.slaveBattery = decodeSlaveBattery(b[PayloadIndex::SLAVE_BATT_MSB],
                                                   b[PayloadIndex::SLAVE_BATT_LSB]);

        logger.log(LOG_INFO, "  Master Batt: " + std::to_string(payload.masterBattery.measurement) + " V");
        logger.log(LOG_INFO, "  Slave Batt:  " + std::to_string(payload.slaveBattery.measurement) + " V");
        logger.log(LOG_INFO, "  Int Temp:    " + std::to_string(payload.internalTemp.measurement) + " C");
        logger.log(LOG_INFO, "  Ext Temp:    " + std::to_string(payload.externalTemp.measurement) + " C");

        // ===== [47..49] MODEM STATUS =====
        payload.modem.currentCode    = b[PayloadIndex::MODEM_CURRENT];
        payload.modem.previousCode   = b[PayloadIndex::MODEM_PREVIOUS];
        payload.modem.txSuccessCount = b[PayloadIndex::TX_SUCCESS_COUNT];
        payload.modem.currentDesc    = getModemStatusDescription(payload.modem.currentCode);
        payload.modem.previousDesc   = getModemStatusDescription(payload.modem.previousCode);

        logger.log(LOG_INFO, "  Modem: current=" + std::to_string(payload.modem.currentCode) +
            " (" + payload.modem.currentDesc + "), prev=" + std::to_string(payload.modem.previousCode) +
            ", txOK=" + std::to_string(payload.modem.txSuccessCount));

        payload.isValid = payload.headerValid;

    } catch (const std::exception& e) {
        logger.log(LOG_ERROR, "Error decoding hex payload: " + std::string(e.what()));
        payload.isValid = false;
    }
    return payload;
}

// ============================================================
//   ANALOG SENSOR DECODERS
// ============================================================

AnalogSensor Decoder::decodeMasterBattery(uint16_t rawADC) {
    AnalogSensor s;
    s.name = "Master Battery";
    // Source: analogRead(29) on the RP2040 built-in ADC.
    // ADC_TO_VOLTAGE = 0.00322 V/count; MASTER_BATT_CAL = 0.971 (divider ratio).
    s.voltage = rawADC * SensorCal::ADC_TO_VOLTAGE;
    s.measurement = s.voltage / SensorCal::MASTER_BATT_CAL;
    s.unit = "V";
    s.isValid = true;
    return s;
}

AnalogSensor Decoder::decodeSlaveBattery(uint8_t msb, uint8_t lsb) {
    AnalogSensor s;
    s.name = "Slave Battery";
    uint16_t rawADC = (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
    // Source: slave fdr.ADCread(7), forwarded over BT in lastValidBtRx[16..17].
    // The slave uses the external fdr ADC chip (same chip as master temp sensors),
    // so FDR_ADC_TO_VOLTAGE applies here — NOT the RP2040 ADC_TO_VOLTAGE.
    // The slave board calibration equation is: BatteryV = 1.049 × V_measured.
    s.voltage = rawADC * SensorCal::FDR_ADC_TO_VOLTAGE;
    s.measurement = s.voltage * SensorCal::SLAVE_BATT_CAL;
    s.unit = "V";
    s.isValid = true;
    return s;
}

AnalogSensor Decoder::decodeInternalTemp(uint16_t rawADC) {
    AnalogSensor s;
    s.name = "Internal Temperature";
    // Source: fdr.ADCread(ADC_PORT_INT_TEMP) — external fdr ADC chip.
    // Must use FDR_ADC_TO_VOLTAGE, NOT ADC_TO_VOLTAGE.
    // Using ADC_TO_VOLTAGE (0.00322) inflated voltage ~19× → ~1200°C instead of ~22°C.
    s.voltage = rawADC * SensorCal::FDR_ADC_TO_VOLTAGE;
    float tempC = (s.voltage - SensorCal::INT_TEMP_OFFSET_V) / SensorCal::TEMP_SCALE_VPDC;
    float tempF = tempC * 1.8f + 32.0f;
    s.measurement = tempC;
    s.unit = "°C (" + std::to_string(static_cast<int>(tempF)) + "°F)";
    s.isValid = true;
    return s;
}

AnalogSensor Decoder::decodeExternalTemp(uint16_t rawADC) {
    AnalogSensor s;
    s.name = "External Temperature";
    // Source: fdr.ADCread(ADC_PORT_EXT_TEMP) — external fdr ADC chip.
    // Same fix as decodeInternalTemp: must use FDR_ADC_TO_VOLTAGE, not ADC_TO_VOLTAGE.
    s.voltage = rawADC * SensorCal::FDR_ADC_TO_VOLTAGE;
    float tempC = (s.voltage - SensorCal::EXT_TEMP_OFFSET_V) / SensorCal::TEMP_SCALE_VPDC;
    float tempF = tempC * 1.8f + 32.0f;
    s.measurement = tempC;
    s.unit = "°C (" + std::to_string(static_cast<int>(tempF)) + "°F)";
    s.isValid = true;
    return s;
}

// ============================================================
//   MODEM STATUS DESCRIPTIONS
// ============================================================

std::string Decoder::getModemStatusDescription(uint8_t code) {
    // Stored as single byte (low byte of the full return code)
    switch (code) {
        case 144: return "Ping Through MPM And Modem Success";   // 400 & 0xFF
        case 145: return "Modem Ready For Use";                  // 401
        case 146: return "Transmit Successful, No Receive";      // 402
        case 147: return "Transmit And Receive Successful";      // 403
        case 148: return "TX+RX Successful, Receive Pending";    // 404
        case 151: return "Receive Data In Array";                // 407

        case 44:  return "Success After SBDIX";                  // 300
        case 48:  return "OK Found";                             // 304
        case 55:  return "Idle";                                 // 311
        case 57:  return "Network Available, Good Signal";       // 313

        case 200: return "Transmit Failed";                      // 200
        case 34:  return "Write To MO Buffer Failed";            // 290
        case 35:  return "Wrong Modem, Check Serial Number";     // 291

        case 0:   return "No Status";
        default:  return "Code " + std::to_string(code);
    }
}

// ============================================================
//   toString
// ============================================================

std::string PayloadData::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << " DECODED PAYLOAD (v4.2 — 50 byte frame: 5 mfr + 45 TX)\n";
    ss << "Header:          " << header << (headerValid ? " Valid" : " Not Valid") << "\n";
    ss << "Serial Number:   " << serialNumber << "\n";
    ss << "Flight Phase:    " << flightStatus.getPhaseString()
       << "  [0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
       << (int)flightStatus.raw << std::dec << "]\n";
    ss << "  BT RX: " << (flightStatus.btRxSuccess ? "OK" : (flightStatus.btRxFailure ? "FAIL" : "NONE"))
       << "  Burst: " << (flightStatus.balloonBurst ? "YES" : "no")
       << "  Landing: " << (flightStatus.landing ? "YES" : "no") << "\n";

    ss << "\nGNSS: " << (gnssValid ? "(VALID)" : "(INVALID — junk data)") << "\n";
    ss << "  UTC:       " << (int)utcHours << ":" << (int)utcMinutes << ":" << (int)utcSeconds << "\n";
    ss << "  Latitude:  " << latitude << " ("
       << (int)latitudeDMS.degrees << "d " << (int)latitudeDMS.minutes << "' "
       << (int)latitudeDMS.seconds << "\" " << latitudeDMS.hemisphere << ")\n";
    ss << "  Longitude: " << longitude << " ("
       << (int)longitudeDMS.degrees << "d " << (int)longitudeDMS.minutes << "' "
       << (int)longitudeDMS.seconds << "\" " << longitudeDMS.hemisphere << ")\n";
    ss << "  Altitude:  " << altitude << " " << altitudeUnits << "\n";

    ss << "\nI2C Sensors:\n";
    ss << "  AHT20:     " << aht20.humidityRH << "% RH, "
       << aht20.tempC << " C (" << aht20.tempF << " F) [" << aht20.statusString() << "]\n";
    ss << "  Radio:     " << radio.frequencyMHz << " MHz, signal "
       << (int)radio.signalStrength << "/15 " << (radio.stereo ? "STEREO" : "MONO") << "\n";

    ss << "\nAnalog Sensors:\n";
    ss << "  " << masterBattery.name << ": " << masterBattery.measurement << " " << masterBattery.unit << "\n";
    ss << "  " << slaveBattery.name  << ": " << slaveBattery.measurement  << " " << slaveBattery.unit << "\n";
    ss << "  " << internalTemp.name  << ": " << internalTemp.measurement  << " " << internalTemp.unit << "\n";
    ss << "  " << externalTemp.name  << ": " << externalTemp.measurement  << " " << externalTemp.unit << "\n";

    ss << "\nModem:\n";
    ss << "  Current:   " << (int)modem.currentCode << " — " << modem.currentDesc << "\n";
    ss << "  Previous:  " << (int)modem.previousCode << " — " << modem.previousDesc << "\n";
    ss << "  TX Count:  " << (int)modem.txSuccessCount << "\n";

    return ss.str();
}

std::string EmailContent::toString() const {
    std::stringstream ss;
    ss << " BALLOON TELEMETRY (MOMSN: " << momsn << ")\n";
    ss << "IMEI:            " << imei << "\n";
    ss << "Transmit Time:   " << transmitTime << "\n";
    ss << "Iridium Lat:     " << iridiumLatitude << "\n";
    ss << "Iridium Lon:     " << iridiumLongitude << "\n";
    ss << "Iridium CEP:     " << iridiumCep << "\n";
    ss << "Session Status:  " << sessionStatus << "\n";
    ss << "\n" << payload.toString();
    return ss.str();
}