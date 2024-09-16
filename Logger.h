#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <SD.h>
// #include <EEPROM.h>

// Base Logger Class
class Logger {
public:
    virtual void begin() = 0;
    virtual void logData(const String& data) = 0;
    virtual void flush() = 0;
    virtual ~Logger() {}
};

// SerialLogger Class
class SerialLogger : public Logger {
public:
     void begin() override {
        Serial.begin(9600);
    }

    void logData(const String& data) override {
        if (Serial) { // Check if Serial is available
            Serial.println(data);
        }
    }

    void flush() override {
        // Serial doesn't require flushing
    }
};

// SDLogger Class
class SDLogger : public Logger {
public:
    SDLogger(const char* filename, uint8_t csPin) : filename(filename), chipSelect(csPin), fileOpen(false) {}

    void begin() override {
        if (!SD.begin(chipSelect)) {
            Serial.println("SD card initialization failed!");
            // Handle error appropriately
            fileOpen = false;
            return;
        }
        file = SD.open(filename, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open file on SD card!");
            // Handle error appropriately
            fileOpen = false;
            return;
        }
        fileOpen = true;
    }

    void logData(const String& data) override {
        if (fileOpen) {
            file.println(data);
        }
    }

    void flush() override {
        if (fileOpen) {
            file.flush();
        }
    }

    ~SDLogger() {
        if (fileOpen) {
            file.close();
        }
    }

private:
    const char* filename;
    File file;
    uint8_t chipSelect;
    bool fileOpen;
};

// FlashMemoryLogger Class
// class FlashMemoryLogger : public Logger {
// public:
//     void begin() override {
//         currentAddress = 0;
//         EEPROM.begin(EEPROM_SIZE); // Initialize EEPROM with specified size
//     }

//     void logData(const String& data) override {
//         int len = data.length();
//         if (currentAddress + len + 1 < EEPROM_SIZE) {
//             for (int i = 0; i < len; i++) {
//                 EEPROM.write(currentAddress++, data[i]);
//             }
//             EEPROM.write(currentAddress++, '\n');
//         } else {
//             Serial.println("EEPROM memory full!");
//             // Handle memory full situation
//         }
//     }

//     void flush() override {
//         EEPROM.commit(); // Ensure data is written to flash
//     }

// private:
//     int currentAddress;
//     static const int EEPROM_SIZE = 1024; // Adjust based on your microcontroller's EEPROM size
// };

// LoggingMethod Enumeration
enum LoggingMethod {
    LOG_SERIAL,
    LOG_SD
    // LOG_FLASH
};

// LoggerSelector Class
class LoggerSelector {
public:
    LoggerSelector(LoggingMethod method) : method(method), logger(nullptr) {}

    void begin() {
        switch (method) {
            case LOG_SERIAL:
                logger = new SerialLogger();
                break;
            case LOG_SD:
                logger = new SDLogger("flight_data.csv", 10); // Adjust CS pin as needed
                break;
            //case LOG_FLASH:
            //    logger = new FlashMemoryLogger();
            //    break;
        }
        if (logger) {
            logger->begin();
        }
    }

    void logData(const String& data) {
        if (logger) {
            logger->logData(data);
        }
    }

    void flush() {
        if (logger) {
            logger->flush();
        }
    }

    ~LoggerSelector() {
        if (logger) {
            delete logger;
        }
    }

private:
    LoggingMethod method;
    Logger* logger;
};

#endif // LOGGER_H
