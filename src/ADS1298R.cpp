#include "ADS1298R.h"

/**
 * @file ADS1298R.cpp
 * @brief Y3X ADS1298R Library Implementation
 * @author Ashfaque Khan
 */

ADS1298R* ADS1298R::instance = nullptr;

ADS1298R::ADS1298R(uint8_t cs, uint8_t drdy, uint8_t start, uint8_t reset, uint32_t spiFreq) {
    pinCS = cs;
    pinDRDY = drdy;
    pinSTART = start;
    pinRESET = reset;
    spiFrequency = spiFreq;
    
    initialized = false;
    acquiring = false;
    dataReady = false;
    dataOverrun = false;
    lastError = ERR_NONE;
    
    instance = this;
}

bool ADS1298R::begin() {
    pinMode(pinDRDY, INPUT);
    pinMode(pinCS, OUTPUT);
    pinMode(pinSTART, OUTPUT);
    pinMode(pinRESET, OUTPUT);
    
    digitalWrite(pinCS, HIGH);
    digitalWrite(pinSTART, LOW);
    digitalWrite(pinRESET, HIGH);
    
    if (!PicoSPI0.configure(2, 3, 4, 5, spiFrequency, 1, false)) {
        lastError = ERR_SPI_INIT_FAILED;
        return false;
    }
    
    digitalWrite(pinRESET, LOW);
    delay(10);
    digitalWrite(pinRESET, HIGH);
    delay(100);
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_SDATAC);
    PicoSPI0.endTransaction();
    delay(10);
    
    uint8_t deviceId = getDeviceID();
    if ((deviceId & 0xF0) != 0xD0) {
        lastError = ERR_INVALID_DEVICE_ID;
        return false;
    }
    
    attachInterrupt(digitalPinToInterrupt(pinDRDY), handleInterrupt, FALLING);
    
    initialized = true;
    lastError = ERR_NONE;
    return true;
}

bool ADS1298R::startAcquisition() {
    if (!initialized) {
        lastError = ERR_NOT_INITIALIZED;
        return false;
    }
    
    if (acquiring) {
        lastError = ERR_ALREADY_ACQUIRING;
        return false;
    }
    
    dataReady = false;
    dataOverrun = false;
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_RDATAC);
    PicoSPI0.endTransaction();
    delay(10);
    
    digitalWrite(pinSTART, HIGH);
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_START);
    PicoSPI0.endTransaction();
    
    acquiring = true;
    return true;
}

bool ADS1298R::stopAcquisition() {
    if (!acquiring) {
        lastError = ERR_NOT_ACQUIRING;
        return false;
    }
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_STOP);
    PicoSPI0.endTransaction();
    digitalWrite(pinSTART, LOW);
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_SDATAC);
    PicoSPI0.endTransaction();
    delay(10);
    
    acquiring = false;
    return true;
}

bool ADS1298R::isDataReady() {
    return dataReady;
}

bool ADS1298R::readData(Data& data) {
    if (!acquiring) {
        lastError = ERR_NOT_ACQUIRING;
        return false;
    }
    
    if (!dataReady) {
        return false;
    }
    
    if (dataOverrun) {
        lastError = ERR_DATA_OVERRUN;
        dataOverrun = false;
    }
    
    PicoSPI0.beginTransaction();
    
    data.status = 0;
    for (int i = 0; i < 3; i++) {
        data.status = (data.status << 8) | PicoSPI0.transfer(0x00);
    }
    
    for (int i = 0; i < 8; i++) {
        data.channels[i] = 0;
        for (int j = 0; j < 3; j++) {
            uint8_t byte = PicoSPI0.transfer(0x00);
            data.channels[i] = (data.channels[i] << 8) | byte;
        }
        
        if (data.channels[i] & 0x800000) {
            data.channels[i] |= 0xFF000000;
        }
    }
    
    PicoSPI0.endTransaction();
    
    updateLeadOffStatus(data);
    
    data.timestamp = micros();
    data.valid = true;
    dataReady = false;
    
    return true;
}

bool ADS1298R::readDataWithQualityCheck(Data& data) {
    if (!readData(data)) {
        return false;
    }
    
    uint8_t loffStatP = readRegister(REG_LOFF_STATP);
    uint8_t loffStatN = readRegister(REG_LOFF_STATN);
    
    for (int i = 0; i < 8; i++) {
        data.leadOffStatus[i] = (loffStatP & (1 << i)) || (loffStatN & (1 << i));
    }
    
    return true;
}

void ADS1298R::writeRegister(uint8_t address, uint8_t value) {
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_WREG | address);
    PicoSPI0.transfer(0x00);
    PicoSPI0.transfer(value);
    PicoSPI0.endTransaction();
    delayMicroseconds(10);
}

uint8_t ADS1298R::readRegister(uint8_t address) {
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_RREG | address);
    PicoSPI0.transfer(0x00);
    uint8_t value = PicoSPI0.transfer(0x00);
    PicoSPI0.endTransaction();
    return value;
}

void ADS1298R::setPowerMode(PowerMode mode, DataRate rate) {
    uint8_t config1 = (uint8_t)rate;
    if (mode == HIGH_RESOLUTION) {
        config1 |= 0x80;
    }
    writeRegister(REG_CONFIG1, config1);
}

void ADS1298R::setChannel(uint8_t channel, const ChannelConfig& config) {
    if (channel >= 8) {
        lastError = ERR_INVALID_CHANNEL;
        return;
    }
    
    uint8_t chReg = 0;
    if (!config.enabled) {
        chReg |= 0x80;
    }
    chReg |= (uint8_t)config.gain;
    chReg |= (uint8_t)config.input;
    
    writeRegister(REG_CH1SET + channel, chReg);
}

void ADS1298R::setReference(ReferenceVoltage ref) {
    uint8_t config3 = 0x80;
    config3 |= 0x40;
    
    if (ref == INTERNAL_4V) {
        config3 |= 0x20;
    }
    
    config3 |= 0x08;
    config3 |= 0x04;
    
    writeRegister(REG_CONFIG3, config3);
    
    if (ref != EXTERNAL_REF) {
        delay(150);
    }
}

void ADS1298R::setTestSignal(TestSignal mode) {
    uint8_t config2 = readRegister(REG_CONFIG2);
    config2 &= 0xF0;
    
    if (mode != TEST_DISABLED) {
        config2 |= 0x10;
        config2 |= (uint8_t)mode;
    }
    
    writeRegister(REG_CONFIG2, config2);
}

void ADS1298R::setRLD(uint8_t positiveMask, uint8_t negativeMask) {
    writeRegister(REG_RLD_SENSP, positiveMask);
    writeRegister(REG_RLD_SENSN, negativeMask);
}

void ADS1298R::setLeadOff(LeadOffCurrent current, LeadOffFrequency freq, uint8_t threshold) {
    uint8_t loff = threshold;
    loff |= (uint8_t)current;
    loff |= (uint8_t)freq;
    writeRegister(REG_LOFF, loff);
    
    uint8_t config4 = readRegister(REG_CONFIG4);
    config4 |= 0x02;
    writeRegister(REG_CONFIG4, config4);
}

void ADS1298R::setNotchFilter(bool enable50Hz, bool enable60Hz) {
    uint8_t config2 = readRegister(REG_CONFIG2);
    config2 &= 0xFC;
    
    if (enable50Hz && enable60Hz) {
        config2 |= 0x03;
    } else if (enable60Hz) {
        config2 |= 0x01;
    } else if (enable50Hz) {
        config2 |= 0x02;
    }
    
    writeRegister(REG_CONFIG2, config2);
}

void ADS1298R::setGPIO(uint8_t direction, uint8_t data) {
    uint8_t gpio = ((~direction & 0x0F) << 4) | (data & 0x0F);
    writeRegister(REG_GPIO, gpio);
}

uint8_t ADS1298R::readGPIO() {
    return readRegister(REG_GPIO) & 0x0F;
}

bool ADS1298R::configureWCTOptimized(bool enableChop) {
    if (!initialized) {
        lastError = ERR_NOT_INITIALIZED;
        return false;
    }
    
    uint8_t config2 = readRegister(REG_CONFIG2);
    if (enableChop) {
        config2 |= 0x80;
    } else {
        config2 &= ~0x80;
    }
    config2 &= ~0x10;
    writeRegister(REG_CONFIG2, config2);
    delay(10);
    
    uint8_t wct1_target = 0x0B;
    writeRegister(REG_WCT1, wct1_target);
    delay(5);
    
    uint8_t wct1_verify = readRegister(REG_WCT1);
    if (wct1_verify != wct1_target) {
        lastError = ERR_REGISTER_VERIFY_FAILED;
        return false;
    }
    
    uint8_t wct2_target = 0xD4;
    writeRegister(REG_WCT2, wct2_target);
    delay(5);
    
    uint8_t wct2_verify = readRegister(REG_WCT2);
    if (wct2_verify != wct2_target) {
        lastError = ERR_REGISTER_VERIFY_FAILED;
        return false;
    }
    
    uint8_t config4 = readRegister(REG_CONFIG4);
    config4 |= 0x04;
    config4 |= 0x02;
    writeRegister(REG_CONFIG4, config4);
    
    delay(100);
    
    uint8_t config4_verify = readRegister(REG_CONFIG4);
    if ((config4_verify & 0x06) != 0x06) {
        lastError = ERR_REGISTER_VERIFY_FAILED;
        return false;
    }
    
    uint8_t loffSensP = 0xF9;
    uint8_t loffSensN = 0xF9;
    writeRegister(REG_LOFF_SENSP, loffSensP);
    writeRegister(REG_LOFF_SENSN, loffSensN);
    
    lastError = ERR_NONE;
    return true;
}

void ADS1298R::diagnoseV6Channel() {
    Serial.println("\n=== V6 Channel Diagnostics ===");
    
    uint8_t wct1 = readRegister(REG_WCT1);
    uint8_t wct2 = readRegister(REG_WCT2);
    uint8_t config4 = readRegister(REG_CONFIG4);
    uint8_t ch1set = readRegister(REG_CH1SET);
    
    Serial.print("WCT1: 0x"); Serial.print(wct1, HEX);
    Serial.print(" - WCTA: "); Serial.println((wct1 & 0x08) ? "ON" : "OFF");
    
    Serial.print("WCT2: 0x"); Serial.print(wct2, HEX);
    Serial.print(" - WCTB: "); Serial.print((wct2 & 0x40) ? "ON" : "OFF");
    Serial.print(", WCTC: "); Serial.println((wct2 & 0x80) ? "ON" : "OFF");
    
    Serial.print("CONFIG4: 0x"); Serial.print(config4, HEX);
    Serial.print(" - WCT_TO_RLD: "); Serial.println((config4 & 0x04) ? "ON" : "OFF");
    
    Serial.print("CH1SET: 0x"); Serial.print(ch1set, HEX);
    Serial.print(" - V6 Channel: "); Serial.println((ch1set & 0x80) ? "DISABLED" : "ENABLED");
    
    uint8_t loffStatP = readRegister(REG_LOFF_STATP);
    uint8_t loffStatN = readRegister(REG_LOFF_STATN);
    Serial.print("V6 Lead-off: P="); Serial.print((loffStatP & 0x01) ? "OFF" : "OK");
    Serial.print(", N="); Serial.println((loffStatN & 0x01) ? "OFF" : "OK");
    
    Serial.println("Testing V6 with 1mV test signal...");
    setTestSignal(TEST_1MV_FAST);
    delay(100);
    
    if (isDataReady()) {
        Data testData;
        if (readData(testData)) {
            Serial.print("V6 test amplitude: "); Serial.println(testData.channels[0]);
            Serial.println("Expected: ~8388 counts for 1mV@gain=6");
        }
    }
    
    setTestSignal(TEST_DISABLED);
    Serial.println("=== End V6 Diagnostics ===\n");
}

void ADS1298R::reset() {
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(CMD_RESET);
    PicoSPI0.endTransaction();
    delay(100);
}

uint8_t ADS1298R::getDeviceID() {
    return readRegister(REG_ID);
}

float ADS1298R::toVoltage(int32_t raw, Gain gain, float vref) {
    int gainValue = getGainValue(gain);
    float fullScale = vref / gainValue;
    return (raw * fullScale) / 8388608.0f;
}

float ADS1298R::toMillivolts(int32_t raw, Gain gain, float vref) {
    return toVoltage(raw, gain, vref) * 1000.0f;
}

float ADS1298R::toMicrovolts(int32_t raw, Gain gain, float vref) {
    return toVoltage(raw, gain, vref) * 1000000.0f;
}

uint16_t ADS1298R::getSampleRate(PowerMode power, DataRate rate) {
    uint16_t baseRate = (power == HIGH_RESOLUTION) ? 512000 : 256000;
    uint16_t dividers[] = {16, 32, 64, 128, 256, 512, 1024};
    return baseRate / dividers[(int)rate];
}

const char* ADS1298R::getErrorString(ErrorCode error) {
    switch(error) {
        case ERR_NONE: return "No error";
        case ERR_SPI_INIT_FAILED: return "SPI initialization failed";
        case ERR_INVALID_DEVICE_ID: return "Invalid device ID";
        case ERR_NOT_INITIALIZED: return "Device not initialized";
        case ERR_ALREADY_ACQUIRING: return "Already acquiring data";
        case ERR_NOT_ACQUIRING: return "Not acquiring data";
        case ERR_INVALID_CHANNEL: return "Invalid channel number";
        case ERR_DATA_OVERRUN: return "Data overrun detected";
        case ERR_REGISTER_VERIFY_FAILED: return "Register verification failed";
        default: return "Unknown error";
    }
}

void ADS1298R::sendCommand(uint8_t command) {
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(command);
    PicoSPI0.endTransaction();
}

void ADS1298R::updateLeadOffStatus(Data& data) {
    uint8_t loffStatP = readRegister(REG_LOFF_STATP);
    uint8_t loffStatN = readRegister(REG_LOFF_STATN);
    
    for (int i = 0; i < 8; i++) {
        data.leadOffStatus[i] = (loffStatP & (1 << i)) || (loffStatN & (1 << i));
    }
}

void ADS1298R::handleInterrupt() {
    if (instance) {
        if (instance->dataReady) {
            instance->dataOverrun = true;
        }
        instance->dataReady = true;
    }
}

int ADS1298R::getGainValue(Gain gain) {
    switch(gain) {
        case GAIN_1: return 1;
        case GAIN_2: return 2;
        case GAIN_3: return 3;
        case GAIN_4: return 4;
        case GAIN_6: return 6;
        case GAIN_8: return 8;
        case GAIN_12: return 12;
        default: return 6;
    }
}