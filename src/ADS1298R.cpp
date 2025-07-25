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
    updatePaceStatus(data);
    
    data.timestamp = micros();
    data.valid = true;
    dataReady = false;
    
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

// ============================================================================
// LEAD-OFF DETECTION FUNCTIONS
// ============================================================================

void ADS1298R::configureLeadOff(const LeadOffConfig& config) {
    // Configure lead-off detection parameters
    setLeadOffCurrent(config.current, config.frequency, config.threshold);
    
    // Set positive and negative sensing masks
    uint8_t positiveMask = 0;
    uint8_t negativeMask = 0;
    
    for(int i = 0; i < 8; i++) {
        if(config.enabledPositive[i]) {
            positiveMask |= (1 << i);
        }
        if(config.enabledNegative[i]) {
            negativeMask |= (1 << i);
        }
    }
    
    writeRegister(REG_LOFF_SENSP, positiveMask);
    writeRegister(REG_LOFF_SENSN, negativeMask);
    
    // Enable lead-off comparators in CONFIG4
    uint8_t config4 = readRegister(REG_CONFIG4);
    config4 |= 0x02;  // PD_LOFF_COMP=1
    writeRegister(REG_CONFIG4, config4);
}

void ADS1298R::enableLeadOff(uint8_t channel, bool enablePositive, bool enableNegative) {
    if (channel >= 8) {
        lastError = ERR_INVALID_CHANNEL;
        return;
    }
    
    uint8_t positiveMask = readRegister(REG_LOFF_SENSP);
    uint8_t negativeMask = readRegister(REG_LOFF_SENSN);
    
    if (enablePositive) {
        positiveMask |= (1 << channel);
    } else {
        positiveMask &= ~(1 << channel);
    }
    
    if (enableNegative) {
        negativeMask |= (1 << channel);
    } else {
        negativeMask &= ~(1 << channel);
    }
    
    writeRegister(REG_LOFF_SENSP, positiveMask);
    writeRegister(REG_LOFF_SENSN, negativeMask);
    
    // Ensure lead-off comparators are enabled
    uint8_t config4 = readRegister(REG_CONFIG4);
    config4 |= 0x02;
    writeRegister(REG_CONFIG4, config4);
}

void ADS1298R::disableLeadOff(uint8_t channel) {
    enableLeadOff(channel, false, false);
}

void ADS1298R::setLeadOffCurrent(LeadOffCurrent current, LeadOffFrequency freq, uint8_t threshold) {
    uint8_t loff = (threshold & 0xE0);  // Bits 7-5: threshold
    loff |= (uint8_t)current;           // Bits 4-2: current
    loff |= (uint8_t)freq;              // Bits 1-0: frequency
    writeRegister(REG_LOFF, loff);
}

bool ADS1298R::getLeadOffStatus(uint8_t channel, bool& positiveOff, bool& negativeOff) {
    if (channel >= 8) {
        lastError = ERR_INVALID_CHANNEL;
        return false;
    }
    
    uint8_t loffStatP = readRegister(REG_LOFF_STATP);
    uint8_t loffStatN = readRegister(REG_LOFF_STATN);
    
    positiveOff = (loffStatP & (1 << channel)) != 0;
    negativeOff = (loffStatN & (1 << channel)) != 0;
    
    return true;
}

uint8_t ADS1298R::getLeadOffStatusByte(bool positive) {
    if (positive) {
        return readRegister(REG_LOFF_STATP);
    } else {
        return readRegister(REG_LOFF_STATN);
    }
}

// ============================================================================
// PACE DETECTION FUNCTIONS
// ============================================================================

void ADS1298R::configurePace(PaceChannel oddChannel, PaceChannel evenChannel) {
    uint8_t paceReg = 0x00;
    
    // Enable pace amplifiers (clear PD_PACE bit)
    paceReg &= ~0x01;
    
    // Configure odd channel (PACEO[1:0] - bits 2:1)
    if (oddChannel != PACE_DISABLED) {
        uint8_t oddChannelCode = 0;
        switch(oddChannel) {
            case PACE_CH1: oddChannelCode = 0x00; break;
            case PACE_CH3: oddChannelCode = 0x01; break;
            case PACE_CH5: oddChannelCode = 0x02; break;
            case PACE_CH7: oddChannelCode = 0x03; break;
            default: oddChannelCode = 0x00; break;
        }
        paceReg |= (oddChannelCode << 1);
    }
    
    // Configure even channel (PACEE[1:0] - bits 4:3)
    if (evenChannel != PACE_DISABLED) {
        uint8_t evenChannelCode = 0;
        switch(evenChannel) {
            case PACE_CH2: evenChannelCode = 0x00; break;
            case PACE_CH4: evenChannelCode = 0x01; break;
            case PACE_CH6: evenChannelCode = 0x02; break;
            case PACE_CH8: evenChannelCode = 0x03; break;
            default: evenChannelCode = 0x00; break;
        }
        paceReg |= (evenChannelCode << 3);
    }
    
    writeRegister(REG_PACE, paceReg);
    
    // Configure GPIO1 as input for external pace detection
    // GPIO direction: 0=output, 1=input
    uint8_t gpioDir = 0x03;  // GPIO1 and GPIO2 as inputs
    uint8_t gpioData = 0x00; // Initial data
    setGPIO(gpioDir, gpioData);
}

void ADS1298R::enablePace(bool enable) {
    uint8_t paceReg = readRegister(REG_PACE);
    
    if (enable) {
        paceReg &= ~0x01;  // Clear PD_PACE bit (enable)
    } else {
        paceReg |= 0x01;   // Set PD_PACE bit (disable)
    }
    
    writeRegister(REG_PACE, paceReg);
}

void ADS1298R::disablePace() {
    uint8_t paceReg = 0x01;  // Power down pace amplifiers
    writeRegister(REG_PACE, paceReg);
}

bool ADS1298R::getPaceStatus(uint8_t paceAmp) {
    if (paceAmp > 1) return false;
    
    // PACE status is typically read from GPIO or status bits
    // This is a simplified implementation - actual pace detection
    // may require external circuitry and GPIO monitoring
    uint8_t gpio = readGPIO();
    
    if (paceAmp == 0) {
        return (gpio & 0x01) != 0;  // GPIO1 for PACE1
    } else {
        return (gpio & 0x02) != 0;  // GPIO2 for PACE2
    }
}

// ============================================================================
// WCT OPTIMIZATION
// ============================================================================

bool ADS1298R::configureWCTOptimized(bool enableChop) {
    if (!initialized) {
        lastError = ERR_NOT_INITIALIZED;
        return false;
    }
    
    // Enable WCT chopping to reduce offset noise in precordial leads
    uint8_t config2 = readRegister(REG_CONFIG2);
    if (enableChop) {
        config2 |= 0x80;  // WCT_CHOP=1
    } else {
        config2 &= ~0x80; // WCT_CHOP=0
    }
    writeRegister(REG_CONFIG2, config2);
    delay(10);
    
    // Configure WCT1: WCTA amplifier (typically RA electrode)
    uint8_t wct1 = 0x0B;  // PD_WCTA=1, WCTA[2:0]=011 (CH2 negative input)
    writeRegister(REG_WCT1, wct1);
    delay(5);
    
    // Configure WCT2: WCTB and WCTC amplifiers (typically LA and LL electrodes)
    uint8_t wct2 = 0xD4;  // PD_WCTC=1, PD_WCTB=1, WCTB=010 (CH2+), WCTC=100 (CH3+)
    writeRegister(REG_WCT2, wct2);
    delay(5);
    
    // Connect WCT to RLD for better common-mode rejection
    uint8_t config4 = readRegister(REG_CONFIG4);
    config4 |= 0x04;  // WCT_TO_RLD=1
    config4 |= 0x02;  // PD_LOFF_COMP=1 (enable lead-off comparators)
    writeRegister(REG_CONFIG4, config4);
    
    // Allow WCT amplifiers to settle
    delay(100);
    
    // Verify configuration
    uint8_t wct1_verify = readRegister(REG_WCT1);
    uint8_t wct2_verify = readRegister(REG_WCT2);
    uint8_t config4_verify = readRegister(REG_CONFIG4);
    
    if (wct1_verify != wct1 || wct2_verify != wct2 || !(config4_verify & 0x04)) {
        lastError = ERR_REGISTER_VERIFY_FAILED;
        return false;
    }
    
    lastError = ERR_NONE;
    return true;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

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

void ADS1298R::updatePaceStatus(Data& data) {
    // PACE detection status is available through GPIO pins when configured
    // Check if PACE amplifiers are enabled first
    uint8_t paceReg = readRegister(REG_PACE);
    bool paceEnabled = !(paceReg & 0x01);  // PD_PACE=0 means enabled
    
    if (!paceEnabled) {
        data.paceDetected[0] = false;
        data.paceDetected[1] = false;
        return;
    }
    
    // Read GPIO status for pace detection
    uint8_t gpio = readGPIO();
    
    // PACE detection typically uses GPIO1 input for external pace detection
    // The actual implementation depends on external circuitry
    data.paceDetected[0] = (gpio & 0x01) != 0;  // PACE1 on GPIO1
    data.paceDetected[1] = (gpio & 0x02) != 0;  // PACE2 on GPIO2
    
    // Alternative: Check status register bits if available
    // Some implementations use status bits in the data stream
    uint32_t status = data.status;
    // Bits in status word may indicate pace detection - this is device specific
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