#pragma once

#include <Arduino.h>
#include <PicoSPI.h>

/**
 * @file ADS1298R.h
 * @brief Y3X ADS1298R Library
 * @author Ashfaque Khan
 */

class ADS1298R {
public:
    // =============================================================================
    // ENUMERATIONS
    // =============================================================================
    
    enum PowerMode {
        LOW_POWER = 0,
        HIGH_RESOLUTION = 1
    };
    
    enum DataRate {
        RATE_32K_16K = 0,   // HR: 32kSPS, LP: 16kSPS
        RATE_16K_8K = 1,    // HR: 16kSPS, LP: 8kSPS
        RATE_8K_4K = 2,     // HR: 8kSPS,  LP: 4kSPS
        RATE_4K_2K = 3,     // HR: 4kSPS,  LP: 2kSPS
        RATE_2K_1K = 4,     // HR: 2kSPS,  LP: 1kSPS
        RATE_1K_500 = 5,    // HR: 1kSPS,  LP: 500SPS
        RATE_500_250 = 6    // HR: 500SPS, LP: 250SPS
    };
    
    enum Gain {
        GAIN_1 = 0x10,      // ±2.4V range
        GAIN_2 = 0x20,      // ±1.2V range
        GAIN_3 = 0x30,      // ±800mV range
        GAIN_4 = 0x40,      // ±600mV range
        GAIN_6 = 0x00,      // ±400mV range (typical ECG)
        GAIN_8 = 0x50,      // ±300mV range
        GAIN_12 = 0x60      // ±200mV range
    };
    
    enum InputMux {
        NORMAL_ELECTRODE = 0x00,
        INPUT_SHORTED = 0x01,
        RLD_MEASUREMENT = 0x02,
        MVDD_SUPPLY = 0x03,
        TEMPERATURE = 0x04,
        TEST_SIGNAL = 0x05,
        RLD_DRP = 0x06,
        RLD_DRN = 0x07
    };
    
    enum ReferenceVoltage {
        INTERNAL_2_4V = 0x00,
        INTERNAL_4V = 0x20,
        EXTERNAL_REF = 0x40
    };
    
    enum TestSignal {
        TEST_DISABLED = 0x00,
        TEST_1MV_SLOW = 0x01,
        TEST_1MV_FAST = 0x02,
        TEST_2MV_SLOW = 0x05,
        TEST_2MV_FAST = 0x06
    };
    
    enum LeadOffCurrent {
        LEADOFF_DISABLED = 0x00,
        LEADOFF_6NA = 0x04,
        LEADOFF_12NA = 0x08,
        LEADOFF_18NA = 0x0C,
        LEADOFF_24NA = 0x10
    };
    
    enum LeadOffFrequency {
        LEADOFF_DC = 0x00,
        LEADOFF_AC_QUARTER = 0x01,  // fDR/4
        LEADOFF_AC_HALF = 0x02,     // fDR/2
        LEADOFF_AC_DR = 0x03        // fDR
    };
    
    enum RespirationMode {
        RESP_DISABLED = 0x00,
        RESP_EXTERNAL = 0x01,
        RESP_INTERNAL_32K = 0x02,
        RESP_INTERNAL_64K = 0x03
    };
    
    enum RespirationPhase {
        RESP_PHASE_22_5 = 0x00,
        RESP_PHASE_45 = 0x01,
        RESP_PHASE_67_5 = 0x02,
        RESP_PHASE_90 = 0x03,
        RESP_PHASE_112_5 = 0x04,
        RESP_PHASE_135 = 0x05,
        RESP_PHASE_157_5 = 0x06
    };
    
    enum ErrorCode {
        ERR_NONE = 0,
        ERR_SPI_INIT_FAILED,
        ERR_INVALID_DEVICE_ID,
        ERR_NOT_INITIALIZED,
        ERR_ALREADY_ACQUIRING,
        ERR_NOT_ACQUIRING,
        ERR_INVALID_CHANNEL,
        ERR_DATA_OVERRUN,
        ERR_REGISTER_VERIFY_FAILED
    };

    // =============================================================================
    // DATA STRUCTURES
    // =============================================================================
    
    struct Data {
        uint32_t status;
        int32_t channels[8];
        uint32_t timestamp;
        bool leadOffStatus[8];
        bool valid;
        
        Data() {
            status = 0;
            timestamp = 0;
            valid = false;
            for(int i = 0; i < 8; i++) {
                channels[i] = 0;
                leadOffStatus[i] = false;
            }
        }
    };
    
    struct ChannelConfig {
        bool enabled;
        Gain gain;
        InputMux input;
        
        ChannelConfig(bool en = false, Gain g = GAIN_6, InputMux inp = NORMAL_ELECTRODE) 
            : enabled(en), gain(g), input(inp) {}
    };

    // =============================================================================
    // REGISTER ADDRESSES
    // =============================================================================
    
    static const uint8_t REG_ID = 0x00;
    static const uint8_t REG_CONFIG1 = 0x01;
    static const uint8_t REG_CONFIG2 = 0x02;
    static const uint8_t REG_CONFIG3 = 0x03;
    static const uint8_t REG_LOFF = 0x04;
    static const uint8_t REG_CH1SET = 0x05;
    static const uint8_t REG_CH2SET = 0x06;
    static const uint8_t REG_CH3SET = 0x07;
    static const uint8_t REG_CH4SET = 0x08;
    static const uint8_t REG_CH5SET = 0x09;
    static const uint8_t REG_CH6SET = 0x0A;
    static const uint8_t REG_CH7SET = 0x0B;
    static const uint8_t REG_CH8SET = 0x0C;
    static const uint8_t REG_RLD_SENSP = 0x0D;
    static const uint8_t REG_RLD_SENSN = 0x0E;
    static const uint8_t REG_LOFF_SENSP = 0x0F;
    static const uint8_t REG_LOFF_SENSN = 0x10;
    static const uint8_t REG_LOFF_FLIP = 0x11;
    static const uint8_t REG_LOFF_STATP = 0x12;
    static const uint8_t REG_LOFF_STATN = 0x13;
    static const uint8_t REG_GPIO = 0x14;
    static const uint8_t REG_PACE = 0x15;
    static const uint8_t REG_RESP = 0x16;
    static const uint8_t REG_CONFIG4 = 0x17;
    static const uint8_t REG_WCT1 = 0x18;
    static const uint8_t REG_WCT2 = 0x19;

private:
    // Hardware pins
    uint8_t pinCS, pinDRDY, pinSTART, pinRESET;
    uint32_t spiFrequency;
    
    // State
    bool initialized;
    bool acquiring;
    volatile bool dataReady;
    volatile bool dataOverrun;
    ErrorCode lastError;
    
    // Static instance for interrupt
    static ADS1298R* instance;
    
    // Commands
    static const uint8_t CMD_WAKEUP = 0x02;
    static const uint8_t CMD_STANDBY = 0x04;
    static const uint8_t CMD_RESET = 0x06;
    static const uint8_t CMD_START = 0x08;
    static const uint8_t CMD_STOP = 0x0A;
    static const uint8_t CMD_RDATAC = 0x10;
    static const uint8_t CMD_SDATAC = 0x11;
    static const uint8_t CMD_RREG = 0x20;
    static const uint8_t CMD_WREG = 0x40;

public:
    // =============================================================================
    // PUBLIC METHODS
    // =============================================================================
    
    // Constructor
    ADS1298R(uint8_t cs, uint8_t drdy, uint8_t start, uint8_t reset, uint32_t spiFreq = 1000000);
    
    // Basic operations
    bool begin();
    bool startAcquisition();
    bool stopAcquisition();
    bool isDataReady();
    bool readData(Data& data);
    
    // Register access
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    
    // Configuration helpers
    void setPowerMode(PowerMode mode, DataRate rate);
    void setChannel(uint8_t channel, const ChannelConfig& config);
    void setReference(ReferenceVoltage ref);
    void setTestSignal(TestSignal mode);
    void setRLD(uint8_t positiveMask, uint8_t negativeMask);
    void setLeadOff(LeadOffCurrent current, LeadOffFrequency freq, uint8_t threshold = 0x00);
    void setRespiration(RespirationMode mode, RespirationPhase phase);
    void setNotchFilter(bool enable50Hz, bool enable60Hz);
    void setGPIO(uint8_t direction, uint8_t data);
    uint8_t readGPIO();
    
    // Utility
    void reset();
    uint8_t getDeviceID();
    bool isAcquiring() const { return acquiring; }
    ErrorCode getLastError() const { return lastError; }
    
    // Data conversion
    static float toVoltage(int32_t raw, Gain gain, float vref = 2.4f);
    static float toMillivolts(int32_t raw, Gain gain, float vref = 2.4f);
    static float toMicrovolts(int32_t raw, Gain gain, float vref = 2.4f);
    static uint16_t getSampleRate(PowerMode power, DataRate rate);
    static const char* getErrorString(ErrorCode error);

private:
    // Internal methods
    void sendCommand(uint8_t command);
    void updateLeadOffStatus(Data& data);
    static void handleInterrupt();
    static int getGainValue(Gain gain);
};