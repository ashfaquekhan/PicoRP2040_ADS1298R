/**
 * @file main.cpp
 * @brief ADS1298R Library Usage Examples
 * 
 */

#include <Arduino.h>
#include "ADS1298R.h"

// Pin definitions
#define ADS1298_CS_PIN    5
#define ADS1298_DRDY_PIN  6
#define ADS1298_START_PIN 7
#define ADS1298_RESET_PIN 8
#define SPI_FREQ          1000000  // 1 MHz

// Create ADS1298R instance
ADS1298R ecg(ADS1298_CS_PIN, ADS1298_DRDY_PIN, ADS1298_START_PIN, 
             ADS1298_RESET_PIN, SPI_FREQ);

// Select which example to run (1-6)
#define EXAMPLE_MODE 1

// =============================================================================
// Example 1: 6-Lead ECG Configuration
// =============================================================================
void configure6LeadECG() {
    // Set high resolution mode at 500 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure channels
    // CH1: Power down
    ecg.setChannel(0, ADS1298R::ChannelConfig(false));
    
    // CH2: Lead I (LA-RA) - Gain 6, normal electrode
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH3: Lead II (LL-RA) - Gain 6, normal electrode
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH4-8: Power down
    for(int i = 3; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    // Configure RLD (Right Leg Drive) - channels 2 and 3
    ecg.setRLD(0x06, 0x06);  // Bits 1,2 = channels 2,3
    
    Serial.println("Configured for 6-lead ECG");
}

// =============================================================================
// Example 2: 12-Lead ECG Configuration
// =============================================================================
void configure12LeadECG() {
    // Set high resolution mode at 1000 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_2K_1K);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Enable WCT (Wilson Central Terminal) in CONFIG4
    uint8_t config4 = ecg.readRegister(ADS1298R::REG_CONFIG4);
    config4 |= 0x04;  // PD_WCT=1
    ecg.writeRegister(ADS1298R::REG_CONFIG4, config4);
    
    // Configure all 8 channels
    // CH1-6: V1-V6 precordial leads
    for(int i = 0; i < 6; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    }
    
    // CH7: LA for Lead I
    ecg.setChannel(6, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH8: LL for Lead II
    ecg.setChannel(7, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // Configure WCT registers
    ecg.writeRegister(ADS1298R::REG_WCT1, 0xC0);  // Enable WCT with CH7
    ecg.writeRegister(ADS1298R::REG_WCT2, 0x80);  // Use CH8
    
    // Configure RLD for all channels
    ecg.setRLD(0xFF, 0xFF);
    
    Serial.println("Configured for 12-lead ECG");
}

// =============================================================================
// Example 3: Filtered ECG Configuration
// =============================================================================
void configureFilteredECG() {
    // Start with 6-lead configuration
    configure6LeadECG();
    
    // Enable filters
    ecg.setNotchFilter(true, false);  // Enable 50Hz notch filter
    
    // Enable AC lead-off detection (includes high-pass filter)
    ecg.setLeadOff(ADS1298R::LEADOFF_24NA, ADS1298R::LEADOFF_AC_QUARTER, 0x00);
    
    // Configure lead-off detection for channels 2,3
    ecg.writeRegister(ADS1298R::REG_LOFF_SENSP, 0x06);  // Positive
    ecg.writeRegister(ADS1298R::REG_LOFF_SENSN, 0x06);  // Negative
    
    Serial.println("Configured for filtered ECG with 50Hz notch");
}

// =============================================================================
// Example 4: Respiration + ECG Configuration
// =============================================================================
void configureRespirationECG() {
    // Set high resolution mode at 500 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure respiration
    ecg.setRespiration(ADS1298R::RESP_INTERNAL_32K, ADS1298R::RESP_PHASE_135);
    
    // CH1: Respiration measurement
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_4, ADS1298R::NORMAL_ELECTRODE));
    
    // CH2-3: ECG channels
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH4-7: Power down
    for(int i = 3; i < 7; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    // CH8: Respiration reference
    ecg.setChannel(7, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_4, ADS1298R::NORMAL_ELECTRODE));
    
    // Configure RLD for ECG channels
    ecg.setRLD(0x06, 0x06);
    
    Serial.println("Configured for respiration + ECG");
}

// =============================================================================
// Example 5: High-Speed Acquisition
// =============================================================================
void configureHighSpeed() {
    // Set high resolution mode at 8000 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_8K_4K);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure only 2 channels for high-speed
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // Power down unused channels
    for(int i = 2; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    // Configure RLD
    ecg.setRLD(0x03, 0x03);
    
    Serial.println("Configured for high-speed acquisition (8kSPS)");
}

// =============================================================================
// Example 6: Custom Multi-Signal Configuration
// =============================================================================
void configureCustom() {
    // Set high resolution mode at 2000 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_4K_2K);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure different signals with different gains
    // CH1-2: EEG signals (high gain)
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_12, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_12, ADS1298R::NORMAL_ELECTRODE));
    
    // CH3-4: ECG signals (medium gain)
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(3, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH5-6: EMG signals (low gain)
    ecg.setChannel(4, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_2, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(5, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_2, ADS1298R::NORMAL_ELECTRODE));
    
    // CH7: Temperature monitoring
    ecg.setChannel(6, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_1, ADS1298R::TEMPERATURE));
    
    // CH8: Supply voltage monitoring
    ecg.setChannel(7, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_1, ADS1298R::MVDD_SUPPLY));
    
    // Configure RLD for ECG channels only
    ecg.setRLD(0x0C, 0x0C);  // Channels 3,4
    
    // Enable filters
    ecg.setNotchFilter(false, true);  // 60Hz notch
    
    Serial.println("Configured for custom multi-signal acquisition");
}

// =============================================================================
// Lead Calculations
// =============================================================================

struct SixLeadECG {
    int32_t lead_I;
    int32_t lead_II;
    int32_t lead_III;
    int32_t aVR;
    int32_t aVL;
    int32_t aVF;
};

SixLeadECG calculate6LeadECG(const ADS1298R::Data& data) {
    SixLeadECG leads;
    leads.lead_I = data.channels[1];    // LA - RA
    leads.lead_II = data.channels[2];   // LL - RA
    leads.lead_III = leads.lead_II - leads.lead_I;
    leads.aVR = -(leads.lead_I + leads.lead_II) / 2;
    leads.aVL = leads.lead_I - leads.lead_II / 2;
    leads.aVF = leads.lead_II - leads.lead_I / 2;
    return leads;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("ADS1298R Example Starting...");
    
    // Initialize the device
    if (!ecg.begin()) {
        Serial.print("ERROR: Initialization failed - ");
        Serial.println(ADS1298R::getErrorString(ecg.getLastError()));
        while(1);
    }
    
    Serial.print("Device ID: 0x");
    Serial.println(ecg.getDeviceID(), HEX);
    
    // Configure based on selected example
    #if EXAMPLE_MODE == 1
        configure6LeadECG();
    #elif EXAMPLE_MODE == 2
        configure12LeadECG();
    #elif EXAMPLE_MODE == 3
        configureFilteredECG();
    #elif EXAMPLE_MODE == 4
        configureRespirationECG();
    #elif EXAMPLE_MODE == 5
        configureHighSpeed();
    #elif EXAMPLE_MODE == 6
        configureCustom();
    #endif
    
    // Optional: Set test signal for verification
    // ecg.setTestSignal(ADS1298R::TEST_1MV_FAST);
    
    // Start data acquisition
    ecg.startAcquisition();
    Serial.println("Acquisition started!\n");
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    // Check for new data
    if (ecg.isDataReady()) {
        ADS1298R::Data data;
        
        if (ecg.readData(data)) {
            #if EXAMPLE_MODE == 1 || EXAMPLE_MODE == 3
                // 6-lead ECG output
                SixLeadECG leads = calculate6LeadECG(data);
                
                Serial.print(">");
                Serial.print("I:"); Serial.print(leads.lead_I);
                Serial.print(",II:"); Serial.print(leads.lead_II);
                Serial.print(",III:"); Serial.print(leads.lead_III);
                Serial.print(",aVR:"); Serial.print(leads.aVR);
                Serial.print(",aVL:"); Serial.print(leads.aVL);
                Serial.print(",aVF:"); Serial.print(leads.aVF);
                Serial.println();
                
            #elif EXAMPLE_MODE == 2
                // 12-lead ECG output
                Serial.print(">");
                for(int i = 0; i < 8; i++) {
                    Serial.print("CH"); Serial.print(i+1);
                    Serial.print(":"); Serial.print(data.channels[i]);
                    if(i < 7) Serial.print(",");
                }
                Serial.println();
                
            #elif EXAMPLE_MODE == 4
                // Respiration + ECG
                Serial.print(">");
                Serial.print("RESP:"); Serial.print(data.channels[0]);
                Serial.print(",ECG1:"); Serial.print(data.channels[1]);
                Serial.print(",ECG2:"); Serial.print(data.channels[2]);
                Serial.println();
                
            #elif EXAMPLE_MODE == 5
                // High-speed raw data
                Serial.print(data.channels[0]);
                Serial.print(",");
                Serial.println(data.channels[1]);
                
            #elif EXAMPLE_MODE == 6
                // Custom multi-signal with conversions
                Serial.print(">");
                Serial.print("EEG1:"); 
                Serial.print(ADS1298R::toMicrovolts(data.channels[0], ADS1298R::GAIN_12), 1);
                Serial.print("uV,EEG2:");
                Serial.print(ADS1298R::toMicrovolts(data.channels[1], ADS1298R::GAIN_12), 1);
                Serial.print("uV,ECG1:");
                Serial.print(ADS1298R::toMillivolts(data.channels[2], ADS1298R::GAIN_6), 2);
                Serial.print("mV,ECG2:");
                Serial.print(ADS1298R::toMillivolts(data.channels[3], ADS1298R::GAIN_6), 2);
                Serial.print("mV,EMG1:");
                Serial.print(ADS1298R::toMillivolts(data.channels[4], ADS1298R::GAIN_2), 2);
                Serial.print("mV,EMG2:");
                Serial.print(ADS1298R::toMillivolts(data.channels[5], ADS1298R::GAIN_2), 2);
                Serial.print("mV,TEMP:");
                
                // Calculate temperature from channel 7
                float temp_uV = ADS1298R::toMicrovolts(data.channels[6], ADS1298R::GAIN_1);
                float temp_C = ((temp_uV - 145300.0f) / 490.0f) + 25.0f;
                Serial.print(temp_C, 1);
                Serial.print("C,VDD:");
                
                // Supply voltage from channel 8 (divided by 4 internally)
                float vdd = ADS1298R::toVoltage(data.channels[7], ADS1298R::GAIN_1) * 4.0f;
                Serial.print(vdd, 2);
                Serial.println("V");
            #endif
            
            // Check lead-off status (if configured)
            #if EXAMPLE_MODE == 3
                for(int i = 0; i < 8; i++) {
                    if(data.leadOffStatus[i]) {
                        Serial.print("!!! Lead-off detected on channel ");
                        Serial.println(i + 1);
                    }
                }
            #endif
        }
    }
    
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 's':  // Stop
                ecg.stopAcquisition();
                Serial.println("\nAcquisition stopped");
                break;
                
            case 'r':  // Resume
                ecg.startAcquisition();
                Serial.println("\nAcquisition resumed");
                break;
                
            case 'i':  // Info
                Serial.println("\n=== Device Info ===");
                Serial.print("Device ID: 0x");
                Serial.println(ecg.getDeviceID(), HEX);
                Serial.print("Acquiring: ");
                Serial.println(ecg.isAcquiring() ? "Yes" : "No");
                break;
                
            case 't':  // Test signal toggle
                static bool testEnabled = false;
                testEnabled = !testEnabled;
                ecg.setTestSignal(testEnabled ? ADS1298R::TEST_1MV_FAST : ADS1298R::TEST_DISABLED);
                Serial.print("\nTest signal: ");
                Serial.println(testEnabled ? "ON" : "OFF");
                break;
                
            case '?':  // Help
                Serial.println("\n=== Commands ===");
                Serial.println("s - Stop acquisition");
                Serial.println("r - Resume acquisition");
                Serial.println("i - Device info");
                Serial.println("t - Toggle test signal");
                Serial.println("? - Help");
                break;
        }
    }
}