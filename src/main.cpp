/**
 * @file main.cpp
 * @brief ADS1298R Library - 6-Lead and 12-Lead ECG Examples
 * @author Ashfaque Khan
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

// Select ECG mode: 1 = 6-Lead, 2 = 12-Lead
#define ECG_MODE 2

// =============================================================================
// Lead calculation structures
// =============================================================================

struct SixLeadECG {
    int32_t aVL;
    int32_t aVF;
    int32_t aVR;
    int32_t L1;   // Lead I
    int32_t L2;   // Lead II
    int32_t L3;   // Lead III
};

// =============================================================================
// Lead Calculations
// =============================================================================;

struct TwelveLeadECG {
    int32_t aVL;
    int32_t aVF;
    int32_t aVR;
    int32_t L1;   // Lead I
    int32_t L2;   // Lead II
    int32_t L3;   // Lead III
    int32_t V1;   // Precordial leads
    int32_t V2;
    int32_t V3;
    int32_t V4;
    int32_t V5;
    int32_t V6;
};

// =============================================================================
// 6-Lead ECG Configuration
// =============================================================================
void configure6LeadECG() {
    // Set high resolution mode at 500 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    
    // Set internal 2.4V reference
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure channels for limb leads
    // CH1: RA (Right Arm) - reference
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH2: LA (Left Arm)
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH3: LL (Left Leg)
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH4-8: Power down unused channels
    for(int i = 3; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    // Configure RLD (Right Leg Drive) for channels 1,2,3
    ecg.setRLD(0x07, 0x07);  // Bits 0,1,2 = channels 1,2,3
    
    Serial.println("Configured for 6-lead ECG (aVL, aVF, aVR, L1, L2, L3)");
}

// =============================================================================
// 12-Lead ECG Configuration (Matching ADS1298R EVM Firmware)
// =============================================================================
void configure12LeadECG() {
    // Set high resolution mode at 1000 SPS
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    
    // Set internal 2.4V reference with proper CONFIG3 setup
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure CONFIG4 register for WCT operation
    uint8_t config4 = 0x00;
    config4 |= 0x04;  // WCT_TO_RLD=1 (connect WCT to RLD for better CMR)
    config4 |= 0x02;  // PD_LOFF_COMP=1 (enable lead-off comparators)
    ecg.writeRegister(ADS1298R::REG_CONFIG4, config4);
    
    // Configure Wilson Central Terminal (WCT) registers
    // Based on EVM firmware: "route CH2P, CH2M, and CH3P (RA, LA, LL) to internal buffers"
    // From jumper configuration:
    // - CH2+ = LA (Left Arm)  
    // - CH2- = RA (Right Arm)
    // - CH3+ = LL (Left Leg)
    // WCT = (RA + LA + LL)/3
    
    // WCT1 Register: Configure WCTA (typically RA electrode)
    uint8_t wct1 = 0x00;
    wct1 |= 0x08;   // PD_WCTA=1 (power on WCTA)
    wct1 |= 0x03;   // WCTA[2:0]=011 (CH2 negative input = RA)
    ecg.writeRegister(ADS1298R::REG_WCT1, wct1);
    
    // WCT2 Register: Configure WCTB (typically LA) and WCTC (typically LL)
    uint8_t wct2 = 0x00;
    wct2 |= 0x80;   // PD_WCTC=1 (power on WCTC)
    wct2 |= 0x40;   // PD_WCTB=1 (power on WCTB) 
    wct2 |= 0x10;   // WCTB[2:0]=010 (CH2 positive input = LA)
    wct2 |= 0x04;   // WCTC[2:0]=100 (CH3 positive input = LL)
    ecg.writeRegister(ADS1298R::REG_WCT2, wct2);
    
    // Configure all 8 channels according to Table 2:
    
    // CH1: V6 = V6 - WCT (requires JP33 jumpers: V6 to CH1+, WCT to CH1-)
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH2: LEAD I = LA - RA (hardware configured: LA to CH2+, RA to CH2-)
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH3: LEAD II = LL - RA (hardware configured: LL to CH3+, RA to CH3-)
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH4: V2 = V2 - WCT (requires JP30 jumpers: V2 to CH4+, WCT to CH4-)
    ecg.setChannel(3, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH5: V3 = V3 - WCT (requires JP29 jumpers: V3 to CH5+, WCT to CH5-)
    ecg.setChannel(4, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH6: V4 = V4 - WCT (requires JP28 jumpers: V4 to CH6+, WCT to CH6-)
    ecg.setChannel(5, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH7: V5 = V5 - WCT (requires JP27 jumpers: V5 to CH7+, WCT to CH7-)
    ecg.setChannel(6, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // CH8: V1 = V1 - WCT (requires JP26 jumpers: V1 to CH8+, WCT to CH8-)
    ecg.setChannel(7, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // Configure RLD (Right Leg Drive) for limb leads
    // Use channels 2,3 which contain the RA, LA, LL electrode data
    ecg.setRLD(0x06, 0x06);  // Channels 2,3 (bits 1,2 set)
    
    Serial.println("Configured for 12-lead ECG with corrected WCT setup");
    Serial.println("WARNING: Requires EVM hardware with proper jumper configuration!");
    Serial.println("JP26-JP30: Connect WCT to CH negative inputs, V1-V5 to CH positive inputs");
    Serial.println("JP31: RA to CH3-, LL to CH3+");
    Serial.println("JP32: RA to CH2-, LA to CH2+");
    Serial.println("JP33: V6 to CH1+, WCT to CH1-");
}

// =============================================================================
// Register Verification Functions
// =============================================================================

void verifyRegisterConfiguration() {
    Serial.println("\n=== Register Configuration Verification ===");
    
    // Read and display key registers
    uint8_t deviceId = ecg.readRegister(ADS1298R::REG_ID);
    uint8_t config1 = ecg.readRegister(ADS1298R::REG_CONFIG1);
    uint8_t config2 = ecg.readRegister(ADS1298R::REG_CONFIG2);
    uint8_t config3 = ecg.readRegister(ADS1298R::REG_CONFIG3);
    uint8_t config4 = ecg.readRegister(ADS1298R::REG_CONFIG4);
    uint8_t wct1 = ecg.readRegister(ADS1298R::REG_WCT1);
    uint8_t wct2 = ecg.readRegister(ADS1298R::REG_WCT2);
    uint8_t rldSensP = ecg.readRegister(ADS1298R::REG_RLD_SENSP);
    uint8_t rldSensN = ecg.readRegister(ADS1298R::REG_RLD_SENSN);
    
    Serial.print("Device ID: 0x"); Serial.println(deviceId, HEX);
    Serial.print("CONFIG1: 0x"); Serial.println(config1, HEX);
    Serial.print("CONFIG2: 0x"); Serial.println(config2, HEX);
    Serial.print("CONFIG3: 0x"); Serial.println(config3, HEX);
    Serial.print("CONFIG4: 0x"); Serial.println(config4, HEX);
    Serial.print("WCT1: 0x"); Serial.println(wct1, HEX);
    Serial.print("WCT2: 0x"); Serial.println(wct2, HEX);
    Serial.print("RLD_SENSP: 0x"); Serial.println(rldSensP, HEX);
    Serial.print("RLD_SENSN: 0x"); Serial.println(rldSensN, HEX);
    
    // Display channel configurations
    Serial.println("\nChannel Configurations:");
    for(int i = 0; i < 8; i++) {
        uint8_t chSet = ecg.readRegister(ADS1298R::REG_CH1SET + i);
        Serial.print("CH"); Serial.print(i+1); 
        Serial.print("SET: 0x"); Serial.print(chSet, HEX);
        Serial.print(" ("); 
        if(chSet & 0x80) {
            Serial.print("POWERED DOWN");
        } else {
            Serial.print("ENABLED, GAIN=");
            uint8_t gain = (chSet >> 4) & 0x07;
            switch(gain) {
                case 0: Serial.print("6"); break;
                case 1: Serial.print("1"); break;
                case 2: Serial.print("2"); break;
                case 3: Serial.print("3"); break;
                case 4: Serial.print("4"); break;
                case 5: Serial.print("8"); break;
                case 6: Serial.print("12"); break;
                default: Serial.print("?"); break;
            }
            Serial.print(", MUX=");
            uint8_t mux = chSet & 0x07;
            switch(mux) {
                case 0: Serial.print("NORMAL"); break;
                case 1: Serial.print("SHORTED"); break;
                case 2: Serial.print("RLD_MEAS"); break;
                case 3: Serial.print("MVDD"); break;
                case 4: Serial.print("TEMP"); break;
                case 5: Serial.print("TEST"); break;
                default: Serial.print("RESERVED"); break;
            }
        }
        Serial.println(")");
    }
    
    #if ECG_MODE == 2
        // Verify WCT configuration for 12-lead mode
        Serial.println("\nWCT Configuration Analysis:");
        Serial.print("WCTA enabled: "); Serial.println((wct1 & 0x08) ? "YES" : "NO");
        Serial.print("WCTB enabled: "); Serial.println((wct2 & 0x40) ? "YES" : "NO");
        Serial.print("WCTC enabled: "); Serial.println((wct2 & 0x80) ? "YES" : "NO");
        
        // Decode WCTA input (should be 011 = CH2-)
        uint8_t wcta_input = wct1 & 0x07;
        Serial.print("WCTA input: ");
        if(wcta_input == 0x03) Serial.println("CH2- (RA) ✓");
        else Serial.print("UNEXPECTED: "); Serial.println(wcta_input, BIN);
        
        // Decode WCTB input (should be 010 = CH2+)
        uint8_t wctb_input = (wct2 >> 3) & 0x07;
        Serial.print("WCTB input: ");
        if(wctb_input == 0x02) Serial.println("CH2+ (LA) ✓");
        else Serial.print("UNEXPECTED: "); Serial.println(wctb_input, BIN);
        
        // Decode WCTC input (should be 100 = CH3+)
        uint8_t wctc_input = wct2 & 0x07;
        Serial.print("WCTC input: ");
        if(wctc_input == 0x04) Serial.println("CH3+ (LL) ✓");
        else Serial.print("UNEXPECTED: "); Serial.println(wctc_input, BIN);
        
        Serial.print("WCT to RLD: "); Serial.println((config4 & 0x04) ? "CONNECTED" : "DISCONNECTED");
        
        Serial.println("\nExpected WCT = (RA + LA + LL)/3 where:");
        Serial.println("- RA comes from CH2- electrode"); 
        Serial.println("- LA comes from CH2+ electrode");
        Serial.println("- LL comes from CH3+ electrode");
        
        Serial.println("\nHardware Requirements:");
        Serial.println("JP32: ECG_RA to CH2-, ECG_LA to CH2+");
        Serial.println("JP31: ECG_RA to CH3-, ECG_LL to CH3+");
        Serial.println("JP26-JP30: WCT to CH negative inputs, V1-V5 to CH positive inputs");
        Serial.println("JP33: ECG_V6 to CH1+, WCT to CH1-");
    #endif
    
    Serial.println("=== End Verification ===\n");
}

// =============================================================================
// Lead Calculations  
// =============================================================================

SixLeadECG calculate6LeadECG(const ADS1298R::Data& data) {
    SixLeadECG leads;
    
    // Raw electrode readings
    int32_t RA = data.channels[0];  // Right Arm
    int32_t LA = data.channels[1];  // Left Arm  
    int32_t LL = data.channels[2];  // Left Leg
    
    // Calculate standard limb leads
    leads.L1 = LA - RA;           // Lead I = LA - RA
    leads.L2 = LL - RA;           // Lead II = LL - RA
    leads.L3 = LL - LA;           // Lead III = LL - LA
    
    // Calculate augmented limb leads
    leads.aVR = -(LA + LL) / 2;   // aVR = -(LA + LL)/2
    leads.aVL = LA - (RA + LL) / 2;  // aVL = LA - (RA + LL)/2
    leads.aVF = LL - (RA + LA) / 2;  // aVF = LL - (RA + LA)/2
    
    return leads;
}

TwelveLeadECG calculate12LeadECG(const ADS1298R::Data& data) {
    TwelveLeadECG leads;
    
    // According to Table 2: ADS1298R Lead Measurements
    // CH2 = LEAD I = LA - RA (computed in analog domain)
    // CH3 = LEAD II = LL - RA (computed in analog domain)
    leads.L1 = data.channels[1];  // Channel 2: LEAD I = LA - RA
    leads.L2 = data.channels[2];  // Channel 3: LEAD II = LL - RA
    
    // According to Table 3: Derived Lead Calculations
    leads.L3 = leads.L2 - leads.L1;           // LEAD III = LEAD II - LEAD I
    leads.aVR = -(leads.L1 + leads.L2) / 2;   // aVR = -(LEAD I + LEAD II) / 2
    leads.aVL = leads.L1 - leads.L2 / 2;      // aVL = LEAD I - LEAD II / 2
    leads.aVF = leads.L2 - leads.L1 / 2;      // aVF = LEAD II - LEAD I / 2
    
    // Precordial leads (already computed by hardware with respect to WCT)
    // According to Table 2 channel mapping:
    leads.V6 = data.channels[0];  // Channel 1: V6 = V6 - WCT
    leads.V2 = data.channels[3];  // Channel 4: V2 = V2 - WCT
    leads.V3 = data.channels[4];  // Channel 5: V3 = V3 - WCT
    leads.V4 = data.channels[5];  // Channel 6: V4 = V4 - WCT
    leads.V5 = data.channels[6];  // Channel 7: V5 = V5 - WCT
    leads.V1 = data.channels[7];  // Channel 8: V1 = V1 - WCT
    
    return leads;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("ADS1298R ECG Monitor Starting...");
    
    // Initialize the device
    if (!ecg.begin()) {
        Serial.print("ERROR: Initialization failed - ");
        Serial.println(ADS1298R::getErrorString(ecg.getLastError()));
        while(1);
    }
    
    Serial.print("Device ID: 0x");
    Serial.println(ecg.getDeviceID(), HEX);
    
    // Configure based on selected mode
    #if ECG_MODE == 1
        configure6LeadECG();
    #elif ECG_MODE == 2
        configure12LeadECG();
    #endif
    
    // Start data acquisition
    ecg.startAcquisition();
    Serial.println("Acquisition started!\n");
    
    #if ECG_MODE == 2
        Serial.println("⚠️ IMPORTANT: V1-V6 leads require proper hardware setup!");
        Serial.println("If V1-V6 show poor/incorrect data, check jumper configurations.");
        Serial.println("Use 'v' command to verify register settings match expected values.");
    #endif
    
    // Print header based on mode
    #if ECG_MODE == 1
        Serial.println("Format: aVL:<value>,aVF:<value>,aVR:<value>,L1:<value>,L2:<value>,L3:<value>");
    #elif ECG_MODE == 2
        Serial.println("Format: aVL:<value>,aVF:<value>,aVR:<value>,L1:<value>,L2:<value>,L3:<value>,V1:<value>,V2:<value>,V3:<value>,V4:<value>,V5:<value>,V6:<value>");
    #endif
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    // Check for new data
    if (ecg.isDataReady()) {
        ADS1298R::Data data;
        
        if (ecg.readData(data)) {
            #if ECG_MODE == 1
                // 6-lead ECG output
                SixLeadECG leads = calculate6LeadECG(data);
                
                Serial.print(">");
                Serial.print("aVL:"); Serial.print(leads.aVL); Serial.print(",");
                Serial.print("aVF:"); Serial.print(leads.aVF); Serial.print(",");
                Serial.print("aVR:"); Serial.print(leads.aVR); Serial.print(",");
                Serial.print("L1:"); Serial.print(leads.L1); Serial.print(",");
                Serial.print("L2:"); Serial.print(leads.L2); Serial.print(",");
                Serial.print("L3:"); Serial.print(leads.L3);
                Serial.println();
                
            #elif ECG_MODE == 2
                // 12-lead ECG output
                TwelveLeadECG leads = calculate12LeadECG(data);
                
                Serial.print(">");
                Serial.print("aVL:"); Serial.print(leads.aVL); Serial.print(",");
                Serial.print("aVF:"); Serial.print(leads.aVF); Serial.print(",");
                Serial.print("aVR:"); Serial.print(leads.aVR); Serial.print(",");
                Serial.print("L1:"); Serial.print(leads.L1); Serial.print(",");
                Serial.print("L2:"); Serial.print(leads.L2); Serial.print(",");
                Serial.print("L3:"); Serial.print(leads.L3); Serial.print(",");
                Serial.print("V1:"); Serial.print(leads.V1); Serial.print(",");
                Serial.print("V2:"); Serial.print(leads.V2); Serial.print(",");
                Serial.print("V3:"); Serial.print(leads.V3); Serial.print(",");
                Serial.print("V4:"); Serial.print(leads.V4); Serial.print(",");
                Serial.print("V5:"); Serial.print(leads.V5); Serial.print(",");
                Serial.print("V6:"); Serial.print(leads.V6);
                Serial.println();
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
                #if ECG_MODE == 1
                    Serial.println("Mode: 6-Lead ECG");
                #elif ECG_MODE == 2
                    Serial.println("Mode: 12-Lead ECG");
                #endif
                break;
                
            case 't':  // Test signal toggle
                static bool testEnabled = false;
                testEnabled = !testEnabled;
                ecg.setTestSignal(testEnabled ? ADS1298R::TEST_1MV_FAST : ADS1298R::TEST_DISABLED);
                Serial.print("\nTest signal: ");
                Serial.println(testEnabled ? "ON" : "OFF");
                break;
                
            case 'v':  // Verify configuration
                verifyRegisterConfiguration();
                break;
                
            case 'd':  // Debug mode - show raw channel data
                Serial.println("\nDEBUG: Raw Channel Data");
                if (ecg.isDataReady()) {
                    ADS1298R::Data debugData;
                    if (ecg.readData(debugData)) {
                        for(int i = 0; i < 8; i++) {
                            Serial.print("CH"); Serial.print(i+1); 
                            Serial.print(": "); Serial.print(debugData.channels[i]);
                            Serial.print(" (0x"); Serial.print(debugData.channels[i], HEX);
                            Serial.println(")");
                        }
                        Serial.print("Status: 0x"); Serial.println(debugData.status, HEX);
                    }
                } else {
                    Serial.println("No data ready");
                }
                break;
                
            case '?':  // Help
                Serial.println("\n=== Commands ===");
                Serial.println("s - Stop acquisition");
                Serial.println("r - Resume acquisition");
                Serial.println("i - Device info");
                Serial.println("t - Toggle test signal");
                Serial.println("v - Verify register configuration");
                Serial.println("d - Debug: show raw channel data");
                Serial.println("? - Help");
                #if ECG_MODE == 1
                    Serial.println("\nMode: 6-Lead ECG");
                    Serial.println("Output: aVL:<value>,aVF:<value>,aVR:<value>,L1:<value>,L2:<value>,L3:<value>");
                #elif ECG_MODE == 2
                    Serial.println("\nMode: 12-Lead ECG"); 
                    Serial.println("Output: aVL:<value>,aVF:<value>,aVR:<value>,L1:<value>,L2:<value>,L3:<value>,V1:<value>,V2:<value>,V3:<value>,V4:<value>,V5:<value>,V6:<value>");
                    Serial.println("Note: V1-V6 require proper EVM jumper configuration for accurate readings");
                #endif
                break;
        }
    }
}