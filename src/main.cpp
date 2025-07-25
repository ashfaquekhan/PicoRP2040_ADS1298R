/**
 * @file main.cpp
 * @brief ADS1298R Library - 6-Lead and 12-Lead ECG Examples
 * @author Ashfaque Khan
 */

#include <Arduino.h>
#include "ADS1298R.h"

#define ADS1298_CS_PIN    5
#define ADS1298_DRDY_PIN  6
#define ADS1298_START_PIN 7
#define ADS1298_RESET_PIN 8
#define SPI_FREQ          1000000

ADS1298R ecg(ADS1298_CS_PIN, ADS1298_DRDY_PIN, ADS1298_START_PIN, 
             ADS1298_RESET_PIN, SPI_FREQ);

#define ECG_MODE 2  // 1 = 6-Lead, 2 = 12-Lead

struct SixLeadECG {
    int32_t aVL, aVF, aVR, L1, L2, L3;
};

struct TwelveLeadECG {
    int32_t aVL, aVF, aVR, L1, L2, L3, V1, V2, V3, V4, V5, V6;
};

void configure6LeadECG() {
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    
    // Configure channels for limb leads
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    // Power down unused channels
    for(int i = 3; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    // Configure RLD for limb leads
    ecg.setRLD(0x07, 0x07);
    
    // Enable lead-off detection for active channels
    for(int i = 0; i < 3; i++) {
        ecg.enableLeadOff(i, true, true);
    }
    
    Serial.println("Configured for 6-lead ECG with lead-off detection");
}

void configure12LeadECG() {
    // Use 8kSPS for pace detection (minimum required)
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_8K_4K);
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    delay(150);
    
    // Enable notch filters for better V-lead quality
    ecg.setNotchFilter(true, true);
    
    // Configure optimized WCT for precordial leads
    ecg.configureWCTOptimized(true);
    
    // Configure all 8 channels
    for(int i = 0; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    }
    
    // Configure RLD for limb leads (channels 2,3)
    ecg.setRLD(0x06, 0x06);
    
    // Enable lead-off detection for all channels
    for(int i = 0; i < 8; i++) {
        ecg.enableLeadOff(i, true, true);
    }
    
    // Configure pace detection for Lead II (CH3) and V1 (CH8)
    ecg.configurePace(ADS1298R::PACE_CH3, ADS1298R::PACE_CH8);
    ecg.enablePace(true);
    
    Serial.println("Configured for 12-lead ECG with lead-off and pace detection");
    Serial.println("Sample rate: 8kSPS (required for pace detection)");
}

SixLeadECG calculate6LeadECG(const ADS1298R::Data& data) {
    SixLeadECG leads;
    
    int32_t RA = data.channels[0];
    int32_t LA = data.channels[1];
    int32_t LL = data.channels[2];
    
    leads.L1 = LA - RA;
    leads.L2 = LL - RA;
    leads.L3 = LL - LA;
    
    leads.aVR = -(LA + LL) / 2;
    leads.aVL = LA - (RA + LL) / 2;
    leads.aVF = LL - (RA + LA) / 2;
    
    return leads;
}

TwelveLeadECG calculate12LeadECG(const ADS1298R::Data& data) {
    TwelveLeadECG leads;
    
    // Limb leads (computed in analog domain by hardware)
    leads.L1 = data.channels[1];  // CH2: LA - RA
    leads.L2 = data.channels[2];  // CH3: LL - RA
    
    // Derived limb leads (computed digitally)
    leads.L3 = leads.L2 - leads.L1;
    leads.aVR = -(leads.L1 + leads.L2) / 2;
    leads.aVL = leads.L1 - leads.L2 / 2;
    leads.aVF = leads.L2 - leads.L1 / 2;
    
    // Precordial leads (computed in analog domain: Vn - WCT)
    leads.V6 = data.channels[0];  // CH1: V6 - WCT
    leads.V2 = data.channels[3];  // CH4: V2 - WCT
    leads.V3 = data.channels[4];  // CH5: V3 - WCT
    leads.V4 = data.channels[5];  // CH6: V4 - WCT
    leads.V5 = data.channels[6];  // CH7: V5 - WCT
    leads.V1 = data.channels[7];  // CH8: V1 - WCT
    
    return leads;
}

void printLeadOffStatus(const ADS1298R::Data& data) {
    bool hasLeadOff = false;
    for(int i = 0; i < 8; i++) {
        if(data.leadOffStatus[i]) {
            hasLeadOff = true;
            break;
        }
    }
    
    if(hasLeadOff) {
        Serial.print(",LEADOFF:");
        for(int i = 0; i < 8; i++) {
            if(data.leadOffStatus[i]) {
                Serial.print("CH"); Serial.print(i+1); Serial.print(" ");
            }
        }
    }
}

void printPaceStatus(const ADS1298R::Data& data) {
    // Print pace status in your preferred format
    // Always print pace status when pace detection is enabled
    bool paceEnabled = true; // Assume pace is enabled if we're checking
    
    if(paceEnabled) {
        // Print Pace1 status
        Serial.print(",Pace1:");
        Serial.print(data.paceDetected[0] ? "1" : "0");
        
        // Print Pace2 status  
        Serial.print(",Pace2:");
        Serial.print(data.paceDetected[1] ? "1" : "0");
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("ADS1298R 12-Lead ECG Monitor with Lead-off and Pace Detection");
    
    if (!ecg.begin()) {
        Serial.print("ERROR: Initialization failed - ");
        Serial.println(ADS1298R::getErrorString(ecg.getLastError()));
        while(1);
    }
    
    Serial.print("Device ID: 0x");
    Serial.println(ecg.getDeviceID(), HEX);
    
    #if ECG_MODE == 1
        configure6LeadECG();
    #elif ECG_MODE == 2
        configure12LeadECG();
    #endif
    
    ecg.startAcquisition();
    Serial.println("Acquisition started!\n");
    
    #if ECG_MODE == 1
        Serial.println("Format: aVL,aVF,aVR,L1,L2,L3[,LEADOFF:channels][,Pace1:0/1,Pace2:0/1]");
    #elif ECG_MODE == 2
        Serial.println("Format: aVL,aVF,aVR,L1,L2,L3,V1,V2,V3,V4,V5,V6[,LEADOFF:channels][,Pace1:0/1,Pace2:0/1]");
    #endif
    Serial.println("Commands: s=stop, r=resume, t=test, l=leadoff_status, p=pace_toggle, g=gpio_status, ?=help");
}

void loop() {
    if (ecg.isDataReady()) {
        ADS1298R::Data data;
        
        if (ecg.readData(data)) {
            #if ECG_MODE == 1
                SixLeadECG leads = calculate6LeadECG(data);
                Serial.print(">");
                Serial.print("aVL:"); Serial.print(leads.aVL); Serial.print(",");
                Serial.print("aVF:"); Serial.print(leads.aVF); Serial.print(",");
                Serial.print("aVR:"); Serial.print(leads.aVR); Serial.print(",");
                Serial.print("L1:"); Serial.print(leads.L1); Serial.print(",");
                Serial.print("L2:"); Serial.print(leads.L2); Serial.print(",");
                Serial.print("L3:"); Serial.print(leads.L3);
                
                // Add lead-off and pace status
                printLeadOffStatus(data);
                printPaceStatus(data);
                Serial.println();
                
            #elif ECG_MODE == 2
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
                
                // Add lead-off and pace status
                printLeadOffStatus(data);
                printPaceStatus(data);
                Serial.println();
            #endif
        }
    }
    
    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 's':  // Stop
                ecg.stopAcquisition();
                Serial.println("Acquisition stopped");
                break;
                
            case 'r':  // Resume
                ecg.startAcquisition();
                Serial.println("Acquisition resumed");
                break;
                
            case 't':  // Test signal toggle
                {
                    static bool testEnabled = false;
                    testEnabled = !testEnabled;
                    ecg.setTestSignal(testEnabled ? ADS1298R::TEST_1MV_FAST : ADS1298R::TEST_DISABLED);
                    Serial.print("Test signal: ");
                    Serial.println(testEnabled ? "ON" : "OFF");
                }
                break;
                
            case 'l':  // Lead-off status
                Serial.println("\n=== Lead-off Status ===");
                for(int i = 0; i < 8; i++) {
                    bool posOff, negOff;
                    if(ecg.getLeadOffStatus(i, posOff, negOff)) {
                        Serial.print("CH"); Serial.print(i+1); Serial.print(": ");
                        if(!posOff && !negOff) {
                            Serial.println("OK");
                        } else {
                            Serial.print("LEAD-OFF (");
                            if(posOff) Serial.print("+ ");
                            if(negOff) Serial.print("- ");
                            Serial.println(")");
                        }
                    }
                }
                Serial.println("====================\n");
                break;
                
            case 'p':  // Pace detection toggle
                {
                    static bool paceEnabled = true;
                    paceEnabled = !paceEnabled;
                    ecg.enablePace(paceEnabled);
                    Serial.print("Pace detection: ");
                    Serial.println(paceEnabled ? "ON" : "OFF");
                }
                break;
                
            case 'g':  // GPIO status (for pace debugging)
                {
                    uint8_t gpio = ecg.readGPIO();
                    uint8_t pace = ecg.readRegister(ADS1298R::REG_PACE);
                    Serial.println("\n=== Pace Debug Info ===");
                    Serial.print("PACE Register: 0x"); Serial.println(pace, HEX);
                    Serial.print("GPIO Status: 0x"); Serial.println(gpio, HEX);
                    Serial.print("GPIO1 (Pace1): "); Serial.println((gpio & 0x01) ? "HIGH" : "LOW");
                    Serial.print("GPIO2 (Pace2): "); Serial.println((gpio & 0x02) ? "HIGH" : "LOW");
                    Serial.print("PACE Enabled: "); Serial.println((pace & 0x01) ? "NO" : "YES");
                    Serial.println("======================\n");
                }
                break;
                
            case 'i':  // Info
                Serial.println("\n=== Device Info ===");
                Serial.print("Device ID: 0x"); Serial.println(ecg.getDeviceID(), HEX);
                Serial.print("Acquiring: "); Serial.println(ecg.isAcquiring() ? "Yes" : "No");
                #if ECG_MODE == 1
                    Serial.println("Mode: 6-Lead ECG");
                #elif ECG_MODE == 2
                    Serial.println("Mode: 12-Lead ECG");
                    Serial.println("Sample Rate: 8kSPS (for pace detection)");
                #endif
                Serial.println("Lead-off: Enabled for all channels");
                Serial.println("Pace: Enabled on Lead II and V1");
                Serial.println("==================\n");
                break;
                
            case '?':  // Help
                Serial.println("\n=== Commands ===");
                Serial.println("s - Stop acquisition");
                Serial.println("r - Resume acquisition");
                Serial.println("t - Toggle test signal");
                Serial.println("l - Show lead-off status");
                Serial.println("p - Toggle pace detection");
                Serial.println("g - Show GPIO/pace debug info");
                Serial.println("i - Device info");
                Serial.println("? - Help");
                Serial.println("================\n");
                break;
                
            default:
                Serial.println("Unknown command. Type '?' for help.");
                break;
        }
    }
}