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
    
    ecg.setChannel(0, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(1, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    ecg.setChannel(2, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    
    for(int i = 3; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(false));
    }
    
    ecg.setRLD(0x07, 0x07);
    
    Serial.println("✓ Configured for 6-lead ECG");
}

void configure12LeadECG() {
    ecg.setPowerMode(ADS1298R::HIGH_RESOLUTION, ADS1298R::RATE_1K_500);
    ecg.setReference(ADS1298R::INTERNAL_2_4V);
    delay(150);
    
    ecg.setNotchFilter(true, true);
    
    if (!ecg.configureWCTOptimized(true)) {
        Serial.print("ERROR: WCT configuration failed - ");
        Serial.println(ADS1298R::getErrorString(ecg.getLastError()));
    }
    
    for(int i = 0; i < 8; i++) {
        ecg.setChannel(i, ADS1298R::ChannelConfig(true, ADS1298R::GAIN_6, ADS1298R::NORMAL_ELECTRODE));
    }
    
    ecg.setRLD(0x06, 0x06);
    
    ecg.setLeadOff(ADS1298R::LEADOFF_6NA, ADS1298R::LEADOFF_DC, 0x60);
    
    Serial.println("✓ Configured for 12-lead ECG with V6 optimizations");
    Serial.println("⚠ CRITICAL: Verify JP16 jumper is installed on EVM!");
}

void verifyConfiguration() {
    Serial.println("\n=== Configuration Verification ===");
    
    uint8_t deviceId = ecg.readRegister(ADS1298R::REG_ID);
    uint8_t config1 = ecg.readRegister(ADS1298R::REG_CONFIG1);
    uint8_t config2 = ecg.readRegister(ADS1298R::REG_CONFIG2);
    uint8_t config4 = ecg.readRegister(ADS1298R::REG_CONFIG4);
    uint8_t wct1 = ecg.readRegister(ADS1298R::REG_WCT1);
    uint8_t wct2 = ecg.readRegister(ADS1298R::REG_WCT2);
    
    Serial.print("Device ID: 0x"); Serial.println(deviceId, HEX);
    Serial.print("CONFIG1: 0x"); Serial.println(config1, HEX);
    Serial.print("CONFIG2: 0x"); Serial.print(config2, HEX);
    Serial.print(" (WCT_CHOP="); Serial.print((config2 & 0x80) ? "ON" : "OFF");
    Serial.print(", Notch="); Serial.print(config2 & 0x03, BIN); Serial.println(")");
    Serial.print("CONFIG4: 0x"); Serial.print(config4, HEX);
    Serial.print(" (WCT_TO_RLD="); Serial.print((config4 & 0x04) ? "ON" : "OFF"); Serial.println(")");
    
    #if ECG_MODE == 2
        Serial.print("WCT1: 0x"); Serial.println(wct1, HEX);
        Serial.print("WCT2: 0x"); Serial.println(wct2, HEX);
        
        bool wctOK = (wct1 == 0x0B) && (wct2 == 0xD4) && (config4 & 0x04);
        Serial.print("WCT Configuration: "); Serial.println(wctOK ? "✓ OK" : "✗ FAIL");
        
        if (!wctOK) {
            Serial.println("Expected: WCT1=0x0B, WCT2=0xD4, CONFIG4 bit 2=1");
        }
    #endif
    
    Serial.println("=== End Verification ===\n");
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
    
    leads.L1 = data.channels[1];
    leads.L2 = data.channels[2];
    leads.L3 = leads.L2 - leads.L1;
    leads.aVR = -(leads.L1 + leads.L2) / 2;
    leads.aVL = leads.L1 - leads.L2 / 2;
    leads.aVF = leads.L2 - leads.L1 / 2;
    
    leads.V6 = data.channels[0];
    leads.V2 = data.channels[3];
    leads.V3 = data.channels[4];
    leads.V4 = data.channels[5];
    leads.V5 = data.channels[6];
    leads.V1 = data.channels[7];
    
    return leads;
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("ADS1298R ECG Monitor - Fixed Version");
    
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
    
    verifyConfiguration();
    
    ecg.startAcquisition();
    Serial.println("✓ Acquisition started!\n");
    
    #if ECG_MODE == 1
        Serial.println("Format: aVL,aVF,aVR,L1,L2,L3");
    #elif ECG_MODE == 2
        Serial.println("Format: aVL,aVF,aVR,L1,L2,L3,V1,V2,V3,V4,V5,V6");
        Serial.println("Commands: s=stop, r=resume, i=info, t=test, v=verify, x=V6diag, n=notch");
    #endif
}

void loop() {
    if (ecg.isDataReady()) {
        ADS1298R::Data data;
        
        #if ECG_MODE == 2
            if (ecg.readDataWithQualityCheck(data)) {
        #else
            if (ecg.readData(data)) {
        #endif
            
            #if ECG_MODE == 1
                SixLeadECG leads = calculate6LeadECG(data);
                Serial.print(">");
                Serial.print(leads.aVL); Serial.print(",");
                Serial.print(leads.aVF); Serial.print(",");
                Serial.print(leads.aVR); Serial.print(",");
                Serial.print(leads.L1); Serial.print(",");
                Serial.print(leads.L2); Serial.print(",");
                Serial.print(leads.L3);
                Serial.println();
                
            #elif ECG_MODE == 2
                TwelveLeadECG leads = calculate12LeadECG(data);
                
                String qualFlag = "";
                if (data.leadOffStatus[0]) qualFlag += "V6_OFF ";
                if (abs(data.channels[0]) < 100) qualFlag += "V6_LOW ";
                
                Serial.print(">");
                Serial.print("aVL:");Serial.print(leads.aVL); Serial.print(",");
                Serial.print("aVF:");Serial.print(leads.aVF); Serial.print(",");
                Serial.print("aVR:");Serial.print(leads.aVR); Serial.print(",");
                Serial.print("L1:");Serial.print(leads.L1); Serial.print(",");
                Serial.print("L2:");Serial.print(leads.L2); Serial.print(",");
                Serial.print("L3:");Serial.print(leads.L3); Serial.print(",");
                Serial.print("V1:");Serial.print(leads.V1); Serial.print(",");
                Serial.print("V2:");Serial.print(leads.V2); Serial.print(",");
                Serial.print("V3:");Serial.print(leads.V3); Serial.print(",");
                Serial.print("V4:");Serial.print(leads.V4); Serial.print(",");
                Serial.print("V5:");Serial.print(leads.V5); Serial.print(",");
                Serial.print("V6:");Serial.print(leads.V6);
                Serial.println();
            #endif
        }
    }
    
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 's':
                ecg.stopAcquisition();
                Serial.println("\n✓ Acquisition stopped");
                break;
                
            case 'r':
                ecg.startAcquisition();
                Serial.println("\n✓ Acquisition resumed");
                break;
                
            case 'i':
                Serial.println("\n=== Device Info ===");
                Serial.print("Device ID: 0x"); Serial.println(ecg.getDeviceID(), HEX);
                Serial.print("Acquiring: "); Serial.println(ecg.isAcquiring() ? "Yes" : "No");
                #if ECG_MODE == 1
                    Serial.println("Mode: 6-Lead ECG");
                #elif ECG_MODE == 2
                    Serial.println("Mode: 12-Lead ECG");
                #endif
                break;
                
            case 't':
                {
                    static bool testEnabled = false;
                    testEnabled = !testEnabled;
                    ecg.setTestSignal(testEnabled ? ADS1298R::TEST_1MV_FAST : ADS1298R::TEST_DISABLED);
                    Serial.print("\n✓ Test signal: "); Serial.println(testEnabled ? "ON" : "OFF");
                }
                break;
                
            case 'v':
                verifyConfiguration();
                break;
                
            case 'x':
                #if ECG_MODE == 2
                    ecg.diagnoseV6Channel();
                #endif
                break;
                
            case 'n':
                {
                    static bool notchEnabled = true;
                    notchEnabled = !notchEnabled;
                    ecg.setNotchFilter(notchEnabled, notchEnabled);
                    Serial.print("\n✓ Notch filters: "); Serial.println(notchEnabled ? "ON" : "OFF");
                }
                break;
                
            case 'd':
                Serial.println("\n=== Raw Channel Data ===");
                if (ecg.isDataReady()) {
                    ADS1298R::Data debugData;
                    if (ecg.readData(debugData)) {
                        for(int i = 0; i < 8; i++) {
                            Serial.print("CH"); Serial.print(i+1); 
                            Serial.print(": "); Serial.print(debugData.channels[i]);
                            if (debugData.leadOffStatus[i]) Serial.print(" [LEADOFF]");
                            Serial.println();
                        }
                    }
                }
                break;
                
            case '?':
                Serial.println("\n=== Commands ===");
                Serial.println("s - Stop acquisition");
                Serial.println("r - Resume acquisition");
                Serial.println("i - Device info");
                Serial.println("t - Toggle test signal");
                Serial.println("v - Verify configuration");
                Serial.println("d - Debug raw data");
                #if ECG_MODE == 2
                    Serial.println("x - V6 channel diagnostics");
                    Serial.println("n - Toggle notch filters");
                #endif
                Serial.println("? - Help");
                break;
        }
    }
}