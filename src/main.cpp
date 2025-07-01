#include <Arduino.h>
#include <PicoSPI.h>

// Pin definitions
#define ADS1298_DRDY_PIN  6
#define ADS1298_CS_PIN    5
#define ADS1298_START_PIN 7
#define ADS1298_RESET_PIN 8

// ADS1298R Commands
#define ADS1298_CMD_SDATAC   0x11
#define ADS1298_CMD_RDATAC   0x10
#define ADS1298_CMD_START    0x08
#define ADS1298_CMD_STOP     0x0A
#define ADS1298_CMD_RREG     0x20
#define ADS1298_CMD_WREG     0x40

// ADS1298R Registers
#define ADS1298_REG_CONFIG1  0x01
#define ADS1298_REG_CONFIG2  0x02
#define ADS1298_REG_CONFIG3  0x03
#define ADS1298_REG_CH1SET   0x05
#define ADS1298_REG_CH2SET   0x06
#define ADS1298_REG_CH3SET   0x07
#define ADS1298_REG_CH4SET   0x08
#define ADS1298_REG_CH5SET   0x09
#define ADS1298_REG_CH6SET   0x0A
#define ADS1298_REG_CH7SET   0x0B
#define ADS1298_REG_CH8SET   0x0C
#define ADS1298_REG_RLD_SENSP 0x0D
#define ADS1298_REG_RLD_SENSN 0x0E

#define SPI_FREQ 1

// Six ECG leads calculated from two hardware measurements
struct SixLeadECG {
  int32_t lead_I;   // Hardware: Channel 2 = LA - RA
  int32_t lead_II;  // Hardware: Channel 3 = LL - RA  
  int32_t lead_III; // Calculated: Lead II - Lead I
  int32_t aVR;      // Calculated: -(Lead I + Lead II)/2
  int32_t aVL;      // Calculated: Lead I - Lead II/2
  int32_t aVF;      // Calculated: Lead II - Lead I/2
};

class ECGSystem {
private:
  volatile bool dataReady;
  static ECGSystem* instance; // Static instance for interrupt access
  
public:
  ECGSystem() : dataReady(false) {
    instance = this; // Set the static instance pointer
  }
  
  bool begin() {
    Serial.println("Initializing ADS1298R...");
    
    // Setup pins
    pinMode(ADS1298_DRDY_PIN, INPUT);
    pinMode(ADS1298_CS_PIN, OUTPUT);
    pinMode(ADS1298_START_PIN, OUTPUT);
    pinMode(ADS1298_RESET_PIN, OUTPUT);
    
    // Set initial pin states
    digitalWrite(ADS1298_CS_PIN, HIGH);
    digitalWrite(ADS1298_START_PIN, LOW);
    digitalWrite(ADS1298_RESET_PIN, HIGH);
    
    // Initialize SPI
    if (!PicoSPI0.configure(2, 3, 4, 5, SPI_FREQ*1000000ul, 1, false)) {
      Serial.println("SPI initialization failed!");
      return false;
    }
    
    // Hardware reset sequence
    digitalWrite(ADS1298_RESET_PIN, LOW);
    delay(10);
    digitalWrite(ADS1298_RESET_PIN, HIGH);
    delay(100);
    
    // Stop continuous mode for configuration
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(ADS1298_CMD_SDATAC);
    PicoSPI0.endTransaction();
    delay(10);
    
    // Configure the device for ECG measurement
    configureForECG();
    
    // Setup interrupt using static member function
    attachInterrupt(digitalPinToInterrupt(ADS1298_DRDY_PIN), handleInterrupt, FALLING);
    
    Serial.println("ADS1298R initialized successfully!");
    return true;
  }
  
  void configureForECG() {
    Serial.println("Configuring for 6-lead ECG system...");
    
    // CONFIG1: High-resolution mode, 500 SPS
    // HR=1 (bit 7), DR=110 (500 SPS)
    writeRegister(ADS1298_REG_CONFIG1, 0x86);
    
    // CONFIG2: Normal operation, no test signals
    writeRegister(ADS1298_REG_CONFIG2, 0x00);
    
    // CONFIG3: Internal reference enabled, RLD enabled
    // PD_REFBUF=1, RLD_MEAS=0, RLDREF_INT=1, PD_RLD=1
    writeRegister(ADS1298_REG_CONFIG3, 0xCC);
    
    // Channel configurations
    writeRegister(ADS1298_REG_CH1SET, 0x81); // CH1: Power down (not used)
    writeRegister(ADS1298_REG_CH2SET, 0x00); // CH2: Normal electrode, Gain=6 (for Lead I)
    writeRegister(ADS1298_REG_CH3SET, 0x00); // CH3: Normal electrode, Gain=6 (for Lead II)
    writeRegister(ADS1298_REG_CH4SET, 0x81); // CH4-8: Power down (not used)
    writeRegister(ADS1298_REG_CH5SET, 0x81);
    writeRegister(ADS1298_REG_CH6SET, 0x81);
    writeRegister(ADS1298_REG_CH7SET, 0x81);
    writeRegister(ADS1298_REG_CH8SET, 0x81);
    
    // Configure RLD (Right Leg Drive) for channels 2 and 3
    // This enables common-mode noise rejection
    writeRegister(ADS1298_REG_RLD_SENSP, 0x06); // Enable RLD for CH2 and CH3 positive
    writeRegister(ADS1298_REG_RLD_SENSN, 0x06); // Enable RLD for CH2 and CH3 negative
    
    Serial.println("Configuration complete!");
    Serial.println("CH2 configured for Lead I = LA - RA");
    Serial.println("CH3 configured for Lead II = LL - RA");
  }
  
  void writeRegister(uint8_t reg, uint8_t value) {
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(reg + 0x40); // WREG command
    PicoSPI0.transfer(0x00);       // Write one register
    PicoSPI0.transfer(value);      // Register data
    PicoSPI0.endTransaction();
    delayMicroseconds(10); // Allow time for register write to complete
  }
  
  void startAcquisition() {
    Serial.println("Starting data acquisition...");
    
    // Start continuous data read mode
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(ADS1298_CMD_RDATAC);
    PicoSPI0.endTransaction();
    delay(10);
    
    // Start conversions
    digitalWrite(ADS1298_START_PIN, HIGH);
    
    PicoSPI0.beginTransaction();
    PicoSPI0.transfer(ADS1298_CMD_START);
    PicoSPI0.endTransaction();
    
    Serial.println("Data acquisition started!");
    Serial.println("Output format: Lead_I,Lead_II,Lead_III,aVR,aVL,aVF");
  }
  
  bool isDataReady() {
    bool ready = dataReady;
    dataReady = false; // Clear the flag
    return ready;
  }
  
  void readAndCalculateLeads(SixLeadECG* leads) {
    int32_t channelData[8];
    
    PicoSPI0.beginTransaction();
    
    // Skip status bytes (3 bytes) - we don't need them for basic ECG
    for (int i = 0; i < 3; i++) {
      PicoSPI0.transfer(0x00);
    }
    
    // Read all 8 channels (3 bytes each for 24-bit data)
    for (int i = 0; i < 8; i++) {
      channelData[i] = 0;
      for (int j = 0; j < 3; j++) {
        uint8_t byte = PicoSPI0.transfer(0x00);
        channelData[i] = (channelData[i] << 8) | byte;
      }
      
      // Convert 24-bit signed to 32-bit signed (sign extension)
      if (channelData[i] & 0x800000) {
        channelData[i] |= 0xFF000000;
      }
    }
    
    PicoSPI0.endTransaction();
    
    // Extract the two hardware-measured leads
    // Based on Table 2 from ADS1298RECG-FE documentation:
    leads->lead_I = channelData[1];   // Channel 2: Lead I = LA - RA
    leads->lead_II = channelData[2];  // Channel 3: Lead II = LL - RA
    
    // Calculate the four derived leads using Table 3 formulas:
    leads->lead_III = leads->lead_II - leads->lead_I;                // LL - RA - LA = LEAD II - LEAD I
    leads->aVR = -(leads->lead_I + leads->lead_II) / 2;             // RA - (LA + LL)/2 = -(LEAD I + LEAD II)/2
    leads->aVL = leads->lead_I - leads->lead_II / 2;                // LA - (RA + LL)/2 = LEAD I - LEAD II/2
    leads->aVF = leads->lead_II - leads->lead_I / 2;                // LL - (RA + LA)/2 = LEAD II - LEAD I/2
  }
  
  void printCSV(const SixLeadECG* leads) {
    // Simple comma-separated output for your plotting application
    Serial.print(leads->lead_I);  Serial.print(",");
    Serial.print(leads->lead_II); Serial.print(",");
    Serial.print(leads->lead_III);Serial.print(",");
    Serial.print(leads->aVR);     Serial.print(",");
    Serial.print(leads->aVL);     Serial.print(",");
    Serial.print(leads->aVF);
    Serial.println();
  }
  
  // Static interrupt handler that can access the instance
  static void handleInterrupt() {
    if (instance) {
      instance->dataReady = true;
    }
  }
};

// Define the static member
ECGSystem* ECGSystem::instance = nullptr;

// Global instance
ECGSystem ecg;

void setup() {
  Serial.begin(115200);
  delay(3000); 
  
  // Initialize the ECG system
  if (!ecg.begin()) {
    Serial.println("ERROR: Failed to initialize ADS1298R!");
    while(true); // Stop here if initialization fails
  }
  
  Serial.println();
  
  // Start data acquisition
  ecg.startAcquisition();
  delay(3000);
}

void loop() {
  // Check if new ECG data is available
  if (ecg.isDataReady()) {
    SixLeadECG leads;
    
    // Read hardware data and calculate all 6 leads
    ecg.readAndCalculateLeads(&leads);
    
    // Output as comma-separated values for your plotting application
    ecg.printCSV(&leads);
  }
  
  // Optional: Handle any serial commands for debugging
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "info") {
      Serial.println("System running normally");
      Serial.println("Output: Lead_I,Lead_II,Lead_III,aVR,aVL,aVF");
    }
  }
}