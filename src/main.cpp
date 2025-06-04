#include <Arduino.h>
#include <PicoSPI.h>

// Pin definitions
#define ADS1298_DRDY_PIN  6
#define ADS1298_CS_PIN    5
#define ADS1298_START_PIN 7
#define ADS1298_RESET_PIN 8

// ADS1298R Commands
#define ADS1298_CMD_WAKEUP   0x02
#define ADS1298_CMD_STANDBY  0x04
#define ADS1298_CMD_RESET    0x06
#define ADS1298_CMD_START    0x08
#define ADS1298_CMD_STOP     0x0A
#define ADS1298_CMD_RDATAC   0x10
#define ADS1298_CMD_SDATAC   0x11
#define ADS1298_CMD_RDATA    0x12
#define ADS1298_CMD_RREG     0x20
#define ADS1298_CMD_WREG     0x40

// ADS1298R Registers
#define ADS1298_REG_ID       0x00
#define ADS1298_REG_CONFIG1  0x01
#define ADS1298_REG_CONFIG2  0x02
#define ADS1298_REG_CONFIG3  0x03
#define ADS1298_REG_LOFF     0x04
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
#define ADS1298_REG_LOFF_SENSP 0x0F
#define ADS1298_REG_LOFF_SENSN 0x10
#define ADS1298_REG_LOFF_FLIP 0x11
#define ADS1298_REG_LOFF_STATP 0x12
#define ADS1298_REG_LOFF_STATN 0x13
#define ADS1298_REG_GPIO     0x14
#define ADS1298_REG_PACE     0x15
#define ADS1298_REG_RESP     0x16
#define ADS1298_REG_CONFIG4  0x17
#define ADS1298_REG_WCT1     0x18
#define ADS1298_REG_WCT2     0x19

// SPI frequency in MHz
#define SPI_FREQ 1 // 1MHz like in the original example

// Device Status and Data Structure
struct ADS1298RData {
  int32_t channelData[8];  // 24-bit signed data
  uint8_t leadOffStatusP;  // Lead-off status positive
  uint8_t leadOffStatusN;  // Lead-off status negative
  uint8_t gpioData;        // GPIO data
};

class ADS1298R {
private:
  volatile bool dataReady;
  volatile bool continuousMode;
  uint8_t regCache[26];  // Cache for register values
  
  void waitForSPI() { delayMicroseconds(5); }
  void waitForDecode() { delayMicroseconds(10); }
  
public:
  ADS1298R();
  
  // Initialization and Control
  bool begin();
  void reset();
  void hardReset();
  void start();
  void stop();
  void standby();
  void wakeup();
  
  // Register Access  
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  
  // High-level Configuration
  void setDataRate(uint8_t rate);
  void setGain(uint8_t channel, uint8_t gain);
  void setChannelInput(uint8_t channel, uint8_t input);
  void enableRLD(bool enable);
  void configureRLD(uint8_t posChannels, uint8_t negChannels);
  void configureAllChannels(uint8_t gain, uint8_t input);
  
  // Data Acquisition
  void startContinuous();
  void stopContinuous();
  bool isDataReady();
  void readData(ADS1298RData* data);
  
  // Diagnostics
  void printRegisters();
  void printChannelData(const ADS1298RData* data);
  bool checkConnection();
  
  // DRDY interrupt handler
  static void handleDRDY();
  
  static ADS1298R* instance;
};

ADS1298R* ADS1298R::instance = nullptr;

ADS1298R::ADS1298R() : dataReady(false), continuousMode(false) {
  instance = this;
}

bool ADS1298R::begin() {
  Serial.println("Initializing ADS1298R...");
  
  // Setup pins
  pinMode(ADS1298_DRDY_PIN, INPUT);
  pinMode(ADS1298_CS_PIN, OUTPUT);
  pinMode(ADS1298_START_PIN, OUTPUT);
  pinMode(ADS1298_RESET_PIN, OUTPUT);
  
  // Set initial pin states
  digitalWrite(ADS1298_CS_PIN, HIGH);      // Deselect chip
  digitalWrite(ADS1298_START_PIN, LOW);    // No conversion
  digitalWrite(ADS1298_RESET_PIN, HIGH);   // Not in reset
  
  // Initialize PicoSPI - Using pins GP2 (SCK), GP3 (MOSI), GP4 (MISO), GP5 (CS)
  Serial.println("Initializing PicoSPI...");
  if (!PicoSPI0.configure(2, 3, 4, 5, SPI_FREQ*1000000ul, 1, false)) {
    Serial.println("SPI configuration FAILED!");
    return false;
  }
  Serial.println("SPI configuration successful");
  
  // Hardware reset the ADS1298R
  hardReset();
  
  // Stop continuous data mode to allow register operations
  Serial.println("Sending SDATAC command");
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_SDATAC);
  PicoSPI0.endTransaction();
  delay(10);
  
  // Try to read device ID
  Serial.println("Reading Device ID Register");
  uint8_t id = readRegister(ADS1298_REG_ID);
  Serial.print("Device ID: 0x");
  if (id < 0x10) Serial.print("0");
  Serial.println(id, HEX);
  
  // Check for valid device response
  if (id == 0x00 || id == 0xFF) {
    Serial.println("No response from device - SPI communication failure");
    Serial.println("Please check your connections and try again");
    return false;
  } else {
    Serial.println("Valid device response received");
    
    // Attach DRDY interrupt
    attachInterrupt(digitalPinToInterrupt(ADS1298_DRDY_PIN), handleDRDY, FALLING);
    return true;
  }
}

void ADS1298R::hardReset() {
  Serial.println("Performing hardware reset");
  
  // Reset low
  digitalWrite(ADS1298_RESET_PIN, LOW);
  delay(10);
  
  // Reset high
  digitalWrite(ADS1298_RESET_PIN, HIGH);
  delay(100);
  
  Serial.println("Reset complete");
}

uint8_t ADS1298R::readRegister(uint8_t reg) {
  uint8_t data;
  uint8_t opcode1 = reg + 0x20; // RREG command
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(opcode1);
  PicoSPI0.transfer(0x00); // Read one register
  data = PicoSPI0.transfer(0x00); // Read register data
  PicoSPI0.endTransaction();
  
  return data;
}

void ADS1298R::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t opcode1 = reg + 0x40; // WREG command
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(opcode1);
  PicoSPI0.transfer(0x00); // Write one register
  PicoSPI0.transfer(value); // Write register data
  PicoSPI0.endTransaction();
  
  waitForDecode();
  
  // Verify the write by reading back
  uint8_t readback = readRegister(reg);
  Serial.print("Write to register 0x");
  if (reg < 0x10) Serial.print("0");
  Serial.print(reg, HEX);
  Serial.print(": 0x");
  if (value < 0x10) Serial.print("0");
  Serial.print(value, HEX);
  Serial.print(", Readback: 0x");
  if (readback < 0x10) Serial.print("0");
  Serial.print(readback, HEX);
  
  if (readback == value) {
    Serial.println(" - SUCCESS");
  } else {
    Serial.println(" - FAILED");
  }
}

void ADS1298R::configureAllChannels(uint8_t gain, uint8_t input) {
  for (int i = 1; i <= 8; i++) {
    uint8_t reg = ADS1298_REG_CH1SET + (i - 1);
    // Gain is in bits [6:4], Input selection is in bits [2:0]
    uint8_t value = (gain << 4) | (input & 0x07);
    writeRegister(reg, value);
  }
}

void ADS1298R::handleDRDY() {
  if (instance) {
    instance->dataReady = true;
  }
}

bool ADS1298R::isDataReady() {
  bool ready = dataReady;
  dataReady = false;
  return ready;
}

void ADS1298R::readData(ADS1298RData* data) {
  uint8_t inByte;
  int32_t stat = 0;
  
  PicoSPI0.beginTransaction();
  
  // Read status word (3 bytes)
  for (int i = 0; i < 3; i++) {
    inByte = PicoSPI0.transfer(0x00);
    stat = (stat << 8) | inByte;
  }
  
  // Read channel data (8 channels, 3 bytes each = 24 bytes)
  for (int i = 0; i < 8; i++) {
    data->channelData[i] = 0; // Reset channel data
    for (int j = 0; j < 3; j++) {
      inByte = PicoSPI0.transfer(0x00);
      data->channelData[i] = (data->channelData[i] << 8) | inByte;
    }
    
    // Sign extension for negative numbers
    if (data->channelData[i] & 0x800000) {
      data->channelData[i] |= 0xFF000000;
    }
  }
  
  PicoSPI0.endTransaction();
}

void ADS1298R::startContinuous() {
  // Start data collection
  Serial.println("Starting continuous data mode");
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_RDATAC);
  PicoSPI0.endTransaction();
  delay(10);
  
  digitalWrite(ADS1298_START_PIN, HIGH);
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_START);
  PicoSPI0.endTransaction();
  
  continuousMode = true;
}

void ADS1298R::printChannelData(const ADS1298RData* data) {

    Serial.print(data->channelData[0]);
    Serial.print(",");
    Serial.print(data->channelData[1]);
    Serial.print(",");
    Serial.print(data->channelData[2]);
    Serial.print(",");
    Serial.print(data->channelData[3]);
    Serial.print(",");
    Serial.print(data->channelData[4]);
    Serial.print(",");
    Serial.print(data->channelData[5]);
    Serial.print(",");
    Serial.print(data->channelData[6]);
    Serial.print(",");
    Serial.print(data->channelData[7]);
    // Serial.print(" | Lead-off P: ");
    // Serial.print(data->leadOffStatusP, BIN);
    // Serial.print(" | Lead-off N: ");
    // Serial.print(data->leadOffStatusN, BIN);
    // Serial.print(" | GPIO: ");
    // Serial.print(data->gpioData, BIN);
    // Serial.print(" | Status: ");
    // Serial.print((data->channelData[0] & 0x800000) ? "Negative" : "Positive");
    // Serial.print(" | DRDY: ");
    // Serial.print(digitalRead(ADS1298_DRDY_PIN) == HIGH ? "High" : "Low"); 
    // Serial.print(" | Continuous: ");
    // Serial.print(continuousMode ? "Yes" : "No");
    // Serial.print(" | SPI Frequency: ");
    // Serial.print(SPI_FREQ);
    // Serial.print(" MHz | ");
    // Serial.print("Data Ready: ");
    // Serial.print(isDataReady() ? "Yes" : "No");
    // Serial.print(" | ");
    // Serial.print("Time: ");
    // Serial.print(millis());
    // Serial.print(" ms");
    // Serial.print(" | ");
    // Serial.print("Timestamp: ");
    // Serial.print(millis() / 1000);
    // Serial.print(" seconds");
    // Serial.print(" | ");
     Serial.println();
}

// Global instance
ADS1298R ads1298r;

void setup() {
  Serial.begin(115200);
  delay(5000); // Give time for serial monitor to open
  
  Serial.println("\n\n========================================");
  Serial.println("ADS1298R All 8 Channels @ 500SPS");
  Serial.println("========================================");
  
  if (!ads1298r.begin()) {
    Serial.println("Failed to initialize ADS1298R!");
    while(true) {}
  }
  
  Serial.println("ADS1298R initialized successfully");
  
  // Configure registers based on requirements
  Serial.println("Configuring registers");
  
  // CONFIG1: Set HR mode, DR = 500SPS (110)
  // HR=1 (bit 7), DR=110 (bits 2:0)
  ads1298r.writeRegister(ADS1298_REG_CONFIG1, 0x86);
  
  // CONFIG2: Not using test signals
  ads1298r.writeRegister(ADS1298_REG_CONFIG2, 0x00);
  
  // CONFIG3: Enable internal reference buffer (bit 7=1, bit 6=1), Enable RLD (bit 3=1)
  ads1298r.writeRegister(ADS1298_REG_CONFIG3, 0xC8);
  
  // Configure all channels for ECG measurement
  // Gain = 6 (000), Input = Normal electrode (000)
  // For each channel: bits[6:4] = gain, bits[2:0] = input
  ads1298r.configureAllChannels(0, 0); // gain=6 (000), input=normal electrode (000)
  
  // Optional: Configure RLD (right leg drive) for channels 2 and 3
  // This helps reduce common-mode noise
  ads1298r.writeRegister(ADS1298_REG_RLD_SENSP, 0x0C); // Channels 2 and 3 positive
  ads1298r.writeRegister(ADS1298_REG_RLD_SENSN, 0x0C); // Channels 2 and 3 negative
  
  Serial.println("\nRegister Configuration:");
  Serial.println("======================");
  Serial.println("CONFIG1: High-resolution mode, 500 SPS");
  Serial.println("CONFIG2: Normal operation (no test signals)");
  Serial.println("CONFIG3: Internal reference enabled, RLD enabled");
  Serial.println("All channels: Gain=6, Normal electrode input");
  Serial.println("RLD: Channels 2 and 3 for common-mode reduction");
  Serial.println();
  
  // Start continuous data mode
  ads1298r.startContinuous();
  
  Serial.println("Data acquisition started. Channel data format: CH1: value | CH2: value | ...");
  Serial.println("===========================================================================");
}

void loop() {
  if (ads1298r.isDataReady()) {
    ADS1298RData data;
    ads1298r.readData(&data);
    
    // Print all channel data in a single line
    ads1298r.printChannelData(&data);
    
    // delay(10); // Small delay to avoid flooding output
  }
}