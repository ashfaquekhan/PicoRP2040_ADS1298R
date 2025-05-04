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

// SPI Configuration
#define SPI_FREQ 2 // 2MHz for better reliability

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
  
  void waitForSPI();
  void waitForDecode();
  uint8_t readRegisterInternal(uint8_t reg);
  void writeRegisterInternal(uint8_t reg, uint8_t value);
  
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
  void readRegisterBlock(uint8_t startReg, uint8_t numRegs, uint8_t* data);
  void writeRegisterBlock(uint8_t startReg, uint8_t numRegs, const uint8_t* data);
  
  // High-level Configuration
  void setDataRate(uint8_t rate);
  void setGain(uint8_t channel, uint8_t gain);
  void setChannelInput(uint8_t channel, uint8_t input);
  void enableRLD(bool enable);
  void configureRLD(uint8_t posChannels, uint8_t negChannels);
  void configureLeadOff(uint8_t posChannels, uint8_t negChannels);
  void configureWCT(uint8_t wct1, uint8_t wct2);
  void configureRespiration(uint8_t respControl);
  
  // Data Acquisition
  void startContinuous();
  void stopContinuous();
  bool isDataReady();
  void readData(ADS1298RData* data);
  
  // Diagnostics
  void printRegisters();
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
  // Configure pins
  pinMode(ADS1298_DRDY_PIN, INPUT);
  pinMode(ADS1298_CS_PIN, OUTPUT);
  pinMode(ADS1298_START_PIN, OUTPUT);
  pinMode(ADS1298_RESET_PIN, OUTPUT);
  
  digitalWrite(ADS1298_CS_PIN, HIGH);
  digitalWrite(ADS1298_START_PIN, LOW);
  digitalWrite(ADS1298_RESET_PIN, HIGH);
  
  // Initialize SPI
  if (!PicoSPI0.configure(2, 3, 4, 5, SPI_FREQ*1000000ul, 1, false)) {
    return false;
  }
  
  // Hard reset
  hardReset();
  
  // Check connection
  if (!checkConnection()) {
    return false;
  }
  
  // Attach DRDY interrupt
  attachInterrupt(digitalPinToInterrupt(ADS1298_DRDY_PIN), handleDRDY, FALLING);
  
  return true;
}

void ADS1298R::hardReset() {
  digitalWrite(ADS1298_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ADS1298_RESET_PIN, HIGH);
  delay(150); // Wait for reset to complete
  
  // Read device ID to flush SPI
  readRegister(ADS1298_REG_ID);
}

void ADS1298R::reset() {
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_RESET);
  PicoSPI0.endTransaction();
  delay(150); // Wait for reset to complete
}

void ADS1298R::start() {
  if (!continuousMode) {
    digitalWrite(ADS1298_START_PIN, HIGH);
  }
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_START);
  PicoSPI0.endTransaction();
  waitForDecode();
}

void ADS1298R::stop() {
  if (!continuousMode) {
    digitalWrite(ADS1298_START_PIN, LOW);
  }
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_STOP);
  PicoSPI0.endTransaction();
  waitForDecode();
}

void ADS1298R::waitForSPI() {
  delayMicroseconds(8); // Minimum SCLK period + command decode
}

void ADS1298R::waitForDecode() {
  delayMicroseconds(8); // 4 tCLK for command decode
}

uint8_t ADS1298R::readRegisterInternal(uint8_t reg) {
  uint8_t data;
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_RREG | reg);
  PicoSPI0.transfer(0x00); // Read one register
  data = PicoSPI0.transfer(0x00);
  PicoSPI0.endTransaction();
  
  regCache[reg] = data; // Update cache
  return data;
}

void ADS1298R::writeRegisterInternal(uint8_t reg, uint8_t value) {
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_WREG | reg);
  PicoSPI0.transfer(0x00); // Write one register
  PicoSPI0.transfer(value);
  PicoSPI0.endTransaction();
  
  regCache[reg] = value; // Update cache
  waitForDecode();
}

uint8_t ADS1298R::readRegister(uint8_t reg) {
  if (continuousMode) {
    return regCache[reg]; // Return cached value in continuous mode
  }
  return readRegisterInternal(reg);
}

void ADS1298R::writeRegister(uint8_t reg, uint8_t value) {
  if (continuousMode) {
    stopContinuous();
    writeRegisterInternal(reg, value);
    startContinuous();
  } else {
    writeRegisterInternal(reg, value);
  }
}

void ADS1298R::readRegisterBlock(uint8_t startReg, uint8_t numRegs, uint8_t* data) {
  if (continuousMode) {
    stopContinuous();
  }
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_RREG | startReg);
  PicoSPI0.transfer(numRegs - 1);
  
  for (uint8_t i = 0; i < numRegs; i++) {
    data[i] = PicoSPI0.transfer(0x00);
    regCache[startReg + i] = data[i];
  }
  
  PicoSPI0.endTransaction();
  
  if (continuousMode) {
    startContinuous();
  }
}

void ADS1298R::writeRegisterBlock(uint8_t startReg, uint8_t numRegs, const uint8_t* data) {
  if (continuousMode) {
    stopContinuous();
  }
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_WREG | startReg);
  PicoSPI0.transfer(numRegs - 1);
  
  for (uint8_t i = 0; i < numRegs; i++) {
    PicoSPI0.transfer(data[i]);
    regCache[startReg + i] = data[i];
  }
  
  PicoSPI0.endTransaction();
  
  waitForDecode();
  
  if (continuousMode) {
    startContinuous();
  }
}

void ADS1298R::setDataRate(uint8_t rate) {
  uint8_t config1 = readRegister(ADS1298_REG_CONFIG1);
  config1 = (config1 & 0xF8) | (rate & 0x07);
  writeRegister(ADS1298_REG_CONFIG1, config1);
}

void ADS1298R::setGain(uint8_t channel, uint8_t gain) {
  if (channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t val = readRegister(reg);
  val = (val & 0x8F) | ((gain & 0x07) << 4);
  writeRegister(reg, val);
}

void ADS1298R::setChannelInput(uint8_t channel, uint8_t input) {
  if (channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t val = readRegister(reg);
  val = (val & 0xF8) | (input & 0x07);
  writeRegister(reg, val);
}

void ADS1298R::enableRLD(bool enable) {
  uint8_t config3 = readRegister(ADS1298_REG_CONFIG3);
  if (enable) {
    config3 |= 0x04; // Set PD_RLD bit
  } else {
    config3 &= ~0x04; // Clear PD_RLD bit
  }
  writeRegister(ADS1298_REG_CONFIG3, config3);
}

void ADS1298R::configureRLD(uint8_t posChannels, uint8_t negChannels) {
  writeRegister(ADS1298_REG_RLD_SENSP, posChannels);
  writeRegister(ADS1298_REG_RLD_SENSN, negChannels);
}

void ADS1298R::configureLeadOff(uint8_t posChannels, uint8_t negChannels) {
  writeRegister(ADS1298_REG_LOFF_SENSP, posChannels);
  writeRegister(ADS1298_REG_LOFF_SENSN, negChannels);
}

void ADS1298R::configureWCT(uint8_t wct1, uint8_t wct2) {
  writeRegister(ADS1298_REG_WCT1, wct1);
  writeRegister(ADS1298_REG_WCT2, wct2);
}

void ADS1298R::configureRespiration(uint8_t respControl) {
  writeRegister(ADS1298_REG_RESP, respControl);
}

void ADS1298R::startContinuous() {
  continuousMode = true;
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_RDATAC);
  PicoSPI0.endTransaction();
  
  waitForDecode();
  start();
}

void ADS1298R::stopContinuous() {
  if (!continuousMode) return;
  
  continuousMode = false;
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_SDATAC);
  PicoSPI0.endTransaction();
  
  waitForDecode();
}

bool ADS1298R::isDataReady() {
  bool ready = dataReady;
  dataReady = false;
  return ready;
}

void ADS1298R::readData(ADS1298RData* data) {
  uint8_t statusByte[3];
  
  PicoSPI0.beginTransaction();
  
  // Read status bytes
  for (int i = 0; i < 3; i++) {
    statusByte[i] = PicoSPI0.transfer(0x00);
  }
  
  // Extract status information
  data->leadOffStatusP = statusByte[0];
  data->leadOffStatusN = statusByte[1];
  data->gpioData = (statusByte[2] >> 4) & 0x0F;
  
  // Read channel data
  for (int i = 0; i < 8; i++) {
    data->channelData[i] = 0;
    for (int j = 0; j < 3; j++) {
      data->channelData[i] = (data->channelData[i] << 8) | PicoSPI0.transfer(0x00);
    }
    
    // Sign extend if negative
    if (data->channelData[i] & 0x800000) {
      data->channelData[i] |= 0xFF000000;
    }
  }
  
  PicoSPI0.endTransaction();
}

void ADS1298R::handleDRDY() {
  if (instance) {
    instance->dataReady = true;
  }
}

bool ADS1298R::checkConnection() {
  uint8_t id = readRegister(ADS1298_REG_ID);
  // ADS1298R ID should be 0x92
  return (id & 0x1F) == 0x12;
}

void ADS1298R::printRegisters() {
  Serial.println("\n=== ADS1298R Register Map ===");
  
  const char* regNames[] = {
    "ID", "CONFIG1", "CONFIG2", "CONFIG3", "LOFF",
    "CH1SET", "CH2SET", "CH3SET", "CH4SET", "CH5SET", "CH6SET", "CH7SET", "CH8SET",
    "RLD_SENSP", "RLD_SENSN", "LOFF_SENSP", "LOFF_SENSN", "LOFF_FLIP", 
    "LOFF_STATP", "LOFF_STATN", "GPIO", "PACE", "RESP", "CONFIG4", "WCT1", "WCT2"
  };
  
  for (int i = 0; i <= 0x19; i++) {
    Serial.print(regNames[i]);
    Serial.print(" (0x");
    if (i < 0x10) Serial.print("0");
    Serial.print(i, HEX);
    Serial.print("): 0x");
    uint8_t val = readRegister(i);
    if (val < 0x10) Serial.print("0");
    Serial.println(val, HEX);
  }
}

// Global instance
ADS1298R ads1298r;

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("\n=== ADS1298R Full Driver Example ===");
  
  if (!ads1298r.begin()) {
    Serial.println("Failed to initialize ADS1298R!");
    while(1);
  }
  
  Serial.println("ADS1298R initialized successfully");
  
  // Configure for ECG
  ads1298r.writeRegister(ADS1298_REG_CONFIG1, 0x86); // HR mode, 500 SPS
  ads1298r.writeRegister(ADS1298_REG_CONFIG2, 0x00); // Normal operation
  ads1298r.writeRegister(ADS1298_REG_CONFIG3, 0xC0); // Internal reference, RLD enabled
  
  // Configure channels
  for (uint8_t i = 1; i <= 8; i++) {
    ads1298r.setGain(i, 6); // 6x gain
    ads1298r.setChannelInput(i, 0); // Normal electrode input
  }
  
  // Configure RLD for channels 2 and 3 (RA and LA)
  ads1298r.configureRLD(0x0C, 0x0C); // Channels 2 and 3
  
  // Print register status
  ads1298r.printRegisters();
  
  // Start continuous data acquisition
  ads1298r.startContinuous();
}

void loop() {
  if (ads1298r.isDataReady()) {
    ADS1298RData data;
    ads1298r.readData(&data);
    
    // Print channel 1 data
    Serial.print("CH1: ");
    Serial.print(data.channelData[0]);
    
    // Print GPIO status
    Serial.print(" | GPIO: 0x");
    Serial.println(data.gpioData, HEX);
    
    delay(50); // Rate limit output
  }
}