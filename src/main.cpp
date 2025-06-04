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
#define SPI_FREQ 1 // 1MHz

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
  
  // Individual Channel Configuration
  void setChannelGain(uint8_t channel, uint8_t gain);
  void setChannelInput(uint8_t channel, uint8_t input);
  void setChannelPowerDown(uint8_t channel, bool powerDown);
  void configureChannel(uint8_t channel, uint8_t gain, uint8_t input, bool powerDown = false);
  void powerDownChannel(uint8_t channel);
  void powerUpChannel(uint8_t channel, uint8_t gain, uint8_t input);
  void printChannelConfig(uint8_t channel);
  void printAllChannelConfigs();
  
  // High-level Configuration
  void setDataRate(uint8_t rate);
  void enableRLD(bool enable);
  void configureRLD(uint8_t posChannels, uint8_t negChannels);
  void configureWCT(uint8_t wctA, uint8_t wctB, uint8_t wctC);
  
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

// Individual channel configuration functions
void ADS1298R::setChannelGain(uint8_t channel, uint8_t gain) {
  if (channel < 1 || channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t currentValue = readRegister(reg);
  
  // Clear gain bits [6:4] and set new gain
  uint8_t newValue = (currentValue & 0x8F) | ((gain & 0x07) << 4);
  writeRegister(reg, newValue);
}

void ADS1298R::setChannelInput(uint8_t channel, uint8_t input) {
  if (channel < 1 || channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t currentValue = readRegister(reg);
  
  // Clear input bits [2:0] and set new input
  uint8_t newValue = (currentValue & 0xF8) | (input & 0x07);
  writeRegister(reg, newValue);
}

void ADS1298R::setChannelPowerDown(uint8_t channel, bool powerDown) {
  if (channel < 1 || channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t currentValue = readRegister(reg);
  
  // Set or clear power down bit [7]
  uint8_t newValue = powerDown ? (currentValue | 0x80) : (currentValue & 0x7F);
  writeRegister(reg, newValue);
}

void ADS1298R::configureChannel(uint8_t channel, uint8_t gain, uint8_t input, bool powerDown) {
  if (channel < 1 || channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t value = (powerDown ? 0x80 : 0x00) | ((gain & 0x07) << 4) | (input & 0x07);
  writeRegister(reg, value);
  
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" configured - Gain: ");
  
  // Print actual gain value
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
  
  Serial.print(", Input: ");
  
  // Print input type
  switch(input) {
    case 0: Serial.print("Normal"); break;
    case 1: Serial.print("Shorted"); break;
    case 2: Serial.print("RLD_MEAS"); break;
    case 3: Serial.print("MVDD"); break;
    case 4: Serial.print("Temp"); break;
    case 5: Serial.print("Test"); break;
    case 6: Serial.print("RLD_DRP"); break;
    case 7: Serial.print("RLD_DRN"); break;
    default: Serial.print("?"); break;
  }
  
  Serial.print(", Power: ");
  Serial.println(powerDown ? "DOWN" : "UP");
}

void ADS1298R::powerDownChannel(uint8_t channel) {
  setChannelPowerDown(channel, true);
  // Also set input to shorted to minimize noise
  setChannelInput(channel, 1); // Input shorted
}

void ADS1298R::powerUpChannel(uint8_t channel, uint8_t gain, uint8_t input) {
  configureChannel(channel, gain, input, false);
}

void ADS1298R::printChannelConfig(uint8_t channel) {
  if (channel < 1 || channel > 8) return;
  
  uint8_t reg = ADS1298_REG_CH1SET + (channel - 1);
  uint8_t value = readRegister(reg);
  
  bool powerDown = (value & 0x80) != 0;
  uint8_t gain = (value >> 4) & 0x07;
  uint8_t input = value & 0x07;
  
  Serial.print("CH");
  Serial.print(channel);
  Serial.print(": Power=");
  Serial.print(powerDown ? "DOWN" : "UP");
  Serial.print(", Gain=");
  
  // Print actual gain value
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
  
  Serial.print(", Input=");
  
  // Print input type
  switch(input) {
    case 0: Serial.print("Normal"); break;
    case 1: Serial.print("Shorted"); break;
    case 2: Serial.print("RLD_MEAS"); break;
    case 3: Serial.print("MVDD"); break;
    case 4: Serial.print("Temp"); break;
    case 5: Serial.print("Test"); break;
    case 6: Serial.print("RLD_DRP"); break;
    case 7: Serial.print("RLD_DRN"); break;
    default: Serial.print("?"); break;
  }
  
  Serial.println();
}

void ADS1298R::printAllChannelConfigs() {
  Serial.println("Channel Configurations:");
  Serial.println("======================");
  for (int i = 1; i <= 8; i++) {
    printChannelConfig(i);
  }
  Serial.println();
}

void ADS1298R::configureRLD(uint8_t posChannels, uint8_t negChannels) {
  writeRegister(ADS1298_REG_RLD_SENSP, posChannels);
  writeRegister(ADS1298_REG_RLD_SENSN, negChannels);
  
  Serial.print("RLD configured - Positive channels: 0x");
  if (posChannels < 0x10) Serial.print("0");
  Serial.print(posChannels, HEX);
  Serial.print(", Negative channels: 0x");
  if (negChannels < 0x10) Serial.print("0");
  Serial.println(negChannels, HEX);
}

void ADS1298R::configureWCT(uint8_t wctA, uint8_t wctB, uint8_t wctC) {
  // WCT1 register: Enable WCTA and set input selection
  uint8_t wct1_val = 0x08 | (wctA & 0x07); // Enable WCTA (bit 3) + input selection
  writeRegister(ADS1298_REG_WCT1, wct1_val);
  
  // WCT2 register: Enable WCTB and WCTC with input selections
  uint8_t wct2_val = 0xC0 | ((wctB & 0x07) << 3) | (wctC & 0x07);
  writeRegister(ADS1298_REG_WCT2, wct2_val);
  
  Serial.println("WCT configured for Wilson Central Terminal");
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
  
  // Extract status information
  data->leadOffStatusP = (stat >> 16) & 0xFF;
  data->leadOffStatusN = (stat >> 8) & 0xFF;
  data->gpioData = stat & 0x0F;
  
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

void ADS1298R::stopContinuous() {
  // Stop data collection
  Serial.println("Stopping continuous data mode");
  
  digitalWrite(ADS1298_START_PIN, LOW);
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_STOP);
  PicoSPI0.endTransaction();
  delay(10);
  
  PicoSPI0.beginTransaction();
  PicoSPI0.transfer(ADS1298_CMD_SDATAC);
  PicoSPI0.endTransaction();
  delay(10);
  
  continuousMode = false;
}

void ADS1298R::printChannelData(const ADS1298RData* data) {
  // Print only active channels (1-3 in our setup)
  Serial.print(data->channelData[0]); // CH1 - RA
  Serial.print(",");
  Serial.print(data->channelData[1]); // CH2 - LA
  Serial.print(",");
  Serial.print(data->channelData[2]); // CH3 - LL
  Serial.println();
}

// Global instance
ADS1298R ads1298r;

void setup() {
  Serial.begin(115200);
  delay(5000); // Give time for serial monitor to open
  
  Serial.println("\n\n========================================");
  Serial.println("ADS1298R Individual Channel Control");
  Serial.println("RA, LA, LL ECG Configuration");
  Serial.println("========================================");
  
  if (!ads1298r.begin()) {
    Serial.println("Failed to initialize ADS1298R!");
    while(true) {}
  }
  
  Serial.println("ADS1298R initialized successfully");
  
  // Configure registers
  Serial.println("Configuring registers");
  
  // CONFIG1: Set HR mode, DR = 500SPS (110)
  // HR=1 (bit 7), DR=110 (bits 2:0)
  ads1298r.writeRegister(ADS1298_REG_CONFIG1, 0x86);
  
  // CONFIG2: Normal operation (no test signals)
  ads1298r.writeRegister(ADS1298_REG_CONFIG2, 0x00);
  
  // CONFIG3: Enable internal reference buffer (bit 7=1), Set VREF=2.4V (bit 5=0), Enable RLD (bit 3=1), Internal RLDREF (bit 2=1)
  ads1298r.writeRegister(ADS1298_REG_CONFIG3, 0xCC);
  
  // Configure individual channels
  Serial.println("\nConfiguring individual channels:");
  Serial.println("================================");
  
  // Configure channels for RA, LA, LL ECG measurement
  ads1298r.configureChannel(1, 0, 0, false); // CH1: RA - Gain=6, Normal electrode, Power UP
  ads1298r.configureChannel(2, 0, 0, false); // CH2: LA - Gain=6, Normal electrode, Power UP  
  ads1298r.configureChannel(3, 0, 0, false); // CH3: LL - Gain=6, Normal electrode, Power UP
  
  // Power down unused channels to save power and reduce noise
  ads1298r.powerDownChannel(4); // CH4: Power DOWN
  ads1298r.powerDownChannel(5); // CH5: Power DOWN
  ads1298r.powerDownChannel(6); // CH6: Power DOWN
  ads1298r.powerDownChannel(7); // CH7: Power DOWN
  ads1298r.powerDownChannel(8); // CH8: Power DOWN
  
  // Configure RLD for channels 1, 2, 3 (RA, LA, LL)
  // This helps reduce common-mode noise
  ads1298r.configureRLD(0x07, 0x07); // Channels 1, 2, 3 for both positive and negative
  
  // Configure Wilson Central Terminal using RA, LA, LL
  // WCTA = CH1P (RA), WCTB = CH2P (LA), WCTC = CH3P (LL)
  ads1298r.configureWCT(0, 2, 4); // CH1P, CH2P, CH3P
  
  // Print current configuration
  ads1298r.printAllChannelConfigs();
  
  Serial.println("\nRegister Configuration Summary:");
  Serial.println("==============================");
  Serial.println("CONFIG1: High-resolution mode, 500 SPS");
  Serial.println("CONFIG2: Normal operation (no test signals)");
  Serial.println("CONFIG3: Internal reference enabled, RLD enabled");
  Serial.println("CH1-3: Gain=6, Normal electrode input (RA, LA, LL)");
  Serial.println("CH4-8: Powered down");
  Serial.println("RLD: All three channels for common-mode reduction");
  Serial.println("WCT: Wilson Central Terminal configured");
  Serial.println();
  
  // Start continuous data mode
  ads1298r.startContinuous();
  
  Serial.println("Data acquisition started for RA, LA, LL");
  Serial.println("Data format: RA, LA, LL");
  Serial.println("========================================");
}

void loop() {
  if (ads1298r.isDataReady()) {
    ADS1298RData data;
    ads1298r.readData(&data);
    
    // Print channel data for RA, LA, LL
    ads1298r.printChannelData(&data);
  }
  
  // Optional: Add commands to change configuration during runtime
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "config") {
      ads1298r.printAllChannelConfigs();
    }
    else if (command == "stop") {
      ads1298r.stopContinuous();
      Serial.println("Data acquisition stopped. Type 'start' to resume.");
    }
    else if (command == "start") {
      ads1298r.startContinuous();
      Serial.println("Data acquisition started.");
    }
    else if (command.startsWith("gain")) {
      // Example: "gain 1 5" sets channel 1 to gain 8
      int channel = command.substring(5, 6).toInt();
      int gain = command.substring(7).toInt();
      if (channel >= 1 && channel <= 8) {
        ads1298r.setChannelGain(channel, gain);
        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(" gain changed to ");
        Serial.println(gain);
      }
    }
    else if (command == "help") {
      Serial.println("\nAvailable commands:");
      Serial.println("config - Show channel configurations");
      Serial.println("stop - Stop data acquisition");
      Serial.println("start - Start data acquisition");
      Serial.println("gain [ch] [gain] - Set channel gain (e.g., 'gain 1 5')");
      Serial.println("help - Show this help");
      Serial.println();
    }
  }
}