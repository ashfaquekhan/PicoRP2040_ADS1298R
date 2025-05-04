/*
 * Hardware connections:
 * - Pico GP2 -> ADS1298R SCLK
 * - Pico GP3 -> ADS1298R MOSI (DIN)
 * - Pico GP4 -> ADS1298R MISO (DOUT)  
 * - Pico GP5 -> ADS1298R CS
 * - Pico GP6 -> ADS1298R DRDY 
 * - Pico GP7 -> ADS1298R START
 * - Pico GP8 -> ADS1298R RESET
 */

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
 
 // ADS1298R Register Addresses
 #define ADS1298_REG_ID       0x00
 #define ADS1298_REG_CONFIG1  0x01
 #define ADS1298_REG_CONFIG2  0x02
 #define ADS1298_REG_CONFIG3  0x03
 #define ADS1298_REG_LOFF     0x04
 #define ADS1298_REG_CH1SET   0x05
 
 // Global variables
 volatile boolean dataReady = false;
 long channelData[8]; // To store the channel readings
 int stat = 0;        // Status register value
 
 // SPI frequency in MHz
 #define SPI_FREQ 1 // 1MHz
 
 // Function declarations
 void drdy_handler();
 void hardReset();
 void sendCommand(byte cmd);
 byte readRegister(byte reg);
 void writeRegister(byte reg, byte value);
 void updateChannelData();
 void printHex(byte data);
 
 void setup() {
   // Initialize serial port for debugging
   Serial.begin(115200);
   delay(3000); // Give time for serial monitor to open
   
   Serial.println("\n\n========================================");
   Serial.println("ADS1298R Simplified Test - Pico RP2040 W");
   Serial.println("========================================");
   
   // Setup pins
   pinMode(ADS1298_DRDY_PIN, INPUT);
   pinMode(ADS1298_CS_PIN, OUTPUT);
   pinMode(ADS1298_START_PIN, OUTPUT);
   pinMode(ADS1298_RESET_PIN, OUTPUT);
   
   // Set initial pin states
   digitalWrite(ADS1298_CS_PIN, HIGH);      // Deselect chip
   digitalWrite(ADS1298_START_PIN, LOW);    // No conversion
   
   
   // Initialize PicoSPI - Using pins GP2 (SCK), GP3 (MOSI), GP4 (MISO), GP5 (CS)
   Serial.println("Initializing PicoSPI...");
   if (!PicoSPI0.configure(2, 3, 4, 5, SPI_FREQ*1000000ul, 1, false)) {
     Serial.println("SPI configuration FAILED!");
     while(true) {}
   }
   Serial.println("SPI configuration successful");
   
   // Hardware reset the ADS1298R
   hardReset();
   
   // Stop continuous data mode to allow register operations
   Serial.println("Sending SDATAC command");
   sendCommand(ADS1298_CMD_SDATAC);
   delay(10);
   
   // Try to read device ID
   Serial.println("Reading Device ID Register");
   byte id = readRegister(ADS1298_REG_ID);
   Serial.print("Device ID: 0x");
   printHex(id);
   Serial.println();
   
   // If we get a valid ID, continue with configuration
   if (id != 0x00) {
     Serial.println("Valid device response received");
     
     // Configure registers based on requirements
     Serial.println("Configuring registers");
     
     // CONFIG1: Set HR mode, DR = 500SPS (110)
     writeRegister(ADS1298_REG_CONFIG1, 0x86);
     
     // CONFIG2: Not using test signals
     writeRegister(ADS1298_REG_CONFIG2, 0x00);
     
     // CONFIG3: Enable internal reference buffer (bit 7=1, bit 6=1)
     writeRegister(ADS1298_REG_CONFIG3, 0xC0);
     
     // Set first channel to input short for testing
     writeRegister(ADS1298_REG_CH1SET, 0x01);
     
     // Start data collection
     Serial.println("Starting continuous data mode");
     sendCommand(ADS1298_CMD_RDATAC);
     delay(10);
     
     digitalWrite(ADS1298_START_PIN, HIGH);
     sendCommand(ADS1298_CMD_START);
     
     // Attach DRDY interrupt
     attachInterrupt(digitalPinToInterrupt(ADS1298_DRDY_PIN), drdy_handler, FALLING);
   } else {
     Serial.println("No response from device - SPI communication failure");
     Serial.println("Please check your connections and try again");
   }
 }
 
 void loop() {
   if (dataReady) {
     dataReady = false;
     updateChannelData();
     
     // Print just the first channel
     Serial.print("CH1: ");
     Serial.println(channelData[0]);
     
     delay(100); // Small delay to avoid flooding output
   }
 }
 
 // DRDY interrupt handler
 void drdy_handler() {
   dataReady = true;
 }
 
 // Hardware reset the ADS1298R
 void hardReset() {
   Serial.println("Performing hardware reset");
   
   // Reset low
   digitalWrite(ADS1298_RESET_PIN, LOW);
   delay(10);
   
   // Reset high
   digitalWrite(ADS1298_RESET_PIN, HIGH);
   delay(100);
   
   Serial.println("Reset complete");
 }
 
 // Send a command to the ADS1298R
 void sendCommand(byte cmd) {
   PicoSPI0.beginTransaction();
   PicoSPI0.transfer(cmd);
   PicoSPI0.endTransaction();
   delayMicroseconds(5); // Brief delay after commands
 }
 
 // Read a register from the ADS1298R
 byte readRegister(byte reg) {
   byte data;
   byte opcode1 = reg + 0x20; // RREG command
   
   PicoSPI0.beginTransaction();
   PicoSPI0.transfer(opcode1);
   PicoSPI0.transfer(0x00); // Read one register
   data = PicoSPI0.transfer(0x00); // Read register data
   PicoSPI0.endTransaction();
   
   return data;
 }
 
 // Write to a register on the ADS1298R
 void writeRegister(byte reg, byte value) {
   byte opcode1 = reg + 0x40; // WREG command
   
   PicoSPI0.beginTransaction();
   PicoSPI0.transfer(opcode1);
   PicoSPI0.transfer(0x00); // Write one register
   PicoSPI0.transfer(value); // Write register data
   PicoSPI0.endTransaction();
   
   // Verify the write by reading back
   byte readback = readRegister(reg);
   Serial.print("Write to register 0x");
   printHex(reg);
   Serial.print(": 0x");
   printHex(value);
   Serial.print(", Readback: 0x");
   printHex(readback);
   
   if (readback == value) {
     Serial.println(" - SUCCESS");
   } else {
     Serial.println(" - FAILED");
   }
 }
 
 // Read channel data in continuous mode
 void updateChannelData() {
   byte inByte;
   
   PicoSPI0.beginTransaction();
   
   // Read status word (3 bytes)
   stat = 0;
   for (int i = 0; i < 3; i++) {
     inByte = PicoSPI0.transfer(0x00);
     stat = (stat << 8) | inByte;
   }
   
   // Read channel data (8 channels, 3 bytes each = 24 bytes)
   for (int i = 0; i < 8; i++) {
     channelData[i] = 0; // Reset channel data
     for (int j = 0; j < 3; j++) {
       inByte = PicoSPI0.transfer(0x00);
       channelData[i] = (channelData[i] << 8) | inByte;
     }
     
     // Sign extension for negative numbers
     if (channelData[i] & 0x800000) {
       channelData[i] |= 0xFF000000;
     }
   }
   
   PicoSPI0.endTransaction();
 }
 
 // Helper function to print byte as hex with leading zero
 void printHex(byte data) {
   if (data < 0x10) Serial.print("0");
   Serial.print(data, HEX);
 }