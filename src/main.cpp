// Cryoegg Datalogger 2023 code
// Runs on Adafruit Feather M0 Adalogger board, with Datalogger2023 motherboard
// Developed using Arduino IDE 2.0.4

// IMPORTANT NOTE - need to comment out SERCOM5-related lines in variant.cpp for Feather M0 Board Support Package - this seems to be an issue with Adafruit's BSP for the Feather M0

// Libraries to include. If the library is not an Arduino standard one, include a source link.
// For all libraries, add the version number
#include <Adafruit_SharpMem.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GenericDevice.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h> 
#include <Adafruit_SPIDevice.h>

#include <SparkFun_BMA400_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

#include <MS5607.h> // v1.1 source: https://github.com/UravuLabs/MS5607 - forked copy at https://github.com/mrpj100/MS5607 just in case the original disappears
#include <RTCZero.h> // v1.6 obtained from Arduino

#include <IridiumSBD.h> // v3.0.6 obtained from Arduino, documentation https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
#include <SPI.h> // built-in
#include <SdFat.h> // v1.2.4 obtained from Arduino
// #include <SD.h>
#include <Wire.h> // built-in 
#include <time.h>

// include Arduino private library to enable access to pin reconfiguration
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // gives access to pinPeripheral() function, needed to configure SERCOMs later

#include "WMBusProcessor.h" // class file for processing Cryoegg packets

//////////////////////////////////////////////////////////////////////////////
// Logger properties
//////////////////////////////////////////////////////////////////////////////
#define DIAGNOSTICS true // change this to see diagnostic messages from the main logger code
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Physical board definitions
//////////////////////////////////////////////////////////////////////////////
#define I2C_ADDR_ACCEL      20
#define I2C_ADDR_TEMPHUMID  45
#define I2C_ADDR_GPS        42
#define I2C_ADDR_BATTMON    64
#define I2C_ADDR_POWER      113
//////////////////////////////////////////////////////////////////////////////#
// These are masks used to enable/disable powr to various sensors
#define PWR_RADIO_CH1 0b00000001
#define PWR_RADIO_CH2 0b00000010
#define PWR_SENSORS   0b00000100
#define PWR_GPS       0b00001000
// define registers for TPS22993 power control chip
#define REG_TPS22993_CONTROL 0x05
//////////////////////////////////////////////////////////////////////////////
#define DISPLAY_SWITCH_PIN A3
#define SHARP_SCK  24
#define SHARP_MOSI 23
#define SHARP_SS   18
//////////////////////////////////////////////////////////////////////////////
// Display constants
#define BLACK 0
#define WHITE 1
#define DISPLAY_SLOW 0
#define DISPLAY_FAST 1
#define DISPLAY_WIDTH 400
#define DISPLAY_HEIGHT 240
//////////////////////////////////////////////////////////////////////////////
// Display images
//////////////////////////////////////////////////////////////////////////////
const unsigned char epd_bitmap_ir [] PROGMEM = {
	0xff, 0xff, 0xfc, 0x3f, 0xf3, 0xcf, 0xef, 0xf7, 0xdc, 0x3f, 0xdb, 0xdf, 0xb7, 0xff, 0xb6, 0xff, 
	0xb7, 0xff, 0xb7, 0x43, 0xdb, 0x49, 0xdf, 0x41, 0xff, 0x47, 0xff, 0x4b, 0xff, 0x4d, 0xff, 0xff
};
const unsigned char epd_bitmap_gps [] PROGMEM = {
	0xff, 0xff, 0xfc, 0x7f, 0xf2, 0x9f, 0xee, 0xef, 0xdc, 0x77, 0xda, 0xb7, 0xb6, 0xdb, 0x80, 0x03, 
	0xb6, 0xdf, 0xda, 0xb3, 0xdc, 0x6d, 0xee, 0xef, 0xf2, 0xa9, 0xfc, 0x6d, 0xff, 0xf3, 0xff, 0xff
};
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Iridium modem parameters
//////////////////////////////////////////////////////////////////////////////
#define IridiumSerial Serial3
// modem sleep pin configuration to switch it into sleep mode
// pin D13
#define IRIDIUM_SLEEP_PIN 13
//////////////////////////////////////////////////////////////////////////////
// Iridium configuration settings
#define GET_IRIDIUM_TIME false // if this is true and USE_IRIDIUM is false, we get the time from Iridium but don't try and send any messages
#define USE_IRIDIUM false // if true, we get the time from Iridium and also try and send Iridium packets; if false we don't bother. If false we rely on getting the compilation time to set the RTC.
#define IRIDIUM_DIAGNOSTICS false // change this to see diagnostic messages from the Iridium library
#define FAKE_IRIDIUM false // set to false to actually send SBD messages
#define IRIDIUM_CONNECT_ATTEMPTS 1 // How many attempts to connect to Iridium and get the timestamp before resetting
#define IRIDIUM_HOUSEKEEPING_INTERVAL 60 * 5 // 60 * 60 * 10 // Create an interval (in ms) after which the IRIDIUM_FAIL flag will be reset
#define SBD_SEND_DELAY 3600 // delay period before sending an SBD message - value in seconds
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// EGG / WURST whitelist
//////////////////////////////////////////////////////////////////////////////
// The `egg_` variables below are to store received eggs and define a 
// whitelist of acceptable SBD egg IDs to transmit, so that we avoid
// transmitting any corrupt data.
//////////////////////////////////////////////////////////////////////////////
const uint32_t egg_ids[] = {
  // Tom's Crevasse eggs
  0xCE240001, 0xCE240002,
  // Test egg
  0xCE240005, 0xCE220002, 0xCE230004, 0xCE240005, 0xCE240006, 0xCE240007, 0xCE240008, 0xCE240009, 0xCE25AAA0, 0xCE25009B,
  // 30 bar eggs 
  0xCE230001, 0xCE240011, 0xCE240012, 0xCE240013, 0xCE240014, 0xCE240015,
  // 100 bar eggs
  0xCE240016, 0xCE240017, 0xCE240018, 0xCE240019, 0xCE240020
};
const uint8_t egg_count = 23;
const int MAX_EGG_PACKET_LENGTH = 50;
byte* egg_packets[egg_count];
uint8_t egg_packet_lengths[egg_count];
uint32_t egg_packet_rx_times[egg_count];
uint8_t egg_sbd_tosend_flag[egg_count];
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Constant variable defintiions
//////////////////////////////////////////////////////////////////////////////
const int SDchipSelect = 4; // SD chip select on Digital pin 4
const int solarVoltagePin = A0; // potential divider measuring input voltage from solar/battery supply
const float solarVoltageCalibrationFactor = 4.9; // this is the conversion due to the potential divider on the input (390k + 100k / 100k) - can be adjusted to calibrate
const int update_interval_millis = 30000; // interval at which to update logger parameters, in milliseconds
const int update_display_fast = 1000; // fast interval in millis to update display
const int update_display_slow = update_interval_millis; // slow interval in millis to update display
//////////////////////////////////////////////////////////////////////////////
typedef struct {
  uint16_t temperature;
  uint16_t humidity;
} sht30_data;

//////////////////////////////////////////////////////////////////////////////
// Object declariations
//////////////////////////////////////////////////////////////////////////////
// Define hardware SPI
SPIClass *hSPI;
// Serial1 is already defined by default and is wired to Radio Modem 1
Uart Serial2(&sercom5, A5, 6, SERCOM_RX_PAD_2, UART_TX_PAD_0); // note that to transmit on Serial2 (to radio modem 2)
Uart Serial3(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2); // comms to satellite modem via RS232 level shifter
MS5607 P_Sens; // barometric pressure sensor
RTCZero rtc; // internal real-time-clock on Cortex-M0 proccessor
IridiumSBD modem(IridiumSerial, IRIDIUM_SLEEP_PIN);
//initialise packet decoders
WMBusProcessor WMBus_Ch1(1,DIAGNOSTICS);
WMBusProcessor WMBus_Ch2(2,DIAGNOSTICS);
// Sharp Memory in Pixel display
// - need to pass pointer to the same SPI hardware as the SD card in order to avoid
//   the Adafruit library overwriting the configuration for the SPI bus
Adafruit_SharpMem display(&SPI, SHARP_SS, DISPLAY_WIDTH, DISPLAY_HEIGHT, 12000000);
BMA400 accelerometer;
sht30_data sht30_results;
SFE_UBLOX_GNSS gnss;
SdFat SD;
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////////
void refreshDisplay();
void updateDisplayMode();

void powerSwitchSetup();
void powerSwitchEnable(uint8_t channel);
void powerSwitchDisable(uint8_t channel);
void set_clock_time_compiler();

uint8_t sht30Read();

uint8_t accelRead();

//////////////////////////////////////////////////////////////////////////////
// Global variable defintions
//////////////////////////////////////////////////////////////////////////////
volatile uint8_t displayMode = DISPLAY_SLOW;
uint8_t displayToggle = 0;
char timeBuf[32];
unsigned long last_parameter_update = 0; // time at which last update was made
unsigned long last_display_refresh = 0;
float logger_baro_pressure = 0;
float logger_baro_temperature = 0;
int solarVoltageRaw = 0; // raw ADC value of solar voltage
float solar_voltage = 0;
bool logger_ready = false; 
uint8_t iridium_attempts = 0;
bool iridium_ok = false;
// storage for SBD messages
uint8_t SBD_message[340]; // maximum size of an SBD message is 340 bytes
int message_offset = 0;
// uint32_t last_packet_rx_time = 0; // keep track of the last time we received a packet - we wait for a short delay in case any other packets come in (e.g. on the other channel)
uint32_t last_iridium_time = 0;
uint32_t last_iridium_clock_update = 0;
uint16_t packets_sent_since_garbage_ch1 = 0; // number of packets that have been thrown away
uint16_t packets_sent_since_garbage_ch2 = 0; // number of packets that have been thrown away 
bool gnss_ok = false;
bool bma400_ok = false;
uint8_t displayPacket[MAX_EGG_PACKET_LENGTH];
//////////////////////////////////////////////////////////////////////////////

// interrupt handlers for serial ports
void SERCOM1_Handler()
{
  Serial3.IrqHandler();
}

void SERCOM5_Handler()
{
  Serial2.IrqHandler();
}

void updateDisplayMode() {
  // We swap to fast mode if we switch to GND 
  displayMode = digitalRead(DISPLAY_SWITCH_PIN) == 0 ? DISPLAY_FAST : DISPLAY_SLOW;
  last_display_refresh = 0;
}

void drawDisplay() {
  last_display_refresh = millis();
  // display.clearDisplay();
  
  // Toggle display refresh indicator
  displayToggle = !displayToggle;
  // Serial.println("#Refreshing display");
  // Clear screen without flicker
  display.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, WHITE);

  /////////////////////////////////////////////////////////////////////////////////
  // Draw banner
  /////////////////////////////////////////////////////////////////////////////////
  display.fillRect(0, 0, DISPLAY_WIDTH, 18, BLACK);
  display.fillRect(DISPLAY_WIDTH - 17, 1, 16, 16, displayToggle ? BLACK : WHITE);
  // Icons
  if (iridium_ok)
    display.drawBitmap(DISPLAY_WIDTH - 33, 1, (uint8_t*)epd_bitmap_ir, 16, 16, WHITE);
  if (gnss_ok)
    display.drawBitmap(DISPLAY_WIDTH - 49, 1, (uint8_t*)epd_bitmap_gps, 16, 16, WHITE);

  // Draw timestamp  
  display.setTextColor(WHITE, BLACK);
  display.setCursor(1,1);
  display.setTextSize(2);
  sprintf(timeBuf, "%d-%02d-%02d %02d:%02d:%02d",
        rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  display.print(timeBuf);
  /////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////
  // Plot logger sensors
  /////////////////////////////////////////////////////////////////////////////////
  display.setCursor(1,24);
  display.setTextSize(2);
  display.setTextColor(BLACK, WHITE);
  display.println("LOGGER SENSORS:");
  display.print("TEMP ");
  display.setTextColor(WHITE, BLACK);
  display.print(logger_baro_temperature, 1); 
  display.setTextSize(1);
  display.print("C");
  display.setTextSize(2);
  display.setTextColor(BLACK, WHITE);
  display.print(" PRESSURE ");
  display.setTextColor(WHITE, BLACK);
  display.print(logger_baro_pressure, 1);
  display.setTextSize(1); 
  display.print("mBar");

  /////////////////////////////////////////////////////////////////////////////////
  // Make a sensor orientation gizmo
  /////////////////////////////////////////////////////////////////////////////////
  const int gizmo_width = 24;
  const int gizmo_size = 8;
  const int gizmo_radius = gizmo_width - gizmo_size;
  float gizmo_x = accelerometer.data.accelX > 1 ? 1 : (accelerometer.data.accelX < -1 ? -1 : accelerometer.data.accelX);
  float gizmo_y = accelerometer.data.accelY > 1 ? 1 : (accelerometer.data.accelY < -1 ? -1 : accelerometer.data.accelY);

  display.drawCircle(DISPLAY_WIDTH - gizmo_width, 24 + gizmo_width, gizmo_width, BLACK);
  display.drawCircle(DISPLAY_WIDTH - gizmo_width, 24 + gizmo_width, 16, BLACK);
  display.drawCircle(DISPLAY_WIDTH - gizmo_width, 24 + gizmo_width, 8, BLACK);
  display.drawLine(DISPLAY_WIDTH - gizmo_width, 24, DISPLAY_WIDTH - gizmo_width, 24 + 2 * gizmo_width, BLACK);
  display.drawLine(DISPLAY_WIDTH - 2 * gizmo_width, 24 + gizmo_width, DISPLAY_WIDTH, 24 + gizmo_width, BLACK);
  if (accelerometer.data.accelZ >= 0)
    display.fillCircle(DISPLAY_WIDTH - gizmo_width + gizmo_width * gizmo_x, 24 + gizmo_width - gizmo_radius * gizmo_y, gizmo_size, BLACK);
  else 
    display.drawCircle(DISPLAY_WIDTH - gizmo_width + gizmo_width * gizmo_x, 24 + gizmo_width - gizmo_radius * gizmo_y, gizmo_size, BLACK);
  /////////////////////////////////////////////////////////////////////////////////
  
  /////////////////////////////////////////////////////////////////////////////////
  // Draw instrument table
  /////////////////////////////////////////////////////////////////////////////////
  // identify packet to display by finding the most recent packet which hasn't been
  // sent by SBD
  uint32_t egg_rx_max = 0;
  uint16_t egg_idx = 0;
  for (uint16_t dIdx = 0; dIdx < egg_count; dIdx++) {
    if (egg_sbd_tosend_flag[dIdx] && egg_packet_rx_times[dIdx] > egg_rx_max) {
      egg_rx_max = egg_packet_rx_times[dIdx];
      egg_idx = dIdx;
    }
  }
  // although it might be better to do this when we received the packets and check
  // the IDs?

  if (egg_rx_max) {
    
    // Clear display packet
    memset(displayPacket, 0, MAX_EGG_PACKET_LENGTH);
    // Copy packet
    memcpy(displayPacket, egg_packets[egg_idx], egg_packet_lengths[egg_idx]);

    time_t tmp = egg_packet_rx_times[egg_idx];
    struct tm time_struct;
    gmtime_r(&tmp, &time_struct);

    display.setCursor(1,64);
    display.setTextSize(2);
    display.setTextColor(BLACK, WHITE);
    display.println("LATEST PACKET:");
    display.setTextColor(WHITE, BLACK);
    display.print("TIME:");
    display.setTextColor(BLACK, WHITE);
    sprintf(timeBuf, " %d-%02d-%02d %02d:%02d:%02d",
        time_struct.tm_year + 1900, 
        time_struct.tm_mon, 
        time_struct.tm_mday, 
        time_struct.tm_hour, 
        time_struct.tm_min,
        time_struct.tm_sec);
    display.println(timeBuf);

    display.setTextColor(WHITE, BLACK);
    display.print("ID  :");
    display.setTextColor(BLACK, WHITE);
    display.print(" ");
    uint32_t packet_id = *(uint32_t*)(displayPacket + 18 + 3);
    // uint32_t packet_id = * (uint32_t*) (packet+18+3);
    sprintf(timeBuf, "%08X", packet_id);
    display.println(timeBuf);

    uint16_t ec, pressure, temperature, battery;
    ec = *(uint16_t*)(displayPacket + 28);
    pressure = *(uint16_t*)(displayPacket + 32);
    temperature = *(uint16_t*)(displayPacket + 34);
    battery = *(uint16_t*)(displayPacket + 36);

    display.setTextColor(WHITE, BLACK);
    display.print("BATT:");
    display.setTextColor(BLACK, WHITE);
    display.print(" ");
    display.print(battery);
    display.setTextSize(1);
    display.print("V");
    display.setTextSize(2);
    display.println();

    display.setTextColor(WHITE, BLACK);
    display.print("EC  :");
    display.setTextColor(BLACK, WHITE);
    display.print(" ");
    display.print(ec);
    display.setTextSize(1);
    display.print("raw");
    display.setTextSize(2);
    display.println();

    display.setTextColor(WHITE, BLACK);
    display.print("PRES:");
    display.setTextColor(BLACK, WHITE);
    display.print(" ");
    display.print((float)(pressure - 16384)/327.68);
    display.setTextSize(1);
    display.print("% FS");
    display.setTextSize(2);
    display.print(" | ");
    display.print(pressure);
    display.println(" RAW");

    display.setTextColor(WHITE, BLACK);
    display.print("TEMP:");
    display.setTextColor(BLACK, WHITE);
    display.print(" ");
    display.print(((temperature >> 4) - 24) * 0.05 - 50);
    display.setTextSize(1);
    display.print("C");
    display.setTextSize(2);
    display.print(" | ");
    display.print(temperature);
    display.println(" RAW");



  }
  
  display.refresh();
}

void refreshDisplay() {
  // Select interval for refresh 
  uint32_t interval = (
    displayMode == DISPLAY_FAST ? update_display_fast : update_display_slow
  );  

  if (millis() - last_display_refresh > interval) {
    drawDisplay();
  }
}

void powerSwitchSetup() {
  uint8_t error;
  // Write config
  // delay(100);
  Serial.println("#Initialised I2C");
  Wire.beginTransmission(I2C_ADDR_POWER);
  Serial.println("#Writing data");
  Wire.write(REG_TPS22993_CONTROL);
  /*       MSB 76543210 LSB 
               +------------ GPIO/I2C CH4 (GPS) 
               |+---------- GPIO/I2C CH3 (Sensors)
               ||+--------- GPIO/I2C CH2 (Radio 2)
               |||+-------- GPIO/I2C CH1 (Radio 1)
               ||||+------- EN CH4 (GPS)
               |||||+------ EN CH3 (Sensors)
               ||||||+----- EN CH2 (Radio 2)
               |||||||+---- EN CH1 (Radio 1) 
               ||||||||                       */
  Wire.write(0b11111111);
  Serial.println("#Written data");
  error = Wire.endTransmission();
  Serial.print("#Ended transmission with ");
  Serial.println(error);

  // Try reading INA3221
  Serial.println("#Trying INA3221");
  Wire.beginTransmission(I2C_ADDR_BATTMON);
  Wire.write(0x00);
  error = Wire.endTransmission();
  Serial.print("#Ended INA with: ");
  Wire.requestFrom(I2C_ADDR_BATTMON, 2);
  Serial.print("#I2C config:");
  Serial.print(Wire.read(), HEX);
  Serial.println(Wire.read(), HEX);
  
}

void powerSwitchEnable(uint8_t channel) {
  // channel should be one of PWR_RADIO_CH1, PWR_RADIO_CH2, PWR_SENSORS, PWR_GPS
  // to enable multiple channels, provide the channel argument as a bitwise OR 
  // of the channels to enable
  uint8_t reg;
  uint8_t status;
  // read register from I2C bus
  Wire.beginTransmission(I2C_ADDR_POWER);
  Wire.write(REG_TPS22993_CONTROL);
  status = Wire.endTransmission();
  Serial.print("#PWR_EndTx:");
  Serial.println(status);
  Wire.requestFrom(I2C_ADDR_POWER, 1); // Read config register
  Serial.print("#PWR_AVAILABLE:");
  Serial.println(Wire.available());
  reg = Wire.read();
  Serial.print("#PWR_REG     :");
  Serial.println(reg, BIN);
  reg = 0xF0 | reg | channel;
  Serial.print("#PWR_REG(MOD):");
  Serial.println(reg, BIN);
  Wire.beginTransmission(I2C_ADDR_POWER);
  Wire.write(REG_TPS22993_CONTROL);
  Wire.write(reg);
  Wire.endTransmission();
  // Ideally add a check here to ensurew ehaave switched
}

void powerSwitchDisable(uint8_t channel) {
  // channel should be one of PWR_RADIO_CH1, PWR_RADIO_CH2, PWR_SENSORS, PWR_GPS
  // to enable multiple channels, provide the channel argument as a bitwise OR 
  // of the channels to enable
  uint8_t reg;
  // read register from I2C bus
  Wire.beginTransmission(I2C_ADDR_POWER);
  Wire.write(REG_TPS22993_CONTROL);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR_POWER, 1); // Read config register
  reg = Wire.read();
  Serial.print("#PWR_REG     :");
  Serial.println(reg, BIN);
  reg = 0xF0 | (reg & (~channel | 0xF0));
  Serial.print("#PWR_REG(MOD):");
  Serial.println(reg, BIN);
  Wire.beginTransmission(I2C_ADDR_POWER);
  Wire.write(REG_TPS22993_CONTROL);
  Wire.write(reg);
  Wire.endTransmission();
  // Ideally add a check here to ensurew ehaave switched
}

uint8_t sht30Read() {
  uint8_t tmp;
  Wire.beginTransmission(I2C_ADDR_TEMPHUMID);
  // Write with medium repeatability and clock stretching disabled
  Wire.write(0x24);
  Wire.write(0x16);
  tmp = Wire.endTransmission();
  Serial.print("#Received resp from SHT30: ");
  Serial.println(tmp);
   delay(1000); 
  // Request 
  Wire.requestFrom(I2C_ADDR_TEMPHUMID, 6);
  Serial.print("#Retrieved from SHT30 ");
  Serial.println(Wire.available());
  if (Wire.available() != 6)
    return 0;
  sht30_results.temperature = (Wire.read() << 8) | Wire.read();
  Wire.read(); // throwaway CRC for now
  sht30_results.humidity    = (Wire.read() << 8) | Wire.read();
  Wire.read(); // throwaway CRC for now
  return 1;
}

uint8_t accelRead() {
  return 0;
}

/* 
  __setup_iridium_init() configures the pins and Serial connection for the 
  Iridium modem and sets up the power profile.
*/
int __setup_iridium_init() {
  
  pinMode(IRIDIUM_SLEEP_PIN, OUTPUT);
    
  IridiumSerial.begin(19200);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // we're powering from 12V so we should have plenty of current to charge the supercap

  // Begin satellite modem operation

  Serial.println(F("#Starting Iridium modem..."));
  return modem.begin();

}

/*
  setup_iridum() tries for IRIDIUM_CONNECT_ATTEMPTS number of attempts
  to connect to the Iridium modem

  If it fails, iridium_ok will be set to false
  If it succeeds, iridium_ok will be set to true

  On failure, it will return the error code from the iridium modem
  On success, it will return 0.
*/
uint16_t setup_iridium() {

  int err = 0; // Iridium error code
  iridium_attempts = 0;
  
  while (iridium_attempts < IRIDIUM_CONNECT_ATTEMPTS) {

    // call iridium setup routine and get return code
    err = __setup_iridium_init();
    Serial.println("#Returned from __setup_iridium");

    if (err != ISBD_SUCCESS && err != ISBD_ALREADY_AWAKE)
    {

      iridium_attempts++;
      Serial.print(F("#Begin failed: error "));
      Serial.println(err);
      if (err == ISBD_NO_MODEM_DETECTED)
        Serial.println(F("#No Iridium modem detected: check wiring."));
    
      // Return to start of next loop
      continue;

    } else {
      iridium_ok = true;
      break;
    }

  }

  return err;

}

/*
  iridium_update_time() tries for IRIDIUM_CONNECT_ATTEMPTS number of tries to
  get the time from the Iridium moed.

  On failure, time_OK will be set to false and an error 
  On success, time_OK will be set to true, the RTC clock updated and ISBD_SUCCESS returned.
*/
uint16_t iridium_update_time() {

  if (GET_IRIDIUM_TIME && iridium_ok) {
   // get the time off the Iridium modem and update the RTC with it
   bool time_OK = false;
   struct tm t;
   iridium_attempts = 0;

   if (!FAKE_IRIDIUM) {
   while (iridium_attempts++ < IRIDIUM_CONNECT_ATTEMPTS) {

    Serial.println(F("#Getting time from Iridium"));  
    int err = modem.getSystemTime(t);
    if (err == ISBD_SUCCESS)
    {
        char buf[32];
        sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d",
          t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);

        Serial.print(F("#Iridium time/date is "));
        Serial.println(buf);
        time_OK = true;
        rtc.setEpoch(mktime(&t));
        last_iridium_clock_update = rtc.getEpoch();
        return ISBD_SUCCESS;
    
    } else if (err == ISBD_NO_NETWORK) { // Did it fail because the transceiver has not yet seen the network?
    
        Serial.println(F("#No network detected.  Waiting 10 seconds."));
        delay(10 * 1000UL);
    
    } else {

        Serial.print(F("#Unexpected Iridium error "));
        Serial.println(err);
        iridium_ok = false;
        return err;
        //return; - removed as this makes it break out of the loop if the Iridium fails, which means we go into logger operational mode
    }

    // End of while loop
    }

    Serial.print("#RTC initialised - set to: ");
    Serial.print(rtc.getDay());
    Serial.print("/");
    Serial.print(rtc.getMonth());
    Serial.print("/");
    Serial.print(rtc.getYear());
    Serial.print("\t");

    Serial.print(rtc.getHours());
    Serial.print(":");
    Serial.print(rtc.getMinutes());
    Serial.print(":");
    Serial.println(rtc.getSeconds());
    return ISBD_SUCCESS;
  
  } // end of fake iridium capture
  // No Iridium enabled, do nothing
  } else {
    // either we're not getting the iridium time or the satellite modem isn't okay
    return 0; // No iridium available 
  }

}

void setup() {
  
  uint16_t err = 0;
  // open Serial-over-USB for console output
  Serial.begin(19200);
  delay(2000); // wait 2 seconds for the USB-to-serial to come back up after a board reset so we actually see some debug output from setup()

  //initialise real-time clock
  rtc.begin();
  set_clock_time_compiler(); // get the an initial estimate of the time from the compiler timestamp - this is useful for development
  
  Wire.begin();

  if ((USE_IRIDIUM || GET_IRIDIUM_TIME) && !FAKE_IRIDIUM) {

    err = setup_iridium();
    if (err != ISBD_SUCCESS) {
      iridium_ok = false;
      // Print error message
      if (err == ISBD_NO_MODEM_DETECTED)
        Serial.println(F("#No Iridium modem detected: check wiring."));
      else
        Serial.println(F("#Iridium error"));
    
    } else {
      iridium_ok = true;
    }
    // set up the Iridium modem
    
  }

  // setup time here
  err = iridium_update_time();
  if (err != ISBD_SUCCESS) {
    Serial.println("#Error getting Iridium time, defaulting to compiler value!");
  }
  
  // put modem into sleep mode to save power
  if (!FAKE_IRIDIUM) {
  Serial.println("#Putting modem to sleep");
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print("#sleep failed: error ");
    Serial.println(err);
  }
  }

  // Setup packet log
  for (uint8_t k = 0; k < egg_count; k++) {
    egg_packets[k] = (byte*) malloc(MAX_EGG_PACKET_LENGTH);
    egg_packet_rx_times[k]  = 0; // rtc.getEpoch();
    egg_packet_lengths[k]   = 0;
    egg_sbd_tosend_flag[k]  = 0;
  }


  // See if initialising display first helps...
  Serial.println("#Initialising display");
  display.begin();
  // initialise SD card
  Serial.print("#Initializing SD card... ");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDchipSelect, SD_SCK_MHZ(12))) {
  // if (!SD.begin(SDchipSelect)) {
  Serial.println("#Card failed, or not present");
  // don't do anything more:
  // while (1);
  }
  Serial.println("#card initialized OK");
  
  // Setup power switching
  Serial.println("#Initialising TPS22993 power switch");
  powerSwitchSetup();
  // Disable all channels
  powerSwitchEnable(PWR_SENSORS);
  delay(500);

  // radio modules
  Serial1.begin(19200);
  Serial2.begin(19200);
  pinPeripheral(19, PIO_SERCOM);
  pinPeripheral(6, PIO_SERCOM);
  // Setup other pins

  // Display switch pin should be set to a pullup:
  // HIGH = SLOW display mode
  // LOW  = FAST display mode
  pinMode(DISPLAY_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(DISPLAY_SWITCH_PIN, updateDisplayMode, CHANGE);
  // Configure display SPI connection
  updateDisplayMode();
  drawDisplay();

  if(accelerometer.beginI2C(I2C_ADDR_ACCEL) != BMA400_OK)
  {
    // Not connected, inform user
    Serial.println("#BMA400 not connected, check wiring and I2C address!");
  } else {
    Serial.println("#BMA400 initialised");
    bma400_ok = true;
  }

  // initialise barometric pressure sensor
  if(!P_Sens.begin()){
    Serial.println("#Error in communicating with barometric pressure sensor");
  }else{
    if (DIAGNOSTICS) {
      Serial.println("#MS5607 initialization successful!");
    }
    
  }
  delay(200);
  // initialise GPS sensor
  powerSwitchEnable(PWR_GPS | PWR_SENSORS);
  delay(200);
  Serial.println("#Initialising GNSS sensor");
  if (gnss.begin() == false) {
    Serial.println("#Failed to initialise GNSS");
    gnss_ok = false;
    powerSwitchDisable(PWR_GPS); 
  } else {
    gnss_ok = true;
    Serial.println("#Initialised and disabled GNSS");
    powerSwitchDisable(PWR_GPS); 
  }

  logger_ready = true; // we're ready to go

}

void Send_SBD(void) {
  
    int iridium_err = 0;
    uint8_t sbd_message_buffer[340];
    uint8_t packet_count = 0;
    uint16_t sbd_message_size = 0;
    bool more_to_send = false;
    int err;

    // Set last iridium attempt to now
    last_iridium_time = rtc.getEpoch();

    // Populate the SBD buffer from packets we have to send
    for (uint8_t k = 0; k < egg_count; k++) {
      if (egg_sbd_tosend_flag[k]) {
        // Check we can add the next message
        if (sbd_message_size + egg_packet_lengths[k] < 340) {
          memcpy(&sbd_message_buffer[sbd_message_size], egg_packets[k], egg_packet_lengths[k]);
          sbd_message_size += egg_packet_lengths[k];
          egg_sbd_tosend_flag[k] = 0;
          packet_count++;
        } else {
          more_to_send = true;
        }
      }
    }

    if (sbd_message_size == 0) {
      Serial.println("#No message to send");
      last_iridium_time = rtc.getEpoch();
      return;
    } else {
      Serial.print("#Trying to send ");
      Serial.print(packet_count);
      Serial.println(" packets by SBD");
    }

    if (more_to_send) {
      // we have more packets waiting, try and send another SBD immediately
      // by setting the last iridium time to 0
      last_iridium_time = 0;
    }
    // copy the message into another buffer
    // memcpy(sbd_message_buffer, SBD_message, message_offset);
    // sbd_message_size = message_offset;

    if (!FAKE_IRIDIUM) {
    // wake the modem
    Serial.println("#Waking up modem...");
    err = modem.begin();
    if (err != ISBD_SUCCESS)
    {
      Serial.print("#Begin failed: error ");
      Serial.println(err);
      if (err == ISBD_NO_MODEM_DETECTED)
        Serial.println("#No modem detected: check wiring.");
      return;
    }
      
    // actually send the message 
   
    Serial.println("#About to send message");
    iridium_err =  modem.sendSBDBinary(sbd_message_buffer, sbd_message_size);
    // the above call may take several minutes to return
    // while it is waiting for the message to go through, the library calls ISBDCallback() repeatedly to avoid totally blocking the thread
    // this will continue to accept incoming packets and store them
    
    if (iridium_err != ISBD_SUCCESS) {

      Serial.print("#SBD binary send failed, error code:");
      Serial.println(iridium_err);
    } else {
      Serial.println("#Message sent successfully");
    }
    
    // put modem back to sleep
    iridium_err = modem.sleep();
    if (iridium_err != ISBD_SUCCESS) 
    {
      Serial.print(F("#sleep failed: error "));
      Serial.println(iridium_err);
      
    }

    //fake iridium behaviour
    } else {
      Serial.println("#Sending fake iridium message");
      Serial.print("#");
      for (uint16_t k = 0; k < sbd_message_size; k++) 
        Serial.print(sbd_message_buffer[k], HEX);
      Serial.println();
    }
  
}

extern "C" char* sbrk(int incr);

int freeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

void store_packet (char * packet, int channel) {

// Cryoegg data packet format for SD and SBD
// 2 byte packet header - ASCII "C1" for "Cryoegg version 1"
// 4 byte UNIX timestamp
// 4 byte temperature from MS5607 sensor (floating point value in degC)
// 4 byte pressure from MS5607 sensor (floating point value in millibar)
// 2 byte ADC reading from solar battery voltage (multiply by 4.9 to get real volts)
// (16 bytes so far)
// 1 byte channel number (is it modem1 or modem 2)
// 1 byte length byte
// 1 byte C field
// 2 byte M ID
// 4 byte U ID
// 1 byte ver
// 1 byte dev
// 1 byte CI field
// (28 bytes so far)

// 2 byte electrical conductivity (ADC value)
// 2 byte PT1000 value (likely not used) (ADC value)
// 2 byte Keller pressure
// 2 byte Keller temperature
// 2 byte battery voltage (mV)
// 1 byte sequence number
// 1 byte RSSI
// 40 bytes 
// 10 bytes left for future expansion, as it is billed in 50 byte units so we want to try and keep to 50


  // array to hold data for writing to the SD card

  uint8_t SD_data[50]; // let's assume we won't go over 50 bytes

  // check that our packet isn't too big - 22 bytes is what we expect


  #define MAX_CRYOEGG_PACKET_LENGTH 22


  if (packet[0] > MAX_CRYOEGG_PACKET_LENGTH) {
    Serial.print("#Oversize packet received, truncating. Packet is ");
    Serial.print(packet[0]);
    Serial.println(" bytes long.");


    packet[0] = MAX_CRYOEGG_PACKET_LENGTH;
  }

  // SD card packet header - ASCII "C1"
  SD_data[0] = (uint8_t)'C';
  SD_data[1] = (uint8_t)'1';

  // get a UNIX timestamp from the real time clock
  uint32_t unix_timestamp;
  unix_timestamp = rtc.getEpoch();

  // convert timestamp to four bytes, little-endian to match the rest of the Cryoegg data
  SD_data[2] = unix_timestamp;
  SD_data[3] = unix_timestamp >> 8;
  SD_data[4] = unix_timestamp >> 16;
  SD_data[5] = unix_timestamp >> 24; 

  // datalogger sensors

  byte *t_ptr = (byte *) &logger_baro_temperature;
  byte *p_ptr = (byte *) &logger_baro_pressure;
  byte *s_ptr = (byte *) &solarVoltageRaw;

  SD_data[6] = t_ptr[0];
  SD_data[7] = t_ptr[1];
  SD_data[8] = t_ptr[2];
  SD_data[9] = t_ptr[3];

  SD_data[10] = p_ptr[0];
  SD_data[11] = p_ptr[1];
  SD_data[12] = p_ptr[2];
  SD_data[13] = p_ptr[3];

  SD_data[14] = s_ptr[0];
  SD_data[15] = s_ptr[1];

  // channel number

  SD_data[16] = channel;

  // now append the Cryoegg packet (including the length byte)

  memcpy( &(SD_data[17]), &(packet[0]), packet[0]+1); 

  // was FILE_WRITE
  File dataFile = SD.open("cryoegg.log", FILE_WRITE);
  
  if (dataFile) {
    dataFile.write(SD_data, packet[0]+19);
    dataFile.close();
  }
  else {
    Serial.println("#SD card writing error");
  }

  // Send packet by Iridium
  // if (USE_IRIDIUM) {
  // Change below made JH 2024-07-15 to prevent overwriting SBD buffer
  // if (USE_IRIDIUM && (packets_sent_since_garbage_ch1 > SBD_THROWAWAY_PACKETS -1 || packets_sent_since_garbage_ch2 > SBD_THROWAWAY_PACKETS - 1)) {
  //   // now append this whole new packet to the SBD buffer

  //   memcpy( &(SBD_message[message_offset]), SD_data, packet[0]+18); // received packet length, plus length byte, plus 17 bytes of logger data

  //   // adjust message_offset for next time
  //   message_offset = message_offset+packet[0]+18; // it's packet length + length byte + 17 bytes of logger data

  //   if (message_offset >=286) {

  //     // maximum size of an SBD message is 340 bytes. Assuming 54-bytes per packet, if we have fewer than 286 byte remaining we don't have enough space to store the next incoming packet

  //     // in which case, we're going to reset message_offset to 0 and overwrite some of the queued data
  //     // we can change this behaviour later if it becomes a problem
  //     Serial.println("#SBD message buffer full - overwriting previous queued data");
  //     message_offset = 0;
  //   }

    // reset last-packet-received time
    // last_packet_rx_time = unix_timestamp;

    uint32_t packet_id = * (uint32_t*) &(SD_data[18+3]);
    Serial.print("#Received from ");
    Serial.println(packet_id, HEX);
    
    // Search for packet in list
    for (uint8_t k = 0; k < egg_count; k++) {
      if (packet_id == egg_ids[k]) {
        Serial.print("#Identified from ");
        Serial.println(egg_ids[k], HEX);
        memcpy(egg_packets[k], SD_data, packet[0]+19);
        egg_packet_lengths[k] = packet[0]+19;
        egg_packet_rx_times[k] = unix_timestamp;
        egg_sbd_tosend_flag[k] = 1;
      }
    }

  }

  /* old iridium code removed

    int status;

    // wake modem
    status = modem.begin();
    if (status != ISBD_SUCCESS)
    {
      Serial.print(F("#Begin failed: error "));
      Serial.println(status);
      if (status == ISBD_NO_MODEM_DETECTED)
        Serial.println(F("#No Iridium modem detected: check wiring."));
      return;
    }


    status = modem.sendSBDBinary(SD_data, packet[0]+18); // send the whole packet to the modem. It's Cryoegg packet length + 18 bytes.

    if (status != ISBD_SUCCESS) 
    {
      Serial.print(F("#sendSBDBinary failed: error "));
      Serial.println(status);
      if (status == ISBD_SENDRECEIVE_TIMEOUT)
        Serial.println(F("#Try again with a better view of the sky."));
    }

    // put modem back to sleep
    status = modem.sleep();
    if (status != ISBD_SUCCESS) 
    {
      Serial.print(F("#sleep failed: error "));
      Serial.println(status);
      
    }


  }
  */


// }

void output_packet (char * packet, int channel) {
      // output the packet on the Native USB port so that we can decode it with the Python decoder
      if(Serial) {

        // we need to distinguish between the channel numbers here
        // so we'll send the channel number and the logger parameters as extra bytes
        // so that means we also increase the length byte so that the Python decoder knows to look for extra bytes

        packet[0] = packet[0] + 1; // increase length by 1
        packet[packet[0]] = channel; // add channel number 

        // temperature

        int8_t temp_int = 0;
        temp_int = (int8_t) (round(logger_baro_temperature)); // just take integer temperature, rounded to the nearest degree, and store in one byte. (-128 to +127 C)

        packet[0] = packet[0] + 1; // increase length by 1
        packet[packet[0]] = temp_int; // add temperature value 

        // pressure

        int16_t pressure_int = 0;
        pressure_int = (int16_t) (round(logger_baro_pressure*10)); // pressure in tenths of a millibar. (two bytes)
        byte *p_ptr = (byte *) &pressure_int; // create byte pointer so we can access bytes individually

        packet[0] = packet[0] + 2; // increase length by 2
        packet[packet[0]-1] = p_ptr[0];
        packet[packet[0]] = p_ptr[1];

        // solar voltage

        int16_t solar_int = 0;
        solar_int = (int16_t) (round(solar_voltage * 1000)) ; // solar/battery voltage in millivolts
        byte *s_ptr = (byte *) &solar_int; 

        packet[0] = packet[0] + 2; // increase length by 2
        packet[packet[0]-1] = s_ptr[0];
        packet[packet[0]] = s_ptr[1];




        Serial.write(packet, packet[0]+1);
      }

}

void processWirelessMBusSerial(void) {
   char WMB_incoming = 0;
   char new_packet[128];
   char *packet_ptr;

     // check for incoming packets from Cryoegg on Ch1
    if (Serial1.available() > 0 ) {
      Serial.println("# Serial1 available");
      WMB_incoming = Serial1.read();
    
      WMBus_Ch1.processByte(WMB_incoming); // send the byte to be made up into a packet

      if (WMBus_Ch1.isPacketReady() == true) {
        // we have a packet ready
        packet_ptr = WMBus_Ch1.getPacket();
        memcpy(new_packet, packet_ptr, (*packet_ptr)+1); // copy the packet
        store_packet(new_packet, 1);
        output_packet(new_packet, 1);
        packets_sent_since_garbage_ch1++;
      }
    }

     // check for incoming packets from Cryoegg on Ch2
    if (Serial2.available() > 0 ) {
      Serial.println("# Serial2 available");
      WMB_incoming = Serial2.read();
    
      WMBus_Ch2.processByte(WMB_incoming); // send the byte to be made up into a packet

      if (WMBus_Ch2.isPacketReady() == true) {
        // we have a packet ready
        packet_ptr = WMBus_Ch2.getPacket();
        memcpy(new_packet, packet_ptr, (*packet_ptr)+1); // copy the packet
        store_packet(new_packet, 2);
        output_packet(new_packet, 2);
        packets_sent_since_garbage_ch2++;
      }
    }

}

void updateLoggerParameters() {
  // this function updates the dataloggers own sensors/measurements
  unsigned long time_now = 0;

  time_now = millis();

  if (time_now <= (last_parameter_update + update_interval_millis)) {
    return; // don't bother updating if not enough time has passed since we last did it
  }

  last_parameter_update = time_now;

  // powerSwitchEnable(PWR_SENSORS | PWR_GPS);

  // update barometric sensor pressure and temperature

  if(P_Sens.readDigitalValue()) {
    logger_baro_temperature = P_Sens.getTemperature();
    logger_baro_pressure = P_Sens.getPressure();
  } else {
    Serial.println("#Error in reading digital value in barometric sensor.");
  }

  // Get BMA400 data
  accelerometer.getSensorData();

  if (DIAGNOSTICS) {
      Serial.print("#Temperature :  ");
      Serial.print(logger_baro_temperature);
      Serial.println(" C");
      Serial.print("#Pressure    :  ");
      Serial.print(logger_baro_pressure);
      Serial.println(" mBar");
      // 
      // Serial.print("#Temperature (SHT30): ");
      // Serial.print(sht30_results.temperature);
      // // Serial.print((float)sht30_results.temperature/(65535.0)*175-45);
      // Serial.println(" C");
      // Serial.print("#Humidity (SHT30): ");
      // Serial.print(sht30_results.humidity);
      // // Serial.print((float)sht30_results.humidity/(65535.0)*100);
      // Serial.println("% RH");
    
      // Print acceleration data
      Serial.print("#Acceleration in g's");
      Serial.print("\t");
      Serial.print("X: ");
      Serial.print(accelerometer.data.accelX, 3);
      Serial.print("\t");
      Serial.print("Y: ");
      Serial.print(accelerometer.data.accelY, 3);
      Serial.print("\t");
      Serial.print("Z: ");
      Serial.println(accelerometer.data.accelZ, 3);
  }

  // update solar/battery voltage


  analogReadResolution(12); // set 12-bit resolution for the ADC (0-4095)
  solarVoltageRaw = analogRead(solarVoltagePin);

  if (DIAGNOSTICS) {
    Serial.print("#Raw solar voltage value: ");
    Serial.println(solarVoltageRaw);
  }


  float solarVoltage = 0;
  // potential divider is 390k / 100k
  solarVoltage = solarVoltageRaw * 3.3; // scale for reference voltage
  solarVoltage = solarVoltage * solarVoltageCalibrationFactor; // scaling for potential divider
  solarVoltage = solarVoltage / 4095; // divide down for resolution of ADC

  if (DIAGNOSTICS) {
    Serial.print("#Solar voltage: ");
    Serial.print(solarVoltage);
    Serial.println(" V");

  }

  solar_voltage = solarVoltage;

  // powerSwitchDisable(PWR_SENSORS | PWR_GPS);


}

void loop() {
  // put your main code here, to run repeatedly:
  processWirelessMBusSerial();
  updateLoggerParameters();

  // Check if we are outside the time limit for updating the Iridium modem
  if (USE_IRIDIUM || GET_IRIDIUM_TIME) {

    if (rtc.getEpoch() > last_iridium_clock_update + IRIDIUM_HOUSEKEEPING_INTERVAL) {

      last_iridium_clock_update = rtc.getEpoch();
      Serial.println(F("#Begin Iridium housekeeping"));

      if (!iridium_ok) {
        Serial.println(F("#Performing iridium setup - in housekeeping"));
        setup_iridium();
      
      }

      if (iridium_ok) {
        Serial.println(F("#Getting iridium time - in housekeeping"));
        iridium_update_time();
      }

    }

  }

  if (USE_IRIDIUM && 
      rtc.getEpoch() > last_iridium_time + SBD_SEND_DELAY) {

      Serial.println("#Staring SBD message");
      Send_SBD();

  }

  refreshDisplay();

 
  // send an SBD packet
  // if (
  //   message_offset > 0 
  //   && ((last_packet_rx_time + SBD_SEND_DELAY) > rtc.getEpoch() > last_iridium_time + SBD_SEND_DEL) 
  //   && (packets_sent_since_garbage_ch1 > SBD_THROWAWAY_PACKETS -1 || packets_sent_since_garbage_ch2 > SBD_THROWAWAY_PACKETS - 1)
  // ) {
  // // Original IF statement below
  // // if (message_offset >0 && ((last_packet_rx_time + SBD_SEND_DELAY) > rtc.getEpoch() > last_iridium_time + SBD_SEND_DEL)
  //   // if there are messages in the buffer, and more than SBD_SEND_DELAY second has elapsed since we last received a packet, then go right ahead
  //   Send_SBD();
  // }
}

// callback from ISBD library to keep processing serial packets coming in while we wait for Iridium messages to send
bool ISBDCallback(void) {

  if (logger_ready==false) return true; // don't process data until everything is initialised

  processWirelessMBusSerial();
  updateLoggerParameters();

  return true;
}


// Cryoegg data packet format for SBD
// 2 byte packet header - ASCII "C1" for "Cryoegg version 1"
// 4 byte UNIX timestamp
// 4 byte temperature from MS5607 sensor (floating point value in degC)
// 4 byte pressure from MS5607 sensor (floating point value in millibar)
// 2 byte ADC reading from solar battery voltage (multiply by 4.9 to get real volts)
// (16 bytes so far)
// 1 byte channel number (is it modem1 or modem 2)
// 1 byte length byte
// 1 byte C field
// 2 byte M ID
// 4 byte U ID
// 1 byte ver
// 1 byte dev
// 1 byte CI field
// (28 bytes so far)
// 2 byte electrical conductivity (ADC value)
// 2 byte PT1000 value (likely not used) (ADC value)
// 2 byte Keller pressure
// 2 byte Keller temperature
// 2 byte battery voltage (mV)
// 2 byte sequence number
// 40 bytes 
// 10 bytes left for future expansion, as it is billed in 50 byte units so we want to try and keep to 50

void set_clock_time_compiler() {
  // This function uses the compiler set the current date and time for us.
  // This will be a few seconds off due to the time it takes to compile the
  // .ino file and upload the app. But pretty close.
  // code copied from Isaac Sobey's work on Hydrobean
   
  char s_month[5];
  int tmonth, tday, tyear, thour, tminute, tsecond;
  static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  byte days = 1;
  byte months = 1;
  byte years = 1;
  byte hours = 1;
  byte minutes = 1;
  byte seconds =1;
  
    // __DATE__ is a C++ preprocessor string with the current date in it.
  // It will look something like 'Mar  13  2016'.
  // So we need to pull those values out and convert the month string to a number.
  sscanf(__DATE__, "%s %d %d", s_month, &tday, &tyear);

  // Similarly, __TIME__ will look something like '09:34:17' so get those numbers.
  sscanf(__TIME__, "%d:%d:%d", &thour, &tminute, &tsecond);

  // Find the position of this month's string inside month_names, do a little
  // pointer subtraction arithmetic to get the offset, and divide the
  // result by 3 since the month names are 3 chars long.
  tmonth = (strstr(month_names, s_month) - month_names) / 3;

  months = tmonth + 1;  // The RTC library expects months to be 1 - 12.
  days = tday;
  years = tyear - 2000; // The RTC library expects years to be from 2000.
  hours = thour;
  minutes = tminute;
  seconds = tsecond;

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(days, months, years);
}

#if IRIDIUM_DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
