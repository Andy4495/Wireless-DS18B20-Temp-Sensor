/* -----------------------------------------------------------------
    Pond Sensor
    https://github.com/Andy4495/wireless-ds18b20-temp-sensor

    09/09/2018 - A.T. - Original
    10/14/2018 - A.T. - Add support for DS18B20 Temp Sensor over
                        1-Wire communication.
    02/19/2019 - A.T. - Simplify code by removing support for LiPo-
                        based BATTPACK/BATPAKII BoosterPacks (since
                        LiPos don't work well over outdoor temp range)
*/
/* -----------------------------------------------------------------

   Uses "struct_type" value to differentiate from weather sensor.

   Configuration:
   - Update sleepTime variable to set the number of milliseconds to
     sleep between sensor readings.

   setup()
     I/O and sensor setup

   loop()
     Read and process individual sensor data
     Send data to receiver hub (via repeater)

   Data collected:                               Units
   - MSP430: Internal Temperature                F * 10
   - MSP430: Supply voltage (Vcc)                mV
   - External submerged temperature              F * 10 (not implemented yet)
   - Pump status                                 (not implemented yet)
   - Aerator status                              (not implemented yet)
   - Internal Timing:
                  Current value of millis()


  /*
    External libraries:
      Calibrated Temp and Vcc library "MspTandV"
         https://github.com/Andy4495/MspTandV

*/


#if defined(__MSP430FR4133__)
#define BOARD_LED LED2
#endif

#if defined(__MSP430FR6989__)
#define BOARD_LED GREEN_LED
#endif

#if defined(__MSP430G2553__)
#define BOARD_LED RED_LED
#endif

#if defined(__MSP430F5529__)
#define BOARD_LED GREEN_LED
#endif

#if defined(__MSP430FR2433__)
#define BOARD_LED LED2
#endif

#if defined(__MSP430FR5969__)
#define BOARD_LED LED2
#endif

// ****** Compile-time Configuration Options ****** //
// If using without CC110L BoosterPack,
// then comment out the following line:
#define ENABLE_RADIO
// Sleep Time in seconds
#define sleepTime 69
// *******************************
// Pin number used to power the DS18B20 (instead of tieing directly to Vcc)
#define DS18B20_SIGNAL_PIN  13
#define DS18B20_POWER_PIN   11
// Undefine BOARD_LED when putting the firmware into live system to save power
#undef BOARD_LED
// ************************************************ //


#include "MspTandV.h"       // https://github.com/Andy4495/MspTandV

#include <SPI.h>
#include <AIR430BoostFCC.h>
#include <OneWire.h>

// CC110L Declarations
#define ADDRESS_LOCAL   0x07    // This device
#define ADDRESS_REMOTE  0x01    // Receiver hub
#define DEFAULT_CHANNEL CHANNEL_3
#define ALT_CHANNEL     CHANNEL_1
channel_t txChannel;

enum {WEATHER_STRUCT, TEMP_STRUCT, POND_STRUCT};   // Added POND_STRUCT

struct sPacket
{
  uint8_t from;           // Local node address that message originated from
  uint8_t struct_type;    // Flag to indicate type of message structure
  uint8_t message[58];    // Local node message
};

struct sPacket txPacket;

struct PondData {
  int             MSP_T;          // Tenth degrees F
  int             Submerged_T;    // Tenth degrees F
  unsigned int    Batt_mV;        // milliVolts
  int             Pump_Status;    // Unimplemented
  int             Aerator_Status; // Unimplemented
  unsigned long   Millis;
};


PondData ponddata;

MspTemp msp430Temp;
MspVcc  msp430Vcc;

OneWire  ds18b20(DS18B20_SIGNAL_PIN);  // Requires a 4.7K pull-up resistor
// Address is not needed since we only have one device on the bus
uint8_t scratchpad[9];
// OneWire commands
#define GETTEMP         0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8
// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

int            msp430T;
int            msp430mV;

int            led_status = 0;

void setup() {

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, LOW);
  pinMode(BOARD_LED, OUTPUT);
#endif

  // Hold down PUSH2 during reset to choose alternate TX channel
  pinMode(PUSH2, INPUT_PULLUP);

  // If PUSH1 pressed during reset, then use alternate channel. Otherwise use default channel.
  if ( digitalRead(PUSH2) == HIGH) txChannel = DEFAULT_CHANNEL;
  else txChannel = ALT_CHANNEL;

#ifdef BOARD_LED
  // Flash the LED to indicate we started
  // Number of flashes indicates TX channel number
  int flashes;
  switch (txChannel) {
    case CHANNEL_1:
      flashes = 1;
      break;
    case CHANNEL_2:
      flashes = 2;
      break;
    case CHANNEL_3:
      flashes = 3;
      break;
    case CHANNEL_4:
      flashes = 4;
      break;
    default:
      flashes = 9;
      break;
  }
  for (int i = 0; i < flashes ; i++) {
    digitalWrite(BOARD_LED, HIGH);
    delay(350);
    digitalWrite(BOARD_LED, LOW);
    delay(350);
  }
#endif

  // CC110L Setup
  txPacket.from = ADDRESS_LOCAL;
  txPacket.struct_type = POND_STRUCT;
  memset(txPacket.message, 0, sizeof(txPacket.message));
#ifdef ENABLE_RADIO
  Radio.begin(ADDRESS_LOCAL, txChannel, POWER_MAX);
#endif

  // Set all structure values to zero on startup
  ponddata.MSP_T = 0;
  ponddata.Submerged_T = 0;
  ponddata.Batt_mV = 0;
  ponddata.Pump_Status = 0;
  ponddata.Aerator_Status = 0;
  ponddata.Millis = 0;

  // DS18B20 Setup
  digitalWrite(DS18B20_POWER_PIN, HIGH);
  pinMode(DS18B20_POWER_PIN, OUTPUT);          // Turn on the power

  // Read scratchpad to get current resolution value
  ds18b20.reset();
  ds18b20.skip();                    // Only one device on the bus, so don't need to bother with the address
  ds18b20.write(READSCRATCH);        // no parasitic power (2nd argument defaults to zero)

  for (int i = 0; i < 9; i++) {      // Read all 9 bytes
    scratchpad[i] = ds18b20.read();
  }

  switch (scratchpad[CONFIGURATION]) {
    case TEMP_9_BIT:
      setResolution(TEMP_10_BIT);         // Change resolution to 10 bits
      break;
    case TEMP_10_BIT:
      break;
    case TEMP_11_BIT:
      setResolution(TEMP_10_BIT);         // Change resolution to 10 bits
      break;
    case TEMP_12_BIT:
      setResolution(TEMP_10_BIT);         // Change resolution to 10 bits
      break;
    default:
      // Note: Unexpected CONFIG value
      break;
  }
}


void loop() {
  int16_t celsius, fahrenheit;

  // MSP430 internal temp sensor
  msp430Temp.read(CAL_ONLY);   // Only get the calibrated reading
  msp430T = msp430Temp.getTempCalibratedF();

  // MSP430 battery voltage (Vcc)
  msp430Vcc.read(CAL_ONLY);    // Only get the calibrated reading
  msp430mV = msp430Vcc.getVccCalibrated();

  ponddata.MSP_T     = msp430T;
  ponddata.Millis    = millis();
  ponddata.Batt_mV   = msp430mV;

  // Read the submerged temperature sensor (DS18B20)
  // Turn the power back on
  digitalWrite(DS18B20_POWER_PIN, HIGH);
  pinMode(DS18B20_POWER_PIN, OUTPUT);

  // Start temperature conversion
  ds18b20.reset();
  ds18b20.skip();                // Only one device on the bus, so don't need to bother with the address
  ds18b20.write(GETTEMP);        // no parasitic power (2nd argument defaults to zero)
  delay(225);     // 10-bit needs 187.5 ms for conversion, add a little extra just in case

  // Read back the temperature
  ds18b20.reset();
  ds18b20.skip();
  ds18b20.write(READSCRATCH);
  for ( int i = 0; i < 2; i++) {           // Only need 2 bytes to get the temperature
    scratchpad[i] = ds18b20.read();
  }
  pinMode(DS18B20_POWER_PIN, INPUT);          // Turn off the power to DS18B20

  // Convert the data to actual temperature
  int16_t raw = (scratchpad[1] << 8) | scratchpad[0]; // Put the temp bytes into a 16-bit integer
  raw = raw & ~0x03;               // 10-bit resolution, so ignore 2 lsb
  // Raw result is in 16ths of a degree Celsius
  // fahrenheit = celsius * 1.8 + 32.0, but we want to use integer math
  celsius = (raw * 10) >> 4;                   // Convert to 10th degree celsius
  fahrenheit = ((celsius * 9) / 5) + 320;      // C to F using integer math (values are in tenth degrees)
  ponddata.Submerged_T = fahrenheit;

  // Eventually add pump and aerator status


  // Send the data over-the-air
  memcpy(&txPacket.message, &ponddata, sizeof(PondData));
#ifdef ENABLE_RADIO
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(PondData) + 4);
#endif

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, led_status);
  led_status = ~led_status;
#endif

  sleepSeconds(sleepTime);
} /* loop() */

void setResolution(uint8_t resolution) {
  ds18b20.reset();
  ds18b20.skip();                          // Only one device on the bus, so don't need to bother with the address
  ds18b20.write(WRITESCRATCH);             // no parasitic power (2nd argument defaults to zero)

  scratchpad[CONFIGURATION] = resolution;  // Set the resolution value. Don't care about TH and TL, so don't bother setting.

  for (int i = HIGH_ALARM_TEMP; i <= CONFIGURATION; i++) {  // 3 bytes required for the WRITESCRATCH command
    ds18b20.write(scratchpad[i]);
  }

  ds18b20.reset();
  ds18b20.skip();                          // Only one device on the bus, so don't need to bother with the address
  ds18b20.write(COPYSCRATCH);              // no parasitic power (2nd argument defaults to zero)

  delay(15);                               // Need a minimum of 10ms per datasheet after copy scratch
}
