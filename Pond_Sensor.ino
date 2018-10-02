/* -----------------------------------------------------------------
    Pond Sensor
    https://gitlab.com/Andy4495/Pond-Sensor

    09/09/2018 - A.T. - Original
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
         https://gitlab.com/Andy4495/MspTandV

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
#define sleepTime 63000UL
// ************************************************ //



#include "MspTandV.h"

#include <SPI.h>
#include <AIR430BoostFCC.h>

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

}


void loop() {

  // MSP430 internal temp sensor
  msp430Temp.read(CAL_ONLY);   // Only get the calibrated reading
  msp430T = msp430Temp.getTempCalibratedF();

  // MSP430 battery voltage (Vcc)
  msp430Vcc.read(CAL_ONLY);    // Only get the calibrated reading
  msp430mV = msp430Vcc.getVccCalibrated();

  ponddata.MSP_T     = msp430T;
  ponddata.Batt_mV   = msp430mV;
  ponddata.Millis    = millis();

  // Send the data over-the-air
  memcpy(&txPacket.message, &ponddata, sizeof(PondData));
#ifdef ENABLE_RADIO
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(PondData) + 4);
#endif

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, led_status);
  led_status = ~led_status;
#endif

  sleep(sleepTime);
}
