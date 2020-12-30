#ifndef __INTERNAL_CONSTANTS_H
#define __INTERNAL_CONSTANTS_H

#include <Adafruit_ADS1015.h>

// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

#define ADS1115_0_GAIN GAIN_SIXTEEN
#define ADS1115_1_GAIN GAIN_SIXTEEN
#define ADS1115_2_GAIN GAIN_ONE

#define INTERRUPT_PERIOD_MICROSECONDS 500

#define OUTPUT0 3
#define OUTPUT1 5
#define OUTPUT2 6
#define OUTPUT3 9
#define OUTPUT4 4 /* NO HARDWARE PWM */
#define OUTPUT5 7 /* NO HARDWARE PWM */

// Fatal error flash codes - make sure none of these are ambiguous when flashing as two hex nybbles
// 0x0? is acceptable but not 0x?0
// Maybe avoid 1's if possible
// Also avoid high numbers - maybe keep them under 8?
#define ERR_FATAL_WDT_TIMEOUT						0x03
#define ERR_FATAL_DEBUG_4							0x04

#define ERR_FATAL_UNHANDLED_INTERRUPT_1				0x11
#define ERR_FATAL_UNHANDLED_INTERRUPT_2				0x12
#define ERR_FATAL_UNHANDLED_INTERRUPT_3				0x13
#define ERR_FATAL_UNHANDLED_INTERRUPT_4				0x14

#define ERR_FATAL_RX_QUEUE_OVERFLOW					0x21
#define ERR_FATAL_TX_QUEUE_OVERFLOW					0x22

struct BMConst {
  double amps_multiplier; // Used to set mv/A, including any calibration
  double volts_multiplier; // for calibration
  uint16_t amphours_capacity; // Total battery capacity
  float volts_charged; // Voltage at which the battery is considered "charged"
  int minutes_charged_detection_time; // Amount of time the "charged" conditions need to be met before considered "charged"
  float current_threshold; // Amount of (abs) current that is considered "noise" and ignored.
  float tail_current_factor; // Current at which the battery is considered "charged"
  float peukert_factor;
  float charge_efficiency_factor;
};

// TODO: This needs to be manually kept in sync with the above structure.  Find a better way that automatically adjusts?
const uint8_t eeaddr_bank0_base = 0;

const uint8_t eeaddr_bank0_amps_multiplier = eeaddr_bank0_base;
const uint8_t eeaddr_bank0_volts_multiplier = eeaddr_bank0_amps_multiplier + sizeof(double);
const uint8_t eeaddr_bank0_amphours_capacity = eeaddr_bank0_volts_multiplier + sizeof(double);
const uint8_t eeaddr_bank0_volts_charged = eeaddr_bank0_amphours_capacity + sizeof(uint16_t);
const uint8_t eeaddr_bank0_minutes_charged_detection_time = eeaddr_bank0_volts_charged + sizeof(float);
const uint8_t eeaddr_bank0_current_threshold = eeaddr_bank0_minutes_charged_detection_time + sizeof(int);
const uint8_t eeaddr_bank0_tail_current_factor = eeaddr_bank0_current_threshold + sizeof(float);
const uint8_t eeaddr_bank0_peukert_factor = eeaddr_bank0_tail_current_factor + sizeof(float);
const uint8_t eeaddr_bank0_charge_efficiency_factor = eeaddr_bank0_peukert_factor + sizeof(float);

#endif /* __INTERNAL_CONSTANTS_H */
