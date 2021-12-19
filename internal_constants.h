#ifndef __INTERNAL_CONSTANTS_H
#define __INTERNAL_CONSTANTS_H

#if defined(TEENSYDUINO) 

    //  --------------- Teensy -----------------

    #if defined(__AVR_ATmega32U4__)
        #define BOARD "Teensy 2.0"
    #elif defined(__AVR_AT90USB1286__)       
        #define BOARD "Teensy++ 2.0"
    #elif defined(__MK20DX128__)       
        #define BOARD "Teensy 3.0"
    #elif defined(__MK20DX256__)       
        #define BOARD "Teensy 3.2" // and Teensy 3.1 (obsolete)
    #elif defined(__MKL26Z64__)       
        #define BOARD "Teensy LC"
    #elif defined(__MK64FX512__)
        #define BOARD "Teensy 3.5"
    #elif defined(__MK66FX1M0__)
        #define BOARD "Teensy 3.6"
    #else
       #error "Unknown board"
    #endif

#else // --------------- Arduino ------------------

    #if   defined(ARDUINO_AVR_ADK)       
        #define BOARD "Mega Adk"
    #elif defined(ARDUINO_AVR_BT)    // Bluetooth
        #define BOARD "Bt"
    #elif defined(ARDUINO_AVR_DUEMILANOVE)       
        #define BOARD "Duemilanove"
    #elif defined(ARDUINO_AVR_ESPLORA)       
        #define BOARD "Esplora"
    #elif defined(ARDUINO_AVR_ETHERNET)       
        #define BOARD "Ethernet"
    #elif defined(ARDUINO_AVR_FIO)       
        #define BOARD "Fio"
    #elif defined(ARDUINO_AVR_GEMMA)
        #define BOARD "Gemma"
    #elif defined(ARDUINO_AVR_LEONARDO)       
        #define BOARD "Leonardo"
    #elif defined(ARDUINO_AVR_LILYPAD)
        #define BOARD "Lilypad"
    #elif defined(ARDUINO_AVR_LILYPAD_USB)
        #define BOARD "Lilypad Usb"
    #elif defined(ARDUINO_AVR_MEGA)       
        #define BOARD "Mega"
    #elif defined(ARDUINO_AVR_MEGA2560)       
        #define BOARD "Mega 2560"
    #elif defined(ARDUINO_AVR_MICRO)       
        #define BOARD "Micro"
    #elif defined(ARDUINO_AVR_MINI)       
        #define BOARD "Mini"
    #elif defined(ARDUINO_AVR_NANO)       
        #define BOARD "Nano"
    #elif defined(ARDUINO_AVR_NG)       
        #define BOARD "NG"
    #elif defined(ARDUINO_AVR_PRO)       
        #define BOARD "Pro"
    #elif defined(ARDUINO_AVR_ROBOT_CONTROL)       
        #define BOARD "Robot Ctrl"
    #elif defined(ARDUINO_AVR_ROBOT_MOTOR)       
        #define BOARD "Robot Motor"
    #elif defined(ARDUINO_AVR_UNO)       
        #define BOARD "Uno"
    #elif defined(ARDUINO_AVR_YUN)       
        #define BOARD "Yun"

    // These boards must be installed separately:
    #elif defined(ARDUINO_SAM_DUE)       
        #define BOARD "Due"
    #elif defined(ARDUINO_SAMD_ZERO)       
        #define BOARD "Zero"
    #elif defined(ARDUINO_ARC32_TOOLS)       
        #define BOARD "101"
    #else
       #error "Unknown board"
    #endif

#endif

#define INTERRUPT_PERIOD_MICROSECONDS 500

#define ENCODER1_STEP 10

#if defined(ARDUINO_AVR_NANO)
  #define INDICATOR_LED 13

  #define ENCODER1_CHA_PIN A0
  #define ENCODER1_CHB_PIN A1
  #define ENCODER1_BTN_PIN A2

  #define ENCODER1_LED_PIN 8
  #define CEILING_LIGHT 3
  #define RESERVED_OUTPUT_2 5
  #define RESERVED_OUTPUT_3 6
  #define RESERVED_OUTPUT_4 9
  #define RESERVED_OUTPUT_5 4
  #define RESERVED_OUTPUT_6 7
  #define RESERVED_OUTPUT_7 3

#elif defined(ARDUINO_AVR_LEONARDO)
  #define INDICATOR_LED 13

  #define ENCODER1_CHA_PIN 9 /* Requires interrupt */
  #define ENCODER1_CHB_PIN 8 /* Requires interrupt */
  #define ENCODER1_BTN_PIN 12

  #define CEILING_LIGHT 11

  #define ENCODER1_LED_PIN INDICATOR_LED /* Prefers PWM */
  #define RESERVED_OUTPUT_2 10 /* WARNING: The PWM on this pin did not give consistent 0-100% steps when driving ceiling lights */
  #define RESERVED_OUTPUT_3 A0 /* NO HARDWARE PWM */
  #define RESERVED_OUTPUT_4 A1 /* NO HARDWARE PWM */
  #define RESERVED_OUTPUT_5 A2 /* NO HARDWARE PWM */
  #define RESERVED_OUTPUT_6 A3 /* NO HARDWARE PWM */
  #define RESERVED_OUTPUT_7 A4 /* NO HARDWARE PWM */

#else
  #error "CODE STUB"
#endif

#define DEBOUNCE_DELAY_MS 50

#define OUTPUT0 CEILING_LIGHT
#define OUTPUT1 ENCODER1_LED_PIN
#define OUTPUT2 RESERVED_OUTPUT_2
#define OUTPUT3 RESERVED_OUTPUT_3
#define OUTPUT4 RESERVED_OUTPUT_4
#define OUTPUT5 RESERVED_OUTPUT_5
#define OUTPUT6 RESERVED_OUTPUT_6
#define OUTPUT7 RESERVED_OUTPUT_7

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

enum ChargeState {
  CS_NONE,
  CS_DISCHARGING,
  CS_CHARGING,
  CS_CHARGED_PENDING,
  CS_CHARGED
};

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

struct BMVar {
  float volts; // Battery voltage at last sample
  float amps; // Battery current at last sample
  float amphours_remaining; // Tracks coulombs
  // TODO: Add total Ah in and out
  //float percent_soc; // Redundant? Only use amphours_remaining / amphours_capacity ?
  unsigned int charge_state;
  unsigned long charged_pending_timestamp;
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


// TODO: To save some memory, consider referencing group string prefixes instead of the duplication below.
static const char topic_name_01[] PROGMEM = "og/setting/broadcast_period_ms";
static const char topic_name_02[] PROGMEM = "og/bm/0/volts";
static const char topic_name_03[] PROGMEM = "og/bm/0/amps";
static const char topic_name_04[] PROGMEM = "og/bm/0/ah";
static const char topic_name_05[] PROGMEM = "og/bm/0/soc";
static const char topic_name_06[] PROGMEM = "og/bm/0/amps_multiplier";
static const char topic_name_07[] PROGMEM = "og/bm/0/volts_multiplier";
static const char topic_name_08[] PROGMEM = "og/bm/0/amphours_capacity";
static const char topic_name_09[] PROGMEM = "og/bm/0/volts_charged";
static const char topic_name_10[] PROGMEM = "og/bm/0/minutes_charged_detection_time";
static const char topic_name_11[] PROGMEM = "og/bm/0/current_threshold";
static const char topic_name_12[] PROGMEM = "og/bm/0/tail_current_factor";
static const char topic_name_13[] PROGMEM = "og/bm/0/peukert_factor";
static const char topic_name_14[] PROGMEM = "og/bm/0/charge_efficiency_factor";
static const char topic_name_15[] PROGMEM = "og/house/light/ceiling";
static const char topic_name_16[] PROGMEM = "og/house/light/ceiling_encoder";
static const char topic_name_17[] PROGMEM = "og/bm/0/ttg";
static const char topic_name_20[] PROGMEM = "og/solar/volts";
static const char topic_name_21[] PROGMEM = "og/solar/amps";
static const char topic_name_22[] PROGMEM = "og/vehicle/volts";
static const char topic_name_23[] PROGMEM = "og/vehicle/amps";
static const char topic_name_24[] PROGMEM = "og/inverter/volts";
static const char topic_name_25[] PROGMEM = "og/inverter/amps";
static const char topic_name_26[] PROGMEM = "og/bm/0/cs";

static const char topic_unit_01[] PROGMEM = "ms";
static const char topic_unit_02[] PROGMEM = "V";
static const char topic_unit_03[] PROGMEM = "A";
static const char topic_unit_04[] PROGMEM = "Ah";
static const char topic_unit_05[] PROGMEM = "%";
static const char topic_unit_06[] PROGMEM = "";
static const char topic_unit_07[] PROGMEM = "";
static const char topic_unit_08[] PROGMEM = "Ah";
static const char topic_unit_09[] PROGMEM = "V";
static const char topic_unit_10[] PROGMEM = "min";
static const char topic_unit_11[] PROGMEM = "A";
static const char topic_unit_12[] PROGMEM = "A";
static const char topic_unit_13[] PROGMEM = "";
static const char topic_unit_14[] PROGMEM = "";
static const char topic_unit_15[] PROGMEM = "%";
static const char topic_unit_16[] PROGMEM = "%";
static const char topic_unit_17[] PROGMEM = "min";
static const char topic_unit_20[] PROGMEM = "V";
static const char topic_unit_21[] PROGMEM = "A";
static const char topic_unit_22[] PROGMEM = "V";
static const char topic_unit_23[] PROGMEM = "A";
static const char topic_unit_24[] PROGMEM = "V";
static const char topic_unit_25[] PROGMEM = "A";
static const char topic_unit_26[] PROGMEM = "";

/* TODO: Might be nice to put all of this in PROGMEM, but seems like it might be more hassle than it's worth */
static const Interface interface[] = {
  { MEMMAP_SETTING_BROADCAST_PERIOD_MS,     4,   0, AM_READWRITE, 0,  topic_name_01, topic_unit_01 },

  { MEMMAP_BANK0_VOLTS,                     2,  -2, AM_READ,      1,  topic_name_02, topic_unit_02 },
  { MEMMAP_BANK0_AMPS,                      2,  -1, AM_READ,      1,  topic_name_03, topic_unit_03 },
  { MEMMAP_BANK0_AH_LEFT,                   2,  -1, AM_READWRITE, 0,  topic_name_04, topic_unit_04 },
  { MEMMAP_BANK0_SOC,                       2,  -2, AM_READWRITE, 1,  topic_name_05, topic_unit_05 },
  { MEMMAP_BANK0_TTG,                       2,  -1, AM_READ,      0,  topic_name_17, topic_unit_17 },
  { MEMMAP_BANK0_CS,                        1,   0, AM_READ,      1,  topic_name_26, topic_unit_26 },

  { MEMMAP_BANK0_AMPS_MULTIPLIER,           4,  -6, AM_READWRITE, 0,  topic_name_06, topic_unit_06 },
  { MEMMAP_BANK0_VOLTS_MULTIPLIER,          4,  -6, AM_READWRITE, 0,  topic_name_07, topic_unit_07 },
  { MEMMAP_BANK0_AH_CAPACITY,               2,  -1, AM_READWRITE, 0,  topic_name_08, topic_unit_08 },
  { MEMMAP_BANK0_VOLTS_CHARGED,             2,  -3, AM_READWRITE, 0,  topic_name_09, topic_unit_09 },
  { MEMMAP_BANK0_CHRG_DET_TIME,             2,  -1, AM_READWRITE, 0,  topic_name_10, topic_unit_10 },
  { MEMMAP_BANK0_CURRENT_THRESHOLD,         4,  -6, AM_READWRITE, 0,  topic_name_11, topic_unit_11 },
  { MEMMAP_BANK0_TAIL_CURRENT,              1,  -2, AM_READWRITE, 0,  topic_name_12, topic_unit_12 },
  { MEMMAP_BANK0_PEUKERT_FACTOR,            1,  -2, AM_READWRITE, 0,  topic_name_13, topic_unit_13 },
  { MEMMAP_BANK0_CHRG_EFFICIENCY,           1,  -2, AM_READWRITE, 0,  topic_name_14, topic_unit_14 },

  { MEMMAP_PWM_OUTPUT0,                     1,   0, AM_READWRITE, 1,  topic_name_15, topic_unit_15 },

  { MEMMAP_SOLAR_VOLTS,                     2,  -2, AM_READ,      1,  topic_name_20, topic_unit_20 },
  { MEMMAP_SOLAR_AMPS,                      2,  -1, AM_READ,      1,  topic_name_21, topic_unit_21 },

  { MEMMAP_VEHICLE_VOLTS,                   2,  -2, AM_READ,      1,  topic_name_22, topic_unit_22 },
  { MEMMAP_VEHICLE_AMPS,                    2,  -1, AM_READ,      1,  topic_name_23, topic_unit_23 },

  { MEMMAP_INVERTER_VOLTS,                  2,  -2, AM_READ,      0,  topic_name_24, topic_unit_24 },
  { MEMMAP_INVERTER_AMPS,                   2,  -1, AM_READ,      1,  topic_name_25, topic_unit_25 },

};

#endif /* __INTERNAL_CONSTANTS_H */
