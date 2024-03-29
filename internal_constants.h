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
//  #define RESERVED_OUTPUT_3 6
//  #define RESERVED_OUTPUT_4 9
//  #define RESERVED_OUTPUT_5 4
//  #define RESERVED_OUTPUT_6 7
//  #define RESERVED_OUTPUT_7 3

#elif defined(ARDUINO_AVR_LEONARDO)
  #define INDICATOR_LED 13

  #define ENCODER1_CHA_PIN 9 /* Requires interrupt */
  #define ENCODER1_CHB_PIN 8 /* Requires interrupt */
  #define ENCODER1_BTN_PIN 12

  #define CEILING_LIGHT 11

  #define ENCODER1_LED_PIN INDICATOR_LED /* Prefers PWM */
  #define RESERVED_OUTPUT_2 10 /* WARNING: The PWM on this pin did not give consistent 0-100% steps when driving ceiling lights */

  #define HVAC_ERR_IN       4
  #define HVAC_HEAT_ON_IN   5
  #define HVAC_REQ_FAN      6
  #define HVAC_REQ_HEAT     7
//  #define HVAC_REQ_COOL     ?

// DO NOT USE 3, 2, 1, 0 BECAUSE OF CONFLICT WITH I2C AND SERIAL //

#else
  #error "CODE STUB"
#endif

#define DEBOUNCE_DELAY_MS 50

#define OUTPUT0 CEILING_LIGHT
#define OUTPUT1 ENCODER1_LED_PIN
#define OUTPUT2 RESERVED_OUTPUT_2



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
static const char topic_name_27[] PROGMEM = "og/hvac/error";
static const char topic_name_28[] PROGMEM = "og/hvac/heating";
static const char topic_name_29[] PROGMEM = "og/temperature/floor";
static const char topic_name_30[] PROGMEM = "og/temperature/ceiling";
static const char topic_name_31[] PROGMEM = "og/hvac/desired_temperature";
static const char topic_name_32[] PROGMEM = "og/hvac/state";
static const char topic_name_33[] PROGMEM = "og/bm/0/watts";
static const char topic_name_34[] PROGMEM = "og/solar/watts";
static const char topic_name_35[] PROGMEM = "og/inverter/watts";
static const char topic_name_36[] PROGMEM = "og/vehicle/watts";
static const char topic_name_37[] PROGMEM = "og/system_clock_h";
static const char topic_name_38[] PROGMEM = "og/system_clock_l";

static const char topic_unit_NONE[] PROGMEM = "";
static const char topic_unit_A[] PROGMEM = "A";
static const char topic_unit_ms[] PROGMEM = "ms";
static const char topic_unit_V[] PROGMEM = "V";
static const char topic_unit_Ah[] PROGMEM = "Ah";
static const char topic_unit_PCT[] PROGMEM = "%";
static const char topic_unit_min[] PROGMEM = "min";
static const char topic_unit_DEGC[] PROGMEM = "°C";
static const char topic_unit_W[] PROGMEM = "W";

/* TODO: Might be nice to put all of this in PROGMEM, but seems like it might be more hassle than it's worth */
static const Interface interface[] = {
  { MEMMAP_SETTING_BROADCAST_PERIOD_MS,     4,   0, AM_READWRITE, 0,  topic_name_01, topic_unit_ms },

  { MEMMAP_BANK0_VOLTS,                     2,  -2, AM_READ,      1,  topic_name_02, topic_unit_V },
  { MEMMAP_BANK0_AMPS,                      2,  -1, AM_READ,      1,  topic_name_03, topic_unit_A },
  { MEMMAP_BANK0_WATTS,                     2,  -1, AM_READ,      1,  topic_name_33, topic_unit_W },
  { MEMMAP_BANK0_AH_LEFT,                   2,  -1, AM_READWRITE, 0,  topic_name_04, topic_unit_Ah },
  { MEMMAP_BANK0_SOC,                       2,  -2, AM_READWRITE, 1,  topic_name_05, topic_unit_PCT },
  { MEMMAP_BANK0_TTG,                       2,  -1, AM_READ,      0,  topic_name_17, topic_unit_min },
  { MEMMAP_BANK0_CS,                        1,   0, AM_READ,      1,  topic_name_26, topic_unit_NONE },

  { MEMMAP_BANK0_AMPS_MULTIPLIER,           4,  -6, AM_READWRITE, 0,  topic_name_06, topic_unit_NONE },
  { MEMMAP_BANK0_VOLTS_MULTIPLIER,          4,  -6, AM_READWRITE, 0,  topic_name_07, topic_unit_NONE },
  { MEMMAP_BANK0_AH_CAPACITY,               2,  -1, AM_READWRITE, 0,  topic_name_08, topic_unit_Ah },
  { MEMMAP_BANK0_VOLTS_CHARGED,             2,  -3, AM_READWRITE, 0,  topic_name_09, topic_unit_V },
  { MEMMAP_BANK0_CHRG_DET_TIME,             2,  -1, AM_READWRITE, 0,  topic_name_10, topic_unit_min },
  { MEMMAP_BANK0_CURRENT_THRESHOLD,         4,  -6, AM_READWRITE, 0,  topic_name_11, topic_unit_A },
  { MEMMAP_BANK0_TAIL_CURRENT,              1,  -2, AM_READWRITE, 0,  topic_name_12, topic_unit_A },
  { MEMMAP_BANK0_PEUKERT_FACTOR,            1,  -2, AM_READWRITE, 0,  topic_name_13, topic_unit_NONE },
  { MEMMAP_BANK0_CHRG_EFFICIENCY,           1,  -2, AM_READWRITE, 0,  topic_name_14, topic_unit_NONE },

  { MEMMAP_PWM_OUTPUT0,                     1,   0, AM_READWRITE, 0,  topic_name_15, topic_unit_PCT },

  { MEMMAP_SOLAR_VOLTS,                     2,  -2, AM_READ,      1,  topic_name_20, topic_unit_V },
  { MEMMAP_SOLAR_AMPS,                      2,  -1, AM_READ,      1,  topic_name_21, topic_unit_A },
  { MEMMAP_SOLAR_WATTS,                     2,  -1, AM_READ,      1,  topic_name_34, topic_unit_W },

  { MEMMAP_VEHICLE_VOLTS,                   2,  -2, AM_READ,      1,  topic_name_22, topic_unit_V },
  { MEMMAP_VEHICLE_AMPS,                    2,  -1, AM_READ,      1,  topic_name_23, topic_unit_A },
  { MEMMAP_VEHICLE_WATTS,                   2,  -1, AM_READ,      1,  topic_name_36, topic_unit_W },

  { MEMMAP_INVERTER_VOLTS,                  2,  -2, AM_READ,      0,  topic_name_24, topic_unit_V },
  { MEMMAP_INVERTER_AMPS,                   2,  -1, AM_READ,      0,  topic_name_25, topic_unit_A },
  { MEMMAP_INVERTER_WATTS,                  2,  -1, AM_READ,      1,  topic_name_35, topic_unit_W },

  { MEMMAP_HVAC_ERROR,                      1,   0, AM_READ,      1,  topic_name_27, topic_unit_NONE },
  { MEMMAP_HVAC_HEAT_ON,                    1,   0, AM_READ,      1,  topic_name_28, topic_unit_NONE },

  { MEMMAP_TEMPERATURE_FLOOR,               2,  -1, AM_READWRITE, 1,  topic_name_29, topic_unit_DEGC },
  { MEMMAP_TEMPERATURE_CEILING,             2,  -1, AM_READWRITE, 1,  topic_name_30, topic_unit_DEGC },

  { MEMMAP_HVAC_DESIRED_TEMPERATURE,        2,  -1, AM_READWRITE, 1,  topic_name_31, topic_unit_DEGC },
  { MEMMAP_HVAC_STATE,                      1,   0, AM_READ,      1,  topic_name_32, topic_unit_NONE },

  { MEMMAP_SYSTEM_CLOCK_H32,                4,   0, AM_READWRITE, 0,  topic_name_37, topic_unit_NONE },
  { MEMMAP_SYSTEM_CLOCK_L32,                4,   0, AM_READWRITE, 0,  topic_name_38, topic_unit_NONE },
};

#endif /* __INTERNAL_CONSTANTS_H */
