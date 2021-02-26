#include <avr/wdt.h>
#include <EEPROM.h>
#include <TimerOne.h>
//#include <Adafruit_ADS1015.h>
//#include <CircularBuffer.h>

// KEEP THIS UPPERCASE AND FIX OTHER PROBLEMS
#include <Rotary.h> // https://github.com/brianlow/Rotary.git

//#include <SPI.h>
#include <Button.h> // https://github.com/jimmybyrum/arduino/git
//#include <SDHT.h>
//#include <PCF8591.h> /* Can interfere with interrupts! */
//#include <ArduinoUniqueID.h>
//#include <wire.h>

#include "shared_constants.h"
#include "internal_constants.h"

// See avr/iom328p.h for register & bit definitions

/*
    ARDUINO PRO MINI
    +------------------------------------------------+
    |   [ 1] D1/PD1/TXD                   RAW {24}   |
    |   [ 2] D0/PD0/RXD                   GND {23}   |
    |   [ 3] RST/PC6/PCINT14  RST/PC6/PCINT14 {22}   |
    |   [ 4] GND                          VCC {21}   |
    |  *[ 5] D2/PD2/INT0           PC3/A3/D17 {20}   |
    |  *[ 6] D3/PD3/INT1/PWM       PC2/A2/D16 {19}   |
    |  *[ 7] D4/PD4                PC1/A1/D15 {18}   |
    |  *[ 8] D5/PD5/PWM            PC0/A0/D14 {17}   |
    |  *[ 9] D6/PD6/PWM           SCK/PB5/D13 {16}*  |
    |  *[10] D7/PD7              MISO/PB4/D12 {15}*  |
    |   [11] D8/PB0          PWM/MOSI/PB3/D11 {14}*  |
    |  *[12] D9/PB1/PWM        PWM/SS/PB2/D10 {13}*  |
    |                                                |
    |   Additional pin headers:                      |
    |    *A5/D19/PC5/ADC5/SCL/PCINT13                |
    |    *A4/D18/PC4/ADC4/SDA/PCINT12                |
    |                                                |
    |     A7/ADC7                                    |
    |     A6/ADC6                                    |
    +------------------------------------------------+

    *** REVIEW THIS AS IT LIKELY CHANGED IN DEVELOPMENT!!!

    Hardware Notes

    D13/SCK
    D12/MISO
    D11/MOSI
    D10/SS
    D2/INT0 - Interrupt reserved for SPI device

    D3/PWM - OUTPUT0 (Ceiling light)
    D5/PWM - OUTPUT1
    D6/PWM - OUTPUT2
    D9/PWM - OUTPUT3
    D4 - OUTPUT4
    D7 - OUTPUT5
    D8 - OUTPUT6 (Encoder LED)
    ?? - OUTPUT7

    A5/SCL
    A4/SDA

    A3 - Rotary Encoder BTN
    A1 - Rotary Encoder CHA
    A0 - Rotary Encoder CHB

*/

// Look up table for converting ID to actual output
const byte output[8] = {OUTPUT0, OUTPUT1, OUTPUT2, OUTPUT3, OUTPUT4, OUTPUT5, OUTPUT6, OUTPUT7};
byte output_value[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // TODO: read/write should only be done through SetPWM/GetPWM.  To be refactored into a class at some point.

enum ReceiveState {
        GET_STX = 1,
        GET_DATA = 2,
};

// Variables related to status code LED and flashing
volatile bool flash_led = false;
volatile uint8_t flash_countdown;
volatile uint8_t error_code = 0;
volatile uint32_t flash_ticks;
volatile uint32_t error_code_ticks;

volatile bool base_timer_flag = false;

volatile bool one_second_flag = 0;
volatile uint16_t timer_counter_1 = (1000) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool quarter_second_flag = 0;
volatile uint16_t timer_counter_3 = (250) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool ten_millisecond_flag = 0;
volatile uint16_t timer_counter_2 = (10) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool wdt_isr_executed = false;

enum Encoder1Mode {
  ENC1MODE_DIMMER,
  ENC1MODE_NAVIGATE,
} encoder1_mode;

Rotary encoder1 = Rotary(ENCODER1_CHA_PIN, ENCODER1_CHB_PIN);
Button encoder1_button = Button(ENCODER1_BTN_PIN, PULLUP);
volatile int encoder1_value = 0;
int encoder1_value_prev = 0;
bool encoder1_button_value = 0;

// ADC Modules
//Adafruit_ADS1115 ads0(0x48);
//Adafruit_ADS1115 ads1(0x49);
//Adafruit_ADS1115 ads2(0x4A);
//Adafruit_ADS1115 ads3(0x4B); // Possible to add one more if wanted

struct BMConst battery_monitor_const[2];
struct BMVar battery_monitor_var[2];

int32_t adc_sum[8] = {0};
int8_t adc_count[8] = {0};

bool inhibit_broadcast = false;
uint32_t broadcast_period_ms = 1000;
volatile uint32_t broadcast_timer_counter = broadcast_period_ms * (1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001));
volatile bool broadcast_flag = 0;

void setup() {

  Serial.begin(115200);

  noInterrupts();

  // Safety code to start with disabled watchdog timer
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;

  // 500Hz / 500 = 1 second
  Timer1.initialize(INTERRUPT_PERIOD_MICROSECONDS);
  Timer1.attachInterrupt(handleTimerTick);

  for (unsigned int i = 0; i < sizeof(output); i++) {
    pinMode(output[i], OUTPUT);
    SetPWM(i, 0);
  }

//  ads0.setGain(ADS1115_0_GAIN);
//  ads1.setGain(ADS1115_1_GAIN);
//  ads2.setGain(ADS1115_2_GAIN);

//  ads0.begin();
//  ads1.begin();
//  ads2.begin();

  Timer1.start();
  interrupts();

  //setupWatchdogTimer(); // Masks a problem where this stops responding on the CAN bus

  //uint8_t last_mcp_eflg=0, last_mcp_rec=0, last_mcp_tec=0, last_rx_queue_count=0, last_rx_queue_count_max=0;

//  serialize(MSG_POWER_ON, "bbbbb", last_mcp_eflg, last_mcp_rec, last_mcp_tec, last_rx_queue_count, last_rx_queue_count_max);
  serialize(MSG_POWER_ON, "");

  initEncoders();
  initBatteryMonitor();

  returnAllInterfaces(); // As a convenience, do this on startup/reset so that connected device doesn't have to request it
}

void setupWatchdogTimer(void) {
  // Configure the watchdog timer for system reset and interrupt, 8 second timeout.
  noInterrupts();
  wdt_reset();

  MCUSR = 0;
  _WD_CONTROL_REG = (1 << _WD_CHANGE_BIT) | (1 << WDE); // Begin timed sequence - WDCE (change enable) resets in 4 cycles
  //_WD_CONTROL_REG = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP0);  // Interrupt & system reset, 8 second timeout
  //_WD_CONTROL_REG = (1 << WDE) | (1 << WDP3) | (1 << WDP0);  // System reset, 8 second timeout
  _WD_CONTROL_REG = (0 << WDE) | (1 << WDP3) | (1 << WDP0);  // Disable WDT

  interrupts();
}

void loop() {
  processSerialReceive();
  processEncoders();
  //processPWMEnable();

  if (base_timer_flag) {
    base_timer_flag = false;

  }

  if(broadcast_flag) {
    broadcast_flag = false;

    if(!inhibit_broadcast) {
      broadcastPWMValues();
      broadcastADCValues(); /* TODO: TO BE DEPRECATED! */
      broadcastBatteryMonitor();
    }
  }
  
  if (ten_millisecond_flag) {
    ten_millisecond_flag = false;

    processEncoderLEDState();
    ReadADCs(); // TODO: This takes longer than it needs to and should be set up for continuous conversion.
  }

  if (quarter_second_flag) {
    quarter_second_flag = false;

  }

  if (one_second_flag) {
    one_second_flag = false;

    processBatteryMonitor();
  }

}

void initEncoders(void) {
#if defined(ARDUINO_AVR_NANO)
  PCICR = 0b00000010; // Turn on PORTC pins
  PCMSK0 = 0b00000000;
  PCMSK1 = 0b00000011; // Interrupt on change of A0(14), A1(15)
  PCMSK2 = 0b00000000;
#elif defined(ARDUINO_AVR_LEONARDO)
  PCICR = 0b00000001;
  PCMSK0 = 0b00110000;
#else
  #error "CODE STUB"
#endif 
}

void initBatteryMonitor(void) {
  // First time requires that the EEPROM be pre-programmed.  See offgrid_init_eeprom.ino
  EEPROM.get(0, battery_monitor_const); // Copy semi-constants from EEPROM to battery monitor structures

//  for (unsigned int i = 0; i < ( sizeof(battery_monitor_const)/sizeof(battery_monitor_const[0]) ); i++) {
//    Serial.print("Bank ");Serial.print(i); Serial.println(":");
//    Serial.print("amps_multiplier: "); Serial.println(battery_monitor_const[i].amps_multiplier, 6);
//    Serial.print("volts_multiplier: "); Serial.println(battery_monitor_const[i].volts_multiplier, 6);
//    Serial.print("amphours_capacity: "); Serial.println(battery_monitor_const[i].amphours_capacity);
//    Serial.print("volts_charged: "); Serial.println(battery_monitor_const[i].volts_charged, 3);
//    Serial.print("minutes_charged_detection_time: "); Serial.println(battery_monitor_const[i].minutes_charged_detection_time);
//    Serial.print("current_threshold: "); Serial.println(battery_monitor_const[i].current_threshold, 6);
//    Serial.print("tail_current_factor: "); Serial.println(battery_monitor_const[i].tail_current_factor, 2);
//    Serial.print("peukert_factor: "); Serial.println(battery_monitor_const[i].peukert_factor, 3);
//    Serial.print("charge_efficiency_factor: "); Serial.println(battery_monitor_const[i].charge_efficiency_factor, 2);
//    Serial.println();
//  }

  for (unsigned int i = 0; i < ( sizeof(battery_monitor_var) / sizeof(battery_monitor_var[0]) ); i++) {
    // At startup/reset we can assume nothing about these until a full charge synchronization.
    battery_monitor_var[i].amphours_remaining = battery_monitor_const[i].amphours_capacity;
    battery_monitor_var[i].percent_soc = 100;
    battery_monitor_var[i].charge_state = CS_NONE;
  }
}

// TODO: This could be used to signal error codes as well
void processEncoderLEDState(void) {
  if(GetMemoryMap(MEMMAP_PWM_OUTPUT0) <= 0) {
    SetPWM(1, 15);
  }
  else {
    SetPWM(1, 0);
  }
}

void processBatteryMonitor(void) {  
  // TODO: Consider a way to process all battery monitor instances consistently in a loop
  battery_monitor_var[0].amps = ((float)adc_sum[0] / (float)adc_count[0]) * battery_monitor_const[0].amps_multiplier;
  battery_monitor_var[0].volts = ((float)adc_sum[4] / (float)adc_count[4]) * battery_monitor_const[0].volts_multiplier;
//  battery_monitor_var[1].amps = (float)(adc_sum[2] / adc_count[2]) * battery_monitor_const[1].amps_multiplier;
//  battery_monitor_var[1].volts = (float)(adc_sum[5] / adc_count[5]) * battery_monitor_const[1].volts_multiplier;

  // TODO: Add variables for calculated time remaining (shows both charge and discharge by averaging power in/out over time
  for (unsigned int i = 0; i < ( sizeof(battery_monitor_var) / sizeof(battery_monitor_var[0]) ); i++) {
    battery_monitor_var[i].amphours_remaining += 1.0 / 3600.0 * battery_monitor_var[i].amps; // Calculation assumes execution every 1 second!
    battery_monitor_var[i].amphours_remaining = constrain(battery_monitor_var[i].amphours_remaining, 0, battery_monitor_const[i].amphours_capacity);
    battery_monitor_var[i].percent_soc = (battery_monitor_var[i].amphours_remaining / battery_monitor_const[i].amphours_capacity ) * 100;

//#error "Unfinished code - charge state machine"
//
//    switch(battery_monitor_var_charge_state[i]) {
//      CS_NONE: 
//        if(battery_monitor_var[i].amps > 0) { charge_state = CS_CHARGING; }
//        else if(battery_monitor_var[i].amps < 0) { charge_state = CS_DISCHARGING; }
//        break;
//
//      CS_CHARGED:        
//        if(battery_monitor_var[i].amps < 0) { charge_state = CS_DISCHARGING; }
//        break;
//        
//      CS_CHARGING:
//        if(battery_monitor_var[i].amps < 0) { charge_state = CS_DISCHARGING; }
//        else if(battery_monitor_var[i].amps == 0) { charge_state = CS_NONE; }
//        // If voltage rises above 'volts_charged' and current falls below 'tail_current_factor', start a timer and set a "sync_pending" flag
//        // If voltage falls below 'volts_charged' or current rises above 'tail_current_factor' (less some hysterisis amounts), stop the timer and clear the "sync_pending" flag
//
//        // If "sync_pending" is set and timer exceeds 'minutes_charged_detection_time', set all appropriate variables then switch to CHARGED state
//        break;
//        
//      CS_DISCHARGING:
//        if(battery_monitor_var[i].amps > 0) { charge_state = CS_CHARGING; }
//        else if(battery_monitor_var[i].amps == 0) { charge_state = CS_NONE; }
//        break;
//        
//    }

  }

}

void processEncoders() {

  // Only do something if the encoder value has actually changed
  if(encoder1_value != encoder1_value_prev) {
    //Serial.println(encoder1_value);
    SetMemoryMap(MEMMAP_PWM_OUTPUT0, encoder1_value);
    serialize(MSG_RETURN_8_8, "bb", (uint8_t)MEMMAP_PWM_OUTPUT0, (uint8_t)GetMemoryMap(MEMMAP_PWM_OUTPUT0));      

    encoder1_value_prev = encoder1_value;
  }

  encoder1_button_value = encoder1_button.isPressed();

  // TODO: Holding button down for a specific period of time changes encoder mode
  // TODO: Needs debouncing
  if(encoder1_button.stateChanged()) {
    if(encoder1_button_value) { // BUTTON DOWN
      if( GetMemoryMap(MEMMAP_PWM_OUTPUT0) > 0 ) {
        SetMemoryMap(MEMMAP_PWM_OUTPUT0, 0);
      }
      else {
        SetMemoryMap(MEMMAP_PWM_OUTPUT0, 100);
      }
      broadcastPWMValues(); // TODO: Remove this when SetMemoryMap makes the broadcast
    }
    else { // BUTTON RELEASED
    }
  }

}

void returnAllInterfaces(void) {
  for (unsigned int i = 0; i < ( sizeof(interface) / sizeof(struct Interface) ); i++) {
    serialize(MSG_RETURN_INTERFACE, "ibBbSS", interface[i].address, interface[i].bytes, interface[i].exponent, interface[i].access_mask, (const __FlashStringHelper *) interface[i].name, (const __FlashStringHelper *) interface[i].unit);
  }
}

bool returnInterface(uint16_t address) {
  for (unsigned int i = 0; i < ( sizeof(interface) / sizeof(struct Interface) ); i++) {
    if(interface[i].address == address) {
      serialize(MSG_RETURN_INTERFACE, "ibBbSS", interface[i].address, interface[i].bytes, interface[i].exponent, interface[i].access_mask, (const __FlashStringHelper *) interface[i].name, (const __FlashStringHelper *) interface[i].unit);
      return true;
    }
  }

  serialize(MSG_GET_INTERFACE_ERROR, "i", address); // Not found if we didn't return already
}

void broadcastPWMValues(void) {
  serialize(MSG_RETURN_8_8, "bb", (uint8_t)0xA0, (uint8_t)GetMemoryMap(0xA0));
}

void broadcastBatteryMonitor(void) {
  // All values are decimal point shifted from float!
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)MEMMAP_BANK0_VOLTS, (uint16_t)GetMemoryMap(MEMMAP_BANK0_VOLTS));
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)MEMMAP_BANK0_AMPS, (uint16_t)GetMemoryMap(MEMMAP_BANK0_AMPS));  
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)MEMMAP_BANK0_AH_LEFT, (uint16_t)GetMemoryMap(MEMMAP_BANK0_AH_LEFT));  
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)MEMMAP_BANK0_SOC, (uint16_t)GetMemoryMap(MEMMAP_BANK0_SOC));
}

 /* TODO: TO BE DEPRECATED! */
void broadcastADCValues(void) {
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB0, (uint16_t)GetMemoryMap(0xB0)); // charger_input_current
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB1, (uint16_t)GetMemoryMap(0xB1)); // load_center_current
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB2, (uint16_t)GetMemoryMap(0xB2)); // vehicle_input_current
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB3, (uint16_t)GetMemoryMap(0xB3)); // inverter_current
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB4, (uint16_t)GetMemoryMap(0xB4)); // house_batt_voltage
  serialize(MSG_RETURN_8_16, "bi", (uint8_t)0xB5, (uint16_t)GetMemoryMap(0xB5)); // vehicle_batt_voltage
}

// @@@TODO: Consider a weighted average, maybe based on the difference
// between current sample and new sample.  If there's a large difference
// then the sample is given significantly more weight in the average.
unsigned int token = 0;
void ReadADCs(void) {
  const int max_count = 12; // sampled at 80ms, (10ms, 1 in 8 times) this gives an averaging window of ~1s

  if (adc_count[token] >= max_count) {
    adc_sum[token] -= ( adc_sum[token] / adc_count[token] );
  }
  else {
    adc_count[token]++;
  }

/*
  switch(token) {
    case ADC_0_01: adc_sum[token] += ads0.readADC_Differential_0_1(); break;
    case ADC_0_23: adc_sum[token] += ads0.readADC_Differential_2_3(); break;
    case ADC_1_01: adc_sum[token] += ads1.readADC_Differential_0_1(); break;
    case ADC_1_23: adc_sum[token] += ads1.readADC_Differential_2_3(); break;
    
    case ADC_2_0: adc_sum[token] += ads2.readADC_SingleEnded(0); break;
    case ADC_2_1: adc_sum[token] += ads2.readADC_SingleEnded(1); break;
    case ADC_2_2: adc_sum[token] += ads2.readADC_SingleEnded(2); break;
    case ADC_2_3: adc_sum[token] += ads2.readADC_SingleEnded(3); break;
  }
*/

  if (token >= 7) {
    token = 0;
  }
  else {
    token++;
  }

}

// TODO: See about improving this to avoid using floating point
uint8_t cie1931_percent_to_byte(uint8_t percent) {
//  double L = percent;
  float L = percent;
  
  if (L <= 8) {
      L = L / 902.3;
  }
  else {
    L = pow( ( (L + 16.0) / 116.0 ), 3 );
    }
  
  return (uint8_t)(L * 255);
}

void SetPWM(uint8_t output_num, uint8_t value) {
  output_value[output_num] = min(value, 100);

  if ( output_num < sizeof(output_value) ) { // Ensure write to array is within bounds

    // Outputs 0-3 are hardware PWM capable, 4-5 need software control
    // TODO: This is platform dependent!  Create a structure that makes porting easier.
    if (output_num <= 3) {
      // cie1931 conversion should be the last step before output.  Everywhere else should deal with 0-100%
      analogWrite(output[output_num], cie1931_percent_to_byte(output_value[output_num]));
      //Serial.println(cie1931_percent_to_byte(output_value[output_num]));
      //Serial.println("\n");
    }
    else if ( (output_num >= 4) && (output_num <= 7) ) {
      digitalWrite( output[output_num], (output_value[output_num] > 0) );
    }
  }
}

uint8_t GetPWM(uint8_t output_num) {
  return output_value[output_num];
}

/*************************************************/
/* SET VIRTUAL MEMORY MAP                        */
/*************************************************/
bool SetMemoryMap(uint16_t address, uint32_t data) {

// TODO: Set should be followed by publishing the new data.  Everywhere else this has been done needs removed.

  switch (address) {
    // Switches
    case MEMMAP_SETTING_BROADCAST_PERIOD_MS: // Sets the period between data broadcasts.  0xFFFFFFFF disables until another value is set.  Limited to minimum 100ms period.
      inhibit_broadcast = (data == 0xFFFFFFFF);
      broadcast_period_ms = max(100, data);
      broadcast_timer_counter = broadcast_period_ms * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
      return true;

// TODO: Use this to pre-set the internal SOC.  Both are linked and set the other accordingly!
    case MEMMAP_BANK0_AH_LEFT: 
      battery_monitor_var[0].amphours_remaining = (int16_t)data / 10.0;
      battery_monitor_var[0].amphours_remaining = constrain(battery_monitor_var[0].amphours_remaining, 0, battery_monitor_const[0].amphours_capacity);
      battery_monitor_var[0].percent_soc = battery_monitor_var[0].amphours_remaining / battery_monitor_const[0].amphours_capacity * 100;
      return true;
      
    case MEMMAP_BANK0_SOC:
      battery_monitor_var[0].amphours_remaining = ( (int16_t)(data) / 100.0 / 100.0 * battery_monitor_const[0].amphours_capacity );      
      battery_monitor_var[0].amphours_remaining = constrain(battery_monitor_var[0].amphours_remaining, 0, battery_monitor_const[0].amphours_capacity);
      battery_monitor_var[0].percent_soc = battery_monitor_var[0].amphours_remaining / battery_monitor_const[0].amphours_capacity * 100;
      return true;;

    case MEMMAP_BANK0_AMPS_MULTIPLIER:
      battery_monitor_const[0].amps_multiplier = (uint32_t)data * 0.000001;
      EEPROM.put(eeaddr_bank0_amps_multiplier, battery_monitor_const[0].amps_multiplier);
      return true;
      
    case MEMMAP_BANK0_VOLTS_MULTIPLIER:
      battery_monitor_const[0].volts_multiplier = (uint32_t)data * 0.000001;
      EEPROM.put(eeaddr_bank0_volts_multiplier, battery_monitor_const[0].volts_multiplier);
      return true;
      
    case MEMMAP_BANK0_AH_CAPACITY:
      data = max(0.1, data); // must be greater than zero
      battery_monitor_const[0].amphours_capacity = (uint16_t)data *0.1;
      EEPROM.put(eeaddr_bank0_amphours_capacity, battery_monitor_const[0].amphours_capacity);
      return true;
      
    case MEMMAP_BANK0_VOLTS_CHARGED:
      battery_monitor_const[0].volts_charged = (uint16_t)data * 0.001;
      EEPROM.put(eeaddr_bank0_volts_charged, battery_monitor_const[0].volts_charged);
      return true;
      
    case MEMMAP_BANK0_CHRG_DET_TIME:
      data = max(0.1, data); // must be greater than zero
      battery_monitor_const[0].minutes_charged_detection_time = (uint16_t)data * 0.1;
      EEPROM.put(eeaddr_bank0_minutes_charged_detection_time, battery_monitor_const[0].minutes_charged_detection_time);
      return true;
      
    case MEMMAP_BANK0_CURRENT_THRESHOLD:
      battery_monitor_const[0].current_threshold = (uint32_t)data * 0.000001;
      EEPROM.put(eeaddr_bank0_current_threshold, battery_monitor_const[0].current_threshold);
      return true;      

    case MEMMAP_BANK0_TAIL_CURRENT:
      data = max(0.01, data); // must be greater than zero
      battery_monitor_const[0].tail_current_factor = (uint8_t)data * 0.01;
      EEPROM.put(eeaddr_bank0_tail_current_factor, battery_monitor_const[0].tail_current_factor);
      return true;
      
    case MEMMAP_BANK0_PEUKERT_FACTOR:
      data = max(0.01, data); // must be greater than zero
      battery_monitor_const[0].peukert_factor = (uint8_t)data * 0.01;
      EEPROM.put(eeaddr_bank0_peukert_factor, battery_monitor_const[0].peukert_factor);
      return true;
      
    case MEMMAP_BANK0_CHRG_EFFICIENCY:
      data = max(0.01, data); // must be greater than zero
      battery_monitor_const[0].charge_efficiency_factor = (uint8_t)data * 0.01;
      EEPROM.put(eeaddr_bank0_charge_efficiency_factor, battery_monitor_const[0].charge_efficiency_factor);
      return true;      

    // TODO: Add another inhibit for replies to SET commands?  For data that doesn't need acknowledged and speed is paramount.

    // Valid range of address-mapped PWM is 0-100%
    case MEMMAP_PWM_OUTPUT0:
    case MEMMAP_PWM_OUTPUT1:
    case MEMMAP_PWM_OUTPUT2:
    case MEMMAP_PWM_OUTPUT3:
    case MEMMAP_PWM_OUTPUT4:
    case MEMMAP_PWM_OUTPUT5:
    case MEMMAP_PWM_OUTPUT6:
    case MEMMAP_PWM_OUTPUT7:
      SetPWM(address & 0x00000007, data);

      // Other variables linked to outputs - not sure if this is the best way to structure this...
      if( output[address & 0x00000007] == CEILING_LIGHT) { encoder1_value = (uint8_t)data; }
      
      return true;

    // ADC read only inputs
    case MEMMAP_ADC_0_01: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_0_23: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_1_01: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_1_23: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_0: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_1: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_2: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_3: // TODO: TO BE DEPRECATED!
      return false;
  }

  return false;
}

/*************************************************/
/* GET VIRTUAL MEMORY MAP                        */
/*************************************************/
uint32_t GetMemoryMap(uint16_t address) {

  switch (address) {
    case MEMMAP_SETTING_BROADCAST_PERIOD_MS:
      return broadcast_period_ms;

    case MEMMAP_BANK0_VOLTS:              return battery_monitor_var[0].volts * 100;
    case MEMMAP_BANK0_AMPS:               return battery_monitor_var[0].amps * 10;
    case MEMMAP_BANK0_AH_LEFT:            return battery_monitor_var[0].amphours_remaining * 10;
    case MEMMAP_BANK0_SOC:                return battery_monitor_var[0].percent_soc * 100;
    case MEMMAP_BANK0_AMPS_MULTIPLIER:    return battery_monitor_const[0].amps_multiplier * 1000000;
    case MEMMAP_BANK0_VOLTS_MULTIPLIER:   return battery_monitor_const[0].volts_multiplier * 1000000;
    case MEMMAP_BANK0_AH_CAPACITY:        return battery_monitor_const[0].amphours_capacity * 10;
    case MEMMAP_BANK0_VOLTS_CHARGED:      return battery_monitor_const[0].volts_charged * 1000;
    case MEMMAP_BANK0_CHRG_DET_TIME:      return battery_monitor_const[0].minutes_charged_detection_time * 10;
    case MEMMAP_BANK0_TAIL_CURRENT:       return battery_monitor_const[0].tail_current_factor * 100;
    case MEMMAP_BANK0_CURRENT_THRESHOLD:  return battery_monitor_const[0].tail_current_factor * 1000000;
    case MEMMAP_BANK0_PEUKERT_FACTOR:     return battery_monitor_const[0].peukert_factor * 100;
    case MEMMAP_BANK0_CHRG_EFFICIENCY:    return battery_monitor_const[0].charge_efficiency_factor * 100;

// No second bank to test with!
//    case MEMMAP_BANK1_VOLTS: return (uint32_t)battery_monitor_var[1].volts; // float
//    case MEMMAP_BANK1_AMPS: return (uint32_t)battery_monitor_var[1].amps; // float;
//    case MEMMAP_BANK1_AMPHOURS: return (uint32_t)battery_monitor_var[1].amphours_consumed; // float;
//    case MEMMAP_BANK1_SOC: return (uint32_t)battery_monitor_var[1].percent_soc; // float;

    case MEMMAP_PWM_OUTPUT0:
    case MEMMAP_PWM_OUTPUT1:
    case MEMMAP_PWM_OUTPUT2:
    case MEMMAP_PWM_OUTPUT3:
    case MEMMAP_PWM_OUTPUT4:
    case MEMMAP_PWM_OUTPUT5:
    case MEMMAP_PWM_OUTPUT6:
    case MEMMAP_PWM_OUTPUT7:
      return GetPWM(address & 0x0007);

    case MEMMAP_ADC_0_01: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_0_23: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_1_01: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_1_23: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_0: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_1: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_2: // TODO: TO BE DEPRECATED!
    case MEMMAP_ADC_2_3: // TODO: TO BE DEPRECATED!
      return (uint16_t)(adc_sum[address & 0x0007] / adc_count[address & 0x0007]);
  }

  return 0;
}

/* ----- UNTESTED -----
//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
*/

// Little endian (LSB first)
// whitespace in format string is allowed/ignored, can help with readability
// double-quote and backslash in string must be escaped - use C convention " \" "," \\ "
// format characters include:
//    b: unsigned 8-bit whole number
//    B: signed 8-bit whole number
//    i: unsigned 16-bit whole number
//    I: signed 16-bit whole number
//    t: unsigned 24-bit whole number
//    T: signed 24-bit whole number
//    l: unsigned 32-bit whole number
//    L: signed 32-bit whole number
//    S: pointer to string (const __FlashStringHelper *) 
//    s: pointer to string (const char *)

// TODO: Returns false on error - unused?
bool serialize(uint16_t address, const char *fmt, ...) {
  va_list args;
  uint16_t i;
  uint16_t count;
  uint64_t value;
  //uint8_t checksum;
  const char *p = NULL;

  i = 0;
  count = 0;
  value = 0;
 
  while(i < strlen(fmt)) {
    
    if(!isspace(fmt[i])) {
      count += (fmt[i] - (uint8_t)('0'));
    }
      
    if (count > 255) {
      return false; // Fail on message with a payload size greater than 256
    }
    
    i++;
  }

  Serial.write(0x02); // Output start of frame character

  //Serial.write((uint8_t)count); // Count is size of payload not including size, address and checksum

  // Output 16-bit address/id
  Serial.print( (unsigned uint16_t)(address), HEX ); // Won't output any leading zeros, but this should be ok here

  if(count > 0) {
    Serial.print(':');
  }

  // Output variable amount of payload bytes based on format specifiers
  va_start(args, fmt);
  while (*fmt != '\0') {

    if ( isspace(*fmt) ) {
      fmt++;
    }

    //DEBUG//Serial.print("'"); Serial.print(*fmt); Serial.print("'");
    
      switch(*fmt) {

//        case 'd':
//          ieee754_double.d = va_arg(args, double);
//          value = ieee754_double.ui64;
//        break;
//
//        // floats are converted back from the automatic double promotion
//        case 'f':
//          ieee754_double.d = va_arg(args, double);
//          value = 0x00000000FFFFFFFF & (ieee754_double.ui64 >> 32);
//        break;

        case 'l':
          value = va_arg(args, uint32_t);
        break;

        case 'L':
          value = va_arg(args, int32_t);
        break;

        case 't':
          value = va_arg(args, uint32_t);
        break;
  
        case 'T':
          value = va_arg(args, int32_t);
        break;
  
        case 'i':
          value = va_arg(args, uint16_t);
        break;
        
        case 'I':
          value = va_arg(args, int16_t);
        break;
        
        case 'b':
          value = va_arg(args, uint16_t);
        break;

        case 'B':
          value = va_arg(args, int16_t);
        break;

        case 'S':
          p = (const char *)va_arg(args, const __FlashStringHelper * );
        break;
        
        case 's':
          p = va_arg(args, const char * );
        break;
      }

    //DEBUG//Serial.print("<"); Serial.print((uint32_t)value, HEX); Serial.println(">");
  
    switch(*fmt) {
//      case 'd': 
//        Serial.print( (unsigned uint8_t)(value >> 56), HEX );
//        Serial.print( (unsigned uint8_t)(value >> 48), HEX );
//        Serial.print( (unsigned uint8_t)(value >> 40), HEX );
//        Serial.print( (unsigned uint8_t)(value >> 32), HEX );
//        // FALL THROUGH
//      case 'f': 

      // TODO: This has a lot of potential for optimization, but it works for now and we aren't running out of space or time.
      case 'l': 
      case 'L':
        Serial.print( (unsigned uint8_t)(value >> 28 ) & 0x0F, HEX ); // Print by nybble because HEX doesn't output leading '0'
        Serial.print( (unsigned uint8_t)(value >> 24 ) & 0x0F, HEX );
        // FALL THROUGH
      case 't': 
      case 'T': 
        Serial.print( (unsigned uint8_t)(value >> 20 ) & 0x0F, HEX );
        Serial.print( (unsigned uint8_t)(value >> 16 ) & 0x0F, HEX );
        // FALL THROUGH
      case 'i': 
      case 'I': 
        Serial.print( (unsigned uint8_t)(value >> 12 ) & 0x0F, HEX );
        Serial.print( (unsigned uint8_t)(value >> 8  ) & 0x0F, HEX );
        // FALL THROUGH
      case 'b': 
      case 'B': 
        Serial.print( (unsigned uint8_t)(value >> 4  ) & 0x0F, HEX );
        Serial.print( (unsigned uint8_t)(value >> 0  ) & 0x0F, HEX );
      break;      

      case 'S':
        // TODO: Strip out characters from the string that will cause problems (unescaped?, comma, colon?)
        Serial.write('"'); Serial.print( (const __FlashStringHelper *) p ); Serial.write('"');
      break;

      case 's': 
        // TODO: Strip out characters from the string that will cause problems (unescaped?, comma, colon?)
        Serial.write('"'); Serial.print(*p); Serial.write('"');
      break;

    }

    fmt++;

    if (*fmt != '\0') {
      Serial.print(',');
    }
    
  }
  va_end(args);

  // Output checksum
  // TODO
  //checksum = 0xEE;
  //Serial.write( (unsigned uint8_t)(checksum >>  0) );

  //Serial.println();

  Serial.write(0x03); // Output end of frame character

  Serial.println(); // This should be ignored by the receiver and is only here to help with semi-readable output
  
  return true;
}

unsigned int asciiHexToInt(char ch) {
  unsigned int num = 0;

  if( (ch >= '0') && (ch <= '9') ) {
    num = ch - '0';
  }
  else {
    switch(ch) {
      case 'A': case 'a': num = 10; break;
      case 'B': case 'b': num = 11; break;
      case 'C': case 'c': num = 12; break;
      case 'D': case 'd': num = 13; break;
      case 'E': case 'e': num = 14; break;
      case 'F': case 'f': num = 15; break;
      default: num = 0;
    }
  }
  
  return num;
}

//char intToAsciiHex(uint8_t num) {
//  char c = '';
//
//  if( (num >= 0) && (num <= 9) ) {
//    c = num + '0';
//  }
//  else {
//    switch(ch) {
//      case 10: c = 'A'; break;
//      case 11: c = 'B'; break;
//      case 12: c = 'C'; break;
//      case 13: c = 'D'; break;
//      case 14: c = 'E'; break;
//      case 15: c = 'F'; break;
//    }
//  }
//
//  return c;
//}

// TODO: Not working
//void consumeWhitespace(char **p) {
//  while ( isspace(*p) && (*p != '\0') ) {
//    Serial.print('{'); Serial.print(*p); Serial.print('}');
//    *p++;
//  }
//}

// NO WHITESPACE IS ALLOWED IN THE MESSAGE AT THIS TIME
void parse_message(char *msg_buf) {
  char *p = msg_buf;
  uint8_t count = 0;
  uint16_t address = 0;
  uint32_t value_buffer = 0;
  bool invalid_message = false; // TODO: Replace with descriptive error codes later?

  // LIMITED EMBEDDED IMPLEMENTATION
  //   One address and up to four 32-bit parameters will be accepted.  The remainder will be discarded silently.
  uint32_t arg[4] = {0};
  uint8_t arg_count = 0;

  // Message to be composed of only PRINTABLE ASCII - isprint() !
  // Message format is addr:XX, XX, ...
  //   Where a 4-character ASCII-HEX is used for an address
  //   ... followed by optional colon
  //   ... followed by a comma-separated list of ASCII-HEX or STRING values of variable length
  //   ... string values have an additional marker character pre-pended TBD

  //DEBUG//Serial.print("\""); Serial.print(msg_buf); Serial.println("\""); 

  //consumeWhitespace(&p); // TODO: Not working

  // Empty message is invalid.  Parsing loop will be skipped.
  if(*p == '\0') {
    invalid_message = true;
  }

  while ( (*p != '\0') && (!invalid_message) ) {
    // Get up to 4 ASCII-HEX characters for address
    address = 0;
    count = 0;

    if( !isxdigit(*p) ) {
      // TODO - ERROR: Address did not start with hex digit
      invalid_message = true;
      break;
    }
    
    //while ( (*p != '\0') && (*p != ':') && isxdigit(*p) ) {
    while ( isxdigit(*p) ) {
      address <<= 4;
      address += asciiHexToInt(*p);

      count++;
      p++;
    }

    //DEBUG//Serial.print('['); Serial.print(address, HEX); Serial.print(']');
    
    if (count > 4) {
      // TODO: ERROR - address must maximum 4 ASCII-HEX characters
      invalid_message = true;
      break;
    }

    // Colon (continue), end of string (done), else (error)
    if(*p == ':') {
      p++; // Consume colon

      // Get list following colon
      arg_count = 0;
      do {
        if(isxdigit(*p)) {          
          // Get ascii-hex digits up to comma or end of string
          count = 0;
          value_buffer = 0;
          //while ( (*p != '\n') && (*p != ',') && isxdigit(*p) && (count < 8) ) {
          while ( isxdigit(*p) ) {
            value_buffer <<= 4;
            value_buffer += asciiHexToInt(*p);

//            Serial.print("*p = '"); Serial.print(*p); Serial.println("'");
//            Serial.print("value_buffer = "); Serial.println(value_buffer, HEX); //DEBUG//

            count++;
            p++;
          }

          if (count > 8) {
            // TODO: ERROR - max data size is 4 bytes or 8 ascii-hex characters
            invalid_message = true;
            break;
          }
          else {
            // Looks like we have good data.  Do something with it before it goes away in the next loop iteration.
            //DEBUG//Serial.print(value_buffer, HEX);
            arg[arg_count] = value_buffer;
            arg_count++;

            if( arg_count > ( sizeof(arg) / sizeof(arg[0]) ) ) {
              // TODO - ERROR: IMPOSED EMBEDDED LIMITATION
              invalid_message = true;
              break;
            }
          }

          // If we're looking at a the comma following a value, consume it.
          if (*p == ',') {
            p++;
            //DEBUG//Serial.print(';');
          }
          else if (*p != '\0') {
            // TODO: ERROR - Only a comma or end of string should follow a value
            invalid_message = true;
            break;
          }
        }
//        else if (*p == '"') {
//          // get everything up to comma or end of string
//        }
        else {
          // TODO: ERROR - invalid contents
          invalid_message = true;
          break;
        }
      } while (*p != '\0');

      if(invalid_message) {
        // Something went wrong in the above parsing loop getting the list.  Get out of the outer loop as well.
        break;
      }

    }
    else if (*p != '\0') {
      // TODO - ERROR: Something unexpected followed the address
      invalid_message = true;
      break;
    }

    //p += sizeof(char);
    p++;
  }

  if ( !invalid_message ) {
    // If no errors were signalled from above, pack everything up and/or execute whatever based on the parsed message
    //DEBUG//Serial.println("  <-- VALID");

    // v v v v v v v v v v v   Serial Message Processing   v v v v v v v v v v v 

    switch(address) {

      case MSG_KEEP_ALIVE: // Sent by system master (or designate) to ensure bus is operating.  This module will be automatically reset by the watchdog timer if not received in time.
        if(arg_count == 0) {
          wdt_reset();
          serialize(MSG_KEEP_ALIVE, "b", 0);
        }
        break;

      case MSG_SET_8_8:
        if (arg_count == 2) {
          if (SetMemoryMap((uint8_t)arg[0], (uint8_t)arg[1]) ) {
            serialize(MSG_RETURN_8_8, "bb", (uint8_t)arg[0], (uint8_t)GetMemoryMap(arg[0]));
          }
          else {
            serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
          }
          
        }
        break;

      case MSG_GET_8_8:
        if (arg_count == 1) {
          serialize(MSG_RETURN_8_8, "bb", (uint8_t)arg[0], (uint8_t)GetMemoryMap(arg[0]));
        }
        else {
          serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
        }
        break;

      case MSG_SET_8_16:
        if (arg_count == 2) {
          if (SetMemoryMap((uint8_t)arg[0], (uint16_t)arg[1]) ) {
            serialize(MSG_RETURN_8_16, "bi", (uint8_t)arg[0], (uint16_t)GetMemoryMap(arg[0]));
          }
          else {
            serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
          }
        }
        break;

      case MSG_GET_8_16:
        if (arg_count == 1) {
          serialize(MSG_RETURN_8_16, "bi", (uint8_t)arg[0], (uint16_t)GetMemoryMap(arg[0]));
        }
        else {
          serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
        }
        break;

      case MSG_GET_8_32:
        if (arg_count == 1) {
          serialize(MSG_RETURN_8_32, "bl", (uint8_t)arg[0], (uint32_t)GetMemoryMap(arg[0]));
        }
        else {
          serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
        }
        break;

      case MSG_SET_8_32:
        if (arg_count == 2) {
//          Serial.print("arg[1] = "); Serial.println(arg[1], HEX); //DEBUG//
          if (SetMemoryMap((uint8_t)arg[0], (uint32_t)arg[1]) ) {
            serialize(MSG_RETURN_8_32, "bl", (uint8_t)arg[0], (uint32_t)GetMemoryMap(arg[0]));
          }
          else {
            serialize(MSG_GET_SET_ERROR, "b", (uint8_t)arg[0]);
          }
        }
        break;

        case MSG_GET_INTERFACE:
          if (arg_count == 0) {
            returnAllInterfaces();
          }
          else if (arg_count == 1) {
            returnInterface((uint16_t)arg[0]);
          }
          else {
              serialize(MSG_GET_INTERFACE_ERROR, "");
          }
        break;
    }
        
    // ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^  (Serial Message Processing)  ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
    
  }
  else {
    //DEBUG//Serial.println("  <-- INVALID");
    // Quietly ignore invalid messages until there's a reason not to
  }
}

void processSerialReceive(void) {
  static enum ReceiveState receive_state = GET_STX;
  static char message_buffer[32] = {0}; // KEEP INCOMING MESSAGES SMALL!

  char c;
    
  if(Serial.available()) {
    switch(receive_state) {
      case GET_STX:
        if( (c = Serial.read()) == '\x02' ) {
          ////Serial.print("<STX>");
          strncpy(message_buffer, "", sizeof(message_buffer));
          receive_state = GET_DATA;
        }
      break;
  
      case GET_DATA:
        c = Serial.read();
  
        if( isprint(c) ) {
          // Valid data, save/buffer it for later
          ////Serial.print(c);
          strncat(message_buffer, &c, 1);
        }
        else if( c == '\x03' ) {
          ////Serial.println("<ETX>");
          parse_message(message_buffer);
          receive_state = GET_STX;
        }
        else if( c == '\x02' ) {
          // ERROR - expected ETX before STX
          ////Serial.print("   <<<   MISSING ETX\r\n<RECOVER-STX>");
  
          // TODO: Clear any buffered data
          // TODO: Set things up the same as though we've receiv$
  
          strncpy(message_buffer, "", sizeof(message_buffer));
        }
      break;

    }
  }
}

////////////////////////// INTERRUPTS ///////////////////////////////
/* Any variables changed in interrupts must be declared 'volatile' */

void handleTimerTick(void) {
  base_timer_flag = true;

  timer_counter_3--;
  if (timer_counter_3 == 0) {
    quarter_second_flag = true;
    timer_counter_3 = (250) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
  }

  timer_counter_2--;
  if (timer_counter_2 == 0) {
    ten_millisecond_flag = true;
    timer_counter_2 = (10) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
  }

  timer_counter_1--;
  if (timer_counter_1 == 0) {
    one_second_flag = true;
    timer_counter_1 = (1000) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
  }

  broadcast_timer_counter--;
  if (broadcast_timer_counter == 0) {
    broadcast_flag = true;
    broadcast_timer_counter = broadcast_period_ms * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
  }
  
}

ISR(WDT_vect) {
  wdt_isr_executed = true; // Flag the mainline code that this executed so we can re-enable it there
  // Make sure to turn this off when you're sleeping...
  //debugFlashHexCode(ERR_FATAL_WDT_TIMEOUT, false);
  while (1); // Do nothing while waiting for the WDT to reset the system
}

#if defined(ARDUINO_AVR_NANO)
ISR(PCINT1_vect) {
#elif defined(ARDUINO_AVR_LEONARDO)
ISR(PCINT0_vect) {
#else
  #error CODE STUB
#endif  
  uint8_t result;
////  struct EncoderData *encoder_data; // Encoder may be used in different modes
//  int *counter;

  result = encoder1.process();

//  switch (encoder1_mode) {
//    case ENC1MODE_DIMMER:
//      ////*encoder_data = encoder1_data;
//      //*counter = encoder1_value;
//    break;
//
//    case ENC1MODE_NAVIGATE:
//      // TODO
//    break;
//  }
  
  if(result == DIR_CW) {
    ////encoder_data->counter += ENCODER1_STEP;
    //*counter += ENCODER1_STEP;
    encoder1_value += ENCODER1_STEP;
  }
  else if(result == DIR_CCW) {
    ////encoder_data->counter -= ENCODER1_STEP;
    //*counter -= ENCODER1_STEP;
    encoder1_value -= ENCODER1_STEP;
  }

  ////encoder_data->counter = constrain(encoder_data->counter, encoder_data->min, encoder_data->max);
  //*counter = constrain(*counter, 0, 100);
  encoder1_value = constrain(encoder1_value, 0, 100);
  
}
