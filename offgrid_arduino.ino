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
#include <INA226.h> // https://github.com/peterus/INA226Lib /* Consider replacing this with a library that uses all integer math if precision is a problem */

#include "shared_constants.h"
#include "internal_constants.h"

// See avr/iom328p.h for register & bit definitions

// Look up table for converting PWM OUTPUT ID to actual output
const byte output[] = {OUTPUT0, OUTPUT1, OUTPUT2};
byte output_value[] = {0, 0, 0}; // TODO: read/write should only be done through SetPWM/GetPWM.  To be refactored into a class at some point.

enum ReceiveState {
    GET_STX = 1,
    GET_DATA = 2,
};

enum Encoder1Mode {
    ENC1MODE_DIMMER,
    ENC1MODE_NAVIGATE,
} encoder1_mode;

enum HVACState {
    HS_ERROR,
    HS_OFF,
    HS_IN_RANGE,
    HS_HEAT,
    HS_COOL,
};

enum HVACReqState {
    HVAC_RQ_OFF,
    HVAC_RQ_AUTO,
    HVAC_RQ_OVERRIDE,
    HVAC_RQ_TIMER,
};

enum PropexFurnaceErrorCode {
    PFEC_NONE,
    PFEC_FLAME,
    PFEC_OVERHEAT,
    PFEC_VOLTAGE,
    PFEC_AIR
};

// Variables related to status code LED and flashing
volatile bool flash_led = false;
volatile uint8_t flash_countdown;
volatile uint8_t error_code = 0;
volatile uint32_t flash_ticks;
volatile uint32_t error_code_ticks;

volatile bool base_timer_flag = false;

// TODO: Use constants for these preset/init values, as they are used in more than one place.

volatile bool one_second_flag = 0;
volatile uint16_t timer_counter_1 = (1000) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool quarter_second_flag = 0;
volatile uint16_t timer_counter_3 = (250) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool ten_millisecond_flag = 0;
volatile uint16_t timer_counter_2 = (10) * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);

volatile bool wdt_isr_executed = false;

Rotary encoder1 = Rotary(ENCODER1_CHA_PIN, ENCODER1_CHB_PIN);
Button encoder1_button = Button(ENCODER1_BTN_PIN, PULLUP);
volatile int encoder1_value = 0;
int encoder1_value_prev = 0;
bool encoder1_button_value = 0;

// Encoder switch debouncing
//unsigned long encoder1_last_change_ms = 0;
//bool encoder1_button_state_prev = 0;
//bool encoder1_button_state_curr = 0;

// TODO: Some of these are full battery monitors, others are just measuring voltage and current on shunts! Separate the two!
// Battery Monitoring (INA226 modules)
const byte ina226_addr[] = {0x40, 0x41, 0x44, 0x45}; // TODO: Organize these better. Specific object per shunt, set defaults safely
INA226 ina226[sizeof(ina226_addr)];
struct BMConst battery_monitor_const[sizeof(ina226_addr)];
struct BMVar battery_monitor_var[sizeof(ina226_addr)];

float solar_amps, solar_volts;
float inverter_amps, inverter_volts;
float vehicle_amps, vehicle_volts;

enum HVACState hvac_state = HS_IN_RANGE; // TODO: This should start at HS_OFF and be changed when the mode is changed to something else
uint8_t hvac_error_code = 0;
uint8_t hvac_heat_on = 0;
float hvac_desired_temperature = 16.5;
const float HEAT_TURN_ON_MARGIN = 0.75;
const float HEAT_TURN_OFF_MARGIN = 0.75;
const float MIN_DESIRED_TEMPERATURE = 8.0;
const float MAX_DESIRED_TEMPERATURE = 35.0;
float hvac_temperature_ceiling = 0;
float hvac_temperature_floor = 0;

bool inhibit_broadcast = false;
uint32_t broadcast_period_ms = 1000;
volatile uint32_t broadcast_timer_counter = broadcast_period_ms * (1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001));
volatile bool broadcast_flag = 0;

void setup() {

    Serial.begin(115200);

    while (!Serial); // This appears to be needed for native USB ports (

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

    Timer1.start();
    //setupWatchdogTimer(); // Masks a problem where this stops responding on the CAN bus

    initHVAC();
    initEncoders();
    initEnergyMonitors();

    interrupts();

    serialize(MSG_POWER_ON, "");
    returnAllInterfaces(); // As a convenience, do this on startup/reset so that connected device doesn't have to request it

//    serialize(MSG_DEBUG_STRING, "s", "Debugging interface active"); /* DEBUG */

//    serialize(MSG_DEBUG_STRING, "s", "TURNING HEAT ON IN 15 SECONDS"); /* DEBUG */
//    delay(15000);
//    digitalWrite(HVAC_REQ_HEAT, 1);
//    serialize(MSG_DEBUG_STRING, "s", "TESTING - HEAT ON"); /* DEBUG */
}

void loop() {
    processSerialReceive();
    processEncoders();

    if (base_timer_flag) {
        base_timer_flag = false;

    }

    if(broadcast_flag) {
        broadcast_flag = false;

        if(!inhibit_broadcast) {
            broadcastPWMValues();
            broadcastEnergyMonitors();
            broadcastHVAC();
            //broadcastDebug(); /* DEBUG */
        }
    }

    if (ten_millisecond_flag) {
        ten_millisecond_flag = false;

        processEncoderLEDState();
        processHVACInputs();
    }

    if (quarter_second_flag) {
        quarter_second_flag = false;

    }

    if (one_second_flag) {
        one_second_flag = false;

        processEnergyMonitors();
        processHVAC();
    }

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

void initEnergyMonitors(void) {

    // Set up INA226 shunt monitor modules
    for (uint8_t i = 0; i < sizeof(ina226_addr); i++) {
        ina226[i].begin(ina226_addr[i]);
        ina226[i].configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_4156US, INA226_MODE_SHUNT_BUS_CONT);
    }

    // Each one needs to be calibrated diffferently due to the unique shunt voltage and resistance
    // TODO: Need to organize diffferently for better code maintenance and dynamic addition of shunts!

    // First time requires that the EEPROM be pre-programmed.  See offgrid_init_eeprom.ino
    EEPROM.get(0, battery_monitor_const); // Copy semi-constants from EEPROM to battery monitor structures

    // TODO: Fetch these from EEPROM, make programmable through MQTT interface
    ina226[0].calibrate(0.0001f, 200); // Battery Bank 0 (House) shunt
    ina226[1].calibrate(0.0015f, 50); // Solar Shunt
    ina226[2].calibrate(0.000375f, 200); // Inverter Shunt
    ina226[3].calibrate(0.000375f, 200); // Battery Bank 1 (Vehicle) shunt

    //for (unsigned int i = 0; i < ( sizeof(battery_monitor_var) / sizeof(battery_monitor_var[0]) ); i++) {
    for (unsigned int i = 0; i < 1; i++) {
        // At startup/reset we can assume nothing about these until a full charge synchronization.
        battery_monitor_var[i].amphours_remaining = battery_monitor_const[i].amphours_capacity;
        battery_monitor_var[i].charge_state = CS_NONE;
    }
}

void initHVAC(void) {
    pinMode(HVAC_ERR_IN, INPUT_PULLUP); // Active low
    pinMode(HVAC_HEAT_ON_IN, INPUT_PULLUP); // Active low
    pinMode(HVAC_REQ_FAN, OUTPUT); // Active high
    pinMode(HVAC_REQ_HEAT, OUTPUT); // Active high
//    pinMode(HVAC_REQ_COOL, OUTPUT); // Active high

    digitalWrite(HVAC_REQ_FAN, 0);
    digitalWrite(HVAC_REQ_HEAT, 0);

    //hvac_state = HS_OFF;
    // TODO: Initialize setpoints from values stored in EEPROM
}

// NOTE: This must be executed frequently enough to distinguish 500ms half-period pulses, eg. every 10ms.
void processHVACInputs(void) {
    static bool reading_pulses = false;
    static long last_falling_edge;
    static unsigned int pulse_count;
    static unsigned int last_state = HIGH;
    static unsigned int curr_state = HIGH;;

    curr_state = digitalRead(HVAC_ERR_IN);
    hvac_heat_on = !digitalRead(HVAC_HEAT_ON_IN);

    if (!reading_pulses) { // Beginning of error pulse train not yet detected

        if ((curr_state == LOW) /*&& (last_state == HIGH)*/ ) { // Start of pulse train detected
            //serialize(MSG_DEBUG_STRING, "s", "Start of pulse train detected"); /* DEBUG */
            pulse_count = 0;
            last_falling_edge = millis();
            reading_pulses = true;
        }
        else { // Line idle while not receiving pulse train
            if ( (unsigned long)(millis() - last_falling_edge) >= 5000 ) { // Line idle for more than 5 sec resets error code
                hvac_error_code = 0;
            }
        }
    }
    else { // we're somewhere in the middle of an error pulse train
        if ((curr_state == LOW) && (last_state == HIGH)) { // Beginning of next pulse detected
            last_falling_edge = millis();
        }
        else if ((curr_state == HIGH) && (last_state == LOW)) { // End of pulse, low period should be close to 500ms
            if ( ( (unsigned long)(millis() - last_falling_edge) > 475 ) &&
                 ( (unsigned long)(millis() - last_falling_edge) < 525 ) ) {
                 pulse_count++;
            }
        }
        else if ((curr_state == HIGH) && (last_state == HIGH)) { // Line idle for >= 2 sec indicates end of pulse train
            if ( (unsigned long)(millis() - last_falling_edge) >= 2000 ) {
                reading_pulses = false;
                hvac_error_code = pulse_count;
            }
        }
    }

    last_state = curr_state;

    if(hvac_error_code != 0) {
        hvac_state = HS_ERROR;
    }
}

void processHVAC(void) {
    const unsigned short MAX_ERROR_RESET_RETRY = 3;
    char errstr[128];

    static unsigned short err_count = 0;
    static bool heat_reset_in_progress = false;
    static bool hvac_error_message_sent = false;

//    if(heat_reset_in_progress) {
//        return; // Can't do anything else with the heater while we're trying to reset it!
//    }

    // TODO: Implement programmable schedule (maybe the scheduler resides in the Rpi??)

    switch (hvac_state) {

        case HS_ERROR:

            if (hvac_error_code == 0) {
                hvac_state = HS_IN_RANGE;
                hvac_error_message_sent = false;

                serialize(MSG_DEBUG_STRING, "HVAC error cleared", errstr);
            }
            else if (!hvac_error_message_sent) {

                switch (hvac_error_code) {
                    case PFEC_FLAME:
                        sprintf(errstr, "HVAC Error: Flame failure. Check fuel level.");
                    break;

                    case PFEC_OVERHEAT:
                        sprintf(errstr, "HVAC Error: Internal maximum temperature exceeded.");
                    break;

                    case PFEC_VOLTAGE:
                        sprintf(errstr, "HVAC Error: Supply voltage not within acceptable range.");
                    break;

                    case PFEC_AIR:
                        sprintf(errstr, "HVAC Error: Combustion air flow unbalanced. Check for blockage or excess wind.");
                    break;

                    default:
                        sprintf(errstr, "HVAC Error: Undefined [%d] - Contact manufacturer.", hvac_error_code);
                    break;
                }

                // TODO: Add dedicated message for error message reporting
                serialize(MSG_DEBUG_STRING, "s", errstr);
                hvac_error_message_sent = true;
            }


            // TODO: Check for and handle any errors indicated by heater
//          if (err = check_hvac_error()) {
//              err_count++;
//              // TODO: Send a message indicating what the error message indicates, plus how many have been logged so far
//              // TODO: resetHVACError() needs to be non-blocking, but we also need to wait for the reset process to complete before doing anything else with the heater
//              if(resetHVACError(&heat_reset_in_progress) && (err_count < MAX_ERROR_RESET_RETRY)) {
//                  err_count = 0; // If the reset succeeded, zero the error counter
//              }
//          else {
//              hvac_state = HS_ERROR; // No further attempts will be made to reset the error. Manual intervention required!
//          }
//      }
        break;

        case HS_IN_RANGE:
            if ( hvac_temperature_ceiling < (hvac_desired_temperature - HEAT_TURN_ON_MARGIN) ) {
                hvac_state = HS_HEAT;
                digitalWrite(HVAC_REQ_HEAT, 1);
            }
            // TODO: Make sure that heating and cooling overshoot don't activate the other mode
            //else if ( hvac_temperature_ceiling >= (hvac_desired_temperature - HEAT_TURN_OFF_MARGIN) ) {
            //    hvac_state = HS_COOL;
            //    //digitalWrite(HVAC_REQ_COOL, 1); // TODO: Enable when hardware is added
            //}
        break;

        case HS_HEAT:

            // TODO: Start a one-shot timer to test whether the HEAT_ON signal from furnace has been activated within the maximum allowable time. If it hasn't, transition to error state.

            if ( hvac_temperature_ceiling > (hvac_desired_temperature + HEAT_TURN_OFF_MARGIN) ) {
                hvac_state = HS_IN_RANGE;
                digitalWrite(HVAC_REQ_HEAT, 0);
            }
        break;

        //case HS_COOL:
        //break;

    }

}

// Returns the number of hours to full charge or full discharge at present current gain/loss
float CalcTimeToGo(int bank_number)
{
    float ttg;

    if (battery_monitor_var[bank_number].amps > 0) { // Charging
        ttg = (battery_monitor_const[bank_number].amphours_capacity - battery_monitor_var[bank_number].amphours_remaining) / battery_monitor_var[bank_number].amps;
    }
    else if (battery_monitor_var[bank_number].amps < 0) { // Discharging
        ttg = battery_monitor_var[bank_number].amphours_remaining / battery_monitor_var[bank_number].amps;
    }
    else { // No current flow
        ttg = 0; // Should really be infinite, but that would be more difficult to convey
    }

    return ttg;
}

void processEnergyMonitors(void) {

    battery_monitor_var[0].volts = ina226[0].readBusVoltage();
    battery_monitor_var[0].amps = ina226[0].readShuntCurrent();
    battery_monitor_var[0].amphours_remaining += 1.0 / 3600.0 * battery_monitor_var[0].amps; // Calculation assumes execution every 1 second!
    battery_monitor_var[0].amphours_remaining = constrain(battery_monitor_var[0].amphours_remaining, 0, battery_monitor_const[0].amphours_capacity);

    solar_volts = ina226[1].readBusVoltage();
    solar_amps = ina226[1].readShuntCurrent();

    inverter_volts = ina226[2].readBusVoltage();
    inverter_amps = ina226[2].readShuntCurrent();

    vehicle_volts = ina226[3].readBusVoltage();
    vehicle_amps = ina226[3].readShuntCurrent();

//  for (int i = 0; i < ( sizeof(battery_monitor_var) / sizeof(battery_monitor_var[0]) ); i++) {
    for (int i = 0; i < 1; i++) {

        switch(battery_monitor_var[i].charge_state) {

        case CS_NONE:
            if(battery_monitor_var[i].amps > 0) {
                battery_monitor_var[i].charge_state = CS_CHARGING;
            }
            else if(battery_monitor_var[i].amps < 0) {
                battery_monitor_var[i].charge_state = CS_DISCHARGING;
            }
            break;

        case CS_DISCHARGING:
            if(battery_monitor_var[i].amps > 0) {
                battery_monitor_var[i].charge_state = CS_CHARGING;
            }
            else if(battery_monitor_var[i].amps == 0) {
                battery_monitor_var[i].charge_state = CS_NONE;
            }
            break;

        case CS_CHARGING:
            if(battery_monitor_var[i].amps < 0) {
                battery_monitor_var[i].charge_state = CS_DISCHARGING;
            }
            else if(battery_monitor_var[i].amps == 0) {
                battery_monitor_var[i].charge_state = CS_NONE;
            }

            // Check for definition of "charged" and trigger countdown to fully charged state
            if ( battery_monitor_var[i].volts >= battery_monitor_const[i].volts_charged &&
                    battery_monitor_var[i].amps <= battery_monitor_const[i].amphours_capacity * battery_monitor_const[i].tail_current_factor ) {
                battery_monitor_var[i].charge_state = CS_CHARGED_PENDING;
                battery_monitor_var[i].charged_pending_timestamp = millis();
            }
            break;

        case CS_CHARGED_PENDING:
            // Check for conditions that no longer satisfy the definition of being "charged" and leave this state if true
            if ( battery_monitor_var[i].volts < battery_monitor_const[i].volts_charged ||
                    battery_monitor_var[i].amps > battery_monitor_const[i].amphours_capacity * battery_monitor_const[i].tail_current_factor ) {
                battery_monitor_var[i].charge_state = CS_NONE;
            }
            // Conditions were maintained for the specified time so we call it "charged"
            else if ( (unsigned long)(millis() - battery_monitor_var[i].charged_pending_timestamp) >= MinutesToMilliseconds(battery_monitor_const[i].minutes_charged_detection_time) ) {
                battery_monitor_var[i].charge_state = CS_CHARGED;
                battery_monitor_var[i].amphours_remaining = battery_monitor_const[i].amphours_capacity; // SYNC SOC
            }

            break;

        case CS_CHARGED:
            if(battery_monitor_var[i].amps < 0) {
                battery_monitor_var[i].charge_state = CS_DISCHARGING;
            }
            break;
        }
    }
}

void broadcastEnergyMonitors(void) {
    // While it might seem like power (watts) is redundant, it has been added because linking together volts * amps after the fact for visualization hasn't been a simple task.

    // All values are decimal point shifted from float!
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_VOLTS, (int16_t)GetMemoryMap(MEMMAP_BANK0_VOLTS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_AMPS, (int16_t)GetMemoryMap(MEMMAP_BANK0_AMPS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_WATTS, (int16_t)GetMemoryMap(MEMMAP_BANK0_WATTS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_AH_LEFT, (int16_t)GetMemoryMap(MEMMAP_BANK0_AH_LEFT));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_SOC, (int16_t)GetMemoryMap(MEMMAP_BANK0_SOC));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_BANK0_TTG, (int16_t)GetMemoryMap(MEMMAP_BANK0_TTG));
    serialize(MSG_RETURN_8_8,  "bb", (uint8_t)MEMMAP_BANK0_CS, (int8_t)GetMemoryMap(MEMMAP_BANK0_CS));

    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_SOLAR_VOLTS, (int16_t)GetMemoryMap(MEMMAP_SOLAR_VOLTS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_SOLAR_AMPS, (int16_t)GetMemoryMap(MEMMAP_SOLAR_AMPS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_SOLAR_WATTS, (int16_t)GetMemoryMap(MEMMAP_SOLAR_WATTS));

    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_INVERTER_VOLTS, (int16_t)GetMemoryMap(MEMMAP_INVERTER_VOLTS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_INVERTER_AMPS, (int16_t)GetMemoryMap(MEMMAP_INVERTER_AMPS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_INVERTER_WATTS, (int16_t)GetMemoryMap(MEMMAP_INVERTER_WATTS));

    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_VEHICLE_VOLTS, (int16_t)GetMemoryMap(MEMMAP_VEHICLE_VOLTS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_VEHICLE_AMPS, (int16_t)GetMemoryMap(MEMMAP_VEHICLE_AMPS));
    serialize(MSG_RETURN_8_16, "bI", (uint8_t)MEMMAP_VEHICLE_WATTS, (int16_t)GetMemoryMap(MEMMAP_VEHICLE_WATTS));

}

void broadcastDebug(void) {
    char dbgstr[80];

    for (int i = 0; i < sizeof(ina226_addr); i++) {
        sprintf(dbgstr, "ina226[%d].readShuntVoltage() --> %d", i, ina226[i].readShuntCurrent());
        serialize(MSG_DEBUG_STRING, "s", dbgstr); /* DEBUG */
    }
}

void broadcastHVAC(void) {
    serialize(MSG_RETURN_8_8, "bb", (uint8_t)MEMMAP_HVAC_ERROR, (uint8_t)GetMemoryMap(MEMMAP_HVAC_ERROR));
    serialize(MSG_RETURN_8_8, "bb", (uint8_t)MEMMAP_HVAC_HEAT_ON, (uint8_t)GetMemoryMap(MEMMAP_HVAC_HEAT_ON));
    serialize(MSG_RETURN_8_8, "bb", (uint8_t)MEMMAP_HVAC_STATE, (uint8_t)GetMemoryMap(MEMMAP_HVAC_STATE));
    serialize(MSG_RETURN_8_16, "bI", (int16_t)MEMMAP_HVAC_DESIRED_TEMPERATURE, (int16_t)GetMemoryMap(MEMMAP_HVAC_DESIRED_TEMPERATURE));
}

void processEncoders(void) {
    bool encoder1_button_read;

    // Only do something if the encoder value has actually changed
    if(encoder1_value != encoder1_value_prev) {
        //Serial.println(encoder1_value);
        SetMemoryMap(MEMMAP_PWM_OUTPUT0, encoder1_value);
        serialize(MSG_RETURN_8_8, "bb", (uint8_t)MEMMAP_PWM_OUTPUT0, (uint8_t)GetMemoryMap(MEMMAP_PWM_OUTPUT0));

        encoder1_value_prev = encoder1_value;
    }




    /* TEST NEW CODE
    encoder1_button_read = !digitalRead(ENCODER1_BTN_PIN);

    if(encoder1_button_read != encoder1_button_state_prev) {
        encoder1_last_change_ms = millis();
    }

    if((millis() - encoder1_last_change_ms) > DEBOUNCE_DELAY_MS) {
      if(encoder1_button_read != encoder1_button_state_curr) {
        encoder1_button_state_curr = encoder1_button_read;

        if(encoder1_button_state_curr) { // button down
          SetMemoryMap(MEMMAP_PWM_OUTPUT0, GetMemoryMap(MEMMAP_PWM_OUTPUT0) > 0 ? 0 : 100);
        }

      }

      encoder1_button_state_prev = encoder1_button_read;
    }
    // TEST NEW CODE */





// OLD CODE - Switch not debounced properly
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
// OLD CODE - Switch not debounced properly


}

// TODO: This could be used to signal error codes as well
void processEncoderLEDState(void) {
    if(GetMemoryMap(MEMMAP_PWM_OUTPUT0) <= 15) {
        SetPWM(1, 20);
    }
    else {
        SetPWM(1, 0);
    }
}

void returnAllInterfaces(void) {
    for (unsigned int i = 0; i < ( sizeof(interface) / sizeof(struct Interface) ); i++) {
        serialize(MSG_RETURN_INTERFACE, "ibBbbSS", interface[i].address
                  , interface[i].bytes
                  , interface[i].exponent
                  , interface[i].access_mask
                  , interface[i].enable_logging
                  , (const __FlashStringHelper *) interface[i].name
                  , (const __FlashStringHelper *) interface[i].unit
                 );
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
        }
        else if ( (output_num >= 4) && (output_num <= 7) ) {
            digitalWrite( output[output_num], (output_value[output_num] > 0) );
        }
    }
}

uint8_t GetPWM(uint8_t output_num) {
    if ( output_num < sizeof(output_value) ) { // Ensure write to array is within bounds
        return output_value[output_num];
    }
}

void broadcastPWMValues(void) {
    serialize(MSG_RETURN_8_8, "bb", (uint8_t)0xA0, (uint8_t)GetMemoryMap(0xA0));
}

// TODO: Can the interface structure be used to simplify this and make for handling in a more generic manner? Is it possible to require changes to be made in only one or two places instead of three or four?

/*************************************************/
/* SET VIRTUAL MEMORY MAP                        */
/*************************************************/
bool SetMemoryMap(uint16_t address, uint32_t data) {

// TODO: Set should be followed by publishing the new data.  Everywhere else this has been done needs removed.

    switch (address) {
    case MEMMAP_SETTING_BROADCAST_PERIOD_MS: // Sets the period between data broadcasts.  0xFFFFFFFF disables until another value is set.  Limited to minimum 100ms period.
        inhibit_broadcast = (data == 0xFFFFFFFF);
        broadcast_period_ms = max(100, data);
        broadcast_timer_counter = broadcast_period_ms * 1 / (INTERRUPT_PERIOD_MICROSECONDS * 0.001);
        return true;

    case MEMMAP_BANK0_AH_LEFT:
        battery_monitor_var[0].amphours_remaining = (int16_t)data / 10.0;
        battery_monitor_var[0].amphours_remaining = constrain(battery_monitor_var[0].amphours_remaining, 0, battery_monitor_const[0].amphours_capacity);
        return true;

    case MEMMAP_BANK0_SOC:
        battery_monitor_var[0].amphours_remaining = ( (int16_t)(data) / 100.0 / 100.0 * battery_monitor_const[0].amphours_capacity );
        battery_monitor_var[0].amphours_remaining = constrain(battery_monitor_var[0].amphours_remaining, 0, battery_monitor_const[0].amphours_capacity);
        return true;

    // TODO: Many of these have immediate effects that aren't yet being executed

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
        if( output[address & 0x00000007] == CEILING_LIGHT) {
            encoder1_value = (uint8_t)data;
        }

        return true;

    case MEMMAP_TEMPERATURE_FLOOR:
        //TODO: probably better not to constrain these as much or at all
        //hvac_temperature_floor = constrain(data * 0.1, -45.0, 50.0);
        hvac_temperature_floor = data * 0.1;
        return true;

    case MEMMAP_TEMPERATURE_CEILING:
        //hvac_temperature_ceiling = constrain(data * 0.1, -45.0, 50.0);
        hvac_temperature_ceiling = data * 0.1;
        return true;

    case MEMMAP_HVAC_DESIRED_TEMPERATURE:
        //hvac_desired_temperature = constrain(data * 0.1, MIN_DESIRED_TEMPERATURE, MAX_DESIRED_TEMPERATURE);
        hvac_desired_temperature = data * 0.1;
        return true;

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

    case MEMMAP_BANK0_VOLTS:
        return battery_monitor_var[0].volts * 100;
    case MEMMAP_BANK0_AMPS:
        return battery_monitor_var[0].amps * 10;
    case MEMMAP_BANK0_WATTS:
        return battery_monitor_var[0].volts * battery_monitor_var[0].amps * 10;
    case MEMMAP_BANK0_AH_LEFT:
        return battery_monitor_var[0].amphours_remaining * 10;
    case MEMMAP_BANK0_SOC:
        return (100 * battery_monitor_var[0].amphours_remaining / battery_monitor_const[0].amphours_capacity ) * 100;
    case MEMMAP_BANK0_TTG:
        return CalcTimeToGo(0) * 10;
    case MEMMAP_BANK0_CS:
        return battery_monitor_var[0].charge_state;

    case MEMMAP_BANK0_AMPS_MULTIPLIER:
        return battery_monitor_const[0].amps_multiplier * 1000000;
    case MEMMAP_BANK0_VOLTS_MULTIPLIER:
        return battery_monitor_const[0].volts_multiplier * 1000000;
    case MEMMAP_BANK0_AH_CAPACITY:
        return battery_monitor_const[0].amphours_capacity * 10;
    case MEMMAP_BANK0_VOLTS_CHARGED:
        return battery_monitor_const[0].volts_charged * 1000;
    case MEMMAP_BANK0_CHRG_DET_TIME:
        return battery_monitor_const[0].minutes_charged_detection_time * 10;
    case MEMMAP_BANK0_TAIL_CURRENT:
        return battery_monitor_const[0].tail_current_factor * 100;
    case MEMMAP_BANK0_CURRENT_THRESHOLD:
        return battery_monitor_const[0].current_threshold * 1000000;
    case MEMMAP_BANK0_PEUKERT_FACTOR:
        return battery_monitor_const[0].peukert_factor * 100;
    case MEMMAP_BANK0_CHRG_EFFICIENCY:
        return battery_monitor_const[0].charge_efficiency_factor * 100;

    case MEMMAP_SOLAR_VOLTS:
        return solar_volts * 100;
    case MEMMAP_SOLAR_AMPS:
        return solar_amps * 10;
    case MEMMAP_SOLAR_WATTS:
        return solar_volts * solar_amps * 10;

    case MEMMAP_INVERTER_VOLTS:
        return inverter_volts * 100;
    case MEMMAP_INVERTER_AMPS:
        return inverter_amps * 10;
    case MEMMAP_INVERTER_WATTS:
        return (inverter_volts == 0 ? battery_monitor_var[0].volts : inverter_volts) * inverter_amps * 10; // Fallback to main battery if separate inverter voltage not available

    case MEMMAP_VEHICLE_VOLTS:
        return vehicle_volts * 100;
    case MEMMAP_VEHICLE_AMPS:
        return vehicle_amps * 10;
    case MEMMAP_VEHICLE_WATTS:
        return vehicle_volts * vehicle_amps * 10;


    case MEMMAP_HVAC_ERROR:
        return hvac_error_code;
    case MEMMAP_HVAC_HEAT_ON:
        return hvac_heat_on;

    // TODO: Dynamic return of all defined battery bank variables?

    case MEMMAP_PWM_OUTPUT0:
    case MEMMAP_PWM_OUTPUT1:
    case MEMMAP_PWM_OUTPUT2:
    case MEMMAP_PWM_OUTPUT3:
    case MEMMAP_PWM_OUTPUT4:
    case MEMMAP_PWM_OUTPUT5:
    case MEMMAP_PWM_OUTPUT6:
    case MEMMAP_PWM_OUTPUT7:
        return GetPWM(address & 0x0007);

    case MEMMAP_TEMPERATURE_FLOOR:
        return hvac_temperature_floor * 10;
    case MEMMAP_TEMPERATURE_CEILING:
        return hvac_temperature_ceiling * 10;

    case MEMMAP_HVAC_STATE:
        return (uint8_t)hvac_state;
    case MEMMAP_HVAC_DESIRED_TEMPERATURE:
        return hvac_desired_temperature * 10;

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
            count++;
        }

        // TODO: Rewrite this using characters (i, b, B, etc) and string lengths to determine and limit payload size. Originally the format string was the number of bytes, thus this legacy code.
        //if (count > 255) {
        //  return false; // Fail on message with a payload size greater than 256
        //}

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
            p = (const char *)va_arg(args, const char * );
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
            Serial.write('"');
            Serial.print( (const __FlashStringHelper *) p );
            Serial.write('"');
            break;

        case 's':
            // TODO: Strip out characters from the string that will cause problems (unescaped?, comma, colon?)
            Serial.write('"');
            Serial.print( (const char *) p );
            Serial.write('"');
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

unsigned long MinutesToMilliseconds(unsigned int minutes) {
    return minutes * 60 * 1000;
}

unsigned int asciiHexToInt(char ch) {
    unsigned int num = 0;

    if( (ch >= '0') && (ch <= '9') ) {
        num = ch - '0';
    }
    else {
        switch(ch) {
        case 'A':
        case 'a':
            num = 10;
            break;
        case 'B':
        case 'b':
            num = 11;
            break;
        case 'C':
        case 'c':
            num = 12;
            break;
        case 'D':
        case 'd':
            num = 13;
            break;
        case 'E':
        case 'e':
            num = 14;
            break;
        case 'F':
        case 'f':
            num = 15;
            break;
        default:
            num = 0;
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
                            // TODO - ERROR: IMPOSED ARBITRARY EMBEDDED LIMITATION
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
