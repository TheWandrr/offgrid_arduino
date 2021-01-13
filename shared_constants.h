#ifndef __SHARED_CONSTANTS_H
#define __SHARED_CONSTANTS_H

/* MESSAGE CODES SHARED ACROSS NODES */
#define MSG_RESERVED_00		0x00
#define MSG_RESERVED_01		0x01
#define MSG_RESERVED_02		0x02
#define MSG_RESERVED_03		0x03
#define MSG_RESERVED_04   0x04
#define MSG_RESERVED_05   0x05

#define MSG_KEEP_ALIVE		0x06
#define MSG_POWER_ON		0x07

#define MSG_GET_INTERFACE       0x08 /* Query for available memory mapped topics, accessible with MSG_SET_X_X/MSG_GET_X_X.  No data requests that the device return all available interfaces.  Data=address returns only the interface associated with that address, or error/nothing */
#define MSG_RETURN_INTERFACE    0x09 /* Return one message for each available interface when queried by MSG_GET_INTERFACES */
#define MSG_GET_INTERFACE_ERROR 0x0A

#define MSG_RESERVED_10     0x10 
#define MSG_GET_SET_ERROR   0x11 /* Sent by a module in response to a get/set request that could not be satisfied */

#define MSG_SET_8_8         0x12
#define MSG_GET_8_8         0x13
#define MSG_RETURN_8_8      0x14

#define MSG_SET_8_16        0x15
#define MSG_GET_8_16        0x16
#define MSG_RETURN_8_16     0x17

#define MSG_SET_8_32        0x18
#define MSG_GET_8_32        0x19
#define MSG_RETURN_8_32     0x1A

#define MSG_RESERVED_1B     0x1B
#define MSG_RESERVED_1C     0x1C
#define MSG_RESERVED_1D     0x1D
#define MSG_RESERVED_1E     0x1E
#define MSG_RESERVED_1F     0x1F

// Variables accessible through serial interface 0x0010 - 0x001F
#define MEMMAP_SETTING_BROADCAST_PERIOD_MS      0x0010 /* 4 bytes, x1 */

// BANK0 battery monitor 0x0020 - 0x002F
#define MEMMAP_BANK0_VOLTS                      0x0020 /* 2 bytes, x100 */
#define MEMMAP_BANK0_AMPS                       0x0021 /* 2 bytes, x10 */
#define MEMMAP_BANK0_AH_LEFT                    0x0022 /* 2 bytes, x10 */
#define MEMMAP_BANK0_SOC                        0x0023 /* 2 bytes, x100 */

#define MEMMAP_BANK0_AMPS_MULTIPLIER            0x0024 /* 4 bytes, x1000000 */
#define MEMMAP_BANK0_VOLTS_MULTIPLIER           0x0025 /* 4 bytes, x1000000 */
#define MEMMAP_BANK0_AH_CAPACITY                0x0026 /* 2 bytes, x10 */
#define MEMMAP_BANK0_VOLTS_CHARGED              0x0027 /* 2 bytes, x1000 */
#define MEMMAP_BANK0_CHRG_DET_TIME              0x0028 /* 2 bytes, x10 */
#define MEMMAP_BANK0_CURRENT_THRESHOLD          0x0029 /* 4 bytes, x1000000 */
#define MEMMAP_BANK0_TAIL_CURRENT               0x002A /* 1 bytes, x100 */
#define MEMMAP_BANK0_PEUKERT_FACTOR             0x002B /* 1 bytes, x100 */
#define MEMMAP_BANK0_CHRG_EFFICIENCY            0x002C /* 1 bytes, x100 */

// BANK1 battery monitor 0x0030 - 0x003F
// BANK2 battery monitor 0x0040 - 0x004F
// BANK3 battery monitor 0x0050 - 0x005F

// Last 4 bits are used for addressing, keep the same if others are changed
#define MEMMAP_PWM_BASE							0x00A0

#define MEMMAP_PWM_OUTPUT0                      MEMMAP_PWM_BASE + 0 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT1                      MEMMAP_PWM_BASE + 1 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT2                      MEMMAP_PWM_BASE + 2 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT3                      MEMMAP_PWM_BASE + 3 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT4                      MEMMAP_PWM_BASE + 4 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT5                      MEMMAP_PWM_BASE + 5 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT6                      MEMMAP_PWM_BASE + 6 /* 1 bytes, x1 */
#define MEMMAP_PWM_OUTPUT7                      MEMMAP_PWM_BASE + 7 /* 1 bytes, x1 */

#define ADC_0_01 0 /* TODO: TO BE DEPRECATED! */
#define ADC_0_23 1 /* TODO: TO BE DEPRECATED! */
#define ADC_1_01 2 /* TODO: TO BE DEPRECATED! */
#define ADC_1_23 3 /* TODO: TO BE DEPRECATED! */
#define ADC_2_0 4 /* TODO: TO BE DEPRECATED! */
#define ADC_2_1 5 /* TODO: TO BE DEPRECATED! */
#define ADC_2_2 6 /* TODO: TO BE DEPRECATED! */
#define ADC_2_3 7 /* TODO: TO BE DEPRECATED! */

#define MEMMAP_ADC_BASE							0x00B0 /* TODO: TO BE DEPRECATED! */

#define MEMMAP_ADC_0_01                         MEMMAP_ADC_BASE + ADC_0_01 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_0_23                         MEMMAP_ADC_BASE + ADC_0_23 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_1_01                         MEMMAP_ADC_BASE + ADC_1_01 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_1_23                         MEMMAP_ADC_BASE + ADC_1_23 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_2_0                          MEMMAP_ADC_BASE + ADC_2_0 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_2_1                          MEMMAP_ADC_BASE + ADC_2_1 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_2_2                          MEMMAP_ADC_BASE + ADC_2_2 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */
#define MEMMAP_ADC_2_3                          MEMMAP_ADC_BASE + ADC_2_3 /* 2 bytes, raw */ /* TODO: TO BE DEPRECATED! */

struct Topic {
  uint16_t address;
  uint8_t bytes;
  int8_t exponent;
  const char *name;
  const char *unit;
};

#endif /* __SHARED_CONSTANTS_H */
