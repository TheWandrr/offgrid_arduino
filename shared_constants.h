#ifndef __SHARED_CONSTANTS_H
#define __SHARED_CONSTANTS_H

/* MESSAGE CODES SHARED ACROSS NODES */
#define MSG_RESERVED_00		0x00
#define MSG_RESERVED_01		0x01
#define MSG_RESERVED_02		0x02
#define MSG_RESERVED_03		0x03
#define MSG_RESERVED_04		0x04
#define MSG_RESERVED_05		0x05

/* TODO: Consider grouping these under a MSG_CONTROL with sub messages (CTRLMSG_KEEP_ALIVE, etc) in the second byte.  This way we don't use up the rest unnecessarily. */

#define MSG_KEEP_ALIVE		0x06
#define MSG_POWER_ON		0x07

#define MSG_RESERVED_08		0x08

#define MSG_GET_MEMMAP      0x09 /* Not sure yet how this is going to work, but it should allow the master to query writeable/readable addresses from the module */
#define MSG_RETURN_MEMMAP   0x0A

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
#define MEMMAP_BANK0_TAIL_CURRENT               0x0029 /* 1 bytes, x100 */
#define MEMMAP_BANK0_PEUKERT_FACTOR             0x002A /* 1 bytes, x100 */
#define MEMMAP_BANK0_CHRG_EFFICIENCY            0x002B /* 1 bytes, x100 */

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

#define ADC_0_01 0
#define ADC_0_23 1
#define ADC_1_01 2
#define ADC_1_23 3
#define ADC_2_0 4
#define ADC_2_1 5
#define ADC_2_2 6
#define ADC_2_3 7

#define MEMMAP_ADC_BASE							0x00B0

#define MEMMAP_ADC_0_01                         MEMMAP_ADC_BASE + ADC_0_01 /* 2 bytes, raw */
#define MEMMAP_ADC_0_23                         MEMMAP_ADC_BASE + ADC_0_23 /* 2 bytes, raw */
#define MEMMAP_ADC_1_01                         MEMMAP_ADC_BASE + ADC_1_01 /* 2 bytes, raw */
#define MEMMAP_ADC_1_23                         MEMMAP_ADC_BASE + ADC_1_23 /* 2 bytes, raw */
#define MEMMAP_ADC_2_0                          MEMMAP_ADC_BASE + ADC_2_0 /* 2 bytes, raw */
#define MEMMAP_ADC_2_1                          MEMMAP_ADC_BASE + ADC_2_1 /* 2 bytes, raw */
#define MEMMAP_ADC_2_2                          MEMMAP_ADC_BASE + ADC_2_2 /* 2 bytes, raw */
#define MEMMAP_ADC_2_3                          MEMMAP_ADC_BASE + ADC_2_3 /* 2 bytes, raw */

#endif /* __SHARED_CONSTANTS_H */
