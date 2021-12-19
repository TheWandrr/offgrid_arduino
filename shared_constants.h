#ifndef __SHARED_CONSTANTS_H
#define __SHARED_CONSTANTS_H

/* MESSAGE CODES SHARED ACROSS NODES */
#define MSG_RESERVED_00		0x00
#define MSG_RESERVED_01		0x01
#define MSG_RESERVED_02		0x02
#define MSG_RESERVED_03		0x03
#define MSG_RESERVED_04   0x04

#define MSG_DEBUG_STRING  0x05

#define MSG_KEEP_ALIVE		0x06
#define MSG_POWER_ON		0x07

#define MSG_GET_INTERFACE       0x08 /* Query for available memory mapped topics, accessible with MSG_SET_X_X/MSG_GET_X_X.  No data requests that the device return all available interfaces.  Data=address returns only the interface associated with that address, or error/nothing */
#define MSG_RETURN_INTERFACE    0x09 /* Return one message for each available interface when queried by MSG_GET_INTERFACES */
#define MSG_GET_INTERFACE_ERROR 0x0A

#define MSG_RESERVED_10     0x10
#define MSG_GET_SET_ERROR   0x11 /* Sent by a module in response to a get/set request that could not be satisfied */

/* All data sent as signed values */

#define MSG_SET_8_8         0x12
#define MSG_GET_8_8         0x13
#define MSG_RETURN_8_8      0x14

#define MSG_SET_8_16        0x15
#define MSG_GET_8_16        0x16
#define MSG_RETURN_8_16     0x17

//#define MSG_SET_8_24        0x18
//#define MSG_GET_8_24        0x19
//#define MSG_RETURN_8_24     0x20

#define MSG_SET_8_32        0x21
#define MSG_GET_8_32        0x22
#define MSG_RETURN_8_32     0x23

/**** NOTE: Keep these addresses 8-bit until more than that are really needed! ****/

enum Memmap {
  MEMMAP_SETTING_BROADCAST_PERIOD_MS = 0x10,

  MEMMAP_BANK0_VOLTS,
  MEMMAP_BANK0_AMPS,
  MEMMAP_BANK0_AH_LEFT,
  MEMMAP_BANK0_SOC,
  MEMMAP_BANK0_TTG,
  MEMMAP_BANK0_CS,

  MEMMAP_BANK0_AMPS_MULTIPLIER,
  MEMMAP_BANK0_VOLTS_MULTIPLIER,
  MEMMAP_BANK0_AH_CAPACITY,
  MEMMAP_BANK0_VOLTS_CHARGED,
  MEMMAP_BANK0_CHRG_DET_TIME,
  MEMMAP_BANK0_CURRENT_THRESHOLD,
  MEMMAP_BANK0_TAIL_CURRENT,
  MEMMAP_BANK0_PEUKERT_FACTOR,
  MEMMAP_BANK0_CHRG_EFFICIENCY,

  MEMMAP_SOLAR_VOLTS,
  MEMMAP_SOLAR_AMPS,

  MEMMAP_INVERTER_AMPS,
  MEMMAP_INVERTER_VOLTS,

  MEMMAP_VEHICLE_VOLTS,
  MEMMAP_VEHICLE_AMPS,

  // The PWM outputs are bit addressed, so align at 0x_0 - Make sure this doesn't overlap previous enums!
  MEMMAP_PWM_OUTPUT0 = 0xA0,
  MEMMAP_PWM_OUTPUT1,
  MEMMAP_PWM_OUTPUT2,
  MEMMAP_PWM_OUTPUT3,
  MEMMAP_PWM_OUTPUT4,
  MEMMAP_PWM_OUTPUT5,
  MEMMAP_PWM_OUTPUT6,
  MEMMAP_PWM_OUTPUT7,

  MEMMAP_LOCAL_BASE = 0xFF00 /* This needs to match with offgrid-daemon, which is where the local mappings originate */
};

enum InterfaceAccessMask {
  AM_NONE =       0b00000000,
  AM_READ =       0b00000001,
  AM_WRITE =      0b00000010,
  AM_READWRITE =  0b00000011,
};

struct Interface {
  uint16_t address;
  uint8_t bytes;
  int8_t exponent;
  uint8_t access_mask;
  uint8_t enable_logging; /* bool */
  const char *name;
  const char *unit;
};

#endif /* __SHARED_CONSTANTS_H */
