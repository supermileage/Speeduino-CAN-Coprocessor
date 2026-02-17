#ifndef CONFIG_H
#define CONFIG_H

// CAN Bus Settings
#define CAN_RATE_HZ 20
#define SERIAL_TIMEOUT 100 // ms
#define CAN_ID_BASE         3100
#define CAN_ID_STATUS       3100      // Engine Status
#define CAN_ID_SENSORS      3101      // Battery, O2, RPM
#define CAN_ID_TUNE         3102      // AFR, VE, Advance
#define CAN_ID_DIAG         3103      // Loops, FreeRAM
#define CAN_ID_EXTRA        3104      // RPMDot, Flex

// RTOS Settings
#define STACK_SIZE 256
#define PRIORITY_SERIAL 2
#define PRIORITY_CAN 1

#endif