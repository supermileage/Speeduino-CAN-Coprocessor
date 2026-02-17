#ifndef __DATA_H__
#define __DATA_H__


#include <stdint.h>

/*
Speeduino Data Structures, see https://wiki.speeduino.com/en/Secondary_Serial_IO_interface
*/

typedef struct __attribute__((packed)) {
    uint8_t  secl;                // 0: Seconds counter
    uint8_t  status1;             // 1: Bitfield (Inj status, DFCO, etc)
    uint8_t  engine;              // 2: Bitfield (Running, Crank, ASE, WUE)
    uint8_t  dwell;               // 3: Dwell in ms * 10
    uint16_t MAP;                 // 4-5: Manifold Absolute Pressure
    uint8_t  iat;                 // 6: Intake Air Temp (Raw + Offset)
    uint8_t  coolant;             // 7: Coolant Temp (Raw + Offset)
    uint8_t  batCorrection;       // 8: Battery Correction %
    uint8_t  battery10;           // 9: Battery Voltage
    uint8_t  o2;                  // 10: Primary O2
    uint8_t  egoCorrection;       // 11: Exhaust Gas Correction %
    uint8_t  iatCorrection;       // 12: Air Temp Correction %
    uint8_t  wueCorrection;       // 13: Warmup Enrichment %
    uint16_t rpm;                 // 14-15: Engine RPM
    uint8_t  taeAmount;           // 16: Acceleration Enrichment %
    uint8_t  corrections;         // 17: Total GammaE %
    uint8_t  ve;                  // 18: Current VE 1 %
    uint8_t  afrTarget;           // 19: Chosen AFR Target
    uint16_t pw1;                 // 20-21: Pulsewidth 1 (ms * 10)
    uint8_t  tpsDot;              // 22: TPS DOT
    uint8_t  advance;             // 23: Current Spark Advance
    uint8_t  tps;                 // 24: TPS (0-100%)
    uint16_t loopsPerSecond;      // 25-26: Loop frequency
    uint16_t freeRam;             // 27-28: Free memory
    uint8_t  boostTarget;         // 29: Target Boost
    uint8_t  boostDuty;           // 30: Boost PWM Duty
    uint8_t  spark;               // 31: Spark Bitfield (Launch, Sync, etc)
    int16_t  rpmDot;              // 32-33: Rate of change RPM (Signed)
    uint8_t  ethanolPct;          // 34: Flex sensor %
    uint8_t  flexCorrection;      // 35: Flex Fuel Correction %
    uint8_t  flexIgnCorrection;   // 36: Flex Ignition Correction
    uint8_t  idleLoad;            // 37: Idle Load
    uint8_t  testOutputs;         // 38: Test mode bitfield
    uint8_t  o2_2;                // 39: Second O2 sensor
    uint8_t  baro;                // 40: Barometer
    uint16_t canin[16];           // 41-72: CAN Input Channels (16 x 2 bytes)
    uint8_t  tpsADC;              // 73: Raw TPS ADC (0-255)
    uint8_t  getNextError;        // 74: Error Codes
    uint8_t  launchCorrection;    // 75: Launch Correction %
    uint16_t pw2;                 // 76-77: Pulsewidth 2
    uint16_t pw3;                 // 78-79: Pulsewidth 3
    uint16_t pw4;                 // 80-81: Pulsewidth 4
    uint8_t  status3;             // 82: Status bitfield (Nitrous, VSS, etc)
    uint8_t  engineProtectStatus; // 83: Protection bitfield
    uint16_t fuelLoad;            // 84-85: Current Fuel Load
    uint16_t ignLoad;             // 86-87: Current Ignition Load
    uint16_t injAngle;            // 88-89: Injection Angle
    uint8_t  idleDuty;            // 90: Idle Duty Cycle
    uint8_t  CLIdleTarget;        // 91: Closed Loop Idle Target
    uint8_t  mapDOT;              // 92: MAP rate of change
    int8_t   vvt1Angle;           // 93: VVT 1 Actual Angle
    uint8_t  vvt1TargetAngle;     // 94: VVT 1 Target
    uint8_t  vvt1Duty;            // 95: VVT 1 Duty
    uint16_t flexBoostCorrection; // 96-97: Flex Boost Correction
    uint8_t  baroCorrection;      // 98: Baro Correction %
    uint8_t  ASEValue;            // 99: Current ASE %
    uint16_t vss;                 // 100-101: Vehicle Speed Sensor
    uint8_t  gear;                // 102: Current Gear
    uint8_t  fuelPressure;        // 103: Fuel Pressure
    uint8_t  oilPressure;         // 104: Oil Pressure
    uint8_t  wmiPW;               // 105: Water Methanol Injection PW
    uint8_t  status4;             // 106: WMI empty, VVT errors
    int8_t   vvt2Angle;           // 107: VVT 2 Actual Angle
    uint8_t  vvt2TargetAngle;     // 108: VVT 2 Target
    uint8_t  vvt2Duty;            // 109: VVT 2 Duty
    uint8_t  outputsStatus;       // 110: Output Bitfield
    uint8_t  fuelTemp;            // 111: Fuel Temp (Raw + Offset)
    uint8_t  fuelTempCorrection;  // 112: Fuel Temp Correction %
    uint8_t  ve1;                 // 113: VE 1
    uint8_t  ve2;                 // 114: VE 2
    uint8_t  advance1;            // 115: Advance 1
    uint8_t  advance2;            // 116: Advance 2
    uint8_t  nitrous_status;      // 117: Nitrous Status
    uint8_t  TS_SD_Status;        // 118: SD Card Status
} speeduino_data_t;

typedef union {
    speeduino_data_t data;
    uint8_t bytes[sizeof(speeduino_data_t)];
} speeduino_data_union_t;

#endif