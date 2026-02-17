#include "bridge_rtos.h"
#include "main.h"
#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os2.h"

// main.c
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;


#define SPEEDUINO_A_DATA_LEN 75
static uint8_t shared_a75[SPEEDUINO_A_DATA_LEN]; // Stores the latest 'A' data from Speeduino
static osMutexId_t a75_mutex; // Mutex to protect access to shared_a75
static osThreadId_t serialTaskHandle; // Task handle for the serial reading task
static osThreadId_t canTaskHandle; // Task handle for the CAN output task

static void SerialReadTask(void *argument);
static void CanOutputTask(void *argument);

static uint8_t shared_a75[SPEEDUINO_A_DATA_LEN]; // Stores the latest 'A' data from Speeduino
static osMutexId_t a75_mutex; // CMSIS-RTOS2 mutex
static osThreadId_t serialTaskHandle; // CMSIS-RTOS2 thread handle for serial
static osThreadId_t canTaskHandle; // CMSIS-RTOS2 thread handle for CAN

// Polls the UART for a single byte with a timeout
// returns true if a byte was read
static bool uart_read_byte(uint8_t *out, uint32_t timeout_ms) {
    return (HAL_UART_Receive(&huart2, out, 1, timeout_ms) == HAL_OK);
}

// Flushes the UART buffer
static void uart_drain(uint32_t max_bytes) {
    uint8_t b = 0;
    for (uint32_t i = 0;  i < max_bytes; i++) {
        if (!uart_read_byte(&b, 0)) // Read/flush bytes until empty
            break;
    }
}

// Requests the 'A' data from Speeduino, tries to get the 75 bytes into the buffer
// See Speeduino documentation plz
static bool speeduino_request_A(uint8_t buffer[SPEEDUINO_A_DATA_LEN], uint32_t timeout_ms) {
    const uint8_t cmd = 'A';
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)&cmd, 1, 10);

    // Resync: wait for echo 'A'
    const uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t b = 0;
        if (uart_read_byte(&b, 10)) {
            if (b == (uint8_t)'A')
                break;
        }
        if ((HAL_GetTick() - start) >= timeout_ms)
            return false;
    }

    // Read the 75 bytes of 'A' data into the provided buffer
    return (HAL_UART_Receive(&huart2, buffer, SPEEDUINO_A_DATA_LEN, timeout_ms) == HAL_OK);
}

// Given an ID and 8 bytes of data, send a CAN message with retries
// returns true if successful.
static bool can_send_any_id(uint32_t id, const uint8_t data[8]) {
    CAN_TxHeaderTypeDef header = {0}; // CAN message header struct
    uint32_t mailbox = 0; // Mailbox variable for HAL function

    // Standard 11-bit CAN IDs
    header.StdId = (uint16_t)(id & 0x7FFU);
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    for (uint32_t attempt = 0; attempt < 5; attempt++) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
            if (HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t*)data, &mailbox) == HAL_OK) {
                return true;
            }
        }
        osDelay(1); // CMSIS-RTOS2 delay
    }
    return false;
}

static void bridge_send_dxcontrol_frames_from_A(const uint8_t *a75) {
    // Unpack necessary 16-bit values from the byte array
    // Speeduino serial protocol is Little Endian for multi-byte values
    uint16_t map = (uint16_t)a75[4] | ((uint16_t)a75[5] << 8);
    uint16_t rpm = (uint16_t)a75[14] | ((uint16_t)a75[15] << 8);
    uint16_t pw1 = (uint16_t)a75[20] | ((uint16_t)a75[21] << 8);

    const uint8_t f0[8] = {
        a75[0],              // secl
        a75[1],              // status1
        a75[2],              // engine
        a75[3],              // dwell
        (uint8_t)(map >> 1), // MAP / 2
        a75[6],              // IAT
        a75[7],              // CLT
        a75[73],             // tpsADC
    };

    const uint8_t f1[8] = {
        a75[9],                 // battery10
        a75[10],                // O2
        a75[11],                // egoCorrection
        a75[12],                // iatCorrection
        a75[13],                // wueCorrection
        (uint8_t)(rpm & 0xFFU), // rpm LB
        (uint8_t)(rpm >> 8),    // rpm HB
        a75[16],                // TAEamount
    };

    const uint8_t f2[8] = {
        0x00,                  // Baro correction (unused in dxControl v1 mapping)
        a75[17],               // corrections
        a75[18],               // VE
        a75[19],               // afrTarget
        (uint8_t)(pw1 / 100U), // PW1 scaling per dxControl mapping
        a75[22],               // tpsDOT
        a75[23],               // advance
        a75[24],               // TPS
    };

    const uint8_t f3[8] = {
        a75[25], // loopsPerSecond LB
        a75[26], // loopsPerSecond HB
        a75[27], // freeRAM LB
        a75[28], // freeRAM HB
        a75[8],  // batCorrection
        a75[31], // spark bitfield
        a75[39], // O2_2
        0x00,
    };

    const uint8_t f4[8] = {
        0x00,
        a75[32], // rpmDOT LB
        a75[33], // rpmDOT HB
        a75[34], // flex sensor % (ethanol)
        0x00,
        0x00,
        0x00,
        0x00,
    };

    (void)can_send_any_id(CAN_ID_STATUS, f0);
    (void)can_send_any_id(CAN_ID_SENSORS, f1);
    (void)can_send_any_id(CAN_ID_TUNE, f2);
    (void)can_send_any_id(CAN_ID_DIAG, f3);
    (void)can_send_any_id(CAN_ID_EXTRA, f4);
}

void Bridge_Hardware_Init(void) {
    // Configure a permissive CAN filter (accept all)
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();

    uart_drain(256); // Flush any junk from UART
}

void Bridge_RTOS_Init(void) {
    static const osMutexAttr_t a75_mutex_attr = {
        .name = "a75_mutex"
    };
    a75_mutex = osMutexNew(&a75_mutex_attr);
    if (a75_mutex == NULL) Error_Handler();

    static const osThreadAttr_t serialTask_attr = {
        .name = "SerialReadTask",
        .stack_size = STACK_SIZE,
        .priority = (osPriority_t)PRIORITY_SERIAL
    };
    serialTaskHandle = osThreadNew(SerialReadTask, NULL, &serialTask_attr);
    if (serialTaskHandle == NULL) Error_Handler();

    static const osThreadAttr_t canTask_attr = {
        .name = "CanOutputTask",
        .stack_size = STACK_SIZE,
        .priority = (osPriority_t)PRIORITY_CAN
    };
    canTaskHandle = osThreadNew(CanOutputTask, NULL, &canTask_attr);
    if (canTaskHandle == NULL) Error_Handler();
}

static void SerialReadTask(void *argument) {
    const uint32_t period_ms = (CAN_RATE_HZ > 0) ? (1000U / (uint32_t)CAN_RATE_HZ) : 50U;
    for (;;) {
        uint8_t temp[SPEEDUINO_A_DATA_LEN] = {0};
        if (speeduino_request_A(temp, SERIAL_TIMEOUT)) {
            if (osMutexAcquire(a75_mutex, osWaitForever) == osOK) {
                memcpy(shared_a75, temp, SPEEDUINO_A_DATA_LEN);
                osMutexRelease(a75_mutex);
            }
        }
        osDelay(period_ms);
    }
}

static void CanOutputTask(void *argument) {
    const uint32_t period_ms = (CAN_RATE_HZ > 0) ? (1000U / (uint32_t)CAN_RATE_HZ) : 50U;
    for (;;) {
        if (osMutexAcquire(a75_mutex, osWaitForever) == osOK) {
            bridge_send_dxcontrol_frames_from_A(shared_a75);
            osMutexRelease(a75_mutex);
        }
        osDelay(period_ms);
    }
}