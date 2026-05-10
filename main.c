/*
 * main.c
 * IMU Sensor Fusion - FreeRTOS Application
 *
 * Three tasks:
 *   SensorTask (priority 3) - reads MPU-6050 at 100 Hz
 *   FusionTask (priority 2) - runs complementary filter
 *   CommTask   (priority 1) - sends orientation via UART with CRC
 *
 * Built on the Lab 7 FreeRTOS skeleton (posix_demo/freertos).
 *
 * REUSED FROM LABS:
 *   UART setup     -> Lab 6 Ex 1.1
 *   Hardware CRC   -> Lab 6 Ex 2.1
 *   FreeRTOS tasks -> Lab 7 Ex 3.2
 *   LED control    -> Lab 2 Ex 1.1
 */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "ti_msp_dl_config.h"
#include "imu_common.h"
#include "mpu6050.h"
#include "fusion.h"
#include "protocol.h"

/* ===== Task Config ===== */
#define TASK_STACK_SIZE     1536
#define SENSOR_PRIORITY     3
#define FUSION_PRIORITY     2
#define COMM_PRIORITY       1

#define POWER_STARTUP_DELAY 32

/* ===== Shared State (global) ===== */
static QueueHandle_t      sensorQueue;
static QueueHandle_t      orientQueue;
static SemaphoreHandle_t  configMutex;
static SystemConfig_t     sysConfig;
static SystemHealth_t     sysHealth;
static CalibrationData_t  calData;

/* ================================================================
 * UART SETUP (from Lab 6 Exercise 1.1)
 * ================================================================ */

static const DL_UART_Main_ClockConfig gUART_ClkConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

static void uart_init(void)
{
    DL_UART_Main_reset(UART0);
    DL_UART_Main_enablePower(UART0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(UART0,
        (DL_UART_Main_ClockConfig *)&gUART_ClkConfig);
    DL_UART_Main_init(UART0,
        (DL_UART_Main_Config *)&gUART_Config);
    DL_UART_Main_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART0, 17, 23); /* 115200 baud */
    DL_UART_Main_enable(UART0);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21,
        IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22,
        IOMUX_PINCM22_PF_UART0_RX);
}

static void uart_send_byte(uint8_t byte)
{
    while (DL_UART_isTXFIFOFull(UART0));
    DL_UART_Main_transmitData(UART0, byte);
}

static void uart_send_buf(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        uart_send_byte(buf[i]);
}

/* ================================================================
 * CRC SETUP (from Lab 6 Exercise 2.1)
 * ================================================================ */

static void crc_init(void)
{
    DL_CRC_reset(CRC);
    DL_CRC_enablePower(CRC);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_CRC_init(CRC, DL_CRC_32_POLYNOMIAL, DL_CRC_BIT_REVERSED,
        DL_CRC_INPUT_ENDIANESS_LITTLE_ENDIAN,
        DL_CRC_OUTPUT_BYTESWAP_DISABLED);
}

static uint32_t crc_compute(const uint8_t *data, uint16_t len)
{
    DL_CRC_setSeed32(CRC, 0xFFFFFFFF);
    for (uint16_t i = 0; i < len; i++)
        DL_CRC_feedData8(CRC, data[i]);
    return DL_CRC_getResult32(CRC);
}

/* ================================================================
 * LED HELPERS (from Lab 2)
 * ================================================================ */

static void led_init(void)
{
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(32);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);  /* off (active-low) */
}

static void led_on(void)
{
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);
}

static void led_off(void)
{
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);
}

/* ================================================================
 * PACKET SENDER
 * ================================================================ */

static void send_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t pkt_buf[PACKET_HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE];
    uint8_t crc_buf[2 + MAX_PAYLOAD_SIZE];

    crc_buf[0] = msg_id;
    crc_buf[1] = len;
    memcpy(&crc_buf[2], payload, len);

    uint32_t crc_val = crc_compute(crc_buf, 2 + len);
    uint16_t pkt_len = packet_build(pkt_buf, msg_id, payload, len, crc_val);

    uart_send_buf(pkt_buf, pkt_len);
    sysHealth.packets_sent++;
}

/* ================================================================
 * SENSOR TASK
 * Reads MPU-6050, calibrates on startup, sends to FusionTask.
 * ================================================================ */
static void *SensorTask(void *arg0)
{
    (void)arg0;

    i2c_init();

    if (!mpu6050_init())
    {
        /* Sensor init failed - blink LED as error */
        while (1)
        {
            DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_0);
            usleep(100000);
        }
    }

    /* Calibrate (LED on during calibration) */
    led_on();
    mpu6050_calibrate(&calData);
    sysHealth.calibrated = 1;
    led_off();

    /* 100 Hz sensor loop */
    SensorSample_t sample;

    while (1)
    {
        if (mpu6050_read(&sample))
        {
            mpu6050_apply_calibration(&sample, &calData);
            sample.timestamp = xTaskGetTickCount();

            if (xQueueSend(sensorQueue, &sample, 0) != pdTRUE)
                sysHealth.packets_dropped++;
        }
        else
        {
            sysHealth.sensor_errors++;
            if (sysHealth.sensor_errors % 3 == 0)
            {
                i2c_bus_recovery();
                mpu6050_init();
            }
        }

        usleep(10000); /* 10 ms = 100 Hz */
    }
}

/* ================================================================
 * FUSION TASK
 * Receives samples, runs complementary filter, sends orientation.
 * ================================================================ */
static void *FusionTask(void *arg0)
{
    (void)arg0;

    FusionState_t fusionState;
    fusion_init(&fusionState);

    SensorSample_t sample;
    FusedOrientation_t orient;

    while (1)
    {
        if (xQueueReceive(sensorQueue, &sample, portMAX_DELAY) == pdTRUE)
        {
            /* Read config (mutex-protected) */
            float alpha = DEFAULT_ALPHA;
            if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                alpha = sysConfig.alpha;
                xSemaphoreGive(configMutex);
            }

            fusion_update_complementary(&fusionState, &sample, alpha, DT);
            fusion_get_orientation(&fusionState, &orient, sample.timestamp);

            xQueueSend(orientQueue, &orient, 0);
        }
    }
}

/* ================================================================
 * COMMUNICATION TASK
 * Sends orientation packets, handles incoming host commands.
 * ================================================================ */
static void *CommTask(void *arg0)
{
    (void)arg0;

    uart_init();
    crc_init();

    FusedOrientation_t orient;
    RxParser_t rxParser;
    rx_parser_init(&rxParser);

    uint8_t payload[MAX_PAYLOAD_SIZE];

    while (1)
    {
        /* Send orientation if available */
        if (xQueueReceive(orientQueue, &orient, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            uint8_t len = payload_build_orientation(payload, &orient);
            send_packet(MSG_ORIENTATION_EULER, payload, len);
        }

        /* Check for incoming commands */
        while (!DL_UART_isRXFIFOEmpty(UART0))
        {
            uint8_t byte = DL_UART_Main_receiveData(UART0);

            if (rx_parser_feed(&rxParser, byte))
            {
                /* Verify CRC */
                uint8_t crc_input[2 + MAX_PAYLOAD_SIZE];
                crc_input[0] = rxParser.msg_id;
                crc_input[1] = rxParser.length;
                memcpy(&crc_input[2], rxParser.payload, rxParser.length);

                uint32_t computed = crc_compute(crc_input, 2 + rxParser.length);
                uint32_t received = rx_parser_get_crc(&rxParser);

                if (computed == received)
                {
                    switch (rxParser.msg_id)
                    {
                    case CMD_SET_FILTER_PARAM:
                        if (rxParser.length >= 4)
                        {
                            float new_alpha;
                            memcpy(&new_alpha, rxParser.payload, 4);
                            if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                            {
                                sysConfig.alpha = new_alpha;
                                xSemaphoreGive(configMutex);
                            }
                        }
                        payload[0] = rxParser.msg_id;
                        payload[1] = 0x00;
                        send_packet(MSG_COMMAND_ACK, payload, 2);
                        break;

                    case CMD_REQUEST_HEALTH:
                        sysHealth.uptime_ms = xTaskGetTickCount();
                        {
                            uint8_t hlen = payload_build_health(payload, &sysHealth);
                            send_packet(MSG_SYSTEM_HEALTH, payload, hlen);
                        }
                        break;

                    case CMD_TRIGGER_CALIBRATION:
                        sysHealth.calibrated = 0;
                        payload[0] = rxParser.msg_id;
                        payload[1] = 0x00;
                        send_packet(MSG_COMMAND_ACK, payload, 2);
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }
}

/* ================================================================
 * MAIN (from Lab 7 FreeRTOS template)
 * ================================================================ */

static void prvSetupHardware(void);

int main(void)
{
    pthread_t sensorThread, fusionThread, commThread;
    pthread_attr_t attrs;
    struct sched_param priParam;
    int retc;

#ifdef __ICCARM__
    __iar_Initlocks();
#endif

    prvSetupHardware();

    /* Init shared config */
    sysConfig.filter_type    = 0;
    sysConfig.alpha          = DEFAULT_ALPHA;
    sysConfig.beta           = DEFAULT_BETA;
    sysConfig.sample_rate_hz = SAMPLE_RATE_HZ;
    sysConfig.telemetry_mask = 0xFF;
    memset(&sysHealth, 0, sizeof(sysHealth));

    /* Init LED */
    led_init();

    /* Create queues and mutex */
    sensorQueue = xQueueCreate(SENSOR_QUEUE_DEPTH, sizeof(SensorSample_t));
    orientQueue = xQueueCreate(ORIENT_QUEUE_DEPTH, sizeof(FusedOrientation_t));
    configMutex = xSemaphoreCreateMutex();

    /* Create tasks */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, TASK_STACK_SIZE);

    priParam.sched_priority = SENSOR_PRIORITY;
    pthread_attr_setschedparam(&attrs, &priParam);
    retc = pthread_create(&sensorThread, &attrs, SensorTask, NULL);
    if (retc != 0) while (1) {}

    priParam.sched_priority = FUSION_PRIORITY;
    pthread_attr_setschedparam(&attrs, &priParam);
    retc = pthread_create(&fusionThread, &attrs, FusionTask, NULL);
    if (retc != 0) while (1) {}

    priParam.sched_priority = COMM_PRIORITY;
    pthread_attr_setschedparam(&attrs, &priParam);
    retc = pthread_create(&commThread, &attrs, CommTask, NULL);
    if (retc != 0) while (1) {}

    vTaskStartScheduler();

    return 0;
}

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}

/* Stack overflow hook (from Lab 7 template) */
#if (configCHECK_FOR_STACK_OVERFLOW)
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    while (1) {}
}
#endif
