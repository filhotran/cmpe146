/*
 * main.c
 * IMU Sensor Fusion - FreeRTOS Application
 * CMPE 146 - Spring 2026
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

#define THREADSTACKSIZE  1536
#define POWER_STARTUP_DELAY 32

/* ===== Shared state ===== */
static QueueHandle_t     sensorQ;
static QueueHandle_t     orientQ;
static SemaphoreHandle_t configMutex;
static float             g_alpha = DEFAULT_ALPHA;
static CalData_t         g_cal;
static Health_t          g_health;

/* UART setup */

static const DL_UART_Main_ClockConfig gUART_ClkCfg = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_Cfg = {
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
    DL_UART_Main_setClockConfig(UART0, (DL_UART_Main_ClockConfig *)&gUART_ClkCfg);
    DL_UART_Main_init(UART0, (DL_UART_Main_Config *)&gUART_Cfg);
    DL_UART_Main_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART0, 17, 23);
    DL_UART_Main_enable(UART0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);
}

static void uart_send(uint8_t byte)
{
    while (DL_UART_isTXFIFOFull(UART0));
    DL_UART_Main_transmitData(UART0, byte);
}

static void uart_send_buf(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
        uart_send(buf[i]);
}

/* CRC setup */

static void crc_init(void)
{
    DL_CRC_reset(CRC);
    DL_CRC_enablePower(CRC);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_CRC_init(CRC, DL_CRC_32_POLYNOMIAL, DL_CRC_BIT_REVERSED,
        DL_CRC_INPUT_ENDIANESS_LITTLE_ENDIAN, DL_CRC_OUTPUT_BYTESWAP_DISABLED);
}

static uint32_t crc_compute(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    DL_CRC_setSeed32(CRC, 0xFFFFFFFF);
    for (i = 0; i < len; i++)
        DL_CRC_feedData8(CRC, data[i]);
    return DL_CRC_getResult32(CRC);
}

/* LED setup */

static void led_init(void)
{
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(32);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);
}

/* ===== Send a UART packet: [AA][55][id][len][payload][crc32] ===== */

static void send_packet(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[4 + 255 + 4];
    uint8_t crc_in[2 + 255];
    uint32_t crc;
    uint16_t idx = 0;

    /* CRC over msg_id + len + payload */
    crc_in[0] = msg_id;
    crc_in[1] = len;
    memcpy(&crc_in[2], payload, len);
    crc = crc_compute(crc_in, 2 + len);

    /* Build packet */
    buf[idx++] = SYNC_0;
    buf[idx++] = SYNC_1;
    buf[idx++] = msg_id;
    buf[idx++] = len;
    memcpy(&buf[idx], payload, len);
    idx += len;
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 24) & 0xFF);

    uart_send_buf(buf, idx);
    g_health.packets_sent++;
}

/* ===== Parse incoming host commands ===== */

typedef enum { WAIT_S0, WAIT_S1, WAIT_ID, WAIT_LEN, WAIT_DATA, WAIT_CRC } RxState;

static RxState   rx_state = WAIT_S0;
static uint8_t   rx_id, rx_len, rx_payload[255], rx_crc_buf[4];
static uint8_t   rx_pi, rx_ci;

static bool rx_feed(uint8_t b)
{
    switch (rx_state) {
    case WAIT_S0:  if (b == SYNC_0) rx_state = WAIT_S1; break;
    case WAIT_S1:
        if (b == SYNC_1) rx_state = WAIT_ID;
        else if (b != SYNC_0) rx_state = WAIT_S0;
        break;
    case WAIT_ID:  rx_id = b; rx_state = WAIT_LEN; break;
    case WAIT_LEN:
        rx_len = b; rx_pi = 0; rx_ci = 0;
        rx_state = (b == 0) ? WAIT_CRC : WAIT_DATA;
        break;
    case WAIT_DATA:
        rx_payload[rx_pi++] = b;
        if (rx_pi >= rx_len) rx_state = WAIT_CRC;
        break;
    case WAIT_CRC:
        rx_crc_buf[rx_ci++] = b;
        if (rx_ci >= 4) { rx_state = WAIT_S0; return true; }
        break;
    }
    return false;
}

static void process_command(void)
{
    uint8_t crc_in[2 + 255];
    uint32_t computed, received;
    uint8_t ack[2];

    crc_in[0] = rx_id;
    crc_in[1] = rx_len;
    memcpy(&crc_in[2], rx_payload, rx_len);
    computed = crc_compute(crc_in, 2 + rx_len);
    received = (uint32_t)rx_crc_buf[0] | ((uint32_t)rx_crc_buf[1] << 8)
             | ((uint32_t)rx_crc_buf[2] << 16) | ((uint32_t)rx_crc_buf[3] << 24);

    if (computed != received) return;  /* bad CRC, drop */

    switch (rx_id) {
    case CMD_SET_ALPHA:
        if (rx_len >= 4) {
            float a;
            memcpy(&a, rx_payload, 4);
            if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_alpha = a;
                xSemaphoreGive(configMutex);
            }
        }
        ack[0] = rx_id; ack[1] = 0;
        send_packet(MSG_ACK, ack, 2);
        break;

    case CMD_CALIBRATE:
        g_health.calibrated = 0;
        ack[0] = rx_id; ack[1] = 0;
        send_packet(MSG_ACK, ack, 2);
        break;

    case CMD_GET_HEALTH:
        g_health.uptime_ms = xTaskGetTickCount();
        {
            uint8_t hp[12];
            memcpy(&hp[0], &g_health.uptime_ms, 4);
            memcpy(&hp[4], &g_health.sensor_errors, 4);
            memcpy(&hp[8], &g_health.packets_sent, 4);
            send_packet(MSG_HEALTH, hp, 12);
        }
        break;
    }
}

/* ===== SENSOR TASK ===== */

static void *SensorTask(void *arg)
{
    (void)arg;
    SensorSample_t s;

    i2c_init();

    if (!mpu6050_init()) {
        while (1) {
            DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_0);
            usleep(100000);
        }
    }

    /* Calibrate (red LED on during calibration) */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);  /* LED on */
    mpu6050_calibrate(&g_cal);
    g_health.calibrated = 1;
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);    /* LED off */

    while (1) {
        if (mpu6050_read(&s)) {
            mpu6050_apply_cal(&s, &g_cal);
            s.tick = xTaskGetTickCount();
            xQueueSend(sensorQ, &s, 0);
        } else {
            g_health.sensor_errors++;
            if (g_health.sensor_errors % 3 == 0) {
                i2c_bus_recovery();
                mpu6050_init();
            }
        }
        usleep(10000);  /* 10 ms = 100 Hz */
    }
}

/* ===== FUSION TASK ===== */

static void *FusionTask(void *arg)
{
    (void)arg;
    FusionState_t filt;
    SensorSample_t s;
    Orientation_t o;
    float alpha;

    fusion_init(&filt);

    while (1) {
        if (xQueueReceive(sensorQ, &s, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                alpha = g_alpha;
                xSemaphoreGive(configMutex);
            } else {
                alpha = DEFAULT_ALPHA;
            }

            fusion_update(&filt, &s, alpha, DT);

            o.pitch = filt.pitch;
            o.roll  = filt.roll;
            o.yaw   = filt.yaw;
            o.tick  = s.tick;
            xQueueSend(orientQ, &o, 0);
        }
    }
}

/* ===== COMM TASK ===== */

static void *CommTask(void *arg)
{
    (void)arg;
    Orientation_t o;
    uint8_t payload[12];

    uart_init();
    crc_init();

    while (1) {
        if (xQueueReceive(orientQ, &o, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(&payload[0], &o.pitch, 4);
            memcpy(&payload[4], &o.roll, 4);
            memcpy(&payload[8], &o.yaw, 4);
            send_packet(MSG_ORIENTATION, payload, 12);
        }

        while (!DL_UART_isRXFIFOEmpty(UART0)) {
            if (rx_feed(DL_UART_Main_receiveData(UART0)))
                process_command();
        }
    }
}

/* Main */

static void prvSetupHardware(void);

int main(void)
{
    pthread_t t1, t2, t3;
    pthread_attr_t attrs;
    struct sched_param pri;

#ifdef __ICCARM__
    __iar_Initlocks();
#endif

    prvSetupHardware();

    memset(&g_health, 0, sizeof(g_health));
    led_init();

    sensorQ    = xQueueCreate(4, sizeof(SensorSample_t));
    orientQ    = xQueueCreate(4, sizeof(Orientation_t));
    configMutex = xSemaphoreCreateMutex();

    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);

    pri.sched_priority = 3;
    pthread_attr_setschedparam(&attrs, &pri);
    pthread_create(&t1, &attrs, SensorTask, NULL);

    pri.sched_priority = 2;
    pthread_attr_setschedparam(&attrs, &pri);
    pthread_create(&t2, &attrs, FusionTask, NULL);

    pri.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &pri);
    pthread_create(&t3, &attrs, CommTask, NULL);

    vTaskStartScheduler();
    return 0;
}

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}

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
{ while (1) {} }
#endif
