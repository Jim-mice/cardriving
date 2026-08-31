#include <stdio.h>
#include <stdint.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "app_time.h"
#include "los_memory.h"

#include "Peripheral.h"
#include "wifi_connect.h"
#include "uart_task9.h"
#include "task10_servo_distance.h"
#include "task10_bluetooth.h"
#include "task_sensor.h"
#include "task_wifi.h"
#include "task_hello.h"
#include "task_cloud.h"
#include "task_car_control.h"
#include "udp_telemetry.h"
#include "task_encoder_cal.h"

#define WIFI_STA_START_WAIT_MS 6000
#define CLOUD_VALIDATION_MODE 1
#define CLOUD_LAUNCH_RETRY_MS 2000U

/* Low-frequency boot-only allocator diagnostics. LOS_MEM_STATUS.totalSize is
 * the largest contiguous free block in this Hi3861 LiteOS implementation. */
static void MemBootPublish(const char *stage)
{
    LOS_MEM_STATUS status = {0};

    if (LOS_MemInfoGet((VOID *)OS_SYS_MEM_ADDR, &status) == LOS_OK) {
        printf("MEMBOOT stage=%s used=%u free=%u max_free_block=%u alloc_count=%u free_count=%u\r\n",
               stage, (unsigned int)status.usedSize, (unsigned int)status.freeSize,
               (unsigned int)status.totalSize, (unsigned int)status.allocCount,
               (unsigned int)status.freeCount);
    } else {
        printf("MEMBOOT stage=%s status=UNAVAILABLE\r\n", stage);
    }
}

static void QstCarTask(void)
{
    uint8_t cloudStarted = 0U;
    uint8_t cloudNetworkReadyLogged = 0U;
    uint32_t cloudRetryElapsed = CLOUD_LAUNCH_RETRY_MS;

    printf("QST car start\r\n");
    printf("kernel tick freq = %u Hz\r\n", (unsigned int)osKernelGetTickFreq());
    MemBootPublish("APP_ENTER");

    Peripheral_Init();
    TaskWifiInit();
    MemBootPublish("AFTER_WIFI_INIT");

    /* Only wait for hi_wifi_sta_start(), never for scan, connect or DHCP. */
    if (TaskWifiWaitStaStarted(WIFI_STA_START_WAIT_MS) == 1) {
        printf("wifi sta started\r\n");
    } else {
        printf("wifi sta start wait timeout or failed\r\n");
    }

#if (CLOUD_VALIDATION_MODE == 0)
    TaskHelloInit();
#endif

    /* UART2 is initialized by Peripheral_Init before its RX tasks start. */
    MemBootPublish("BEFORE_TASK_CREATE");
    UartTask9Init();
    (void)TaskCarControlInit();
    Task10BluetoothInit();
    UdpTelemetryInit();
    EncoderCalInit();
    Task10ServoDistanceInit();

    TaskSensorInit();
    MemBootPublish("AFTER_TASK_CREATE");

    printf("Peripheral init done\r\n");
    printf("cloud launch waiting network\r\n");

    for (;;) {
        if (cloudStarted == 0U && TaskWifiIsNetworkReady() != 0) {
            if (cloudNetworkReadyLogged == 0U) {
                printf("cloud launch network ready\r\n");
                cloudNetworkReadyLogged = 1U;
            }

            if (cloudRetryElapsed >= CLOUD_LAUNCH_RETRY_MS) {
                if (CloudTaskInit() == 0) {
                    cloudStarted = 1U;
                    printf("cloud thread created\r\n");
                } else {
                    printf("cloud thread create retry\r\n");
                }
                cloudRetryElapsed = 0U;
            }
        }

        osDelay(AppMsToTicks(1000U));
        if (cloudStarted == 0U && cloudRetryElapsed < CLOUD_LAUNCH_RETRY_MS) {
            cloudRetryElapsed += 1000U;
        }
    }
}

static void QstCarEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "qst_car_task";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;


    osThreadNew(
        (osThreadFunc_t)QstCarTask,
        NULL,
        &attr
    );
}


SYS_RUN(QstCarEntry);
