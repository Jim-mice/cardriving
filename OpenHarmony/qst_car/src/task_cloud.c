#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "cloud_config.h"
#include "oc_mqtt.h"
#include "task_cloud.h"
#include "task_sensor.h"
#include "task_wifi.h"

#define CLOUD_REPORT_PERIOD_MS       5000U
#define CLOUD_CONNECT_RETRY_MS       5000U
#define CLOUD_WAIT_NETWORK_LOG_MS    5000U

static int CloudCredentialsConfigured(void)
{
    return CLOUD_CLIENT_ID[0] != '\0' && CLOUD_USERNAME[0] != '\0' &&
        CLOUD_PASSWORD[0] != '\0' && CLOUD_DEVICE_ID[0] != '\0';
}

static int CloudReportLatestSensors(void)
{
    static char serviceId[] = "qstcar";
    static char temperatureKey[] = "temp";
    static char humidityKey[] = "humi";
    static char lightKey[] = "lumi";
    float temperature;
    float humidity;
    uint16_t light;
    uint16_t ir;
    uint16_t proximity;
    int temperatureValue;
    int humidityValue;
    int lightValue;
    oc_mqtt_profile_kv_t temperatureKv;
    oc_mqtt_profile_kv_t humidityKv;
    oc_mqtt_profile_kv_t lightKv;
    oc_mqtt_profile_service_t service;
    int ret;

    if (TaskSensorGetSht20Latest(&temperature, &humidity) == 0 ||
        TaskSensorGetAp3216Latest(&light, &ir, &proximity) == 0) {
        printf("cloud: sensor data not ready\r\n");
        return -1;
    }

    /*
     * The imported IoTDA model declares temp/humi/lumi as int. Do not send
     * floats for these fields until the cloud model is changed accordingly.
     */
    temperatureValue = (int)temperature;
    humidityValue = (int)humidity;
    lightValue = (int)light;

    temperatureKv.nxt = &humidityKv;
    temperatureKv.key = temperatureKey;
    temperatureKv.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    temperatureKv.value = &temperatureValue;

    humidityKv.nxt = &lightKv;
    humidityKv.key = humidityKey;
    humidityKv.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    humidityKv.value = &humidityValue;

    lightKv.nxt = NULL;
    lightKv.key = lightKey;
    lightKv.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    lightKv.value = &lightValue;

    service.nxt = NULL;
    service.service_id = serviceId;
    service.event_time = NULL;
    service.service_property = &temperatureKv;

    ret = oc_mqtt_profile_propertyreport((char *)CLOUD_DEVICE_ID, &service);
    printf("cloud report T=%.2f H=%.2f Lux=%u ret=%d\r\n",
        (double)temperature, (double)humidity, (unsigned int)light, ret);
    return ret;
}

static void CloudTask(void *argument)
{
    int ret;
    uint32_t waitLogElapsed = CLOUD_WAIT_NETWORK_LOG_MS;
    (void)argument;

    printf("cloud task start\r\n");
    while (TaskWifiIsNetworkReady() == 0) {
        if (waitLogElapsed >= CLOUD_WAIT_NETWORK_LOG_MS) {
            printf("cloud waiting network\r\n");
            waitLogElapsed = 0U;
        }
        osDelay(1000U);
        waitLogElapsed += 1000U;
    }

    printf("cloud network ready\r\n");
    if (CloudCredentialsConfigured() == 0) {
        printf("cloud credentials missing\r\n");
        return;
    }

    for (;;) {
        printf("cloud mqtt connecting\r\n");
        printf("cloud broker: %s:%u\r\n", CLOUD_MQTT_SERVER,
            (unsigned int)CLOUD_MQTT_PORT);
        if (device_info_init(CLOUD_CLIENT_ID, CLOUD_USERNAME, CLOUD_PASSWORD) != 0) {
            printf("cloud mqtt connect failed: invalid credentials\r\n");
            osDelay(CLOUD_CONNECT_RETRY_MS);
            continue;
        }

        ret = oc_mqtt_init();
        if (ret == 0) {
            printf("cloud mqtt connected\r\n");
            break;
        }

        printf("cloud mqtt connect failed: %d\r\n", ret);
        osDelay(CLOUD_CONNECT_RETRY_MS);
    }

    for (;;) {
        (void)CloudReportLatestSensors();
        osDelay(CLOUD_REPORT_PERIOD_MS);
    }
}

int CloudTaskInit(void)
{
    static int cloudTaskCreated;
    osThreadAttr_t attr;

    if (cloudTaskCreated != 0) {
        return 0;
    }

    attr.name = "cloud_task";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(CloudTask, NULL, &attr) == NULL) {
        return -1;
    }

    cloudTaskCreated = 1;
    return 0;
}
