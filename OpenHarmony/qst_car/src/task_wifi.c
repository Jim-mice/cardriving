#include <stdio.h>

#include "cmsis_os2.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "task_wifi.h"
#include "wifi_connect.h"

#define WIFI_STA_SSID       "SSID"
#define WIFI_STA_PASSWORD   "PASSWORD"
#define WIFI_CONNECT_TIMEOUT_MS 60000

static volatile int g_wifiConnectFinished;

static void PrintWifiIp(void)
{
    struct netif *netif = netifapi_netif_find("wlan0");
    const ip4_addr_t *ip;

    if (netif == NULL) {
        printf("ip: unavailable\r\n");
        return;
    }

    ip = netif_ip4_addr(netif);
    printf("ip: %u.%u.%u.%u\r\n", (unsigned int)ip4_addr1(ip),
        (unsigned int)ip4_addr2(ip), (unsigned int)ip4_addr3(ip),
        (unsigned int)ip4_addr4(ip));
}

static void WifiConnectTask(void *argument)
{
    int result;
    (void)argument;

    printf("wifi connecting...\r\n");
    result = WifiConnect(WIFI_STA_SSID, WIFI_STA_PASSWORD);
    g_wifiConnectFinished = 1;

    if (result == 0) {
        printf("wifi connected\r\n");
        PrintWifiIp();
    } else {
        printf("wifi connect failed\r\n");
    }
}

static void WifiTimeoutTask(void *argument)
{
    (void)argument;

    osDelay(WIFI_CONNECT_TIMEOUT_MS);
    if (g_wifiConnectFinished == 0) {
        printf("wifi connect timeout\r\n");
    }
}

void TaskWifiInit(void)
{
    osThreadAttr_t connectAttr;
    osThreadAttr_t timeoutAttr;

    g_wifiConnectFinished = 0;

    connectAttr.name = "wifi_connect";
    connectAttr.attr_bits = 0;
    connectAttr.cb_mem = NULL;
    connectAttr.cb_size = 0;
    connectAttr.stack_mem = NULL;
    connectAttr.stack_size = 4096;
    connectAttr.priority = osPriorityBelowNormal;

    timeoutAttr.name = "wifi_timeout";
    timeoutAttr.attr_bits = 0;
    timeoutAttr.cb_mem = NULL;
    timeoutAttr.cb_size = 0;
    timeoutAttr.stack_mem = NULL;
    timeoutAttr.stack_size = 2048;
    timeoutAttr.priority = osPriorityBelowNormal;

    if (osThreadNew(WifiConnectTask, NULL, &connectAttr) == NULL) {
        printf("wifi thread create failed\r\n");
        return;
    }
    if (osThreadNew(WifiTimeoutTask, NULL, &timeoutAttr) == NULL) {
        printf("wifi timeout thread create failed\r\n");
    }
}
