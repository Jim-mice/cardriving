#include <stdio.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "task_wifi.h"
#include "wifi_connect.h"

#define WIFI_STA_SSID       "hamster"
#define WIFI_STA_PASSWORD   "12345678"

/* 0: waiting, 1: EnableWifi succeeded, -1: EnableWifi failed. */
static volatile int g_wifiStaStartState;
static volatile int g_wifiNetworkReady;

static void WifiStaStartNotify(int result)
{
    if (result == 0) {
        g_wifiStaStartState = 1;
    } else {
        g_wifiStaStartState = -1;
    }
}

static void PrintIpv4Address(const char *name, const ip4_addr_t *address)
{
    printf("%s: %u.%u.%u.%u\r\n", name, (unsigned int)ip4_addr1(address),
        (unsigned int)ip4_addr2(address), (unsigned int)ip4_addr3(address),
        (unsigned int)ip4_addr4(address));
}

static void PrintWifiIp(void)
{
    struct netif *netif = netifapi_netif_find("wlan0");
    ip4_addr_t ip = {0};
    ip4_addr_t mask = {0};
    ip4_addr_t gateway = {0};

    if (netif == NULL) {
        printf("ip: unavailable\r\n");
        return;
    }

    if (netifapi_netif_get_addr(netif, &ip, &mask, &gateway) != ERR_OK) {
        printf("ip: unavailable\r\n");
        return;
    }

    PrintIpv4Address("ip", &ip);
    PrintIpv4Address("mask", &mask);
    PrintIpv4Address("gateway", &gateway);
}

static void WifiConnectTask(void *argument)
{
    int result;
    (void)argument;

    printf("wifi connecting...\r\n");
    result = WifiConnect(WIFI_STA_SSID, WIFI_STA_PASSWORD);

    if (result == 0) {
        g_wifiNetworkReady = 1;
        printf("wifi connected\r\n");
        PrintWifiIp();
    } else {
        g_wifiNetworkReady = 0;
        printf("wifi connect failed\r\n");
    }
}

void TaskWifiInit(void)
{
    osThreadAttr_t connectAttr;

    g_wifiStaStartState = 0;
    g_wifiNetworkReady = 0;
    WifiConnectSetStaStartCallback(WifiStaStartNotify);

    connectAttr.name = "wifi_connect";
    connectAttr.attr_bits = 0;
    connectAttr.cb_mem = NULL;
    connectAttr.cb_size = 0;
    connectAttr.stack_mem = NULL;
    connectAttr.stack_size = 4096;
    connectAttr.priority = osPriorityBelowNormal;

    if (osThreadNew(WifiConnectTask, NULL, &connectAttr) == NULL) {
        printf("wifi thread create failed\r\n");
    }
}

int TaskWifiWaitStaStarted(uint32_t timeoutMs)
{
    uint32_t elapsed = 0;

    while (g_wifiStaStartState == 0 && elapsed < timeoutMs) {
        osDelay(AppMsToTicks(10U));
        elapsed += 10;
    }

    return (int)g_wifiStaStartState;
}

int TaskWifiIsNetworkReady(void)
{
    return (int)g_wifiNetworkReady;
}
