#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/ip4_addr.h"
#include "lwip/api_shell.h"

#include "cmsis_os2.h"
#include "app_time.h"
#include "hos_types.h"
#include "wifi_device.h"
#include "wifiiot_errno.h"
#include "ohos_init.h"
#include "wifi_connect.h"

#define DEF_TIMEOUT 15
#define ONE_SECOND 1
#define WIFI_SCAN_RETRY_MAX 3
#define WIFI_SCAN_RETRY_DELAY_MS 2000
#define DHCP_TIMEOUT_MS 20000
#define DHCP_POLL_INTERVAL_MS 1000

#define SELECT_WIFI_SECURITYTYPE WIFI_SEC_TYPE_PSK
#define SELECT_WLAN_PORT "wlan0"

static void WiFiInit(void);
static void WaitSacnResult(void);
static int WaitConnectResult(void);
static void OnWifiScanStateChangedHandler(int state, int size);
static void OnWifiConnectionChangedHandler(int state, WifiLinkedInfo *info);
static void OnHotspotStaJoinHandler(StationInfo *info);
static void OnHotspotStateChangedHandler(int state);
static void OnHotspotStaLeaveHandler(StationInfo *info);

static int g_staScanSuccess = 0;
static int g_ConnectSuccess = 0;
static int ssid_count = 0;
static WifiStaStartCallback g_staStartCallback;
WifiEvent g_wifiEventHandler = {0};
WifiErrorCode error;

void WifiConnectSetStaStartCallback(WifiStaStartCallback callback)
{
    g_staStartCallback = callback;
}

int WifiConnect(const char *ssid, const char *psk)
{
    WifiScanInfo *info = NULL;
    unsigned int size = WIFI_SCAN_HOTSPOT_LIMIT;
    struct netif *lwipNetif = NULL;
    WifiErrorCode enableRet;
    unsigned int retryCount = 0;
    int selectedIndex = -1;

    osDelay(AppMsToTicks(200U));
    printf("<--System Init-->\r\n");

    WiFiInit();

    enableRet = EnableWifi();
    if (enableRet != WIFI_SUCCESS) {
        printf("EnableWifi failed, ret=%d\r\n", enableRet);
        if (g_staStartCallback != NULL) {
            g_staStartCallback((int)enableRet);
        }
        return -1;
    }
    if (g_staStartCallback != NULL) {
        g_staStartCallback(WIFI_SUCCESS);
    }

    if (IsWifiActive() == 0) {
        printf("Wifi station is not actived.\r\n");
        return -1;
    }

    info = malloc(sizeof(WifiScanInfo) * WIFI_SCAN_HOTSPOT_LIMIT);
    if (info == NULL) {
        return -1;
    }

    while (selectedIndex < 0) {
        ssid_count = 0;
        g_staScanSuccess = 0;
        size = WIFI_SCAN_HOTSPOT_LIMIT;
        Scan();
        WaitSacnResult();

        if (g_staScanSuccess == 1) {
            error = GetScanInfoList(info, &size);
            if (error == WIFI_SUCCESS) {
                printf("********************\r\n");
                for (uint8_t i = 0; i < ssid_count; i++) {
                    printf("no:%03d, ssid:%-30s, rssi:%5d\r\n",
                        i + 1, info[i].ssid, info[i].rssi / 100);
                    if (strcmp(ssid, info[i].ssid) == 0) {
                        selectedIndex = (int)i;
                    }
                }
                printf("********************\r\n");
            }
        }

        if (selectedIndex >= 0) {
            break;
        }
        if (retryCount >= WIFI_SCAN_RETRY_MAX) {
            printf("ERROR: No wifi as expected\r\n");
            free(info);
            return -1;
        }
        retryCount++;
        printf("WiFi scan retry %u/%u\r\n", retryCount, WIFI_SCAN_RETRY_MAX);
        osDelay(AppMsToTicks(WIFI_SCAN_RETRY_DELAY_MS));
    }

    {
        int result;
        WifiDeviceConfig selectApConfig = {0};

        printf("Select:%3d wireless, Waiting...\r\n", selectedIndex + 1);
        strcpy(selectApConfig.ssid, info[selectedIndex].ssid);
        strcpy(selectApConfig.preSharedKey, psk);
        selectApConfig.securityType = SELECT_WIFI_SECURITYTYPE;

        if (AddDeviceConfig(&selectApConfig, &result) != WIFI_SUCCESS) {
            free(info);
            return -1;
        }
        if (ConnectTo(result) != WIFI_SUCCESS || WaitConnectResult() != 1) {
            free(info);
            return -1;
        }

        printf("WiFi connect succeed!\r\n");
        lwipNetif = netifapi_netif_find(SELECT_WLAN_PORT);
    }
    free(info);

    if (lwipNetif == NULL) {
        printf("wlan0 unavailable\r\n");
        return -1;
    }

    dhcp_start(lwipNetif);
    printf("begain to dhcp\r\n");

    {
        unsigned int dhcpElapsed = 0;
        while (dhcpElapsed < DHCP_TIMEOUT_MS) {
            if (dhcp_is_bound(lwipNetif) == ERR_OK) {
                printf("<-- DHCP state:OK -->\r\n");
                netifapi_netif_common(lwipNetif, dhcp_clients_info_show, NULL);
                osDelay(AppMsToTicks(100U));
                return 0;
            }

            printf("<-- DHCP state:Inprogress -->\r\n");
            osDelay(AppMsToTicks(DHCP_POLL_INTERVAL_MS));
            dhcpElapsed += DHCP_POLL_INTERVAL_MS;
        }
    }

    printf("dhcp timeout\r\n");
    return -1;
}

static void WiFiInit(void)
{
    printf("<--Wifi Init-->\r\n");
    g_wifiEventHandler.OnWifiScanStateChanged = OnWifiScanStateChangedHandler;
    g_wifiEventHandler.OnWifiConnectionChanged = OnWifiConnectionChangedHandler;
    g_wifiEventHandler.OnHotspotStaJoin = OnHotspotStaJoinHandler;
    g_wifiEventHandler.OnHotspotStaLeave = OnHotspotStaLeaveHandler;
    g_wifiEventHandler.OnHotspotStateChanged = OnHotspotStateChangedHandler;
    error = RegisterWifiEvent(&g_wifiEventHandler);
    if (error != WIFI_SUCCESS) {
        printf("register wifi event fail!\r\n");
    } else {
        printf("register wifi event succeed!\r\n");
    }
}

static void OnWifiScanStateChangedHandler(int state, int size)
{
    if (size > 0) {
        ssid_count = size;
        g_staScanSuccess = 1;
    }
    printf("callback function for wifi scan:%d, %d\r\n", state, size);
}

static void OnWifiConnectionChangedHandler(int state, WifiLinkedInfo *info)
{
    if (info == NULL) {
        printf("WifiConnectionChanged:info is null, stat is %d.\r\n", state);
    } else if (state == WIFI_STATE_AVALIABLE) {
        g_ConnectSuccess = 1;
    } else {
        g_ConnectSuccess = 0;
    }
}

static void OnHotspotStaJoinHandler(StationInfo *info)
{
    (void)info;
    printf("STA join AP\r\n");
}

static void OnHotspotStaLeaveHandler(StationInfo *info)
{
    (void)info;
    printf("HotspotStaLeave:info is null.\r\n");
}

static void OnHotspotStateChangedHandler(int state)
{
    printf("HotspotStateChanged:state is %d.\r\n", state);
}

static void WaitSacnResult(void)
{
    int scanTimeout = DEF_TIMEOUT;
    while (scanTimeout > 0) {
        sleep(ONE_SECOND);
        scanTimeout--;
        if (g_staScanSuccess == 1) {
            printf("WaitSacnResult:wait success[%d]s\r\n", DEF_TIMEOUT - scanTimeout);
            break;
        }
    }
    if (scanTimeout <= 0) {
        printf("WaitSacnResult:timeout!\r\n");
    }
}

static int WaitConnectResult(void)
{
    int connectTimeout = DEF_TIMEOUT;
    while (connectTimeout > 0) {
        sleep(ONE_SECOND);
        connectTimeout--;
        if (g_ConnectSuccess == 1) {
            printf("WaitConnectResult:wait success[%d]s\r\n", DEF_TIMEOUT - connectTimeout);
            break;
        }
    }
    if (connectTimeout <= 0) {
        printf("WaitConnectResult:timeout!\r\n");
        return 0;
    }
    return 1;
}
