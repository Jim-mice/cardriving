#ifndef __WIFI_CONNECT_H__
#define __WIFI_CONNECT_H__

typedef void (*WifiStaStartCallback)(int result);

/* Called immediately after EnableWifi() returns. */
void WifiConnectSetStaStartCallback(WifiStaStartCallback callback);
int WifiConnect(const char *ssid, const char *psk);

#endif /* __WIFI_CONNECT_H__ */
