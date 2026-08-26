---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 81916c3ab2f20f66c1939a508c5615cb_a7c4ad0aa0fb11f1a65b525400826444
    ReservedCode1: T8crJtJnudtNK79cFNIeA7WYYdtImFlF84Wc7y2YrLoksZ0XaGVEstLyOw/2V2S2MjPnBD72IvBtX4AwSojIi//ytKgFvXN2EJTozjph+B/z9P7jCWur+ZH88KGQfhLx+XI1phDpgl9dUPtBLDq5RdVUYUgciy0eO8IDcqZPh+QoqBQKZk6gLXx8a1w=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 81916c3ab2f20f66c1939a508c5615cb_a7c4ad0aa0fb11f1a65b525400826444
    ReservedCode2: T8crJtJnudtNK79cFNIeA7WYYdtImFlF84Wc7y2YrLoksZ0XaGVEstLyOw/2V2S2MjPnBD72IvBtX4AwSojIi//ytKgFvXN2EJTozjph+B/z9P7jCWur+ZH88KGQfhLx+XI1phDpgl9dUPtBLDq5RdVUYUgciy0eO8IDcqZPh+QoqBQKZk6gLXx8a1w=
---



# 华为云 MQTT 控制实现过程

> 实训任务 28：第三阶段综合实验——QST"先锋号"综合小车
> 实现目标：小车传感器状态上报华为云；运动模式（超声波避障 / 红外循迹 / 蓝牙遥控）由华为云平台下发指令远程切换。
> 本文档基于本地 GitHub 仓库 `Jim-mice/cardriving`（`Hi3861/QST_car` 与 `STM32` 目录）的真实代码撰写。

---

## 1. 任务概述

### 1.1 系统架构

小车采用 Hi3861 + STM32 双 MCU 架构：

| 模块 | 职责 |
|---|---|
| Hi3861 | WiFi 连接、MQTT 接入华为云 IoTDA、传感器采集（超声波/红外/光照/温湿度）、决策与运动模式管理 |
| STM32F103C8T6 | 电机执行层：解析 Hi3861 下发的 UART 帧，驱动 L9110S 双电机（含 WS2812 灯带等） |
| 通信链路 | Hi3861 UART2（GPIO11/GPIO12） ↔ STM32 USART1（PA9/PA10），115200-8-N-1 |

### 1.2 任务要求

1. 小车将传感器状态（超声波距离、红外循迹电平、环境光/温湿度等）定时上报华为云平台；
2. 华为云平台下发运动模式指令（超声波 / 红外 / 蓝牙），Hi3861 接收后切换对应运动逻辑；
3. STM32 侧接收 Hi3861 指令执行对应电机动作。

### 1.3 数据链路全景

```
华为云 IoTDA（MQTT Broker）
   │  ▲ 属性上报: $oc/devices/{device_id}/sys/properties/report
   │  │ 命令下发: $oc/devices/{device_id}/sys/commands/request_id={rid}
   ▼  │ 命令应答: $oc/devices/{device_id}/sys/commands/response/request_id={rid}
Hi3861（WiFi → Paho MQTT → oc_mqtt 封装）
   │  UART2 帧协议: FC | A_dir | motorA | B_dir | motorB | FD
   ▼
STM32F103C8T6（USART1 中断收帧 → 解析 → L9110S 电机）
```

---

## 2. 华为云平台侧配置步骤

### 2.1 注册账号与实名认证

1. 访问华为云官网 https://www.huaweicloud.com ，注册华为云账号；
2. 进入"账号中心"完成**实名认证**（个人/企业均可），未实名认证无法使用 IoTDA 设备接入服务。

### 2.2 开通 IoTDA 设备接入服务

1. 控制台搜索"设备接入 IoTDA"，进入服务页面；
2. 选择**基础版**（免费）即可满足实训需求，点击"开通"；
3. 记录**接入地址**：本仓库代码中使用的区域为华北-北京四（cn-north-4）：

```c
// oc_mqtt.h
#define OC_SERVER_IP  "01bf5d8f97.iot-mqtts.cn-north-4.myhuaweicloud.com"
#define OC_SERVER_PORT 1883
```

> 说明：代码中 `OC_SERVER_URL`（`tcp://183.230.40.39:6002`）为旧 OneNET 地址，已被注释；实际连接使用 `OC_SERVER_IP` 指向的华为云 IoTDA 华北-北京四接入点，端口 1883（MQTT 明文端口）。

### 2.3 创建产品与产品模型

1. 进入 IoTDA 控制台 → "产品" → "创建产品"：
   - 产品名称：`QST_car`（示例）
   - 协议类型：**MQTT**
   - 数据格式：**JSON**
   - 所属行业/设备类型按实训要求填写；
2. 在产品详情中"模型定义"页，按上报数据与命令需求**添加服务（Service）**与属性/命令：

**服务建议划分（需与 Hi3861 上报字段保持一致）：**

| 服务 ID | 属性/命令 | 数据类型 | 说明 |
|---|---|---|---|
| `SensorData` | `distance` | int | 超声波距离（cm） |
| | `trace_left` / `trace_right` | int | 红外循迹左右电平 |
| | `lux` | decimal | 环境光照（AP3216C） |
| | `temperature` / `humidity` | decimal | SHT20 温湿度 |
| `ModeControl` | 命令 `SetMode` | string | 下发运动模式：`ultrasonic` / `infrared` / `bluetooth` |

3. 产品模型（ProductModel）定义完成后，平台即可识别设备上报的属性与下发的命令，这是命令/属性 Topic 正常工作的前提。

### 2.4 注册设备获取三元组

1. 产品详情 → "设备" → "注册设备"：
   - 设备标识码（nodeId）：自定，如 `qst_car_01`；
   - 设备名称：`qst_car_01`；
2. 注册成功后，在设备详情中查看 **设备ID（device_id）**，并**下载设备密钥（secret）**；
3. 使用华为云官方工具（如"华为云 IoTDA 设备接入模拟器"或 `iotda-device-demo` 中的 Python 脚本）计算 MQTT 连接三元组：

- **ClientId**：`{device_id}_0_0_时间戳`
- **Username**：`{device_id}`
- **Password**：使用 HMAC-SHA256 以密钥对时间戳签名，公式：`HmacSHA256(secret, 时间戳)` 转为小写十六进制

将算出的三元组写入 Hi3861 代码：

```c
// 主程序（应用层）调用，示例值，请替换为实际三元组
device_info_init(
    "65xxxxxxxxxxxxxx_0_0_20260826120000",   // client_id
    "65xxxxxxxxxxxxxx",                       // username
    "a1b2c3d4e5f6...小写hex签名"               // password
);
```

### 2.5 上报属性与命令 Topic 说明

| 方向 | Topic | 说明 |
|---|---|---|
| 上报属性 | `$oc/devices/{device_id}/sys/properties/report` | 设备上报属性（本任务核心上报通道） |
| 平台下发命令 | `$oc/devices/{device_id}/sys/commands/request_id={request_id}` | 平台向设备下发命令 |
| 命令应答 | `$oc/devices/{device_id}/sys/commands/response/request_id={request_id}` | 设备对命令的执行结果应答 |
| 消息上报 | `$oc/devices/{device_id}/sys/messages/up` | 自定义消息（非结构化） |
| 属性设置应答 | `$oc/devices/{device_id}/sys/properties/set/response/request_id={request_id}` | 属性设置命令应答 |

仓库 `oc_mqtt.c` 中对应宏定义：

```c
// oc_mqtt.c
#define CN_OC_MQTT_PROFILE_MSGUP_TOPICFMT            "$oc/devices/%s/sys/messages/up"
#define CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT   "$oc/devices/%s/sys/properties/report"
#define CN_OC_MQTT_PROFILE_CMDRESP_TOPICFMT          "$oc/devices/%s/sys/commands/response/request_id=%s"
```

> 平台下发的命令 Topic 无需显式订阅：Hi3861 使用 Paho MQTT 的默认消息回调 `mqtt_callback` 接收所有下行消息（见 3.2 节），通过解析 Topic 前缀区分消息类型。

---

## 3. Hi3861 端 MQTT 实现

### 3.1 WiFi 连接

仓库 `wifi_connect.c` 提供 `WifiConnect(ssid, psk)`：

```c
// wifi_connect.c（节选）
int WifiConnect(const char *ssid, const char *psk)
{
    // 1. 初始化并注册 WiFi 事件
    WiFiInit();
    // 2. 使能 STA 模式
    EnableWifi();
    // 3. 扫描周边热点，匹配目标 SSID
    Scan();
    GetScanInfoList(info, &size);
    // 4. 匹配到目标 SSID 后配置 WPA2-PSK 并连接
    AddDeviceConfig(&select_ap_config, &result);
    ConnectTo(result);
    WaitConnectResult();
    // 5. 连接成功后启动 DHCP 获取 IP
    dhcp_start(g_lwip_netif);
    // 6. 等待 DHCP 绑定完成
    if(dhcp_is_bound(g_lwip_netif) == ERR_OK) { ... }
}
```

应用层调用：

```c
WifiConnect("实训WiFi名称", "WiFi密码");   // 返回 0 表示联网成功
```

### 3.2 MQTT 连接配置（IoTDA 接入）

`oc_mqtt.c` 的 `oc_mqtt_entry()` 完成 MQTT 客户端初始化与连接：

```c
// oc_mqtt.c（节选）
static int oc_mqtt_entry(void)
{
    int rc = 0;
    NetworkInit(&n);
    NetworkConnect(&n, OC_SERVER_IP, OC_SERVER_PORT);   // 连接华为云接入点

    buf_size = 2048;
    oc_mqtt_buf = (unsigned char *) malloc(buf_size);
    oc_mqtt_readbuf = (unsigned char *) malloc(buf_size);

    MQTTClientInit(&mq_client, &n, 1000, oc_mqtt_buf, buf_size, oc_mqtt_readbuf, buf_size);
    MQTTStartTask(&mq_client);

    data.keepAliveInterval = 30;        // 心跳 30s
    data.cleansession = 1;              // 干净会话
    data.clientID.cstring = oc_info.client_id;
    data.username.cstring = oc_info.username;
    data.password.cstring = oc_info.password;
    data.MQTTVersion = 3;               // MQTT 3.1.1

    mq_client.defaultMessageHandler = mqtt_callback;   // 注册默认消息回调
    rc = MQTTConnect(&mq_client, &data);
    return rc;
}
```

对外封装（供应用层调用）：

```c
// oc_mqtt.h / oc_mqtt.c
int  oc_mqtt_init(void);                        // 初始化并连接 MQTT
void device_info_init(char *client_id, char *username, char *password);  // 写入三元组
void oc_set_cmd_rsp_cb(void (*cb)(uint8_t*, uint32_t, uint8_t**, uint32_t*)); // 注册命令回调
int  oc_mqtt_publish(char *topic, uint8_t *msg, int msg_len, int qos);   // 底层发布
```

**启动流程（应用层建议顺序）：**

```c
Peripheral_Init();            // 外设初始化（UART2/传感器等）
WifiConnect(SSID, PSK);       // 1. WiFi 联网
device_info_init(client_id, username, password);  // 2. 填入华为云三元组
oc_set_cmd_rsp_cb(cmd_rsp_cb); // 3. 注册命令处理回调
oc_mqtt_init();               // 4. 建立 MQTT 连接
// 5. 创建传感器上报任务 + 运动模式任务（见 3.4 / 3.5）
```

### 3.3 命令 Topic 订阅与解析

#### 3.3.1 默认消息回调

Hi3861 侧通过 Paho MQTT 的 `defaultMessageHandler` 接收下行消息，`oc_mqtt.c` 中已实现 `mqtt_callback`：

```c
// oc_mqtt.c（节选）
void mqtt_callback(MessageData *msg_data)
{
    size_t res_len = 0;
    uint8_t *response_buf = NULL;
    char topicname[45] = { "$crsp/" };

    if (oc_mqtt.cmd_rsp_cb != NULL)
    {
        // 把平台下发消息交给应用层回调处理
        oc_mqtt.cmd_rsp_cb((uint8_t *)msg_data->message->payload,
                           msg_data->message->payloadlen,
                           &response_buf, &res_len);
        // 若回调生成了应答内容，则发布到 $crsp/{request_id} 即命令应答 Topic
        if (response_buf != NULL || res_len != 0)
        {
            strncat(topicname, &(msg_data->topicName->lenstring.data[6]),
                    msg_data->topicName->lenstring.len - 6);
            oc_mqtt_publish(topicname, response_buf,
                            strlen((const char *)response_buf), en_mqtt_al_qos_1);
            free(response_buf);
        }
    }
}
```

机制说明：
- 平台下发命令 Topic 为 `$oc/devices/{device_id}/sys/commands/request_id={rid}`；
- 回调中 `cmd_rsp_cb` 返回应答 JSON 后，代码将 Topic 第 6 个字符起的 `{rid}` 拼接到 `$crsp/` 后，等价于发布到 `$oc/devices/{device_id}/sys/commands/response/request_id={rid}`；
- 因此**应用层只需注册 `cmd_rsp_cb` 并返回应答内容**，Topic 组包由库完成。

#### 3.3.2 应用层命令解析（模式切换核心）

平台下发的命令 Payload 为 JSON，例如：

```json
{
  "object_device_id": "65xxxxxxxxxx_qst_car_01",
  "command_name": "SetMode",
  "service_id": "ModeControl",
  "paras": { "mode": "ultrasonic" }
}
```

应用层 `cmd_rsp_cb` 建议实现（基于仓库 API）：

```c
// 建议实现：应用层命令处理回调（仓库主入口未包含，此处为基于 oc_mqtt 框架的示例）
static void cmd_rsp_cb(uint8_t *recv_data, uint32_t recv_size,
                       uint8_t **resp_data, uint32_t *resp_size)
{
    cJSON *root = cJSON_Parse((char *)recv_data);
    cJSON *command_name = cJSON_GetObjectItem(root, "command_name");
    cJSON *paras = cJSON_GetObjectItem(root, "paras");
    cJSON *mode = paras ? cJSON_GetObjectItem(paras, "mode") : NULL;
    char *resp = NULL;

    if (command_name && strcmp(command_name->valuestring, "SetMode") == 0 && mode)
    {
        if (strcmp(mode->valuestring, "ultrasonic") == 0)
        {
            g_car_status = MODE_ULTRASONIC;   // 切换超声波避障模式
            printf("switch to ultrasonic mode\n");
        }
        else if (strcmp(mode->valuestring, "infrared") == 0)
        {
            g_car_status = MODE_INFRARED;     // 切换红外循迹模式
            printf("switch to infrared mode\n");
        }
        else if (strcmp(mode->valuestring, "bluetooth") == 0)
        {
            g_car_status = MODE_BLUETOOTH;    // 切换蓝牙遥控模式
            printf("switch to bluetooth mode\n");
        }
        // 构造命令应答（result_code: 0 成功）
        resp = (char *)malloc(64);
        snprintf(resp, 64, "{\"result_code\":0,\"response_name\":\"SetMode\",\"paras\":{\"mode\":\"%s\"}}",
                 mode->valuestring);
    }
    else
    {
        resp = (char *)malloc(32);
        snprintf(resp, 32, "{\"result_code\":1,\"response_name\":\"unknown\"}");
    }

    *resp_data = (uint8_t *)resp;
    *resp_size = strlen(resp);
    cJSON_Delete(root);
}
```

> 应答 JSON 结构与 `oc_mqtt_profile_package_cmdresp()`（`result_code` / `response_name` / `paras`）保持一致，也可直接调用该打包函数。

#### 3.3.3 模式切换后的运动逻辑

仓库已实现的两种模式逻辑：

**红外循迹模式**（`trace_model.c`）：读取 GPIO13/GPIO14 红外对管电平，控制左右轮差速：

```c
// trace_model.c（节选）
void timer1_callback(unsigned int arg)
{
    GpioGetInputVal(GPIOL, &io_status_left);
    GpioGetInputVal(GPIOR, &io_status_right);

    if(io_status_right != WIFI_IOT_GPIO_VALUE1 && io_status_left != WIFI_IOT_GPIO_VALUE1){
        car_forward();          // 都在白线内 → 直行
    }
    else if(io_status_right == WIFI_IOT_GPIO_VALUE1 && io_status_left != WIFI_IOT_GPIO_VALUE1){
        car_right_tra();        // 右偏出线 → 右转修正
    }
    else if(io_status_right != WIFI_IOT_GPIO_VALUE1 && io_status_left == WIFI_IOT_GPIO_VALUE1){
        car_left_tra();         // 左偏出线 → 左转修正
    }
    else if(io_status_right == WIFI_IOT_GPIO_VALUE1 && io_status_left == WIFI_IOT_GPIO_VALUE1){
        car_stop();             // 两侧都压线（十字/终点）→ 停止
    }
}

void trace_module(void)
{
    while (1) {
        timer1_callback(0);
        if (sen_or_car_flag != 1 || g_car_status != MODE_INFRARED) {
            printf("trace_module stop!\n");
            car_stop();
            break;
        }
        hi_sleep(30);
    }
}
```

**超声波测距**（`robot_hcsr04.c`）：GPIO7 触发脉冲，GPIO8 测量回响高电平时间，`距离 = 时间 × 0.034 / 2`（cm）：

```c
// robot_hcsr04.c（节选）
float GetDistance(void)
{
    ...
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);   // 触发 20us
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);
    // 等待回响高电平，计时
    ...
    distance = time * 0.034 / 2;    // 距离 = 高电平时间 * 0.034 / 2
    return distance;
}
```

**超声波避障模式（建议实现，复用仓库接口）：**

```c
// 建议实现：超声波避障模式
void ultrasonic_module(void)
{
    while (1) {
        float dist = GetDistance();
        printf("distance: %.1f cm\n", dist);
        if (dist > 0 && dist < 25.0f) {
            car_stop();
            car_backward();          // 后退
            hi_sleep(200);
            car_left();              // 左转避开
            hi_sleep(200);
        } else {
            car_forward();           // 无障碍前进
        }
        if (sen_or_car_flag != 1 || g_car_status != MODE_ULTRASONIC) {
            car_stop();
            break;
        }
        hi_sleep(50);
    }
}
```

**蓝牙遥控模式（建议实现）：** 仓库 `Peripheral.c` 已将 GPIO0/GPIO1 复用为 UART1（蓝牙模块），应用层从 UART1 读取蓝牙指令（如 `F`/`B`/`L`/`R`/`S`）后调用 `car_forward()` 等接口：

```c
// Peripheral.c（节选，UART1 蓝牙串口复用配置）
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
// UartInit(WIFI_IOT_UART_IDX_1, &uart_attr2, NULL);  // 使用蓝牙前取消注释

// 建议实现：蓝牙遥控模式
void bluetooth_module(void)
{
    uint8_t ch;
    while (1) {
        if (UartRead(WIFI_IOT_UART_IDX_1, &ch, 1) > 0) {
            switch (ch) {
                case 'F': car_forward();  break;
                case 'B': car_backward(); break;
                case 'L': car_left();     break;
                case 'R': car_right();    break;
                case 'S': car_stop();     break;
                default: break;
            }
        }
        if (sen_or_car_flag != 1 || g_car_status != MODE_BLUETOOTH) {
            car_stop();
            break;
        }
        hi_sleep(20);
    }
}
```

**模式调度（建议实现，仓库主入口未包含）：** 由命令回调设置 `g_car_status`，统一调度任务：

```c
// 建议实现：运动模式统一调度
void motion_scheduler(void)
{
    while (1) {
        switch (g_car_status) {
            case MODE_ULTRASONIC: ultrasonic_module(); break;
            case MODE_INFRARED:   trace_module();      break;
            case MODE_BLUETOOTH:  bluetooth_module();  break;
            default: hi_sleep(100); break;
        }
        hi_sleep(50);
    }
}
```

> 注：仓库中 `trace_model.c` 通过 `extern` 引用了 `sen_or_car_flag`（0 传感器采集 / 1 小车模式）与 `g_car_status`，这两个全局变量需在应用层主程序中定义。

### 3.4 传感器属性上报

使用 `oc_mqtt_profile_propertyreport()` 上报属性，其内部将属性打包为华为云标准 JSON 并发布到 `$oc/devices/{device_id}/sys/properties/report`：

```c
// oc_mqtt.c（节选）
#define CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT  "$oc/devices/%s/sys/properties/report"
int oc_mqtt_profile_propertyreport(char *deviceid, oc_mqtt_profile_service_t *payload)
{
    ...
    topic = topic_make(CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT, deviceid, NULL);
    msg = oc_mqtt_profile_package_propertyreport(payload);   // cJSON 打包
    ret = oc_mqtt_publish(topic, (uint8_t *)msg, strlen(msg), en_mqtt_al_qos_1);
    ...
}
```

打包后上报消息格式（`oc_mqtt_profile_package.c` 生成）：

```json
{
  "services": [
    {
      "service_id": "SensorData",
      "properties": {
        "distance": 35,
        "trace_left": 1,
        "trace_right": 0,
        "lux": 260.5,
        "temperature": 28.3,
        "humidity": 45.2
      }
    }
  ]
}
```

**属性上报任务（建议实现，基于仓库 API）：**

```c
// 建议实现：传感器周期上报任务
void sensor_report_task(void)
{
    oc_mqtt_profile_kv_t distance, trace_l, trace_r;
    oc_mqtt_profile_kv_t lux, temp, humi;
    oc_mqtt_profile_service_t service;
    char dist_buf[8], lux_buf[8], temp_buf[8], humi_buf[8];

    while (1) {
        float d = GetDistance();                 // 超声波距离
        uint8_t tl, tr;
        GpioGetInputVal(13, &tl); GpioGetInputVal(14, &tr);

        snprintf(dist_buf, sizeof(dist_buf), "%.1f", d);
        snprintf(lux_buf,  sizeof(lux_buf),  "%.1f", GetLux());   // AP3216C 光照（示例函数名）
        snprintf(temp_buf, sizeof(temp_buf), "%.1f", GetTemp());  // SHT20 温度（示例函数名）
        snprintf(humi_buf, sizeof(humi_buf), "%.1f", GetHumi());  // SHT20 湿度（示例函数名）

        // 组装 KV 链表
        distance.key = "distance";     distance.type = EN_OC_MQTT_PROFILE_VALUE_STRING; distance.value = dist_buf; distance.nxt = &trace_l;
        trace_l.key  = "trace_left";   trace_l.type  = EN_OC_MQTT_PROFILE_VALUE_INT;    trace_l.value  = (int *)&tl; trace_l.nxt = &trace_r;
        trace_r.key  = "trace_right";  trace_r.type  = EN_OC_MQTT_PROFILE_VALUE_INT;    trace_r.value  = (int *)&tr; trace_r.nxt = &lux;
        lux.key      = "lux";          lux.type      = EN_OC_MQTT_PROFILE_VALUE_STRING; lux.value     = lux_buf; lux.nxt = &temp;
        temp.key     = "temperature";  temp.type     = EN_OC_MQTT_PROFILE_VALUE_STRING; temp.value    = temp_buf; temp.nxt = &humi;
        humi.key     = "humidity";     humi.type     = EN_OC_MQTT_PROFILE_VALUE_STRING; humi.value    = humi_buf; humi.nxt = NULL;

        service.service_id = "SensorData";
        service.service_property = &distance;
        service.event_time = NULL;
        service.nxt = NULL;

        oc_mqtt_profile_propertyreport(NULL, &service);   // deviceid 传 NULL 使用连接设备
        hi_sleep(2000);                                    // 每 2s 上报一次
    }
}
```

> 说明：仓库 `Peripheral.h` 已定义 `Peripheral_Data_TypeDef`（`Lux` / `Humidity` / `Temperature`），SHT20 与 AP3216C 驱动位于 `hal_bsp_sht20.c` / `hal_bsp_ap3216c.c`，可读取温湿度与光照参与上报；上例中 `GetLux/GetTemp/GetHumi` 为示意函数名，实际请以驱动头文件导出的接口为准。

---

## 4. STM32 端配合说明

### 4.1 UART 帧协议

Hi3861 通过 UART2 下发 6 字节定长帧（`robot_l9110s.c`）：

```c
// robot_l9110s.c（Hi3861 侧发送）
uart_sendbuf[0] = 0xFC;   // 帧头
uart_sendbuf[1] = A_dir;  // 左轮方向：0 正转 / 1 反转
uart_sendbuf[2] = motorA; // 左轮速度（0~150）
uart_sendbuf[3] = B_dir;  // 右轮方向：0 正转 / 1 反转
uart_sendbuf[4] = motorB; // 右轮速度（0~150）
uart_sendbuf[5] = 0xFD;   // 帧尾
UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
```

STM32 侧帧定义（`drv_usart1_hi3861.h`）：

```c
#define USART1_HI3861_FRAME_HEAD 0xFC
#define USART1_HI3861_FRAME_TAIL 0xFD
#define USART1_HI3861_FRAME_LEN  6
```

### 4.2 STM32 接收与解析

`drv_usart1_hi3861.c` 通过 USART1 接收中断按状态机组帧（首字节 0xFC 开始，第 6 字节 0xFD 结束），完整帧放入 `USART1_RX_Buffer` 并置 `USART1_RX_Flag`：

```c
// drv_usart1_hi3861.c（节选：中断组帧逻辑）
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        if (g_rxFrameIndex == 0) {
            if (data == USART1_HI3861_FRAME_HEAD) { g_rxFrame[g_rxFrameIndex++] = data; }
        } else if (g_rxFrameIndex < (USART1_HI3861_FRAME_LEN - 1)) {
            g_rxFrame[g_rxFrameIndex++] = data;
        } else if (data == USART1_HI3861_FRAME_TAIL) {
            // 组帧完成，拷贝到前台缓冲区并置标志
            USART1_RX_Flag = 1; g_rxFrameIndex = 0;
        } ...
    }
}
```

当前 `User/main.c` 已实现收帧打印与 LED 翻转演示，但尚未按帧驱动电机：

```c
// STM32/User/main.c（节选）
while(1)
{
    Light_Run();
    if(USART1_GetReceivedFrame(frame))     // 取到完整 6 字节帧
    {
        USART1_PrintFrame(frame);          // 打印帧内容
        // TODO: 按 frame[1..4] 驱动 L9110S 电机
    }
}
```

**电机执行扩展（建议实现）：** 将收到的帧解析后交给 L9110S 驱动（`bsp_motor_l9110.c` 提供 `motor_left_set / motor_right_set / motor_stop`）：

```c
// 建议实现：根据 Hi3861 帧驱动电机
void Motor_ExecuteFrame(uint8_t *frame)
{
    uint8_t a_dir = frame[1]; uint8_t speed_a = frame[2];
    uint8_t b_dir = frame[3]; uint8_t speed_b = frame[4];

    // 方向为正转时用正值，反转时用负值（L9110S 驱动接口按需适配）
    int16_t left  = a_dir ? -speed_a : speed_a;
    int16_t right = b_dir ? -speed_b : speed_b;

    if (left == 0 && right == 0) {
        motor_stop();
    } else {
        motor_left_set(left);
        motor_right_set(right);
    }
}
```

---

## 5. 联调测试步骤

### 5.1 平台侧准备

1. 华为云 IoTDA 控制台确认产品模型已定义 `SensorData` 服务属性与 `SetMode` 命令；
2. 设备在线（设备列表中状态为"在线"）；
3. 打开设备详情 → "设备影子" / "消息跟踪"，便于观察上报与命令。

### 5.2 联调流程

| 步骤 | 操作 | 预期结果 |
|---|---|---|
| 1 | 烧录 Hi3861 与 STM32 固件，上电 | STM32 串口打印 `STM32 READY` |
| 2 | 观察 Hi3861 日志 | WiFi 扫描 → 连接成功 → DHCP OK → MQTT Connect 成功 |
| 3 | 平台"设备"页面查看 | 设备状态变为**在线** |
| 4 | 等待 2s 上报周期 | 平台"设备详情→设备影子"出现 `SensorData` 属性（distance/lux/temperature 等），数值随传感器变化 |
| 5 | 平台下发命令 `SetMode`，paras.mode=infrared | Hi3861 打印 `switch to infrared mode`；小车开始循迹动作；平台收到命令应答 result_code=0 |
| 6 | 依次下发 ultrasonic / bluetooth | 小车切换为避障 / 蓝牙遥控；切换后旧模式任务退出、新模式任务接管 |
| 7 | 手遮超声波模块 | 上报 distance 值明显变化；避障模式下小车后退转向 |
| 8 | 蓝牙模式下发 F/B/L/R/S | 小车前进/后退/左转/右转/停止 |

### 5.3 常见问题

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| MQTT 连接失败（返回 -5/-6 等） | 三元组计算错误 / 时间戳过期 | 重新生成 ClientId（时间戳更新）与 Password（HMAC-SHA256）；核对 Username 为 device_id |
| WiFi 一直扫描不到热点 | SSID 名称不匹配 / 距离过远 | 确认 `WifiConnect` 入参与热点完全一致；查看扫描打印列表 |
| 设备上线但属性上报无数据 | 产品模型属性名与上报 JSON key 不一致 | 对照 3.4 节 JSON 与平台模型定义逐字段核对 |
| 命令下发小车无动作 | 命令名称/paras 与回调解析不一致 | 开启平台"消息跟踪"，对比实际下发 JSON 与 `cmd_rsp_cb` 解析字段 |
| 小车电机不转 | STM32 侧未实现帧解析驱动 / 帧协议不一致 | 确认 STM32 收到帧打印（`STM32 RX: FC ... FD`）；检查 UART2/USART1 波特率 115200 |
| 切换模式后旧逻辑仍在跑 | 模式任务退出条件未生效 | 确保各模块循环体内检查 `g_car_status` 并在变化时 `car_stop()` + break |
| 平台命令应答超时 | 应答 JSON 格式不符合规范 | 使用 `oc_mqtt_profile_cmdresp()` 或按 `result_code/response_name/paras` 结构手工构造 |

---

## 6. 代码文件索引

| 文件 | 作用 |
|---|---|
| `Hi3861/QST_car/src/wifi_connect.c` | WiFi 连接（扫描/连接/DHCP） |
| `Hi3861/QST_car/src/oc_mqtt.c` | MQTT 连接、默认消息回调、属性上报/命令应答封装 |
| `Hi3861/QST_car/src/oc_mqtt_profile_package.c` | 上报/应答 JSON 打包（cJSON） |
| `Hi3861/QST_car/src/Peripheral.c` | 外设初始化（UART2 通信串口、UART1 蓝牙、I2C、GPIO） |
| `Hi3861/QST_car/src/robot_l9110s.c` | UART2 电机帧发送与小车运动函数 |
| `Hi3861/QST_car/src/trace_model.c` | 红外循迹模式 |
| `Hi3861/QST_car/src/robot_hcsr04.c` | 超声波测距 |
| `Hi3861/QST_car/src/robot_sg90.c` | SG90 舵机（避障辅助） |
| `STM32/Drivers/drv_usart1_hi3861.c/.h` | STM32 USART1 收帧（FC..FD 协议） |
| `STM32/User/main.c` | STM32 主程序（收帧打印，电机驱动待扩展） |
| `STM32/BSP/bsp_motor_l9110.c/.h` | L9110S 电机驱动 |

> 说明：仓库中未包含 Hi3861 应用层主入口（`app_main`）与命令解析函数，第 3.3~3.4 节的模式切换/属性上报示例代码均基于仓库现有 API（`oc_mqtt` / `robot_l9110s` / `trace_model` / `GetDistance` 等）给出建议实现，实际编写时以工程整体为准。
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
