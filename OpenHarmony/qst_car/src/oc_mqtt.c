
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "MQTTClient.h"
#include <unistd.h>
#include "cJSON.h"
#include "cmsis_os2.h"
#include <oc_mqtt.h>
#include <oc_mqtt_profile_package.h>

typedef struct
{
    char                        *device_id;
    fn_oc_mqtt_profile_rcvdeal   rcvfunc;
}oc_mqtt_profile_cb_t;

static oc_mqtt_profile_cb_t s_oc_mqtt_profile_cb;
static int init_ok;
static MQTTClient mq_client;
struct bp_oc_info oc_info;
struct oc_device
{
    struct bp_oc_info *oc_info;

    void(*cmd_rsp_cb)(uint8_t *recv_data, size_t recv_size, uint8_t **resp_data, size_t *resp_size);

} oc_mqtt;

 void mqtt_callback(MessageData *msg_data)
{
    size_t res_len = 0;
    uint8_t *response_buf = NULL;
    char topicname[45] = { "$crsp/" };

    LOS_ASSERT(msg_data);

    //LOG_D("topic %.*s receive a message", msg_data->topicName->lenstring.len, msg_data->topicName->lenstring.data);

    //LOG_D("message length is %d", msg_data->message->payloadlen);

    if (oc_mqtt.cmd_rsp_cb != NULL)
    {
        oc_mqtt.cmd_rsp_cb((uint8_t *) msg_data->message->payload, msg_data->message->payloadlen, &response_buf,
                &res_len);

        if (response_buf != NULL || res_len != 0)
        {
            strncat(topicname, &(msg_data->topicName->lenstring.data[6]), msg_data->topicName->lenstring.len - 6);

            oc_mqtt_publish(topicname, response_buf, strlen((const char *)response_buf),(int)en_mqtt_al_qos_1);

            free(response_buf);

        }

    }

}
unsigned char *oc_mqtt_buf;
unsigned char *oc_mqtt_readbuf;
int buf_size;

Network n;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;  

static void OCMqttReleaseResources(void)
{
    if (oc_mqtt_buf != NULL) {
        free(oc_mqtt_buf);
        oc_mqtt_buf = NULL;
    }
    if (oc_mqtt_readbuf != NULL) {
        free(oc_mqtt_readbuf);
        oc_mqtt_readbuf = NULL;
    }
}

static int OCMqttCopyString(char *destination, size_t destinationSize,
    const char *source)
{
    size_t length;

    if (destination == NULL || source == NULL) {
        return -1;
    }
    length = strlen(source);
    if (length >= destinationSize) {
        return -1;
    }
    memcpy(destination, source, length + 1U);
    return 0;
}

static int oc_mqtt_entry(void)
{
    int rc;

    NetworkInit(&n);
    rc = NetworkConnect(&n, OC_SERVER_IP, OC_SERVER_PORT);
    printf("cloud mqtt NetworkConnect ret=%d\r\n", rc);
    if (rc != 0) {
        return (int)en_oc_mqtt_err_network;
    }

    buf_size = 2048;
    oc_mqtt_buf = (unsigned char *)malloc((size_t)buf_size);
    oc_mqtt_readbuf = (unsigned char *)malloc((size_t)buf_size);
    if (oc_mqtt_buf == NULL || oc_mqtt_readbuf == NULL) {
        printf("cloud mqtt buffer allocation failed\r\n");
        OCMqttReleaseResources();
        NetworkDisconnect(&n);
        return (int)en_oc_mqtt_err_sysmem;
    }

    MQTTClientInit(&mq_client, &n, 1000U, oc_mqtt_buf, (size_t)buf_size,
        oc_mqtt_readbuf, (size_t)buf_size);
    data.keepAliveInterval = 30;
    data.cleansession = 1;
    data.clientID.cstring = oc_info.client_id;
    data.username.cstring = oc_info.username;
    data.password.cstring = oc_info.password;
    data.MQTTVersion = 3;
    mq_client.defaultMessageHandler = mqtt_callback;

    rc = MQTTConnect(&mq_client, &data);
    printf("cloud mqtt MQTTConnect ret=%d\r\n", rc);
    if (rc != 0) {
        OCMqttReleaseResources();
        NetworkDisconnect(&n);
        return rc;
    }

    rc = MQTTStartTask(&mq_client);
    if (rc == 0) {
        printf("cloud mqtt MQTTStartTask failed\r\n");
        MQTTDisconnect(&mq_client);
        OCMqttReleaseResources();
        NetworkDisconnect(&n);
        return (int)en_oc_mqtt_err_system;
    }
    return 0;
}

int device_info_init(const char *client_id, const char *username,
    const char *password)
{
    if (OCMqttCopyString(oc_info.client_id, sizeof(oc_info.client_id), client_id) != 0 ||
        OCMqttCopyString(oc_info.username, sizeof(oc_info.username), username) != 0 ||
        OCMqttCopyString(oc_info.password, sizeof(oc_info.password), password) != 0) {
        return -1;
    }

    oc_info.user_device_id_flg = 1;
    return 0;
}

/**
 * oc mqtt client init.
 *
 * @param   NULL
 *
 * @return  0 : init success
 *         -1 : get device info fail
 *         -2 : oc mqtt client init fail
 */
int oc_mqtt_init(void)
{
    int result;

    if (init_ok != 0) {
        return 0;
    }

    result = oc_mqtt_entry();
    if (result != 0) {
        printf("cloud mqtt init failed: %d\r\n", result);
        return result;
    }

    init_ok = 1;
    return 0;
}
/**
 * set the command responses call back function
 *
 * @param   cmd_rsp_cb  command responses call back function
 *
 * @return  0 : set success
 *         -1 : function is null
 */
void oc_set_cmd_rsp_cb(void (*cmd_rsp_cb)(uint8_t *recv_data, uint32_t recv_size, uint8_t **resp_data, uint32_t *resp_size))
{

    oc_mqtt.cmd_rsp_cb = cmd_rsp_cb;

}


/**
 * mqtt publish msg to topic
 *
 * @param   topic   target topic
 * @param   msg     message to be sent
 * @param   len     message length
 *
 * @return  0 : publish success
 *         -1 : publish fail
 */
int oc_mqtt_publish(char *topic, uint8_t *msg, int msg_len, int qos)
{
    MQTTMessage message;
    int rc;

    if (topic == NULL || msg == NULL || msg_len < 0) {
        return (int)en_oc_mqtt_err_parafmt;
    }
    if (init_ok == 0 || MQTTIsConnected(&mq_client) == 0) {
        return (int)en_oc_mqtt_err_noconected;
    }

    message.qos = (enum QoS)qos;
    message.retained = 0;
    message.payload = (void *)msg;
    message.payloadlen = (size_t)msg_len;

    rc = MQTTPublish(&mq_client, topic, &message);
    printf("cloud mqtt publish ret=%d\r\n", rc);
    return (rc == 0) ? (int)en_oc_mqtt_err_ok : (int)en_oc_mqtt_err_publish;
}
///< use this function to make a topic to publish
///< if request_id  is needed depends on the fmt
static char *topic_make(const char *fmt, const char *device_id,
    const char *request_id)
{
    int length;
    char *ret;

    if (fmt == NULL || device_id == NULL) {
        return NULL;
    }

    if (request_id == NULL) {
        length = snprintf(NULL, 0U, fmt, device_id);
    } else {
        length = snprintf(NULL, 0U, fmt, device_id, request_id);
    }
    if (length < 0) {
        return NULL;
    }

    ret = (char *)malloc((size_t)length + 1U);
    if (ret == NULL) {
        return NULL;
    }

    if (request_id == NULL) {
        (void)snprintf(ret, (size_t)length + 1U, fmt, device_id);
    } else {
        (void)snprintf(ret, (size_t)length + 1U, fmt, device_id, request_id);
    }
    return ret;
}


///< use this function to report the messsage
#define CN_OC_MQTT_PROFILE_MSGUP_TOPICFMT   "$oc/devices/%s/sys/messages/up"
int oc_mqtt_profile_msgup(char *deviceid,oc_mqtt_profile_msgup_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->msg))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_MSGUP_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_msgup(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT   "$oc/devices/%s/sys/properties/report"
int oc_mqtt_profile_propertyreport(char *deviceid,oc_mqtt_profile_service_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL== payload->service_id) || (NULL == payload->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_PROPERTYREPORT_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_propertyreport(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_GWPROPERTYREPORT_TOPICFMT   "$oc/devices/%s/sys/gateway/sub_devices/properties/report"
int oc_mqtt_profile_gwpropertyreport(char *deviceid,oc_mqtt_profile_device_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL == payload->subdevice_id)||(NULL== payload->subdevice_property) ||\
       (NULL== payload->subdevice_property->service_id)||(NULL== payload->subdevice_property->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_GWPROPERTYREPORT_TOPICFMT, deviceid,NULL);
    msg = oc_mqtt_profile_package_gwpropertyreport(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}


#define CN_OC_MQTT_PROFILE_ROPERTYSETRESP_TOPICFMT   "$oc/devices/%s/sys/properties/set/response/request_id=%s"
int oc_mqtt_profile_propertysetresp(char *deviceid,oc_mqtt_profile_propertysetresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->request_id))
    {
        return ret;
    }
    topic = topic_make(CN_OC_MQTT_PROFILE_ROPERTYSETRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_propertysetresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}


#define CN_OC_MQTT_PROFILE_ROPERTYGETRESP_TOPICFMT   "$oc/devices/%s/sys/properties/get/response/request_id=%s"
int oc_mqtt_profile_propertygetresp(char *deviceid,oc_mqtt_profile_propertygetresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL== payload) || (NULL == payload->request_id) || \
       (NULL== payload->services->service_id) || (NULL == payload->services->service_property))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_ROPERTYGETRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_propertygetresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}

#define CN_OC_MQTT_PROFILE_CMDRESP_TOPICFMT   "$oc/devices/%s/sys/commands/response/request_id=%s"
int oc_mqtt_profile_cmdresp(char *deviceid,oc_mqtt_profile_cmdresp_t *payload)
{
    int ret = (int)en_oc_mqtt_err_parafmt;
    char *topic;
    char *msg;

    if(NULL == deviceid)
    {
        if(NULL == s_oc_mqtt_profile_cb.device_id)
        {
            return ret;
        }
        else
        {
            deviceid = s_oc_mqtt_profile_cb.device_id;
        }
    }

    if((NULL == payload) || (NULL == payload->request_id))
    {
        return ret;
    }

    topic = topic_make(CN_OC_MQTT_PROFILE_CMDRESP_TOPICFMT, deviceid,payload->request_id);
    msg = oc_mqtt_profile_package_cmdresp(payload);

    if((NULL != topic) && (NULL != msg))
    {
        ret = oc_mqtt_publish(topic,(uint8_t *)msg,strlen(msg),(int)en_mqtt_al_qos_1);
    }
    else
    {
        ret = (int)en_oc_mqtt_err_sysmem;
    }

    free(topic);
    free(msg);

    return ret;
}
