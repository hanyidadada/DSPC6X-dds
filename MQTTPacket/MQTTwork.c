#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <ti/sysbios/knl/Task.h>
#include "MQTTPacket.h"
#include "transport.h"
#include "MQTTwork.h"
#include "driver/c66x_uart.h"

extern uint8_t rx_flag;
extern uint8_t uartbuf[100];
extern uint32_t index;
extern uint8_t err_flag;
extern int8_t err_type;

void MQTTSubscribeWork(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int rc = 0;
    int len = 0;
    int req_qos = 0;
    int req_qos_a[4] = {0};
    int msgid = 1;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString2 = MQTTString_initializer;
    MQTTString topicString[4] = {0};
    /* subscribe */
    topicString[0].cstring = UPPERTOPIC;
    topicString[1].cstring = NODESTATUS;
    topicString[2].cstring = NODERESULT;
    topicString[3].cstring = "test/#";
    len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 4, topicString, req_qos_a);

    transport_sendPacketBuffer(mysock, buf, len);
    if (MQTTPacket_read(buf, buflen, transport_getdata) == SUBACK)  /* wait for suback */
    {
        unsigned short submsgid;
        int subcount;
        int granted_qos;
        int granted_qos_a[4];

        rc = MQTTDeserialize_suback(&submsgid, 4, &subcount, granted_qos_a, buf, buflen);
        if (granted_qos_a[0] != 0 || rc < 0)
        {
            printf("granted qos != 0, %d\n", granted_qos_a[0]);
            goto exit;
        }
    }
    else
        goto exit;

    while (1) {
        /* transport_getdata() has a built-in 1 second timeout,
        your mileage will vary */
        if (MQTTPacket_read(buf, buflen, transport_getdata) == PUBLISH) {
            unsigned char dup;
            int qos;
            unsigned char retained;
            unsigned short msgid;
            int payloadlen_in;
            unsigned char* payload_in;
            MQTTString receivedTopic;

            rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
                    &payload_in, &payloadlen_in, buf, buflen);
            uart_printf("message arrived %.*s \ttopic: %.*s\n", payloadlen_in, payload_in, receivedTopic.lenstring.len, receivedTopic.lenstring.data);
        }
    }
exit:
    printf("Subscribe error!\n");
    fdCloseSession(TaskSelf());
    return ;
}


void MQTTPublishWork(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int len = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    uint32_t wait_time;

    topicString.cstring = MASTERSERIALTOPIC;
    while (1){
        if(rx_flag != 0){
            while (1) {
                if(rx_flag) {
                    rx_flag = 0;
                    /*
                     * wait time set as 1s, base on cpu freq as 1000MHz.
                     * If no interrupt is triggered within 1s,
                     * the file transfer is complete.
                     */
                    wait_time = 10000;
                } else {
                    /* delay 1 cpu cyle */
                    asm(" nop");
                    wait_time--;
                    if(wait_time == 0)
                        break;
                }
                if(err_flag) {
                    uart_printf("uart transmission error occurred,code: %d\n", err_type);
                    return ;
                }
            }
            len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)uartbuf, index);
            if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
                break;
            }
            index = 0;
        }
        Task_sleep(10);
    }
    uart_printf("Publish error!\n");
    fdCloseSession(TaskSelf());
    return ;
}

void MQTTPublish2Work(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int len = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    char *payload = "sensing message test!!";
    int payloadlen = strlen(payload);

    topicString.cstring = "master/result/sensing";
    while (1){
        len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
        if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
            break;
        }
        Task_sleep(1000);
    }
    uart_printf("Publish error!\n");
    fdCloseSession(TaskSelf());
    return ;
}

void MQTTPublish3Work(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int len = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    char *payload = "attitude message test!!";
    int payloadlen = strlen(payload);

    topicString.cstring = "master/result/attitude";
    while (1){
        len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
        if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
            break;
        }
        Task_sleep(1000);
    }
    uart_printf("Publish error!\n");
    fdCloseSession(TaskSelf());
    return ;
}

void MQTTPublish4Work(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int len = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    char *payload = "status message test!!";
    int payloadlen = strlen(payload);

    topicString.cstring = "master/status";
    while (1){
        len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
        if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
            break;
        }
        Task_sleep(1000);
    }
    uart_printf("Publish error!\n");
    fdCloseSession(TaskSelf());
    return ;
}


void MQTTHeartWork(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    unsigned char buf[50];
    int buflen = sizeof(buf);
    int len;
    while(1) {
        len = MQTTSerialize_pingreq(buf, buflen);
        if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
            break;
        }
        Task_sleep(20000);
    }
    uart_printf("PINREQ TO MQTT SERVER ERROR!\n");
    fdCloseSession(TaskSelf());
}

int MQTTWork(void)
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	SOCKET mysock = 0;
	unsigned char buf[50];
	int buflen = sizeof(buf);
	int len = 0;
	int rc = 0;
	char *host = MQTTHost;
	int port = MQTTPort;

	pthread_t subthread;
	pthread_t pubthread;
	pthread_t heartthread;

	fdOpenSession(TaskSelf());
	mysock = transport_open(host, port);
	if(mysock == INVALID_SOCKET){
	    goto exit;
	}

	data.clientID.cstring = "mater node";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "master";
	data.password.cstring = "csu502";

	len = MQTTSerialize_connect(buf, buflen, &data);
	transport_sendPacketBuffer(mysock, buf, len);

	/* wait for connack */
	if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)
    {
        unsigned char sessionPresent, connack_rc;

        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
            uart_printf("Unable to connect, return code %d\n", connack_rc);
            goto exit;
        }
    }
    else
        goto exit;
	uart_printf("MQTT:Connect to hostname %s port %d\n", host, port);

    rc = pthread_create(&subthread, NULL, (void *(*)(void *))MQTTSubscribeWork, &mysock);
    if (rc != 0) {
        uart_printf("Suscribe thread create failed!\n");
        goto exit;
    }

    rc = pthread_create(&pubthread, NULL, (void *(*)(void *))MQTTPublishWork, &mysock);
    if (rc != 0) {
        uart_printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }

    rc = pthread_create(&pubthread, NULL, (void *(*)(void *))MQTTPublish2Work, &mysock);
    if (rc != 0) {
        uart_printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }
    rc = pthread_create(&pubthread, NULL, (void *(*)(void *))MQTTPublish3Work, &mysock);
    if (rc != 0) {
        uart_printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }
    rc = pthread_create(&pubthread, NULL, (void *(*)(void *))MQTTPublish4Work, &mysock);
    if (rc != 0) {
        uart_printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }

    rc = pthread_create(&heartthread, NULL, (void *(*)(void *))MQTTHeartWork, &mysock);
    if (rc != 0) {
        uart_printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }
    pthread_join(subthread, NULL);
    pthread_join(pubthread, NULL);
    pthread_join(heartthread, NULL);
exit:
    uart_printf("MQTT work exit!\n");
	transport_close(mysock);
	fdCloseSession(TaskSelf());
	return 0;
}

