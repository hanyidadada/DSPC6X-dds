#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <ti/sysbios/knl/Task.h>
#include "MQTTPacket.h"
#include "transport.h"
#include "MQTTwork.h"

int MQTTSubscribeWork(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int rc = 0;
    int len = 0;
    int req_qos = 0;
    int msgid = 1;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    /* subscribe */
    topicString.cstring = SUBTOPIC;
    len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 1, &topicString, &req_qos);

    transport_sendPacketBuffer(mysock, buf, len);
    if (MQTTPacket_read(buf, buflen, transport_getdata) == SUBACK)  /* wait for suback */
    {
        unsigned short submsgid;
        int subcount;
        int granted_qos;

        rc = MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, buflen);
        if (granted_qos != 0 || rc < 0)
        {
            printf("granted qos != 0, %d\n", granted_qos);
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
            printf("message arrived %.*s\n", payloadlen_in, payload_in);
        }
    }
exit:
    printf("Subscribe error!\n");
    fdCloseSession(TaskSelf());
    return -1;

}

int MQTTPublishWork(void *arg)
{
    fdOpenSession(TaskSelf());
    SOCKET mysock = *(SOCKET *)arg;
    int len = 0;
    unsigned char buf[50];
    int buflen = sizeof(buf);
    MQTTString topicString = MQTTString_initializer;
    char *payload = "mypayload";
    int payloadlen = strlen(payload);

    topicString.cstring = PUBTOPIC;
    while(1) {
        len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
        if(transport_sendPacketBuffer(mysock, buf, len) < 0) {
            break;
        }
        Task_sleep(1000);
    }
    printf("Publish error!\n");
    fdCloseSession(TaskSelf());
    return -1;
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

	fdOpenSession(TaskSelf());
	mysock = transport_open(host, port);
	if(mysock == INVALID_SOCKET)
		return -1;

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
            printf("Unable to connect, return code %d\n", connack_rc);
            goto exit;
        }
    }
    else
        goto exit;
    printf("MQTT:Connect to hostname %s port %d\n", host, port);

    rc = pthread_create(&subthread, NULL, (void *(*)(void *))MQTTSubscribeWork, &mysock);
    if (rc != 0) {
        printf("Suscribe thread create failed!\n");
        goto exit;
    }
    rc = pthread_create(&pubthread, NULL, (void *(*)(void *))MQTTPublishWork, &mysock);
    if (rc != 0) {
        printf("Publish thread create failed!\n");
        pthread_cancel(subthread);
        goto exit;
    }
    pthread_join(subthread, NULL);
    pthread_join(pubthread, NULL);
exit:
    printf("MQTT work exit!\n");
	transport_close(mysock);
	fdCloseSession(TaskSelf());
	return 0;
}

