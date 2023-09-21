/*
 * net_trans.h
 *
 *  Created on: 2023Äê9ÔÂ21ÈÕ
 *      Author: hanyi
 */

#ifndef CORE_NET_TRANS_H_
#define CORE_NET_TRANS_H_

/*
 * benchmark test data size = TEST_DATA_SIZE * MB
 * the unit is Byte
 */
#define TEST_DATA_SIZE      1024
#define MB                  (1024 * 1024)

/* Socket test mode */
#undef  TCP_CLIENT
#undef  TCP_SERVER
#undef  UDP_CLIENT
#undef  UDP_SERVER

/* TCP & UDP socket port id */
#define TCP_SERVER_PORT     1001
#define TCP_CLIENT_PORT     1002
#define UDP_SERVER_PORT     1003
#define UDP_CLIENT_PORT     1004

/* TCP & UDP single transmit size, the unit is Byte */
#define UDP_SERVER_BUFSIZE  1024
#define UDP_CLIENT_BUFSIZE  1024
#define TCP_SERVER_BUFSIZE  (1024 * 1024)
#define TCP_CLIENT_BUFSIZE  (1024 * 1024)

typedef struct {
    void *(*func)(void*);
    char *name;
    void *args;
}TransWork_t;

void psc_init();
void init_sgmii (uint32_t macPortNum);
int queue_manager_init(void);
int ndk_init(UArg arg0, UArg arg1);


#endif /* CORE_NET_TRANS_H_ */
