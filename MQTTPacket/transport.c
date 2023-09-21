/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *    Sergio R. Caprile - "commonalization" from prior samples and/or documentation extension
 *******************************************************************************/


#if !defined(SOCKET_ERROR)
	/** error in socket operation */
	#define SOCKET_ERROR -1
#endif
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <ti/ndk/inc/netmain.h>
#include "transport.h"
/**
This simple low-level implementation assumes a single connection for a single thread. Thus, a static
variable is used for that connection.
On other scenarios, the user must solve this by taking into account that the current implementation of
MQTTPacket_read() has a function pointer for a function call to get the data to a buffer, but no provisions
to know the caller or other indicator (the socket id): int (*getfn)(unsigned char*, int)
*/
static SOCKET mysock = INVALID_SOCKET;


int transport_sendPacketBuffer(SOCKET sock, unsigned char* buf, int buflen)
{
	int rc = 0;
	rc = send(sock, buf, buflen, 0);
	return rc;
}


int transport_getdata(unsigned char* buf, int count)
{
	int rc = recv(mysock, buf, count, 0);
	//printf("received %d bytes count %d\n", rc, (int)count);
	return rc;
}

int transport_getdatanb(void *sck, unsigned char* buf, int count)
{
	SOCKET sock = *((SOCKET *)sck); 	/* sck: pointer to whatever the system may use to identify the transport */
	/* this call will return after the timeout set on initialization if no bytes;
	   in your system you will use whatever you use to get whichever outstanding
	   bytes your socket equivalent has ready to be extracted right now, if any,
	   or return immediately */
	int rc = recv(sock, buf, count, 0);
	if (rc == -1) {
		/* check error conditions from your system here, and return -1 */
		return -1;
	}
	return rc;
}

/**
return >=0 for a socket descriptor, <0 for an error code
@todo Basically moved from the sample without changes, should accomodate same usage for 'sock' for clarity,
removing indirections
*/
SOCKET transport_open(char* addr, int port)
{
	SOCKET* sock = &mysock;
	int type = SOCK_STREAM;
	struct sockaddr_in address;

	int rc = -1;

	uint8_t family = AF_INET;
	static struct timeval tv;

	*sock = INVALID_SOCKET;
	if (addr[0] == '[')
	  ++addr;

	address.sin_port = NDK_htons(port);
	address.sin_family = family = AF_INET;
	address.sin_addr.s_addr = inet_addr(addr);


	*sock =	socket(family, type, 0);
	if (*sock != INVALID_SOCKET) {
		if (family == AF_INET){
			rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
			if (rc == -1) {
				return INVALID_SOCKET;
			}
		}

	}
	if (mysock == INVALID_SOCKET)
		return INVALID_SOCKET;

	tv.tv_sec = 0;  /* 1 second Timeout */
	tv.tv_usec = 500000;
	setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	return mysock;
}

int transport_close(SOCKET sock)
{
	int rc;

	rc = shutdown(sock, SHUT_WR);
	rc = recv(sock, NULL, (size_t)0, 0);
	rc = fdClose(sock);

	return rc;
}
