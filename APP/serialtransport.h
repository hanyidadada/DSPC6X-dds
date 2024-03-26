/*
 * uxr_transport_usb.h
 *
 *  Created on: Dec 11, 2023
 *      Author: pansamic
 */

#ifndef INC_UXR_TRANSPORT_H_
#define INC_UXR_TRANSPORT_H_

#include "uxr/client/transport.h"
#include "uxr/client/client.h"

//#define RMW_UXRCE_TRANSPORT_CUSTOM_SERIAL_DMA

// #define RMW_UXRCE_TRANSPORT_CUSTOM_SERIAL_IT

// #define RMW_UXRCE_TRANSPORT_CUSTOM_USB

bool serial_transport_open(struct uxrCustomTransport * transport);
bool serial_transport_close(struct uxrCustomTransport * transport);
size_t serial_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
size_t serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
//void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);
#endif /* INC_UXR_TRANSPORT_H_ */
