#include <uxr/client/transport.h>
#include "serialtransport.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


#ifdef RMW_UXRCE_TRANSPORT_CUSTOM_SERIAL_DMA

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

uint8_t recv_buffer[UART_DMA_BUFFER_SIZE];
uint8_t fifo_buff[UART_DMA_BUFFER_SIZE];
int fifo_top = 0; //队顶指针
int fifo_bot = 0; //队底指针
int fifo_size = 0;//队长度
int recv_flag = 0;
int rx_len = 0;

int fifo_in(uint8_t * Addr,int len){
    int i;
    //判断FIFO是否还有空间
    if(len>(UART_DMA_BUFFER_SIZE - fifo_size))
        return 0;
    for(i=0;i<len;i++){
        fifo_buff[fifo_top]=Addr[i];
        //计算下一个地址
        fifo_top=(fifo_top+1)%UART_DMA_BUFFER_SIZE;
    }
    fifo_size+=len;
    return 1;
}

int fifo_out(char* buf, int size){
    int i;
    if(fifo_size<size)
        return 0;//FIFO剩余数据量不足
    for(i=0;i<size;i++){
        buf[i]=fifo_buff[fifo_bot];
        fifo_bot=(fifo_bot+1)%UART_DMA_BUFFER_SIZE;
    }
    fifo_size-=size;
    return 1;
}

int is_fifo_empty(){
    return fifo_size == 0;
}

bool serial_transport_open(uxrCustomTransport * transport){
    UNUSED(transport);
    HAL_UART_Receive_DMA(&huart1, recv_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool serial_transport_close(uxrCustomTransport * transport){
    UNUSED(transport);
    HAL_UART_DMAStop(&huart1);
    return true;
}

size_t serial_transport_write(uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    UNUSED(transport);
    HAL_StatusTypeDef ret = HAL_OK;
    if (huart1.gState == HAL_UART_STATE_READY) {
        ret = HAL_UART_Transmit(&huart1, buf, len, 1000);
        while (ret == HAL_OK && huart1.gState != HAL_UART_STATE_READY) {
            // osDelay(1);
        }
        
        return (ret == HAL_OK) ? len : 0;
    }else{
        return 0;
    }
}

size_t serial_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UNUSED(transport);
    int wrote = 0;
    uint32_t start_time = uxr_millis();
    uint32_t end_time;
    while (wrote < len) {
        end_time = uxr_millis();
        if (end_time - start_time > timeout) {
            break;
        }
        if(!is_fifo_empty() && fifo_size > len){ 
            fifo_out((char*)buf, len);
            wrote = len;
        }
    }   
    return wrote;
}


void HAL_UART_IDLECallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET){
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            HAL_UART_AbortReceive(huart);
            rx_len = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
            if (rx_len != 0 && rx_len < UART_DMA_BUFFER_SIZE) {
                fifo_in(recv_buffer, rx_len);
            }
            HAL_UART_Receive_DMA(huart, recv_buffer, 2048);
        }
    }
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM
