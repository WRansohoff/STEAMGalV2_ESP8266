#ifndef _VVC_PERIPHS_H
#define _VVC_PERIPHS_H

#include "global.h"

/* Timer Peripherals */
void stop_timer(TIM_TypeDef *TIMx);
void start_ms_timer(TIM_TypeDef *TIMx,
                    uint16_t ms,
                    uint8_t with_interrupt);
void start_timer(TIM_TypeDef *TIMx,
                 uint16_t prescaler,
                 uint16_t period,
                 uint8_t  with_interrupt);

/* USART Peripherals */
void init_uart(USART_TypeDef *USARTx,
               uint32_t baud_rate);
void tx_byte(USART_TypeDef *USARTx, uint8_t tx);
void tx_string(USART_TypeDef *USARTx, const char *str);
uint8_t rx_byte(USART_TypeDef *USARTx, uint32_t timeout);
void rx_str(USART_TypeDef *USARTx, uint32_t timeout_cycles);

#endif
