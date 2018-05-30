#include "peripherals.h"

/* Timer Peripherals */

/*
 * Stop a running timer on a given Timer peripheral.
 */
void stop_timer(TIM_TypeDef *TIMx) {
  // Turn off the timer's 'counter'.
  TIMx->CR1 &= ~(TIM_CR1_CEN);
  // Clear the 'pending update interrupt' flag.
  TIMx->SR  &= ~(TIM_SR_UIF);
}

/*
 * 'Start Milliseconds Timer'
 * This method starts a timer to tick over every N
 * milliseconds, and optionally trigger a hardware interrupt.
 */
void start_ms_timer(TIM_TypeDef *TIMx,
                    uint16_t ms,
                    uint8_t with_interrupt) {
  start_timer(TIMx, core_clock_hz / 1000, ms, with_interrupt);
}

/*
 * 'Start Timer'
 * This method starts a new 'counter' timer, which triggers
 * an 'UPDATE' interrupt every time that it triggers.
 * It should trigger every N seconds, where:
 * N = (period) / (core clock frequency / prescaler)
 */
void start_timer(TIM_TypeDef *TIMx,
                 uint16_t prescaler,
                 uint16_t period,
                 uint8_t  with_interrupt) {
  // Initialize the 'TIMx' timer peripheral. (TODO: F303 support)
  // Start by making sure the timer's 'counter' is off.
  TIMx->CR1 &= ~(TIM_CR1_CEN);
  // Next, reset the peripheral. (This is where a HAL can help)
  if (TIMx == TIM2) {
    RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM2RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
  }
  else if (TIMx == TIM3) {
    RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM3RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
  }
  else if (TIMx == TIM16) {
    RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM16RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);
  }
  else if (TIMx == TIM17) {
    RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM17RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
  }
  // Set clock division to 1; the timer triggers every N events.
  // Also set the counter to count up.
  TIMx->CR1  &= ~(TIM_CR1_DIR |
                  TIM_CR1_CKD);
  // Set the 'prescaler'; the timer 'ticks' every N clock cycles.
  TIMx->PSC   =  prescaler;
  // Set 'auto-reload'; this is the timer's period in 'ticks'.
  TIMx->ARR   =  period;
  // Set the repetition counter to 0; I dunno, unused for now.
  TIMx->RCR   =  0;
  // Before enabling the counter, trigger an 'update' event to
  // make sure that the chip registers the 'prescaler' settings.
  TIMx->EGR  |=  (TIM_EGR_UG);
  if (with_interrupt) {
    // Enable the 'update' timer event/interrupt.
    TIMx->DIER |=  (TIM_DIER_UIE);
  }
  // Finally, enable the timer.
  TIMx->CR1  |=  (TIM_CR1_CEN);
}

/*
 * Initialize a USART peripheral in asynchronous mode,
 * with a given baud rate.
 */
void init_uart(USART_TypeDef *USARTx,
               uint32_t baud_rate) {
  // Disable the peripheral.
  USARTx->CR1 &= ~(USART_CR1_UE);
  // Set the baud rate.
  uint16_t br = core_clock_hz / baud_rate;
  USARTx->BRR  =  (br);
  // Enable both transmit and receive lines.
  // (Actually, it seems best to avoid doing this until
  //  you actually need to transceive.)
  //USARTx->CR1 |=  (USART_CR1_TE |
  //                 USART_CR1_RE);
  // Enable the peripheral.
  USARTx->CR1 |=  (USART_CR1_UE);
}

/*
 * Transmit a byte over the USART peripheral.
 */
void tx_byte(USART_TypeDef *USARTx, uint8_t tx) {
  // Wait for 'transmit buffer empty' signal.
  while (!(USARTx->ISR & USART_ISR_TXE)) {};
  USARTx->TDR = (tx & 0xFF);
}

/*
 * Transmit a string over the USART peripheral.
 * The given string must be null-terminated.
 */
void tx_string(USART_TypeDef *USARTx, const char *str) {
  uint32_t i = 0;
  uint8_t tc = (uint8_t)str[i];
  while (tc != '\0') {
    tc = (uint8_t)str[i];
    tx_byte(USARTx, tc);
    ++i;
  }
  // DEBUG: wait for 'Transfer Complete'.
  while (!(USARTx->ISR & USART_ISR_TC)) {};
}

/*
 * Read a byte out of the USART's input queue.
 */
uint8_t rx_byte(USART_TypeDef *USARTx, uint32_t timeout) {
  uint32_t ti = 0;
  // Wait for 'receive buffer not empty' signal.
  while (!(USARTx->ISR & USART_ISR_RXNE)) {
    ++ti;
    if (ti >= timeout) { return 0; }
  };
  return (USARTx->RDR & 0xFF);
}

/*
 * Read a transmission from the USART's input,
 * as long as bytes are available.
 * Timeout is selectable, but based on loop iterations not time.
 * Maximum string size is specified in 'src/global.h'
 */
void rx_str(USART_TypeDef *USARTx, uint32_t timeout_cycles) {
  uint32_t t_iter = 0;
  uint32_t buf_pos = 0;
  uint8_t r_val = 0;
  uint8_t r_pg = 0;
  // Receive bytes until the timeout is reached.
  for (t_iter = 0; t_iter < timeout_cycles; ++t_iter) {
    if (USARTx->ISR & USART_ISR_RXNE) {
      if (buf_pos >= RX_LINE_LEN-1) {
        if (r_pg < MAX_RX_PAGE) {
          // (Maximum line length.)
          rx_buf[r_pg][RX_LINE_LEN-1] = '\0';
          ++r_pg;
          buf_pos = 0;
        }
      }
      // Whether or not the buffer will be filled, the RDR
      // register needs to be read to reset the RXNE flag.
      r_val = USARTx->RDR & 0xFF;
      if (r_pg < MAX_RX_PAGE) {
        // Store the byte, reset the timeout.
        // If the byte is a newline character, act differently.
        if (r_val == '\r' ||
            r_val == '\0' ||
            (r_val == ' ' && buf_pos == 0)) {}
        else if (r_val == '\n') {
          if (buf_pos != 0) {
            rx_buf[r_pg][buf_pos] = '\0';
            ++r_pg;
            buf_pos = 0;
          }
        }
        else {
          rx_buf[r_pg][buf_pos] = r_val;
          ++buf_pos;
        }
      }
      t_iter = 0;
    }
  }
  // (Timeout reached.)
  if (buf_pos == 0 && r_pg == 0) {
    snprintf((char*)rx_buf, 15, "%s", "Timeout\0");
  }
}
