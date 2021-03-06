#ifndef _VVC_GLOBAL_H
#define _VVC_GLOBAL_H

#include <stdio.h>

// Core includes.
#ifdef VVC_F0
  #include "stm32f0xx.h"
#elif VVC_L0
  #include "stm32l0xx.h"
#elif VVC_F3
  #include "stm32f3xx.h"
#endif

// Define GPIOB pin mappings for software '4-wire' SPI interface.
#define PB_MOSI     (5)
#define PB_SCK      (3)
#define PB_DC       (4)
#define PA_ESP_CHPD (1)
#define PA_CS       (15)
#define PA_RST      (12)
// Global pin definitions.
#define PA_LED      (11)

// Peripheral definitions.
#if defined(VVC_F0) || defined(VVC_F3)
  #define VVC_UARTx     (USART1)
#elif VVC_L0
  #define VVC_UARTx     (USART2)
#endif

// Assembly methods.
extern void delay_cycles(unsigned int d);
extern void delay_us(unsigned int d);
extern void delay_ms(unsigned int d);
extern void delay_s(unsigned int d);
extern void pulse_out_pin(volatile void* gpiox_odr,
                          unsigned int pulse_pinmask,
                          unsigned int pulse_halfw,
                          unsigned int num_pulses);

// ----------------------
// Global variables and defines.
volatile uint8_t  uled_state;
volatile uint8_t  should_refresh_display;
volatile uint8_t  should_transceive_cmd;
volatile  int8_t  rx_page;
volatile  int16_t selected_at_cmd;
volatile uint32_t core_clock_hz;
//#define MAX_RX_LEN      (128)
//volatile char     rx_buf[MAX_RX_LEN];
#define RX_LINE_LEN     (14)
#define MAX_RX_PAGE     (16)
volatile char     rx_buf[MAX_RX_PAGE][RX_LINE_LEN];

// List of AT commands which can be issued.
#define NUM_AT_COMMANDS (3)
#define MAX_AT_LEN      (16)
static const char at_commands[NUM_AT_COMMANDS][MAX_AT_LEN] = {
  "AT\r\n\0",
  "AT+RST\r\n\0",
  "AT+GMR\r\n\0",
};
// (Currently-unused commands)
//  "AT+CWMODE_DEF=1\r\n\0"
//  "AT+CIOBAUD=9600\r\n\0"
//  "AT+CIOBAUD=115200\r\n\0"
//  "AT+CWLAP\r\n\0"

// SSD1331 OLED information (96x64 pixels)
#define OLED_MAX_X  (95)
#define OLED_MAX_Y  (63)
// OLED colors.
#define OLED_BLK    (0x0000)
#define OLED_LGRN   (0x8628)
#define OLED_MGRN   (0x5488)
#define OLED_DGRN   (0x2E47)
#define OLED_BRGNDY (0xD208)
#define OLED_YLW    (0xEF64)
#define OLED_ORNG   (0xEDC0)
#define OLED_TEAL   (0x2779)
#define OLED_PNK    (0xFC7F)
#define OLED_BLU    (0x141B)
#define OLED_PRP    (0xA118)
#define OLED_BRWN   (0x7B27)
#define OLED_LGRY   (0xCE59)
#define OLED_MGRY   (0x8C51)
#define OLED_DGRY   (0x4A69)
#define OLED_WHT    (0xFFFF)
// Color palette.
static const uint16_t oled_colors[16] = {
  OLED_BLK, OLED_LGRN, OLED_MGRN, OLED_DGRN,
  OLED_BRGNDY, OLED_YLW, OLED_ORNG, OLED_TEAL,
  OLED_PNK, OLED_BLU, OLED_PRP, OLED_BRWN,
  OLED_LGRY, OLED_MGRY, OLED_DGRY, OLED_WHT
};
// Buffer for the OLED screen.
// To fit in 4KB of SRAM, use 4 bits per pixel, to
// map to up to 16 colors defined above. So, 2px per byte.
// (1 Byte = 2 pixels)
#define OLED_FB_SIZE ((96 * 64) / 2)
volatile uint8_t oled_fb[OLED_FB_SIZE];
// Buffer for drawing lines of text to the OLED.
char oled_line_buf[18];

// Global defines.
// Define a simple monospace font; each character is 6x8 pixels,
// which comes out to 6 bytes or 3 words for every 2 characters.
#define OLED_CH_A0       0x1F688868
#define OLED_CH_B0       0xFF898989
#define OLED_CH_A1B1     0x1F007600
#define OLED_CH_C0       0x7E818181
#define OLED_CH_D0       0xFF818181
#define OLED_CH_C1D1     0x66007E00
#define OLED_CH_E0       0xFF898989
#define OLED_CH_F0       0xFF888888
#define OLED_CH_E1F1     0x81008000
#define OLED_CH_G0       0x7E818989
#define OLED_CH_H0       0xFF080808
#define OLED_CH_G1H1     0x6E00FF00
#define OLED_CH_I0       0x8181FF81
#define OLED_CH_J0       0x868181FE
#define OLED_CH_I1J1     0x81008000
#define OLED_CH_K0       0xFF182442
#define OLED_CH_L0       0xFF010101
#define OLED_CH_K1L1     0x81000100
#define OLED_CH_M0       0xFF403040
#define OLED_CH_N0       0xFF601806
#define OLED_CH_M1N1     0xFF00FF00
#define OLED_CH_O0       0x7E818181
#define OLED_CH_P0       0xFF888888
#define OLED_CH_O1P1     0x7E007000
#define OLED_CH_Q0       0x7E818582
#define OLED_CH_R0       0xFF888C8A
#define OLED_CH_Q1R1     0x7D007100
#define OLED_CH_S0       0x66919989
#define OLED_CH_T0       0x8080FF80
#define OLED_CH_S1T1     0x66008000
#define OLED_CH_U0       0xFE010101
#define OLED_CH_V0       0x701C031C
#define OLED_CH_U1V1     0xFE00E000
#define OLED_CH_W0       0xFE010601
#define OLED_CH_X0       0xC3241824
#define OLED_CH_W1X1     0xFE00C300
#define OLED_CH_Y0       0xE0100F10
#define OLED_CH_Z0       0x838599A1
#define OLED_CH_Y1Z1     0xE000C100
#define OLED_CH_a0       0x06292929
#define OLED_CH_b0       0xFF090909
#define OLED_CH_a1b1     0x1F000600
#define OLED_CH_c0       0x1E212121
#define OLED_CH_d0       0x060909FF
#define OLED_CH_c1d1     0x12000100
#define OLED_CH_e0       0x3E494949
#define OLED_CH_f0       0x087F8888
#define OLED_CH_e1f1     0x3A006000
#define OLED_CH_g0       0x32494949
#define OLED_CH_h0       0xFF080808
#define OLED_CH_g1h1     0x3E000700
#define OLED_CH_i0       0x00004F00
#define OLED_CH_j0       0x0006015E
#define OLED_CH_i1j1     0x00000000
#define OLED_CH_k0       0x00FF1C23
#define OLED_CH_l0       0x0000FF00
#define OLED_CH_k1l1     0x00000000
#define OLED_CH_m0       0x3F101F10
#define OLED_CH_n0       0x3F10100F
#define OLED_CH_m1n1     0x0F000000
#define OLED_CH_o0       0x0E111111
#define OLED_CH_p0       0x003F2424
#define OLED_CH_o1p1     0x0E001800
#define OLED_CH_q0       0x3048487E
#define OLED_CH_r0       0x003F1010
#define OLED_CH_q1r1     0x01000800
#define OLED_CH_s0       0x00324949
#define OLED_CH_t0       0x20FE2121
#define OLED_CH_s1t1     0x26000200
#define OLED_CH_u0       0x3C02023E
#define OLED_CH_v0       0x18060106
#define OLED_CH_u1v1     0x03001800
#define OLED_CH_w0       0x1E010201
#define OLED_CH_x0       0x110A040A
#define OLED_CH_w1x1     0x1E001100
#define OLED_CH_y0       0x3209093E
#define OLED_CH_z0       0x11131519
#define OLED_CH_y1z1     0x00001100
#define OLED_CH_00       0x7EE19987
#define OLED_CH_10       0x2141FF01
#define OLED_CH_0111     0x7E000100
#define OLED_CH_20       0x63878D99
#define OLED_CH_30       0x66818989
#define OLED_CH_2131     0x71007600
#define OLED_CH_40       0xF80808FF
#define OLED_CH_50       0xE2919191
#define OLED_CH_4151     0x08008E00
#define OLED_CH_60       0x7E919191
#define OLED_CH_70       0x60838CB0
#define OLED_CH_6171     0x4E00C000
#define OLED_CH_80       0x6E919191
#define OLED_CH_90       0x72898989
#define OLED_CH_8191     0x6E007E00
#define OLED_CH_col0     0x00002400
#define OLED_CH_per0     0x00000002
#define OLED_CH_col1per1 0x00000000
#define OLED_CH_exc0     0x007A0000
#define OLED_CH_fws0     0x00061860
#define OLED_CH_exc1fws1 0x00000000
#define OLED_CH_hyp0     0x00080808
#define OLED_CH_pls0     0x00081C08
#define OLED_CH_hyp1pls1 0x00000000
#define OLED_CH_lct0     0x00081422
#define OLED_CH_rct0     0x00442810
#define OLED_CH_lct1rct1 0x00000000

#endif
