#ifndef __AUXILIARY_H_
#define __AUXILIARY_H_

#include "asf.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "ai.h"

#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1u << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1u << LED2_PIO_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1u << LED3_PIO_IDX)

#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// hw
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void init(void);

// sw
void but1_callback(void);
_Bool get_but1_flag(void);
_Bool set_but1_flag(_Bool val);

void but2_callback(void);
_Bool get_but2_flag(void);
_Bool set_but2_flag(_Bool val);

void but3_callback(void);
_Bool get_but3_flag(void);
_Bool set_but3_flag(_Bool val);

void clear_screen(int loc);
void draw_string( char *text);
void draw_dots(int x);
void draw_time(Time t);
void pin_toggle(Pio *pio, uint32_t mask);
void blink_led(Pio *p_pio, const uint32_t ul_mask, int n, int t);

#endif