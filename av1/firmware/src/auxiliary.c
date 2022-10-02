#include "auxiliary.h"

// globals
volatile _Bool but1_flag = 0;
volatile _Bool but2_flag = 0;
volatile _Bool but3_flag = 0;

// hw
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();

    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC);

    /** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
    tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

    /* Configura NVIC*/
    NVIC_SetPriority(ID_TC, 4);
    NVIC_EnableIRQ((IRQn_Type)ID_TC);
    tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource)
{
    uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

    rtt_sel_source(RTT, false);
    rtt_init(RTT, pllPreScale);

    if (rttIRQSource & RTT_MR_ALMIEN)
    {
        uint32_t ul_previous_time;
        ul_previous_time = rtt_read_timer_value(RTT);
        while (ul_previous_time == rtt_read_timer_value(RTT))
            ;
        rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
    }

    /* config NVIC */
    NVIC_DisableIRQ(RTT_IRQn);
    NVIC_ClearPendingIRQ(RTT_IRQn);
    NVIC_SetPriority(RTT_IRQn, 4);
    NVIC_EnableIRQ(RTT_IRQn);

    /* Enable RTT interrupt */
    if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
        rtt_enable_interrupt(RTT, rttIRQSource);
    else
        rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}


void init(void)
{
    // setup
    board_init();
    sysclk_init();
    delay_init();
    gfx_mono_ssd1306_init();
    WDT->WDT_MR = WDT_MR_WDDIS;

    // init clocks
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);
    pmc_enable_periph_clk(ID_PIOC);
    pmc_enable_periph_clk(ID_PIOD);

    // btn1
    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
    pio_handler_set(BUT1_PIO,
                    BUT1_PIO_ID,
                    BUT1_PIO_IDX_MASK,
                    PIO_IT_FALL_EDGE,
                    but1_callback);
    pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
    pio_get_interrupt_status(BUT1_PIO);
    NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 4);

    // btn2
    pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
    pio_handler_set(BUT2_PIO,
                    BUT2_PIO_ID,
                    BUT2_PIO_IDX_MASK,
                    PIO_IT_FALL_EDGE,
                    but2_callback);
    pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
    pio_get_interrupt_status(BUT2_PIO);
    NVIC_EnableIRQ(BUT2_PIO_ID);
    NVIC_SetPriority(BUT2_PIO_ID, 4);

    // btn3
    pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
    pio_handler_set(BUT3_PIO,
                    BUT3_PIO_ID,
                    BUT3_PIO_IDX_MASK,
                    PIO_IT_FALL_EDGE,
                    but3_callback);
    pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
    pio_get_interrupt_status(BUT3_PIO);
    NVIC_EnableIRQ(BUT3_PIO_ID);
    NVIC_SetPriority(BUT3_PIO_ID, 4);

    // led
    pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
    pio_set(LED1_PIO, LED1_PIO_IDX_MASK);

    // led2
    pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
    pio_set(LED2_PIO, LED2_PIO_IDX_MASK);

    // led3
    pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
    pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
}

// sw
void clear_screen(int loc)
{
    if (loc == -1)
    {
        gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
        // gfx_mono_draw_string("             ", 0, 16, &sysfont);
    }
    else if (loc == 0)
        gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
    else if (loc == 1)
        gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);

    return;
}

void draw_string(char *text)
{
    char str[20];
    sprintf(str, "%s", text);
    gfx_mono_draw_string("            ", 0, 3, &sysfont);
    gfx_mono_draw_string(str, 0, 3, &sysfont);
}

void draw_dots(int x)
{
    gfx_mono_draw_filled_circle(x, 25, 2, GFX_PIXEL_SET, GFX_WHOLE);
}

void draw_time(Time t)
{
    char str[20];
    clear_screen(1);
    sprintf(str, "%02d:%02d", t.min, t.sec);
    gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
    gfx_mono_draw_string(str, 43, 4, &sysfont);
}

void but1_callback(void)
{
    but1_flag = 1;
}
_Bool get_but1_flag(void)
{
    return but1_flag;
}
_Bool set_but1_flag(_Bool val)
{
    but1_flag = val;
}

void but2_callback(void)
{
    but2_flag = 1;
}
_Bool get_but2_flag(void)
{
    return but2_flag;
}
_Bool set_but2_flag(_Bool val)
{
    but2_flag = val;
}

void but3_callback(void)
{
    but3_flag = 1;
}
_Bool get_but3_flag(void)
{
    return but3_flag;
}
_Bool set_but3_flag(_Bool val)
{
    but3_flag = val;
}


void pin_toggle(Pio *pio, uint32_t mask)
{
    if (pio_get_output_data_status(pio, mask))
        pio_clear(pio, mask);
    else
        pio_set(pio, mask);
}

void blink_led(Pio *p_pio, const uint32_t ul_mask, int n, int t)
{
    for (int i = 1; i <= n; i++)
    {
        pio_clear(p_pio, ul_mask);
        delay_ms(t / 2);
        pio_set(p_pio, ul_mask);
        delay_ms(t / 2);
    }
}