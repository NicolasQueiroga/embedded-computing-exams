#include "auxiliary.h"

void TC0_Handler(void);
void TC1_Handler(void);
void RTT_Handler(void);

volatile _Bool tc0_flag = 0;
volatile _Bool tc1_flag = 0;
volatile _Bool start = 1;
volatile _Bool oppened = 0;
volatile _Bool passed = 0;

void TC0_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 0);
	tc0_flag = 1;
}

void TC1_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 1);
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
	tc1_flag = 1;
}

void RTT_Handler(void)
{
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	}

	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
	}
}

void set_time(Time *t)
{
	if (get_but2_flag())
	{
		start = 0;
		oppened = 0;
		tc_stop(TC0, 1);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		set_but2_flag(0);
		if (t->sec + 10 > 60 && t->min + 1 <= 60)
		{
			t->min++;
			t->sec = 60 - t->sec;
		}
		else if (t->sec + 10 <= 60)
		{
			t->sec += 10;
		}
	}
	if (get_but3_flag())
	{
		start = 0;
		oppened = 0;
		tc_stop(TC0, 1);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		set_but3_flag(0);
		if (t->sec + 10 > 60 && t->min + 1 <= 60)
		{
			t->min++;
			t->sec = 60 - t->sec;
		}
		else if (t->sec + 5 <= 60)
		{
			t->sec += 5;
		}
	}
}

void use_door(Time *t)
{
	if (get_but1_flag())
	{
		tc_stop(TC0, 1);
		while (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK))
		{
			tc_stop(TC0, 0);
			pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		}
		set_but1_flag(0);
		if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) && t->min >= 0 && t->sec > 0)
		{
			tc_start(TC0, 0);
			pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
		}
		if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) && t->min == 0 && t->sec == 0)
		{
			tc_start(TC0, 0);
			oppened = 1;
			pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		}
	}
}

void reset(Time *t)
{
	if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK) && !pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK))
	{
		oppened = 1;
		set_but2_flag(0);
		set_but3_flag(0);
		t->min = 0;
		t->sec = 0;
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		delay_ms(500);
	}
}

void main(void)
{
	init();
	Time t = {0, 0};
	draw_time(t);

	// TC
	TC_init(TC0, ID_TC0, 0, 1);
	tc_start(TC0, 0);

	TC_init(TC0, ID_TC1, 1, 10);
	// ---

	// RTT
	// ---

	while (1)
	{
		reset(&t);
		if (tc0_flag)
		{
			tc0_flag = 0;
			if (t.min >= 0 && t.sec > 0)
			{
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				if (t.sec - 1 < 0 && t.min >= 0)
				{
					t.min--;
					t.sec = 60;
				}
				else
					t.sec--;
			}
			if (t.min == 0 && t.sec == 0 && !start && !oppened)
			{

				tc_start(TC0, 1);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				// pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				if (!passed)
				{
					passed = 1;
					RTT_init(4, 60, RTT_MR_ALMIEN);
				}
			}
		}

		// if (tc1_flag)
		// {
		// 	tc1_flag = 0;
		// 	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
		// }

		use_door(&t);
		set_time(&t);
		draw_time(t);
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}