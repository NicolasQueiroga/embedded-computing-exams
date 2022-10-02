#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/** IOS **/
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

#define BUT_EXTRA_PIO PIOA
#define BUT_EXTRA_PIO_ID ID_PIOA
#define BUT_EXTRA_IDX 2
#define BUT_EXTRA_IDX_MASK (1u << BUT_EXTRA_IDX)

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0

#define AFEC_POT2 AFEC1
#define AFEC_POT2_ID ID_AFEC1
#define AFEC_POT2_CHANNEL 1

/** RTOS  */
#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_EVENT_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_EVENT_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

// data structures
typedef struct _Calendar
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} Calendar;

typedef struct _EventData
{
	char id;
	char state;
} EventData;

// globals
volatile QueueHandle_t xQueueAfec;
volatile QueueHandle_t xQueueAfec2;
volatile QueueHandle_t xQueueEvent;
volatile SemaphoreHandle_t xSemaphoreMutex;
volatile SemaphoreHandle_t xSemaphoreEventAlarm;
volatile SemaphoreHandle_t xSemaphoreAfecAlarm;
volatile _Bool flag_rtc_second = 0;
volatile _Bool flag_rtc_alarm = 0;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void io_init(void);
void blink_led(Pio *p_pio, const uint32_t ul_mask, int n, int t);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;)
	{
	}
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void)
{
	configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void)
{
	EventData ed = {'1', !pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)};
	xQueueSendFromISR(xQueueEvent, &ed, 0);
}

void but2_callback(void)
{
	EventData ed = {'2', !pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK)};
	xQueueSendFromISR(xQueueEvent, &ed, 0);
}

void but3_callback(void)
{
	EventData ed = {'3', !pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK)};
	xQueueSendFromISR(xQueueEvent, &ed, 0);
}

void butEXTRA_callback(void)
{
	EventData ed = {'E', !pio_get(BUT_EXTRA_PIO, PIO_INPUT, BUT_EXTRA_IDX_MASK)};
	xQueueSendFromISR(xQueueEvent, &ed, 0);
}

void pot_callback(void)
{
	uint32_t adc, adc2;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	adc = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	xQueueSendFromISR(xQueueAfec, &adc, &xHigherPriorityTaskWoken);

	adc2 = afec_channel_get_value(AFEC_POT2, AFEC_POT2_CHANNEL);
	xQueueSendFromISR(xQueueAfec2, &adc2, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters)
{
	gfx_mono_ssd1306_init();

	for (;;)
	{
		gfx_mono_draw_filled_circle(12, 12, 4, GFX_PIXEL_XOR, GFX_QUADRANT0 | GFX_QUADRANT1 | GFX_QUADRANT2 | GFX_QUADRANT3);
		vTaskDelay(200);
	}
}

void task_afec(void *pvParameters)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t adc;
	int cnt = 1;
	_Bool alarmed = 0;
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	while (1)
	{
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
		if (xQueueReceive(xQueueAfec, &(adc), 1000))
		{
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("[AFEC1] %02d:%02d:%04d %02d:%02d:%02d $%d\n", current_day, current_month, current_year, current_hour, current_min, current_sec, adc);
			xSemaphoreGive(xSemaphoreMutex);
			if (adc >= 3000)
				cnt++;
		}
		else
		{
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("Nao chegou um novo dado em 1 segundo");
			xSemaphoreGive(xSemaphoreMutex);
		}

		if (cnt >= 5 && !alarmed)
		{
			xSemaphoreGiveFromISR(xSemaphoreAfecAlarm, &xHigherPriorityTaskWoken);
			alarmed = 1;
		}
	}
}

void task_afec2(void *pvParameters)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t adc2;
	int cnt = 1;
	_Bool alarmed = 0;
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	while (1)
	{
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
		if (xQueueReceive(xQueueAfec2, &(adc2), 1000))
		{
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("[AFEC2] %02d:%02d:%04d %02d:%02d:%02d $%d\n", current_day, current_month, current_year, current_hour, current_min, current_sec, adc2);
			xSemaphoreGive(xSemaphoreMutex);
			if (adc2 >= 3000)
				cnt++;
		}
		else
		{
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("Nao chegou um novo dado em 1 segundo");
			xSemaphoreGive(xSemaphoreMutex);
		}

		if (cnt >= 5 && !alarmed)
		{
			xSemaphoreGiveFromISR(xSemaphoreAfecAlarm, &xHigherPriorityTaskWoken);
			alarmed = 1;
		}
	}
}

void task_event(void *pvParameters)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EventData queue;
	char id;
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	for (;;)
	{
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
		if (xQueueReceive(xQueueEvent, &(queue), 1000))
		{
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("[EVENT] %02d:%02d:%04d %02d:%02d:%02d $%c:%d\n", current_day, current_month, current_year, current_hour, current_min, current_sec, queue.id, queue.state);
			xSemaphoreGive(xSemaphoreMutex);
		}
		if (!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK) && !pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK) ||
			!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK) && !pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK) ||
			!pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK) && !pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK) ||
			!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK) && !pio_get(BUT_EXTRA_PIO, PIO_INPUT, BUT_EXTRA_IDX_MASK) ||
			!pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK) && !pio_get(BUT_EXTRA_PIO, PIO_INPUT, BUT_EXTRA_IDX_MASK) ||
			!pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK) && !pio_get(BUT_EXTRA_PIO, PIO_INPUT, BUT_EXTRA_IDX_MASK))
			xSemaphoreGiveFromISR(xSemaphoreEventAlarm, &xHigherPriorityTaskWoken);
	}
}

void task_alarm(void *pvParameters)
{
	TC_init(TC0, ID_TC1, 1, 10);
	TC_init(TC0, ID_TC2, 2, 10);
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	for (;;)
	{
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
		if (xSemaphoreTake(xSemaphoreEventAlarm, 1))
		{
			tc_start(TC0, 1);
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("[ALARM] %02d:%02d:%04d %02d:%02d:%02d $%s\n", current_day, current_month, current_year, current_hour, current_min, current_sec, "EVENT");
			xSemaphoreGive(xSemaphoreMutex);
		}
		if (xSemaphoreTake(xSemaphoreAfecAlarm, 1))
		{
			tc_start(TC0, 2);
			xSemaphoreTake(xSemaphoreMutex, portMAX_DELAY);
			printf("[ALARM] %02d:%02d:%04d %02d:%02d:%02d $%s\n", current_day, current_month, current_year, current_hour, current_min, current_sec, "AFEC");
			xSemaphoreGive(xSemaphoreMutex);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask)
{
	pio_get_output_data_status(pio, mask) ? pio_clear(pio, mask) : pio_set(pio, mask);
}

void TC0_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 0);
	afec_start_software_conversion(AFEC_POT);
	afec_start_software_conversion(AFEC_POT2);
}

void TC1_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 1);
	pin_toggle(LED_1_PIO, LED_1_IDX_MASK);
}

void TC2_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 2);
	pin_toggle(LED_2_PIO, LED_2_IDX_MASK);
}

void RTT_Handler(void)
{
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
	}

	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
		afec_start_software_conversion(AFEC_POT);
		afec_start_software_conversion(AFEC_POT2);
		RTT_init(1, 5, RTT_SR_RTTINC);
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC)
		flag_rtc_second = 1;

	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM)
		flag_rtc_alarm = 1;

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

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

	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
	else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, Calendar t, uint32_t irq_type)
{
	pmc_enable_periph_clk(ID_RTC);
	rtc_set_hour_mode(rtc, 0);

	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	rtc_enable_interrupt(rtc, irq_type);
}

void configure_afec(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback)
{
	afec_enable(afec);
	struct afec_config afec_cfg;
	afec_get_config_defaults(&afec_cfg);
	afec_init(afec, &afec_cfg);
	afec_set_trigger(afec, AFEC_TRIG_SW);

	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	afec_channel_set_analog_offset(afec, afec_channel, 0x200);
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void io_init(void)
{
	pmc_enable_periph_clk(LED_1_PIO_ID);
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pmc_enable_periph_clk(LED_3_PIO_ID);
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);

	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_EXTRA_PIO, PIO_INPUT, BUT_EXTRA_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_EDGE,
					but1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_EDGE,
					but2_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_EDGE,
					but3_callback);
	pio_handler_set(BUT_EXTRA_PIO, BUT_EXTRA_PIO_ID, BUT_EXTRA_IDX_MASK, PIO_IT_EDGE,
					butEXTRA_callback);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_enable_interrupt(BUT_EXTRA_PIO, BUT_EXTRA_IDX_MASK);

	pio_get_interrupt_status(BUT_1_PIO);
	pio_get_interrupt_status(BUT_2_PIO);
	pio_get_interrupt_status(BUT_3_PIO);
	pio_get_interrupt_status(BUT_EXTRA_PIO);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_EXTRA_PIO_ID);
	NVIC_SetPriority(BUT_EXTRA_PIO_ID, 4);

	pio_set(LED_1_PIO, LED_1_IDX_MASK);
	pio_set(LED_2_PIO, LED_2_IDX_MASK);
	pio_set(LED_3_PIO, LED_3_IDX_MASK);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	configure_console();
	io_init();

	Calendar rtc_initial = {2018, 3, 19, 12, 15, 45, 1};
	RTC_init(RTC, ID_RTC, rtc_initial, 0);

	xSemaphoreMutex = xSemaphoreCreateMutex();
	xQueueEvent = xQueueCreate(10, sizeof(EventData));
	xQueueAfec = xQueueCreate(32, sizeof(uint32_t));
	xQueueAfec2 = xQueueCreate(32, sizeof(uint32_t));

	xSemaphoreAfecAlarm = xSemaphoreCreateBinary();
	if (xSemaphoreAfecAlarm == NULL)
		printf("falha em criar o semaforo \n");

	xSemaphoreEventAlarm = xSemaphoreCreateBinary();
	if (xSemaphoreEventAlarm == NULL)
		printf("falha em criar o semaforo \n");

	configure_afec(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, pot_callback);
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	configure_afec(AFEC_POT2, AFEC_POT2_ID, AFEC_POT2_CHANNEL, pot_callback);
	afec_channel_enable(AFEC_POT2, AFEC_POT2_CHANNEL);
	// RTT_init(1, 5, RTT_SR_RTTINC);
	TC_init(TC0, ID_TC0, 0, 1);
	tc_start(TC0, 0);

	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_afec, "afec", TASK_ADC_STACK_SIZE, NULL, TASK_ADC_STACK_PRIORITY, NULL) != pdPASS)
		printf("Failed to create afec task\r\n");

	if (xTaskCreate(task_afec2, "afec2", TASK_ADC_STACK_SIZE, NULL, TASK_ADC_STACK_PRIORITY, NULL) != pdPASS)
		printf("Failed to create afec2 task\r\n");

	if (xTaskCreate(task_event, "EVENT", TASK_EVENT_STACK_SIZE, NULL, TASK_EVENT_STACK_PRIORITY, NULL) != pdPASS)
		printf("Failed to create EVENT task\r\n");

	if (xTaskCreate(task_alarm, "ALARM", TASK_EVENT_STACK_SIZE, NULL, TASK_EVENT_STACK_PRIORITY, NULL) != pdPASS)
		printf("Failed to create ALARM task\r\n");

	vTaskStartScheduler();
	while (1)
	{
	}

	return 0;
}
