#include "asf.h"
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

#define MAG_1_PIO PIOA
#define MAG_1_PIO_ID ID_PIOA
#define MAG_1_IDX 3
#define MAG_1_IDX_MASK (1 << MAG_1_IDX)

#define MAG_2_PIO PIOA
#define MAG_2_PIO_ID ID_PIOA
#define MAG_2_IDX 2
#define MAG_2_IDX_MASK (1 << MAG_2_IDX)

#define MAG_3_PIO PIOC
#define MAG_3_PIO_ID ID_PIOC
#define MAG_3_IDX 19
#define MAG_3_IDX_MASK (1 << MAG_3_IDX)

#define MAG_4_PIO PIOA
#define MAG_4_PIO_ID ID_PIOA
#define MAG_4_IDX 6
#define MAG_4_IDX_MASK (1 << MAG_4_IDX)

#define MAG2_1_PIO PIOB
#define MAG2_1_PIO_ID ID_PIOB
#define MAG2_1_IDX 4
#define MAG2_1_IDX_MASK (1 << MAG2_1_IDX)

#define MAG2_2_PIO PIOA
#define MAG2_2_PIO_ID ID_PIOA
#define MAG2_2_IDX 4
#define MAG2_2_IDX_MASK (1 << MAG2_2_IDX)

#define MAG2_3_PIO PIOA
#define MAG2_3_PIO_ID ID_PIOA
#define MAG2_3_IDX 24
#define MAG2_3_IDX_MASK (1 << MAG2_3_IDX)

#define MAG2_4_PIO PIOD
#define MAG2_4_PIO_ID ID_PIOD
#define MAG2_4_IDX 26
#define MAG2_4_IDX_MASK (1 << MAG2_4_IDX)

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

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

typedef struct _motorData
{
	uint status; // 0 desligado, 1 ligado
	uint dir;	 // 0 hor�rio, 1 anti-hor�rio
	uint vel;	 // 1, 2, 3, 4
} motorData;

SemaphoreHandle_t xSemaphoreTC, xSemaphoreFullSpin;
volatile QueueHandle_t xQueueBtn;
volatile QueueHandle_t xQueueMotor, xQueueMotor2;
volatile xTaskHandle xHandleCntrl;
volatile xTaskHandle xHandleMotor, xHandleMotor2;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
										  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void io_init(void);
void updateMotor(int *rotation, int dir);
void updateMotor2(int *rotation, int dir);
void TC0_Handler(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
										  signed char *pcTaskName)
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
	char id = '1';
	xQueueSendFromISR(xQueueBtn, &id, 0);
}

void but2_callback(void)
{
	char id = '2';
	xQueueSendFromISR(xQueueBtn, &id, 0);
}

void but3_callback(void)
{
	char id = '3';
	xQueueSendFromISR(xQueueBtn, &id, 0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters)
{
	gfx_mono_ssd1306_init();

	for (;;)
	{
		// gfx_mono_draw_filled_circle(100, 30, 4, GFX_PIXEL_XOR, GFX_QUADRANT0 | GFX_QUADRANT1 | GFX_QUADRANT2 | GFX_QUADRANT3);
		vTaskDelay(200);
	}
}

static void task_cntr(void *pvParameters)
{
	char queue;
	_Bool turn_on_first_time = 1;
	_Bool is_on = 0;
	_Bool is_clk_wise = 0;
	int vel[4] = {100, 50, 25, 10};
	int index = 0;
	motorData md = {is_on, is_clk_wise, vel[0]};
	motorData md2 = {is_on, is_clk_wise, vel[0]};
	gfx_mono_draw_string("Power OFF", 0, 3, &sysfont);
	gfx_mono_draw_string("CLK Wise         ", 0, 23, &sysfont);
	gfx_mono_draw_string("RPM: 0   ", 60, 3, &sysfont);
	for (;;)
	{
		if (xSemaphoreTake(xSemaphoreFullSpin, 1))
		{
			is_on = !is_on;
			is_on ? vTaskResume(xHandleMotor) : vTaskSuspend(xHandleMotor);
			is_on ? vTaskResume(xHandleMotor2) : vTaskSuspend(xHandleMotor2);
			gfx_mono_draw_string(is_on ? "Power ON " : "Power OFF", 0, 3, &sysfont);
		}
		if (xQueueReceive(xQueueBtn, &queue, (TickType_t)500))
		{
			printf("but%c\n", queue);
			if (queue == '1')
			{
				is_on = !is_on;
				md.status = is_on;
				md2.status = is_on;
				is_on ? vTaskResume(xHandleMotor) : vTaskSuspend(xHandleMotor);
				is_on ? vTaskResume(xHandleMotor2) : vTaskSuspend(xHandleMotor2);
				gfx_mono_draw_string(is_on ? "Power ON " : "Power OFF", 0, 3, &sysfont);
			}
			if (queue == '2')
			{
				is_clk_wise = !is_clk_wise;
				md.dir = is_clk_wise;
				md2.dir = is_clk_wise;
				gfx_mono_draw_string(is_clk_wise ? "CLK Wise         " : "Counter CLK Wise", 0, 23, &sysfont);
			}
			if (queue == '3')
			{
				md.vel = vel[++index % 3];
				md2.vel = vel[++index % 3];
				char spd[100];
				sprintf(spd, "RPM: %.2lf   ", 0.72 / 360 * 60 * 1000 / md.vel);
				gfx_mono_draw_string(spd, 60, 3, &sysfont);
			}

			xQueueSend(xQueueMotor, &md, 1);
			xQueueSend(xQueueMotor2, &md2, 1);
		}
	}
}

static void task_motor(void *pvParameters)
{
	motorData md = {0, 0, 10};
	int rotation = 1u << 3;
	int cnt = 0;
	for (;;)
	{
		if (xQueueReceive(xQueueMotor, &md, (TickType_t)1))
		{
			TC_init(TC0, ID_TC0, 0, 1000 / md.vel);
			tc_start(TC0, 0);
		}
		if (xSemaphoreTake(xSemaphoreTC, 10))
		{
			updateMotor(&rotation, md.dir);
			if (cnt++ >= 500)
			{
				cnt = 0;
				tc_stop(TC0, 0);
				xSemaphoreGive(xSemaphoreFullSpin);
			}
		}
	}
}

static void task_motor2(void *pvParameters)
{
	motorData md = {0, 0, 100};
	int rotation = 1u << 3;
	int cnt = 0;
	for (;;)
	{
		if (xQueueReceive(xQueueMotor2, &md, (TickType_t)1))
		{
			TC_init(TC0, ID_TC0, 0, 1000 / md.vel);
			tc_start(TC0, 0);
		}
		if (xSemaphoreTake(xSemaphoreTC, 10))
		{
			updateMotor2(&rotation, md.dir);
			if (cnt++ >= 500)
			{
				cnt = 0;
				tc_stop(TC0, 0);
				xSemaphoreGive(xSemaphoreFullSpin);
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void updateMotor(int *rotation, int dir)
{

	if (dir)
		*rotation = *rotation == 1u ? 1u << 3 : *rotation >> 1;
	else
		*rotation = *rotation > (1u << 3) ? 1u : *rotation << 1;

	if (*rotation & (1 << 0))
	{
		pio_clear(MAG_4_PIO, MAG_4_IDX_MASK);
		pio_clear(MAG_2_PIO, MAG_2_IDX_MASK);
		pio_set(MAG_1_PIO, MAG_1_IDX_MASK);
	}
	else if (*rotation & (1 << 1))
	{
		pio_clear(MAG_1_PIO, MAG_1_IDX_MASK);
		pio_clear(MAG_3_PIO, MAG_3_IDX_MASK);
		pio_set(MAG_2_PIO, MAG_2_IDX_MASK);
	}
	else if (*rotation & (1 << 2))
	{
		pio_clear(MAG_2_PIO, MAG_2_IDX_MASK);
		pio_clear(MAG_4_PIO, MAG_4_IDX_MASK);
		pio_set(MAG_3_PIO, MAG_3_IDX_MASK);
	}
	else if (*rotation & (1 << 3))
	{
		pio_clear(MAG_3_PIO, MAG_3_IDX_MASK);
		pio_clear(MAG_1_PIO, MAG_1_IDX_MASK);
		pio_set(MAG_4_PIO, MAG_4_IDX_MASK);
	}
}

void updateMotor2(int *rotation, int dir)
{

	if (dir)
		*rotation = *rotation == 1u ? 1u << 3 : *rotation >> 1;
	else
		*rotation = *rotation > (1u << 3) ? 1u : *rotation << 1;

	if (*rotation & (1 << 0))
	{
		pio_clear(MAG2_4_PIO, MAG2_4_IDX_MASK);
		pio_clear(MAG2_2_PIO, MAG2_2_IDX_MASK);
		pio_set(MAG2_1_PIO, MAG2_1_IDX_MASK);
	}
	else if (*rotation & (1 << 1))
	{
		pio_clear(MAG2_1_PIO, MAG2_1_IDX_MASK);
		pio_clear(MAG2_3_PIO, MAG2_3_IDX_MASK);
		pio_set(MAG2_2_PIO, MAG2_2_IDX_MASK);
	}
	else if (*rotation & (1 << 2))
	{
		pio_clear(MAG2_2_PIO, MAG2_2_IDX_MASK);
		pio_clear(MAG2_4_PIO, MAG2_4_IDX_MASK);
		pio_set(MAG2_3_PIO, MAG2_3_IDX_MASK);
	}
	else if (*rotation & (1 << 3))
	{
		pio_clear(MAG2_3_PIO, MAG2_3_IDX_MASK);
		pio_clear(MAG2_1_PIO, MAG2_1_IDX_MASK);
		pio_set(MAG2_4_PIO, MAG2_4_IDX_MASK);
	}
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

void TC0_Handler(void)
{
	volatile uint32_t status = tc_get_status(TC0, 0);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreTC, &xHigherPriorityTaskWoken);
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

	pio_clear(LED_1_PIO, LED_1_IDX_MASK);
	pio_clear(LED_2_PIO, LED_2_IDX_MASK);
	pio_clear(LED_3_PIO, LED_3_IDX_MASK);

	pio_configure(MAG_1_PIO, PIO_OUTPUT_0, MAG_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG_2_PIO, PIO_OUTPUT_0, MAG_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG_3_PIO, PIO_OUTPUT_0, MAG_3_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG_4_PIO, PIO_OUTPUT_0, MAG_4_IDX_MASK, PIO_DEFAULT);

	pio_clear(MAG_1_PIO, MAG_1_IDX_MASK);
	pio_clear(MAG_2_PIO, MAG_2_IDX_MASK);
	pio_clear(MAG_3_PIO, MAG_3_IDX_MASK);
	pio_clear(MAG_4_PIO, MAG_4_IDX_MASK);

	pio_configure(MAG2_1_PIO, PIO_OUTPUT_0, MAG2_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG2_2_PIO, PIO_OUTPUT_0, MAG2_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG2_3_PIO, PIO_OUTPUT_0, MAG2_3_IDX_MASK, PIO_DEFAULT);
	pio_configure(MAG2_4_PIO, PIO_OUTPUT_0, MAG2_4_IDX_MASK, PIO_DEFAULT);

	pio_clear(MAG2_1_PIO, MAG2_1_IDX_MASK);
	pio_clear(MAG2_2_PIO, MAG2_2_IDX_MASK);
	pio_clear(MAG2_3_PIO, MAG2_3_IDX_MASK);
	pio_clear(MAG2_4_PIO, MAG2_4_IDX_MASK);

	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK,
				  PIO_PULLUP | PIO_DEBOUNCE);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
					but1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
					but2_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
					but3_callback);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

	pio_get_interrupt_status(BUT_1_PIO);
	pio_get_interrupt_status(BUT_2_PIO);
	pio_get_interrupt_status(BUT_3_PIO);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
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

	xSemaphoreTC = xSemaphoreCreateBinary();
	xSemaphoreFullSpin = xSemaphoreCreateBinary();
	xQueueBtn = xQueueCreate(32, sizeof(char));
	xQueueMotor = xQueueCreate(32, sizeof(motorData));
	xQueueMotor2 = xQueueCreate(32, sizeof(motorData));

	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL,
					TASK_OLED_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_cntr, "task_cntr", TASK_BUT_STACK_SIZE, NULL, TASK_BUT_STACK_PRIORITY, &xHandleCntrl) != pdPASS)
		printf("Failed to create task_cntr\r\n");

	if (xTaskCreate(task_motor, "task_motor", TASK_BUT_STACK_SIZE, NULL, TASK_BUT_STACK_PRIORITY, &xHandleMotor) != pdPASS)
		printf("Failed to create task_motor\r\n");

	if (xTaskCreate(task_motor2, "task_motor2", TASK_BUT_STACK_SIZE, NULL, TASK_BUT_STACK_PRIORITY, &xHandleMotor2) != pdPASS)
		printf("Failed to create task_motor2\r\n");

	vTaskSuspend(xHandleMotor);
	vTaskSuspend(xHandleMotor2);
	vTaskStartScheduler();
	while (1)
	{
	}

	return 0;
}
