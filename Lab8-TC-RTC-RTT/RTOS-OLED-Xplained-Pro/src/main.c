#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

// OLED 
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

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueBTN;
QueueHandle_t xQueueHandlers;

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void io_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void escreve_hora(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

void but1_callback(void) {
  printf("but1 foi apertado\n");
  int id = 1;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueBTN, &id, &xHigherPriorityTaskWoken);
}

void but2_callback(void) {
  printf("but2 foi apertado\n");
  int id = 2;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueBTN, &id, &xHigherPriorityTaskWoken);
}

void but3_callback(void) { 
  printf("but3 foi apertado\n");
  int id = 3;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueueBTN, &id, &xHigherPriorityTaskWoken);
}

void TC0_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 0);
	// printf("Handler\n");

	// Manda pra queue o id do Handler TC0
	int id = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  	xQueueSendFromISR(xQueueHandlers, &id, &xHigherPriorityTaskWoken);
}

void TC3_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 0);
	// printf("Handler\n");

	// Manda pra queue o id do Handler TC0
	int id = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  	xQueueSendFromISR(xQueueHandlers, &id, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		int id = 2;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(xQueueHandlers, &id, &xHigherPriorityTaskWoken);
	}  
}

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
	// o código para irq de segundo vem aqui
		// printf("Segundo\n");
		int id = 4;
		xQueueSendFromISR(xQueueHandlers, &id, &xHigherPriorityTaskWoken);
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
		// printf("RTC HANDLER\n");
		int id = 3;
		xQueueSendFromISR(xQueueHandlers, &id, &xHigherPriorityTaskWoken);
    }

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	// gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	// gfx_mono_draw_string("oii", 0, 20, &sysfont);

	io_init();
	pio_set(LED_1_PIO, LED_1_IDX_MASK);
	pio_set(LED_2_PIO, LED_2_IDX_MASK);
	pio_set(LED_3_PIO, LED_3_IDX_MASK);
	pio_set(LED_PIO, LED_IDX_MASK);

	// TC 
	TC_init(TC0, ID_TC0, 0, 5);
	tc_start(TC0, 0);
	TC_init(TC1, ID_TC3, 0, 4);
	tc_start(TC1, 0);

	// RTT
	RTT_init(10, 40, RTT_MR_ALMIEN);

	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);

	int id_handler;
	int id_BTN;
	uint32_t current_hour, current_min, current_sec;

	int led_3_on = 0;

	escreve_hora();
	for (;;)  {
		if (xQueueReceive(xQueueHandlers, &id_handler, 100)) {
			if (id_handler == 0) {
				pin_toggle(LED_PIO, LED_IDX_MASK);
			} else if (id_handler == 1) {
				pin_toggle(LED_1_PIO, LED_1_IDX_MASK);
				if (led_3_on) {
					pin_toggle(LED_3_PIO, LED_3_IDX_MASK);
					led_3_on = 0;
				}
			} else if (id_handler == 2) {
				pin_toggle(LED_2_PIO, LED_2_IDX_MASK);
				RTT_init(10, 40, RTT_MR_ALMIEN);
			} else if (id_handler == 3) {
				pin_toggle(LED_3_PIO, LED_3_IDX_MASK);
				led_3_on = 1;
			} else if (id_handler == 4) {
				escreve_hora();
			}
		}

		if (xQueueReceive(xQueueBTN, &id_BTN, 1)) {
			if (id_BTN == 3) {
				rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
				rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
			}
		}
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void escreve_hora(void) {
	char str[128];
	uint32_t current_hour, current_min, current_sec;

	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	sprintf(str, current_sec > 9 ? "%d:%d:%d" : "%d:%d:0%d", current_hour, current_min, current_sec);

	gfx_mono_draw_filled_rect(0, 0, 128, 30, GFX_PIXEL_CLR);
	gfx_mono_draw_string(str, 0, 0, &sysfont);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
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

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
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
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void pin_toggle(Pio *pio, uint32_t mask) {
  if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio,mask);
}

void io_init(void) {
  pmc_enable_periph_clk(LED_1_PIO_ID);
  pmc_enable_periph_clk(LED_2_PIO_ID);
  pmc_enable_periph_clk(LED_3_PIO_ID);
  pmc_enable_periph_clk(LED_PIO_ID);
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);

  pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
  but1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but3_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but2_callback);

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

static void configure_console(void) {
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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	xQueueBTN = xQueueCreate(100, sizeof(int));
	if (xQueueBTN == NULL)
		printf("falha em criar a queue xQueueADC \n");

	xQueueHandlers = xQueueCreate(100, sizeof(int));
	if (xQueueHandlers == NULL)
		printf("falha em criar a queue xQueueADC \n");

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
