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

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

// PIO DO TRIGGER
#define TRIG_PIO       PIOD
#define TRIG_PIO_ID    ID_PIOD
#define TRIG_IDX       30u
#define TRIG_IDX_MASK  (1u << TRIG_IDX)

// PIO DO ECHO
#define ECHO_PIO       PIOA
#define ECHO_PIO_ID    ID_PIOA
#define ECHO_IDX       6u
#define ECHO_IDX_MASK  (1u << ECHO_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void echo_init(void);
void trig_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

QueueHandle_t xQueueTempo;
QueueHandle_t xQueueMedidas;

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

void echo_callback(void) {
	if(pio_get(ECHO_PIO, PIO_INPUT, ECHO_IDX_MASK) == 1 ){
		RTT_init(18000, 0, 0);
	} else {
		uint32_t dt = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueTempo, &dt, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	// gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	// gfx_mono_draw_string("oii", 0, 20, &sysfont);
	uint32_t cm;
	for (;;)  {
		if (xQueueReceive(xQueueMedidas, &cm, 2000)) {
			char str[128]; //
			sprintf(str, "%d cm  ", cm); //
			gfx_mono_draw_string(str, 50, 16, &sysfont);
			printf("%d cm\n", cm);
		} else {
			printf("erro no oled\n");
		}

	}
}

static void task_hcsr04(void *pvParameters) {
	echo_init();
	trig_init();
	uint32_t dt;
	for (;;)  {
		pio_set(TRIG_PIO, TRIG_IDX_MASK);
		delay_us(10);
		pio_clear(TRIG_PIO, TRIG_IDX_MASK);
		if( xQueueReceive( xQueueTempo, &dt, 8000 )){
			int cms = 34000 * dt / (18000 * 2);
			xQueueSend(xQueueMedidas, &cms, 1000);
		} else {
			printf("erro \n");
		}
		vTaskDelay(1000);
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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

void trig_init(void){
	  pmc_enable_periph_clk(TRIG_PIO_ID);
	  pio_configure(TRIG_PIO, PIO_OUTPUT_1, TRIG_IDX_MASK, PIO_DEFAULT);
}

static void echo_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_IDX_MASK, 0);
	pio_enable_interrupt(ECHO_PIO, ECHO_IDX_MASK);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_IDX_MASK, PIO_IT_EDGE , echo_callback);
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



/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueTempo = xQueueCreate(32, sizeof(uint32_t) );
	xQueueMedidas = xQueueCreate(32, sizeof(uint32_t) );

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_hcsr04, "hcsr04", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create hcsr05 task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
