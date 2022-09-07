#include <asf.h>
#include <time.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LED
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)

// BTN1 OLED
#define BTN1_PIO      PIOD
#define BTN1_PIO_ID       ID_PIOD
#define BTN1_IDX      28
#define BTN1_IDX_MASK (1 << BTN1_IDX)

// BTN2 OLED
#define BTN2_PIO      PIOC
#define BTN2_PIO_ID       ID_PIOC
#define BTN2_IDX      31
#define BTN2_IDX_MASK (1 << BTN2_IDX)

// BTN3 OLED
#define BTN3_PIO      PIOA
#define BTN3_PIO_ID       ID_PIOA
#define BTN3_IDX      19
#define BTN3_IDX_MASK (1 << BTN3_IDX)

/* flag */
volatile char btn1_flag;
volatile char btn2_flag;
volatile char btn3_flag;


/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
int pisca_led(int n, int t, int idx_pisca);

void but1_callback(void) {
	btn1_flag = 1;
}

void but2_callback(void) {
	btn2_flag = 1;
}

void but3_callback(void) {
	btn3_flag = 1;
}

// pisca led N vez no periodo T
int pisca_led(int n, int t, int idx_pisca) {
	for (int i = idx_pisca; i < n; i++) {
		if (btn1_flag || btn2_flag || btn3_flag){
			return i;
		}
		float j = 3.5 * i + 15;
		gfx_mono_draw_rect((int) j, 5, 2, 10, GFX_PIXEL_SET);
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
	}
	for(int i=120;i>=15;i-=2){
		
		gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
		
	}
	return 0;
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void) {

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	// BTN 1
	pmc_enable_periph_clk(BTN1_PIO_ID);
	pio_configure(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN1_PIO, BTN1_IDX_MASK, 60);
	pio_handler_set(BTN1_PIO,
	BTN1_PIO_ID,
	BTN1_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);
	pio_enable_interrupt(BTN1_PIO, BTN1_IDX_MASK);
	pio_get_interrupt_status(BTN1_PIO);
	NVIC_EnableIRQ(BTN1_PIO_ID);
	NVIC_SetPriority(BTN1_PIO_ID, 4); // Prioridade 4
	
	// BTN 2
	pmc_enable_periph_clk(BTN2_PIO_ID);
	pio_configure(BTN2_PIO, PIO_INPUT, BTN2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN2_PIO, BTN2_IDX_MASK, 60);
	pio_handler_set(BTN2_PIO,
	BTN2_PIO_ID,
	BTN2_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	pio_enable_interrupt(BTN2_PIO, BTN2_IDX_MASK);
	pio_get_interrupt_status(BTN2_PIO);
	NVIC_EnableIRQ(BTN2_PIO_ID);
	NVIC_SetPriority(BTN2_PIO_ID, 4); // Prioridade 4
	
	// BTN 3
	pmc_enable_periph_clk(BTN3_PIO_ID);
	pio_configure(BTN3_PIO, PIO_INPUT, BTN3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN3_PIO, BTN3_IDX_MASK, 60);
	pio_handler_set(BTN3_PIO,
	BTN3_PIO_ID,
	BTN3_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);
	pio_enable_interrupt(BTN3_PIO, BTN3_IDX_MASK);
	pio_get_interrupt_status(BTN3_PIO);
	NVIC_EnableIRQ(BTN3_PIO_ID);
	NVIC_SetPriority(BTN3_PIO_ID, 4); // Prioridade 4
}
	

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

    // Init OLED
	gfx_mono_ssd1306_init();
  
	// configura botao com interrupcao
	io_init();
	int periodo = 400;
	int pisca = 1;
	int idx_pisca = 0;
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {

			if (btn1_flag) {
				delay_ms(150);
				if (!pio_get(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK)){
					periodo -= 100;
				} else {
					periodo += 100;
				}
				btn1_flag = 0;
			} else if (btn2_flag){
				pisca = !pisca;
				btn2_flag = 0;
			} else if (btn3_flag) {
				delay_ms(150);
				if (pio_get(BTN3_PIO, PIO_INPUT, BTN3_IDX_MASK)){
					periodo += 100;
				}
				btn3_flag = 0;
			} else {
				// Escreve na tela um circulo e um texto
				if (pisca) {
					char str[128]; //

					sprintf(str, "%d", periodo); //

					gfx_mono_draw_string(str, 50, 16, &sysfont);
		
					idx_pisca = pisca_led(30, periodo, idx_pisca);
				}
			}

			
	}
}
