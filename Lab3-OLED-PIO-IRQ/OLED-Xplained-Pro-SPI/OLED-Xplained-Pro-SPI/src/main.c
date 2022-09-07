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
#define BTN2_ID       ID_PIOC
#define BTN2_IDX      31
#define BTN2_IDX_MASK (1 << BTN2_IDX)

/* flag */
volatile char btn1_flag;

int freq = 400;
long counts = 0;


/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t);

void but_callback(void) {
	btn1_flag = 1;
}

// pisca led N vez no periodo T
void pisca_led(int n, int t) {
		for (int i = 0; i < n; i++) {
			if (btn1_flag){
				return;
			}
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(t);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(t);
		}
	
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void) {

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BTN1_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN1_PIO, BTN1_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BTN1_PIO,
	BTN1_PIO_ID,
	BTN1_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BTN1_PIO, BTN1_IDX_MASK);
	pio_get_interrupt_status(BTN1_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BTN1_PIO_ID);
	NVIC_SetPriority(BTN1_PIO_ID, 4); // Prioridade 4
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
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {

			if (btn1_flag) {
				delay_ms(150);
				if (!pio_get(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK)){
					freq -= 100;
				} else {
					freq += 100;
				}
				btn1_flag = 0;
			} else {
				// Escreve na tela um circulo e um texto
				char str[128]; //

				sprintf(str, "%d", freq); //

				gfx_mono_draw_string(str, 50, 16, &sysfont);
				
				
				pisca_led(30, freq);
			}

			
	}
}
