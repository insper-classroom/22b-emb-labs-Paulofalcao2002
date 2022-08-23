/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 // periferico que controla o LED
// #
#define LED_PIO_ID        ID_PIOC                  // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Configuracoes do botao
#define BUT_PIO       PIOA
#define BUT_PIO_ID    ID_PIOA
#define BUT_PIO_IDX    11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

// LED OLED1 
#define OLED1_PIO      PIOA
#define OLED1_ID       ID_PIOA
#define OLED1_IDX      0
#define OLED1_IDX_MASK (1 << OLED1_IDX)

// LED OLED2
#define OLED2_PIO      PIOC
#define OLED2_ID       ID_PIOC
#define OLED2_IDX      30
#define OLED2_IDX_MASK (1 << OLED2_IDX)

// LED OLED3
#define OLED3_PIO      PIOB
#define OLED3_ID       ID_PIOB
#define OLED3_IDX      2
#define OLED3_IDX_MASK (1 << OLED3_IDX)

// BTN1 OLED
#define BTN1_PIO      PIOD
#define BTN1_ID       ID_PIOD
#define BTN1_IDX      28
#define BTN1_IDX_MASK (1 << BTN1_IDX)

// BTN2 OLED
#define BTN2_PIO      PIOC
#define BTN2_ID       ID_PIOC
#define BTN2_IDX      31
#define BTN2_IDX_MASK (1 << BTN2_IDX)

// BTN3 OLED
#define BTN3_PIO      PIOA
#define BTN3_ID       ID_PIOA
#define BTN3_IDX      19
#define BTN3_IDX_MASK (1 << BTN3_IDX)
 

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	 // Initialize the board clock
	 sysclk_init();

	 // Desativa WatchDog Timer
	 WDT->WDT_MR = WDT_MR_WDDIS;
	 
	 // Ativa o PIO na qual o LED foi conectado
	 // para que possamos controlar o LED.
	 pmc_enable_periph_clk(LED_PIO_ID);
	 
	 //Inicializa PC8 como saída
	 pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);

	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	
	// OLED1
	pmc_enable_periph_clk(OLED1_ID);
	pio_set_output(OLED1_PIO, OLED1_IDX_MASK, 0, 0, 0);
	
	// OLED2
	pmc_enable_periph_clk(OLED2_ID);
	pio_set_output(OLED2_PIO, OLED2_IDX_MASK, 0, 0, 0);
	
	// OLED3
	pmc_enable_periph_clk(OLED3_ID);
	pio_set_output(OLED3_PIO, OLED3_IDX_MASK, 0, 0, 0);
	
	// BTN1
	pmc_enable_periph_clk(BTN1_ID);
	pio_set_input(BTN1_PIO, BTN1_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BTN1_PIO, BTN1_IDX_MASK, 1);
	
	// BTN2
	pmc_enable_periph_clk(BTN2_ID);
	pio_set_input(BTN2_PIO, BTN2_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BTN2_PIO, BTN2_IDX_MASK, 1);
	
	// BTN3
	pmc_enable_periph_clk(BTN3_ID);
	pio_set_input(BTN3_PIO, BTN3_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BTN3_PIO, BTN3_IDX_MASK, 1);

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  // inicializa sistema e IOs
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  pio_set(LED_PIO, LED_PIO_IDX_MASK);  
	  int valorBtn = pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK);
	  if (!valorBtn){
		  for (int i = 0; i < 5; i++){
			  pio_clear(LED_PIO, LED_PIO_IDX_MASK); 
			  delay_ms(500);
			  pio_set(LED_PIO, LED_PIO_IDX_MASK);  
			  delay_ms(500);
		  }                 
	  }  
	  int valorBtn1 = pio_get(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK);
	  if (!valorBtn1){
		  pio_clear(OLED1_PIO, OLED1_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  pio_set(OLED1_PIO, OLED1_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
	  
	  int valorBtn2 = pio_get(BTN2_PIO, PIO_INPUT, BTN2_IDX_MASK);
	  if (!valorBtn2){
		  pio_clear(OLED2_PIO, OLED2_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  pio_set(OLED2_PIO, OLED2_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
	  
	  int valorBtn3 = pio_get(BTN3_PIO, PIO_INPUT, BTN3_IDX_MASK);
	  if (!valorBtn3){
		  pio_clear(OLED3_PIO, OLED3_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  pio_set(OLED3_PIO, OLED3_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
  }
  return 0;
}
