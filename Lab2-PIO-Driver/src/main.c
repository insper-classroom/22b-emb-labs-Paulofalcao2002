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

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)


 

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

/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
	if (ul_pull_up_enable){
		p_pio->PIO_PUER = ul_mask;	
	} else {
		p_pio->PIO_PUDR = ul_mask;
	}
 }
 
 /**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute) {
	if (ul_attribute != 0){
		if (ul_attribute && _PIO_PULLUP){
			_pio_pull_up(p_pio, ul_mask, 1);
		}
		if (ul_attribute && _PIO_DEGLITCH){
			p_pio->PIO_IFSCDR = ul_mask;
			p_pio->PIO_IFER = ul_mask;
		}
		if (ul_attribute && _PIO_DEBOUNCE){
			p_pio->PIO_IFSCER = ul_mask;
			p_pio->PIO_IFER = ul_mask;
		}
	}

}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain.
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_default_level,
        const uint32_t ul_multidrive_enable,
        const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;
	if (ul_default_level){
		_pio_set(p_pio, ul_mask);
	} else {
		_pio_clear(p_pio, ul_mask);
	}
	if (ul_multidrive_enable){
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}
	_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
}


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
	 _pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);

	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);

	
	// OLED1
	pmc_enable_periph_clk(OLED1_ID);
	pio_set_output(OLED1_PIO, OLED1_IDX_MASK, 0, 0, 0);
	
	// OLED2
	pmc_enable_periph_clk(OLED2_ID);
	_pio_set_output(OLED2_PIO, OLED2_IDX_MASK, 0, 0, 0);
	
	// OLED3
	pmc_enable_periph_clk(OLED3_ID);
	_pio_set_output(OLED3_PIO, OLED3_IDX_MASK, 0, 0, 0);
	
	// BTN1
	pmc_enable_periph_clk(BTN1_ID);
	_pio_set_input(BTN1_PIO, BTN1_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	//_pio_pull_up(BTN1_PIO, BTN1_IDX_MASK, 1);
	
	// BTN2
	pmc_enable_periph_clk(BTN2_ID);
	_pio_set_input(BTN2_PIO, BTN2_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	// _pio_pull_up(BTN2_PIO, BTN2_IDX_MASK, 1);
	
	// BTN3
	pmc_enable_periph_clk(BTN3_ID);
	_pio_set_input(BTN3_PIO, BTN3_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	// _pio_pull_up(BTN3_PIO, BTN3_IDX_MASK, 1);

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
	  _pio_set(LED_PIO, LED_PIO_IDX_MASK);  
	  int valorBtn = pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK);
	  if (!valorBtn){
		  for (int i = 0; i < 5; i++){
			  _pio_clear(LED_PIO, LED_PIO_IDX_MASK); 
			  delay_ms(500);
			  _pio_set(LED_PIO, LED_PIO_IDX_MASK);  
			  delay_ms(500);
		  }                 
	  }  
	  int valorBtn1 = pio_get(BTN1_PIO, PIO_INPUT, BTN1_IDX_MASK);
	  if (!valorBtn1){
		  _pio_clear(OLED1_PIO, OLED1_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  _pio_set(OLED1_PIO, OLED1_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
	  
	  int valorBtn2 = pio_get(BTN2_PIO, PIO_INPUT, BTN2_IDX_MASK);
	  if (!valorBtn2){
		  _pio_clear(OLED2_PIO, OLED2_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  _pio_set(OLED2_PIO, OLED2_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
	  
	  int valorBtn3 = pio_get(BTN3_PIO, PIO_INPUT, BTN3_IDX_MASK);
	  if (!valorBtn3){
		  _pio_clear(OLED3_PIO, OLED3_IDX_MASK);   // ligar o led se tiver apertado
	  } else {
		  _pio_set(OLED3_PIO, OLED3_IDX_MASK);        // desligar o led quando o btn não está apertado
	  }
  }
  return 0;
}
