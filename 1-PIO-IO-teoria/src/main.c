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
#define LED_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED
// Configuracoes do botao
#define BUT_PIO         PIOA                   // Periferico que controla o botao
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO_IDX     11
#define BUT_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.


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
void pisca(int tms);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void) {
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;	
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_set_output(LED_PIO, LED_IDX_MASK, 0, 0, 0);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// configura pino ligado ao botão como entrada com um pull-up.
	pio_set_input(BUT_PIO,BUT_IDX_MASK,PIO_DEFAULT);
	
	// configurando o pull-up
	pio_pull_up(BUT_PIO,BUT_IDX_MASK,PIO_PULLUP);
}

void pisca(int tms){
	pio_clear(LED_PIO, LED_IDX_MASK);    // Coloca 0 no pino do LED
	delay_ms(tms);                        // Delay por software de 200 ms
		
	pio_set(LED_PIO, LED_IDX_MASK);      // Coloca 1 no pino LED
	delay_ms(tms);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1){
	
	/*
	pio_clear(LED_PIO, LED_IDX_MASK);    // Coloca 0 no pino do LED
	delay_ms(3000);                        // Delay por software de 200 ms
	  
	pio_set(LED_PIO, LED_IDX_MASK);      // Coloca 1 no pino LED
	delay_ms(1000);                        // Delay por software de 200 ms
	*/
	
	int estado_pino = 0;
	estado_pino = pio_get(BUT_PIO,PIO_INPUT,BUT_IDX_MASK);
	
	if (!estado_pino){
		for(int i=0;i<5;i++){
			pisca(200);
		}
	} else {
		pio_set(LED_PIO, LED_IDX_MASK);      // Coloca 1 no pino LED
	}
	}
  return 0;
}