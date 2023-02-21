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
/* LED 1*/
#define LED1_PIO           PIOA                // periferico que controla o LED
#define LED1_PIO_ID        ID_PIOA              // ID do periférico PIOC (controla LED)
#define LED1_PIO_IDX       0                   // ID do LED no PIO
#define LED1_IDX_MASK  (1 << LED1_PIO_IDX)   // Mascara para CONTROLARMOS o LED
/* LED2 */
#define LED2_PIO           PIOC                // periferico que controla o LED
#define LED2_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED)
#define LED2_PIO_IDX       30                   // ID do LED no PIO
#define LED2_IDX_MASK  (1 << LED2_PIO_IDX)   // Mascara para CONTROLARMOS o LED
/* LED 3 */
#define LED3_PIO           PIOB                // periferico que controla o LED
#define LED3_PIO_ID        ID_PIOB              // ID do periférico PIOC (controla LED)
#define LED3_PIO_IDX       2                   // ID do LED no PIO
#define LED3_IDX_MASK  (1 << LED3_PIO_IDX)   // Mascara para CONTROLARMOS o LED
// Configuracoes do botao
/* botao 1*/
#define BUT1_PIO         PIOD                   // Periferico que controla o botao
#define BUT1_PIO_ID      ID_PIOD
#define BUT1_PIO_IDX     28
#define BUT1_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.
/* BOTAO 2*/
#define BUT2_PIO         PIOC                   // Periferico que controla o botao
#define BUT2_PIO_ID      ID_PIOC
#define BUT2_PIO_IDX     31
#define BUT2_IDX_MASK (1u << BUT2_PIO_IDX) // esse já está pronto.
/* BOTAO 3 */
#define BUT3_PIO         PIOA                   // Periferico que controla o botao
#define BUT3_PIO_ID      ID_PIOA
#define BUT3_PIO_IDX     19
#define BUT3_IDX_MASK (1u << BUT3_PIO_IDX) // esse já está pronto.


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
void pisca_led(Pio* pio, uint32_t mask, uint32_t tms);
void set_led(uint32_t id, Pio* pio, uint32_t led_mask);
void set_but(uint32_t id, Pio* pio, uint32_t mask);

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
	
	/* SET LED */
	set_led(LED1_PIO_ID,LED1_PIO,LED1_IDX_MASK);
	set_led(LED2_PIO_ID,LED2_PIO,LED2_IDX_MASK);
	set_led(LED3_PIO_ID,LED3_PIO,LED3_IDX_MASK);
	
	/* set button */
	set_but(BUT1_PIO_ID,BUT1_PIO,BUT1_IDX_MASK);
	set_but(BUT2_PIO_ID,BUT2_PIO,BUT2_IDX_MASK);
	set_but(BUT3_PIO_ID,BUT3_PIO,BUT3_IDX_MASK);
}

void set_but(uint32_t id, Pio* pio, uint32_t mask){
	// Inicializa PIO do botao
	pmc_enable_periph_clk(id);
	// pio_configure(pio, PIO_INPUT, mask, PIO_PULLUP);

	// // configura pino ligado ao botão como entrada com um pull-up.
	pio_set_input(pio,mask,PIO_PULLUP);
	// // configurando o pull-up
	// pio_pull_up(pio,mask,PIO_PULLUP);
}

void set_led(uint32_t id,Pio* pio, uint32_t led_mask){
	/* nivel 1 apaga o led, nivel 0 acende o led */
	pmc_enable_periph_clk(id);
	// pio_configure(pio, PIO_OUTPUT_1, led_mask, PIO_DEFAULT);
	pio_set_output(pio,led_mask,1,0,0);
}

void pisca_led(Pio* pio, uint32_t mask, uint32_t tms){
	
	/* 0 ACENDE E 1 APAGA */
	pio_clear(pio, mask);    // Coloca 0 no pino do LED
	delay_ms(tms);           // Delay por software de 200 ms
	pio_set(pio, mask);      // Coloca 1 no pino LED
	delay_ms(tms);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void) {
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1){
	
	if (!pio_get(BUT1_PIO,PIO_INPUT,BUT1_IDX_MASK)){
		for(int i=0;i<5;i++){
			pisca_led(LED1_PIO,LED1_IDX_MASK,100);
		}
	} else {
		pio_set(LED1_PIO, LED1_IDX_MASK);
	}

	if (!pio_get(BUT2_PIO,PIO_INPUT,BUT2_IDX_MASK)){
		for(int i=0;i<5;i++){
			pisca_led(LED2_PIO,LED2_IDX_MASK,100);
		}
	} else {
		pio_set(LED2_PIO, LED2_IDX_MASK);
	}

	if (!pio_get(BUT3_PIO,PIO_INPUT,BUT3_IDX_MASK)){
	for(int i=0;i<5;i++){
		pisca_led(LED3_PIO,LED3_IDX_MASK,100);
	}
	} else {
		pio_set(LED3_PIO, LED3_IDX_MASK);
	}
}
  return 0;
}