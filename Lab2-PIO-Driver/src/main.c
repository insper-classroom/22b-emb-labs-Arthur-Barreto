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

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

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
void _pio_set(Pio *p_pio, const uint32_t ul_mask);
void _pio_clear(Pio *p_pio, const uint32_t ul_mask);
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable);
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_attribute);
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_default_level,
	const uint32_t ul_multidrive_enable,const uint32_t ul_pull_up_enable);
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask);
void _delay_ms(int delay_ms);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void _pio_set(Pio *p_pio, const uint32_t ul_mask){
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask){
	p_pio->PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
	if(ul_pull_up_enable){
		p_pio->PIO_PUER = ul_mask;
	} else {
		p_pio->PIO_PUDR = ul_mask;
	}
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_attribute){
	if (_PIO_PULLUP & ul_attribute){
		_pio_pull_up(p_pio,ul_mask,1);
	}
	if (_PIO_DEGLITCH & ul_attribute){
		p_pio->PIO_IFSCDR = ul_mask;
	}
	if(_PIO_DEBOUNCE & ul_attribute){
		p_pio->PIO_IFSCER = ul_mask;
	}
}

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_default_level,
const uint32_t ul_multidrive_enable,const uint32_t ul_pull_up_enable){
	/* habilita para configurar */
	p_pio->PIO_PER=ul_mask;
	/* habilita para saida */
	p_pio->PIO_OER=ul_mask;
	/* definindo saída inicial como 0 ou 1 */
	/* pio_set inicializa com 1, led apagado */
	if (ul_default_level){
		_pio_set(p_pio,ul_mask);
	} else {
		_pio_clear(p_pio,ul_mask);
	}
	/* dessativando o open drain */
	p_pio->PIO_MDDR = ul_multidrive_enable;
	/* ativando o pull-up */
	_pio_pull_up(p_pio,ul_mask,ul_pull_up_enable);
}

uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask){
	
	/* existe PIO_INPUT, PIO_OUTPUT_0 e PIO_OUTPUT_1 */
	uint32_t ul_reg;
	/* VERIFICAR O TIPO, se entrada ou saida */
	if ((ul_type == PIO_OUTPUT_0) || (ul_type == PIO_OUTPUT_1)) {
		ul_reg = p_pio->PIO_ODSR;
	} else {
		ul_reg = p_pio->PIO_PDSR;
	}
	/* bitwise and para dar o retorno */
	if ((ul_reg & ul_mask) == 0) {
		return 0;
	} else {
		return 1;
	}
}

void _delay_ms(int delay_ms){
	/* CLOCK LINHA A LINHA = 2/3 do clock real */
	/* clck = 200 Mh, 200e6 */
	/* tmepo em ms, logo multiplicar a entrada por 1e-3 */
	int ciclos = delay_ms*200e3;
	for (int i=0;i<ciclos;i++){
		asm("nop");
	}
}

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
	_pio_set_input(BUT1_PIO, BUT1_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	// pio_configure(pio, PIO_INPUT, mask, PIO_PULLUP);
}

void set_led(uint32_t id, Pio* pio, uint32_t led_mask){
	/* nivel 1 apaga o led, nivel 0 acende o led */
	pmc_enable_periph_clk(id);
	_pio_set_output(pio,led_mask,1,0,0);
	// pio_configure(pio, PIO_OUTPUT_1, led_mask, PIO_DEFAULT);
}

void pisca_led(Pio* pio, uint32_t mask, uint32_t tms){
	
	/* 0 ACENDE E 1 APAGA */
	_pio_clear(pio, mask);    // Coloca 0 no pino do LED
	_delay_ms(tms);           // Delay por software de 200 ms
	_pio_set(pio, mask);      // Coloca 1 no pino LED
	_delay_ms(tms);
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
	
	if (!_pio_get(BUT1_PIO,PIO_INPUT,BUT1_IDX_MASK)){
		for(int i=0;i<5;i++){
			pisca_led(LED1_PIO,LED1_IDX_MASK,300);
		}
	} else {
		_pio_set(LED1_PIO, LED1_IDX_MASK);
	}

	if (!_pio_get(BUT2_PIO,PIO_INPUT,BUT2_IDX_MASK)){
		for(int i=0;i<5;i++){
			pisca_led(LED2_PIO,LED2_IDX_MASK,300);
		}
	} else {
		_pio_set(LED2_PIO, LED2_IDX_MASK);
	}

	if (!_pio_get(BUT3_PIO,PIO_INPUT,BUT3_IDX_MASK)){
	for(int i=0;i<5;i++){
		pisca_led(LED3_PIO,LED3_IDX_MASK,300);
	}
	} else {
		_pio_set(LED3_PIO, LED3_IDX_MASK);
	}
}
  return 0;
}