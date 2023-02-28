#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* includes                                                             */
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

/* flag */
volatile char but1_flag = 0;
volatile char but2_flag = 0;
volatile char but3_flag = 0;
volatile int tms = 100;

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
void pisca_led(Pio* pio, uint32_t mask);
void escreve_frequencia(int t);
void limpa_oled(void);

void io_init(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

void but1_callback(void) {
    but1_flag = 1;
}

void but2_callback(void) {
    but2_flag = 1;
}

void but3_callback(void) {
    but3_flag = 1;
}

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Inicializa botao button1 do kit com interrupcao
void io_init(void) {

    // Configura led
    pmc_enable_periph_clk(LED1_PIO_ID);
    pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);

    // Inicializa clock do periférico PIO responsavel pelo botao
	/* precisa ativar para cada botão */
    pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

    // Configura PIO para lidar com o pino do botão como entrada
    // com pull-up
    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);

	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);

	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

    // Configura interrupção no pino referente ao botao e associa
    // função de callback caso uma interrupção for gerada
    // a função de callback é a: but_callback()
    pio_handler_set(BUT1_PIO,
                    BUT1_PIO_ID,
                    BUT1_IDX_MASK,
                    PIO_IT_EDGE,
                    &but1_callback);

	pio_handler_set(BUT2_PIO,
				BUT2_PIO_ID,
				BUT2_IDX_MASK,
				PIO_IT_RISE_EDGE,
				&but2_callback);

	pio_handler_set(BUT3_PIO,
			BUT3_PIO_ID,
			BUT3_IDX_MASK,
			PIO_IT_RISE_EDGE,
			&but3_callback);

	/* PIO_IT_RISE_EDGE */

    // Ativa interrupção e limpa primeira IRQ gerada na ativacao
    pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
    pio_get_interrupt_status(BUT1_PIO);

	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
    pio_get_interrupt_status(BUT2_PIO);

	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);
    pio_get_interrupt_status(BUT3_PIO);

    // Configura NVIC para receber interrupcoes do PIO do botao
    // com prioridade 4 (quanto mais próximo de 0 maior)
    
	/* botão de iniciar com a menor prioridade */
	/* botão de parar com a maior prioridade */

	NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 6); // prioridade 6

	NVIC_EnableIRQ(BUT2_PIO_ID);
    NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4

	NVIC_EnableIRQ(BUT3_PIO_ID);
    NVIC_SetPriority(BUT3_PIO_ID, 5); // Prioridade 5
}

void pisca_led(Pio* pio, uint32_t mask){
	/* 0 ACENDE E 1 APAGA */
	for (int i = 0; i < 30; i++){
		if (but2_flag){
			but2_flag = 0;
			break;
		}
		if (but3_flag){
			but3_flag = 0;
			if (tms > 20){
				tms -= 20;
			}
		}
		/* escreve barrinha de progueso */
		gfx_mono_draw_rect(4*i, 22, 2, 10, GFX_PIXEL_SET);
		escreve_frequencia(tms);
		pio_clear(pio, mask);    // Coloca 0 no pino do LED
		delay_ms(tms);          
		pio_set(pio, mask);      // Coloca 1 no pino LED
		delay_ms(tms);
	}
	limpa_oled();
}

void escreve_frequencia(int t){
	float freq = 1/(2*t*1e-3);
	char str[128]; 
	sprintf(str, "f = %.2f Hz", freq);
	gfx_mono_draw_string(str, 0, 0, &sysfont);
}

void limpa_oled(void){
	char str[128]; 
	sprintf(str, "Terminou !");
	gfx_mono_draw_string(str, 0, 0, &sysfont);
	for(int i = 100; i <= 128 ; i += 2){
		gfx_mono_draw_rect(i, 0, 2, 14, GFX_PIXEL_CLR);
		delay_us(1);	
	}
	for(int i=120;i>=0;i-=2){
		gfx_mono_draw_rect(i, 22, 2, 10, GFX_PIXEL_CLR);
		delay_us(1);	
	}
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.

int main (void) {
	// Inicializa clock
    sysclk_init();
	// Desativa watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
	// configura botao com interrupcao
    io_init();

	board_init();
	sysclk_init();
	delay_init();

 	// Init OLED
	gfx_mono_ssd1306_init();
	/* desenha circulo e paralavra */
	// gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
  	// gfx_mono_draw_string("mundo", 50,16, &sysfont);
  
  	/* Insert application code here, after the board has been initialized. */
	while(1) {

		/* interrupção irá mudar a flag, e no loop iremos tratar o caso */
		if (but1_flag) {
			but1_flag = 0;
			delay_ms(300);
			if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)) {
				// PINO == 1 --> Borda de subida
				tms += 100;
			} 
			else {
				// PINO == 0 --> Borda de descida
				if (tms > 100){
					tms -= 100;
				}
			}
			pisca_led(LED1_PIO, LED1_IDX_MASK);
		}

		// Escreve na tela a barra que vai e volta
		// for(int i=0;i<=120;i+=2){
		// 	gfx_mono_draw_rect(i, 22, 2, 10, GFX_PIXEL_SET);
		// 	delay_ms(10);	
		// }
		// for(int i=120;i>=0;i-=2){
		// 	gfx_mono_draw_rect(i, 22, 2, 10, GFX_PIXEL_CLR);
		// 	delay_ms(10);	
		// }

		// Entra em sleep mode
        pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		
	}
	return 0;
}
