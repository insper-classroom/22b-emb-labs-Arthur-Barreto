#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

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


/* RTOS */
SemaphoreHandle_t xSemaphoreRTT;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
void TC8_Handler(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void io_init(void);

/* interrupções */

volatile int flag_rtt;

/* RTT */
void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTC);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}
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

/* FIM RTT */

/**
* @Brief Inicializa o pino do LED
*/

/**
* @Brief Inverte o valor do pino 0->1/ 1->0
*/
void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/* INICIO TC */

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	/** ATIVA PMC PCK6 TIMER_CLOCK1  */
	if(ul_tcclks == 0 )
	pmc_enable_pck(PMC_PCK_6);
	
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC8_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 2);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_IDX_MASK);  
}

/* FIM DO TC */

static void task_led_rtt(void){
	
	int estado_led_2 = 0;
	
	for (;;) {
		 if (xSemaphoreTake(xSemaphoreRTC, 4)) {
			 
			 /* inverte o estado do led */
			 estado_led_2 = !estado_led_2;
			 
			 if (estado_led_2) {
				 pio_clear(LED2_PIO,LED2_IDX_MASK);
			 } else {
				 pio_set(LED2_PIO,LED2_IDX_MASK);
			 }
			 
		 }
	}
}

void io_init(void){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_1, LED1_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(LED3_PIO_IDX);
	pio_configure(LED3_PIO, PIO_OUTPUT_1, LED3_IDX_MASK, PIO_DEFAULT);
}

int main (void) {
	board_init();
	sysclk_init();
	io_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	 xSemaphoreRTT = xSemaphoreCreateBinary();
	 if (xSemaphoreRTT == NULL)
		printf("falha em criar o semaforo do bot?o da placa \n");
	
	delay_init();
  	// Init OLED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string("mundo", 50,16, &sysfont);

	TC_init(TC2, ID_TC8, 2, 4);
	tc_start(TC2, 2);
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
