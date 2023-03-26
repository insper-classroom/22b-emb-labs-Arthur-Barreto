#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* echo pin */
#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_PIN 6
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

/* TRIGGER */
#define TRIGGER_PIO PIOD
#define TRIGGER_PIO_ID ID_PIOD
#define TRIGGER_PIO_PIN 30
#define TRIGGER_PIO_PIN_MASK (1 << TRIGGER_PIO_PIN)

/* volatile  */
volatile int estado_echo = 0;
volatile float vec_dist[10] = {0,0,0,0,0,0,0,0,0,0};
volatile char error = 0;

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void echo_calback(void);
static void io_init(void);


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


void echo_calback(void) {
	
	if (estado_echo == 0) {
		/* comeca a conta */
		//rtt_sel_source(RTT, false);
		rtt_init(RTT,1);
		estado_echo = 1;
	} else {
		/* calcula a distancia */
		int contagens = rtt_read_timer_value(RTT);
		
		float delta_t =  ((float) contagens) / 32768.0;
		float dist = delta_t * 170.0;
		char str[10];
		
		/* limpando o oled */
		gfx_mono_draw_filled_rect(0,0,128,32,GFX_PIXEL_CLR);
		// gfx_mono_draw_string("          ", 0, 0, &sysfont);
		// gfx_mono_draw_string("          ", 0, 25, &sysfont);

		/* identificar se houve leitura errada  */
		if (dist > 4.0) {
			gfx_mono_draw_string("Error", 0, 0, &sysfont);
			/* fazer a ultima leitura ser igual a atual e ajustar o vetor de distancias */
			for (int i = 0; i <= 8 ; i++){
				vec_dist[i] = vec_dist[i+1];
			}
			vec_dist[9] = vec_dist[8];

		} else {

			sprintf(str, "%.2f m", dist);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			/* deslocar todos as leituras uma casa a esquerda, e depois guardar a atual */
			for (int i = 0; i <= 8 ; i++){
				vec_dist[i] = vec_dist[i+1];
			}

			vec_dist[9] = dist;

			for (int i = 0; i<10; i++){
				float altura = -10.2*vec_dist[i] + 25.0;
				gfx_mono_draw_string("--", i*12, (int) altura, &sysfont);
			}
		}
		estado_echo = 0;
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();

	for (;;) {
		// setr e dar clear depois de 10 micro o trigger
		pio_set(TRIGGER_PIO, TRIGGER_PIO_PIN_MASK);
		delay_us(10);
		pio_clear(TRIGGER_PIO, TRIGGER_PIO_PIN_MASK);
		vTaskDelay(250 / portTICK_PERIOD_MS);
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

static void io_init(void) {

	/* configurando o trigger como saida */
	pmc_enable_periph_clk(TRIGGER_PIO_ID);
	pio_configure(TRIGGER_PIO, PIO_OUTPUT_0, TRIGGER_PIO_PIN_MASK, PIO_DEFAULT);

	/* configurando o echo como entrada */
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);

	/* configurando o echo para interrução */
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, &echo_calback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
