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
volatile double vec_dist[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

/** RTOS  */

#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

/* fila adicional para queue 5 */
QueueHandle_t xQueueMy;

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

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (estado_echo == 0) {
		/* comeca a conta */
		//rtt_sel_source(RTT, false);
		rtt_init(RTT,1);
		estado_echo = 1;
	} else {
		/* calcula a distancia */
		int contagens = rtt_read_timer_value(RTT);
		double delta_t =  ((double) contagens) / 32768.0;
		double dist = delta_t * 170.0;

		/* enviar dado para fila  */
		estado_echo = 0;
		xQueueSendFromISR(xQueueMy, &dist, &xHigherPriorityTaskWoken);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	char str[32];
	for (;;) {
		// setr e dar clear depois de 10 micro o trigger
		pio_set(TRIGGER_PIO, TRIGGER_PIO_PIN_MASK);
		delay_us(10);
		pio_clear(TRIGGER_PIO, TRIGGER_PIO_PIN_MASK);
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}

static void task_final(void *pvParameters) {

    double dist;
    while (1) {

        if (xQueueReceive(xQueueMy, &dist, (TickType_t)0)) {
			printf("entrou no if \n");
            char str[15];

            /* limpando oled */
            gfx_mono_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
            /* verificando se temos uma leitura fora do range */
            if (dist > 4.0 || dist < 0.02) {
                gfx_mono_draw_string("Error", 0, 0, &sysfont);

            } else {

                sprintf(str, "%2.2lf m", dist);
                gfx_mono_draw_string(str, 0, 0, &sysfont);

                /* deslocar todos as leituras uma casa a esquerda, e depois guardar a atual */
                for (int i = 0; i <= 8; i++) {
                    vec_dist[i] = vec_dist[i + 1];
                }

                vec_dist[9] = dist;

                for (int i = 0; i < 10; i++) {
                    double altura = -10.2 * vec_dist[i] + 25.0;
                    int x = i * 12;
                    int y = altura;

                    gfx_mono_draw_string("--", i * 12, (int)altura, &sysfont);
                }
            }
        }
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

	/* Create task final */
	if (xTaskCreate(task_final, "final", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create final task\r\n");
	}

	/* criando fila adicional */
    xQueueMy = xQueueCreate(32, sizeof(double));

	if (xQueueMy == NULL){
        printf("Não criou a fila adicional");
    }

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
