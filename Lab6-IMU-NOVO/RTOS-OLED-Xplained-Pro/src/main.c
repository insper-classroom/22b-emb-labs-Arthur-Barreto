#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_text.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "sysfont.h"

#include "mcu6050.h"

/* LED 1*/
#define LED1_PIO PIOA                     // periferico que controla o LED
#define LED1_PIO_ID ID_PIOA               // ID do periférico PIOC (controla LED)
#define LED1_PIO_IDX 0                    // ID do LED no PIO
#define LED1_IDX_MASK (1 << LED1_PIO_IDX) // Mascara para CONTROLARMOS o LED
/* LED2 */
#define LED2_PIO PIOC                     // periferico que controla o LED
#define LED2_PIO_ID ID_PIOC               // ID do periférico PIOC (controla LED)
#define LED2_PIO_IDX 30                   // ID do LED no PIO
#define LED2_IDX_MASK (1 << LED2_PIO_IDX) // Mascara para CONTROLARMOS o LED
/* LED 3 */
#define LED3_PIO PIOB                     // periferico que controla o LED
#define LED3_PIO_ID ID_PIOB               // ID do periférico PIOC (controla LED)
#define LED3_PIO_IDX 2                    // ID do LED no PIO
#define LED3_IDX_MASK (1 << LED3_PIO_IDX) // Mascara para CONTROLARMOS o LED
/* botao 1*/
#define BUT1_PIO PIOD // Periferico que controla o botao
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.
/* BOTAO 2*/
#define BUT2_PIO PIOC // Periferico que controla o botao
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_IDX_MASK (1u << BUT2_PIO_IDX) // esse já está pronto.
/* BOTAO 3 */
#define BUT3_PIO PIOA // Periferico que controla o botao
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_IDX_MASK (1u << BUT3_PIO_IDX) // esse já está pronto.

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_IMU_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
static void io_init(void);
void mcu6050_i2c_bus_init(void);
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
    printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    for (;;) {
    }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
    configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void){

}

void but2_callback(void){

}

void but3_callback(void){

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_imu(void) {
	mcu6050_i2c_bus_init();

	/* buffer para recebimento de dados */
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];

	/* resultado da função */
	uint8_t rtn;

	rtn = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
    if(rtn != TWIHS_SUCCESS){
        printf("[ERRO] [i2c] [probe] \n");
    } else {
        printf("[DADO] [i2c] probe OK\n" );
    }

	// Lê registrador WHO AM I
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	} else {
		printf("[DADO] [i2c] %x:%x", MPU6050_RA_WHO_AM_I, bufferRX[0]);
	}

    for (;;) {
    }
}

static void task_oled(){
	for(;;){
		
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void mcu6050_i2c_bus_init(void) {
    twihs_options_t mcu6050_option;
    pmc_enable_periph_clk(ID_TWIHS2);

    /* Configure the options of TWI driver */
    mcu6050_option.master_clk = sysclk_get_cpu_hz();
    mcu6050_option.speed = 40000;
    twihs_master_init(TWIHS2, &mcu6050_option);
}

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
	/* configuração i2c */
    pmc_enable_periph_clk(ID_PIOD);
    pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
    pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);

	/* configuração leds e botoes */
    pmc_enable_periph_clk(LED1_PIO_ID);
    pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED2_PIO_ID);
    pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED3_PIO_ID);
    pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);

    pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);

	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);

	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

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

    pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
    pio_get_interrupt_status(BUT1_PIO);

	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
    pio_get_interrupt_status(BUT2_PIO);

	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);
    pio_get_interrupt_status(BUT3_PIO);

	NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT2_PIO_ID);
    NVIC_SetPriority(BUT2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT3_PIO_ID);
    NVIC_SetPriority(BUT3_PIO_ID, 4);

}


int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip = dev_addr;
    p_packet.addr[0] = reg_addr;
    p_packet.addr_length = 1;
    p_packet.buffer = reg_data;
    p_packet.length = cnt;

    ierror = twihs_master_write(TWIHS2, &p_packet);

    return (int8_t)ierror;
}

int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip = dev_addr;
    p_packet.addr[0] = reg_addr;
    p_packet.addr_length = 1;
    p_packet.buffer = reg_data;
    p_packet.length = cnt;

    // TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
    //       conseguirmos pegar o valor correto.
    ierror = twihs_master_read(TWIHS2, &p_packet);

    return (int8_t)ierror;
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

    if (xTaskCreate(task_imu, "imu", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create imu task\r\n");
    }

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* RTOS n�o deve chegar aqui !! */
    while (1) {
    }

    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0;
}
