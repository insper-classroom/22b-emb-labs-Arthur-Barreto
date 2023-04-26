#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "mcu6050.h"
#include <math.h>
#include "Fusion/Fusion.h"

/* led placa */
#define LED_PIO PIOC
#define LED_ID ID_PIOC
#define LED_IDX 8
#define LED_MASK (1 << LED_IDX)

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

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_SENSOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_SENSOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueLed;
QueueHandle_t xQueueYaw;
SemaphoreHandle_t xSemaphoreLed;

BaseType_t xHigherPriorityTaskWoken = pdTRUE;

/* prototypes */
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

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

void RTT_Handler(void){
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);
	
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS){
	}
}


int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
	int32_t ierror = 0x00;
	twihs_packet_t p_packet;
	p_packet.chip = dev_addr;
	p_packet.addr[0] = reg_addr;
	p_packet.addr_length = 1;
	p_packet.buffer = reg_data;
	p_packet.length = cnt;
	ierror = twihs_master_write(TWIHS2, &p_packet);
	return (int8_t) ierror;
}

int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
	int32_t ierror = 0x00;
	twihs_packet_t p_packet;
	p_packet.chip = dev_addr;
	p_packet.addr[0] = reg_addr;
	p_packet.addr_length = 1;
	p_packet.buffer = reg_data;
	p_packet.length = cnt;
	ierror = twihs_master_read(TWIHS2, &p_packet);
}

void mcu6050_i2c_bus(void){
	twihs_options_t mcu6050_option;
	pmc_enable_periph_clk(ID_TWIHS2);
	mcu6050_option.master_clk = sysclk_get_cpu_hz;
	mcu6050_option.speed = 40000;
	twihs_master_init(TWIHS2, &mcu6050_option);
}

void mcu6050_i2c_bus_init(void){
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
}

float modulos(float x, float y, float z){
	return (float) sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

static void task_oled(void *pvParameters){
	gfx_mono_ssd1306_init();
	
	for(;;){
		//vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

static void task_sensor(void *pvParameters){
	
	for (;;)  {
		//vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

static void task_imu(void *pvParameters){
	mcu6050_i2c_bus_init();
	mcu6050_i2c_bus();
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);
	
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];
	uint8_t rtn;
	int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
	volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
	volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
	float proc_acc_x, proc_acc_y, proc_acc_z;

	int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
	float proc_gyr_x, proc_gyr_y, proc_gyr_z;
	
	rtn = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [probe]\n");
	} else{
		printf("[DADO] [i2c] probe OK\n");
	}
	
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX,1);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	}
	
	if(bufferRX[0] != 0x68){
		printf("[ERRO] [mcu] [Wrong device] [0x%2X]\n", bufferRX[0]);
	} else {
		printf("[Correct Value] [bufferRX]");
	}
	
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");

	// Aceletromtro em 2G
	bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");

	// Configura range giroscopio para operar com 250 �/s
	bufferTX[0] = 0x00; // 250 �/s
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");
	
	while(1){
		  // Le valor do acc X High e Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);

		  // Le valor do acc y High e  Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);

		  // Le valor do acc z HIGH e Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);

		  // Dados s�o do tipo complemento de dois
		  raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		  raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		  raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);

		  // Le valor do gyr X High e Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);

		  // Le valor do gyr y High e  Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);

		  // Le valor do gyr z HIGH e Low
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		  mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);

		  // Dados s�o do tipo complemento de dois
		  raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		  raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		  raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);

		  // Dados em escala real
		  proc_acc_x = (float)raw_acc_x/16384;
		  proc_acc_y = (float)raw_acc_y/16384;
		  proc_acc_z = (float)raw_acc_z/16384;

		  proc_gyr_x = (float)raw_gyr_x/131;
		  proc_gyr_y = (float)raw_gyr_y/131;
		  proc_gyr_z = (float)raw_gyr_z/131;
		  //printf("Acel. x: %f ,y: %f ,z: %f", proc_acc_x, proc_acc_y, proc_acc_z);
		  //printf("Girs. x: %f ,y: %f ,z: %f", proc_gyr_x, proc_gyr_y, proc_gyr_z);
		  if (modulos(proc_acc_x, proc_acc_y, proc_acc_z) > 2){
			  xSemaphoreGiveFromISR(xSemaphoreLed, &xHigherPriorityTaskWoken);
			  //printf("%f\n", modulos(proc_acc_x,proc_acc_y,proc_acc_z));
		  }
		  const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z};
		  const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};
		  // uma amostra a cada 1ms
		  float dT = 0.1;
		  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);
		  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		  float yaw = euler.angle.yaw;
		  xQueueSendFromISR(xQueueYaw, &yaw, &xHigherPriorityTaskWoken);
		  //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw); 
		  vTaskDelay(100);
	}
}

static void task_house_down(void *pvParameters){
	while(1){
		if (xSemaphoreTake(xSemaphoreLed, 10)) {
			pio_clear(LED_PIO, LED_MASK);
			vTaskDelay(100);
			pio_set(LED_PIO, LED_MASK);
			vTaskDelay(100);
		}
	}	
};

void led_orientacao(int orienta){
	if (orienta == 0){ //esquerda
		pio_clear(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(100);
		pio_set(LED1_PIO, LED1_IDX_MASK);
	}
	
	if (orienta == 1){ //frente
		pio_clear(LED2_PIO, LED2_IDX_MASK);
		vTaskDelay(100);
		pio_set(LED2_PIO, LED2_IDX_MASK);
	}
	
	if (orienta == 2){ //direita
		pio_clear(LED3_PIO, LED3_IDX_MASK);
		vTaskDelay(100);
		pio_set(LED3_PIO, LED3_IDX_MASK);
	}
}

static void task_orientacao(void *pvParameters){
	float yaw = 0.00;
	gfx_mono_ssd1306_init();
	
	while(1){
		if(xQueueReceive(xQueueYaw, &yaw, 1000)){
			//printf("%f", yaw);
			if (yaw < -10.0){
				led_orientacao(0);
				printf("Esquerda");
			}
			if (yaw >= -10.0 & yaw < 50.0){
				led_orientacao(1);
				printf("Centro");
			}
			if(yaw >= 50.0){
				led_orientacao(2);
				printf("Direita");
			}
		}
	}
}

void led_init(){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	
	pio_set(LED1_PIO, LED1_IDX_MASK);
	pio_set(LED2_PIO, LED2_IDX_MASK);
	pio_set(LED3_PIO, LED3_IDX_MASK);
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

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	led_init();
	/* Initialize the console uart */
	configure_console();
	printf("Comecou\n");
	
	xSemaphoreLed = xSemaphoreCreateBinary();
	xQueueLed = xQueueCreate(32, sizeof(int));
	xQueueYaw = xQueueCreate(32, sizeof(float));
	
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_sensor, "sensor", TASK_SENSOR_STACK_SIZE, NULL, TASK_SENSOR_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create sensor task\r\n");
	}
	
	if (xTaskCreate(task_imu, "imu", TASK_SENSOR_STACK_SIZE, NULL, TASK_SENSOR_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create imu task\r\n");
	}
	
	if(xTaskCreate(task_house_down, "led", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create led task\r\n");
	}
	
	if(xTaskCreate(task_orientacao, "orientacao", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create orientacao task\r\n");
	}
	
	if (xQueueLed == NULL){
		printf("Failed to create Ping List\n");
	}
	
	if (xSemaphoreLed == NULL){
		printf("Failed to create semaphore\n");
	}
	
	if (xQueueYaw == NULL){
		printf("Failed to create Yaw List\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();
	/* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
