/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "conf_board.h"

#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define c_EOF 'X'

#define VERIFICA_COMMAND_ID 'S'

#define BUT1_COMMAND_ID '1'
#define BUT1_PIO		   PIOD
#define BUT1_PIO_ID		   ID_PIOD
#define BUT1_PIO_IDX       28
#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)

//BOTÃO PLAY/PAUSE
#define BUT2_COMMAND_ID '2'
#define BUT2_PIO		   PIOC
#define BUT2_PIO_ID		   ID_PIOC
#define BUT2_PIO_IDX       31
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

#define BUT3_COMMAND_ID '3'
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// LED1
#define LED1_PIO      PIOA
#define LED1_PIO_ID   ID_PIOA
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED2
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED3
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

//Prioridade dos Botoes
#define BUT1_PRIOR 5 //Prioridade botao XOLED1
#define BUT2_PRIOR 5 //Prioridade botao XOLED2
#define BUT3_PRIOR 5 //Prioridade botao XOLED3

void BUT1_callback(void);
void BUT2_callback(void);
void BUT3_callback(void);

typedef struct {
	uint value;
} adcData;

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

volatile uint32_t g_ul_value = 0;

QueueHandle_t queue_adc;
QueueHandle_t queue_but0;
QueueHandle_t queue_but1;
QueueHandle_t queue_but2;


SemaphoreHandle_t semaphore_adc;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void BUT1_callback(void){
	
	char but_flag = 0;
	if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) == 0){
		but_flag = '0';
		xQueueSendFromISR(queue_but0, &but_flag, 0);
		} else {
		but_flag = '1';
		xQueueSendFromISR(queue_but0, &but_flag, 0);
	}	
}

void BUT2_callback(void){
	char but_flag = 0;
	
	if(pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK) == 0){
		but_flag = '0';
		xQueueSendFromISR(queue_but1, &but_flag, 0);
		} else {
		but_flag = '1';
		xQueueSendFromISR(queue_but1, &but_flag, 0);
	}
}

void BUT3_callback(void){
	char but_flag = 0;
	
	if(pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK) == 0){
		but_flag = '0';
		xQueueSendFromISR(queue_but2, &but_flag, 0);
		} else {
		but_flag = '1';
		xQueueSendFromISR(queue_but2, &but_flag, 0);
	}
}

static void AFEC_pot_Callback(void){
	g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	g_is_conversion_done = true;
	
	adcData adc_vol;
	adc_vol.value  = g_ul_value;
	xQueueSendFromISR(queue_adc, &adc_vol,0);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);
	
	/* configura IRQ */
	afec_set_callback(afec, afec_channel,	callback, 1);
	NVIC_SetPriority(afec_id, 5);
	NVIC_EnableIRQ(afec_id);
}

void LED1_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, estado, 0, 0 );
};

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) { }

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

void vol_func(uint32_t g_ul_value,char *vol_char){
	// 0 .. 9
	if (g_ul_value<=370){
		*vol_char = '0';
	}
	else if(g_ul_value<=740){
		*vol_char = '1';
	}
	else if(g_ul_value<=1110){
		*vol_char = '2';
	}
	else if(g_ul_value<=1480){
		*vol_char = '3';
	}
	else if(g_ul_value<=1850){
		*vol_char = '4';
	}
	else if(g_ul_value<=2220){
		*vol_char = '5';
	}
	else if(g_ul_value<=2590){
		*vol_char = '6';
	}
	else if(g_ul_value<=2960){
		*vol_char = '7';
	}
	else if(g_ul_value<=3330){
		*vol_char = '8';
	}
	else{
		*vol_char = '9';
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
void send_command(char id, char status){
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, id);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, status);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, c_EOF);
}

int response(){
	uint32_t resp = 0;
	if (!usart_read(UART_COMM, &resp));
	{
		if (resp=='X')
		{
			return 1;
		}
	}
	return 0;
}

static void task_led(void *pvParameters) {
	UNUSED(pvParameters);
	for (;;) {
		LED_Toggle(LED0);
		vTaskDelay(1000);
	}
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

void init(void){
	
	LED1_init(1);
	board_init();
	sysclk_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	configure_console();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	pio_set_input(BUT1_PIO,BUT1_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT2_PIO,BUT2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUT3_PIO,BUT3_PIO_IDX_MASK,PIO_DEFAULT);

	pio_pull_up(BUT1_PIO,BUT1_PIO_IDX_MASK,1);
	pio_pull_up(BUT2_PIO,BUT2_PIO_IDX_MASK,1);
	pio_pull_up(BUT3_PIO,BUT3_PIO_IDX_MASK,1);
	
	//Set interruptores do projeto
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, BUT1_PRIOR); // Priority 2
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, BUT2_PRIOR); // Priority 2
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, BUT3_PRIOR); // Priority 2

	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, BUT1_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, BUT2_callback);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, BUT3_callback);
	

	
}

void task_adc(void){
	
	while(true){
	
		vTaskDelay(500/portTICK_PERIOD_MS);
		
		afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
		afec_start_software_conversion(AFEC_POT);
	}
}

void task_send(void){
	
	queue_but0 = xQueueCreate(8, sizeof(char));
	queue_but1 = xQueueCreate(8, sizeof(char));
	queue_but2 = xQueueCreate(8, sizeof(char));
	
	adcData adc;
	queue_adc = xQueueCreate(5, sizeof(adcData));

	init();

	
	/* inicializa e configura adc up*/
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	//afec_start_software_conversion(AFEC_POT);
	
	char but;
	char vol_char = '0';
	char vol_char_old = '0';
	adcData adc_vol;
	
		while(true){
			if(xQueueReceive(queue_but0, &but, (TickType_t)0) == pdTRUE){
			send_command(BUT1_COMMAND_ID, but);
			}
			
			if(xQueueReceive(queue_but1, &but, (TickType_t)0) == pdTRUE){
				send_command(BUT2_COMMAND_ID, but);
			}
			
			if(xQueueReceive(queue_but2, &but, (TickType_t)0) == pdTRUE){
				send_command(BUT3_COMMAND_ID, but);
			}
			
			if(xQueueReceive(queue_adc,&(adc_vol), (TickType_t)  100 / portTICK_PERIOD_MS)){
					vol_func(g_ul_value, &adc_vol.value);
				
					// garante que volume só e' enviado quando for um novo valor
					if (adc_vol.value != vol_char_old) {
						send_command('v', adc_vol.value);
					}
					vol_char_old = vol_char;
			}
		}
};

void task_response(void){
	while(true){
		send_command(VERIFICA_COMMAND_ID, '0');
		vTaskDelay(1000);
		if (response())
		{
			pio_clear(LED1_PIO,LED1_IDX_MASK);
			}else{
			pio_set(LED1_PIO,LED1_IDX_MASK);
		}
	}
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	if (xTaskCreate(task_send, "send", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test send task\r\n");
	}
	if (xTaskCreate(task_adc, "adc", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
	}
	if (xTaskCreate(task_response, "response", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test response task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
