#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "rtt.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)


#define ECHO_PIO          PIOA
#define ECHO_PIO_ID       ID_PIOA
#define ECHO_PIO_IDX      24
#define ECHO_PIO_IDX_MASK (1 << ECHO_PIO_IDX)

#define TRIGGER_PIO          PIOA
#define TRIGGER_PIO_ID       ID_PIOA
#define TRIGGER_PIO_IDX      2
#define TRIGGER_PIO_IDX_MASK (1 << TRIGGER_PIO_IDX)



/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_TRIGGER_SIZE                    (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TRIGGER_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ECHO_SIZE                    (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ECHO_STACK_PRIORITY            (tskIDLE_PRIORITY)



SemaphoreHandle_t xSemaphoreEcho;



extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void BUT_init(void);

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

volatile char isReading = 0;
volatile char tempo = 0;
volatile short distancia = 0;

void but_callback(void) {
}

void echo_callback(void){
	//printf("peixinho glub glub \n");
	if(pio_get(ECHO_PIO, PIO_INPUT,ECHO_PIO_IDX_MASK))
		rtt_init(RTT,1);
	else
		tempo = rtt_read_timer_value(RTT);
		distancia = tempo*1000000/32768;
		distancia = distancia*348/2/10000;
	isReading != isReading;

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	short tempostr[14];
	gfx_mono_draw_string("Distancia \n", 0, 0, &sysfont);
	for (;;)  {
		sprintf(tempostr, "%d cm", distancia);
    	gfx_mono_draw_string(tempostr, 0, 20, &sysfont);
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static void task_trigger(void *pvParameters){
	printf("entrou na task trigger \n");
	for(;;){
		printf("trigger\n");
		pio_set(TRIGGER_PIO,TRIGGER_PIO_IDX_MASK);
		delay_us(10);
		pio_clear(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
		
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void configure_console(void) {
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

void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	
}

void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

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


void TRG_init(void){
		pmc_enable_periph_clk(TRIGGER_PIO_ID);
		pmc_enable_periph_clk(ECHO_PIO_ID);
		
		pio_configure(TRIGGER_PIO, PIO_OUTPUT_0, TRIGGER_PIO_IDX_MASK, PIO_DEFAULT);
		pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_PULLUP);
		
		//   // Inicializa clock do periférico PIO responsavel pelo botao
		// 	pmc_enable_periph_clk(BUT1_PIO_ID);
		// 	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
		//   // a função de callback é a: but_callback()
		//   pio_handler_set(BUT1_PIO,
		//                   BUT1_PIO_ID,
		//                   BUT1_PIO_IDX_MASK,
		//                   PIO_IT_FALL_EDGE,
		//                   but1_callback);

		pio_handler_set(ECHO_PIO,
		ECHO_PIO_ID,
		ECHO_PIO_IDX_MASK,
		PIO_IT_EDGE,
		echo_callback);
		
		//   pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
		pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
		
		//   pio_get_interrupt_status(BUT1_PIO);
		pio_get_interrupt_status(ECHO_PIO);
		//
		//   pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 40);
		pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_IDX_MASK, 40);
		
		NVIC_EnableIRQ(ECHO_PIO_ID);
		//   NVIC_EnableIRQ(BUT1_PIO_ID);

		//   NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
		NVIC_SetPriority(ECHO_PIO_ID, 2);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	TRG_init();
	
	//Create the echo queue
	xSemaphoreEcho = xSemaphoreCreateBinary();
    if (xSemaphoreEcho == NULL)
	  printf("falha em criar a queue xQueueADC \n");


	/* Create control tasks */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS)
	  printf("Failed to create oled task\r\n");

 	if (xTaskCreate(task_trigger, "trigger", TASK_TRIGGER_SIZE, NULL, TASK_TRIGGER_STACK_PRIORITY, NULL) != pdPASS)
 		printf("Failed to create trigger task\r\n");
 		
// 	if (xTaskCreate(task_echo, "echo", TASK_OLED_STACK_SIZE, NULL, TASK_ECHO_STACK_PRIORITY, NULL) != pdPASS)
// 		printf("Failed to create task \r\n");


	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
	}