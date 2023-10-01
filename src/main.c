#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* Sensor */
#define ECHO_PIO PIOA 
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_PIN 21
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

#define TRIG_PIO PIOD 
#define TRIG_PIO_ID ID_PIOD
#define TRIG_PIO_PIN 27
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ECHO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ECHO_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueTempoF;
QueueHandle_t xQueueDist;

/** prototypes */
void but_callback(void);
static void BUT_init(void);
static void Echo_init(void);
static void Trig_init(void);
static void task_echo(void *pvParameters);
static void task_oled(void *pvParameters);
volatile float distancia;

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

void but_callback(void) {
	if (pio_get(ECHO_PIO,PIO_INPUT, ECHO_PIO_PIN_MASK)) {
		// printf("Echo: %d\n", pio_get(ECHO_PIO,PIO_INPUT, ECHO_PIO_PIN_MASK));
		rtt_init(RTT, 1);
	}
	else {
		// printf("Echo: %d\n", pio_get(ECHO_PIO,PIO_INPUT, ECHO_PIO_PIN_MASK));
		uint32_t time = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueTempoF, &time, 1000);
	}
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	for (;;)  {
		if (xQueueReceive(xQueueDist, &distancia, 10)){
			if (distancia >=2 && distancia <=400){
			char dist_final[10];
			sprintf(dist_final, "%.2f", distancia);
			gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			gfx_mono_draw_string("Distancia: ", 0, 0, &sysfont);
			gfx_mono_draw_string(dist_final, 0, 10, &sysfont);
			printf("Distancia: {%s}", dist_final);}
			else {
				printf("Erro");
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Distancia: ", 0, 0, &sysfont);
				gfx_mono_draw_string("Erro", 0, 10, &sysfont);
			}
		}
	}
}

static void task_echo(void *pvParameters) {
	for (;;)  {
		
		pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
		delay_us(10);
		pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);
		uint32_t tempof;
		
		if (xQueueReceive(xQueueTempoF, &tempof, 1000)){
			// calcula a distancia
			float distancia = ((tempof *340*100)/32678)/2; 
			//printar no serial
			printf("Distancia: %.2f \n", distancia);
			// envia a distancia para a queue da task oled
			xQueueSend(xQueueDist, &distancia, 10);

			//delay de 1s
			vTaskDelay(1000);
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


static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);

}

static void Trig_init(void){
	/*conf Trig*/
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_PIN_MASK, PIO_DEFAULT);
	// pio_set_output(TRIG_PIO, TRIG_PIO_PIN_MASK, 0, 0, 0);	
	
}


static void Echo_init(void){
	/*conf Echo*/
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

	/* conf bot�o como entrada */
	pmc_enable_periph_clk(ECHO_PIO);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, 0	);
	// pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_PIN_MASK, 60);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID,ECHO_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
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
	Trig_init();
	Echo_init();

	xQueueDist = xQueueCreate(32, sizeof(float));
	if (xQueueDist == NULL){
		printf("falha em criar a queue \n");
	}
	xQueueTempoF = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueTempoF == NULL){
		printf("falha em criar a queue \n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_echo, "sensor", TASK_ECHO_STACK_SIZE, NULL, TASK_ECHO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create sensor task\r\n");
	}
	printf("Tasks created\r");
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

