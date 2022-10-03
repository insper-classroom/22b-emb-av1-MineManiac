#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa 1*/
#define BUT1_PIO     PIOD
#define BUT1_PIO_ID  ID_PIOD
#define BUT1_PIO_PIN 28
#define BUT1_PIO_PIN_MASK (1 << BUT1_PIO_PIN)

/* Botao da placa 2*/
#define BUT2_PIO     PIOC
#define BUT2_PIO_ID  ID_PIOC
#define BUT2_PIO_PIN 31
#define BUT2_PIO_PIN_MASK (1 << BUT2_PIO_PIN)

/* Botao da placa 3*/
#define BUT3_PIO     PIOA
#define BUT3_PIO_ID  ID_PIOA
#define BUT3_PIO_PIN 19
#define BUT3_PIO_PIN_MASK (1 << BUT3_PIO_PIN)

/* Fase 0*/
#define FASE0_PIO     PIOD
#define FASE0_PIO_ID  ID_PIOD
#define FASE0_PIO_PIN 30
#define FASE0_PIO_PIN_MASK (1 << FASE0_PIO_PIN)

/* Fase 1*/
#define FASE1_PIO     PIOA
#define FASE1_PIO_ID  ID_PIOA
#define FASE1_PIO_PIN 6
#define FASE1_PIO_PIN_MASK (1 << FASE1_PIO_PIN)

/* Fase 2*/
#define FASE2_PIO     PIOC
#define FASE2_PIO_ID  ID_PIOC
#define FASE2_PIO_PIN 19
#define FASE2_PIO_PIN_MASK (1 << FASE2_PIO_PIN)

/* Fase 3*/
#define FASE3_PIO     PIOA
#define FASE3_PIO_ID  ID_PIOA
#define FASE3_PIO_PIN 2
#define FASE3_PIO_PIN_MASK (1 << FASE3_PIO_PIN)


/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
void FASE_init(void);

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
/* recursos RTOS                                                        */
/************************************************************************/

/** Queue for msg log send data */
QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	int angulo = 45;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueModo, (void *) &angulo,&xHigherPriorityTaskWoken);
}

void but2_callback(void) {
	int angulo = 90;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueModo, (void *) &angulo,&xHigherPriorityTaskWoken);
}

void but3_callback(void) {
	int angulo = 180;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueModo, (void *) &angulo,&xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	/* iniciliza botoes */
	BUT_init();
	
	uint32_t graus = 0;
	
	uint32_t steps = 0;
	
	gfx_mono_ssd1306_init();
	

	for (;;)  {
		if (xQueueReceive(xQueueModo, &graus, (TickType_t) 0)) {
			/* atualiza frequencia */
			steps = graus / 0.17578125;
			
			/* envia nova frequencia para a task_led */
			xQueueSend(xQueueSteps, (void *)&steps, 5);
				
			printf("MODO: %d \n", graus);
			gfx_mono_draw_string("MODO", 0, 0, &sysfont);
			sprintf(graus, "%d", graus);
			gfx_mono_draw_string(graus, 0, 20, &sysfont);
		}
	}
}

static void task_motor(void *pvParameters) {
	

	for (;;)  {
		

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
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but1_callback);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_PIN_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but2_callback);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_PIN_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but3_callback);
}

void FASE_init(){
	pmc_enable_periph_clk(FASE0_PIO_ID);
	pio_set_output(FASE0_PIO, FASE0_PIO_PIN_MASK, 1, 0, 0);

	pmc_enable_periph_clk(FASE1_PIO_ID);
	pio_set_output(FASE1_PIO, FASE1_PIO_PIN_MASK, 1, 0, 0);
	
	pmc_enable_periph_clk(FASE2_PIO_ID);
	pio_set_output(FASE2_PIO, FASE2_PIO_PIN_MASK, 1, 0, 0);
	
	pmc_enable_periph_clk(FASE3_PIO_ID);
	pio_set_output(FASE3_PIO, FASE3_PIO_PIN_MASK, 1, 0, 0);
};

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	/* cria queue com 32 "espacos" */
	/* cada espaço possui o tamanho de um inteiro*/
	xQueueModo = xQueueCreate(32, sizeof(uint32_t));
	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	
	if (xQueueModo == NULL){
		printf("falha em criar a queue Modo \n");
	}
	if (xQueueSteps == NULL){
		printf("falha em criar a queue Steps \n");
	}


	/* Create task to control oled */
	if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	} else {
	printf("task modo created \r\n");
	}
	
	if (xTaskCreate(task_modo, "motor", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
		} else {
		printf("task motor created \r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
