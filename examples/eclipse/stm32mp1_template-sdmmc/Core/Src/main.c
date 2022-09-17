#include "main.h"
#include "stm32mp1xx_hal.h"
#include "ff.h"
#include "diskio.h"

void UART4_Init(void);
void GPIO_Init(void);
void set_interrupt(uint32_t irq_num, uint32_t cpu, uint32_t priority, uint32_t edge_detect, void* callback);
void Error_Handler(void);
void soft_breakpoint(void);

UART_HandleTypeDef huart4;
void (*irq_callback[249])();

void main() {
	HAL_Init();
	HAL_Delay(1);
	GPIO_Init();
	UART4_Init();
	puts("\r\n");

	FATFS fs;
	FIL file;
	char path[] = "0:0";
	uint32_t bytesRead;
	uint32_t bytesWritten;
	uint8_t res;
	uint8_t readText[100];
	uint8_t writeText[] = "stm32mp1-baremetal is AWESOME!!!";
	DIR dir;
	FILINFO fno;

	f_mount(&fs, path, 0);
	// Open root directory
	if(f_opendir(&dir, "/") == FR_OK) {
		puts("The following directories and files found in /\r");
		do
		{
			f_readdir(&dir, &fno);
			if(fno.fname[0] != 0)
				// Print directories and files in root
				printf("\t%s\r\n", fno.fname);
		} while(fno.fname[0] != 0);
		f_closedir(&dir);
	}
	if(f_open(&file, "stm32mp1.txt", FA_READ) == FR_OK) {
		if(f_read(&file, readText, sizeof(readText), (void *)&bytesRead) == FR_OK) {
			f_close(&file);
			printf("stm32mp1.txt: \r\n%.*s\r\n", bytesRead, readText);
		}
	} else {
		if(f_open(&file, "stm32mp1.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
			puts("Created file stm32mp1.txt\r");
			res = f_write(&file, writeText, sizeof(writeText), (void *)&bytesWritten);
			if((res == FR_OK) && (bytesWritten > 0)) {
				f_close(&file);
				printf("%u bytes written to stm32mp1.txt\r\n", bytesWritten);
			} else {
				printf("Failed to write to stm32mp1.txt! Error 0x%08x, wrote %d bytes\r\n", res, bytesWritten);
			}
		}
	}

	while (1) {
		HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
		HAL_Delay(200);
	}
}

void UART4_Init(void)
{
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.FifoMode = UART_FIFOMODE_ENABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
}

void GPIO_Init() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Red and Blue LEDs */
	GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_interrupt(uint32_t irq_num, uint32_t cpu, uint32_t priority, uint32_t edge_detect, void* callback) {
	GIC_DisableIRQ(irq_num);
	GIC_SetTarget(irq_num, cpu);
	GIC_SetPriority(irq_num, priority);
	GIC_SetConfiguration(irq_num, edge_detect);
	irq_callback[irq_num] = callback;
	GIC_ClearPendingIRQ(irq_num);
	GIC_EnableIRQ(irq_num);
}

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

// Handy utility when using a debugger
void soft_breakpoint() {
	volatile int stop = 1;
	while (stop) {
		// Attach a debugger and manually change the value at the address of `stop` in RAM from 1 to 0
	}
}

void __attribute__((interrupt("IRQ"))) IRQ_Handler() {
	uint32_t irq_num = GIC_AcknowledgePending();
	irq_callback[irq_num]();
	GIC_EndInterrupt(irq_num);
}

void Error_Handler() {
	while(1) {
		HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
		HAL_Delay(250);
		// handle error
	}
}
