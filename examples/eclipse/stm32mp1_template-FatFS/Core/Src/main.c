#include "main.h"
#include "stm32mp1xx_hal.h"
#include "ff.h"
#include "diskio.h"

void CLK_Config(void);
void UART4_Init(void);
void GPIO_Init(void);
void set_interrupt(uint32_t irq_num, uint32_t cpu, uint32_t priority, uint32_t edge_detect, void* callback);
void Error_Handler(void);
void soft_breakpoint(void);
char *path_to(char *str);

UART_HandleTypeDef huart4;
void (*irq_callback[249])();

char sd_path[] = "0:/";
char mmc_path[] = "1:/";
char path_buf[128];
char *path;

void main() {
	HAL_Init();
	CLK_Config();
	HAL_Delay(1);
	GPIO_Init();
	UART4_Init();

	FATFS fs;
	FIL file;
	uint32_t bytesRead;
	uint32_t bytesWritten;
	uint8_t res;
	uint8_t readText[100];
	uint8_t writeText[] = "stm32mp1-baremetal is AWESOME!!!";
	DIR dir;
	FILINFO fno;

	// Set device to test
	path = mmc_path;

	puts("Force mounting drive\r");
	if(f_mount(&fs, path, 1) != FR_OK) {
			puts("Failed to mount drive!\r\n");
			Error_Handler();
	}
	puts("Drive mounted!\r");
	puts("Creating directory \"test\"\r");
	res = f_mkdir(path_to("test4"));
	if(res == FR_EXIST) {
		puts("Directory already exists\r");
	} else if(res != FR_OK) {
		puts("mkdir failed!\r\n");
		Error_Handler();
	}
	puts("Opening root directory\r");
	res = f_opendir(&dir, path);
	if(res == FR_OK) {
		puts("The following directories and files found in /\r");
		do
		{
			f_readdir(&dir, &fno);
			if(fno.fname[0] != 0)
				// Print directories and files in root
				printf("\t%s\r\n", fno.fname);
		} while(fno.fname[0] != 0);
		f_closedir(&dir);
	} else {
		puts("Error opening dir\r");
		Error_Handler();
	}
	puts("Open or Create file stm32mp1.txt\r");
	res = f_open(&file, path_to("stm32mp1.txt"), FA_READ);
	if(res == FR_OK) {
		puts("stm32mp1.txt exists\r");
		puts("Reading file...\r");
		res = f_read(&file, readText, sizeof(readText), (void *)&bytesRead);
		if(res == FR_OK) {
			f_close(&file);
			printf("stm32mp1.txt: \r\n%.*s\r\n", bytesRead, readText);
		} else {
			puts("Failed to read file!\r");
			Error_Handler();
		}
	} else {
		puts("stm32mp1.txt does not exist\r");
		puts("Creating file...\r");
		res = f_open(&file, path_to("stm32mp1.txt"), FA_CREATE_ALWAYS | FA_WRITE);
		if(res == FR_OK) {
			puts("Created file stm32mp1.txt\r");
			res = f_write(&file, writeText, sizeof(writeText), (void *)&bytesWritten);
			if((res == FR_OK) && (bytesWritten > 0)) {
				f_close(&file);
				printf("%u bytes written to stm32mp1.txt\r\n", bytesWritten);
			} else {
				printf("Failed to write to stm32mp1.txt! Error 0x%08x, wrote %d bytes\r\n", res, bytesWritten);
			}
		} else {
			puts("Failed to create new file!\r");
			Error_Handler();
		}
	}

puts("Entering while loop\r");
	while (1) {
		for(uint8_t i=0;i<4;i++) {
			HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
			HAL_Delay(200);
		}
		HAL_Delay(1000);
	}
}

char *path_to(char *str) {
	sprintf(path_buf, "%s%s", path, str);
	return path_buf;
}

void CLK_Config() {
	RCC_PLLInitTypeDef pll4 = {0};
	RCC_PeriphCLKInitTypeDef perck = {0};
	pll4.PLLState = RCC_PLL_ON;
	pll4.PLLMODE = RCC_PLL_INTEGER;
	pll4.PLLSource = RCC_PLL4SOURCE_HSE;
	pll4.PLLM = 4;
	pll4.PLLN = 99;
	pll4.PLLP = 6;
	pll4.PLLQ = 20;
	pll4.PLLR = 8;

	if(RCCEx_PLL4_Config(&pll4) != HAL_OK) {
		puts("Clock Error!\r");
		Error_Handler();
	}

	// I2C to HSI
	perck.PeriphClockSelection |= RCC_PERIPHCLK_I2C12;
	perck.I2c12ClockSelection = RCC_I2C12CLKSOURCE_HSI;
	// UART4
	perck.PeriphClockSelection |= RCC_PERIPHCLK_UART24;
	perck.Uart24ClockSelection = RCC_UART24CLKSOURCE_HSI;
	// FDCAN
	perck.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
	perck.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;

	// SDMMC12
	__HAL_RCC_SDMMC12_CONFIG(RCC_SDMMC12CLKSOURCE_PLL4);
	// ADC
	__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_PLL4);
	HAL_RCCEx_PeriphCLKConfig(&perck);
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
	puts("\r\n");
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
