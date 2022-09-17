#include "main.h"
#include "stm32mp1xx_hal.h"
#include "lcd.h"
#include "ft6x06.h"
#include "../lvgl/lvgl.h"
#include "../lvgl/examples/lv_examples.h"
#include "../lvgl/demos/lv_demos.h"

#define H_RES 800
#define V_RES 480

void UART4_Init(void);
void GPIO_Init(void);
void TIM2_Init(void);
void I2C1_Init(void);
void MDMA_Config(int32_t buf_offset);
void MDMA_TransferComplete(MDMA_HandleTypeDef *han);
void MDMA_Callback(void);
void my_flush_cb(lv_disp_drv_t * disp_drv_f, const lv_area_t * area, lv_color_t * color_p);
void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t * data);
void tim2_elapsed(void);
void set_interrupt(uint32_t irq_num, uint32_t cpu, uint32_t priority, uint32_t edge_detect, void* callback);
void Error_Handler(void);
void soft_breakpoint(void);

UART_HandleTypeDef huart4;
TIM_HandleTypeDef htim2;
I2C_HandleTypeDef hi2c1;
MDMA_HandleTypeDef hmdma;
void (*irq_callback[249])();

uint8_t run_lv_task_handler = 0;
static lv_disp_drv_t disp_drv;

__attribute__((section (".RAM2"))) static lv_color_t fb_1[H_RES * V_RES];
__attribute__((section (".RAM2"))) static lv_color_t buf_1[H_RES * V_RES / 10];
__attribute__((section (".RAM2"))) static lv_color_t buf_2[H_RES * V_RES / 10];

void main() {
	HAL_Init();
	HAL_Delay(1);
	GPIO_Init();
	UART4_Init();
	I2C1_Init();
	TIM2_Init();
	SecondaryCore_start();

	while (1) {

	}
}

void aux_core_main(void) {
	LCD_Init(OTM8009A_ORIENTATION_LANDSCAPE, (uint32_t)fb_1);
	ft6x06_Init(0x70);
	lv_init();
	static lv_disp_draw_buf_t disp_buf;
	lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2, H_RES * V_RES / 10);
	lv_disp_drv_init(&disp_drv);		/*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;	/*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;	/*Set a flush callback to draw to the display*/
	disp_drv.hor_res = H_RES;		/*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = V_RES;		/*Set the vertical resolution in pixels*/
	static lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);		/*Basic initialization*/
	indev_drv.type = LV_INDEV_TYPE_POINTER;	/*See below.*/
	indev_drv.read_cb = my_input_read;		/*See below.*/
	/*Register the driver in LVGL and save the created input device object*/
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);

	HAL_TIM_Base_Start_IT(&htim2);
	lv_demo_widgets();
	while(1) {
		if(run_lv_task_handler) {
			run_lv_task_handler = 0;
			lv_task_handler();
		}
	}
}

void my_flush_cb(lv_disp_drv_t * disp_drv_f, const lv_area_t * area, lv_color_t * color_p) {
	/*Return if the area is out the screen*/
	if (area->x2 < 0) return;
	if (area->y2 < 0) return;
	if (area->x1 > H_RES - 1) return;
	if (area->y1 > V_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > H_RES - 1 ? H_RES - 1 : area->x2;
	int32_t act_y2 = area->y2 > V_RES - 1 ? V_RES - 1 : area->y2;

	uint32_t x_size = ((uint32_t)act_x2 - (uint32_t)act_x1 + 1) * 4;
	uint32_t y_size = (uint32_t)act_y2 - (uint32_t)act_y1 + 1;
	uint32_t src_addr = (uint32_t)color_p;
	uint32_t dest_addr = (uint32_t)fb_1+(((act_y1 * H_RES)+act_x1)*4);
	MDMA_Config(x_size);
	if(HAL_MDMA_Start_IT(&hmdma,src_addr, dest_addr, x_size, y_size) != HAL_OK) {
		Error_Handler();
	}
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t * data) {
	if(EXTI->FPR1 & TP_INT_PIN) {
		if(ft6x06_TS_DetectTouch(0x70)) {
			if(H_RES > V_RES) {
				// x and y touch points swapped when in landscape
				ft6x06_TS_GetXY(0x70, &data->point.y, &data->point.x);
				data->point.y = 480 - data->point.y;
			} else {
				ft6x06_TS_GetXY(0x70, &data->point.x, &data->point.y);
			}
			data->state = LV_INDEV_STATE_PRESSED;
		}
		EXTI->FPR1 = TP_INT_PIN;
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
}

void TIM2_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 20814;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 50;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	set_interrupt(TIM2_IRQn, 1, 0, 0, tim2_elapsed);
}

void I2C1_Init() {
	__HAL_RCC_I2C1_CLK_ENABLE();
	hi2c1.Instance = I2C1;
	HAL_I2C_DeInit(&hi2c1);
	hi2c1.Init.Timing = 0x20706A61;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 1) != HAL_OK) {
		Error_Handler();
	}
}

void MDMA_Config(int32_t buf_x_size) {
	__HAL_RCC_MDMA_CLK_ENABLE();

	hmdma.Instance = MDMA_Channel0;
	HAL_MDMA_DeInit(&hmdma);

	hmdma.Init.Request 			= MDMA_REQUEST_SW;
	hmdma.Init.TransferTriggerMode 	= MDMA_REPEAT_BLOCK_TRANSFER;
	hmdma.Init.Priority 			= MDMA_PRIORITY_HIGH;
	hmdma.Init.Endianness 			= MDMA_LITTLE_ENDIANNESS_PRESERVE;
	hmdma.Init.DataAlignment 		= MDMA_DATAALIGN_PACKENABLE;
	hmdma.Init.SourceBurst 		= MDMA_SOURCE_BURST_SINGLE;
	hmdma.Init.DestBurst 			= MDMA_DEST_BURST_SINGLE;
	hmdma.Init.BufferTransferLength 	= 128;
	hmdma.Init.SourceBlockAddressOffset  	= 0;
	hmdma.Init.DestBlockAddressOffset 	= (H_RES * 4) - buf_x_size;
	hmdma.Init.SourceDataSize 		= MDMA_SRC_DATASIZE_WORD;
	hmdma.Init.DestDataSize 		= MDMA_DEST_DATASIZE_WORD;
	hmdma.Init.SourceInc 			= MDMA_SRC_INC_WORD;
	hmdma.Init.DestinationInc 		= MDMA_DEST_INC_WORD;

	if (HAL_MDMA_Init(&hmdma) != HAL_OK) {
	Error_Handler();
	}
	HAL_MDMA_RegisterCallback(&hmdma, HAL_MDMA_XFER_CPLT_CB_ID, MDMA_TransferComplete);
	set_interrupt(MDMA_IRQn, 1, 0, 0, MDMA_Callback);
}

void MDMA_TransferComplete(MDMA_HandleTypeDef *han) {
	lv_disp_flush_ready(&disp_drv);
}

void MDMA_Callback() {
	HAL_MDMA_IRQHandler(&hmdma);
}

void tim2_elapsed() {
	run_lv_task_handler = 1;
	lv_tick_inc(5);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
}

void TS_IO_Init(void) {

}

void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value) {
	uint8_t data[] = {Reg, Value};
	HAL_I2C_Master_Transmit(&hi2c1, Addr, data, 2, 100);
}

uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg) {
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, &data, 1, 100);
	return data;
}

uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length) {
	HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, Buffer, Length, 100);
	return 0;
}

void TS_IO_Delay(uint32_t Delay) {
	HAL_Delay(Delay);
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
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/* Red and Blue LEDs */
	GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* I2C1 SCL */
	GPIO_InitStruct.Pin = I2C1_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_I2C1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* I2C1 SDA */
	GPIO_InitStruct.Pin = I2C1_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_I2C1;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/* TouchPanel Interrupt */
	GPIO_InitStruct.Pin = TP_INT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TP_INT_PORT, &GPIO_InitStruct);
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
