/*
 * stm32mp1xx_hal_msp.c
 *
 *  Created on: Aug 25, 2022
 *      Author: cody
 */

#include "stm32mp1xx_hal.h"
#include "main.h"

#ifdef HAL_SD_MODULE_ENABLED
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hsd->Instance==SDMMC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PC8      ------> SDMMC1_D0
    PC9      ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    PD2      ------> SDMMC1_CMD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

/**
* @brief SD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  if(hsd->Instance==SDMMC1)
  {
  /* USER CODE BEGIN SDMMC3_MspDeInit 0 */

	SDMMC1->ACKTIME = 0U;
	SDMMC1->ARG = 0U;
	SDMMC1->CLKCR = 0U;
	SDMMC1->CMD = 0U;
	SDMMC1->DCTRL = 0U;
	SDMMC1->DLEN = 0U;
	SDMMC1->DTIMER = 0U;
	SDMMC1->FIFO = 0U;
	SDMMC1->ICR = 0xFFFFFFFF;
	SDMMC1->IDMABAR = 0U;
	SDMMC1->IDMABASE0 = 0U;
	SDMMC1->IDMABASE1 = 0U;
	SDMMC1->IDMABSIZE = 0U;
	SDMMC1->IDMACTRL = 0U;
	SDMMC1->IDMALAR = 0U;
	SDMMC1->MASK = 0U;
	SDMMC1->POWER = 0U;  /* USER CODE END SDMMC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /**SDMMC1 GPIO Configuration
    PC8      ------> SDMMC1_D0
    PC9      ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    PD2      ------> SDMMC1_CMD
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
  }
}
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clock
	*/
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	  Error_Handler();
	}

	/* Peripheral clock enable */
	__HAL_RCC_FDCAN_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**FDCAN1 GPIO Configuration
	PA11     ------> FDCAN1_RX
	PA12     ------> FDCAN1_TX
	*/
	GPIO_InitStruct.Pin = FDCAN1_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = FDCAN1_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
#endif

#ifdef HAL_ADC_MODULE_ENABLED
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if(hadc->Instance==ADC1) {
		/* Peripheral clock enable */
		__HAL_RCC_ADC12_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		ANA0     ------> ADC1_INP0
		*/
		HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);
	}
}
#endif

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(htim_base->Instance==TIM2) {
		__HAL_RCC_TIM2_CLK_ENABLE();
//		__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_0; //VSS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF;
		GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	} else if(htim_base->Instance==TIM5) {
		__HAL_RCC_TIM5_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_4; //VSS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	} else if(htim_base->Instance==TIM8) {
		__HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_0; //VSS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
#endif

#ifdef HAL_LTDC_MODULE_ENABLED
void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Set pixel clock to 33 MHz
	__HAL_RCC_DSI_CLK_DISABLE();
	__HAL_RCC_LTDC_CLK_DISABLE();
	RCC->PLL4CR &= ~RCC_PLL4CR_DIVQEN;
	RCC->PLL4CFGR2 &= ~RCC_PLL4CFGR2_DIVQ;
	RCC->PLL4CFGR2 |= (17U << RCC_PLL4CFGR2_DIVQ_Pos);	// DIVQ4 /18
	RCC->PLL4CR |= RCC_PLL4CR_DIVQEN;

	// Enable LTDC clock
	__HAL_RCC_LTDC_CLK_ENABLE();

	// Enable LTDC GPIO clocks
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();

	// Configure GPIO Pins
	GPIO_InitStruct.Pin = LTDC_VS_PIN | LTDC_HS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_B0_PIN | LTDC_B3_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_G1_PIN | LTDC_DE_PIN | LTDC_R7_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_G0_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF13_LCD;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_CLK_PIN | LTDC_B2_PIN | LTDC_B1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_R0_PIN | LTDC_R1_PIN | LTDC_R2_PIN | LTDC_R3_PIN | LTDC_R4_PIN | LTDC_R5_PIN \
			| LTDC_R6_PIN | LTDC_G2_PIN | LTDC_G3_PIN | LTDC_G4_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LTDC_G5_PIN | LTDC_G6_PIN | LTDC_G7_PIN | LTDC_B4_PIN | LTDC_B5_PIN | LTDC_B6_PIN \
			| LTDC_B7_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF14_LCD;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOI, LTDC_HS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOI, LTDC_VS_PIN, GPIO_PIN_RESET);
}
#endif
