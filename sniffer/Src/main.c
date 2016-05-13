/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
CAN_FilterConfTypeDef filter;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void led_usb_init(void);

static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile int led_state = 0;
void flip_led()
{
	led_state ^= 0x1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  led_usb_init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* olimexino stm32 has poorly documented DISC pin which needs to be pulled to ground in order to enable USB. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  for (uint8_t i = 0; i < 10; ++i)
  {
	  flip_led();
	  HAL_Delay(100);
  }

  MX_CAN_Init();
  
  //  uint8_t msg[] = "hui pizda djigurda\n";

  uint32_t id = 0;
  CanTxMsgTypeDef tx_msg;
  CanRxMsgTypeDef rx_msg;

  uint8_t skip_msg[5] = "skip\n";
  
  uint8_t data[2];
  data[1] = '\n';
  
  while (1)
  {
	  flip_led();

	  tx_msg.RTR = CAN_RTR_DATA;
	  tx_msg.IDE = CAN_ID_STD;
	  tx_msg.StdId = tx_msg.ExtId = 13;
	  tx_msg.DLC = 1;
	  tx_msg.Data[0] = (id++) & 0xff;
	  hcan.pTxMsg = &tx_msg;
	  HAL_CAN_Transmit(&hcan, 10);

	  uint8_t any = 1;
	  uint32_t startTick = HAL_GetTick();
	  while (__HAL_CAN_MSG_PENDING(&hcan, CAN_FIFO0) <= 0)
	  {
		 if (HAL_GetTick() - startTick > 200)
		 {
			any = 0;
			break;
		 }
	  }

	  if (!any)
	  {
		 CDC_Transmit_FS(skip_msg, 5);
		 continue;
	  }

	  

	 hcan.pRxMsg = &rx_msg;
	 data[0] = (uint8_t) HAL_CAN_Receive(&hcan, CAN_FIFO0, 3);
	 CDC_Transmit_FS(data, 2);
	 
	  //  CDC_Transmit_FS(msg, 19);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
static void led_usb_init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* CAN init function */
void MX_CAN_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_InitStruct.Pin = GPIO_Pin_8;
   GPIO_InitStruct.Mode = GPIO_MODE_IPU;//!
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_Pin_9;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//!
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
   GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
  
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

   hcan.Instance = CAN1;
   hcan.Init.Prescaler = 6;
   hcan.Init.Mode = CAN_MODE_LOOPBACK;
   hcan.Init.SJW = CAN_SJW_1TQ;
   hcan.Init.BS1 = CAN_BS1_6TQ;
   hcan.Init.BS2 = CAN_BS2_5TQ;
   hcan.Init.TTCM = DISABLE;
   hcan.Init.ABOM = DISABLE;
   hcan.Init.AWUM = DISABLE;
   hcan.Init.NART = DISABLE;
   hcan.Init.RFLM = DISABLE;
   hcan.Init.TXFP = DISABLE;
   hcan.pTxMsg = NULL;
   HAL_CAN_Init(&hcan);

   filter.FilterIdHigh = 0;
   filter.FilterIdLow = 0;
   filter.FilterMaskIdHigh = 0;
   filter.FilterMaskIdLow = 0;
   filter.FilterMode = CAN_FILTERMODE_IDMASK;
   filter.FilterScale = CAN_FILTERSCALE_32BIT;
   filter.FilterNumber = 0;
   filter.FilterFIFOAssignment = CAN_FIFO0;
   filter.BankNumber = 0;
   filter.FilterActivation = ENABLE;
   HAL_CAN_ConfigFilter(&hcan, &filter);
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
