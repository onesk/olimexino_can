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
#include "can.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterConfTypeDef filter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void led_usb_init(void);
static void can_filter_init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define SLCAN_MTU 64

#define SLCAN_STD_ID_LEN 3
#define SLCAN_EXT_ID_LEN 8

volatile int led_state = 0;
void flip_led()
{
	led_state ^= 0x1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int8_t slcan_parse_frame(uint8_t *buf, CanRxMsgTypeDef *frame) {
    uint8_t i = 0;
    uint8_t id_len, j;
    uint32_t tmp;

    for (j=0; j < SLCAN_MTU; j++) {
        buf[j] = '\0';
    }

    // add character for frame type
    if (frame->RTR == CAN_RTR_DATA) {
        buf[i] = 't';
    } else if (frame->RTR == CAN_RTR_REMOTE) {
        buf[i] = 'r';
    }

    // assume standard identifier
    id_len = SLCAN_STD_ID_LEN;
    tmp = frame->StdId;
    // check if extended
    if (frame->IDE == CAN_ID_EXT) {
        // convert first char to upper case for extended frame
        buf[i] -= 32;
        id_len = SLCAN_EXT_ID_LEN;
        tmp = frame->ExtId;
    }
    i++;

    // add identifier to buffer
    for(j=id_len; j > 0; j--) {
        // add nibble to buffer
        buf[j] = (tmp & 0xF);
        tmp = tmp >> 4;
        i++;
    }

    // add DLC to buffer
    buf[i++] = frame->DLC;

    // add data bytes
    int dlc = frame->DLC;
    if (dlc > 8) dlc = 8;
    for (j = 0; j < dlc; j++) {
        buf[i++] = (frame->Data[j] >> 4);
        buf[i++] = (frame->Data[j] & 0x0F);
    }

    // convert to ASCII (2nd character to end)
    for (j = 1; j < i; j++) {
        if (buf[j] < 0xA) {
            buf[j] += 0x30;
        } else {
            buf[j] += 0x37;
        }
    }

    // add carrage return (slcan EOL)
    buf[i++] = '\r';

    // return number of bytes in string
    return i;
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
  MX_USB_DEVICE_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
  led_usb_init();
  can_filter_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* olimexino stm32 has poorly documented DISC pin which needs to be pulled to ground in order to enable USB. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /* signal start of operation by blinking */
  for (uint8_t i = 0; i < 32; ++i)
  {
	  flip_led();
	  HAL_Delay(100);
  }

  uint32_t id = 0;
  CanTxMsgTypeDef tx_msg;

  // loop forever
  CanRxMsgTypeDef rx_msg;
  uint32_t length;
  uint8_t msg_buf[SLCAN_MTU];


  tx_msg.IDE = CAN_ID_EXT;
  tx_msg.StdId = 13;
  tx_msg.ExtId = 13;
  tx_msg.DLC = 2;
  tx_msg.Data[0] = id & 0xff;
  tx_msg.Data[1] = (id >> 8) & 0xff;
  hcan.pTxMsg = &tx_msg;
  HAL_CAN_Transmit(&hcan, 10);
  id++;

  while (1)
  {
     /* busy wait */
     while (__HAL_CAN_MSG_PENDING(&hcan, CAN_FIFO0) <= 0) ;
     flip_led();

     hcan.pRxMsg = &rx_msg;

     if (HAL_CAN_Receive(&hcan, CAN_FIFO0, 3) == HAL_OK)
     {
        length = slcan_parse_frame(msg_buf, &rx_msg);
        CDC_Transmit_FS(msg_buf, length);
     }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
     HAL_Delay(500);
  tx_msg.IDE = CAN_ID_EXT;
  tx_msg.StdId = 13;
  tx_msg.ExtId = 13;
  tx_msg.DLC = 2;
  tx_msg.Data[0] = id & 0xff;
  tx_msg.Data[1] = (id >> 8) & 0xff;
  hcan.pTxMsg = &tx_msg;
  HAL_CAN_Transmit(&hcan, 10);
  id++;

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

static void can_filter_init(void) {
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

    hcan.pTxMsg = NULL;
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
