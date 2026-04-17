/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdlib.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "lora.h"
#include "telemetry.h"
#include "error_sdr.h"
#include "usb.h"
#include "usb_cdc_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* USB data buffer */
uint8_t usb_tx_byte[ USB_BUF_SIZE ];
uint8_t usb_rx_byte[ USB_BUF_SIZE ];

/* LoRa global receive buffer */
LORA_STATUS lora_status;
LORA_MESSAGE last_lora_message;
bool start_lora = false;
uint32_t sequence_number = 0;

/* LoRa config settings */
LORA_PRESET lora_preset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum
{
    USB_RX_MODE_IDLE = 0,
    USB_RX_MODE_BLOCKING,
    USB_RX_MODE_INTERRUPT
} usb_rx_mode_t;

static volatile usb_rx_mode_t s_rx_mode = USB_RX_MODE_IDLE;
static uint8_t *s_rx_buf = NULL;
static volatile size_t s_rx_have = 0U;
static size_t s_rx_goal = 0U;
static uint8_t s_rx_stash[CDC_DATA_FS_MAX_PACKET_SIZE];
static volatile size_t s_rx_stash_len = 0U;

static void *s_tx_it_data = NULL;
static size_t s_tx_it_len = 0U;

__attribute__( ( weak ) ) void usb_receive_complete_callback( void *data, size_t len )
{
    (void)data;
    (void)len;

    terminal_loop();
}

__attribute__( ( weak ) ) void usb_transmit_complete_callback( void *data, size_t len )
{
    (void)data;
    (void)len;

    usb_receive_IT( usb_rx_byte, 1 );
}

static void usb_rx_drain_stash_locked( void )
{
    while ( s_rx_stash_len > 0U && s_rx_mode != USB_RX_MODE_IDLE && s_rx_buf != NULL && s_rx_have < s_rx_goal )
        {
        size_t need = s_rx_goal - s_rx_have;
        size_t take = ( s_rx_stash_len < need ) ? s_rx_stash_len : need;
        memcpy( s_rx_buf + s_rx_have, s_rx_stash, take );
        s_rx_have += take;
        size_t rem = s_rx_stash_len - take;
        if ( rem > 0U )
            {
            memmove( s_rx_stash, s_rx_stash + take, rem );
            }
        s_rx_stash_len = rem;
        }
}

void usb_process_cdc_rx( uint8_t *buf, uint32_t len )
{
    if ( len == 0U )
        {
        return;
        }

    if ( s_rx_mode == USB_RX_MODE_IDLE )
        {
        if ( s_rx_stash_len + len > sizeof( s_rx_stash ) )
            {
            return;
            }
        memcpy( s_rx_stash + s_rx_stash_len, buf, len );
        s_rx_stash_len += len;
        return;
        }

    uint32_t pos = 0U;
    while ( pos < len && s_rx_have < s_rx_goal )
        {
        size_t need = s_rx_goal - s_rx_have;
        uint32_t avail = len - pos;
        size_t chunk = ( avail < need ) ? (size_t)avail : need;
        memcpy( s_rx_buf + s_rx_have, buf + pos, chunk );
        s_rx_have += chunk;
        pos += chunk;
        }

    if ( pos < len )
        {
        uint32_t left = len - pos;
        if ( s_rx_stash_len + left > sizeof( s_rx_stash ) )
            {
            return;
            }
        memcpy( s_rx_stash + s_rx_stash_len, buf + pos, left );
        s_rx_stash_len += left;
        }

    if ( s_rx_have >= s_rx_goal && s_rx_mode == USB_RX_MODE_INTERRUPT )
        {
        s_rx_mode = USB_RX_MODE_IDLE;
        void *cbdata = s_rx_buf;
        size_t cblen = s_rx_goal;
        s_rx_buf = NULL;
        usb_receive_complete_callback( cbdata, cblen );
        }
}

void USBD_App_CDC_TxComplete( void )
{
    usb_transmit_complete_callback( s_tx_it_data, s_tx_it_len );
}

USB_STATUS usb_receive_IT( void *data, size_t len )
{
    if ( data == NULL || len == 0U )
        {
        return USB_FAIL;
        }

    __disable_irq();
    if ( s_rx_mode != USB_RX_MODE_IDLE )
        {
        __enable_irq();
        return USB_FAIL;
        }

    s_rx_buf = (uint8_t *)data;
    s_rx_goal = len;
    s_rx_have = 0U;
    s_rx_mode = USB_RX_MODE_INTERRUPT;
    usb_rx_drain_stash_locked();

    if ( s_rx_have >= s_rx_goal )
        {
        s_rx_mode = USB_RX_MODE_IDLE;
        void *p = s_rx_buf;
        size_t n = s_rx_goal;
        s_rx_buf = NULL;
        __enable_irq();
        usb_receive_complete_callback( p, n );
        return USB_OK;
        }

    __enable_irq();
    return USB_OK;
}

USB_STATUS usb_receive( void *data, size_t len, uint32_t timeout )
{
    if ( data == NULL || len == 0U )
        {
        return USB_FAIL;
        }

    __disable_irq();
    if ( s_rx_mode != USB_RX_MODE_IDLE )
        {
        __enable_irq();
        return USB_FAIL;
        }

    s_rx_buf = (uint8_t *)data;
    s_rx_goal = len;
    s_rx_have = 0U;
    s_rx_mode = USB_RX_MODE_BLOCKING;
    usb_rx_drain_stash_locked();
    __enable_irq();

    if ( s_rx_have >= s_rx_goal )
        {
        __disable_irq();
        s_rx_mode = USB_RX_MODE_IDLE;
        s_rx_buf = NULL;
        __enable_irq();
        return USB_OK;
        }

    uint32_t tickstart = HAL_GetTick();
    while ( s_rx_have < s_rx_goal )
        {
        if ( timeout != HAL_MAX_DELAY && ( HAL_GetTick() - tickstart ) >= timeout )
            {
            __disable_irq();
            s_rx_mode = USB_RX_MODE_IDLE;
            s_rx_buf = NULL;
            __enable_irq();
            return USB_TIMEOUT;
            }
        }

    __disable_irq();
    s_rx_mode = USB_RX_MODE_IDLE;
    s_rx_buf = NULL;
    __enable_irq();
    return USB_OK;
}

USB_STATUS usb_transmit( void *data, size_t len, uint32_t timeout )
{
    uint32_t timeout_start = HAL_GetTick();
    while ( CDC_Transmit_FS( (uint8_t *)data, (uint16_t)len ) == USBD_BUSY
            && ( HAL_GetTick() - timeout_start ) < timeout )
        ;
    return USB_OK;
}

USB_STATUS usb_transmit_IT( void *data, size_t len )
{
    uint8_t st = CDC_Transmit_FS( (uint8_t *)data, (uint16_t)len );
    if ( st == USBD_BUSY )
        {
        return USB_FAIL;
        }
    if ( st != USBD_OK )
        {
        return USB_FAIL;
        }
    s_tx_it_data = data;
    s_tx_it_len = len;
    return USB_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // NOTE: INITIALIZE DRIVERS SETUPS HERE
  LORA_PRESET preset = {
    LORA_SPREAD_12, /* SF 6 - 12 supported. Validate the range. */
    LORA_BANDWIDTH_125_KHZ, /* enum -- spec defined in LORA_BANDWIDTH. Packed to one byte. */
    5, /* error coding options are 4:5, 4:6, 4:7, and 4:8 */
    0, /* true: +20 dBm boost */
    915000 /* frequency in kHz */
    /* omitted: chipmode, header mode (defined by fw) */
    };
  LORA_STATUS lora_init_status = lora_configure(&preset); /* use a nullptr so we use dflt cfgs */

if( lora_init_status == LORA_USING_DEFAULTS )
    {
    /* give an indicator of default configs*/
    for( int i = 0; i < 4; i++ )
        {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
        HAL_Delay(200);
        }
    }
else if( lora_init_status != LORA_OK )
    {
    error_fail_fast( ERROR_LORA_INIT_ERROR );
    }

/* Initialize LoRa buffer */
memset(&last_lora_message, 0, LORA_MESSAGE_SIZE);

/* start terminal loop */
usb_receive_IT( usb_rx_byte, 1 );

/* Terminal Mode */
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

static const LORA_MESSAGE dashboard_dump_msg =
    {
    .header =
        {
        .uid =
            {
            .wafer_coords = 0x00000000UL,
            .lot_num_1    = { '\0', '\0', '\0' },
            .wafer_num    = 0x00,
            .lot_num_2    = { '\0', '\0', '\0', '\0' }
            },
        .mid       = LORA_MSG_DASHBOARD_DATA,
        .timestamp = 0x00000000UL
        },
    .payload.dashboard_dump =
        {
        .fsm_state = 0x00,
        .data =
            {
            .acc_x             = -0.5f,
            .acc_y             = 0.5f,
            .acc_z             = 9.6f,
            .gyro_x            = 0.0f,
            .gyro_y            = 0.0f,
            .gyro_z            = 0.0f,
            .roll_angle        = 0.0f,
            .pitch_angle       = 0.0f,
            .yaw_angle         = 0.0f,
            .roll_rate         = 0.0f,
            .pitch_rate        = 0.0f,
            .yaw_rate          = 0.0f,
            .baro_pressure     = 98000.0f,
            .baro_temp         = 0.0f,
            .baro_alt          = 1000.0f,
            .baro_velo         = 0.0f,
            .gps_dec_longitude = 80.0f,
            .gps_dec_latitude  = -20.0f
            },
        .explicit_padding = { 0x00, 0x00, 0x00 }
        }
    };

static const LORA_MESSAGE vehicle_id_msg =
    {
    .header =
        {
        .uid =
            {
            .wafer_coords = 0x00000000UL,
            .lot_num_1    = { '\0', '\0', '\0' },
            .wafer_num    = 0x00,
            .lot_num_2    = { '\0', '\0', '\0', '\0' }
            },
        .mid       = LORA_MSG_VEHICLE_ID,
        .timestamp = 0x00000000UL
        },
    .payload.vehicle_id =
        {
        .hw_opcode        = 0x05,
        .fw_opcode        = 0x06,
        .version          = 0x0206000AUL, /* hw : fw : patch : prerelease (MSB→LSB) */
        .flight_id        = "FLIGHT-010\0\0\0\0\0",  /* 16 bytes, null padded */
        .explicit_padding = { 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00 }  /* 54 bytes */
        }
    };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*------------------------------------------------------------------------------
  Event Loop                                                                  
  ------------------------------------------------------------------------------*/
  lora_status = lora_set_chip_mode( LORA_RX_CONTINUOUS_MODE );
  while (1)
    {
      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */
      if( lora_receive_ready() == LORA_READY )
        {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
        uint8_t rx_buf[LORA_MESSAGE_SIZE];
        uint8_t rx_size = 0;
	    lora_status = lora_receive(rx_buf, LORA_MESSAGE_SIZE, &rx_size);

        if( lora_status == LORA_OK && rx_size == LORA_MESSAGE_SIZE )
            {
            memcpy( &last_lora_message, rx_buf, LORA_MESSAGE_SIZE );
            }
        sequence_number++;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_RST_Pin|LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  error_fail_fast( ERROR_UNKNOWN_FATAL_ERROR );
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
