/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "sd_functions.h"
#include "driver_ina219.h"
#include "app_wrapper.h"
#include "config.h"
#include "vl53l1_api.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define NODE_SENSOR        2
#define NODE_ACTUATOR      1
#define NODE_ID            NODE_SENSOR
#define CAN_ID_HEARTBEAT   0x200
#define FW_MAJOR           0
#define FW_MINOR           1

/* ---- Tang goi: mot dinh nghia dung chung cho CAN va UART ----
 * CAN  : StdId = NODE_BASE + pkt_id
 * UART : $<node>,<pkt>,<uptime>,<payload hex>*<xor checksum>
 */
#define NODE_BASE_ACTUATOR 0x100
#define NODE_BASE_SENSOR   0x200
#define NODE_BASE          NODE_BASE_SENSOR

#define PKT_HEARTBEAT      0x00
#define PKT_ENV            0x01

#define LM35_VDDA_MV       3300UL
#define LM35_OVERSAMPLE    16U
#define LM35_DIVISOR       (4096UL * LM35_OVERSAMPLE)

static CAN_TxHeaderTypeDef canTxHeader;
static uint8_t  canTxData[8];
static uint32_t canTxMailbox;

static ina219_handle_t ina219_handle;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_NodeInit(void);
void CAN_SendHeartbeat(uint32_t uptime_ms);
void pkt_send(uint8_t pkt_id, const uint8_t *data, uint8_t len);
void Sensor_SendEnv(void);
void SPI_Loopback(void);
void SPI_CsBlink(uint32_t ms);

/* --- CÁC HÀM CẦU NỐI CHO INA219 --- */
static uint8_t ina219_iic_init(void) { return 0; }
static uint8_t ina219_iic_deinit(void) { return 0; }
static uint8_t ina219_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	// hi2c1 đã được khởi tạo trong MX_I2C1_Init()
	return (HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 0 : 1;
}
static uint8_t ina219_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	return (HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 0 : 1;
}
static void ina219_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}
static void ina219_debug_print(const char *const fmt, ...)
{
	// Để trống nếu không dùng debug log nội bộ của driver
}

static void scan_bus(const char *title)
{
	printf("%-35s -> Dev: ", title);
	uint8_t count = 0;
	for (uint8_t a = 0x08; a <= 0x77; a++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 5) == HAL_OK) {
			printf("0x%02X ", a);
			count++;
		}
	}
	if (count == 0) printf("None");
	printf("\r\n");
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_CAN_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */


	//  Test
	//	SPI_Loopback();
	//  SD CARD
	//	sd_mount();
	//	sd_list_files();



	//
	//	sd_unmount();

	setvbuf(stdout, NULL, _IONBF, 0);

	printf("\r\n================ BAT DAU CHUOI TEST ================\r\n");
	/* =================================================================
	 * GIAI ĐOẠN 1: TEST SPI TRONG 5 GIÂY (Mỗi giây 1 mẫu)
	 * ================================================================= */
	//	printf("\n--- 1. TEST SPI (5s) ---\r\n");
	//	for (int i = 1; i <= 5; i++)
	//	{
	//		uint8_t tx = 0xA0 + i, rx = 0x00;
	//		HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 50);
	//		printf("[SPI %ds] TX: 0x%02X | RX: 0x%02X (%s)\r\n",
	//				i, tx, rx, (rx == tx) ? "OK" : "NO LOOP");
	//		HAL_Delay(1000);
	//	}
	//	HAL_Delay(2000); // Nghỉ 2 giây trước khi qua phần 2
	//	/* =================================================================
	//	 * GIAI ĐOẠN 2: TEST I2C BUS SCAN TRONG 5 GIÂY (Mỗi giây quét 1 lần)
	//	 * ================================================================= */
	//	printf("\n--- 2. TEST I2C BUS (5s) ---\r\n");
	//	for (int i = 1; i <= 5; i++)
	//	{
	//		uint8_t count = 0;
	//		printf("[I2C %ds] Dev: ", i);
	//		for (uint8_t addr = 0x08; addr <= 0x77; addr++)
	//		{
	//			if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 5) == HAL_OK)
	//			{
	//				printf("0x%02X ", addr);
	//				count++;
	//			}
	//		}
	//		if (count == 0) printf("None");
	//		printf("\r\n");
	//		HAL_Delay(1000);
	//	}
	//	HAL_Delay(2000); // Nghỉ 2 giây trước khi qua phần 3
	//	/* =================================================================
	//	 * GIAI ĐOẠN 3: TEST CẢM BIẾN DÒNG/ÁP INA219 TRONG 5 GIÂY
	//	 * ================================================================= */
	//	printf("\n--- 3. TEST INA219 (5s) ---\r\n");
	//	DRIVER_INA219_LINK_INIT(&ina219_handle, ina219_handle_t);
	//	DRIVER_INA219_LINK_IIC_INIT(&ina219_handle, ina219_iic_init);
	//	DRIVER_INA219_LINK_IIC_DEINIT(&ina219_handle, ina219_iic_deinit);
	//	DRIVER_INA219_LINK_IIC_READ(&ina219_handle, ina219_iic_read);
	//	DRIVER_INA219_LINK_IIC_WRITE(&ina219_handle, ina219_iic_write);
	//	DRIVER_INA219_LINK_DELAY_MS(&ina219_handle, ina219_delay_ms);
	//	DRIVER_INA219_LINK_DEBUG_PRINT(&ina219_handle, ina219_debug_print);
	//	ina219_set_addr_pin(&ina219_handle, INA219_ADDRESS_0);
	//	if (ina219_init(&ina219_handle) == 0)
	//	{
	//		for (int i = 1; i <= 5; i++)
	//		{
	//			uint16_t v_raw = 0;
	//			int16_t s_raw = 0;
	//			float v_bus_mV = 0.0f, v_shunt_mV = 0.0f;
	//			ina219_read_bus_voltage(&ina219_handle, &v_raw, &v_bus_mV);
	//			ina219_read_shunt_voltage(&ina219_handle, &s_raw, &v_shunt_mV);
	//			int v_mv = (int)v_bus_mV;
	//			int i_ma = (int)(v_shunt_mV * 10.0f);
	//			printf("[INA219 %ds] V: %d.%02dV | I: %d mA\r\n",
	//					i, v_mv / 1000, (v_mv % 1000) / 10, i_ma);
	//
	//			HAL_Delay(1000);
	//		}
	//	}
	//	else
	//	{
	//		printf("[INA219] Khởi tạo thất bại!\r\n");
	//	}
	//	HAL_Delay(2000); // Nghỉ 2 giây trước khi qua phần 4
	//	/* =================================================================
	//	 * GIAI ĐOẠN 4: TEST NHIỆT ĐỘ LM35 TRONG 10 GIÂY
	//	 * ================================================================= */
	//	printf("\n--- 4. TEST LM35 NHIỆT ĐỘ (10s) ---\r\n");
	//	for (int i = 1; i <= 10; i++)
	//	{
	//		HAL_ADC_Start(&hadc1);
	//		if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK)
	//		{
	//			uint32_t raw = HAL_ADC_GetValue(&hadc1);
	//			uint32_t temp_x10 = (raw * 3300) / 4095; // Nhân 10 để giữ 1 số thập phân
	//			printf("[LM35 %2ds] Raw: %4lu | Temp: %lu.%lu C\r\n", i, (unsigned long)raw, temp_x10 / 10, temp_x10 % 10);
	//
	//		}
	//		else
	//		{
	//			printf("[LM35 %2ds] ADC Timeout\r\n", i);
	//		}
	//		HAL_ADC_Stop(&hadc1);
	//		HAL_Delay(1000);
	//	}
	//	HAL_Delay(2000); // Nghỉ 2 giây trước khi qua phần 5
	//	/* =================================================================
	//	 * GIAI ĐOẠN 5: TEST ĐÈN LED TRONG 5 GIÂY (Đổi màu mỗi 500ms)
	//	 * ================================================================= */
	//	printf("\n--- 5. TEST LED RGB (5s) ---\r\n");
	//	HAL_GPIO_WritePin(GPIOB, LEDR_Pin | LEDG_Pin | LEDB_Pin, GPIO_PIN_RESET);
	//	for (int i = 1; i <= 5; i++)
	//	{
	//		// 1. Đỏ (LEDR - PB12)
	//		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	//		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	//		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	//		printf("[LED %ds] -> ĐỎ\r\n", i);
	//		HAL_Delay(350);
	//		// 2. Xanh lá (LEDG - PB11)
	//		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	//		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	//		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	//		printf("[LED %ds] -> XANH LÁ\r\n", i);
	//		HAL_Delay(350);
	//		// 3. Xanh dương (LEDB - PB10)
	//		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	//		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	//		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	//		printf("[LED %ds] -> XANH DƯƠNG\r\n", i);
	//		HAL_Delay(300);
	//	}
	//	HAL_GPIO_WritePin(GPIOB, LEDR_Pin | LEDG_Pin | LEDB_Pin, GPIO_PIN_RESET);
	//	HAL_Delay(2000); // Nghỉ 2 giây trước khi qua phần 6
	/* =================================================================
	 * GIAI ĐOẠN 6: CHẨN ĐOÁN XSHUT & ĐỊA CHỈ I2C CỦA VL53L1X
	 * ================================================================= */
	printf("\r\n================ CHUẨN ĐOÁN CẤU HÌNH VL53L1X ================\r\n");

	/* BƯỚC 1: Tắt cả 2 cảm biến (XSHUT = LOW) */
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	scan_bus("[1] Tat ca XSHUT = LOW (Mong doi: Khong co VL53L1 nao)");

	/* BƯỚC 2: Bật Cảm biến 0 (PC15) -> Xem địa chỉ khởi tạo gốc */
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	printf("[2] Bat Sensor 0 (PC15): Dia chi khoi tao mac dinh = 0x52 (7-bit: 0x29)\r\n");
	scan_bus("    -> Quet I2C sau khi bat Sensor 0");

	/* BƯỚC 3: Tắt lại để nhường cho hàm SensorManager tự quản lý */
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	/* BƯỚC 4: In cấu hình địa chỉ dự kiến gán mới */
	printf("\r\n--- Cau hinh dia chi moi tu config.h ---\r\n");
	for (uint8_t s = 0; s < L1_COUNT; s++) {
		printf("Sensor %u (Pin: %s) -> Dia chi moi: 0x%02X (7-bit: 0x%02X)\r\n",
				s, (s == 0) ? "PC15" : "PB8", L1_I2C_ADDRS[s], L1_I2C_ADDRS[s] >> 1);
	}

	/* BƯỚC 5: Chạy khởi tạo SensorManager */
	printf("\r\nDang thuc hien khoi tao va doi dia chi I2C...\r\n");
	bool sensors_ok = sensor_system_init();
	if (sensors_ok) {
		printf("SensorManager: KHOI TAO THANH CONG!\r\n");
	} else {
		printf("SensorManager: KHOI TAO THAT BAI!\r\n");
	}

	/* BƯỚC 6: Quét bus I2C để kiểm chứng địa chỉ thực tế sau khi đổi */
	scan_bus("[3] Quet I2C thuc te sau khi doi dia chi (Mong doi: thay 0x30 va 0x31)");
	printf("=============================================================\r\n");

	printf("\r\n================ HOAN TAT BAI TEST ================\r\n");






	//
	//    /* ADC cua F103 phai hieu chuan mot lan sau moi lan cap nguon,
	//     * neu khong so doc bi lech offset. CubeMX khong sinh dong nay. */
	//    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	//      Error_Handler();
	//
	//    CAN_NodeInit();
	////    printf("\r\n=== node %u boot, fw %u.%u ===\r\n", NODE_ID, FW_MAJOR, FW_MINOR);
	//
	//    uint32_t t_hb = 0, t_env = 0;
	//    while (1)
	//    {
	//      uint32_t now = HAL_GetTick();
	//
	//      if (now - t_hb >= 200U)
	//      {
	//        t_hb = now;
	//        HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);
	//
	//        /* ESR cho biet CAN dang khoe hay dang chet */
	//        uint32_t esr = hcan.Instance->ESR;
	////        printf("[%lu] node=%u tec=%u rec=%u lec=%u boff=%u\r\n",
	////               now, NODE_ID,
	////               (unsigned)((esr >> 16) & 0xFF),
	////               (unsigned)((esr >> 24) & 0xFF),
	////               (unsigned)((esr >> 4) & 0x07),
	////               (unsigned)((esr >> 2) & 0x01));
	//
	//        CAN_SendHeartbeat(now);
	//      }
	//
	//      if (now - t_env >= 500U)
	//      {
	//        t_env = now;
	//        Sensor_SendEnv();
	//      }
	//    }
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t loop_sec = 0;
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		loop_sec++;
		printf("\r\n================ [VL53L1 TEST - %4lus] ================\r\n", (unsigned long)loop_sec);
		for (uint8_t i = 0; i < L1_COUNT; i++) {
			printf("Sensor %u: ", i);
			uint16_t dist = sensor_system_get_distance(i);
			if (dist != 0xFFFF) {
				printf("Khoang cach: %4d mm | TOT\r\n", dist);
			} else {
				printf(" LOI (0xFFFF)\r\n");
			}
			HAL_Delay(100);
		}
		/* Hien thi 1 lan tren 1 giay (chu ky 1000ms) */
		HAL_Delay(1000);
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LEDB_Pin|LEDG_Pin|LEDR_Pin|MPU_BOOT_Pin
			|MPU_RST_Pin|XSHUT1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : GPIO1_0_Pin */
	GPIO_InitStruct.Pin = GPIO1_0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO1_0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : XSHUT0_Pin */
	GPIO_InitStruct.Pin = XSHUT0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XSHUT0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_Pin */
	GPIO_InitStruct.Pin = SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN2_Pin BTN1_Pin PB13 */
	GPIO_InitStruct.Pin = BTN2_Pin|BTN1_Pin|GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LEDB_Pin LEDG_Pin LEDR_Pin MPU_BOOT_Pin
                           MPU_RST_Pin XSHUT1_Pin */
	GPIO_InitStruct.Pin = LEDB_Pin|LEDG_Pin|LEDR_Pin|MPU_BOOT_Pin
			|MPU_RST_Pin|XSHUT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO1_1_Pin */
	GPIO_InitStruct.Pin = GPIO1_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO1_1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* Chia doi bai toan: noi tam PA7 (MOSI) vao PA6 (MISO) bang mot day nhay roi
 * goi ham nay. Byte gui ra phai quay ve nguyen ven.
 *   DAT     -> SPI, SCK va hai chan du lieu cua MCU deu tot, loi o phia the
 *   THAT BAI -> loi nam trong MCU: clock khong ra, hoac chan sai che do */
void SPI_Loopback(void)
{
	static const uint8_t pattern[] = {0x00, 0xFF, 0x5A, 0xA5, 0x01, 0x80};
	uint8_t rx, bad = 0;

	printf("[SPI] loopback: noi PA7 vao PA6 truoc khi chay\r\n");
	for (uint8_t i = 0; i < sizeof(pattern); i++)
	{
		if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&pattern[i], &rx, 1, 100) != HAL_OK)
		{
			printf("[SPI] HAL loi o byte %u\r\n", (unsigned)i);
			bad++;
			continue;
		}
		printf("[SPI] gui %02X  nhan %02X  %s\r\n",
				pattern[i], rx, (rx == pattern[i]) ? "ok" : "SAI");
		if (rx != pattern[i])
			bad++;
	}
	printf("[SPI] %s\r\n", bad ? "THAT BAI: loi nam o MCU/SPI"
			: "DAT: MCU tot, loi o the nho hoac day noi");
}

/* Dap chan CS lien tuc de do bang dong ho. Do NGAY TAI CHAN CS CUA MODULE,
 * khong do tai PA4 - nhu vay moi kiem tra duoc ca doan day. Muc DC doc duoc
 * phai khoang 1,65 V. Doc 3,3 V hoac 0 V nghia la day dut hoac ngan mach. */
void SPI_CsBlink(uint32_t ms)
{
	uint32_t t0 = HAL_GetTick();

	printf("[SPI] dap CS trong %lu ms, do tai chan CS cua module\r\n",
			(unsigned long)ms);
	while (HAL_GetTick() - t0 < ms)
	{
		HAL_GPIO_TogglePin(SPI_CS_GPIO_Port, SPI_CS_Pin);
		HAL_Delay(1);
	}
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	printf("[SPI] xong, CS tra ve muc cao\r\n");
}

void CAN_NodeInit(void)
{
	CAN_FilterTypeDef filter = {0};
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000; /* mask 0 = nhan tat ca */
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
		Error_Handler();
	if (HAL_CAN_Start(&hcan) != HAL_OK)
		Error_Handler();

	canTxHeader.StdId = CAN_ID_HEARTBEAT;
	canTxHeader.ExtId = 0;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = 8;
	canTxHeader.TransmitGlobalTime = DISABLE;
}

/* Gui mot goi ra CA HAI duong truyen voi cung mot payload.
 * CAN  : StdId = NODE_BASE + pkt_id, toi da 8 byte.
 * UART : $<node>,<pkt>,<uptime>,<hex>*<xor>  -- doc duoc ngay trong tab Debug. */
void pkt_send(uint8_t pkt_id, const uint8_t *data, uint8_t len)
{
	if (len > 8) len = 8;

	/* --- CAN --- */
	canTxHeader.StdId = NODE_BASE + pkt_id;
	canTxHeader.DLC   = len;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
		HAL_CAN_AddTxMessage(&hcan, &canTxHeader, (uint8_t *)data, &canTxMailbox);

	/* --- UART --- */
	char body[48];
	int n = snprintf(body, sizeof(body), "%u,%02X,%lu,",
			(unsigned)NODE_ID, (unsigned)pkt_id,
			(unsigned long)HAL_GetTick());
	if (n < 0 || n >= (int)sizeof(body))
		return;

	for (uint8_t i = 0; i < len; i++)
	{
		if (n + 3 > (int)sizeof(body))
			break;
		n += snprintf(body + n, (size_t)(sizeof(body) - n), "%02X", data[i]);
	}

	HAL_Delay(200);

	uint8_t xo = 0;
	for (int i = 0; i < n; i++)
		xo ^= (uint8_t)body[i];

	char line[56];
	int m = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, xo);
	if (m > 0 && m < (int)sizeof(line))
		HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)m, 100);
}

/* Cong don LM35_OVERSAMPLE mau tren kenh LM35 (PA0 / ADC1_IN0).
 * Tra ve TONG chua chia, de phep tinh nhiet do giu duoc do phan giai. */
static uint32_t LM35_ReadSum(void)
{
	uint32_t sum = 0;

	for (uint8_t i = 0; i < LM35_OVERSAMPLE; i++)
	{
		if (HAL_ADC_Start(&hadc1) != HAL_OK)
			return 0;
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			sum += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	return sum;
}

void Sensor_SendEnv(void)
{
	uint32_t sum      = LM35_ReadSum();
	uint16_t raw_avg  = (uint16_t)(sum / LM35_OVERSAMPLE);
	int16_t  temp_x10 = (int16_t)((sum * LM35_VDDA_MV) / LM35_DIVISOR);

	uint8_t p[4];
	p[0] = (uint8_t)(raw_avg);
	p[1] = (uint8_t)(raw_avg >> 8);
	p[2] = (uint8_t)(temp_x10);
	p[3] = (uint8_t)(temp_x10 >> 8);
	pkt_send(PKT_ENV, p, 4);

	printf("[%lu] LM35 raw=%u  temp=%d.%d C\r\n",
			(unsigned long)HAL_GetTick(),
			(unsigned)raw_avg,
			(int)(temp_x10 / 10), (int)(temp_x10 % 10));
}

void CAN_SendHeartbeat(uint32_t uptime_ms)
{
	canTxData[0] = (uint8_t)(uptime_ms);
	canTxData[1] = (uint8_t)(uptime_ms >> 8);
	canTxData[2] = (uint8_t)(uptime_ms >> 16);
	canTxData[3] = (uint8_t)(uptime_ms >> 24);
	canTxData[4] = FW_MAJOR;
	canTxData[5] = FW_MINOR;
	canTxData[6] = NODE_ID;
	canTxData[7] = 0x00;
	pkt_send(PKT_HEARTBEAT, canTxData, 8);
}


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
	while (1)
	{
	}
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
