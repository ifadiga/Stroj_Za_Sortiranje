/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* za printf */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CIJEV_PUNO 3000 //postaviti vrijednost adc-senzora kada u cijevi ima predmeta
//senzor mora oèitati vecu vrijednost od CIJEV_PUNO da bi stroj radio
#define BROJ_KORAKAK_ZA_KRUG 200  //definiramo koliko stepper motor mora napraviti koraka da bi obišao jedan krug
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t rx_buff[RX_BUFF_SIZE];

uint8_t msg_protocol[33];
uint8_t det_obj_buff[3];

uint8_t sys_flag = 0;

//faktor za vagu koji treba podesiti da bi dobro vagala( za svaki load cell drugaciji)
float calibration_factor = -900;
long OFFSET = 0;
float SCALE = 1;
int stepperPosition = 0;

struct Osobine {
	char boja;
	char oblik;
	int masa;
	int min_masa;
	int max_masa;
	int broj_poklapanja;
};

struct Osobine Predmet;
struct Osobine Spremnik[3];

//enum za pwm signal
typedef enum {
	PWM1 = TIM_CHANNEL_1,
	PWM2 = TIM_CHANNEL_2,
	PWM3 = TIM_CHANNEL_3,
	PWM4 = TIM_CHANNEL_4,

} PWM_CHANNEL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//funkcija za parsiranje podataka za spremnike
void Parse_spremnik();

//funkcija za parsiranje podataka o slikanom predmetu
void Parse_predmet();

int CharToInt3(int n, uint8_t* Polje);
int CharToInt4(int n, uint8_t* Polje);

/*prototip funkcije za analizu predmeta
 *funkcija prema definiranim osobinama spremnika i predmeta sprema u varijablu odabrani_spremnik
 *prema cemu se poklapa svaki spremnik sa trenutnim predmetom(boja, masa i oblik)
 *i onda samo treba vidjeti u toj varijabli koji spremnik ima najviše poklapanja i staviti predmet u njega
 */
void Analiza_predmeta(void);

//funkcija koja ovisno o tome koji spremnik ima najviše ispunjenih uvjeta odabire jedan spremnik za trenutni predmet
int Odabir_spremnika(void);

/*funkcije za vagu*/
int HX711_read(void);
long HX711_read_average(int times);
double HX711_get_value(int times);
float HX711_get_units(int times);
void HX711_set_offset(long offset);
void HX711_set_scale(float scale);
void HX711_Tare(int times);

/*funkcije za stepper motor*/
void Stepper_Step(int dir, int step);

//funkcija za servo motor, timer je za koji timer postavljamo pwm, a kut je kut motora
void Servo_motor(PWM_CHANNEL PWM_CH, int kut);
//funkcija koja pomice predmet iz cijevi na vagu.
void Pomakni_na_vagu(void);
//gurne predmet sa vage u spremnik
void Makni_sa_vage(void);
//funkcija koja postavlja spremnik koji joj predamo kao argument
void Postavi_spremnik(int spremnik);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // <-------- ENABLE RXNE
	/*
	 HX711_set_scale(1);
	 HX711_Tare(10);
	 HX711_set_scale(calibration_factor);
	 */
	//kod za pokretanje TIM1 i PWM signala
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Start Pwm signal on PA-8 Pin
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Start Pwm signal on PA-9 Pin
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Start Pwm signal on PA-10 Pin
	//kod za pokretanje ADC-a
	HAL_ADC_Start(&hadc1);

	//postavljamo servo motore na pozicije u kojima miruju
	Servo_motor(PWM2, 180);
	Servo_motor(PWM3, 80);
	int number = 0;
	sys_flag = 0;
	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (sys_flag == 0) {
			Parse_spremnik();
		}
		//------------------------------SORTIRANJE PREDMETA----------------------------
		else if (sys_flag == 1) {
			//ocitava vrijednost senzora te ako ima predmeta u cijevi krece raditi
			if ((HAL_ADC_GetValue(&hadc1)) >= CIJEV_PUNO) {
				Pomakni_na_vagu();
				//pomicemo predmet na pola vage
				Servo_motor(PWM2, 160);
				HAL_Delay(500);
				Servo_motor(PWM2, 180);
				//vaganje i slanje signala rpi za slikanje
				Predmet.masa = HX711_get_units(10);
				HAL_GPIO_WritePin(GPIOA, RPI_GPIO_Pin, 1);
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOA, RPI_GPIO_Pin, 0);

				while (det_obj_buff[2] == '#')
					;
				Parse_predmet();
				int sprem = Odabir_spremnika();
				Postavi_spremnik(sprem);
				Makni_sa_vage();
			} else {
				printf("SORTIRANJE GOTOVO\r\n");
			}
		}
		//----------------------------ANALIZA PREDMETA----------------------
		else if (sys_flag == 2) {

		}

		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 20;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 7656;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			SENZOR_LED_Pin | VAGA_SCK_Pin | STEPPER_DIR_Pin | STEPPER_STEP_Pin
					| RPI_GPIO_Pin | STEPPER_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SENZOR_LED_Pin VAGA_SCK_Pin STEPPER_DIR_Pin STEPPER_STEP_Pin
	 RPI_GPIO_Pin STEPPER_EN_Pin */
	GPIO_InitStruct.Pin = SENZOR_LED_Pin | VAGA_SCK_Pin | STEPPER_DIR_Pin
			| STEPPER_STEP_Pin | RPI_GPIO_Pin | STEPPER_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : VAGA_DT_Pin */
	GPIO_InitStruct.Pin = VAGA_DT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VAGA_DT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//funkcija za parsiranje podataka za spremnike
void Parse_spremnik() {
	for (int j = 0; j < 3; j++) {
		Spremnik[j].boja = msg_protocol[(12 * j + 2)];
		Spremnik[j].oblik = msg_protocol[(12 * j + 3)];
		Spremnik[j].min_masa = CharToInt3((12 * j + 5), msg_protocol);
		Spremnik[j].max_masa = CharToInt4((12 * j + 8), msg_protocol);
	}

}

//funkcija za parsiranje podataka o slikanom predmetu
void Parse_predmet() {
	Predmet.boja = det_obj_buff[0];
	Predmet.oblik = det_obj_buff[1];

}
int CharToInt3(int n, uint8_t* Polje) {
	int broj = 0;
	for (int i = 0; i < 3; i++) {
		if (i == 0 && Polje[n] != '0') {
			broj = (Polje[n] - 48) * 100;
		} else if (i == 1) {
			broj += (Polje[n + 1] - 48) * 10;
		} else if (i == 2) {
			broj += (Polje[n + 2] - 48);
		}
	}
	return broj;

}
int CharToInt4(int n, uint8_t* Polje) {
	int broj = 0;
	for (int i = 0; i < 4; i++) {
		if (i == 0 && Polje[n] != '0') {
			broj = (Polje[n] - 48) * 1000;
		} else if (i == 1) {
			broj += (Polje[n + 1] - 48) * 100;
		} else if (i == 2) {
			broj += (Polje[n + 2] - 48) * 10;
		} else if (i == 3) {
			broj += (Polje[n + 3] - 48);
		}
	}
	return broj;
}

//funkcija za analizu kriteriaj za svaki spremnik i trenutni predmet-----------------------------------
void Analiza_predmeta(void) {
	int i = 0;

	for (i = 0; i < 3; i++) {
		if (Spremnik[i].boja == Predmet.boja) {
			Spremnik[i].broj_poklapanja++;
		}
		if (Spremnik[i].oblik == Predmet.oblik) {
			Spremnik[i].broj_poklapanja++;
		}
		if (Spremnik[i].max_masa >= Predmet.masa
				|| Predmet.masa >= Spremnik[i].min_masa) {
			Spremnik[i].broj_poklapanja++;
		}
	}
}

//funkcija za odabir spremnika------------------ODABIR SPREMNIKA----------------------------------------
int Odabir_spremnika() {
	int max_poklapanja = 0;
	int index = 0;
	for (int i = 0; i < 3; i++) {
		if (Spremnik[i].broj_poklapanja > max_poklapanja) {
			max_poklapanja = Spremnik[i].broj_poklapanja;
			index = i + 1;
		} else {
			index = 4;
		}
	}

	return index;
}

//FUNKCIJE ZA VAGU----------------------------------VAGA------------------------------------------
int HX711_read(void) {
	int buffer;
	buffer = 0;

	while (HAL_GPIO_ReadPin(GPIOA, VAGA_DT_Pin) == 1)
		;

	for (uint8_t i = 0; i < 24; i++) {
		HAL_GPIO_WritePin(GPIOA, VAGA_SCK_Pin, GPIO_PIN_SET);

		buffer = buffer << 1;

		if (HAL_GPIO_ReadPin(GPIOA, VAGA_DT_Pin)) {
			buffer++;
		}

		HAL_GPIO_WritePin(GPIOA, VAGA_SCK_Pin, GPIO_PIN_RESET);
	}

	for (int i = 0; i < 1; i++) {
		HAL_GPIO_WritePin(GPIOA, VAGA_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, VAGA_SCK_Pin, GPIO_PIN_RESET);
	}

	buffer = buffer ^ 0x800000;

	return buffer;
}

long HX711_read_average(int times) {
	long sum = 0;
	for (int i = 0; i < times; i++) {
		sum += HX711_read();
	}

	return sum / times;
}
double HX711_get_value(int times) {
	return HX711_read_average(times) - OFFSET;
}

float HX711_get_units(int times) {
	return HX711_get_value(times) / SCALE;
}

void HX711_set_offset(long offset) {
	OFFSET = offset;
}

void HX711_set_scale(float scale) {
	SCALE = scale;
}

void HX711_Tare(int times) {
	double sum = HX711_read_average(times);

	HX711_set_offset(sum);
}

//funkcija za stepper motor-------------------------STEPPER-------------------------------
void Stepper_Step(int dir, int step) {
	HAL_GPIO_WritePin(GPIOA, STEPPER_EN_Pin, GPIO_PIN_SET);
	if (dir == 1) {
		HAL_GPIO_WritePin(GPIOA, STEPPER_DIR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, STEPPER_DIR_Pin, GPIO_PIN_RESET);
	}
	for (int i = 0; i < step; i++) {
		HAL_GPIO_WritePin(GPIOA, STEPPER_STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOA, STEPPER_STEP_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	}
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, STEPPER_EN_Pin, GPIO_PIN_RESET);
}

void Servo_motor(PWM_CHANNEL PWM_CH, int kut) {
	float stupnjevi = ((7656 / 1800.0) * (kut + 45.0));
	__HAL_TIM_SET_COMPARE(&htim1, PWM_CH, (stupnjevi)); //180 stupnjeva 8000*0,125
}

void Pomakni_na_vagu() {
	Servo_motor(PWM3, 120);
	HAL_Delay(1000);
	Servo_motor(PWM3, 38);
	HAL_Delay(1000);
	Servo_motor(PWM3, 80);
	HAL_Delay(1000);
}
void Makni_sa_vage() {
	Servo_motor(PWM2, 160);
	HAL_Delay(1000);
	Servo_motor(PWM2, 110);
	HAL_Delay(1000);
	Servo_motor(PWM2, 180);
	HAL_Delay(1000);
}
void Postavi_spremnik(int spremnik) {

	switch (spremnik) {
	case 1:
		if (stepperPosition == 2) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 4), 0);
		} else if (stepperPosition == 3) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 2), 0);
		} else if (stepperPosition == 4) {
			Stepper_Step(((BROJ_KORAKAK_ZA_KRUG / 4) * 3), 0);
		}
		stepperPosition = 1;
		break;
	case 2:
		if (stepperPosition == 1) {
			Stepper_Step(((BROJ_KORAKAK_ZA_KRUG / 4) * 3), 0);
		} else if (stepperPosition == 3) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 4), 0);
		} else if (stepperPosition == 4) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 2), 0);
		}
		stepperPosition = 2;
		break;
	case 3:
		if (stepperPosition == 1) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 2), 0);
		} else if (stepperPosition == 2) {
			Stepper_Step(((BROJ_KORAKAK_ZA_KRUG / 4) * 3), 0);
		} else if (stepperPosition == 4) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 4), 0);
		}
		stepperPosition = 3;
		break;
	case 4:
		if (stepperPosition == 1) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 4), 0);
		} else if (stepperPosition == 2) {
			Stepper_Step((BROJ_KORAKAK_ZA_KRUG / 2), 0);
		} else if (stepperPosition == 3) {
			Stepper_Step(((BROJ_KORAKAK_ZA_KRUG / 4) * 3), 0);
		}
		stepperPosition = 4;
		break;
	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of
	 transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

#ifdef  USE_FULL_ASSERT
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
