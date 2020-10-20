/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "metronomo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	PAUSA=1,
	PLAY,
	REC,
} t_sec;

typedef enum{
	PULSO_NULO,
	PULSO_CORTO,
	PULSO_LARGO,

} t_pulso;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Placa_Corriendo(void);
t_pulso  FUNCION_BOTON(void);
void PLAY_PAUSE_REC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int ms2=0;				//Variable para alcanzar con el timer la variable paso
int POTE=0;				//Valor del ADC asociado al cambio de tempo del metrónomo
int ON=500;				//Variable de control,para ver que la placa está funcionando
int secuencia1[256];	//256 elementos para grabar 15 segundos de secuencia
int secuencia2[256];
int semi=0;
int paso=59;		//Corresponde a los milisegundos que hay entre dos semicorcheas con
					// una velocidad de 256 negras por minuto
int flagseq=0;		//Flag para avisar que el timer contó 1ms

int b1=0;			// b1,b2,PLAY,REC flags para avisar que se presionaron
int b2=0;			//los botones que definen la secuencia
int flag_play=0;
int flag_rec=0;

t_pulso BOTON_REC=0;		//Va a borrar la secuencia?
int ESTADO_BOTON=0;		//Variable que avisa que el boton se mantuvo presionado o no
int FLAG_TIME_OUT=0;	//Avisa que se recorto la secuencia
int TIME_OUT=256;		//Valor del vector a recorrer, si graba menos TIME_OUT cambia
int flagb=0;			//Flag para la función de REC_corto o REC_largo




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
  for(uint16_t i=0; i<256;i++)		//inicialización de vectores de secuencia
  {
	  secuencia1[i]=0;
	  secuencia2[i]=0;
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  //SysTick_Handler(void)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Placa_Corriendo();

//POTE metronomo, define tempo
	  HAL_ADC_Start(&hadc1);	//CONVERSION
	  POTE=HAL_ADC_GetValue(&hadc1);//LEE LA CONVERSION
	  POTE=(POTE*100)/447+225;
	  if(flagseq==1){
	  flagseq=0;
	  contadorms();	//actualiza el tiempo dentro del metrónomo (1ms)
	  }
	  Metronomo(POTE);

//Secuenciador


	  BOTON_REC=FUNCION_BOTON();


	  PLAY_PAUSE_REC();



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

/* USER CODE BEGIN 4 */


void Placa_Corriendo(void){
//Uso el sistyck para que prenda y apague el led asociado al PC13 así sé que
// el programa está corriendo.

	  if(ON==0)
	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  ON=500;
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM2){
		//está preguntando si el que termino de contar es el timer 2 :)

		flagseq=1;
		flagb=1;
	}

	}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
//Levanto flags según el botón que toque

 	if(GPIO_Pin==GPIO_PIN_5)
	{
		b1=1;
	}
	if(GPIO_Pin==GPIO_PIN_6)
	{
		b2=1;
	}
	if(GPIO_Pin==PLAY_PAUSE_Pin)
		{
			flag_play=1;
		}
	if(GPIO_Pin==REC_Pin)
		{
			flag_rec=1;

		}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

void PLAY_PAUSE_REC(void){
static t_sec PPR=PAUSA;	//Variable de maquina de estados.

	switch(PPR){
	case 1: //NADA - PAUSE
		// pregunta si tocaste REC
		if(BOTON_REC==PULSO_CORTO){
			PPR=REC;
		}
		// Pulsacion larga?
		if(BOTON_REC==PULSO_LARGO){
					PPR=PAUSA;
		for(uint16_t i=0; i<256;i++)		//inicialización de vectores de secuencia
					  {
						  secuencia1[i]=0;
						  secuencia2[i]=0;
					  }
			FLAG_TIME_OUT=0;
			TIME_OUT=256;
			semi=0;
				}
		if(flag_play==1){	//Toco una vez, empiezo a reproducir la secuencia
			PPR=PLAY;
			flag_play=0;
			semi=0;
		}
		break;

	case 2: //PLAY

		//Reproducción de secuencia

		if(flagseq==1){	//Contador del timer, se incrementa cada 1ms
			  flagseq=0;
			  ms2++; //incrementos de milisegundos para la secuencia


		  }

		if(ms2==paso){
			  semi++;		//Avance dentro del vector con la secuencia, definido en 60ms
			  ms2=0;
		  }

		if(semi==TIME_OUT){
			  semi=0;		//Se reinicia el vector
		  }

		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, secuencia1[semi]);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, secuencia2[semi]);


		if(BOTON_REC==1){
			PPR=REC;
			semi=0;
		}
		// Pulsacion larga?
		if(BOTON_REC==PULSO_LARGO){
						PPR=PAUSA;
		for(uint16_t i=0; i<256;i++)		//inicialización de vectores de secuencia
						  {
							  secuencia1[i]=0;
							  secuencia2[i]=0;
						  }
		FLAG_TIME_OUT=0;
		TIME_OUT=256;
		semi=0;
					}
		if(flag_play==1){		//PAUSA
			PPR=PAUSA;
			flag_play=0;
			semi=0;
		}

			break;

	case 3: //REC

		//Generación de secuencia

		  if(flagseq==1){	//Contador del timer, se incrementa cada 1ms
			  flagseq=0;
			  ms2++; //incrementos de milisegundos para la secuencia


		  }

		  if(ms2==paso){
			  semi++;		//Avance dentro del vector con la secuencia, definido en 60ms
			  ms2=0;
		  }

		  if(semi==TIME_OUT){
			  semi=0;		//Se reinicia el vector
		  }

		  if(b1==1){
			  b1=0;
			  secuencia1[semi]=1;
		  }
		  if(b2==1){
			  b2=0;
			  secuencia2[semi]=1;
		  }

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, secuencia1[semi]);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, secuencia2[semi]);

		  if(BOTON_REC==PULSO_CORTO){
					PPR=PAUSA;
					FLAG_TIME_OUT=1;	//Se acotó la secuencia
					TIME_OUT=semi;
					semi=0;

				}
		  // Pulsacion larga?
		  if(BOTON_REC==PULSO_LARGO){
							PPR=PAUSA;

		  for(uint16_t i=0; i<256;i++)		//inicialización de vectores de secuencia
								  {
									  secuencia1[i]=0;
									  secuencia2[i]=0;
								  }
			FLAG_TIME_OUT=0;
			TIME_OUT=256;
			semi=0;
							}

			break;


	}
}



t_pulso FUNCION_BOTON(void){ //Discrimina entre pulsar y mantener REC (REC_corto y REC_largo)
t_pulso RETORNO=PULSO_NULO;

static int ms_boton=0;

	 switch(ESTADO_BOTON){

		  	 case 0:
		  	 if(flag_rec==1){
		  		 ESTADO_BOTON=1;
		  		 flag_rec=0;
		  		 ms_boton=0;
		  	 }
		  	 break;

		  	 case 1:	//anti rebote
				  	if(flagb==1){
				  		flagb=0;
				  		ms_boton++;
				  	}
			if(ms_boton>29){
				ESTADO_BOTON=2;
			}
			break;

		  	case 2:
			  	if(flagb==1){
			  		flagb=0;
			  		ms_boton++;
			  	}

		  	if(HAL_GPIO_ReadPin(REC_GPIO_Port, REC_Pin)!=0){		//boton sin presionar
		  		ESTADO_BOTON=0;
		  		RETORNO=PULSO_CORTO;
		  	}
		  	if(ms_boton>=1500){		//se mantuvo presionado siempre, paso el primer if
		  	ESTADO_BOTON=0;
		  	RETORNO=PULSO_LARGO;
		  	}
		  	break;

		  	 }
	 	 	flag_rec=0;
		  	return(RETORNO);

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

  /* USER CODE END Error_Handler_Debug */
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
