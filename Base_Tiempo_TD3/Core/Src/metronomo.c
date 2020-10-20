/*
 * metronomo.c
 *
 *  Created on: Sep 18, 2020
 *      Author: leandro
 */

#include "metronomo.h"

int tiempo=1;
int ms=0;

void Metronomo(int ADC){



	  switch(tiempo)
	  {
	  case 1:
		  HAL_GPIO_WritePin(T1_GPIO_Port, T1_Pin,0);
		  HAL_GPIO_WritePin(T1_GPIO_Port, T2_Pin,1);
		  HAL_GPIO_WritePin(T1_GPIO_Port, T3_Pin,1);
		  HAL_GPIO_WritePin(T1_GPIO_Port, T4_Pin,1);

		  if(ms>=ADC){
			  tiempo=2;

	}
	  break;
	  case 2:
		  HAL_GPIO_WritePin(T1_GPIO_Port, T1_Pin,1);
		  HAL_GPIO_WritePin(T2_GPIO_Port, T2_Pin,0);
		  HAL_GPIO_WritePin(T3_GPIO_Port, T3_Pin,1);
		  HAL_GPIO_WritePin(T4_GPIO_Port, T4_Pin,1);

		  if(ms>=2*ADC){
			  tiempo=3;

	}
	  break;
	  case 3:
		  HAL_GPIO_WritePin(T1_GPIO_Port, T1_Pin,1);
		  HAL_GPIO_WritePin(T2_GPIO_Port, T2_Pin,1);
		  HAL_GPIO_WritePin(T3_GPIO_Port, T3_Pin,0);
		  HAL_GPIO_WritePin(T4_GPIO_Port, T4_Pin,1);
		  if(ms>=3*ADC){
			  tiempo=4;

	}
	  break;
	  case 4:
		  HAL_GPIO_WritePin(T1_GPIO_Port, T1_Pin,1);
		  HAL_GPIO_WritePin(T2_GPIO_Port, T2_Pin,1);
		  HAL_GPIO_WritePin(T3_GPIO_Port, T3_Pin,1);
		  HAL_GPIO_WritePin(T4_GPIO_Port, T4_Pin,0);

		  if(ms>=4*ADC){
			  tiempo=1;
			  ms=0;

	}
	  break;
	  }
}

void contadorms(void){	//Cuenta milisegundos
	ms++;
}
