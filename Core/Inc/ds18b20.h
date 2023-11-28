/*
 * ds18b20.h
 *
 *  Created on: Nov 26, 2023
 *      Author: serioja8bit
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#define DS18B20_PORT GPIOA     	    //DS18B20Port Change with your ds18b20 port
#define DS18B20_PIN  GPIO_PIN_1 	//DS18b20Pin  Change with your ds18b20 pin
#define TIMER		 &htim6			//1Mhz timer  Change with your 1Mhz timer for 1 us delay function

#include "tim.h"
#include "stdio.h"

void delay (uint32_t us);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);
float DS18B20_Get_Temperature(void);


#endif /* INC_DS18B20_H_ */
