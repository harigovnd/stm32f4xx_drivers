/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Mar 26, 2024
 *      Author: harigovind
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*A config structure of GPIO pin*/
typedef struct{
	uint8_t GPIO_PinNumber;		/*Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;		/*Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;		/*Possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;/*Possible values from @GPIO_PIN_PU_PD_CONTROL*/
	uint8_t GPIO_PinOPType;		/*Possible values from @GPIO_PIN_OP_TYPE*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



/*A handle structure of GPIO pin*/

typedef struct{
	// pointer to hold the GPIO peripheral base address
	GPIO_RegDef_t *pGPIOx;    /*This holds the base address of GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*This holds the GPIO pin config settings*/

}GPIO_Handle_t;







/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0				//non-interrupt mode
#define GPIO_MODE_OUT 		1				//non-interrupt mode
#define GPIO_MODE_ALTFN 	2				//non-interrupt mode
#define GPIO_MODE_ANALOG 	3				//non-interrupt mode
#define GPIO_MODE_IT_FT     4				//interrupt mode
#define GPIO_MODE_IT_RT     5				//interrupt mode
#define GPIO_MODE_IT_RFT    6				//interrupt mode


/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * @GPIO_PIN_PU_PD_CONTROL
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/**********************************************************************************************
 * APIs Supported by this driver
 * For more info about APIs check function definition
 * ********************************************************************************************/
/* Peripheral clock setup*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Initialization and De-init*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*Data read and write*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber );
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value );
void GPIO_WriteFromInputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value );
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*IRQ config and IRQ handling*/

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

















#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
