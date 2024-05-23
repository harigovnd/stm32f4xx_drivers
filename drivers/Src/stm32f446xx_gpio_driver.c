/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Mar 26, 2024
 *      Author: harigovind
 */

#include "stm32f446xx_gpio_driver.h"






/* Peripheral clock setup*/
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}


}







/*Initialization and De-init*/
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initialize GPIO port peripherals
 *
 * @param[in]         - pointer to the handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; //temp register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non-interrupt mode

		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |=temp; //setting
	}else
	{
		// we'll do it later (IRQ mode)
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_FT)
		{
			//1. config the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RT)
		{
			//1. config the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RFT)
		{
			//1. config the FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. Config GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xF << (4 * temp2));
		SYSCFG->EXTICR[temp1] |= portcode << ( temp2 * 4);
		// 3. Enable EXTI irq delvery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp=0;
	//2. configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |=temp;//setting
	temp=0;
	//3. configure the pupd settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |=temp;//setting
	temp=0;
	//4. configure the optype
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OTYPER |=temp;//setting
	temp=0;
	//5. configure the alt functionality
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_ALTFN)
	{
		// configure alternate fun register
		uint8_t temp1, temp2;
		temp1= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		temp2= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4* temp2));//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2));//setting
	}
}





/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - De-Initialize GPIO port peripherals
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}









/*Data read and write*/
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read from a GPIO pin
 *
 * @param[in]         -	base address of the gpio peripheral
 * @param[in]         -	Pin number
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              - none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
}







/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Read from a GPIO port peripherals
 *
 * @param[in]         -	base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - port of 16 pins
 *
 * @Note              - none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

}







/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write to a GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin number
 * @param[in]         -	Value is either SET/RESET
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value )
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}

}








/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write to a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -	16 pins in a port (1/0)
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value )
{
	pGPIOx->ODR  = Value;

}








/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - toggle the GPIO pin value
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -	Pin number
 * @param[in]         -
 *
 * @return            -	none
 *
 * @Note              - none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);

}









/*
 * IRQ config and IRQ handling
 */

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - configure the interrupt in GPIO pin (Enable/Disable, change priority)
 *
 * @param[in]         -	IRQnumber
 * @param[in]         -	priority of the IRQ
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - IRQGrouping can also be considered, which we have not considered in this project

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}





/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - to handle the interrupt in GPIO pin number
 *
 * @param[in]         - Pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}

