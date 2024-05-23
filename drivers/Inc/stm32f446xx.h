/*
 * stm32f446xx.h
 *
 *  Created on: Mar 26, 2024
 *      Author: harigovind
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>
#include <stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))
/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4










/* base addresses of flash and SRAM memories */
#define FLASH_BASEADDR					0x08000000U			/*   base address of the flash memory      */
#define SRAM1_BASEADDR					0x20000000U			/*   base address of the sram1 memory      */
#define SRAM2_BASEADDR					0x2001C000U			/*   base address of the sram2 memory      */
#define ROM								0x1FFF0000U			/*   base address of the rom memory      */
#define SRAM							SRAM1_BASEADDR

/* base addresses of AHBx and APBx memories */

#define PERIPH_BASE						0x40000000U			/*   base address of the peripheral base      */
#define	APB1PERIPH_BASE					PERIPH_BASE			/*   base address of the APB1 peripheral base      */
#define	APB2PERIPH_BASE					0x40010000U			/*   base address of the APB2 peripheral base      */
#define	AHB1PERIPH_BASE					0x40020000U			/*   base address of the AHB1 peripheral base      */
#define	AHB2PERIPH_BASE					0x50000000U			/*   base address of the AHB2 peripheral base      */
#define AHB3PERIPH_BASE					0xA0001000U			/*   base address of the AHB3 peripheral base      */

/* base addresses of peripherals which are hanging in AHB1 bus */
#define	GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0X0000)
#define	GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0X0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0X1C00)

#define RCC_BASEADDR 					(AHB1PERIPH_BASE + 0X3800)

/* base addresses of peripherals which are hanging in APB1 bus */
#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASE + 0X5C00)
#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0X3C00)
#define USART2_BASEADDR					(APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0X4800)
#define UART4_BASEADDR					(APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0X5000)

/* base addresses of peripherals which are hanging in APB2 bus */
#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0X3C00)
#define SPI1_BASEADDR					(APB2PERIPH_BASE + 0X3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASE + 0X3400)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASE + 0X3800)
#define USART1_BASEADDR					(APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDR					(APB2PERIPH_BASE + 0X1400)
/***************************** Peripheral register definition structure ****/

typedef struct
{
	__vo uint32_t MODER;				/* GPIO port mode register (GPIOx_MODER) (x = A..H)												Address offset: 0x00*/
	__vo uint32_t OTYPER;				/* GPIO port output type register (GPIOx_OTYPER) (x = A..H)										Address offset: 0x04*/
	__vo uint32_t OSPEEDER;				/* GPIO port output speed register (GPIOx_OSPEEDR) (x = A..H) 									Address offset: 0x08*/
	__vo uint32_t PUPDR;				/* GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..H)								Address offset: 0x0C*/
	__vo uint32_t IDR;					/* GPIO port input data register (GPIOx_IDR) (x = A..H)											Address offset: 0x10*/
	__vo uint32_t ODR;					/* GPIO port output data register (GPIOx_ODR) (x = A..H)										Address offset: 0x14*/
	__vo uint32_t BSRR;					/* GPIO port bit set/reset register (GPIOx_BSRR) (x = A..H)										Address offset: 0x18*/
	__vo uint32_t LCKR;					/* GPIO port configuration lock register (GPIOx_LCKR) (x = A..H)								Address offset: 0x1C*/
	__vo uint32_t AFR[2];				/* GPIO alternate function low AFR[0] and high AFR[1] register (GPIOx_AFRL/H) (x = A..H)		Address offset: 0x20*/

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;					/* RCC clock control register (RCC_CR) Address offset: 0x00*/
	__vo uint32_t PLLCFGR;				/* RCC PLL configuration register (RCC_PLLCFGR) Address offset: 0x04*/
	__vo uint32_t CFGR;					/* RCC clock configuration register (RCC_CFGR) Address offset: 0x08*/
	__vo uint32_t CIR;					/* RCC clock interrupt register (RCC_CIR) Address offset: 0x0C*/
	__vo uint32_t AHB1RSTR;				/* RCC AHB1 peripheral reset register (RCC_AHB1RSTR) Address offset: 0x10*/
	__vo uint32_t AHB2RSTR;				/* RCC AHB2 peripheral reset register (RCC_AHB2RSTR) Address offset: 0x14*/
	__vo uint32_t AHB3RSTR;				/* RCC AHB3 peripheral reset register (RCC_AHB3RSTR) Address offset: 0x18*/
	uint32_t RESERVED0;					/* RESERVED Address offset: 0x1C*/
	__vo uint32_t APB1RSTR;				/* RCC APB1 peripheral reset register (RCC_APB1RSTR) Address offset: 0x20*/
	__vo uint32_t APB2RSTR;				/* RCC APB2 peripheral reset register (RCC_APB2RSTR) Address offset: 0x24*/
	uint32_t RESERVED1[2];				/* RESERVED Address offset: 0x28*/
	__vo uint32_t AHB1ENR;				/* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) Address offset: 0x30*/
	__vo uint32_t AHB2ENR;				/* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR) Address offset: 0x34*/
	__vo uint32_t AHB3ENR;				/* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR) Address offset: 0x38*/
	uint32_t RESERVED3;					/* RESERVED Address offset: 0x3C*/
	__vo uint32_t APB1ENR;				/* RCC APB1 peripheral clock enable register (RCC_APB1ENR)Address offset: 0x40*/
	__vo uint32_t APB2ENR;				/* RCC APB2 peripheral clock enable register (RCC_APB2ENR) Address offset: 0x44*/
	uint32_t RESERVED4[2];				/* RESERVED Address offset: 0x48*/
	__vo uint32_t AHB1LPENR;			/* RCC AHB1 peripheral clock enable in low power mode register	(RCC_AHB1LPENR) Address offset: 0x50*/
	__vo uint32_t AHB2LPENR;			/* RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)Address offset: 0x54*/
	__vo uint32_t AHB3LPENR;			/* RCC AHB3 peripheral clock enable in low power mode register(RCC_AHB3LPENR) Address offset: 0x58*/
	uint32_t RESERVED5;					/* RESERVED Address offset: 0x5C*/
	__vo uint32_t APB1LPENR;			/* RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)Address offset: 0x60*/
	__vo uint32_t APB2LPENR;			/* RCC APB2 peripheral clock enabled in low power mode register(RCC_APB2LPENR) Address offset: 0x64*/
	uint32_t RESERVED6[2];				/* RESERVED Address offset: 0x68*/
	__vo uint32_t BDCR;					/* RCC Backup domain control register (RCC_BDCR) Address offset: 0x70*/
	__vo uint32_t CSR;					/* RCC clock control & status register (RCC_CSR) Address offset: 0x74*/
	uint32_t RESERVED7[2];				/* RESERVED Address offset: 0x78*/
	__vo uint32_t SSCGR;				/* RCC spread spectrum clock generation register (RCC_SSCGR) Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;			/* RCC PLLI2S configuration register (RCC_PLLI2SCFGR) Address offset: 0x84*/
	__vo uint32_t PLLSAICFGR;			/* RCC PLL configuration register (RCC_PLLSAICFGR) Address offset: 0x88*/
	__vo uint32_t DCKCFGR;				/* RCC dedicated clock configuration register (RCC_DCKCFGR) Address offset: 0x8C*/
	__vo uint32_t CKGATENR;				/* RCC clocks gated enable register (CKGATENR) Address offset: 0x90*/
	__vo uint32_t DCKCFGR2;				/* RCC dedicated clocks configuration register 2 (DCKCFGR2) Address offset: 0x94*/
}RCC_RegDef_t;

/***************************** Peripheral register definition structure for EXTI****/

typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1; 		     		/*!< Give a short description,                    	Address offset: 0x00      */
	__vo uint32_t CR2;      		    /*!< TODO,     									  	Address offset: 0x04      */
	__vo uint32_t SR;   				/*!< TODO , 									  	Address offset: 0x08      */
	__vo uint32_t DR;  					/*!< TODO          							  		Address offset: 0x0C      */
	__vo uint32_t CRCPR;        		/*!< TODO         								  	Address offset: 0x10      */
	__vo uint32_t RXCRCR;  				/*!<                                             	Address offset: 0x14 	  */
	__vo uint32_t TXCRCR;        		/*!< TODO                                         	Address offset: 0x18   	  */
	__vo uint32_t I2SCFGR;         		/*!< TODO                                        	Address offset: 0x1C   	  */
	__vo uint32_t I2SPR;         		/*!< TODO                                         	Address offset: 0x20   	  */
} SPI_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t USR; 		     		/*!< Give a short description,                    	Address offset: 0x00      */
	__vo uint32_t DR;      		    /*!< TODO,     									  	Address offset: 0x04      */
	__vo uint32_t BRR;   				/*!< TODO , 									  	Address offset: 0x08      */
	__vo uint32_t CR1;  					/*!< TODO          							  		Address offset: 0x0C      */
	__vo uint32_t CR2;        		/*!< TODO         								  	Address offset: 0x10      */
	__vo uint32_t CR3;  				/*!<                                             	Address offset: 0x14 	  */
	__vo uint32_t GTPR;        		/*!< TODO                                         	Address offset: 0x18   	  */
} USART_RegDef_t;







/*  peripheral definitions (peripheral bases typecasted to xxx_Regdef_t)  */
#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)





/*CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS*/
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))


/*CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS*/
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*CLOCK ENABLE MACROS FOR SPIx PERIPHERALS*/
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
/*CLOCK ENABLE MACROS FOR USARTx PERIPHERALS*/
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*CLOCK ENABLE MACROS FOR SYSCGF PERIPHERALS*/
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))



/*CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS*/
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))


/*CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS*/
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*CLOCK DISABLE MACROS FOR SPIx PERIPHERALS*/
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
/*CLOCK DISABLE MACROS FOR USARTx PERIPHERALS*/
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*CLOCK DISABLE MACROS FOR SYSCGF PERIPHERALS*/
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))




/*MACROS TO RESET GPIOx PERIPHERALS*/

#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


/*MACROS TO RESET SPIx PERIPHERALS*/

#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*MACROS TO RESET USARTx PERIPHERALS*/
#define USART1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 <<  4));  (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()               do{ (RCC->APB2RSTR |= (1 <<  5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)



#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7:0)
/*
 * IRQ(Interrupt Request) Numbers of STM32F446x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2 		36
#define IRQ_NO_SPI3 		51

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

//some generic macros

#define ENABLE		 	1
#define DISABLE 		0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9







#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_usart_driver.h"
#endif /* INC_STM32F446XX_H_ */
