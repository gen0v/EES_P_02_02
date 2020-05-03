/*
 * stm32f407xx.h
 * Vorlage fürs Praktikum 
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stddef.h>
#include<stdint.h>

//Allgemeine Makros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET

/*
 * Basis-Adressen der AHBx and APBx Busse
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Basis Adresses der GPIO-Ports
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x2000U)

/*
 * Basis Adresses des Reset and Clock control Module (RCC)
 */
#define RCC_BASEADDR					0x40023800U


/********************************** Register definition structs **********************************/



/*
 * Struct mit Konfigurations und Daten-Registern der GPIO-Ports
 * Welche weiteren Register zur Konfiguration können in dem Struct eingefügt werden und was ist Ihre Bedeutung?
 */
typedef struct
{
  volatile uint32_t MODE_REG;      				/* Address offset: 0x00 */
  volatile uint32_t OUTPUT_TYPE;  				/* Address offset: 0x04 */
  volatile uint32_t OUTPUT_SPEED;  				/* Address offset: 0x08 */
  volatile uint32_t PULL_UP_DOWN;  				/* Address offset: 0x0C */
  volatile uint32_t INPUT_DATA_REG;    			/* Address offset: 0x10 */
  volatile uint32_t OUTPUT_DATA_REG;			/* Address offset: 0x14 */

//  volatile uint16_t BIT_SET_RESET_REG;			/* Address offset: 0x18 */
//  volatile uint32_t ALT_FUNC_LOW_REG;			/* Address offset: 0x20 */
//  volatile uint32_t ALT_FUNC_HIGH_REG;			/* Address offset: 0x24 */
//  volatile uint32_t LOCK_REG;					/* Address offset: 0x1C */

}GPIO_RegDef_t;

/*
 * Konfigurationsregister des RCC
 */
typedef struct
{
	volatile uint32_t CR;      		/* Address offset: 0x00 */
	volatile uint32_t PLLCFGR;      /* Address offset: 0x04 */
	volatile uint32_t CFGR;      	/* Address offset: 0x08 */
	volatile uint32_t CIR;      	/* Address offset: 0x0C */

	volatile uint32_t AHB1RSTR;      /*Address offset: 0x10 */

	volatile uint32_t AHB2RSTR;      /* Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;      /* Address offset: 0x18 */
	uint32_t RESERVER0;				 /* Address offset: 0x1C */
	volatile uint32_t APB1RSTR;      /* Address offset: 0x20 */
	volatile uint32_t APB2RSTR;      /* Address offset: 0x24 */
	uint32_t RESERVED1;				 /* Address offset: 0x28 */
	uint32_t RESERVED2;				 /* Address offset: 0x2C */
	volatile uint32_t AHB1ENR;       /* Address offset: 0x30 */

	volatile uint32_t AHB2ENR;       /* Address offset: 0x34 */
	volatile uint32_t AHB3ENR;       /* Address offset: 0x38 */
	uint32_t RESERVED3;				 /* Address offset: 0x3C */



} RCC_RegDef_t;


/********************************** Macros to access peripherals **********************************/

/*
 * Basisadressen gecastet auf entsprechende structs
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Makros für GPIO-Ports
 * DONE: Schreiben Sie Clock Enable und Disable Makros für die GPIO-Ports A bis I
 * Sehen Sie sich dazu die Informationen im Handbuch zum AHB1ENR-Register des RCC-Moduls an.
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))


/*
 *  Makros um GPIO-Ports zurückzusetzen
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

// ####################################### Teil IRQ ############################################################
/*
 * ARM Cortex Mx Processor NVIC register Addresses (Nested Vector Interrupt Controller)
 * TODO: Tragen sie die Basisadressen aus dem Cortex M4 Handbuch ein.
 * +++
 */

/* ISER : Interrupt Set-enable Register*/
#define NVIC_ISER0			0xE000E100U
#define NVIC_ISER1          0xE000E104U
#define NVIC_ISER2          0xE000E108U
#define NVIC_ISER3          0xE000E10CU

#define NVIC_ICER0 			0xE000E180U
#define NVIC_ICER1			0xE000E184U
#define NVIC_ICER2  		0xE000E188U
#define NVIC_ICER3			0xE000E18CU


/*
 * IRQ(Interrupt Request) Nummern für STM32F407x MCU
 * TODO: Ausfüllen der IRQ--Nummern für EXTI
 */

#define IRQ_NO_EXTI0			0x00000058
#define IRQ_NO_EXTI1 			0x0000005C
#define IRQ_NO_EXTI2 			0x00000060
#define IRQ_NO_EXTI3 			0x00000064
#define IRQ_NO_EXTI4 			0x00000068
#define IRQ_NO_EXTI9_5 			0x0000009C
#define IRQ_NO_EXTI15_10 		0x000000E0

/*
 * EXTI Adressen und Makros
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
/*
 * Sysconfig Adressen und Makros
 */
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * Clock Enable und Disable Makros für SYSCFG
 * DONE: Vervollständigen Sie die Makros zum anschalten und Abschalten der Syscfg Clock
 */
#define SYSCFG_PCLK_EN()	do{RCC->APB2PERIPH_BASEADDR |= (1<<14)}while(0)
#define SYSCFG_PCLK_DI()	do{RCC->APB2PERIPH_BASEADDR &= ~(1<<14)}while(0)

/*
 * Register struct EXTI
 * DONE: Legen Sie ein entsprechendes Struct an
 */
typedef struct
{
	volatile uint32_t IMR;    /*!< Aktivierung Interrupt    Address offset: 0x00 */
	volatile uint32_t EMR;    /*!< Aktivierung Event	    Address offset: 0x04 */
	volatile uint32_t RTSR;    /*!< Rising Edge Trigger    	Address offset: 0x08 */
	volatile uint32_t FTSR;    /*!< Falling Edge Trigger    Address offset: 0x0C */
	volatile uint32_t SWIER;    /*!< TODO    Address offset: 0x10 */
	volatile uint32_t PR;    /*!< Pending interrupts    Address offset: 0x14 */
	//...
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;       /*!< 											Address offset: 0x00		*/
	volatile uint32_t PMC;          /*!<											Address offset: 0x04		*/
	volatile uint32_t EXTICR[4];    /*!									 			Address offset: 0x08-0x14	*/
	uint32_t      RESERVED1[2];  /*!<											Reserved, 0x18-0x1C				*/
	volatile uint32_t CMPCR;        /*!<											Address offset: 0x20		*/
	uint32_t      RESERVED2[2];  /*!<											Reserved, 0x24-0x28 			*/
	volatile uint32_t CFGR;         /*!<											Address offset: 0x2C		*/
} SYSCFG_RegDef_t;


/*
 * Makro zum zurückgeben eines Codes ( between 0 to 7) für den entsprechenden GPIO-PORT (x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)



#include "stm32f407xx_gpio_driver.h"

#endif /* STM32F407XX_H_ */
