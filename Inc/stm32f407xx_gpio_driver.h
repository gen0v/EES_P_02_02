/*
 * stm32f407xx_gpio_driver.h
 * Vorlage fürs Praktikum 
 *  
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Struct für PIN-Konfiguration
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< mögliche Werte in @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< mögliche Werte in @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< mögliche Werte in @GPIO_PIN_PULL_UP_DOWN >*/
	uint8_t GPIO_PinOPType;			/*!< mögliche Werte in @GPIO_OUTPUT_MODES >*/
}GPIO_PinConfig_t;


/*
 * Handle struct für GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/*!< Basisadresse des GPIO ports >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< Pin Nummer und Konfiguration >*/
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin Nummern
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
 * GPIO pin Modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*
 + @GPIO_OUTPUT_MODES
 * GPIO pin Output-Modi
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin output speed settings
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * @GPIO_PIN_PULL_UP_DOWN
 * GPIO pin pull-up/down Konfigurationen
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/******************************************************************************************
 *								Funktionen des Treibers
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIO_Handle);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIO_Handle, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIO_Handle);

/*
 * IRQ Configuration and ISR handling 
 * Wird im zweiten Teil benötigt. 
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_DRIVER_H_ */
