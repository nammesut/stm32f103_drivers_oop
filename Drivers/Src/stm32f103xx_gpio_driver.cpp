/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Apr 11, 2024
 *      Author: DELL
 */
#include "stm32f103xx_gpio_driver.h"

// Constructor init GPIO
GPIO::GPIO(GPIO_Handle_t pGPIOHandle)
{
	this->pGPIOHandle = pGPIOHandle;
}

/* Peripheral Clock config */
void GPIO::GPIO_PeriCockControl(uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (this->pGPIOHandle.pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (this->pGPIOHandle.pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}
	else
	{
		if (this->pGPIOHandle.pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (this->pGPIOHandle.pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
	}
}

void GPIO::GPIO_Init()
{
	uint32_t temp = 0;

	/* Enable GPIO Peripheral */
	GPIO::GPIO_PeriCockControl(ENABLE);

	/* Config GPIO Mode non interrupt */
	if( this->pGPIOHandle.GPIO_PinConfig.GPIO_PinType <= GPIO_MODE_OUT_HIGH_SPEED )
	{
		uint8_t temp1 = 0, temp2 = 0;

		/* temp1 config CRL or CRH */
		temp1 = this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber / 8;

		/* temp2 config position pinnumber */
		temp2 = this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber % 8;

		temp = this->pGPIOHandle.GPIO_PinConfig.GPIO_PinType | this->pGPIOHandle.GPIO_PinConfig.GPIO_PinMode;

		temp = temp << (4 * temp2);

		this->pGPIOHandle.pGPIOx->CR[temp1] &= ~(0xF << (4 * temp2));
		this->pGPIOHandle.pGPIOx->CR[temp1] |= temp;

	/* Config GPIO Mode interrupt */
	}else
	{
		/* 1. Config trigger raising or falling edge */
		if (this->pGPIOHandle.GPIO_PinConfig.GPIO_PinType == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);

		}else if (this->pGPIOHandle.GPIO_PinConfig.GPIO_PinType == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);

		}else if (this->pGPIOHandle.GPIO_PinConfig.GPIO_PinType == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
		}

		/* 2. Config EXTI line */
		uint8_t temp1 = 0, temp2 = 0;

		temp1 = this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber % 4;

		AFIO_PCLK_EN();

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(this->pGPIOHandle.pGPIOx);

		AFIO->EXTICR[temp1] |= (portcode << (4 * temp2));

		/* 3. Enabel interrup IMR */
		EXTI->IMR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
}


/* GPIO DeInit */
void GPIO::GPIO_DeInit()
{
	if (this->pGPIOHandle.pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (this->pGPIOHandle.pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
}

/*
 * GPIO Read - write
 * */
uint8_t GPIO::GPIO_ReadFromInputPin()
{
	uint8_t value;

	value = (uint8_t)((this->pGPIOHandle.pGPIOx->IDR >> this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber) & 0x1);

	return value;
}

uint16_t GPIO::GPIO_ReadFromInputPort()
{
	uint16_t value;

	value = (uint16_t)(this->pGPIOHandle.pGPIOx->IDR);

	return value;
}

void GPIO::GPIO_WriteToOuputPin(uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		this->pGPIOHandle.pGPIOx->ODR |= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
	}else
	{
		this->pGPIOHandle.pGPIOx->ODR &= ~(1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
	}
}

void GPIO::GPIO_WriteToOutputPort(uint16_t Value)
{
	this->pGPIOHandle.pGPIOx->ODR = (uint32_t)Value;
}

void GPIO::GPIO_ToggleOutputPin()
{
	this->pGPIOHandle.pGPIOx->ODR ^= (1 << this->pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
}

/*
 * IRQ config and ISR handling
 * */
void GPIO::GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 63)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 95)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 63)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 95)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO::GPIO_IRQPriorityConfig(uint32_t Priority, uint8_t PinNumber)
{
	uint8_t ipr = PinNumber / 4;
	uint8_t ipr_section = PinNumber % 4;

	uint8_t shift_amout = ((8 * ipr_section) + NO_PR_BITS);

	*(NVIC_IPR + ipr) |= (Priority << shift_amout);
}

void GPIO::GPIO_ISRHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}


