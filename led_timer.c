/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    led_timer.c
 * @brief   Application entry point.
 */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
#define limite 0xff
#define limit 3
#define segundo 42000000

uint8_t state = 1;
bool stop = true;
uint8_t contador = 0;

void verde();
void azul();
void rojo();

/*arreglo de funciones para prender LEDs*/
void (*Funciones[3])()= { &rojo, &verde, &azul };

/*!
 	 \brief	 This is the interrupt services routing (ISR) for
 	 the SW3
 	 Changes the direction of the loop for the colors
 	 \param[in]  void.
 	 \return void
 */
	void PORTA_IRQHandler()
	{
		PORT_ClearPinsInterruptFlags(PORTA, 1 << 4);
		state = (0 == state) ? 1 : 0;
	}

	/*!
	 	 \brief	 This is the interrupt services routing (ISR) for
	 the SW2
	 Disables and enables the PIT interruptions
	 	 \param[in]  void.
	 	 \return void
	 */
	void PORTC_IRQHandler()
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1 << 6);
		if (false == stop)
		{
			EnableIRQ(PIT0_IRQn);
			stop = !stop;
		} else if (true == stop)
		{
			DisableIRQ(PIT0_IRQn);
			stop = !stop;
		}

	}

	/*!
	 	 \brief	 This is the interrupt services routing (ISR) of
	 	 the PIT0
	 	 Loop for both directions for LED lighting
	 	 \param[in]  void.
	 	 \return void
	 */
	void PIT0_IRQHandler()
	{
		PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
		Funciones[contador]();
		if (1 == state)
		{
			contador++;
			contador = (limit <= contador) ? 0 : contador;
		} else if (0 == state)
		{
			contador--;
			contador = (limite == contador) ? 2 : contador;
		}
	}

	int main(void)
	{

		/* Init board hardware. */
		BOARD_InitBootPins();
		BOARD_InitBootClocks();
		BOARD_InitBootPeripherals();
		/* Init FSL debug console. */
		BOARD_InitDebugConsole();

		CLOCK_EnableClock(kCLOCK_PortB);
		CLOCK_EnableClock(kCLOCK_PortA);
		CLOCK_EnableClock(kCLOCK_PortC);
		CLOCK_EnableClock(kCLOCK_PortE);

		port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
				kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
				kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

		PORT_SetPinConfig(PORTB, 21, &config_led);  //Blue
		PORT_SetPinConfig(PORTB, 22, &config_led);	//Red
		PORT_SetPinConfig(PORTE, 26, &config_led);	//Green

		port_pin_config_t config_switch = { kPORT_PullDisable,
				kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
				kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
				kPORT_UnlockRegister };

		PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
		PORT_SetPinConfig(PORTA, 4, &config_switch);

		PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);
		PORT_SetPinConfig(PORTC, 6, &config_switch);

		gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };

		GPIO_PinInit(GPIOB, 21, &led_config_gpio);
		GPIO_PinInit(GPIOB, 22, &led_config_gpio);
		GPIO_PinInit(GPIOE, 26, &led_config_gpio);

		gpio_pin_config_t switch_config_gpio = { kGPIO_DigitalInput, 1 };

		GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
		GPIO_PinInit(GPIOC, 6, &switch_config_gpio);

		NVIC_EnableIRQ(PORTA_IRQn);
		NVIC_EnableIRQ(PORTC_IRQn);

		pit_config_t pit_config = {	true };

		PIT_Init(PIT, &pit_config);

		PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
		EnableIRQ(PIT0_IRQn);

		PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, segundo);

		PIT_StartTimer(PIT, kPIT_Chnl_0);

		while (1) {	}

		return 0;
	}

	/*!
		 	 \brief	 This is the function for green LED color
		 	 \param[in]  void.
		 	 \return void
		 */
	void verde()
	{
		GPIO_WritePinOutput(GPIOE, 26, 0);
		GPIO_WritePinOutput(GPIOB, 22, 1);
		GPIO_WritePinOutput(GPIOB, 21, 1);
	}

	/*!
		 	 \brief	 This is the function for red LED color
		 	 \param[in]  void.
		 	 \return void
		 */
	void rojo()
	{
		GPIO_WritePinOutput(GPIOE, 26, 1);
		GPIO_WritePinOutput(GPIOB, 22, 0);
		GPIO_WritePinOutput(GPIOB, 21, 1);
	}

	/*!
		 	 \brief	 This is the function for blue LED color
		 	 \param[in]  void.
		 	 \return void
		 */
	void azul()
	{
		GPIO_WritePinOutput(GPIOE, 26, 1);
		GPIO_WritePinOutput(GPIOB, 22, 1);
		GPIO_WritePinOutput(GPIOB, 21, 0);
	}
