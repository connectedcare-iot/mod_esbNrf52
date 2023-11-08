#include <stdint.h>
#include <string.h>
#include "rf_test.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_gpio.h"
#include "io_driver.h"
#include "nrf_delay.h"
#include "esb_timer_driver.h"


/** @brief Function for enabling clock.
 */
static void initClock(void)
{
	NRF_RNG->TASKS_START = 1;
	
	// Start 16 MHz crystal oscillator
	NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
	NRF_CLOCK->TASKS_HFCLKSTART     = 1;

	// Wait for the external oscillator to start up
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	{
		// Do nothing.
	}
}


/**@brief Function for disabling the radio.
 */
static void disableRadio(void)
{
	NRF_RADIO->SHORTS          = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
#ifdef NRF51
	NRF_RADIO->TEST            = 0;
#endif
	NRF_RADIO->TASKS_DISABLE   = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0)
	{
		// Do nothing.
	}
	NRF_RADIO->EVENTS_DISABLED = 0;
}


/**@brief Function for turning on the TX carrier test mode.
 *
 * @param channel RF channel.
 */
static void setRadioTxCarrier(uint8_t channel)
{
	disableRadio();
	NRF_RADIO->SHORTS    = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);    
	NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
	NRF_RADIO->FREQUENCY = channel;
#ifdef NRF51
	NRF_RADIO->TEST      = (RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos) \
	                     | (RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
#endif
	NRF_RADIO->TASKS_TXEN = 1;
}


/**@brief RF test handler.
 */
static void rfTestHandler(void)
{
	bool loop = true;
	setRadioTxCarrier(40);
	
	while(loop)
	{
#ifndef RF_TEST_ONLY
		if (!nrf_gpio_pin_read(TXD_PIN_NO) &&
		    !nrf_gpio_pin_read(RXD_PIN_NO))
		{
			loop = false;
		}
#endif // RF_TEST_ONLY
	}
	
	disableRadio();
	nrf_gpio_pin_clear(BUTTON_LED);
	nrf_gpio_pin_clear(FCT_LED);
	nrf_delay_ms(500);
	NVIC_SystemReset();
}


/**@brief Function for handling timer callbacks.
 */
static void timerCallback(void)
{
	static uint8_t  currentChannel = 40;
	static uint32_t buttonCounter  = 0;
	static uint32_t flashCounter   = 0;
	static uint32_t toggleCounter  = 0;
	static uint32_t oldButtonValue = 0;
	static uint8_t  toggleCount    = 0;
	
	if (buttonCounter >= 25)
	{
		uint32_t buttonValue = io_getButtonValue();
		if (buttonValue != oldButtonValue)
		{
			if (buttonValue == 0x0001)
			{
				setRadioTxCarrier(3);
				currentChannel = 3;
			}
			else if (buttonValue == 0x0002)
			{
				setRadioTxCarrier(40);
				currentChannel = 40;
			}
			else if (buttonValue == 0x0004)
			{
				setRadioTxCarrier(80);
				currentChannel = 80;
			}
			oldButtonValue = buttonValue;
		}
		buttonCounter = 0;
	}
	buttonCounter++;
	
	if (toggleCounter >= 250)
	{
		if (toggleCount != 0)
		{
			nrf_gpio_pin_toggle(FCT_LED);
			toggleCount--;
		}
		toggleCounter = 0;
	}
	toggleCounter++;
	
	if (flashCounter >= 3000)
	{
		if (currentChannel == 3)
		{
			toggleCount   = 2;
			toggleCounter = 0;
		}
		else if (currentChannel == 40)
		{
			toggleCount   = 4;
			toggleCounter = 0;
		}
		else if (currentChannel == 80)
		{
			toggleCount   = 6;
			toggleCounter = 0;
		}
		flashCounter = 0;
	}
	flashCounter++;
}


void rfTest_init(void)
{
	//Inputs
	nrf_gpio_cfg_input(TXD_PIN_NO, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(RXD_PIN_NO, NRF_GPIO_PIN_PULLDOWN);
	
#ifndef RF_TEST_ONLY
	if(!nrf_gpio_pin_read(TXD_PIN_NO) && 
	   !nrf_gpio_pin_read(RXD_PIN_NO))
	{
		return;
	}
#endif // RF_TEST_ONLY
	
	initClock();
	io_init();
	timer_msTimerInit(timerCallback);
	rfTestHandler();
}
