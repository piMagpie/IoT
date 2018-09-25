/**

* Copyright (c) 2009 - 2018, Nordic Semiconductor ASA

*

* All rights reserved.

*

* Redistribution and use in source and binary forms, with or without modification,

* are permitted provided that the following conditions are met:

*

* 1. Redistributions of source code must retain the above copyright notice, this

*    list of conditions and the following disclaimer.

*

* 2. Redistributions in binary form, except as embedded into a Nordic

*    Semiconductor ASA integrated circuit in a product or a software update for

*    such product, must reproduce the above copyright notice, this list of

*    conditions and the following disclaimer in the documentation and/or other

*    materials provided with the distribution.

*

* 3. Neither the name of Nordic Semiconductor ASA nor the names of its

*    contributors may be used to endorse or promote products derived from this

*    software without specific prior written permission.

*

* 4. This software, with or without modification, must only be used with a

*    Nordic Semiconductor ASA integrated circuit.

*

* 5. Any software provided in binary form under this license must not be reverse

*    engineered, decompiled, modified and/or disassembled.

*

* THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS

* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES

* OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE

* DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE

* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR

* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE

* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)

* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT

* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT

* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*

*/

/** @file

* @brief Example template project.

* @defgroup nrf_templates_example Example Template

*

*/

 

#include <stdbool.h>

#include <stdint.h>

 

#include "nrf.h"

#include "nordic_common.h"

#include "boards.h"

#include "nrf.h"                        // Device header

#include "RTE_Components.h"             // Component selection

#include "system_nrf52.h"               // NordicSemiconductor::Device:Startup

#include "nrf_timer.h"

#include "nrf_drv_timer.h"

#include "app_util_platform.h"

#include "sdk_common.h"

#include "nrf_gpio.h"

#include "nrf_temp.h"

#include "nrf_drv_gpiote.h"

 

// pins

#define BUTTON_PIN 13

 

// prototypes

void temp_timeout_handler(nrf_timer_event_t event_type, void* p_context);

void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

uint32_t init_timers();

uint32_t config_gpio();

 

// global variables

const nrf_drv_timer_t TIMER_TEMP = NRF_DRV_TIMER_INSTANCE(0);

// volatile is needed because of the interruptions

int32_t volatile temp;

 

int main(void)

{

 

                              // configuring the pins

                              uint32_t err_code1 = config_gpio();

                              uint32_t err_code = init_timers();

                             

                              while (true)

                              {

                                             // Do nothing.

                              }

                             

               // NordicSemiconductor::Device:Startup

 

                             

}

 

/**

* Function for reading on-die temperature

*/

void meas_temp()

{

               NRF_TEMP->TASKS_START = 1; // Start the temperature measurement

// Busy wait while temperature measurement is not finished

               while (NRF_TEMP->EVENTS_DATARDY == 0)

               {

               // Do nothing.

               }

               NRF_TEMP->EVENTS_DATARDY = 0;

// The value in the register is a multiple of 4

               temp = (nrf_temp_read() / 4);

               NRF_TEMP->TASKS_STOP = 1; // Stop the temperature measurement

}

 

/**

* Function for initializing the timer.

*/

/**

* Handler for timer events.
*/

void temp_timeout_handler(nrf_timer_event_t event_type, void* p_context)

{

               meas_temp();

}

/**

* Handler for GPIO events.

*/

void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)

{

               meas_temp();

}

 

 

 

uint32_t init_timers()

{

               uint32_t time_ms = 5000; //Time(in milliseconds) between consecutive compare events

               uint32_t time_ticks;

               uint32_t err_code = NRF_SUCCESS;

               //Configure TIMER_TEMP for measuring the temperature

              

               nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

               err_code = nrf_drv_timer_init(&TIMER_TEMP, &timer_cfg, temp_timeout_handler);

               APP_ERROR_CHECK(err_code);

               time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_TEMP, time_ms);

               nrf_drv_timer_extended_compare(&TIMER_TEMP, NRF_TIMER_CC_CHANNEL0, time_ticks,

               NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

               nrf_drv_timer_enable(&TIMER_TEMP);

              

               return err_code;

}

 

/**

* Function for configuring General Purpose I/O.

*/

uint32_t config_gpio()

{

uint32_t err_code = NRF_SUCCESS;

              

               if(!nrf_drv_gpiote_is_init())

               {

                              err_code = nrf_drv_gpiote_init();

               }

// Set which clock edge triggers the interrupt

               nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

// Configure the internal pull up resistor

               config.pull = NRF_GPIO_PIN_PULLUP;

// Configure the pin as input

               err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);

              

               if (err_code != NRF_SUCCESS)

               {

               // handle error condition

               }

// Enable events

nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);

              

               return err_code;

}

 

/** @} */