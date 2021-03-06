/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2017 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements some simple busy timers.  They are designed to be
 * portable, not performant.
 * TIM2 is appropriated for usage.
 */
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>



static void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM2);
	/* microsecond counter */
	timer_set_prescaler(TIM2, rcc_apb1_frequency / 500000 - 1);
	timer_set_period(TIM2, 0xffff);
	timer_one_shot_mode(TIM2);
}

static void delay_us(uint16_t us)
{
	TIM_ARR(TIM2) = us;
	TIM_EGR(TIM2) = TIM_EGR_UG;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;
	//timer_enable_counter(TIM2);
	while (TIM_CR1(TIM2) & TIM_CR1_CEN);
}


