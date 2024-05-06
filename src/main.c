/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include "usb.h"
#include "usart.h"

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();


	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART2);
}

static void gpio_setup(void)
{
	/* Set GPIO12 (in GPIO port C) to 'output push-pull'. */

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

	gpio_clear(GPIOA, GPIO12);

}

int main(void)
{
	// rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE]);

	/*
	 * This is a somewhat common cheap hack to trigger device re-enumeration
	 * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
	 * setting the pin to output, and driving it explicitly low effectively
	 * "removes" the pullup.  The subsequent USB init will "take over" the
	 * pin, and it will appear as a proper pullup to the host.
	 * The magic delay is somewhat arbitrary, no guarantees on USBIF
	 * compliance here, but "it works" in most places.
	 */

	clock_setup();
	gpio_setup();
	usart_setup();

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	while (1){
		usbd_poll(usbd_dev);
		}
}

uint8_t count=0;
bool flag=1;
uint8_t buf_in[2];

void sys_tick_handler(void)
{	
	uint8_t buf_keyboard[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	

	if(count){
	buf_keyboard[2] = buf_in[0];
	buf_keyboard[0] = buf_in[1];
	count--;
	}
	usbd_ep_write_packet(usbd_dev, 0x81, buf_keyboard, 8);
	
}

void usart2_isr(void){

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		if(flag){
		buf_in[0] = usart_recv(USART2);
		if(buf_in[0])flag=0;else flag=1;
		}else{
		    buf_in[1] = usart_recv(USART2);
		    flag=1;
		}
		
		count=10;

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART2) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART2, buf_in[flag]);


		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART2) &= ~USART_CR1_TXEIE;


	}
}
