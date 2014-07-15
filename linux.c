/* Copyright (c) 2014 Thomas Kuehnel
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

/* Firmware to use with the serial 802.15.4 driver for linux */

#include "board.h"
#include "radio.h"
#include "transceiver.h"
#include "ioutil.h"
#include "hif.h"
#include <avr/eeprom.h>
#include "linux.h"

#define SERIAL_HEADER 4
#define RX_BUF_CNT 2

static uint8_t buf[MAX_FRAME_SIZE + SERIAL_HEADER]; // general purpose buffer
uint8_t rx_buf[SERIAL_HEADER + MAX_FRAME_SIZE]; // for data receiving interrupt
uint8_t rx_read_buf[RX_BUF_CNT][sizeof(buffer_t) + MAX_FRAME_SIZE]; // data for pbuf
buffer_t *pbuf[RX_BUF_CNT]; //received frames

volatile uint8_t rx_block;
volatile bool tx_pending;

static int getc_wait(void)
{
	int inchar;

	do {
		inchar = hif_getc();
	} while (inchar == EOF);
	return inchar;
}

int main(void)
{
	int i, j, k, buf_n = 0;

	/* This will stop the application before initializing the radio
	 * transceiver (ISP issue with MISO pin, see FAQ)
	 */
	trap_if_key_pressed();

	/* Step 0: init MCU peripherals */
	KEY_INIT();
	hif_init(HIF_DEFAULT_BAUDRATE);

	LED_INIT();
	LED_SET_VALUE(0);
	TIMER_INIT();


	for (i = 0; i < RX_BUF_CNT; i++)
		/* Initialize buffer structure */
		pbuf[i] = buffer_init(rx_read_buf[i], sizeof(rx_read_buf[i]), 0);


	radio_init(rx_buf, MAX_FRAME_SIZE);

	radio_set_param(RP_IDLESTATE(STATE_RXAUTO));
	radio_set_param(RP_CHANNEL(17));
	radio_set_param(RP_SHORTADDR(0x1234));
	radio_set_param(RP_PANID(0x5678));
	radio_set_state(STATE_RXAUTO);
	sei();

	/* Step 3: Going to receive frames */
	buf_n = 0;
	while (1) {
		/* Try to listen for incoming commands after sending data to the host */
		for (; !rx_block && buf_n < RX_BUF_CNT; buf_n++) {
			if (BUFFER_IS_LOCKED(pbuf[buf_n]) == true) {
				hif_printf(FLASH_STRING("zb"));
				hif_putc(DATA_RECV_BLOCK);
				hif_putc(buffer_get_char(pbuf[buf_n]));
				k = buffer_get_char(pbuf[buf_n]);
				hif_putc(k);
				for (j = 0; j < k; j++)
					hif_putc(buffer_get_char(pbuf[buf_n]));
				BUFFER_RESET(pbuf[buf_n], 0);
				BUFFER_SET_UNLOCK(pbuf[buf_n]);
				rx_block = 8;
				break;
			}
		}
		buf_n = buf_n % RX_BUF_CNT;

		if (hif_getc() == 'z' && getc_wait() == 'b') {
			/* read first byte non blocking */
			switch (getc_wait()) {
			case CMD_OPEN:
				eeprom_read_block(buf, (void *)0x1F66, 8);
				for (i = 0; i < 8; i++)
					trx_reg_write(RG_IEEE_ADDR_7 + i, buf[i]);
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_OPEN);
				hif_putc(STATUS_SUCCESS);
				break;
			case CMD_CLOSE:
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_CLOSE);
				hif_putc(STATUS_SUCCESS);
				break;
			case CMD_SET_CHANNEL:
				trx_bit_write(SR_CHANNEL, getc_wait() + 10);
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_SET_CHANNEL);
				hif_putc(STATUS_SUCCESS);
				break;
			case CMD_ED:
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_ED);
				hif_putc(STATUS_ERR);
				break;
			case DATA_XMIT_BLOCK: {
				int len = getc_wait();

				if (len > MAX_DATA_SIZE) {
					hif_printf(FLASH_STRING("zb"));
					hif_putc(RESP_XMIT_BLOCK);
					hif_putc(STATUS_ERR);
					break;
				}
				for (i = 0; i < len; i++)
					buf[i] = getc_wait();
				LED_SET(1);
				tx_pending = true;
				radio_set_state(STATE_TXAUTO);
				radio_send_frame(len + 2, buf, 1);
				/* 2 bytes for automagically calculated crc */
				while (tx_pending);
				break;
			}
			case CMD_ADDRESS:
				eeprom_read_block(buf, (void *)0x1F66, 8);
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_ADDRESS);
				hif_putc(STATUS_SUCCESS);
				for (i = 7; i >= 0; i--)
					hif_putc(buf[i]);
				break;
			case CMD_SET_PAN_ID:
				trx_reg_write(RG_PAN_ID_0, getc_wait());
				trx_reg_write(RG_PAN_ID_1, getc_wait());
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_SET_PAN_ID);
				hif_putc(STATUS_SUCCESS);
				break;
			case CMD_SET_SHORT_ADDRESS:
				trx_reg_write(RG_SHORT_ADDR_0, getc_wait());
				trx_reg_write(RG_SHORT_ADDR_1, getc_wait());
				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_SET_SHORT_ADDRESS);
				hif_putc(STATUS_SUCCESS);
				break;
			case CMD_SET_LONG_ADDRESS:
				for (i = 0; i < 8; i++)
					trx_reg_write(RG_IEEE_ADDR_7 - i, getc_wait());

				hif_printf(FLASH_STRING("zb"));
				hif_putc(RESP_SET_LONG_ADDRESS);
				hif_putc(STATUS_SUCCESS);
				break;
			default:
				break;
			}

		}
	}
}

void usr_radio_tx_done(radio_tx_done_t status)
{
    LED_CLR(1);

    hif_printf(FLASH_STRING("zb"));
    hif_putc(RESP_XMIT_BLOCK);
    switch (status) {
    case TX_OK:
	    hif_putc(STATUS_SUCCESS);
	    break;
    default:
	    hif_putc(STATUS_ERR);
    }
    tx_pending = false;
}

uint8_t *usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi,
				 int8_t ed, uint8_t crc)
{
	static uint8_t i = 0;
	uint8_t __sreg = SREG;

	cli();

	LED_SET(0);
	for (; i < RX_BUF_CNT; i++) {
		if (BUFFER_IS_LOCKED(pbuf[i]) == false && crc == 0) {
			buffer_append_char(pbuf[i], lqi);
			buffer_append_char(pbuf[i], len - 2);
			buffer_append_block(pbuf[i], frm, len - 2);
			BUFFER_SET_LOCK(pbuf[i]);
			break;
		}
	}
	i %= RX_BUF_CNT;

	SREG = __sreg;
	LED_CLR(0);
	return frm;
}

ISR(TIMER_IRQ_vect)
{
	if (rx_block > 0)
		rx_block--;
}
