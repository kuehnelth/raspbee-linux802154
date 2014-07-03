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
#include "transceiver.h"
#include "ioutil.h"
#include "hif.h"
#include <avr/eeprom.h>
#include "linux.h"

#define TRX_IDLE 0
#define TRX_TX   1
#define TRX_RX   2
#define TRX_CMD  3

static uint8_t rxfrm[MAX_FRAME_SIZE];
static uint8_t buf[0xFF];
static uint8_t rx_buf[0x102];
static uint8_t rx_len;
static uint8_t state;

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
	trx_regval_t rval;
	int i;

	/* This will stop the application before initializing the radio
	 * transceiver (ISP issue with MISO pin, see FAQ)
	 */
	trap_if_key_pressed();

	/* Step 0: init MCU peripherals */
	hif_init(HIF_DEFAULT_BAUDRATE);

	LED_INIT();
	trx_io_init(SPI_RATE_1_2);
	LED_SET_VALUE(0);

	/* Step 1: initialize the transceiver */
	TRX_RESET_LOW();
	TRX_SLPTR_LOW();
	DELAY_US(TRX_RESET_TIME_US);
	TRX_RESET_HIGH();
	trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);
	DELAY_US(TRX_INIT_TIME_US);
	rval = trx_bit_read(SR_TRX_STATUS);

	/* Step 2: setup transmitter
	 * - configure radio channel
	 * - go into RX state,
	 * - enable "receive end" IRQ
	 */
	trx_bit_write(SR_CHANNEL, 1);
	trx_bit_write(SR_TX_AUTO_CRC_ON, 1);
	trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
#if defined(TRX_IRQ_TRX_END)
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_TRX_END);
#elif defined(TRX_IRQ_RX_END) && defined(TRX_IRQ_TX_END)
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_RX_END | TRX_IRQ_TX_END |
		      TRX_IRQ_RX_START);
#else
#  error "Unknown IRQ bits"
#endif
	sei();
#if HIF_TYPE == HIF_AT90USB
	/*
	 * Wait for terminal user pressing a key so there is time to
	 * attach a terminal emulator after the virtual serial port has
	 * been established within the host OS.
	 */
	do {
		inchar = hif_getc();
	} while (EOF == inchar);
#endif

	/* Step 3: Going to receive frames */
	state = TRX_IDLE;
	rx_len = 0;
	while (1) {
		/*while (state != TRX_IDLE)
		  WAIT_MS(1);

		if (rx_len > 0) {
			//trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);
			hif_printf(FLASH_STRING("zb"));
			hif_putc(DATA_RECV_BLOCK);
			for (i = 0; i < rx_len; i++)
				hif_putc(rx_buf[i]);
			rx_len = 0;
			//trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);

		}
		*/
		if (getc_wait() == 'z' && getc_wait() == 'b') {
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
				state = TRX_TX;
				trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
				trx_frame_write(len + 2, buf);
				/* 2 bytes for automagically calculated crc */
				TRX_SLPTR_HIGH();
				TRX_SLPTR_LOW();
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

ISR(TRX24_RX_END_vect)
{
	uint8_t *pfrm, flen, lqi, i;
	bool crc_ok;

	LED_CLR(0);
	pfrm = rxfrm;
	flen = trx_frame_read_data_crc(pfrm, sizeof(rxfrm), &lqi, &crc_ok);

	/* send data_recv_block */
	
	hif_printf(FLASH_STRING("zb"));
	hif_putc(DATA_RECV_BLOCK);
	hif_putc(lqi);
	hif_putc(flen - 2);
	
	rx_buf[0] = lqi;
	rx_buf[1] = flen - 2;

	for (i = 0; i < flen - 2; i++)
		hif_putc(pfrm[i]);
	//rx_buf[i + 2] = pfrm[i];
	rx_len = flen;
	state = TRX_IDLE;
}

ISR(TRX24_RX_START_vect)
{
	LED_SET(0);
	state = TRX_RX;
}

ISR(TRX24_TX_END_vect)
{
	static volatile trx_regval_t trac_status;

	trac_status = trx_bit_read(SR_TRAC_STATUS);
	trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
	LED_CLR(1);
	PRINT_START_BYTES();//hif_printf(FLASH_STRING("zb"));
	hif_putc(RESP_XMIT_BLOCK);
	if (trac_status == TRAC_SUCCESS)
		hif_putc(STATUS_SUCCESS);
	else
		hif_putc(STATUS_ERR);
	state = TRX_IDLE;
}
