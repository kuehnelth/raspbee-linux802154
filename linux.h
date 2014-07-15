#ifndef LINUX_H
#define LINUX_H

#define START_BYTE1	'z'
#define START_BYTE2	'b'
#define MAX_DATA_SIZE	127

#define TIMEOUT 1000

#define STATUS_SUCCESS	0
#define STATUS_RX_ON	1
#define STATUS_TX_ON	2
#define STATUS_TRX_OFF	3
#define STATUS_IDLE	4
#define STATUS_BUSY	5
#define STATUS_BUSY_RX	6
#define STATUS_BUSY_TX	7
#define STATUS_ERR	8

/*
 * The following messages are used to control ZigBee firmware.
 * All communication has request/response format,
 * except of asynchronous incoming data stream (DATA_RECV_* messages).
 */
enum {
	NO_ID			= 0, /* means no pending id */

	/* Driver to Firmware */
	CMD_OPEN		= 0x01, /* u8 id */
	CMD_CLOSE		= 0x02, /* u8 id */
	CMD_SET_CHANNEL		= 0x04, /* u8 id, u8 channel */
	CMD_ED			= 0x05, /* u8 id */
	CMD_SET_STATE		= 0x07, /* u8 id, u8 flag */
	DATA_XMIT_BLOCK		= 0x09, /* u8 id, u8 len, u8 data[len] */
	RESP_RECV_BLOCK		= 0x0b, /* u8 id, u8 status */
	CMD_ADDRESS		= 0x0d, /* u8 id */
	CMD_SET_PAN_ID		= 0x0f, /* u8 id, u8 u8 panid (MSB first) */
	CMD_SET_SHORT_ADDRESS	= 0x10, /* u8 id, u8 u8 address  (MSB first)*/
	CMD_SET_LONG_ADDRESS	= 0x11, /* u8 id, u8 u8 u8 u8 u8 u8 u8 u8 address (MSB first) */

	/* Firmware to Driver */
	RESP_OPEN		= 0x81, /* u8 id, u8 status */
	RESP_CLOSE		= 0x82, /* u8 id, u8 status */
	RESP_SET_CHANNEL	= 0x84, /* u8 id, u8 status */
	RESP_ED			= 0x85, /* u8 id, u8 status, u8 level */
	RESP_SET_STATE		= 0x87, /* u8 id, u8 status */
	RESP_XMIT_BLOCK		= 0x89, /* u8 id, u8 status */
	DATA_RECV_BLOCK		= 0x8b, /* u8 id, u8 lq, u8 len, u8 data[len] */
	RESP_ADDRESS		= 0x8d, /* u8 id, u8 status, u8 u8 u8 u8 u8 u8 u8 u8 address */
	RESP_SET_PAN_ID		= 0x8f, /* u8 id, u8 status */
	RESP_SET_SHORT_ADDRESS	= 0x90, /* u8 id, u8 status */
	RESP_SET_LONG_ADDRESS	= 0x91, /* u8 id, u8 status */
};

static inline void WAIT_MS(uint16_t t)
{
    while (t--) DELAY_MS(1);
}

#endif
