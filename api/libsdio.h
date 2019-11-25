#ifndef LIBSDIO_H_
#define LIBSDIO_H_

#include "libc/types.h"

/* FIXME should not be exported */
#define r_CORTEX_M_SDIO_CMD		REG_ADDR(0x40012c0c)

#define SDIO_FLAG_CCRCFAIL 0x00000001U
#define SDIO_FLAG_DCRCFAIL 0x00000002U
#define SDIO_FLAG_CTIMEOUT 0x00000004U
#define SDIO_FLAG_DTIMEOUT 0x00000008U
#define SDIO_FLAG_TXUNDERR 0x00000010u
#define SDIO_FLAG_RXOVERR  0X00000020u
#define SDIO_FLAG_CMDREND  0x00000040U
#define SDIO_FLAG_CMDSENT  0X00000080u
#define SDIO_FLAG_DATAEND  0X00000100u
#define SDIO_FLAG_STBITERR 0X00000200u
#define SDIO_FLAG_DBCKEND  0X00000400u
#define SDIO_FLAG_CMDACT   0X00000800u
#define SDIO_FLAG_TXACT    0X00001000u
#define SDIO_FLAG_RXACT    0X00002000u

#define SDIO_FLAG_CMD_RESP (SDIO_FLAG_CTIMEOUT|SDIO_FLAG_CCRCFAIL|SDIO_FLAG_CMDREND)
#define SDIO_FLAG_CMD_NORESP (SDIO_FLAG_CTIMEOUT|SDIO_FLAG_CCRCFAIL|SDIO_FLAG_CMDSENT)
#define SDIO_FLAG_DATA (SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DATAEND|SDIO_FLAG_DBCKEND)

/* SDIO datasheet, timeout is classless defined, 100ms for read access,
 * 250ms for write access for all high speed class SDCard */
#define SDIO_READ_TIMEOUT   5000000
#define SDIO_WRITE_TIMEOUT 12500000



enum sdio_direction { OUT = 0, IN };
/*
*  MMC card can send two types of response 48BIT(short) and 138BITS(LONG)
*  SD and SDIO card can only send short responses!
*/
enum sdio_resp_type { NO_RESP = 0, SHORT_RESP = 1, LONG_RESP = 3};

uint8_t sdio_early_init(void);

uint8_t sdio_init(void);

void sdio_set_timeout(uint32_t timeout);

int sdio_finished_or_error(void);

void sdio_set_irq_handler(uint32_t (*ptr)());

uint32_t    sdio_getflags(uint32_t mask);

uint32_t    sdio_clear_flag(uint32_t mask);

/**
 * sdio_hw_send_cmd - Send a command to the SD card
 * @cmd: Command index
 * @arg: Argument of the command
 * @response: Expected response type
 *
 * Return: This function returns 0 in case of success, -1 otherwise.
 */
int8_t      sdio_hw_send_cmd(uint8_t cmd, uint32_t arg,
                             enum sdio_resp_type response);

/**
 * sdio_hw_get_short_resp - Get the short response of a command
 * Return: The status from the short response of a command that expects a short
 * response
 */
uint32_t    sdio_hw_get_short_resp(void);

/**
 * sdio_hw_get_long_resp - Get the long response of a command
 * @buffer: Address of the buffer in which the card status will be written. The
 * size of the buffer must be at least 16 bytes.
 */
void        sdio_hw_get_long_resp(volatile void *buffer);

/**
 * sdio_hw_set_bus_width - Set the number of wires in the data bus
 * @value: Number of data wires: 1 or 4.
 */
void        sdio_hw_set_bus_width(uint8_t value);

void        sdio_wait_for(uint32_t msk);

/**
 * sdio_hw_set_clock_divisor - Set the divisor use to generate the CLK signal
 * @divisor: New divide factor between the input clock of the HW block and the
 * output clock (CLK signal) which drives communications with the SD card.
 */
void        sdio_hw_set_clock_divisor(int32_t divisor);

void        sdio_launch_dma();
void        sdio_prepare_dma(uint32_t timeout, uint32_t bytes_len,
                             uint32_t blocksize);

void        sdio_prepare_nodma(uint32_t timeout, uint32_t bytes_len,
                             uint32_t blocksize);
/* set custom SDIO timeout (beware when using timeout values, which may be
 * too short for valid read, erase or write access, or too long for
 * correct error handling */
void        sdio_set_timeout(uint32_t timeout);

void        sdio_set_irq_handler(uint32_t(*ptr) ());

/**
 * sdio_get_data_addr - Getter for the SDIO FIFO address
 * Return: The address of the SDIO FIFO address where the data should be read
 * or written.
 */
volatile uint32_t *sdio_get_data_addr(void);

void sdio_hw_write_fifo(uint32_t * buf, uint32_t size);

#endif /*!LIBSDIO_H_*/
