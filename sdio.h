#ifndef SDIO_H
#define  SDIO_H

#include "api/types.h"

enum sd_error { SD_CMD_CRC_FAIL, SD_DATA_CRC_FAIL, SD_CMD_TIMEOUT,
        SD_DATA_TIMEOUT };
struct sd_callbacks {
    void        (*response_received) (uint8_t cmd);
    void        (*command_sent) (uint8_t cmd);
    void        (*error) (enum sd_error);
    void        (*fifo_empty) (uint8_t free_words);
    void        (*fifo_full) (uint8_t available_words);
    void        (*transfer_complete) (void);
};

/**
 * sdio_hw_init - Init the SDIO driver
 * @callbacks: List of callbacks called by the driver when an event occurs.
 */
void        sdio_hw_init(struct sd_callbacks *callbacks);

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
 * sdio_hw_set_clock_divisor - Set the divisor use to generate the CLK signal
 * @divisor: New divide factor between the input clock of the HW block and the
 * output clock (CLK signal) which drives communications with the SD card.
 */
void        sdio_hw_set_clock_divisor(int32_t divisor);

/**
 * sdio_hw_set_bus_width - Set the number of wires in the data bus
 * @value: Number of data wires: 1 or 4.
 */
void        sdio_hw_set_bus_width(uint8_t value);

/**
 * sdio_hw_enable_data_transfer - Start a transfer of data
 * @timeout: Data timeout period, in card bus clock period.
 * @bytes_len: Number of bytes to transfer.
 * @dir: Direction of the transfer: OUT (write) or IN (read).
 */
void        sdio_hw_enable_data_transfer(uint32_t timeout, uint32_t bytes_len,
                                         enum sdio_direction dir);

/**
 * sdio_hw_write_fifo - Write data in the SDIO hardware block FIFO
 * @buf: Address of the buffer which contains the data. The size of the buffer
 * must be at least @size.
 * @size: Number of words (32 bits) to write in the FIFO.
 */
void        sdio_hw_write_fifo(uint32_t * buf, uint32_t size);

/**
 * sdio_hw_read_fifo - Read data in the SDIO hardware block FIFO
 * @buf: Address of the buffer in which data will be written The size of the
 * buffer must be at least @size.
 * @size: Number of words (32 bits) to read.
 */
void        sdio_hw_read_fifo(uint32_t * buf, uint32_t size);

/**
 * sdio_get_data_addr - Getter for the SDIO FIFO address
 * Return: The address of the SDIO FIFO address where the data should be read
 * or written.
 */
volatile uint32_t *sdio_get_data_addr(void);

void        sdio_wait_for_cmd_completion();
void        sdio_set_response_callback(void (*sd_response_received) (uint8_t));

uint32_t    sdio_getflags(uint32_t mask);
uint32_t    sdio_clear_flag(uint32_t mask);
int         sdio_finished_or_error();
void        sdio_hw_init2();
void        sdio_launch_dma();
void        sdio_prepare_dma(uint32_t timeout, uint32_t bytes_len,
                             uint32_t blocksize);
void        sdio_set_timeout(uint32_t timeout);
void        sdio_set_irq_handler(uint32_t(*ptr) ());
void        sdio_wait_for(uint32_t msk);

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
#endif                          /* !SDIO_H */
