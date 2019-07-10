#ifndef SDIO_H
#define  SDIO_H

#include "libc/types.h"

enum sd_error { SD_CMD_CRC_FAIL, SD_DATA_CRC_FAIL, SD_CMD_TIMEOUT,
    SD_DATA_TIMEOUT
};

struct sd_callbacks {
    void    (*response_received)(uint8_t cmd);
    void    (*command_sent)(uint8_t cmd);
    void    (*error)(enum sd_error);
    void    (*fifo_empty)(uint8_t free_words);
    void    (*fifo_full)(uint8_t available_words);
    void    (*transfer_complete)(void);
};

/**
 * sdio_hw_init - Init the SDIO driver
 * @callbacks: List of callbacks called by the driver when an event occurs.
 */
void    sdio_hw_init(struct sd_callbacks *callbacks);

/**
 * sdio_hw_enable_data_transfer - Start a transfer of data
 * @timeout: Data timeout period, in card bus clock period.
 * @bytes_len: Number of bytes to transfer.
 * @dir: Direction of the transfer: OUT (write) or IN (read).
 */
void    sdio_hw_enable_data_transfer(uint32_t timeout, uint32_t bytes_len,
                                     enum sdio_direction dir);
/**
 * sdio_hw_write_fifo - Write data in the SDIO hardware block FIFO
 * @buf: Address of the buffer which contains the data. The size of the buffer
 * must be at least @size.
 * @size: Number of words (32 bits) to write in the FIFO.
 */
void    sdio_hw_write_fifo(uint32_t * buf, uint32_t size);

/**
 * sdio_hw_read_fifo - Read data in the SDIO hardware block FIFO
 * @buf: Address of the buffer in which data will be written The size of the
 * buffer must be at least @size.
 * @size: Number of words (32 bits) to read.
 */
void    sdio_hw_read_fifo(uint32_t * buf, uint32_t size);

void    sdio_wait_for_cmd_completion();
void    sdio_set_response_callback(void (*sd_response_received)(uint8_t));
int     sdio_finished_or_error();
void    sdio_hw_init2();

#endif /* !SDIO_H */
