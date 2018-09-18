#ifndef LIBSDIO_H_
#define LIBSDIO_H_

#include "api/types.h"

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

#endif /*!LIBSDIO_H_*/
