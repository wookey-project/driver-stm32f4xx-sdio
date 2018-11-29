#include "api/types.h"
#include "api/print.h"
#include "api/regutils.h"
#include "kernel/exported/devices.h"
#include "kernel/exported/dmas.h"
#include "api/libsdio.h"
#include "sdio.h"
#include "sdio_regs.h"
#include "api/syscall.h"

#ifdef DEBUG_SDIO_INTERRUPTS
uint32_t    log_status[20];
int         log_count;
void init_log_status(void)
{
    for (int i = 0; i < 20; i++) {
        log_status[i] = 0;
    }
    log_count = 0;
}

void add_log_status(uint32_t status)
{
    log_status[log_count] = status;
    log_count++;
}
#endif

/*
 * This lib requires:
 * BUS permission
 * DMA permission
 */
static const char *name = "sdio";

#define ENABLE_HW_FLOW_CONTROL
volatile uint32_t savestatus;

/* Command Path unit State Machine */
enum CPSM {
    CPSM_IDLE,
    CPSM_SEND,
    CPSM_WAIT_RECEIVE,          /*At our level, the WAIT and RECEIVE states are the same */
    CPSM_PEND
};
static volatile enum CPSM cmd_path_state;

enum DPSM {
    DPSM_IDLE,
    DPSM_WAIT_S,
    DPSM_SEND_BUSY,             /*At our level, SEND and BUSY states are the same */
    DPSM_WAIT_R,
    DPSM_RECEIVE,
    DPSM_READ_WAIT
};
static volatile enum DPSM data_path_state;

#if __GNUG__
# pragma GCC push_options
# pragma GCC optimize("O0")
#endif
#if __clang__
# pragma clang optimize off
#endif
void write_delay(void)
{
    for (int i = 0; i < 20; i++) {
        // FIXME PTR WTF ????
     // TODO: not GCC/LLVM portable   volatile asm ("nop\n\t" :::);
    }
}
#if __clang__
# pragma clang optimize on
#endif
#if __GNUG__
# pragma GCC pop_options
#endif



/***************************************
 * Command relative functions
 ***************************************/

int8_t sdio_hw_send_cmd(uint8_t cmd, uint32_t arg, enum sdio_resp_type response)
{
    uint32_t    tmp = 0;

    savestatus = 0;
#ifdef DEBUG_SDIO_INTERRUPTS
    init_log_status();
#endif
    write_reg_value(r_CORTEX_M_SDIO_ARG, arg);
    set_reg_bits(r_CORTEX_M_SDIO_MASK, SDIO_MASK_CMDACTIE_Msk);

    tmp |= (response << SDIO_CMD_WAITRESP_Pos) & SDIO_CMD_WAITRESP_Msk;
    tmp |= (cmd << SDIO_CMD_CMDINDEX_Pos) & SDIO_CMD_CMDINDEX_Msk;
    tmp |= 1 << SDIO_CMD_CPSMEN_Pos;

    write_delay();
    write_reg_value(r_CORTEX_M_SDIO_CMD, tmp);
    return 0;
}

void sdio_wait_for_cmd_completion()
{
    while (!
           ((savestatus) &
            (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND |
             SDIO_FLAG_CMDSENT))) ;
}

uint32_t sdio_hw_get_short_resp(void)
{
    return read_reg_value(r_CORTEX_M_SDIO_RESP(1));
}

void sdio_hw_get_long_resp(volatile void *buffer)
{
    uint8_t     i;
    volatile uint32_t *buf = buffer;

    for (i = 0; i < 4; i++)
        buf[i] = read_reg_value(r_CORTEX_M_SDIO_RESP(4 - i));
    buf[0] |= 1;
}

/***********************************************
 * SDIO block startup function
 ***********************************************/

static void power_up(void)
{
    uint64_t    i, ii = 0;
    set_reg(r_CORTEX_M_SDIO_POWER, SDIO_POWER_PWRCTRL_POWER_ON,
            SDIO_POWER_PWRCTRL);
    /*
     * SDIO_POWER documentation says:
     * At least seven HCLK clock periods are needed between two write accesses to this register.
     * After a data write, data cannot be written to this register for three SDIOCLK (48 MHz) clock
     * periods plus two PCLK2 clock periods.
     */

    sys_get_systick(&i, PREC_CYCLE);
    printf("debut %x\n", (uint32_t) i);
    for (; (i + 1680000) > ii; sys_get_systick(&ii, PREC_CYCLE)) ;
    printf("fin %x\n", (uint32_t) ii);
    //LOG("Power up !\n");
}

static void enabling_irqs(void)
{
    write_reg_value(r_CORTEX_M_SDIO_MASK, 0);
    set_reg_bits(r_CORTEX_M_SDIO_MASK, SDIO_MASK_CCRCFAILIE_Msk |
                 SDIO_MASK_DCRCFAILIE_Msk |
                 SDIO_MASK_CTIMEOUTIE_Msk |
                 SDIO_MASK_DTIMEOUTIE_Msk |
                 SDIO_MASK_TXUNDERRIE_Msk |
                 SDIO_MASK_RXOVERRIE_Msk |
                 SDIO_MASK_CMDRENDIE_Msk |
                 SDIO_MASK_CMDSENTIE_Msk |
                 SDIO_MASK_DATAENDIE_Msk |
                 SDIO_MASK_STBITERRIE_Msk |
                 SDIO_MASK_DBCKENDIE_Msk | SDIO_MASK_CMDACTIE_Msk);

    // made by kernel when executing sys_init(INIT_DONE):
    // NVIC_EnableIRQ(SDIO_IRQn);
    //LOG("Int enabled\n");
}

void sdio_hw_init2()
{
    uint32_t    tmp;
    cmd_path_state = CPSM_IDLE;
    data_path_state = DPSM_IDLE;

    //set_reg(r_CORTEX_M_SDIO_CLKCR, 150, SDIO_CLKCR_CLKDIV); //FIXME
    //set_reg(r_CORTEX_M_SDIO_CLKCR, SDIO_CLKCR_WIDBUS_1WIDE_MODE, SDIO_CLKCR_WIDBUS);
    //LOG("Clock set\n");

    tmp = 150 | SDIO_CLKCR_WIDBUS_1WIDE_MODE;
    power_up();

#ifdef ENABLE_HW_FLOW_CONTROL
//  set_reg_bits(r_CORTEX_M_SDIO_CLKCR, SDIO_CLKCR_HWFC_EN_Msk);
    tmp |= SDIO_CLKCR_HWFC_EN_Msk;
#endif
    //set_reg_bits(r_CORTEX_M_SDIO_CLKCR, SDIO_CLKCR_CLKEN_Msk);
    write_delay();
    write_reg_value(r_CORTEX_M_SDIO_CLKCR, tmp | SDIO_CLKCR_CLKEN_Msk);
    //LOG("Clock enabled\n");

    enabling_irqs();

    //LOG("Driver initialized\n");
}

/*
 * Clock and bus width setters
 */
void sdio_hw_set_clock_divisor(int32_t divisor)
{
    write_delay();
    if (divisor < 0) {
        set_reg_bits(r_CORTEX_M_SDIO_CLKCR, SDIO_CLKCR_BYPASS_Msk);
    }
    else {
        set_reg(r_CORTEX_M_SDIO_CLKCR, divisor, SDIO_CLKCR_CLKDIV);
    }
}

void sdio_hw_set_bus_width(uint8_t value)
{
    uint32_t    width;
    switch (value) {
    case 1:
        width = SDIO_CLKCR_WIDBUS_1WIDE_MODE;
        break;
    case 4:
        width = SDIO_CLKCR_WIDBUS_4WIDE_MODE;
        break;
    case 8:
        width = SDIO_CLKCR_WIDBUS_8WIDE_MODE;
        break;
    default:
        printf("Set Bus Width aborted: %d is an invalid value\n", value);
        return;
    }
    printf("Changing bus width to %d wires\n", value);
    write_delay();
    set_reg(r_CORTEX_M_SDIO_CLKCR, width, SDIO_CLKCR_WIDBUS);
}

/*****************************************
 * DMA relative functions
 *****************************************/
void sdio_prepare_dma(uint32_t timeout, uint32_t bytes_len, enum sdio_direction
                      __attribute__ ((unused)) dir)
{
    uint32_t    dctrl = *r_CORTEX_M_SDIO_DCTRL;

    write_reg_value(r_CORTEX_M_SDIO_DTIMER, timeout);
    set_reg(r_CORTEX_M_SDIO_DLEN, bytes_len, SDIO_DLEN_DATALENGTH);

    /* FIXME - (9<<4) means 512 bytes */
    write_reg_value(&dctrl,
                    (9 << 4) | SDIO_DCTRL_DMAEN_Msk | SDIO_DCTRL_DTMODE_BLOCK);
    set_reg(&dctrl, SDIO_DCTRL_DTMODE_BLOCK, SDIO_DCTRL_DTMODE);

    write_delay();
    write_reg_value(r_CORTEX_M_SDIO_DCTRL, dctrl);
}

void sdio_launch_dma(int i)
{
    write_delay();
    set_reg(r_CORTEX_M_SDIO_DCTRL, i, SDIO_DCTRL_DTDIR);
    write_delay();
    set_reg(r_CORTEX_M_SDIO_DCTRL, 1, SDIO_DCTRL_DTEN);
}

/* TODO: check bytes_len (block size aligned ?) */
/* TODO: data block size depending on the SD card */

void sdio_hw_enable_data_transfer(uint32_t timeout, uint32_t bytes_len,
                                  enum sdio_direction dir)
{
    uint32_t    dctrl = 0;

    printf("len %x\n", bytes_len);
    write_reg_value(r_CORTEX_M_SDIO_DTIMER, timeout);
    set_reg(r_CORTEX_M_SDIO_DLEN, bytes_len, SDIO_DLEN_DATALENGTH);

    set_reg(&dctrl, 9, SDIO_DCTRL_DBLOCKSIZE);  /* 512 byte */
    set_reg_bits(&dctrl, SDIO_DCTRL_DMAEN_Msk);
    set_reg(&dctrl, SDIO_DCTRL_DTMODE_BLOCK, SDIO_DCTRL_DTMODE);
    set_reg(&dctrl, dir, SDIO_DCTRL_DTDIR);
    set_reg(&dctrl, 1, SDIO_DCTRL_DTEN);

    write_delay();
    write_reg_value(r_CORTEX_M_SDIO_DCTRL, dctrl);

    if (dir == IN) {
        data_path_state = DPSM_WAIT_R;
        set_reg_bits(r_CORTEX_M_SDIO_MASK, SDIO_MASK_RXACTIE_Msk);
    } else {
        data_path_state = DPSM_WAIT_S;
        set_reg_bits(r_CORTEX_M_SDIO_MASK, SDIO_MASK_TXACTIE_Msk);
    }
}

void sdio_hw_write_fifo(uint32_t * buf, uint32_t size)
{
    uint32_t    i;
    for (i = 0; i < size; i++)
        write_reg_value(r_CORTEX_M_SDIO_FIFO, buf[i]);
    //LOG("FIFO filled (%d words)\n", size);
}

void sdio_hw_read_fifo(uint32_t * buf, uint32_t size)
{
    uint32_t    i;
    for (i = 0; i < size; i++)
        buf[i] = read_reg_value(r_CORTEX_M_SDIO_FIFO);
    //LOG("FIFO emptied (%d words)\n", size);
}

volatile uint32_t *sdio_get_data_addr(void)
{
    return r_CORTEX_M_SDIO_FIFO;
}

/*****************************************
 * IRQ Handlers
 *
 * prototype for irq handlers is:
 * static void my_irq_handler(uint8_t irq, // IRQ number
 *                           uint32_t sr   // content of posthook.status,
 *                           uint32_t dr   // content of posthook.data)
 *
 *****************************************/

static      uint32_t(*sd_irq_handler) ();

static void sdio_reset_static_flags()
{
    set_reg_bits(r_CORTEX_M_SDIO_ICR, savestatus);
}

void sdio_set_irq_handler(uint32_t(*ptr) ())
{
    sd_irq_handler = ptr;
}

volatile uint32_t nbinter = 0;

void SDIO_IRQHandler(uint8_t irq __UNUSED,  // IRQ number
                     uint32_t sr,   // content of posthook.status,
                     uint32_t dr)   // content of posthook.data)
{
    uint32_t __attribute__ ((unused)) mask = dr;

    nbinter++;

#ifdef DEBUG_SDIO_INTERRUPTS
    add_log_status(sr);
#endif
    savestatus |= sr;
//  sdio_reset_static_flags();
//  LOG("IRQHANDLER status %x mask %x\n",status,mask);dbg_flush();
    if (sd_irq_handler)
        sd_irq_handler();
}

/********************************************
 * Various utility functions
 ********************************************/

uint32_t sdio_getflags(uint32_t mask)
{
    return savestatus & mask;   //*r_CORTEX_M_SDIO_STA & mask;
}

static volatile uint32_t waitfor_mask;

void sdio_wait_for(uint32_t msk)
{
    waitfor_mask = msk;
}

int sdio_finished_or_error(void)
{
    return sdio_getflags(waitfor_mask);
}

uint32_t sdio_clear_flag(uint32_t mask)
{
    return savestatus &= ~mask; //*r_CORTEX_M_SDIO_STA = mask;
}

void sdio_set_timeout(uint32_t timeout)
{
    write_reg_value(r_CORTEX_M_SDIO_DTIMER, timeout);
}

/*****************************************
 * Initialization functions
 *****************************************/
uint8_t sdio_early_init(void)
{
    uint8_t     ret = 0;

    /*******************************
     * first, SDIO device declaration
     *******************************/
    device_t    dev;
    memset((void*)&dev, 0, sizeof(device_t));
    int         dev_desc = 0;

    /*
     * declare the SDIO device, during initialization phase
     * This function create a device_t, fullfill it, and execute a
     * sys_init(INIT_DEVACCESS) syscall.
     */
    memcpy(dev.name, name, strlen(name));
    dev.address = 0x40012c00;
    dev.size = 0x400;
    dev.irq_num = 1;

    /* IRQ configuration */
    dev.irqs[0].handler = SDIO_IRQHandler;
    dev.irqs[0].irq = 65;       /* starting with STACK */
    dev.irqs[0].mode = IRQ_ISR_STANDARD;    /* if ISR force MT immediat execution, use FORCE_MAINTHREAD instead of STANDARD, and activate FISR permission */

    /*
     * IRQ posthook configuration
     * The posthook is executed at the end of the IRQ handler mode, *before* the ISR.
     * It permit to clean potential status registers (or others) that may generate IRQ loops
     * while the ISR has not been executed.
     * register read can be saved into 'status' and 'data' and given to the ISR in 'sr' and 'dr' argument
     */
    dev.irqs[0].posthook.status = 0x0034;   /* STA */
    dev.irqs[0].posthook.data = 0x003C; /* MASK */

    dev.irqs[0].posthook.action[0].instr = IRQ_PH_READ;
    dev.irqs[0].posthook.action[0].read.offset = 0x0034;    /* STA */

    dev.irqs[0].posthook.action[1].instr = IRQ_PH_READ;
    dev.irqs[0].posthook.action[1].read.offset = 0x003C;    /* MASK */

    dev.irqs[0].posthook.action[2].instr = IRQ_PH_WRITE;
    dev.irqs[0].posthook.action[2].write.offset = 0x003C;   /* MASK */
    dev.irqs[0].posthook.action[2].write.value =
        SDIO_FLAG_CCRCFAIL | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_CTIMEOUT |
        SDIO_FLAG_DTIMEOUT | SDIO_FLAG_TXUNDERR | SDIO_FLAG_RXOVERR |
        SDIO_FLAG_CMDREND | SDIO_FLAG_CMDSENT | SDIO_FLAG_DATAEND |
        SDIO_FLAG_STBITERR;
    dev.irqs[0].posthook.action[2].write.mask = 0xffffff;

    dev.irqs[0].posthook.action[3].instr = IRQ_PH_AND;
    dev.irqs[0].posthook.action[3].and.offset_dest = 0x0038;    /* ICR */
    dev.irqs[0].posthook.action[3].and.offset_src = 0x0034; /* STA */
    dev.irqs[0].posthook.action[3].and.mask = 0x006007ff;   /* AND action mask */
    dev.irqs[0].posthook.action[3].and.mode = 0;    /* ICR |= STA */

    /* Now let's configure the GPIOs */
    dev.gpio_num = 6;

    /* SDIO_D[0..3] are on PC8..PC11 */
    for (uint8_t i = 0; i < 4; ++i) {
        dev.gpios[i].mask =
            GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
            GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
        dev.gpios[i].kref.port = GPIO_PC;
        dev.gpios[i].kref.pin = i + 8;
        dev.gpios[i].mode = GPIO_PIN_ALTERNATE_MODE;
        dev.gpios[i].pupd = GPIO_NOPULL;
        dev.gpios[i].type = GPIO_PIN_OTYPER_PP;
        dev.gpios[i].speed = GPIO_PIN_VERY_HIGH_SPEED;
        dev.gpios[i].afr = GPIO_AF_SDIO;
    }

    /* SDIO_CK is on PC12 *///FIXME
    dev.gpios[4].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
        GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[4].kref.port = GPIO_PC;
    dev.gpios[4].kref.pin = 12;
    dev.gpios[4].mode = GPIO_PIN_ALTERNATE_MODE;
    dev.gpios[4].pupd = GPIO_NOPULL;

    /*
     * push-pull (0) for SD card identification, open-drain (1) for MMC
     * identification (see manual 31.4.4)
     */
    dev.gpios[4].type = GPIO_PIN_OTYPER_PP;
    dev.gpios[4].speed = GPIO_PIN_HIGH_SPEED;
    dev.gpios[4].afr = GPIO_AF_SDIO;

    /* SDIO_CMD is on PD2 *///FIXME
    dev.gpios[5].mask =
        GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE |
        GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    dev.gpios[5].kref.port = GPIO_PD;
    dev.gpios[5].kref.pin = 2;
    dev.gpios[5].mode = GPIO_PIN_ALTERNATE_MODE;
    dev.gpios[5].pupd = GPIO_NOPULL;

    /*
     * push-pull (0) for SD card identification, open-drain (1) for MMC
     * identification (see manual 31.4.4)
     */
    dev.gpios[5].type = GPIO_PIN_OTYPER_PP;
    dev.gpios[5].speed = GPIO_PIN_HIGH_SPEED;
    dev.gpios[5].afr = GPIO_AF_SDIO;

    ret = sys_init(INIT_DEVACCESS, &dev, &dev_desc);

    return ret;
}

uint8_t sdio_init(void)
{
    printf("initializing GPIO block\n");
    /*
     * configure the SDIO device, once it is mapped in task memory
     * This function must be executed *after* sys_init(INIT_DONE).
     */
    sdio_set_irq_handler(NULL);
    sdio_set_timeout(0xffffffff);
    sdio_hw_init2();
    return 0;
}

/* other functions abstracting SDIO interaction (low level commands execution) */
