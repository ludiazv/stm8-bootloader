#include <stdint.h>
#include "config.h"
#include "stm8.h"
#include "ram.h"

static uint8_t CRC;
static uint8_t ivt[128];
static uint8_t f_ram[128];
static uint8_t rx_buffer[BLOCK_SIZE];
static volatile uint8_t RAM_SEG_LEN;
static void (*flash_write_block)(uint16_t addr, const uint8_t *buf) =
        (void (*)(uint16_t, const uint8_t *)) f_ram;


/**
 * Write RAM_SEG section length into RAM_SEG_LEN
 */
inline void get_ram_section_length() {
    __asm__("mov _RAM_SEG_LEN, #l_RAM_SEG");
}

/**
 * Initialize watchdog:
 * prescaler = 32, timeout = 63.70ms -> Serial
 * prescaler = 64, timeout = 255ms ->I2C
 */
inline void iwdg_init() {
    IWDG_KR = IWDG_KEY_ENABLE;
    IWDG_KR = IWDG_KEY_ACCESS;
    #if defined(I2C_ADDR)
        IWDG_PR = 6;
    #else
        IWDG_PR = 2;
    #endif
    IWDG_KR = IWDG_KEY_REFRESH;
}

/**
 * Kick the dog
 */
inline void iwdg_refresh() {
    IWDG_KR = IWDG_KEY_REFRESH;
}

/**
 * Calculate CRC-8-CCIT.
 * Polynomial: x^8 + x^2 + x + 1 (0x07)
 *
 * @param data input byte
 * @param crc initial CRC
 * @return CRC value
 */
inline uint8_t crc8_update(uint8_t data, uint8_t crc) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    return crc;
}


/**
 * Initialize UART1 in 8N1 mode
 */
inline void uart_init() {
 
 #if !defined(I2C_ADDR)   
    /* enable UART clock (STM8L only)*/
    UART_CLK_ENABLE();
    /* madness.. */
    UART_BRR2 = ((UART_DIV >> 8) & 0xF0) + (UART_DIV & 0x0F);
    UART_BRR1 = UART_DIV >> 4;
    /* enable transmitter and receiver */
    UART_CR2 = (1 << UART_CR2_TEN) | (1 << UART_CR2_REN);
#else
    /* Init I2C perfiferical */
      // Reset the peri and configure it
    I2C_CR1 =0;  // Disable periphericas (allso enable stretch & disable general call)
    I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz)=> 16Mhz
    //I2C_CCRL = 0xa0;                    //  SCL clock speed is 100Khz. (will have no efect)
    //I2C_CCRH = 0;                       //  Reset I2C mode and duty cycle
    I2C_OARL = (I2C_ADDR << 1);     // Set Slave address set Addrress + ADD0. ADD0=0 7-bit addrs
    I2C_OARH = 0b01000000;              // bit 6 must be 1 as of RM0016
    I2C_CR1 |= (1<<I2C_CR1_PE);   // Enable periphal
    I2C_CR2 |= (1<<I2C_CR2_ACK);  // Enable ACK respond


#endif

}

#if defined(DELAY_COUNT)
inline void uart_deinit() {

#if !defined(I2C_ADDR)   

    /* enable UART clock (STM8L only)*/
    //UART_CLK_DISABLE(); TODO L VERSION
    UART_BRR2 = 0;
    UART_BRR1 = 0;
    UART_CR2 = 0;

#else

    /* Reset I2C perfiferical */
    I2C_CR1 =0; 
    I2C_OARL = 0;     
    I2C_OARH = 0;       
    I2C_CR2 = 0;

#endif
}

inline uint8_t check_bootloader_activation() {
    uint16_t delay_count=DELAY_COUNT;
    #if defined(I2C_ADDR) /* I2c Activation */
        while( (delay_count > 0) && !(I2C_SR1 & (1 << I2C_SR1_ADDR))) {
            __asm__("nop");
            delay_count--;
        }
    #else   /* Serial activation */
        while( (delay_count > 0) && !(UART_SR & (1 << UART_SR_RXNE)) ) {
            __asm__("nop");
            delay_count--;
        };
    #endif
    return delay_count>0;
}

#endif



#if defined(I2C_ADDR) // I2C bootloader

static void i2c_match() {
    uint8_t dummy;
    /* 1st wait match */
    I2C_CR1 |= (1<<I2C_CR1_PE);
    I2C_CR2 |= (1<<I2C_CR2_ACK);  // Enable ACK respond
    iwdg_refresh();
    while( !(I2C_SR1 & (1 << I2C_SR1_ADDR)));
    dummy=I2C_SR1; // To clear flag as RM0016
    dummy=I2C_SR3; // To clear Flag
}
static void i2c_stop() {
    // this code of wating for is stop is not working in al cases. Workrround is to deactivate de periferal.
    //uint8_t dummy;
    //iwdg_refresh();
    //while(  !( I2C_SR1 & (1 << I2C_SR1_STOPF)) );
    //dummy=I2C_SR1; // required by RM0016
    //I2C_CR2 = 0b100; // Required by RM0016 to clear the stop flag (here we write ACK activeted)
    iwdg_refresh();
    __asm__("nop");
    I2C_CR1 &= ~(1<<I2C_CR1_PE);   // Disable periphal this make sure all flags are cleaned
    //I2C_CR1 |= (1<<I2C_CR1_PE);   // Enable periphal
}
static void read_i2c_block(uint8_t *dest,uint8_t len) {
    uint8_t rx;
   /* 1st wait match */
    i2c_match();
    /* 2nd Loop for reading i2c data */
    for(uint8_t i=0;i<len;i++) {
        iwdg_refresh();
        while( !(I2C_SR1 & (1 << I2C_SR1_RXNE)) );
        rx=I2C_DR;
        *dest++=rx;
        CRC=crc8_update(rx,CRC);
    }
    /* 3rd stop condition */
    i2c_stop();
}
static void i2c_write(uint16_t w) {
    i2c_match();
    for(uint8_t i=0;i<2;i++){
        iwdg_refresh();
        while( !(I2C_SR1 & (1 << I2C_SR1_TXE)) );
        I2C_DR=((uint8_t *)&w)[i];
    }
    i2c_stop();
}

inline void bootloader_exec() {
    uint8_t chunks, crc_rx;
    uint16_t addr = BOOT_ADDR;
    // Transfer entry
    for(;;) {
        read_i2c_block(rx_buffer,7);
        if(*((uint16_t*)rx_buffer)==0xDEAD && *((uint16_t *)&rx_buffer[2])==0xBEEF ) {
            chunks=rx_buffer[4];
            crc_rx=rx_buffer[5];
            if(crc_rx==rx_buffer[6]) break;
            i2c_write(0xDEAD);
        }
    }
    i2c_write(0xAABB); // Respond ACK on next read.
    CRC=0; // clear crc
    FLASH_LED();

#if !RELOCATE_IVT
    /* get application interrupt table */
    read_i2c_block(ivt,BLOCK_SIZE);
    chunks--;
    #if BLOCK_SIZE == 64
    chunks--;
    read_i2c_block(ivt + BLOCK_SIZE,BLOCK_SIZE);
    #endif
#endif
        /* unlock flash */
    FLASH_PUKR = FLASH_PUKR_KEY1;
    FLASH_PUKR = FLASH_PUKR_KEY2;
    while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_PUL)));

    /* get main firmware */
    for (uint8_t i = 0; i < chunks; i++) {
        read_i2c_block(rx_buffer,BLOCK_SIZE);
        flash_write_block(addr, rx_buffer);
        addr += BLOCK_SIZE;
        FLASH_LED();
    }

    /* verify CRC */
    if (CRC != crc_rx) {
        i2c_write(0xDEAD);
        for (;;) ;
    }

#if !RELOCATE_IVT
    /* overwrite vector table preserving the reset interrupt */
    *(uint32_t *) ivt = *(uint32_t *) (0x8000);
    flash_write_block(0x8000, ivt);
    #if BLOCK_SIZE == 64
    flash_write_block(0x8000 + BLOCK_SIZE, ivt + BLOCK_SIZE);
    #endif
#endif

    /* lock flash */
    FLASH_IAPSR &= ~(1 << FLASH_IAPSR_PUL);

    i2c_write(0xAABB);

    /* reboot */
    FLASH_LED();
    for (;;) ;

}

#else // Serial boot loader
/**
 * Write byte into UART
 */
static void uart_write(uint8_t data) {
    UART_DR = data;
    while (!(UART_SR & (1 << UART_SR_TC)));
}

/**
 * Read byte from UART and reset watchdog
 */
static uint8_t uart_read() {
    iwdg_refresh();
    while (!(UART_SR & (1 << UART_SR_RXNE)));
    return UART_DR;
}



/**
 * Send ACK response
 */
static void serial_send_ack() {
    uart_write(0xAA);
    uart_write(0xBB);
}

/**
 * Send NACK response (CRC mismatch)
 */
inline void serial_send_nack() {
    uart_write(0xDE);
    uart_write(0xAD);
}

/**
 * Read BLOCK_SIZE bytes from UART
 *
 * @param dest destination buffer
 */
static void serial_read_block(uint8_t *dest) {
    serial_send_ack();
    for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
        uint8_t rx = uart_read();
        dest[i] = rx;
        CRC = crc8_update(rx, CRC);
    }
}

/**
 * Enter bootloader and perform firmware update
 */
inline void bootloader_exec() {
    uint8_t chunks, crc_rx;
    uint16_t addr = BOOT_ADDR;

    /* enter bootloader */
    for (;;) {
        uint8_t rx = uart_read();
        if (rx != 0xDE) continue;
        rx = uart_read();
        if (rx != 0xAD) continue;
        rx = uart_read();
        if (rx != 0xBE) continue;
        rx = uart_read();
        if (rx != 0xEF) continue;
        chunks = uart_read();
        crc_rx = uart_read();
        rx = uart_read();
        if (crc_rx != rx)
            continue;
        break;
    }
    FLASH_LED();

#if !RELOCATE_IVT
    /* get application interrupt table */
    serial_read_block(ivt);
    chunks--;
    #if BLOCK_SIZE == 64
    chunks--;
    serial_read_block(ivt + BLOCK_SIZE);
    #endif
#endif

    /* unlock flash */
    FLASH_PUKR = FLASH_PUKR_KEY1;
    FLASH_PUKR = FLASH_PUKR_KEY2;
    while (!(FLASH_IAPSR & (1 << FLASH_IAPSR_PUL)));

    /* get main firmware */
    for (uint8_t i = 0; i < chunks; i++) {
        serial_read_block(rx_buffer);
        flash_write_block(addr, rx_buffer);
        addr += BLOCK_SIZE;
        FLASH_LED();
    }

    /* verify CRC */
    if (CRC != crc_rx) {
        serial_send_nack();
        for (;;) ;
    }

#if !RELOCATE_IVT
    /* overwrite vector table preserving the reset interrupt */
    *(uint32_t *) ivt = *(uint32_t *) (0x8000);
    flash_write_block(0x8000, ivt);
    #if BLOCK_SIZE == 64
    flash_write_block(0x8000 + BLOCK_SIZE, ivt + BLOCK_SIZE);
    #endif
#endif

    /* lock flash */
    FLASH_IAPSR &= ~(1 << FLASH_IAPSR_PUL);

    serial_send_ack();
    FLASH_LED();

    /* reboot */
    for (;;) ;
}

#endif

/**
 * Copy ram_flash_write_block routine into RAM
 */
inline void ram_cpy() {
    get_ram_section_length();
    for (uint8_t i = 0; i < RAM_SEG_LEN; i++)
        f_ram[i] = ((uint8_t *) ram_flash_write_block)[i];
}


/**
 * Bootloader entry point
 * 
 */
void bootloader_main() {


#if !defined(DELAY_COUNT)  // Pin based bootloader  
    BOOT_PIN_CR1 = 1 << BOOT_PIN;
    if (!(BOOT_PIN_IDR & (1 << BOOT_PIN))) {
        /* execute bootloader */
        CLK_CKDIVR = 0; // Max speed for internal clock
        #if defined(FLASH_PIN)
            FLASH_PIN_DDR |= (1 << FLASH_PIN);
            FLASH_PIN_CR1 |= (1 << FLASH_PIN);
        #endif
        FLASH_LED();
        ram_cpy();
        iwdg_init();
        uart_init();
        bootloader_exec();
    } else {
        /* jump to application */
        BOOT_PIN_CR1 = 0x00;
        BOOT();
    }
#else
    CLK_CKDIVR=0;
    uart_init();
    if(check_bootloader_activation()) {
        #if defined(FLASH_PIN)
            FLASH_PIN_DDR |= (1 << FLASH_PIN);
            FLASH_PIN_CR1 |= (1 << FLASH_PIN);
        #endif
        FLASH_LED();
        ram_cpy();
        iwdg_init();
        bootloader_exec();
    }
    uart_deinit();
    CLK_CKDIVR=0x18; // restore reset value
    BOOT();
#endif

}
