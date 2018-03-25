#include "uf2.h"
#define SYSCTRL_FUSES_OSC32K_CAL_ADDR   (NVMCTRL_OTP4 + 4)
#define SYSCTRL_FUSES_OSC32K_CAL_Pos   6
#define 	SYSCTRL_FUSES_OSC32K_ADDR   SYSCTRL_FUSES_OSC32K_CAL_ADDR
#define 	SYSCTRL_FUSES_OSC32K_Pos   SYSCTRL_FUSES_OSC32K_CAL_Pos
#define 	SYSCTRL_FUSES_OSC32K_Msk   (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)

volatile bool g_interrupt_enabled = true;

static void gclk_sync(void) {
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;
}

static void dfll_sync(void) {
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0)
        ;
}

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     64

/*  =========================
 *  ===== Sercom SPI
 *  =========================
*/

typedef enum
{
  SPI_SLAVE_OPERATION = 0x2u,
  SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

enum
{
  WIRE_UNKNOWN_STATE = 0x0ul,
  WIRE_IDLE_STATE,
  WIRE_OWNER_STATE,
  WIRE_BUSY_STATE
};

typedef enum
{
  SERCOM_EVEN_PARITY = 0,
  SERCOM_ODD_PARITY,
  SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
  SERCOM_STOP_BIT_1 = 0,
  SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
  MSB_FIRST = 0,
  LSB_FIRST
} SercomDataOrder;

typedef enum
{
  SERCOM_RX_PAD_0 = 0,
  SERCOM_RX_PAD_1,
  SERCOM_RX_PAD_2,
  SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
  SERCOM_SPI_MODE_0 = 0,  // CPOL : 0  | CPHA : 0
  SERCOM_SPI_MODE_1,    // CPOL : 0  | CPHA : 1
  SERCOM_SPI_MODE_2,    // CPOL : 1  | CPHA : 0
  SERCOM_SPI_MODE_3   // CPOL : 1  | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
  SPI_PAD_0_SCK_1 = 0,
  SPI_PAD_2_SCK_3,
  SPI_PAD_3_SCK_1,
  SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
  SPI_CHAR_SIZE_8_BITS = 0x0ul,
  SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;

#define SERCOM_FREQ_REF 48000000

void initSPI( Sercom * sercom, SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder) ;
void initSPIClock( Sercom * sercom, SercomSpiClockMode clockMode, uint32_t baudrate) ;

void resetSPI( Sercom * sercom ) ;
void enableSPI( Sercom * sercom ) ;
void disableSPI( Sercom * sercom ) ;
void setDataOrderSPI( Sercom * sercom, SercomDataOrder dataOrder) ;
SercomDataOrder getDataOrderSPI( Sercom * sercom ) ;
void setBaudrateSPI( Sercom * sercom, uint8_t divider) ;
void setClockModeSPI( Sercom * sercom, SercomSpiClockMode clockMode) ;

void initClock( Sercom *sercom )
{
  uint8_t clockId = 0;

  if(sercom == SERCOM0)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM0_CORE;
  }
  else if(sercom == SERCOM1)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM1_CORE;
  }

#if defined(SERCOM2)
  else if(sercom == SERCOM2)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM2_CORE;
  }
#endif

#if defined(__SAMD21G18A__)

  else if(sercom == SERCOM3)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM3_CORE;
  }
  else if(sercom == SERCOM4)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM4_CORE;
  }
  else if(sercom == SERCOM5)
  {
    clockId = GCLK_CLKCTRL_ID_SERCOM5_CORE;
}
#endif

  //Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
  GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
  GCLK_CLKCTRL_CLKEN ;
  
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_SERCOMX_SLOW ) |
  GCLK_CLKCTRL_GEN_GCLK1 |
  GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }
}

static uint8_t calculateBaudrateSynchronous(uint32_t baudrate)
{
  return SERCOM_FREQ_REF / (2 * baudrate) - 1;
}

void initSPI( Sercom *sercom, SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
  resetSPI(sercom);
  initClock(sercom);

  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN;  //Active the SPI receiver.


}

void initSPIClock( Sercom *sercom, SercomSpiClockMode clockMode, uint32_t baudrate)
{
  //Extract data from clockMode
  int cpha, cpol;

  if((clockMode & (0x1ul)) == 0 )
    cpha = 0;
  else
    cpha = 1;

  if((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg |=  ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                            ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
}

void resetSPI( Sercom *sercom )
{
  //Setting the Software Reset bit to 1
  sercom->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
}

void enableSPI( Sercom *sercom )
{
  //Setting the enable bit to 1
  sercom->SPI.CTRLA.bit.ENABLE = 1;

  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
}

void disableSPI( Sercom *sercom )
{
  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  sercom->SPI.CTRLA.bit.ENABLE = 0;
}

void setDataOrderSPI( Sercom *sercom, SercomDataOrder dataOrder)
{
  //Register enable-protected
  disableSPI(sercom);

  sercom->SPI.CTRLA.bit.DORD = dataOrder;

  enableSPI(sercom);
}

SercomDataOrder getDataOrderSPI( Sercom *sercom )
{
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
}

void setBaudrateSPI( Sercom *sercom, uint8_t divider)
{
  //Can't divide by 0
  if(divider == 0)
    return;

  //Register enable-protected
  disableSPI(sercom);

  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous( SERCOM_FREQ_REF / divider );

  enableSPI(sercom);
}

void setClockModeSPI( Sercom *sercom, SercomSpiClockMode clockMode)
{
  int cpha, cpol;
  if((clockMode & (0x1ul)) == 0)
    cpha = 0;
  else
    cpha = 1;

  if((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  //Register enable-protected
  disableSPI(sercom);

  sercom->SPI.CTRLA.bit.CPOL = cpol;
  sercom->SPI.CTRLA.bit.CPHA = cpha;

  enableSPI(sercom);
}

uint8_t transferDataSPI( Sercom *sercom, uint8_t data)
{
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while( sercom->SPI.INTFLAG.bit.RXC == 0 )
  {
    // Waiting Complete Reception
  }

  return sercom->SPI.DATA.bit.DATA;  // Reading data
}


void system_init(void) {

  NVMCTRL->CTRLB.bit.RWS = 1;

#if defined(CRYSTALLESS)
  /* Configure OSC8M as source for GCLK_GEN 2 */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2);  // Read GENERATOR_ID - GCLK_GEN_2
  gclk_sync();

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSC8M_Val | GCLK_GENCTRL_GENEN;
  gclk_sync();

  // Turn on DFLL with USB correction and sync to internal 8 mhz oscillator
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  dfll_sync();

  SYSCTRL_DFLLVAL_Type dfllval_conf = {0};
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4)
		       + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32))
		     >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
    & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  dfllval_conf.bit.COARSE  = coarse;
  // TODO(tannewt): Load this from a well known flash location so that it can be
  // calibrated during testing.
  dfllval_conf.bit.FINE    = 0x1ff;

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 10 ) |
                         48000;
  SYSCTRL->DFLLVAL.reg = dfllval_conf.reg;
  SYSCTRL->DFLLCTRL.reg = 0;
  dfll_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_MODE |
                          SYSCTRL_DFLLCTRL_CCDIS |
                          SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
                          SYSCTRL_DFLLCTRL_BPLCKC;
  dfll_sync();
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;
  dfll_sync();

  GCLK_CLKCTRL_Type clkctrl={0};
  uint16_t temp;
  GCLK->CLKCTRL.bit.ID = 2; // GCLK_ID - DFLL48M Reference
  temp = GCLK->CLKCTRL.reg;
  clkctrl.bit.CLKEN = 1;
  clkctrl.bit.WRTLOCK = 0;
  clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
  GCLK->CLKCTRL.reg = (clkctrl.reg | temp);

  uint32_t calib = (*((uint32_t *) FUSES_OSC32K_CAL_ADDR) & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;

  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) |
                        SYSCTRL_OSC32K_STARTUP( 0x6u ) | // cf table 15.10 of product datasheet in chapter 15.8.6
                        SYSCTRL_OSC32K_EN32K |
                        SYSCTRL_OSC32K_ENABLE;

#else

    SYSCTRL->XOSC32K.reg =
        SYSCTRL_XOSC32K_STARTUP(6) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    SYSCTRL->XOSC32K.bit.ENABLE = 1;
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0)
        ;

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1);
    gclk_sync();

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;
    gclk_sync();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
    gclk_sync();

    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    dfll_sync();

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(31) | SYSCTRL_DFLLMUL_FSTEP(511) |
                           SYSCTRL_DFLLMUL_MUL((CPU_FREQUENCY / (32 * 1024)));
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |=
        SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE;

    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
           (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0)
        ;
    dfll_sync();

#endif

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0);
    gclk_sync();

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK->GENCTRL.reg =
        GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    gclk_sync();

    SysTick_Config(1000);

    /* Write Generic Clock Generator 1 configuration */
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1U ) | // Generic Clock Generator 1
    #if defined(CRYSTALLESS)
                          GCLK_GENCTRL_SRC_OSC32K | // Selected source is Internal 32KHz Oscillator
    #else
                          GCLK_GENCTRL_SRC_XOSC32K | // Selected source is External 32KHz Oscillator
    #endif
    //                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                          GCLK_GENCTRL_GENEN ;

    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
    /* Wait for synchronization */
    }

    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;

    //init spi flash
    uint32_t port;
    uint8_t pin;

    /* Mask 6th bit in pin number to check whether it is greater than 32
     * i.e., PORTB pin */
    port = (SPI_FLASH_MISO & 0x200000) >> 21;
    pin = SPI_FLASH_MISO >> 16;
    PORT->Group[port].PINCFG[(pin - (port * 32))].bit.PMUXEN = 1;
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg &= ~(0xF << (4 * (pin & 0x01u)));
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg |= (SPI_FLASH_MISO & 0xFF)
                                                           << (4 * (pin & 0x01u));

    port = (SPI_FLASH_MOSI & 0x200000) >> 21;
    pin = SPI_FLASH_MOSI >> 16;
    PORT->Group[port].PINCFG[(pin - (port * 32))].bit.PMUXEN = 1;
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg &= ~(0xF << (4 * (pin & 0x01u)));
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg |= (SPI_FLASH_MOSI & 0xFF)
                                                           << (4 * (pin & 0x01u));

    port = (SPI_FLASH_SCLK & 0x200000) >> 21;
    pin = SPI_FLASH_SCLK >> 16;
    PORT->Group[port].PINCFG[(pin - (port * 32))].bit.PMUXEN = 1;
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg &= ~(0xF << (4 * (pin & 0x01u)));
    PORT->Group[port].PMUX[(pin - (port * 32)) / 2].reg |= (SPI_FLASH_SCLK & 0xFF)
                                                           << (4 * (pin & 0x01u));

    PINOP(SPI_FLASH_SS, DIRSET);
    PINOP(SPI_FLASH_SS, OUTSET);

    PM->APBCMASK.reg |= (1u << PM_APBCMASK_SERCOM1_Pos);

    disableSPI(SPI_SERCOM);

    initSPI(SPI_SERCOM, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
    initSPIClock(SPI_SERCOM, SERCOM_SPI_MODE_0, 4000000);

    enableSPI(SPI_SERCOM);

    //hold the DSP in reset
    PINOP(DSP_RESET_PIN, DIRSET);
    PINOP(DSP_RESET_PIN, OUTCLR);
}

void SysTick_Handler(void) { LED_TICK(); }
