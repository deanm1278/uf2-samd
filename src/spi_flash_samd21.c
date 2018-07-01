#include "uf2.h"

#define  SPIFLASH_SPI_STATREAD      0x02
#define  SPIFLASH_SPI_DATAWRITE     0x01
#define  SPIFLASH_SPI_DATAREAD      0x03
#define  SPIFLASH_SPI_READY         0x01

// Flash status bits
#define  SPIFLASH_STAT_BUSY         0x01   // Erase/Write in Progress
#define  SPIFLASH_STAT_WRTEN        0x02   // Write Enable Latch

// SPI Flash Characteristics (W25Q16BV Specific)
#define W25Q16BV_MAXADDRESS         0x1FFFFF
#define W25Q16BV_PAGESIZE           256    // 256 bytes per programmable page
#define W25Q16BV_PAGES              8192   // 2,097,152 Bytes / 256 bytes per page
#define W25Q16BV_SECTORSIZE         4096   // 1 erase sector = 4096 bytes
#define W25Q16BV_SECTORS            512    // 2,097,152 Bytes / 4096 bytes per sector
#define W25Q16BV_MANUFACTURERID     0xEF   // Used to validate read data
#define W25Q16BV_DEVICEID           0x14   // Used to validate read data

// Erase/Program Instructions
#define W25Q16BV_CMD_WRITEENABLE    0x06   // Write Enabled
#define W25Q16BV_CMD_WRITEDISABLE   0x04   // Write Disabled
#define W25Q16BV_CMD_READSTAT1      0x05   // Read Status Register 1
#define W25Q16BV_CMD_READSTAT2      0x35   // Read Status Register 2
#define W25Q16BV_CMD_WRITESTAT      0x01   // Write Status Register
#define W25Q16BV_CMD_PAGEPROG       0x02   // Page Program
#define W25Q16BV_CMD_QUADPAGEPROG   0x32   // Quad Page Program
#define W25Q16BV_CMD_SECTERASE4     0x20   // Sector Erase (4KB)
#define W25Q16BV_CMD_BLOCKERASE32   0x52   // Block Erase (32KB)
#define W25Q16BV_CMD_BLOCKERASE64   0xD8   // Block Erase (64KB)
#define W25_CMD_CHIPERASE      0x60   // Chip Erase
#define W25Q16BV_CMD_ERASESUSPEND   0x75   // Erase Suspend
#define W25Q16BV_CMD_ERASERESUME    0x7A   // Erase Resume
#define W25Q16BV_CMD_POWERDOWN      0xB9   // Power Down
#define W25Q16BV_CMD_CRMR           0xFF   // Continuous Read Mode Reset
// Read Instructions
#define W25Q16BV_CMD_FREAD          0x0B   // Fast Read
#define W25Q16BV_CMD_FREADDUALOUT   0x3B   // Fast Read Dual Output
#define W25Q16BV_CMD_FREADDUALIO    0xBB   // Fast Read Dual I/O
#define W25Q16BV_CMD_FREADQUADOUT   0x6B   // Fast Read Quad Output
#define W25Q16BV_CMD_FREADQUADIO    0xEB   // Fast Read Quad I/O
#define W25Q16BV_CMD_WREADQUADIO    0xE7   // Word Read Quad I/O
#define W25Q16BV_CMD_OWREADQUADIO   0xE3   // Octal Word Read Quad I/O
// ID/Security Instructions
#define W25Q16BV_CMD_RPWRDDEVID     0xAB   // Release Power Down/Device ID
#define W25Q16BV_CMD_MANUFDEVID     0x90   // Manufacturer/Device ID
#define W25Q16BV_CMD_MANUFDEVID2    0x92   // Manufacturer/Device ID by Dual I/O
#define W25Q16BV_CMD_MANUFDEVID4    0x94   // Manufacturer/Device ID by Quad I/O
#define W25Q16BV_CMD_JEDECID        0x9F   // JEDEC ID
#define W25Q16BV_CMD_READUNIQUEID   0x4B   // Read Unique ID

#define W25Q128FV_BLOCKS 255
#define W25Q128FV_BLOCKSIZE 65536

extern uint8_t transferDataSPI( Sercom * sercom, uint8_t data) ;
extern bool isBufferOverflowErrorSPI( Sercom * sercom ) ;
extern bool isDataRegisterEmptySPI( Sercom * sercom ) ;
extern bool isTransmitCompleteSPI( Sercom * sercom ) ;
extern bool isReceiveCompleteSPI( Sercom * sercom ) ;

#define spiwrite(x) transferDataSPI(SPI_SERCOM, x)
#define spiread() transferDataSPI(SPI_SERCOM, 0x00)

// this actually generates less code than a function
#define wait_ready()                                                                               \
    while (flash_read_status() & SPIFLASH_STAT_BUSY)                                                        \
        ;

static uint8_t flash_read_status()
{
  uint8_t status;

  PINOP(SPI_FLASH_SS, OUTCLR);
  spiwrite(W25Q16BV_CMD_READSTAT1);    // Send read status 1 cmd
  status = spiread();                  // Dummy write
  PINOP(SPI_FLASH_SS, OUTSET);

  return status & (SPIFLASH_STAT_BUSY | SPIFLASH_STAT_WRTEN);
}

void write_enable (bool enable)
{
  PINOP(SPI_FLASH_SS, OUTCLR);
  spiwrite(enable ? W25Q16BV_CMD_WRITEENABLE : W25Q16BV_CMD_WRITEDISABLE);
  PINOP(SPI_FLASH_SS, OUTSET);
}

void erase_block (uint32_t blockNumber)
{
  // Make sure the address is valid
  if (blockNumber >= W25Q128FV_BLOCKS) {
    return;
  }  

  // Wait until the device is ready or a timeout occurs
  while (flash_read_status() & SPIFLASH_STAT_BUSY);

  // Make sure the chip is write enabled
  write_enable(1);

  // Make sure the write enable latch is actually set
  uint8_t status;
  status = flash_read_status();
  if (!(status & SPIFLASH_STAT_WRTEN))  {
    // Throw a write protection error (write enable latch not set)
    return;
  }

  // Send the erase sector command
  uint32_t address = blockNumber * W25Q128FV_BLOCKSIZE;
  PINOP(SPI_FLASH_SS, OUTCLR);
  spiwrite(W25Q16BV_CMD_BLOCKERASE64); 
  spiwrite((address >> 16) & 0xFF);     // address upper 8
  spiwrite((address >> 8) & 0xFF);      // address mid 8
  spiwrite(address & 0xFF);             // address lower 8
  PINOP(SPI_FLASH_SS, OUTSET);

  // Wait until the busy bit is cleared before exiting
  // This can take up to 2s according to the datasheet
  while (flash_read_status() & SPIFLASH_STAT_BUSY);

  return;
}

void flash_erase_to_end(uint32_t *start_address) {
#if 0
    //this erases the whole chip regardless of address
    (void)start_address;

    wait_ready();

    // Make sure the chip is write enabled
    write_enable(1);

    // Send the erase chip command
    PINOP(SPI_FLASH_SS, OUTCLR);
    spiwrite(W25_CMD_CHIPERASE);
    PINOP(SPI_FLASH_SS, OUTSET);

    // Wait until the busy bit is cleared before exiting
    // This can take up to 10 seconds according to the datasheet!
    while (flash_read_status() & SPIFLASH_STAT_BUSY);
#else
    //lets actually erase a few blocks here. I think it times out
    (void)start_address;
    for(int i=0; i<24; i++){
      erase_block(i);
    }
#endif

}

static uint32_t write_page (uint32_t address, uint8_t *buffer, uint32_t len)
{
  uint8_t status;
  uint32_t i;

  // Make sure the address is valid
  if (address >= W25Q16BV_MAXADDRESS)
  {
    return 0;
  }

  // Make sure that the supplied data is no larger than the page size
  if (len > W25Q16BV_PAGESIZE)
  {
    return 0;
  }

  // Make sure that the data won't wrap around to the beginning of the sector
  if ((address % W25Q16BV_PAGESIZE) + len > W25Q16BV_PAGESIZE)
  {
    // If you try to write to a page beyond the last byte, it will
    // wrap around to the start of the page, almost certainly
    // messing up your data
    return 0;
  }

  // Wait until the device is ready or a timeout occurs
  wait_ready();

  // Make sure the chip is write enabled
  write_enable (1);

  // Make sure the write enable latch is actually set
  status = flash_read_status();
  if (!(status & SPIFLASH_STAT_WRTEN))
  {
    // Throw a write protection error (write enable latch not set)
    return 0;
  }

  PINOP(SPI_FLASH_SS, OUTCLR);

  if (SPI_FLASH_ADDRSIZE == 24) {
    // Send page write command (0x02) plus 24-bit address
    spiwrite(W25Q16BV_CMD_PAGEPROG);      // 0x02
    spiwrite((address >> 16) & 0xFF);     // address upper 8
    spiwrite((address >> 8) & 0xFF);      // address mid 8
    if (len == W25Q16BV_PAGESIZE)
      {
    // If len = 256 bytes, lower 8 bits must be 0 (see datasheet 11.2.17)
    spiwrite(0);
      }
    else
      {
    spiwrite(address & 0xFF);           // address lower 8
      }
  } else if (SPI_FLASH_ADDRSIZE == 16) {
    // Send page write command (0x02) plus 16-bit address
    spiwrite(W25Q16BV_CMD_PAGEPROG);      // 0x02
    spiwrite((address >> 8) & 0xFF);     // address upper 8
    spiwrite((address) & 0xFF);      // address lower 8
  } else {
    return 0;
  }

  // Transfer data
  for (i = 0; i < len; i++)
  {
    spiwrite(buffer[i]);
  }
  // Write only occurs after the CS line is de-asserted
  PINOP(SPI_FLASH_SS, OUTSET);

  // Wait at least 3ms (max page program time according to datasheet)
  delay(3);

  // Wait until the device is ready or a timeout occurs
  wait_ready();

  return len;
}

static uint32_t write_buffer(uint32_t address, uint8_t *buffer, uint32_t len)
{
  uint32_t bytestowrite;
  uint32_t bufferoffset;
  uint32_t results;
  uint32_t byteswritten;

  // There's no point duplicating most error checks here since they will all be
  // done in the underlying call to spiflashWritePage

  // If the data is only on one page we can take a shortcut
  if ((address % W25Q16BV_PAGESIZE) + len <= W25Q16BV_PAGESIZE)
  {
    // Only one page ... write and be done with it
    return write_page(address, buffer, len);
  }

  // Block spans multiple pages
  byteswritten = 0;
  bufferoffset = 0;
  while(len)
  {
    // Figure out how many bytes need to be written to this page
    bytestowrite = W25Q16BV_PAGESIZE - (address % W25Q16BV_PAGESIZE);
    // Write the current page
    results = write_page(address, buffer+bufferoffset, bytestowrite);
    byteswritten += results;
    // Abort if we returned an error
    if (!results)
       return byteswritten;  // Something bad happened ... but return what we've written so far
    // Adjust address and len, and buffer offset
    address += bytestowrite;
    len -= bytestowrite;
    bufferoffset+=bytestowrite;
    // If the next page is the last one, write it and exit
    // otherwise stay in the the loop and keep writing
    if (len <= W25Q16BV_PAGESIZE)
    {
      // Write the last frame and then quit
      results = write_page(address, buffer+bufferoffset, len);
      byteswritten += results;
      // Abort if we returned an error
      if (!results)
        return byteswritten;  // Something bad happened ... but return what we've written so far
      // set len to zero to gracefully exit loop
      len = 0;
    }
  }

  // Return the number of bytes written
  return byteswritten;
}

void flash_write_words(uint32_t *dst, uint32_t *src, uint32_t n_words) {
    write_buffer((uint32_t)dst, (uint8_t *)src, n_words*4);
}
