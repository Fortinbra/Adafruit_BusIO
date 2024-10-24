#include "Adafruit_SPIDevice.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// #define DEBUG_SERIAL Serial

/*!
 *    @brief  Create an SPI device with the given CS pin and settings
 *    @param  cspin The arduino pin number to use for chip select
 *    @param  freq The SPI clock frequency to use, defaults to 1MHz
 *    @param  dataOrder The SPI data order to use for bits within each byte,
 * defaults to SPI_BITORDER_MSBFIRST
 *    @param  dataMode The SPI mode to use, defaults to SPI_MODE0
 *    @param  theSPI The SPI bus to use, defaults to &theSPI
 */
Adafruit_SPIDevice::Adafruit_SPIDevice(uint8_t cs, uint32_t freq, BusIOBitOrder dataOrder, uint8_t dataMode, SPIClass *theSPI)
    : _cs(cs), _spi(theSPI), _freq(freq), _dataOrder(dataOrder), _dataMode(dataMode)
{
  // Constructor body can be empty or contain initialization code
}

/*!
 *    @brief  Release memory allocated in constructors
 */
Adafruit_SPIDevice::~Adafruit_SPIDevice()
{
  // No dynamic memory allocation in this implementation
}

/*!
 *    @brief  Initializes SPI bus and sets CS pin high
 *    @return Always returns true because there's no way to test success of SPI
 * init
 */
bool Adafruit_SPIDevice::begin(void)
{
  if (_cs != -1)
  {
    gpio_init(_cs);
    gpio_set_dir(_cs, GPIO_OUT);
    gpio_put(_cs, 1); // Set CS high
  }

  if (_spi)
  { // hardware SPI
    spi_init(_spi, _freq);
    spi_set_format(_spi, 8, (spi_cpol_t)(_dataMode & 0x02), (spi_cpha_t)(_dataMode & 0x01), (spi_order_t)_dataOrder);
  }

  _begun = true;
  return true;
}

/*!
 *    @brief  Transfer (send/receive) a buffer over hard/soft SPI, without
 * transaction management
 *    @param  buffer The buffer to send and receive at the same time
 *    @param  len    The number of bytes to transfer
 */
void Adafruit_SPIDevice::transfer(uint8_t *buffer, size_t len)
{
  spi_write_read_blocking(_spi, buffer, buffer, len);
}

/*!
 *    @brief  Transfer (send/receive) one byte over hard/soft SPI, without
 * transaction management
 *    @param  send The byte to send
 *    @return The byte received while transmitting
 */
uint8_t Adafruit_SPIDevice::transfer(uint8_t send)
{
  uint8_t receive;
  spi_write_read_blocking(_spi, &send, &receive, 1);
  return receive;
}

/*!
 *    @brief  Manually begin a transaction (calls beginTransaction if hardware
 * SPI)
 */
void Adafruit_SPIDevice::beginTransaction(void)
{
  setChipSelect(0); // Set CS low
}

/*!
 *    @brief  Manually end a transaction (calls endTransaction if hardware SPI)
 */
void Adafruit_SPIDevice::endTransaction(void)
{
  setChipSelect(1); // Set CS high
}

/*!
 *    @brief  Assert/Deassert the CS pin if it is defined
 *    @param  value The state the CS is set to
 */
void Adafruit_SPIDevice::setChipSelect(int value)
{
  if (_cs != -1)
  {
    gpio_put(_cs, value);
  }
}

/*!
 *    @brief  Write a buffer or two to the SPI device, with transaction
 * management.
 *    @brief  Manually begin a transaction (calls beginTransaction if hardware
 *            SPI) with asserting the CS pin
 */
void Adafruit_SPIDevice::beginTransactionWithAssertingCS()
{
  beginTransaction();
}

/*!
 *    @brief  Manually end a transaction (calls endTransaction if hardware SPI)
 *            with deasserting the CS pin
 */
void Adafruit_SPIDevice::endTransactionWithDeassertingCS()
{
  endTransaction();
}

/*!
 *    @brief  Write a buffer or two to the SPI device, with transaction
 * management.
 *    @param  buffer Pointer to buffer of data to write
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
bool Adafruit_SPIDevice::write(const uint8_t *buffer, size_t len,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len)
{
  beginTransaction();
  if (prefix_buffer != nullptr && prefix_len > 0)
  {
    spi_write_blocking(_spi, prefix_buffer, prefix_len);
  }
  int result = spi_write_blocking(_spi, buffer, len);
  endTransaction();
  return result >= 0;
}

/*!
 *    @brief  Read from SPI into a buffer from the SPI device, with transaction
 * management.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  sendvalue The 8-bits of data to write when doing the data read,
 * defaults to 0xFF
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
bool Adafruit_SPIDevice::read(uint8_t *buffer, size_t len, uint8_t sendvalue)
{
  beginTransaction();
  int result = spi_read_blocking(_spi, sendvalue, buffer, len);
  endTransaction();
  return result >= 0;
}

/*!
 *    @brief  Write some data, then read some data from SPI into another buffer,
 * with transaction management. The buffers can point to same/overlapping
 * locations. This does not transmit-receive at the same time!
 *    @param  write_buffer Pointer to buffer of data to write from
 *    @param  write_len Number of bytes from buffer to write.
 *    @param  read_buffer Pointer to buffer of data to read into.
 *    @param  read_len Number of bytes from buffer to read.
 *    @param  sendvalue The 8-bits of data to write when doing the data read,
 * defaults to 0xFF
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
bool Adafruit_SPIDevice::write_then_read(const uint8_t *write_buffer,
                                         size_t write_len, uint8_t *read_buffer,
                                         size_t read_len, uint8_t sendvalue)
{
  beginTransaction();
  int result = spi_write_blocking(_spi, write_buffer, write_len);
  if (result < 0)
  {
    endTransaction();
    return false;
  }
  result = spi_read_blocking(_spi, sendvalue, read_buffer, read_len);
  endTransaction();
  return result >= 0;
}

/*!
 *    @brief  Write some data and read some data at the same time from SPI
 * into the same buffer, with transaction management. This is basicaly a wrapper
 * for transfer() with CS-pin and transaction management. This /does/
 * transmit-receive at the same time!
 *    @param  buffer Pointer to buffer of data to write/read to/from
 *    @param  len Number of bytes from buffer to write/read.
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
bool Adafruit_SPIDevice::write_and_read(uint8_t *buffer, size_t len)
{
  beginTransaction();
  int result = spi_write_read_blocking(_spi, buffer, buffer, len);
  endTransaction();
  return result >= 0;
}
