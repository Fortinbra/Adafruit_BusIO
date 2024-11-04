#ifndef Adafruit_SPIDevice_h
#define Adafruit_SPIDevice_h

#include "hardware/spi.h"
#include "hardware/gpio.h"

enum
{
  SPI_MODE0,
  SPI_MODE1,
  SPI_MODE2,
  _SPI_MODE4
};

typedef enum _BitOrder
{
  SPI_BITORDER_MSBFIRST = SPI_MSB_FIRST,
  SPI_BITORDER_LSBFIRST = SPI_LSB_FIRST,
} BusIOBitOrder;

typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;

/**! The class which defines how we will talk to this device over SPI **/
class Adafruit_SPIDevice
{
public:
  Adafruit_SPIDevice(int8_t cspin, uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0, spi_inst_t *theSPI = spi0);
  Adafruit_SPIDevice(int8_t cspin, int8_t sck, int8_t miso, int8_t mosi,
                     uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0);
  ~Adafruit_SPIDevice();

  bool begin(void);
  bool read(uint8_t *buffer, size_t len, uint8_t sendvalue = 0xFF);
  bool write(const uint8_t *buffer, size_t len,
             const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
  bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       uint8_t sendvalue = 0xFF);
  bool write_and_read(uint8_t *buffer, size_t len);

  uint8_t transfer(uint8_t send);
  void transfer(uint8_t *buffer, size_t len);
  void beginTransaction(void);
  void endTransaction(void);
  void beginTransactionWithAssertingCS();
  void endTransactionWithDeassertingCS();

private:
  spi_inst_t *_spi = nullptr;
  uint32_t _freq;
  BusIOBitOrder _dataOrder;
  uint8_t _dataMode;
  void setChipSelect(int value);

  int8_t _cs, _sck, _mosi, _miso;
  BusIO_PortReg *mosiPort, *clkPort, *misoPort, *csPort;
  BusIO_PortMask mosiPinMask, misoPinMask, clkPinMask, csPinMask;
  bool _begun;
};

#endif // Adafruit_SPIDevice_h
