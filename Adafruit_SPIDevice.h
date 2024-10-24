#ifndef Adafruit_SPIDevice_h
#define Adafruit_SPIDevice_h

#include <hardware/spi.h>

// Define SPI modes for Pico SDK
enum
{
  SPI_MODE0 = 0,
  SPI_MODE1 = 1,
  SPI_MODE2 = 2,
  SPI_MODE3 = 3
};

// Define BitOrder for Pico SDK
typedef enum _BitOrder
{
  SPI_BITORDER_MSBFIRST = 0,
  SPI_BITORDER_LSBFIRST = 1
} BusIOBitOrder;

// Define SPIClass for Pico SDK
typedef spi_inst_t SPIClass;

/**! The class which defines how we will talk to this device over SPI **/
class Adafruit_SPIDevice
{
public:
  Adafruit_SPIDevice(uint8_t cs, uint32_t freq = 1000000, BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST, uint8_t dataMode = SPI_MODE0, SPIClass *theSPI = spi0);

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
  uint32_t _freq;
  BusIOBitOrder _dataOrder;
  uint8_t _dataMode;
  void setChipSelect(int value);
  int8_t _cs, _sck, _mosi, _miso;
  bool _begun;
  SPIClass *_spi;
};

#endif // Adafruit_SPIDevice_h
