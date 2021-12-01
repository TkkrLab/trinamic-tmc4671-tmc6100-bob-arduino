#include <SPI.h>

#define SPI_BUS_INST    VSPI
#define PIN_SPI_SCK     18
#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    23
#define PIN_SPI_CS_CTRL 5
#define PIN_SPI_CS_DRV  17
#define SPI_DEV_CTRL    0
#define SPI_DEV_DRV     1

static const int spi_clk = 100000; // 100kHz
static const int spi_devices[] = {PIN_SPI_CS_CTRL, PIN_SPI_CS_DRV};

SPIClass* spi = NULL;

void spi_write(int device, uint8_t reg, uint32_t value) {
  spi->beginTransaction(SPISettings(spi_clk, MSBFIRST, SPI_MODE3));
  digitalWrite(spi_devices[device], LOW);
  spi->transfer(reg | 0x80);
  uint8_t value_be[4];
  value_be[0] = (value >> 24) & 0xFF;
  value_be[1] = (value >> 16) & 0xFF;
  value_be[2] = (value >>  8) & 0xFF;
  value_be[3] = (value >>  0) & 0xFF;
  spi->transfer((uint8_t*) &value_be, 4);
  digitalWrite(spi_devices[device], HIGH);
  spi->endTransaction();
}

uint32_t spi_read(int device, uint8_t reg) {
  spi->beginTransaction(SPISettings(spi_clk, MSBFIRST, SPI_MODE3));
  digitalWrite(spi_devices[device], LOW);
  spi->transfer(reg & 0x7F);
  uint8_t value_be[4] = {0,0,0,0};
  spi->transfer((uint8_t*) &value_be, 4);
  digitalWrite(spi_devices[device], HIGH);
  spi->endTransaction();
  return (value_be[0]<<24) | (value_be[1]<<16) | (value_be[2]<<8) | value_be[3];
}

void setup() {
  Serial.begin(115200);
  spi = new SPIClass(VSPI);
  spi->begin(
    PIN_SPI_SCK,
    PIN_SPI_MISO,
    PIN_SPI_MOSI,
    -1
  );
  digitalWrite(PIN_SPI_CS_CTRL, HIGH);
  digitalWrite(PIN_SPI_CS_DRV, HIGH);
  pinMode(PIN_SPI_CS_CTRL, OUTPUT);
  pinMode(PIN_SPI_CS_DRV, OUTPUT);
}

void readDrv(uint8_t addr) {
  uint32_t value = spi_read(SPI_DEV_DRV, addr);
  Serial.println("DRV  " + String(addr, HEX) + ": " + String(value, HEX));
}

void loop() {
  spi_write(SPI_DEV_DRV, 0, 0x00000044);
  readDrv(0x00);
  readDrv(0x01);
  readDrv(0x04);
  readDrv(0x07);
  readDrv(0x08);
  readDrv(0x09);
  readDrv(0x0A);
  
  
  Serial.print("CTRL ");
  spi_write(SPI_DEV_CTRL, 1, 0);
  uint32_t hw_type = spi_read(SPI_DEV_CTRL, 0);
  char hw_type_str[5];
  hw_type_str[3] = hw_type & 0xFF;
  hw_type_str[2] = (hw_type >> 8) & 0xFF;
  hw_type_str[1] = (hw_type >> 16) & 0xFF;
  hw_type_str[0] = (hw_type >> 24) & 0xFF;
  hw_type_str[4] = 0;
  Serial.print(String(hw_type_str) + " ");
  for (uint8_t i = 1; i < 6; i++) {
    spi_write(SPI_DEV_CTRL, 1, i);
    uint32_t info = spi_read(SPI_DEV_CTRL, 0);
    Serial.print(info, HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
}
