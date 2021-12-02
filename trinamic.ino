/*
MIT License

Copyright (c) 2021 Stichting TkkrLab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <SPI.h>

#include "TMC6100_Register.h"
#include "TMC6100_Fields.h"
#include "TMC6100_Constants.h"

#include "TMC4671_Register.h"
#include "TMC4671_Fields.h"
#include "TMC4671_Constants.h"
#include "TMC4671_Variants.h"

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

  initDriver();

  // Motor type &  PWM configuration
  spi_write(SPI_DEV_CTRL, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
  spi_write(SPI_DEV_CTRL, TMC4671_PWM_POLARITIES, 0x00000000);
  spi_write(SPI_DEV_CTRL, TMC4671_PWM_MAXCNT, 0x000003E7);
  spi_write(SPI_DEV_CTRL, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
  spi_write(SPI_DEV_CTRL, TMC4671_PWM_SV_CHOP, 0x00000007);
  
  // ADC configuration
  spi_write(SPI_DEV_CTRL, TMC4671_ADC_I_SELECT, 0x18000100);
  spi_write(SPI_DEV_CTRL, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
  spi_write(SPI_DEV_CTRL, TMC4671_dsADC_MCLK_A, 0x0051EB85);
  spi_write(SPI_DEV_CTRL, TMC4671_dsADC_MCLK_B, 0x0051EB85);
  spi_write(SPI_DEV_CTRL, TMC4671_dsADC_MDEC_B_MDEC_A, 0x00540054);
  spi_write(SPI_DEV_CTRL, TMC4671_ADC_I0_SCALE_OFFSET, 0x010081CB);
  spi_write(SPI_DEV_CTRL, TMC4671_ADC_I1_SCALE_OFFSET, 0x010081F9);

  // Digital hall settings
  spi_write(SPI_DEV_CTRL,  TMC4671_HALL_MODE, 0x00001001);
  spi_write(SPI_DEV_CTRL,  TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x1F400000);
  
  // ABN encoder settings
  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_MODE, 0x00000000);
  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PPR, 0x00000800);
  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_COUNT, 0x00000539);
  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
  
  // Limits
  spi_write(SPI_DEV_CTRL, TMC4671_PID_TORQUE_FLUX_LIMITS, 2000);
  
  // PI settings
  spi_write(SPI_DEV_CTRL, TMC4671_PID_TORQUE_P_TORQUE_I, (400 << 16) | 20);
  spi_write(SPI_DEV_CTRL, TMC4671_PID_FLUX_P_FLUX_I, (50 << 16) | 10);
  spi_write(SPI_DEV_CTRL, TMC4671_PID_VELOCITY_P_VELOCITY_I, (400 << 16) | 50);

  // Init encoder (mode 0)
  spi_write(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
  spi_write(SPI_DEV_CTRL, TMC4671_PHI_E_SELECTION, 0x00000001);
  spi_write(SPI_DEV_CTRL, TMC4671_PHI_E_EXT, 0x00000000);
  spi_write(SPI_DEV_CTRL, TMC4671_UQ_UD_EXT, 0x000007D0);
  delay(100);

  spi_write(SPI_DEV_CTRL, TMC4671_ABN_DECODER_COUNT, 0x00000000);

  // Feedback selection
  spi_write(SPI_DEV_CTRL, TMC4671_PHI_E_SELECTION, 5); // 3 is encoder, 5 is hal
  spi_write(SPI_DEV_CTRL, TMC4671_VELOCITY_SELECTION, 5);

  // Stop mode
  spi_write(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000000);
}

void readDrv(uint8_t addr) {
  uint32_t value = spi_read(SPI_DEV_DRV, addr);
  Serial.println("DRV  " + String(addr, HEX) + ": " + String(value, HEX));
}

void setTorque(int16_t torque, int16_t flux) {
  spi_write(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 1);
  spi_write(SPI_DEV_CTRL, TMC4671_PID_TORQUE_FLUX_TARGET, (torque << 16) | flux);
}

void setVelocity(int16_t velocity) {
  spi_write(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 2);
  spi_write(SPI_DEV_CTRL, TMC4671_PID_VELOCITY_TARGET, velocity);
}

void setStop() {
  spi_write(SPI_DEV_CTRL, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000000);
}

void setVelocityPi(uint16_t p, uint16_t i) {
  spi_write(SPI_DEV_CTRL, TMC4671_PID_VELOCITY_P_VELOCITY_I, (p << 16) | i);
}

void initDriver() {
  // Initialize the driver chip
  spi_write(SPI_DEV_DRV, TMC6100_GCONF,
    (0 << TMC6100_DISABLE_SHIFT)      | // Enable
    (0 << TMC6100_SINGLELINE_SHIFT)   | // Use individual L+H signals
    (1 << TMC6100_FAULTDIRECT_SHIFT)  | // Fault output shows each protective action
    (1 << TMC6100_CURRENT_ZERO_SHIFT)   // Disable current amplifier
  );
}

String serialInput = "";

int16_t  t = 0;
int16_t  f = 0;

int16_t  v = 0;
uint16_t p = 600; // Velocity P
uint16_t i = 100; // Velocity I

void parseSerial() {
  if (serialInput.length() < 1) return;
  char c = serialInput[0];
  serialInput.remove(0,1);
  switch(c) {
    case 'r':
      initDriver();
      break;
    case 'v':
      v = serialInput.toInt();
      setVelocity(v);
      break;
    case 't':
      t = serialInput.toInt();
      setTorque(t, f);
      break;
    case 'f':
      f = serialInput.toInt();
      setTorque(t, f);
      break;
    case 'p':
      p = serialInput.toInt();
      setVelocityPi(p, i);
      break;
    case 'i':
      i = serialInput.toInt();
      setVelocityPi(p, i);
      break;
  }
  Serial.println("V = "+ String(v, DEC) + ", P = " + String(p, DEC) + ", I = " + String(i, DEC));
  serialInput = "";
}

void checkSerial() {
  while (Serial.available()>0) {
    char c = Serial.read();
    if ((c >= ' ') && (c <= '~')) {
      serialInput += c;
    } else if ((c == '\n') || (c == '\r')) {
      parseSerial();
    } else {
      serialInput = "";
      Serial.println("#Received garbage, buffer cleared");
    }
  }
}

void loop() {
  checkSerial();
  /*Serial.println("Rotate right");
  setTorque(200, 0);
  delay(2000);
  Serial.println("Stop");
  setStop();
  delay(2000);
  Serial.println("Rotate left");
  setTorque(-200, 0);
  delay(2000);
  Serial.println("Stop");
  setStop();
  delay(2000);*/

  /*setVelocity(1000);
  delay(2000);
  setVelocity(0);
  delay(2000);*/

  // Switch to stop mode
  /*Serial.println("Stop mode");
  setStop();
  delay(2000);*/
  /*Serial.print("CTRL ");
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
  delay(100);*/
}
