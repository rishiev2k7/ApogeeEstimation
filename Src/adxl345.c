/*
 * adxl345.c
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */
#include "adxl345.h"

/* Static variables to store handle references */
static SPI_HandleTypeDef *_adxlSpi;
static GPIO_TypeDef* _adxlCsPort;
static uint16_t _adxlCsPin;

/* Helper macros for SPI chip select */
static inline void ADXL345_CS_Select(void)
{
  HAL_GPIO_WritePin(_adxlCsPort, _adxlCsPin, GPIO_PIN_RESET);
}
static inline void ADXL345_CS_Deselect(void)
{
  HAL_GPIO_WritePin(_adxlCsPort, _adxlCsPin, GPIO_PIN_SET);
}

/* Low-level read/write via SPI */
static void ADXL345_WriteReg(uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = { reg, data };
  ADXL345_CS_Select();
  HAL_SPI_Transmit(_adxlSpi, buf, 2, 100);
  ADXL345_CS_Deselect();
}

/* For multi-byte read, set bit 0x80 for read, 0x40 for multi-byte. e.g. reg | 0xC0 */
static void ADXL345_ReadMulti(uint8_t startReg, uint8_t *rxBuf, uint8_t len)
{
  uint8_t regAddr = startReg | 0xC0;
  ADXL345_CS_Select();
  HAL_SPI_Transmit(_adxlSpi, &regAddr, 1, 100);
  HAL_SPI_Receive(_adxlSpi, rxBuf, len, 100);
  ADXL345_CS_Deselect();
}

void ADXL345_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin)
{
  _adxlSpi = hspi;
  _adxlCsPort = csPort;
  _adxlCsPin = csPin;

  /* Deselect at start */
  ADXL345_CS_Deselect();

  /* Check Device ID (optional) */
  uint8_t devid = 0;
  ADXL345_ReadMulti(ADXL345_REG_DEVID, &devid, 1);
  // Ideally, check if devid == 0xE5

  /* Configure data format: +/- 16g (0x01 = +/-2g, 0x0B = +/-16g in FULL_RES mode, etc.) */
  ADXL345_WriteReg(ADXL345_REG_DATA_FORMAT, 0x0B);

  /* Power on: set Measure bit (0x08) in POWER_CTL (0x2D) */
  ADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08);

  /* Could configure ODR or other registers if needed */
}

void ADXL345_ReadAccel(float accel[3])
{
  /* Read 6 bytes: DATAX0..DATAZ1 */
  uint8_t raw[6];
  ADXL345_ReadMulti(ADXL345_REG_DATAX0, raw, 6);

  /* Each axis is 16-bit little-endian. Convert to int16_t. */
  int16_t x = (raw[1] << 8) | raw[0];
  int16_t y = (raw[3] << 8) | raw[2];
  int16_t z = (raw[5] << 8) | raw[4];

  /* ADXL345 in FULL_RES => 4 mg/LSB => 0.004 g/LSB.
     So we multiply by 0.004 to get g. */
  const float scale = 0.004f;
  accel[0] = x * scale;
  accel[1] = y * scale;
  accel[2] = z * scale;
}


