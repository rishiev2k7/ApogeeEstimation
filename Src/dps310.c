/*
 * dps310.c
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */
#include "dps310.h"
#include <math.h> // for powf or logf if needed

static SPI_HandleTypeDef *_dpsSpi;
static GPIO_TypeDef* _dpsCsPort;
static uint16_t _dpsCsPin;

/* Chip select helpers */
static inline void DPS310_CS_Select(void)
{
  HAL_GPIO_WritePin(_dpsCsPort, _dpsCsPin, GPIO_PIN_RESET);
}
static inline void DPS310_CS_Deselect(void)
{
  HAL_GPIO_WritePin(_dpsCsPort, _dpsCsPin, GPIO_PIN_SET);
}

static void DPS310_WriteReg(uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = { reg, data };
  DPS310_CS_Select();
  HAL_SPI_Transmit(_dpsSpi, buf, 2, 100);
  DPS310_CS_Deselect();
}

static void DPS310_ReadMulti(uint8_t startReg, uint8_t *rxBuf, uint8_t len)
{
  uint8_t regAddr = startReg | 0x80; // 0x80 for read in some DPS310 versions
  DPS310_CS_Select();
  HAL_SPI_Transmit(_dpsSpi, &regAddr, 1, 100);
  HAL_SPI_Receive(_dpsSpi, rxBuf, len, 100);
  DPS310_CS_Deselect();
}

void DPS310_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin)
{
  _dpsSpi = hspi;
  _dpsCsPort = csPort;
  _dpsCsPin = csPin;

  DPS310_CS_Deselect();

  /* Example config: set pressure oversampling, etc. (simplified) */
  DPS310_WriteReg(DPS310_REG_CFG, 0x26);     // oversampling config
  DPS310_WriteReg(DPS310_REG_CFG_TMP, 0xA6); // temperature config
}

/* A minimal approach: read "pressure" and "temp", convert to altitude.
   Real DPS310 code involves calibration coefficients. We do a placeholder. */
float DPS310_ReadAltitude(void)
{
  /* Placeholder: read raw pressure. The real driver needs calibration. */
  uint8_t raw[3];
  DPS310_ReadMulti(DPS310_REG_PRS_B2, raw, 3);
  int32_t rawP = ((int32_t)raw[0] << 16) | ((int32_t)raw[1] << 8) | raw[2];
  // sign-extend if needed

  /* Convert raw pressure to hPa, then altitude. This is just a demonstration. */
  float pressure_hPa = (float)rawP / 2048.0f; // fake scaling

  /* Convert to altitude using barometric formula (simple):
     altitude = 44330 * (1 - (pressure / p0)^(1/5.255))
     p0 ~ 1013.25 hPa at sea level
  */
  float alt = 44330.0f * (1.0f - powf((pressure_hPa / 1013.25f), 0.1902949f));

  return alt;
}


