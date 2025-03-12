/*
 * dps310.h
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */

#ifndef INC_DPS310_H_
#define INC_DPS310_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* DPS310 register definitions (simplified) */
#define DPS310_REG_PRS_B2  0x00
#define DPS310_REG_TMP_B2  0x03
#define DPS310_REG_CFG     0x06
#define DPS310_REG_CFG_TMP 0x07
/* ... More if needed */

void DPS310_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin);
float DPS310_ReadAltitude(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DPS310_H_ */
