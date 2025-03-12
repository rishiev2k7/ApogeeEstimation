/*
 * adxl345.h
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // Adjust as needed for the user’s MCU

/* Register definitions (simplified) */
#define ADXL345_REG_DEVID          0x00
#define ADXL345_REG_POWER_CTL      0x2D
#define ADXL345_REG_DATA_FORMAT    0x31
#define ADXL345_REG_DATAX0         0x32 // X, Y, Z data

/* Function prototypes */
void ADXL345_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin);
void ADXL345_ReadAccel(float accel[3]); // returns X/Y/Z in g

#ifdef __cplusplus
}
#endif


#endif /* INC_ADXL345_H_ */
