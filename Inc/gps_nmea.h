/*
 * gps_nmea.h
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */

#ifndef INC_GPS_NMEA_H_
#define INC_GPS_NMEA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float latitude;
  float longitude;
  float altitude;
  float verticalVel;
} GPS_Data_t;

/* Basic parser interface */
bool GPS_CheckForNewSentence(uint8_t *rxBuf);
bool GPS_ParseNMEA(uint8_t *rxBuf, GPS_Data_t *gpsData);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPS_NMEA_H_ */
