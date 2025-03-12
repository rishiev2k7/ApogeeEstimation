/*
 * gps_nmea.c
 *
 *  Created on: Mar 11, 2025
 *      Author: rishi
 */

#include "gps_nmea.h"
#include <string.h>
#include <stdlib.h>

/* Very minimal approach:
   - Look for "$GPGGA" or "$GPGSV"
   - Extract altitude
   - Vertical velocity might come from "$GPVTG" or specialized messages
*/
bool GPS_CheckForNewSentence(uint8_t *rxBuf)
{
  /* Pseudocode: check if there's a '$' and a '\n' in the buffer.
     In practice, a ring buffer or advanced method is recommended. */
  char *start = strstr((char*)rxBuf, "$");
  char *end   = strstr((char*)rxBuf, "\r\n");
  if(start && end && (end > start)) {
    return true;
  }
  return false;
}

bool GPS_ParseNMEA(uint8_t *rxBuf, GPS_Data_t *gpsData)
{
  /* Minimal approach: look for $GPGGA, parse altitude field
     This is purely demonstration code. Real code must handle
     multiple sentences, error checks, etc. */
  char *gga = strstr((char*)rxBuf, "$GPGGA");
  if(!gga) {
    return false;
  }
  /* Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
       altitude = 545.4
  */
  char *altPtr = NULL;
  // We do naive token parsing.
  // GGA fields: time, lat, N/S, lon, E/W, fix, sats, HDOP, alt, altUnit, geoid, ...
  int commaCount = 0;
  char *p = gga;
  while(*p && *p != '\n' && *p != '\r')
  {
    if(*p == ',') {
      commaCount++;
      if(commaCount == 9) {
        altPtr = p+1; // altitude field
        break;
      }
    }
    p++;
  }
  if(!altPtr) {
    return false;
  }

  float altVal = atof(altPtr); // naive parse
  gpsData->altitude = altVal;
  gpsData->verticalVel = 0.0f; // for now, not extracted from NMEA

  return true;
}

