#include "panda.h"

void makePANDA(char *panda, const size_t len, const UBX_NAV_PVT_data_t *pvt, const int8_t latHp, const int8_t lonHp, const float yaw, const float pitch, const float roll)
{
	double lat = ((double)pvt->lat) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
	lat += ((double)latHp) / 1000000000.0;		  // Now add the high resolution component (degrees * 10^-9 )
	double lon = ((double)pvt->lon) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
	lon += ((double)lonHp) / 1000000000.0;		  // Now add the high resolution component (degrees * 10^-9 )

	uint8_t latDegrees = lat;
	double latMinutes = ((lat) - (double)latDegrees) * 60.0f;

	uint8_t lonDegrees = lon;
	double lonMinutes = ((lon) - (double)lonDegrees) * 60.0f;

	char lonLetter = (pvt->lon > 0) ? 'E' : 'W';
	char latLetter = (pvt->lat > 0) ? 'N' : 'S';

	uint8_t fixType = 0;
	if (pvt->flags.bits.gnssFixOK)
	{
		fixType = 1;
	}
	if (pvt->flags.bits.diffSoln)
	{
		fixType = 2;
	}
	if (pvt->flags.bits.carrSoln == 1)
	{
		fixType = 5;
	}
	if (pvt->flags.bits.carrSoln == 2)
	{
		fixType = 4;
	}

	float age = 0.0;
	uint8_t lastCorrectionAge = pvt->flags3.all >> 1;
	switch (lastCorrectionAge)
	{
	case 1:
		age = 1.0;
		break;
	case 2:
		age = 2.0;
		break;
	case 3:
		age = 5.0;
		break;
	case 4:
		age = 10.0;
		break;
	case 5:
		age = 15.0;
		break;
	case 6:
		age = 20.0;
		break;
	case 7:
		age = 30.0;
		break;
	case 8:
		age = 45.0;
		break;
	case 9:
		age = 60.0;
		break;
	case 10:
		age = 90.0;
		break;
	case 11:
		age = 120.0;
		break;
	default:
		break;
	}

	snprintf(panda, len, "$PANDA,%02u%02u%02u.%02u,%02u%2.8f,%c,%03u%3.8f,%c,%u,%u,%.1f,%.2f,%.1f,%.1f,%04u,%02i,%02i,%02u*",
			 pvt->hour,
			 pvt->min,
			 pvt->sec,
			 (uint8_t)((pvt->iTOW % 1000) / 10),
			 latDegrees,
			 latMinutes,
			 latLetter,
			 lonDegrees,
			 lonMinutes,
			 lonLetter,
			 fixType,
			 pvt->numSV,
			 (float)pvt->pDOP * 0.01,
			 (float)pvt->hMSL / 1000.0,
			 age,
			 (float)pvt->gSpeed * 0.00194384,
			 (uint16_t)(yaw * 10),
			 (int16_t)(roll * 10),
			 (int16_t)(pitch * 10),
			 0);

	int16_t sum = 0, inx;
	char tmp;

	// The checksum calc starts after '$' and ends before '*'
	for (inx = 1; inx < 200; inx++)
	{
		tmp = panda[inx];
		// * Indicates end of data and start of checksum
		if (tmp == '*')
			break;
		sum ^= tmp; // Build checksum
	}

	sprintf(panda + strlen(panda), "%02X\r\n", sum);
}