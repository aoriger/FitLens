#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];

uint8_t hour, minute, second;
uint8_t satellites;
double latitude, longitude;
float speed_knots, speed_mph;
float altitude_m;

uint32_t time_since_update = 0; // ms
uint32_t HAL_GetTick(void);
uint32_t last_update = 0;

#define START_DISTANCE 10.0f // 10 meters

uint32_t start_time = 0; // ms
uint32_t finish_time = 0;
uint32_t total_time = 0;
bool start_flag = false;
bool dist_start_flag = false;

float distance_traveled = 0.0f;
float last_lat;
float last_lon;
char fix[4];
static bool initialized = false;

float distance_m(float lat1, float lon1,
                        float lat2, float lon2) {

    const float METERS_PER_DEG = 111320.0f;

    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;

    float lat_avg = (lat1 + lat2) * 0.5f * 0.01745329251f; // degree to radian conversion

    float x = dlon * cosf(lat_avg);
    float y = dlat;

    return sqrtf(x*x + y*y) * METERS_PER_DEG;
}

void get_dist_and_time(float lat, float lon) {

	if (lat == 0 || lon == 0) return;  // skip invalid readings

    if (!initialized) {
        last_lat = lat;
        last_lon = lon;
        initialized = true;
        return;
    }

    // compute distance segment
    float segment = distance_m(last_lat, last_lon, lat, lon);

    // deadband: ignore small movements (< 10 m)
    const float MIN_MOVE_METERS = 10.0f;
    if (segment >= MIN_MOVE_METERS) {
        distance_traveled += segment;
        last_lat = lat;
        last_lon = lon;
    }

    if (start_flag) {
    	finish_time = HAL_GetTick();
    }


	// start activity
	if (!start_flag && (distance_traveled > 0)) {
		start_time = HAL_GetTick();
		start_flag = true;
	}

	// total time in seconds
	total_time = (finish_time - start_time) / 1000;
}

void gps_push_char(uint8_t c)
{
    static char gps_buffer[GPS_BUFFER_SIZE];
    static uint16_t gps_index = 0;

    if (c == '\n' || c == '\r') { // if end of sequence
        if (gps_index == 0) return; // skip empty lines
        gps_buffer[gps_index] = '\0'; // add delimiter

        // check sequence start for type
        if (strncmp(gps_buffer, "$GNGGA", 6) == 0) {
            parse_GPGGA(gps_buffer);
        } else if (strncmp(gps_buffer, "$GNRMC", 6) == 0) {
            parse_GPRMC(gps_buffer);
        }
        gps_index = 0;
    }
    else if (gps_index < GPS_BUFFER_SIZE - 1) {
        gps_buffer[gps_index++] = c; // add char to buffer
    }
    else {
        gps_index = 0;
    }
}

void gps_process_data(uint8_t *buf, uint16_t pos)
{
    static uint16_t last_pos = 0;

    if(pos == last_pos) return;

    if(pos < last_pos) { // DMA wrapped
        for(uint16_t i = last_pos; i < GPS_RX_BUF_SIZE; i++)
            gps_push_char(buf[i]);
        last_pos = 0;
    }

    for(uint16_t i = last_pos; i < pos; i++)
        gps_push_char(buf[i]);

    last_pos = pos;
}

void parse_GPGGA(char *sentence)
{
	char* token;
	int field = 0;
	char lat[16], lon[16], ns, ew, alt[16], sat[4];

	token = strtok(sentence, ",");
	while(token) {
		field++;
		switch(field) {
			case 2: break;
			case 3: strcpy(lat, token); break;
			case 4: ns = token[0]; break;
			case 5: strcpy(lon, token); break;
			case 6: ew = token[0]; break;
			case 7: strcpy(fix, token); break;
			case 8: strcpy(sat, token); break;
			case 10: strcpy(alt, token); break;
		}
		token = strtok(NULL, ",");
	}
	latitude = nmea_to_decimal(lat, ns);
	longitude = nmea_to_decimal(lon, ew);
	satellites = (uint8_t)atoi(sat);
	altitude_m = atof(alt);

	int fix_quality = atoi(fix);
	if(fix_quality > 0) {
		time_since_update = HAL_GetTick() - last_update;
	    last_update = HAL_GetTick();
	    get_dist_and_time(latitude, longitude);
	}
}

void parse_GPRMC(char *sentence)
{
	char* token;
	int field = 0;
	char time_str[16], speed[16];

	token = strtok(sentence, ",");
	while(token) {
		field++;
		switch(field) {
			case 2: strcpy(time_str, token); break;
			case 3: strcpy(fix, token); break;
			case 8: strcpy(speed, token); break;
		}
		token = strtok(NULL, ",");
	}
	char h[3], m[3], s[3];
	strncpy(h, time_str, 2); h[2] = '\0';
	strncpy(m, time_str+2, 2); m[2] = '\0';
	strncpy(s, time_str+4, 2); s[2] = '\0';
	hour = atoi(h);
	minute = atoi(m);
	second = atoi(s);
	speed_knots = atof(speed);
	speed_mph = speed_knots * 1.150779;

	if (fix[0] == 'A') {
		time_since_update = HAL_GetTick() - last_update;
    	last_update = HAL_GetTick();
	}
}

// convert NMEA (ddmm.mmmm) to lat/lon degrees
double nmea_to_decimal(const char* nmea, char dir) {
	double val = atof(nmea); // string to double
	int degrees = (int)(val / 100);
	double minutes = val - degrees * 100;
	double dec = degrees + minutes / 60.0;
	if(dir == 'S' || dir == 'W') dec = -dec; // convention-may want to keep letters later
	return dec;
}
