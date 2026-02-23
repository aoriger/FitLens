#ifndef GPS_H
#define GPS_H

#include <stdint.h>

#define GPS_RX_BUF_SIZE   512   // DMA buffer size
#define GPS_BUFFER_SIZE   128   // Max NMEA sentence length


extern uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];
extern uint8_t hour, minute, second;
extern uint8_t satellites;
extern double latitude, longitude;
extern float speed_knots, speed_mph;
extern float altitude_m;

void gps_push_char(uint8_t c);
void gps_process_data(uint8_t *buf, uint16_t len);
void parse_GPGGA(char *sentence);
void parse_GPRMC(char *sentence);
double nmea_to_decimal(const char *str, char dir);

#endif /* GPS_H */
