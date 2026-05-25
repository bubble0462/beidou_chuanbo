#ifndef APP_LOCATION_REPORT_H
#define APP_LOCATION_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define APP_LOCATION_REPORT_TEXT_MAX_LEN        78U
#define APP_LOCATION_REPORT_DEFAULT_INTERVAL_MS 120000UL
#define APP_LOCATION_REPORT_MAX_POINTS          160U

typedef enum
{
  APP_LOCATION_REPORT_MODE_DEFAULT = 0,
  APP_LOCATION_REPORT_MODE_TIME,
  APP_LOCATION_REPORT_MODE_DISTANCE
} app_location_report_mode_t;

typedef struct
{
  uint32_t lat_nmea;
  uint32_t lon_nmea;
  char lat_dir;
  char lon_dir;
} app_location_point_t;

typedef struct
{
  app_location_report_mode_t mode;
  uint32_t interval_ms;
  uint32_t distance_threshold_m;
  uint32_t accumulated_distance_m;
  uint32_t last_report_ms;
  uint8_t has_last_point;
  app_location_point_t last_point;
  uint16_t point_count;
  app_location_point_t points[APP_LOCATION_REPORT_MAX_POINTS];
} app_location_reporter_t;


void AppLocationReport_Init(app_location_reporter_t *reporter, uint32_t now_ms);
uint8_t AppLocationReport_SetDefault(app_location_reporter_t *reporter, uint32_t now_ms);
uint8_t AppLocationReport_SetTimeMinutes(app_location_reporter_t *reporter,
                                         uint16_t minutes,
                                         uint32_t now_ms);
uint8_t AppLocationReport_SetTimeSeconds(app_location_reporter_t *reporter,
                                         uint32_t seconds,
                                         uint32_t now_ms);
uint8_t AppLocationReport_SetDistanceMeters(app_location_reporter_t *reporter,
                                            uint32_t meters,
                                            uint32_t now_ms);
void AppLocationReport_FormatStatus(const app_location_reporter_t *reporter,
                                    char *buffer,
                                    size_t buffer_size);

uint8_t AppLocationReport_ParseRmc(const char *sentence, app_location_point_t *point);
uint32_t AppLocationReport_DistanceMeters(const app_location_point_t *from,
                                          const app_location_point_t *to);
uint16_t AppLocationReport_BuildPacket(const app_location_point_t *points,
                                       uint16_t point_count,
                                       char *packet,
                                       size_t packet_size);
uint8_t AppLocationReport_ProcessRmc(app_location_reporter_t *reporter,
                                     const char *sentence,
                                     uint32_t now_ms,
                                     char *packet,
                                     size_t packet_size);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOCATION_REPORT_H */
