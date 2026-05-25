#include "app_location_report.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef APP_LOCATION_PI
#define APP_LOCATION_PI 3.14159265358979323846
#endif

#define APP_LOCATION_EARTH_RADIUS_M      6371000.0
#define APP_LOCATION_MIN_TIME_SECONDS    60UL
#define APP_LOCATION_MIN_DISTANCE_METERS 500U
#define APP_LOCATION_MAX_RMC_LENGTH      127U

static uint8_t AppLocation_GetField(const char *sentence,
                                    uint8_t target_index,
                                    char *field,
                                    size_t field_size);
static uint8_t AppLocation_IsRmcSentence(const char *sentence);
static uint8_t AppLocation_ParseCoordinate(const char *value,
                                           char direction,
                                           uint32_t *nmea_raw);
static double AppLocation_NmeaToDecimalDeg(uint32_t nmea_raw, char dir);
static void AppLocation_ResetWindow(app_location_reporter_t *reporter, uint32_t now_ms);
static void AppLocation_StorePoint(app_location_reporter_t *reporter,
                                   const app_location_point_t *point);

void AppLocationReport_Init(app_location_reporter_t *reporter, uint32_t now_ms)
{
  if (reporter == NULL)
  {
    return;
  }

  memset(reporter, 0, sizeof(*reporter));
  reporter->mode = APP_LOCATION_REPORT_MODE_DEFAULT;
  reporter->interval_ms = APP_LOCATION_REPORT_DEFAULT_INTERVAL_MS;
  reporter->last_report_ms = now_ms;
}

uint8_t AppLocationReport_SetDefault(app_location_reporter_t *reporter, uint32_t now_ms)
{
  if (reporter == NULL)
  {
    return 0U;
  }

  reporter->mode = APP_LOCATION_REPORT_MODE_DEFAULT;
  reporter->interval_ms = APP_LOCATION_REPORT_DEFAULT_INTERVAL_MS;
  reporter->distance_threshold_m = 0U;
  AppLocation_ResetWindow(reporter, now_ms);
  return 1U;
}

uint8_t AppLocationReport_SetTimeMinutes(app_location_reporter_t *reporter,
                                         uint16_t minutes,
                                         uint32_t now_ms)
{
  return AppLocationReport_SetTimeSeconds(reporter, (uint32_t)minutes * 60UL, now_ms);
}

uint8_t AppLocationReport_SetTimeSeconds(app_location_reporter_t *reporter,
                                         uint32_t seconds,
                                         uint32_t now_ms)
{
  if ((reporter == NULL) ||
      (seconds < APP_LOCATION_MIN_TIME_SECONDS) ||
      (seconds > (0xFFFFFFFFUL / 1000UL)))
  {
    return 0U;
  }

  reporter->mode = APP_LOCATION_REPORT_MODE_TIME;
  reporter->interval_ms = seconds * 1000UL;
  reporter->distance_threshold_m = 0U;
  AppLocation_ResetWindow(reporter, now_ms);
  return 1U;
}

uint8_t AppLocationReport_SetDistanceMeters(app_location_reporter_t *reporter,
                                            uint32_t meters,
                                            uint32_t now_ms)
{
  if ((reporter == NULL) || (meters < APP_LOCATION_MIN_DISTANCE_METERS))
  {
    return 0U;
  }

  reporter->mode = APP_LOCATION_REPORT_MODE_DISTANCE;
  reporter->distance_threshold_m = meters;
  reporter->interval_ms = APP_LOCATION_REPORT_DEFAULT_INTERVAL_MS;
  AppLocation_ResetWindow(reporter, now_ms);
  return 1U;
}

void AppLocationReport_FormatStatus(const app_location_reporter_t *reporter,
                                    char *buffer,
                                    size_t buffer_size)
{
  if ((reporter == NULL) || (buffer == NULL) || (buffer_size == 0U))
  {
    return;
  }

  switch (reporter->mode)
  {
    case APP_LOCATION_REPORT_MODE_TIME:
      (void)snprintf(buffer,
                     buffer_size,
                     "Report mode: time, interval: %lu s\r\n",
                     (unsigned long)(reporter->interval_ms / 1000UL));
      break;

    case APP_LOCATION_REPORT_MODE_DISTANCE:
      (void)snprintf(buffer,
                     buffer_size,
                     "Report mode: distance, threshold: %lu m\r\n",
                     (unsigned long)reporter->distance_threshold_m);
      break;

    default:
      (void)snprintf(buffer, buffer_size, "Report default: 2 min\r\n");
      break;
  }
}

uint8_t AppLocationReport_ParseRmc(const char *sentence, app_location_point_t *point)
{
  char status[4];
  char lat_value[20];
  char lat_direction[4];
  char lon_value[20];
  char lon_direction[4];
  uint32_t lat_nmea = 0U;
  uint32_t lon_nmea = 0U;

  if ((sentence == NULL) || (point == NULL) || (AppLocation_IsRmcSentence(sentence) == 0U))
  {
    return 0U;
  }

  if ((AppLocation_GetField(sentence, 2U, status, sizeof(status)) == 0U) ||
      (status[0] != 'A'))
  {
    return 0U;
  }

  if ((AppLocation_GetField(sentence, 3U, lat_value, sizeof(lat_value)) == 0U) ||
      (AppLocation_GetField(sentence, 4U, lat_direction, sizeof(lat_direction)) == 0U) ||
      (AppLocation_GetField(sentence, 5U, lon_value, sizeof(lon_value)) == 0U) ||
      (AppLocation_GetField(sentence, 6U, lon_direction, sizeof(lon_direction)) == 0U))
  {
    return 0U;
  }

  if ((AppLocation_ParseCoordinate(lat_value, lat_direction[0], &lat_nmea) == 0U) ||
      (AppLocation_ParseCoordinate(lon_value, lon_direction[0], &lon_nmea) == 0U))
  {
    return 0U;
  }

  point->lat_nmea = lat_nmea;
  point->lon_nmea = lon_nmea;
  point->lat_dir = lat_direction[0];
  point->lon_dir = lon_direction[0];
  return 1U;
}

uint32_t AppLocationReport_DistanceMeters(const app_location_point_t *from,
                                          const app_location_point_t *to)
{
  double lat1 = 0.0;
  double lat2 = 0.0;
  double d_lat = 0.0;
  double d_lon = 0.0;
  double a = 0.0;
  double c = 0.0;
  double distance = 0.0;

  if ((from == NULL) || (to == NULL))
  {
    return 0U;
  }

  lat1 = AppLocation_NmeaToDecimalDeg(from->lat_nmea, from->lat_dir) * APP_LOCATION_PI / 180.0;
  lat2 = AppLocation_NmeaToDecimalDeg(to->lat_nmea, to->lat_dir) * APP_LOCATION_PI / 180.0;
  d_lat = (AppLocation_NmeaToDecimalDeg(to->lat_nmea, to->lat_dir) -
           AppLocation_NmeaToDecimalDeg(from->lat_nmea, from->lat_dir)) * APP_LOCATION_PI / 180.0;
  d_lon = (AppLocation_NmeaToDecimalDeg(to->lon_nmea, to->lon_dir) -
           AppLocation_NmeaToDecimalDeg(from->lon_nmea, from->lon_dir)) * APP_LOCATION_PI / 180.0;

  a = sin(d_lat / 2.0) * sin(d_lat / 2.0) +
      cos(lat1) * cos(lat2) * sin(d_lon / 2.0) * sin(d_lon / 2.0);
  c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  distance = APP_LOCATION_EARTH_RADIUS_M * c;

  return (uint32_t)(distance + 0.5);
}

uint16_t AppLocationReport_BuildPacket(const app_location_point_t *points,
                                       uint16_t point_count,
                                       char *packet,
                                       size_t packet_size)
{
  uint16_t idx[3];
  uint8_t n = 0U;
  uint8_t i = 0U;
  size_t used = 0U;
  int written = 0;

  if ((points == NULL) || (point_count == 0U) || (packet == NULL) || (packet_size == 0U))
  {
    return 0U;
  }

  if (point_count == 1U)
  {
    idx[0] = 0U;
    n = 1U;
  }
  else if (point_count == 2U)
  {
    idx[0] = 0U;
    idx[1] = 1U;
    n = 2U;
  }
  else
  {
    idx[0] = 0U;
    idx[1] = point_count / 2U;
    idx[2] = point_count - 1U;
    n = 3U;
  }

  written = snprintf(packet, packet_size, "D|");
  if ((written != 2) || (packet_size <= 2U))
  {
    return 0U;
  }
  used = 2U;

  for (i = 0U; i < n; i++)
  {
    const app_location_point_t *p = &points[idx[i]];

    if (i > 0U)
    {
      if ((used + 1U) >= packet_size)
      {
        return 0U;
      }
      packet[used++] = ';';
      packet[used] = '\0';
    }

    written = snprintf(&packet[used],
                       packet_size - used,
                       "%09lu%c%010lu%c",
                       (unsigned long)p->lat_nmea,
                       p->lat_dir,
                       (unsigned long)p->lon_nmea,
                       p->lon_dir);
    if ((written != 21) || ((used + 21U) >= packet_size))
    {
      return 0U;
    }
    used += 21U;
  }

  return (uint16_t)used;
}

uint8_t AppLocationReport_ProcessRmc(app_location_reporter_t *reporter,
                                     const char *sentence,
                                     uint32_t now_ms,
                                     char *packet,
                                     size_t packet_size)
{
  app_location_point_t point;
  uint8_t should_report = 0U;
  uint16_t length = 0U;

  if ((reporter == NULL) || (packet == NULL) || (packet_size == 0U))
  {
    return 0U;
  }

  if (AppLocationReport_ParseRmc(sentence, &point) == 0U)
  {
    return 0U;
  }

  if (reporter->has_last_point != 0U)
  {
    reporter->accumulated_distance_m += AppLocationReport_DistanceMeters(&reporter->last_point, &point);
  }

  reporter->last_point = point;
  reporter->has_last_point = 1U;
  AppLocation_StorePoint(reporter, &point);

  if (reporter->point_count == 0U)
  {
    return 0U;
  }

  if (reporter->mode == APP_LOCATION_REPORT_MODE_DISTANCE)
  {
    uint8_t distance_ok = (reporter->accumulated_distance_m >= reporter->distance_threshold_m) ? 1U : 0U;
    uint8_t time_ok = ((uint32_t)(now_ms - reporter->last_report_ms) >= APP_LOCATION_REPORT_DEFAULT_INTERVAL_MS) ? 1U : 0U;
    should_report = distance_ok && time_ok;
  }
  else
  {
    should_report = ((uint32_t)(now_ms - reporter->last_report_ms) >= reporter->interval_ms) ? 1U : 0U;
  }

  if (should_report == 0U)
  {
    return 0U;
  }

  length = AppLocationReport_BuildPacket(reporter->points,
                                         reporter->point_count,
                                         packet,
                                         packet_size);
  if (length == 0U)
  {
    return 0U;
  }

  AppLocation_ResetWindow(reporter, now_ms);
  return 1U;
}

static uint8_t AppLocation_GetField(const char *sentence,
                                    uint8_t target_index,
                                    char *field,
                                    size_t field_size)
{
  uint8_t current_index = 0U;
  size_t out = 0U;
  const char *cursor = sentence;

  if ((sentence == NULL) || (field == NULL) || (field_size == 0U))
  {
    return 0U;
  }

  field[0] = '\0';

  while ((*cursor != '\0') && (*cursor != '\r') && (*cursor != '\n'))
  {
    if ((*cursor == ',') || (*cursor == '*'))
    {
      if (current_index == target_index)
      {
        field[out] = '\0';
        return (out > 0U) ? 1U : 0U;
      }

      current_index++;
      out = 0U;

      if (*cursor == '*')
      {
        break;
      }
    }
    else if (current_index == target_index)
    {
      if (out >= (field_size - 1U))
      {
        return 0U;
      }
      field[out++] = *cursor;
    }

    cursor++;
  }

  if (current_index == target_index)
  {
    field[out] = '\0';
    return (out > 0U) ? 1U : 0U;
  }

  return 0U;
}

static uint8_t AppLocation_IsRmcSentence(const char *sentence)
{
  const char *cursor = sentence;

  if (sentence == NULL)
  {
    return 0U;
  }

  while ((*cursor == ' ') || (*cursor == '\t'))
  {
    cursor++;
  }

  if (*cursor == '$')
  {
    cursor++;
  }

  if (strlen(cursor) < 5U)
  {
    return 0U;
  }

  return ((cursor[2] == 'R') || (cursor[2] == 'r')) &&
         ((cursor[3] == 'M') || (cursor[3] == 'm')) &&
         ((cursor[4] == 'C') || (cursor[4] == 'c')) ? 1U : 0U;
}

static uint8_t AppLocation_ParseCoordinate(const char *value,
                                           char direction,
                                           uint32_t *nmea_raw)
{
  uint32_t raw = 0U;
  const char *p = value;

  if ((value == NULL) || (nmea_raw == NULL) ||
      ((direction != 'N') && (direction != 'S') &&
       (direction != 'E') && (direction != 'W')))
  {
    return 0U;
  }

  while (*p != '\0')
  {
    if ((*p >= '0') && (*p <= '9'))
    {
      raw = raw * 10U + (uint32_t)(*p - '0');
    }
    p++;
  }

  if (raw == 0U)
  {
    return 0U;
  }

  *nmea_raw = raw;
  return 1U;
}

static double AppLocation_NmeaToDecimalDeg(uint32_t nmea_raw, char dir)
{
  uint32_t degrees = nmea_raw / 10000000U;
  uint32_t min_x100000 = nmea_raw % 10000000U;
  double decimal = (double)degrees + ((double)min_x100000 / 100000.0) / 60.0;

  if ((dir == 'S') || (dir == 'W'))
  {
    decimal = -decimal;
  }

  return decimal;
}

static void AppLocation_ResetWindow(app_location_reporter_t *reporter, uint32_t now_ms)
{
  if (reporter == NULL)
  {
    return;
  }

  reporter->point_count = 0U;
  reporter->accumulated_distance_m = 0U;
  reporter->last_report_ms = now_ms;
}

static void AppLocation_StorePoint(app_location_reporter_t *reporter,
                                   const app_location_point_t *point)
{
  if ((reporter == NULL) || (point == NULL))
  {
    return;
  }

  if (reporter->point_count < APP_LOCATION_REPORT_MAX_POINTS)
  {
    reporter->points[reporter->point_count++] = *point;
  }
  else
  {
    reporter->points[APP_LOCATION_REPORT_MAX_POINTS - 1U] = *point;
  }
}
