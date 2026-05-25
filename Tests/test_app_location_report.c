#include "app_location_report.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_parse_gbrmc_to_decimal_microdegrees(void)
{
  app_location_point_t point;
  uint8_t ok = AppLocationReport_ParseRmc("$GBRMC,130727.000,A,2603.92438,N,11909.77779,E,0.021,299.974,070126,,,A,S*00",
                                          &point);

  assert(ok == 1U);
  assert(point.lat_nmea == 260392438U);
  assert(point.lon_nmea == 1190977779U);
  assert(point.lat_dir == 'N');
  assert(point.lon_dir == 'E');
}

static void test_parse_rejects_void_rmc(void)
{
  app_location_point_t point;
  uint8_t ok = AppLocationReport_ParseRmc("$GBRMC,130727.000,V,2603.92438,N,11909.77779,E,0.021,299.974,070126,,,A,S*00",
                                          &point);

  assert(ok == 0U);
}

static void test_location_report_limit_is_78_bytes(void)
{
  assert(APP_LOCATION_REPORT_TEXT_MAX_LEN == 78U);
}

static void test_build_packet_uses_three_nmea_points_without_trailing_separator(void)
{
  app_location_point_t points[] =
  {
    {260392438U, 1190977779U, 'N', 'E'},
    {260392279U, 1190977699U, 'N', 'E'},
    {260392260U, 1190977691U, 'N', 'E'},
  };
  char packet[APP_LOCATION_REPORT_TEXT_MAX_LEN + 1U];
  uint16_t length = AppLocationReport_BuildPacket(points, 3U, packet, sizeof(packet));

  assert(length > 0U);
  assert(strcmp(packet, "D|260392438N1190977779E;260392279N1190977699E;260392260N1190977691E") == 0);
  assert(length == 67U);
  assert(length <= APP_LOCATION_REPORT_TEXT_MAX_LEN);
}

static void test_distance_uses_internal_microdegree_coordinates(void)
{
  app_location_point_t from = {260392438U, 1190977779U, 'N', 'E'};
  app_location_point_t to = {260378570U, 1190974952U, 'N', 'E'};
  uint32_t distance = AppLocationReport_DistanceMeters(&from, &to);

  assert(distance >= 260U);
  assert(distance <= 262U);
}

static void test_time_and_distance_settings_validate_lower_bounds(void)
{
  app_location_reporter_t reporter;

  AppLocationReport_Init(&reporter, 1000U);
  assert(AppLocationReport_SetTimeMinutes(&reporter, 0U, 1000U) == 0U);
  assert(AppLocationReport_SetTimeMinutes(&reporter, 1U, 1000U) == 1U);
  assert(reporter.mode == APP_LOCATION_REPORT_MODE_TIME);
  assert(reporter.interval_ms == 60000U);

  assert(AppLocationReport_SetTimeSeconds(&reporter, 59U, 1000U) == 0U);
  assert(AppLocationReport_SetTimeSeconds(&reporter, 60U, 1000U) == 1U);
  assert(reporter.mode == APP_LOCATION_REPORT_MODE_TIME);
  assert(reporter.interval_ms == 60000U);

  assert(AppLocationReport_SetTimeSeconds(&reporter, 90U, 1000U) == 1U);
  assert(reporter.mode == APP_LOCATION_REPORT_MODE_TIME);
  assert(reporter.interval_ms == 90000U);

  assert(AppLocationReport_SetTimeMinutes(&reporter, 2U, 1000U) == 1U);
  assert(reporter.mode == APP_LOCATION_REPORT_MODE_TIME);
  assert(reporter.interval_ms == 120000U);

  assert(AppLocationReport_SetDistanceMeters(&reporter, 499U, 1000U) == 0U);
  assert(AppLocationReport_SetDistanceMeters(&reporter, 500U, 1000U) == 1U);
  assert(reporter.mode == APP_LOCATION_REPORT_MODE_DISTANCE);
  assert(reporter.distance_threshold_m == 500U);
}

static void test_default_report_triggers_after_two_minutes(void)
{
  app_location_reporter_t reporter;
  char packet[APP_LOCATION_REPORT_TEXT_MAX_LEN + 1U];
  uint8_t ready = 0U;

  AppLocationReport_Init(&reporter, 0U);
  ready = AppLocationReport_ProcessRmc(&reporter,
                                       "$GBRMC,130727.000,A,2604.02152,N,11909.69049,E,0.136,28.018,070126,,,A,S*3B",
                                       1000U,
                                       packet,
                                       sizeof(packet));
  assert(ready == 0U);

  ready = AppLocationReport_ProcessRmc(&reporter,
                                       "$GBRMC,130945.000,A,2604.02161,N,11909.69047,E,0.112,34.197,070126,,,A,S*39",
                                       120000U,
                                       packet,
                                       sizeof(packet));
  assert(ready == 1U);
  assert(strncmp(packet, "D|260402152N1190969049E;", 24U) == 0);
  assert(strlen(packet) <= APP_LOCATION_REPORT_TEXT_MAX_LEN);
}

static void test_second_report_triggers_after_configured_seconds(void)
{
  app_location_reporter_t reporter;
  char packet[APP_LOCATION_REPORT_TEXT_MAX_LEN + 1U];
  uint8_t ready = 0U;

  AppLocationReport_Init(&reporter, 0U);
  assert(AppLocationReport_SetTimeSeconds(&reporter, 90U, 0U) == 1U);

  ready = AppLocationReport_ProcessRmc(&reporter,
                                       "$GBRMC,130727.000,A,2604.02152,N,11909.69049,E,0.136,28.018,070126,,,A,S*3B",
                                       89000U,
                                       packet,
                                       sizeof(packet));
  assert(ready == 0U);

  ready = AppLocationReport_ProcessRmc(&reporter,
                                       "$GBRMC,130945.000,A,2604.02161,N,11909.69047,E,0.112,34.197,070126,,,A,S*39",
                                       90000U,
                                       packet,
                                       sizeof(packet));
  assert(ready == 1U);
  assert(strlen(packet) <= APP_LOCATION_REPORT_TEXT_MAX_LEN);
}

int main(void)
{
  test_parse_gbrmc_to_decimal_microdegrees();
  test_parse_rejects_void_rmc();
  test_location_report_limit_is_78_bytes();
  test_build_packet_uses_three_nmea_points_without_trailing_separator();
  test_distance_uses_internal_microdegree_coordinates();
  test_time_and_distance_settings_validate_lower_bounds();
  test_default_report_triggers_after_two_minutes();
  test_second_report_triggers_after_configured_seconds();

  puts("app_location_report tests passed");
  return 0;
}
