#include "fence_algorithm.h"
#include "fence_manager.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* 桩实现：freertos.c 中的外部符号 */
char g_targetCardId[16] = "0362746";
uint8_t App_SendBd2Message(const char *targetCardId, const char *text)
{
  (void)targetCardId;
  (void)text;
  return 1U;
}
void App_SendBtText(const char *text)
{
  printf("[BT] %s", text);
}

/* ==========================================================
 * 辅助函数（与 freertos.c 中相同的逻辑，用于测试）
 * ========================================================== */
static double Test_PaddedNmeaToDec(uint32_t raw, char dir)
{
  raw = raw * 1000U;
  return fence_nmea_to_decimal_deg(raw, dir);
}

static const char *Test_ParseNextUint(const char *p, uint32_t *out)
{
  char *end = NULL;
  unsigned long val = strtoul(p, &end, 10);
  if (end == p) return NULL;
  *out = (uint32_t)val;
  if (*end == ',') end++;
  return (const char *)end;
}

static const char *Test_ParseNextInt(const char *p, int32_t *out)
{
  char *end = NULL;
  long val = strtol(p, &end, 10);
  if (end == p) return NULL;
  *out = (int32_t)val;
  if (*end == ',') end++;
  return (const char *)end;
}

/* ==========================================================
 * 测试 1: NMEA uint32_t raw 转十进制度数
 * ========================================================== */
static void test_nmea_to_decimal(void)
{
  double lat = fence_nmea_to_decimal_deg(260392438U, 'N');
  double lng = fence_nmea_to_decimal_deg(1190977779U, 'E');

  assert(fabs(lat - 26.0654063) < 0.00001);
  assert(fabs(lng - 119.1629630) < 0.00001);

  assert(fence_nmea_to_decimal_deg(260392438U, 'S') < 0.0);
  assert(fence_nmea_to_decimal_deg(1190977779U, 'W') < 0.0);
}

/* ==========================================================
 * 测试 2: Haversine 距离
 * ========================================================== */
static void test_haversine_distance(void)
{
  double dist = fence_distance_meters(26.06543, 119.16302,
                                      26.05123, 119.17045);
  assert(dist > 1700.0);
  assert(dist < 1800.0);

  assert(fence_distance_meters(26.0, 119.0, 26.0, 119.0) == 0.0);
}

/* ==========================================================
 * 测试 3: 点在多边形内判定
 * ========================================================== */
static void test_point_in_polygon(void)
{
  fence_polygon_t poly;
  fence_point_t vertices[4] = {
    {119.0, 26.0}, {119.1, 26.0}, {119.1, 26.1}, {119.0, 26.1}
  };

  poly.vertex_count = 4;
  for (int i = 0; i < 4; i++) {
    poly.vertices[i] = vertices[i];
  }
  fence_update_bounding_box(&poly);

  fence_point_t center = {119.05, 26.05};
  assert(fence_point_in_polygon(center, &poly) == true);

  assert(fence_point_in_polygon(vertices[0], &poly) == true);

  fence_point_t outside = {119.5, 26.5};
  assert(fence_point_in_polygon(outside, &poly) == false);

  fence_point_t far = {120.0, 27.0};
  assert(fence_point_in_polygon(far, &poly) == false);
}

/* ==========================================================
 * 测试 4: 围栏状态机 — 模拟进入/驶出报警
 * ========================================================== */
static void test_fence_state_machine(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  fence_add_circle(1U, 26.06543, 119.16302, 100.0);

  /* 首次观测：建立基线（在外），不发报警 */
  fence_check_location(260376570U, 'N', 1190974952U, 'E');
  assert(fence_list[0].last_state == 2U);

  /* 持续在外：状态不变 */
  fence_check_location(260376570U, 'N', 1190974952U, 'E');
  assert(fence_list[0].last_state == 2U);

  /* 进入：R1 防抖需连续 FENCE_CONFIRM_SAMPLES 次才确认 */
  fence_check_location(260392438U, 'N', 1190977779U, 'E');
  assert(fence_list[0].last_state == 2U);
  fence_check_location(260392438U, 'N', 1190977779U, 'E');
  assert(fence_list[0].last_state == 2U);
  fence_check_location(260392438U, 'N', 1190977779U, 'E');
  assert(fence_list[0].last_state == 1U);

  /* 驶出：同样需连续 FENCE_CONFIRM_SAMPLES 次确认 */
  fence_check_location(260376570U, 'N', 1190974952U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260376570U, 'N', 1190974952U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260376570U, 'N', 1190974952U, 'E');
  assert(fence_list[0].last_state == 2U);
}

/* ==========================================================
 * 测试 5: 圆形围栏边界判定
 * ========================================================== */
static void test_circle_boundary(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  fence_add_circle(2U, 26.0, 119.0, 1000.0);

  /* 首次观测：圆心，在内 */
  fence_check_location(260000000U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);

  /* 移动但仍在内（~500m） */
  fence_check_location(260026976U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);

  /* 移出到外（~1497m）：R1 防抖需连续 FENCE_CONFIRM_SAMPLES 次确认 */
  fence_check_location(260080940U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260080940U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260080940U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 2U);
}

/* ==========================================================
 * 测试 6: 多边形围栏添加与删除
 * ========================================================== */
static void test_polygon_add_remove(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  double vertices[][2] = {
    {119.0, 26.0}, {119.1, 26.0}, {119.1, 26.1}
  };
  fence_add_polygon(3U, vertices, 3);

  assert(fence_list[0].is_active == true);
  assert(fence_list[0].fence_id == 3U);
  assert(fence_list[0].type == FENCE_TYPE_POLYGON);
  assert(fence_list[0].poly.vertex_count == 3);

  fence_remove(3U);
  assert(fence_list[0].is_active == false);
}

/* ==========================================================
 * 测试 7: NMEA raw 补零转换（截断2dp精度验证）
 * ========================================================== */
static void test_nmea_padding_conversion(void)
{
  double full = fence_nmea_to_decimal_deg(260392438U, 'N');
  double truncated = Test_PaddedNmeaToDec(260392U, 'N');

  /* 误差应小于 0.001° (~111m)，实际约 8m */
  assert(fabs(full - truncated) < 0.001);
  printf("  Full(5dp): %.7f  Truncated(2dp): %.7f  Diff: %.1fm\n",
         full, truncated,
         fence_distance_meters(full, 119.0, truncated, 119.0));

  double full_lng = fence_nmea_to_decimal_deg(1190977779U, 'E');
  double trunc_lng = Test_PaddedNmeaToDec(1190978U, 'E');
  assert(fabs(full_lng - trunc_lng) < 0.001);
}

/* ==========================================================
 * 测试 8: FC 格式解析（圆形）
 * ========================================================== */
static void test_fc_parse(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  const char *text = "FC1,260392,1190978,500";
  assert(text[0] == 'F' && text[1] == 'C');

  uint32_t id = 0U, lat_raw = 0U, lng_raw = 0U, radius = 0U;
  const char *p = &text[2];
  p = Test_ParseNextUint(p, &id);       assert(p != NULL); assert(id == 1U);
  p = Test_ParseNextUint(p, &lat_raw);  assert(p != NULL); assert(lat_raw == 260392U);
  p = Test_ParseNextUint(p, &lng_raw);  assert(p != NULL); assert(lng_raw == 1190978U);
  p = Test_ParseNextUint(p, &radius);   assert(p != NULL); assert(radius == 500U);

  double lat = Test_PaddedNmeaToDec(lat_raw, 'N');
  double lng = Test_PaddedNmeaToDec(lng_raw, 'E');

  fence_add_circle(id, lat, lng, (double)radius);

  assert(fence_list[0].is_active == true);
  assert(fence_list[0].fence_id == 1U);
  assert(fence_list[0].type == FENCE_TYPE_CIRCLE);
  assert(fence_list[0].circle.radius_m == 500.0);
  printf("  FC: id=%lu lat=%.5f lng=%.5f r=%.1f\n", id, lat, lng, (double)radius);
}

/* ==========================================================
 * 测试 9: FP 格式解析（四边形绝对坐标）
 * ========================================================== */
static void test_fp_parse(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  const char *text = "FP2,260392,1190978,260377,1190975,260377,1190980,260392,1190978";
  assert(text[0] == 'F' && text[1] == 'P');

  uint32_t id = 0U;
  double vertices[FENCE_MAX_VERTICES][2];
  int count = 0;
  const char *p = &text[2];

  p = Test_ParseNextUint(p, &id);
  assert(p != NULL && id == 2U);

  while (*p != '\0' && count < FENCE_MAX_VERTICES) {
    uint32_t lat_raw = 0U, lng_raw = 0U;
    const char *next;
    next = Test_ParseNextUint(p, &lat_raw);
    if (next == NULL) break;
    p = next;
    next = Test_ParseNextUint(p, &lng_raw);
    if (next == NULL) break;
    p = next;
    vertices[count][0] = Test_PaddedNmeaToDec(lng_raw, 'E');
    vertices[count][1] = Test_PaddedNmeaToDec(lat_raw, 'N');
    count++;
  }

  assert(count == 4);

  fence_add_polygon(id, (const double (*)[2])vertices, count);

  assert(fence_list[0].is_active == true);
  assert(fence_list[0].fence_id == 2U);
  assert(fence_list[0].type == FENCE_TYPE_POLYGON);
  assert(fence_list[0].poly.vertex_count == 4);
  printf("  FP: id=%lu vertices=%d\n", id, count);
}

/* ==========================================================
 * 测试 10: FQ 格式解析（五边形偏移编码）
 * ========================================================== */
static void test_fq_parse(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  const char *text = "FQ3,260392,1190978,0,0,-14,+7,-14,+8,-5,-2,0,0";
  assert(text[0] == 'F' && text[1] == 'Q');

  uint32_t id = 0U, lat0_raw = 0U, lng0_raw = 0U;
  double vertices[FENCE_MAX_VERTICES][2];
  int count = 0;
  const char *p = &text[2];

  p = Test_ParseNextUint(p, &id);       assert(p != NULL && id == 3U);
  p = Test_ParseNextUint(p, &lat0_raw); assert(p != NULL);
  p = Test_ParseNextUint(p, &lng0_raw); assert(p != NULL);

  double base_lat = Test_PaddedNmeaToDec(lat0_raw, 'N');
  double base_lng = Test_PaddedNmeaToDec(lng0_raw, 'E');
  vertices[0][0] = base_lng;
  vertices[0][1] = base_lat;
  count = 1;

  while (*p != '\0' && count < FENCE_MAX_VERTICES) {
    int32_t dlat = 0, dlng = 0;
    const char *next;
    next = Test_ParseNextInt(p, &dlat);
    if (next == NULL) break;
    p = next;
    next = Test_ParseNextInt(p, &dlng);
    if (next == NULL) break;
    p = next;
    vertices[count][1] = base_lat + ((double)dlat * 0.001);
    vertices[count][0] = base_lng + ((double)dlng * 0.001);
    count++;
  }

  assert(count == 6);

  /* 偏移(0,0) == 基准点 */
  assert(fabs(vertices[1][1] - base_lat) < 0.0001);
  assert(fabs(vertices[1][0] - base_lng) < 0.0001);

  /* 偏移(-14,+7) */
  assert(fabs(vertices[2][1] - (base_lat - 0.014)) < 0.0001);
  assert(fabs(vertices[2][0] - (base_lng + 0.007)) < 0.0001);

  fence_add_polygon(id, (const double (*)[2])vertices, count);

  assert(fence_list[0].is_active == true);
  assert(fence_list[0].fence_id == 3U);
  assert(fence_list[0].type == FENCE_TYPE_POLYGON);
  assert(fence_list[0].poly.vertex_count == 6);
  printf("  FQ: id=%lu vertices=%d base=(%.5f,%.5f)\n", id, count, base_lat, base_lng);
}

/* ==========================================================
 * 测试 11: FQ 围栏进出检测
 * ========================================================== */
static void test_fq_fence_detection(void)
{
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }

  /* 基准点 26.06533, 119.16300 (补零后的NMEA 2dp) */
  double base_lat = Test_PaddedNmeaToDec(260392U, 'N');
  double base_lng = Test_PaddedNmeaToDec(1190978U, 'E');

  double vertices[][2] = {
    {base_lng,         base_lat},
    {base_lng - 0.003, base_lat},
    {base_lng - 0.003, base_lat + 0.003},
    {base_lng,         base_lat + 0.003},
  };
  fence_add_polygon(5U, vertices, 4);

  /* 围栏内部：使用原始5dp NMEA格式（fence_check_location期望原始格式） */
  /* 260392438 = 26.06541, 1190977779 = 119.16296，在围栏内 */
  fence_check_location(260392438U, 'N', 1190977779U, 'E');
  assert(fence_list[0].last_state == 1U);

  /* 围栏外部：远离围栏。R1 防抖需连续 FENCE_CONFIRM_SAMPLES 次确认 */
  fence_check_location(260000000U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260000000U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 1U);
  fence_check_location(260000000U, 'N', 1190000000U, 'E');
  assert(fence_list[0].last_state == 2U);
  printf("  FQ detection: inside/outside OK\n");
}

/* ==========================================================
 * 测试 12: 半径上限（R7）与围栏列表满载（R4）返回值校验
 * ========================================================== */
static void test_radius_bound_and_full(void)
{
  /* R7：半径超限 / 非法应被拒绝且不落库 */
  for (int i = 0; i < MAX_FENCES; i++) {
    fence_list[i].is_active = false;
  }
  assert(fence_add_circle(1U, 26.0, 119.0, FENCE_MAX_RADIUS_M + 1.0) == false);
  assert(fence_add_circle(1U, 26.0, 119.0, -10.0) == false);
  assert(fence_list[0].is_active == false);
  /* 合法半径应成功 */
  assert(fence_add_circle(1U, 26.0, 119.0, 500.0) == true);

  /* R4：围栏列表满载时新增应失败 */
  for (int i = 0; i < MAX_FENCES - 1; i++) {
    uint32_t id = (uint32_t)(100 + i);
    assert(fence_add_circle(id, 26.0, 119.0, 200.0) == true);
  }
  assert(fence_add_circle(999U, 26.0, 119.0, 200.0) == false);
}

int main(void)
{
  test_nmea_to_decimal();
  puts("[PASS] test_nmea_to_decimal");

  test_haversine_distance();
  puts("[PASS] test_haversine_distance");

  test_point_in_polygon();
  puts("[PASS] test_point_in_polygon");

  test_fence_state_machine();
  puts("[PASS] test_fence_state_machine");

  test_circle_boundary();
  puts("[PASS] test_circle_boundary");

  test_polygon_add_remove();
  puts("[PASS] test_polygon_add_remove");

  test_nmea_padding_conversion();
  puts("[PASS] test_nmea_padding_conversion");

  test_fc_parse();
  puts("[PASS] test_fc_parse");

  test_fp_parse();
  puts("[PASS] test_fp_parse");

  test_fq_parse();
  puts("[PASS] test_fq_parse");

  test_fq_fence_detection();
  puts("[PASS] test_fq_fence_detection");

  test_radius_bound_and_full();
  puts("[PASS] test_radius_bound_and_full");

  puts("\n=== all 12 fence tests passed ===");
  return 0;
}
