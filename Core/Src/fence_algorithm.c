/*
 * fence_algorithm.c
 * 电子围栏核心几何算法实现
 *
 * 调用方：
 *   - Core/Src/fence_manager.c（调用本文件所有函数）
 *
 * 说明：此文件为新增实现文件，不读写任何外部数据文件。
 */

#include "fence_algorithm.h"
#include <math.h>
#include <stddef.h>

/* ==========================================================
 * NMEA uint32_t raw 转十进制度数
 * 输入：nmea_raw = 度分格式去掉小数点（如 260392438 表示 2603.92438）
 *       dir = 'N'/'S'/'E'/'W'
 * 输出：十进制度数（如 26.0654063）
 * ========================================================== */
double fence_nmea_to_decimal_deg(uint32_t nmea_raw, char dir)
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

/* ==========================================================
 * Haversine 公式计算两点间距离（米）
 * ========================================================== */
double fence_distance_meters(double lat1, double lng1, double lat2, double lng2)
{
  double d_lat = (lat2 - lat1) * FENCE_ALGO_PI / 180.0;
  double d_lng = (lng2 - lng1) * FENCE_ALGO_PI / 180.0;
  double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) +
             cos(lat1 * FENCE_ALGO_PI / 180.0) *
             cos(lat2 * FENCE_ALGO_PI / 180.0) *
             sin(d_lng / 2.0) * sin(d_lng / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return 6371000.0 * c;
}

/* ==========================================================
 * 更新多边形 AABB 包围盒
 * ========================================================== */
void fence_update_bounding_box(fence_polygon_t *poly)
{
  int i;

  if ((poly == NULL) || (poly->vertex_count <= 0))
  {
    return;
  }

  poly->min_lng = poly->max_lng = poly->vertices[0].lng;
  poly->min_lat = poly->max_lat = poly->vertices[0].lat;

  for (i = 1; i < poly->vertex_count; i++)
  {
    if (poly->vertices[i].lng < poly->min_lng) poly->min_lng = poly->vertices[i].lng;
    if (poly->vertices[i].lng > poly->max_lng) poly->max_lng = poly->vertices[i].lng;
    if (poly->vertices[i].lat < poly->min_lat) poly->min_lat = poly->vertices[i].lat;
    if (poly->vertices[i].lat > poly->max_lat) poly->max_lat = poly->vertices[i].lat;
  }
}

/* ==========================================================
 * 点在多边形内判定 — 优化环绕数法（Winding Number）
 * 步骤：1. AABB包围盒预剔除
 *       2. 环绕数法精确判定
 * ========================================================== */
bool fence_point_in_polygon(fence_point_t p, const fence_polygon_t *poly)
{
  int winding = 0;
  int n;
  int i;
  double x1, y1, x2, y2;
  double cross;

  if ((poly == NULL) || (poly->vertex_count < 3))
  {
    return false;
  }

  /* AABB 预剔除 */
  if ((p.lng < poly->min_lng) || (p.lng > poly->max_lng) ||
      (p.lat < poly->min_lat) || (p.lat > poly->max_lat))
  {
    return false;
  }

  /* 环绕数法 */
  n = poly->vertex_count;
  for (i = 0; i < n; i++)
  {
    x1 = poly->vertices[i].lng;
    y1 = poly->vertices[i].lat;
    x2 = poly->vertices[(i + 1) % n].lng;
    y2 = poly->vertices[(i + 1) % n].lat;

    if (y1 <= p.lat)
    {
      if (y2 > p.lat)
      {
        cross = (x2 - x1) * (p.lat - y1) - (p.lng - x1) * (y2 - y1);
        if (cross > 0.0)
        {
          winding++;
        }
      }
    }
    else
    {
      if (y2 <= p.lat)
      {
        cross = (x2 - x1) * (p.lat - y1) - (p.lng - x1) * (y2 - y1);
        if (cross < 0.0)
        {
          winding--;
        }
      }
    }
  }

  return (winding != 0);
}
