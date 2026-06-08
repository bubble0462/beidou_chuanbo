/*
 * fence_algorithm.h
 * 电子围栏核心几何算法库
 * 包含：点在多边形内判定（优化环绕数法）、NMEA转十进制、Haversine距离计算
 *
 * 调用方：
 *   - Core/Src/fence_algorithm.c（自身实现文件）
 *   - Core/Src/fence_manager.c（调用 fence_point_in_polygon, fence_distance_meters 等）
 *   - Core/Src/freertos.c（不直接调用，通过 fence_manager 间接使用）
 *
 * 说明：zhongduan 工程中目前不存在任何 fence 相关文件（Glob 搜索确认）。
 *       此文件为新增，不读写任何外部数据文件。
 */

#ifndef FENCE_ALGORITHM_H
#define FENCE_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

#define FENCE_ALGO_PI        3.14159265358979323846
#define FENCE_MAX_VERTICES   20

/* 经纬度点（十进制度数） */
typedef struct
{
  double lng;  /* 经度 */
  double lat;  /* 纬度 */
} fence_point_t;

/* 多边形 */
typedef struct
{
  fence_point_t vertices[FENCE_MAX_VERTICES];
  int vertex_count;
  double min_lng, min_lat, max_lng, max_lat;  /* AABB包围盒 */
} fence_polygon_t;

/* NMEA uint32_t raw 转十进制度数 */
double fence_nmea_to_decimal_deg(uint32_t nmea_raw, char dir);

/* Haversine 距离计算（米） */
double fence_distance_meters(double lat1, double lng1, double lat2, double lng2);

/* 更新多边形包围盒 */
void fence_update_bounding_box(fence_polygon_t *poly);

/* 点在多边形内判定（优化环绕数法 + AABB预剔除） */
bool fence_point_in_polygon(fence_point_t p, const fence_polygon_t *poly);

#endif /* FENCE_ALGORITHM_H */
