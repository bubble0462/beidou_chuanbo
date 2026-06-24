/*
 * fence_manager.h
 * 电子围栏管理器 — 船舶用户机版
 * 支持：多边形围栏（四边、五边等）、圆形围栏
 *
 * 调用方：
 *   - Core/Src/fence_manager.c（自身实现文件）
 *   - Core/Src/freertos.c（调用 fence_check_location, fence_add_circle 等）
 *
 * 说明：zhongduan 工程中目前不存在任何 fence 相关文件（Glob 搜索确认）。
 *       此文件为新增，不读写任何外部数据文件。
 */

#ifndef FENCE_MANAGER_H
#define FENCE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "fence_algorithm.h"

#define MAX_FENCES 10

/* 报警防抖（R1）：状态翻转需连续 FENCE_CONFIRM_SAMPLES 次同方向采样才确认。
 * 2Hz 采样下 3 次 ≈ 1.5s，过滤 GPS 边界抖动导致的报警刷屏。 */
#define FENCE_CONFIRM_SAMPLES 3U

/* 圆形围栏半径上限（米，R7），防止误下发巨型围栏 */
#define FENCE_MAX_RADIUS_M     50000.0

/* 围栏类型枚举 */
typedef enum
{
  FENCE_TYPE_POLYGON = 0,  /* 多边形 */
  FENCE_TYPE_CIRCLE  = 1,  /* 圆形 */
} fence_type_t;

/* 圆形围栏附加数据 */
typedef struct
{
  double center_lat;   /* 圆心纬度（十进制度数） */
  double center_lng;   /* 圆心经度（十进制度数） */
  double radius_m;     /* 半径（米） */
} fence_circle_t;

/* 通用围栏结构体 */
typedef struct
{
  bool         is_active;    /* 是否生效 */
  uint32_t     fence_id;     /* 围栏ID */
  fence_type_t type;         /* 围栏类型 */
  fence_polygon_t poly;      /* 多边形数据（type=POLYGON时有效） */
  fence_circle_t  circle;    /* 圆形数据（type=CIRCLE时有效） */
  uint8_t      last_state;     /* 0=未知, 1=在内, 2=在外（已确认状态） */
  uint8_t      pending_state;  /* 待确认的新状态（R1 防抖，0=无待确认） */
  uint8_t      confirm_count;  /* pending_state 的连续确认次数（R1 防抖） */
} fence_t;

extern fence_t fence_list[MAX_FENCES];

/* 添加多边形围栏：vertices[][0]=经度, vertices[][1]=纬度
 * 成功返回 true；顶点数非法或围栏列表已满返回 false（R4） */
bool fence_add_polygon(uint32_t id, const double vertices[][2], int vertex_count);

/* 添加圆形围栏
 * 成功返回 true；半径非法/超限（R7）或围栏列表已满（R4）返回 false */
bool fence_add_circle(uint32_t id, double lat, double lng, double radius_m);

/* 删除围栏 */
void fence_remove(uint32_t id);

/* 检测当前位置：传入NMEA uint32_t raw坐标，内部转换后检查所有围栏 */
void fence_check_location(uint32_t lat_nmea, char lat_dir,
                          uint32_t lon_nmea, char lon_dir);

/* 查询所有围栏并通过蓝牙输出 */
void fence_list_all(void);

#endif /* FENCE_MANAGER_H */
