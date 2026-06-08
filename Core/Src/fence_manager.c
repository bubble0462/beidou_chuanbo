/*
 * fence_manager.c
 * 电子围栏管理器实现 — 船舶用户机版
 *
 * 调用关系：
 *   - Core/Src/freertos.c:StartModuleTask() 在 GNSS $GNRMC 解析后调用
 *     fence_check_location()（新增调用点，位于 freertos.c 第~410行附近）
 *   - Core/Src/freertos.c:App_HandleBtMessage() 中新增 set_fence/del_fence/list_fence
 *     命令处理，调用 fence_add_circle/fence_add_polygon/fence_remove/fence_list_all()
 *     （新增调用点，位于 freertos.c 第~760行附近）
 *   - Core/Src/freertos.c:App_HandleModuleMessage() 被修改以识别岸基围栏配置并转发
 *     （修改点，位于 freertos.c 第~771行附近）
 *
 * 说明：zhongduan 工程中目前不存在任何 fence 相关文件（Glob 搜索确认）。
 *       此文件为新增实现文件，不读写任何外部数据文件。
 *       报警时通过外部函数 App_SendBd2Message（北斗短报文）和
 *       App_SendBtText（蓝牙文本）双发。
 */

#include "fence_manager.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/* ==========================================================
 * 外部依赖声明（定义在 freertos.c 中）
 * ========================================================== */
extern void App_SendBtText(const char *text);
extern uint8_t App_SendBd2Message(const char *targetCardId, const char *text);
extern char g_targetCardId[16];

/* ==========================================================
 * 围栏列表（全局，自动零初始化）
 * ========================================================== */
fence_t fence_list[MAX_FENCES] = {0};

/* ==========================================================
 * 内部辅助：发送报警（北斗短报文 + 蓝牙文本 双发）
 * 北斗格式: FENCE:EXIT,ID:1,L:119.16302,N:26.06543
 * 蓝牙格式: ALARM:EXIT,FENCE:1,LAT:26.06543,LNG:119.16302\r\n
 * ========================================================== */
static void fence_send_alarm(const char *type, uint32_t fence_id,
                             double lat, double lng)
{
  char bd_msg[80];
  char bt_msg[80];

  /* 北斗短报文 */
  (void)snprintf(bd_msg, sizeof(bd_msg),
                 "FENCE:%s,ID:%" PRIu32 ",L:%.5f,N:%.5f",
                 type, fence_id, lng, lat);

  /* 蓝牙文本 */
  (void)snprintf(bt_msg, sizeof(bt_msg),
                 "ALARM:%s,FENCE:%" PRIu32 ",LAT:%.5f,LNG:%.5f\r\n",
                 type, fence_id, lat, lng);

  /* 双发 */
  if (g_targetCardId[0] != '\0')
  {
    (void)App_SendBd2Message(g_targetCardId, bd_msg);
  }
  App_SendBtText(bt_msg);
}

/* ==========================================================
 * 添加多边形围栏
 * vertices[][0] = 经度, vertices[][1] = 纬度
 * ========================================================== */
void fence_add_polygon(uint32_t id, const double vertices[][2], int vertex_count)
{
  int idx = -1;
  int i;

  if ((vertex_count < 3) || (vertex_count > FENCE_MAX_VERTICES))
  {
    return;
  }

  /* 查找同名围栏或空位 */
  for (i = 0; i < MAX_FENCES; i++)
  {
    if (fence_list[i].is_active && (fence_list[i].fence_id == id))
    {
      idx = i;
      break;
    }
    if ((!fence_list[i].is_active) && (idx == -1))
    {
      idx = i;
    }
  }

  if (idx < 0)
  {
    return;
  }

  fence_list[idx].is_active = true;
  fence_list[idx].fence_id  = id;
  fence_list[idx].type      = FENCE_TYPE_POLYGON;
  fence_list[idx].last_state = 0;

  fence_list[idx].poly.vertex_count = vertex_count;
  for (i = 0; i < vertex_count; i++)
  {
    fence_list[idx].poly.vertices[i].lng = vertices[i][0];
    fence_list[idx].poly.vertices[i].lat = vertices[i][1];
  }
  fence_update_bounding_box(&fence_list[idx].poly);
}

/* ==========================================================
 * 添加圆形围栏
 * ========================================================== */
void fence_add_circle(uint32_t id, double lat, double lng, double radius_m)
{
  int idx = -1;
  int i;

  if (radius_m <= 0.0)
  {
    return;
  }

  for (i = 0; i < MAX_FENCES; i++)
  {
    if (fence_list[i].is_active && (fence_list[i].fence_id == id))
    {
      idx = i;
      break;
    }
    if ((!fence_list[i].is_active) && (idx == -1))
    {
      idx = i;
    }
  }

  if (idx < 0)
  {
    return;
  }

  fence_list[idx].is_active = true;
  fence_list[idx].fence_id  = id;
  fence_list[idx].type      = FENCE_TYPE_CIRCLE;
  fence_list[idx].last_state = 0;
  fence_list[idx].circle.center_lat = lat;
  fence_list[idx].circle.center_lng = lng;
  fence_list[idx].circle.radius_m   = radius_m;
}

/* ==========================================================
 * 删除围栏
 * ========================================================== */
void fence_remove(uint32_t id)
{
  int i;

  for (i = 0; i < MAX_FENCES; i++)
  {
    if (fence_list[i].is_active && (fence_list[i].fence_id == id))
    {
      fence_list[i].is_active = false;
      fence_list[i].last_state = 0;
    }
  }
}

/* ==========================================================
 * 核心检测：传入NMEA uint32_t raw坐标，内部转换后检查所有围栏
 * 应在每次GNSS定位后调用（如每2秒）
 * ========================================================== */
void fence_check_location(uint32_t lat_nmea, char lat_dir,
                          uint32_t lon_nmea, char lon_dir)
{
  double current_lat;
  double current_lng;
  fence_point_t p;
  int i;

  /* 跳过无数据（raw 全 0），放在转换前避免无谓运算 */
  if ((lat_nmea == 0U) && (lon_nmea == 0U))
  {
    return;
  }

  /* NMEA 转十进制度数 */
  current_lat = fence_nmea_to_decimal_deg(lat_nmea, lat_dir);
  current_lng = fence_nmea_to_decimal_deg(lon_nmea, lon_dir);

  p.lng = current_lng;
  p.lat = current_lat;

  for (i = 0; i < MAX_FENCES; i++)
  {
    fence_t *f = &fence_list[i];
    uint8_t curr_state;
    bool is_inside = false;

    if (!f->is_active)
    {
      continue;
    }

    /* 根据围栏类型选择判定算法 */
    if (f->type == FENCE_TYPE_POLYGON)
    {
      is_inside = fence_point_in_polygon(p, &f->poly);
    }
    else if (f->type == FENCE_TYPE_CIRCLE)
    {
      double dist = fence_distance_meters(current_lat, current_lng,
                                          f->circle.center_lat,
                                          f->circle.center_lng);
      is_inside = (dist <= f->circle.radius_m);
    }

    curr_state = is_inside ? 1U : 2U;

    /* 状态变化检测（忽略首次未知状态） */
    if (f->last_state != 0U)
    {
      if ((f->last_state == 1U) && (curr_state == 2U))
      {
        /* 驶出围栏 */
        fence_send_alarm("EXIT", f->fence_id, current_lat, current_lng);
      }
      else if ((f->last_state == 2U) && (curr_state == 1U))
      {
        /* 驶入围栏 */
        fence_send_alarm("ENTER", f->fence_id, current_lat, current_lng);
      }
    }

    f->last_state = curr_state;
  }
}

/* ==========================================================
 * 查询所有围栏并通过蓝牙输出
 * ========================================================== */
void fence_list_all(void)
{
  char buf[128];
  int count = 0;
  int i;

  for (i = 0; i < MAX_FENCES; i++)
  {
    if (fence_list[i].is_active)
    {
      count++;
    }
  }

  (void)snprintf(buf, sizeof(buf), "Fences: %d\r\n", count);
  App_SendBtText(buf);

  for (i = 0; i < MAX_FENCES; i++)
  {
    if (!fence_list[i].is_active)
    {
      continue;
    }

    if (fence_list[i].type == FENCE_TYPE_CIRCLE)
    {
      (void)snprintf(buf, sizeof(buf),
                     "ID:%" PRIu32 " CENTER:%.5f,%.5f R:%.1fm\r\n",
                     fence_list[i].fence_id,
                     fence_list[i].circle.center_lat,
                     fence_list[i].circle.center_lng,
                     fence_list[i].circle.radius_m);
    }
    else
    {
      (void)snprintf(buf, sizeof(buf),
                     "ID:%" PRIu32 " POLY:%dpts\r\n",
                     fence_list[i].fence_id,
                     fence_list[i].poly.vertex_count);
    }
    App_SendBtText(buf);
  }
}
