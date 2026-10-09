#ifndef ENGINEERING_INNOVATION_TASK_PLAN_H
#define ENGINEERING_INNOVATION_TASK_PLAN_H

#include "QR.h"
#include "material_tray_servo.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 任务码的三位顺序与车载三工位物料盘固定对应：
 *   第 1 位 -> 物料盘 1 号位
 *   第 2 位 -> 物料盘 2 号位
 *   第 3 位 -> 物料盘 3 号位
 *
 * 颜色码只用于原料区找料；位置码用于粗加工区圆环。
 * 第一批物料在暂存区沿用第一批位置码；第二批在暂存区按照颜色
 * 找到第一批同色物料后进行码垛，不能直接使用第二批粗加工位置码。
 */
typedef enum
{
    TASK_PLAN_BATCH_FIRST = 0,
    TASK_PLAN_BATCH_SECOND
} task_plan_batch_t;

#define TASK_PLAN_INVALID_SEQUENCE_INDEX 0xFFU

typedef struct
{
    task_plan_batch_t batch;
    uint8_t sequence_index;
    uint8_t color_code;
    material_tray_position_t tray_position;
    uint8_t coarse_ring;
    uint8_t temporary_ring;
    uint8_t stack_on_first_batch;
    uint8_t stack_target_sequence_index;
    uint8_t stack_target_color_code;
    uint8_t stack_color_matches;
} task_plan_item_t;

typedef struct
{
    QR_TaskCode_t task;
    task_plan_item_t first_batch[QR_BATCH_ITEM_COUNT];
    task_plan_item_t second_batch[QR_BATCH_ITEM_COUNT];
    uint8_t valid;
} task_execution_plan_t;

/* 将一个已经解析的任务码转换成可执行的两批物料计划。 */
uint8_t task_plan_build(const QR_TaskCode_t *task,
                        task_execution_plan_t *plan);

/* 将 0、1、2 序号映射到物料盘 1、2、3 号位置。 */
material_tray_position_t task_plan_tray_position_for_index(
    uint8_t sequence_index);

/* 读取指定批次和顺序号的物料计划。 */
const task_plan_item_t *task_plan_get_item(
    const task_execution_plan_t *plan,
    task_plan_batch_t batch,
    uint8_t sequence_index);

/* 检查第二批是否都能按同色规则码垛到第一批。 */
uint8_t task_plan_stack_is_valid(const task_execution_plan_t *plan);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK_PLAN_H */
