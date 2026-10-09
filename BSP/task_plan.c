#include "task_plan.h"

#include <string.h>

static uint8_t task_plan_task_valid(const QR_TaskCode_t *task)
{
    uint8_t index;

    if ((task == NULL) || (task->valid == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if ((task->first_batch_color[index] < 1U) ||
            (task->first_batch_color[index] > 6U) ||
            (task->second_batch_color[index] < 1U) ||
            (task->second_batch_color[index] > 6U) ||
            (task->first_batch_position[index] < 1U) ||
            (task->first_batch_position[index] > 3U) ||
            (task->second_batch_position[index] < 1U) ||
            (task->second_batch_position[index] > 3U))
        {
            return 0U;
        }
    }

    return 1U;
}

material_tray_position_t task_plan_tray_position_for_index(
    uint8_t sequence_index)
{
    switch (sequence_index)
    {
        case 0U:
            return MATERIAL_TRAY_POSITION_1;
        case 1U:
            return MATERIAL_TRAY_POSITION_2;
        case 2U:
            return MATERIAL_TRAY_POSITION_3;
        default:
            return MATERIAL_TRAY_POSITION_INVALID;
    }
}

static void task_plan_fill_item(task_plan_item_t *item,
                                task_plan_batch_t batch,
                                uint8_t sequence_index,
                                uint8_t color_code,
                                uint8_t coarse_ring,
                                uint8_t temporary_ring,
                                uint8_t stack_on_first_batch,
                                uint8_t stack_target_sequence_index,
                                uint8_t stack_target_color_code,
                                uint8_t stack_color_matches)
{
    item->batch = batch;
    item->sequence_index = sequence_index;
    item->color_code = color_code;
    item->tray_position =
        task_plan_tray_position_for_index(sequence_index);
    item->coarse_ring = coarse_ring;
    item->temporary_ring = temporary_ring;
    item->stack_on_first_batch = stack_on_first_batch;
    item->stack_target_sequence_index = stack_target_sequence_index;
    item->stack_target_color_code = stack_target_color_code;
    item->stack_color_matches = stack_color_matches;
}

uint8_t task_plan_build(const QR_TaskCode_t *task,
                        task_execution_plan_t *plan)
{
    uint8_t index;
    uint8_t target_index;
    uint8_t target_color;
    uint8_t target_found;

    if ((plan == NULL) || (task_plan_task_valid(task) == 0U))
    {
        return 0U;
    }

    memset(plan, 0, sizeof(*plan));
    memcpy(&plan->task, task, sizeof(plan->task));

    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        task_plan_fill_item(&plan->first_batch[index],
                            TASK_PLAN_BATCH_FIRST,
                            index,
                            task->first_batch_color[index],
                            task->first_batch_position[index],
                            task->first_batch_position[index],
                            0U,
                            TASK_PLAN_INVALID_SEQUENCE_INDEX,
                            0U,
                            0U);

        target_index = TASK_PLAN_INVALID_SEQUENCE_INDEX;
        target_color = 0U;
        target_found = 0U;
        for (target_index = 0U;
             target_index < QR_BATCH_ITEM_COUNT;
             target_index++)
        {
            if (task->first_batch_color[target_index] ==
                task->second_batch_color[index])
            {
                target_color = task->first_batch_color[target_index];
                target_found = 1U;
                break;
            }
        }

        if (target_found == 0U)
        {
            target_index = TASK_PLAN_INVALID_SEQUENCE_INDEX;
        }

        task_plan_fill_item(&plan->second_batch[index],
                            TASK_PLAN_BATCH_SECOND,
                            index,
                            task->second_batch_color[index],
                            task->second_batch_position[index],
                            (target_found != 0U) ?
                                task->first_batch_position[target_index] : 0U,
                            1U,
                            target_index,
                            target_color,
                            (target_found != 0U) &&
                            (target_color == task->second_batch_color[index]));
    }

    plan->valid = 1U;
    return 1U;
}

const task_plan_item_t *task_plan_get_item(
    const task_execution_plan_t *plan,
    task_plan_batch_t batch,
    uint8_t sequence_index)
{
    if ((plan == NULL) || (plan->valid == 0U) ||
        (sequence_index >= QR_BATCH_ITEM_COUNT))
    {
        return NULL;
    }

    if (batch == TASK_PLAN_BATCH_FIRST)
    {
        return &plan->first_batch[sequence_index];
    }
    if (batch == TASK_PLAN_BATCH_SECOND)
    {
        return &plan->second_batch[sequence_index];
    }

    return NULL;
}

uint8_t task_plan_stack_is_valid(const task_execution_plan_t *plan)
{
    uint8_t index;

    if ((plan == NULL) || (plan->valid == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if ((plan->second_batch[index].stack_on_first_batch == 0U) ||
            (plan->second_batch[index].stack_target_sequence_index ==
             TASK_PLAN_INVALID_SEQUENCE_INDEX) ||
            (plan->second_batch[index].stack_color_matches == 0U))
        {
            return 0U;
        }
    }

    return 1U;
}
