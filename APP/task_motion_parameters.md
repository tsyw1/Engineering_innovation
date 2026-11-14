# 任务移动距离参数位置

本文记录任务 1、2、3、5、7 中底盘移动距离参数的定义位置和实际调用位置。
修改距离时，优先修改下面列出的 `.h` 文件中的宏定义。数值单位均为毫米（`mm`）。

## 任务 1：启动、行驶中扫码

文件：`APP/task1_start_qr.h`

| 参数 | 当前值 | 宏定义位置 | 实际调用位置 |
| --- | ---: | --- | --- |
| 第一段前进扫码距离 | `260U` | `APP/task1_start_qr.h:20` | `APP/task1_start_qr.c:205` |
| 第二段前进扫码距离 | `660U` | `APP/task1_start_qr.h:21` | `APP/task1_start_qr.c:247` |

```c
#define TASK1_START_QR_FORWARD_SCAN_FIRST_MM   260U
#define TASK1_START_QR_FORWARD_SCAN_SECOND_MM  660U
```

## 任务 2：第一批原料区平台取料

文件：`APP/task2_platform_pick.h`

| 参数 | 当前值 | 宏定义位置 | 实际调用位置 |
| --- | ---: | --- | --- |
| 右移到平台距离 | `920U` | `APP/task2_platform_pick.h:17` | `APP/task2_platform_pick.c:662` |
| 前进到平台距离 | `70U` | `APP/task2_platform_pick.h:18` | `APP/task2_platform_pick.c:674` |
| 取料结束后的返回距离 | 使用同一个 `70U` 宏 | `APP/task2_platform_pick.h:18` | `APP/task2_platform_pick.c:140` |

```c
#define TASK2_PLATFORM_PICK_RIGHT_MM    920U
#define TASK2_PLATFORM_PICK_FORWARD_MM   70U
```

注意：修改 `TASK2_PLATFORM_PICK_FORWARD_MM` 会同时修改到达平台时的前进距离和取料结束后的后退距离。

## 任务 3：第一批粗加工区放置

文件：`APP/task3_coarse_ring.h`

| 参数 | 当前源码值 | 宏定义位置 | 实际调用位置 |
| --- | ---: | --- | --- |
| 左移距离 | `70U` | `APP/task3_coarse_ring.h:17` | `APP/task3_coarse_ring.c:336` |
| 右移距离 | `1670U` | `APP/task3_coarse_ring.h:19` | `APP/task3_coarse_ring.c:383` |

```c
#define TASK3_COARSE_RING_LEFT_MM   70U
#define TASK3_COARSE_RING_RIGHT_MM 1670U
```

注意：任务要求描述为右移 `1640 mm`，但当前源码中的实际值仍为 `1670U`。如果需要右移 `1640 mm`，应将第 19 行改为：

```c
#define TASK3_COARSE_RING_RIGHT_MM 1640U
```

## 任务 5：暂存区移动

文件：`APP/task5_temporary_ring.h`

| 参数 | 当前值 | 宏定义位置 | 实际选择/调用位置 |
| --- | ---: | --- | --- |
| 从 1 号环后退距离 | `970U` | `APP/task5_temporary_ring.h:20` | `APP/task5_temporary_ring.c:219` |
| 从 2 号中心环后退距离 | `830U` | `APP/task5_temporary_ring.h:21` | `APP/task5_temporary_ring.c:221` |
| 从 3 号环后退距离 | `670U` | `APP/task5_temporary_ring.h:22` | `APP/task5_temporary_ring.c:223` |
| 旋转后到暂存区后退距离 | `820U` | `APP/task5_temporary_ring.h:19` | `APP/task5_temporary_ring.c:454` |

```c
#define TASK5_TEMPORARY_RING_BACKWARD_TO_AREA_MM 820U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_1_MM  970U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_2_MM  830U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_3_MM  670U
```

任务 5 默认从 2 号中心环开始，因此默认使用 `TASK5_TEMPORARY_RING_BACKWARD_RING_2_MM`，即 `830U`。

## 任务 7：第二批粗加工区放置

文件：`APP/task7_coarse_ring.h`

| 参数 | 当前值 | 宏定义位置 | 实际调用位置 |
| --- | ---: | --- | --- |
| 左移距离 | `70U` | `APP/task7_coarse_ring.h:17` | `APP/task7_coarse_ring.c:353` |
| 右移距离 | `1640U` | `APP/task7_coarse_ring.h:19` | `APP/task7_coarse_ring.c:401` |

```c
#define TASK7_COARSE_RING_LEFT_MM   70U
#define TASK7_COARSE_RING_RIGHT_MM 1640U
```

## 修改注意事项

1. 这些距离宏最终会传给底盘闭环运动接口，直接修改宏数值即可改变对应移动距离。
2. `U` 是无符号整数后缀，修改时保留该后缀即可，例如 `850U`。
3. `.c` 文件中的调试事件字符串可能写死了旧距离，例如 `task7_right_1640mm_failed`。修改宏后，实际运动会改变，但日志文字不会自动更新。
4. 修改后应重新编译，并根据实际运行结果校准机械误差。
