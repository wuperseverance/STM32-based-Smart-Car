#ifndef CUBE_COORDS_H
#define CUBE_COORDS_H

#include <stdint.h>
#include <math.h>

// 圆周率定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 2D点结构
typedef struct {
    int16_t x;
    int16_t y;
} Point2D;

// 线条结构
typedef struct {
    Point2D start;
    Point2D end;
} Line2D;

// 立方体投影结果
typedef struct {
    Line2D lines[12];  // 12条边的坐标
    Point2D vertices[8]; // 8个顶点的坐标（可选）
} CubeProjection;

/**
 * @brief 获取旋转立方体的所有边线坐标
 * @param angle_x X轴旋转角度（度）
 * @param angle_y Y轴旋转角度（度）
 * @param angle_z Z轴旋转角度（度）
 * @param scale 缩放比例
 * @param center_x 屏幕中心X坐标
 * @param center_y 屏幕中心Y坐标
 * @return 包含12条边线坐标的结构体
 */
CubeProjection GetCubeLines(float angle_x, float angle_y, float angle_z, 
                           float scale, int16_t center_x, int16_t center_y);

/**
 * @brief 获取旋转立方体的所有边线坐标（使用默认中心点）
 * @param angle_x X轴旋转角度（度）
 * @param angle_y Y轴旋转角度（度）
 * @param angle_z Z轴旋转角度（度）
 * @param scale 缩放比例
 * @return 包含12条边线坐标的结构体
 */
CubeProjection GetCubeLinesDefault(float angle_x, float angle_y, float angle_z, float scale);

#endif // CUBE_COORDS_H