#include "cube.h"

// 立方体顶点（单位立方体，中心在原点）
static const float cube_vertices[8][3] = {
    {-0.5f, -0.5f, -0.5f},  // 0: 左前下
    { 0.5f, -0.5f, -0.5f},  // 1: 右前下
    { 0.5f,  0.5f, -0.5f},  // 2: 右后下
    {-0.5f,  0.5f, -0.5f},  // 3: 左后下
    {-0.5f, -0.5f,  0.5f},  // 4: 左前上
    { 0.5f, -0.5f,  0.5f},  // 5: 右前上
    { 0.5f,  0.5f,  0.5f},  // 6: 右后上
    {-0.5f,  0.5f,  0.5f}   // 7: 左后上
};

// 立方体边连接关系
static const uint8_t cube_edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // 底面
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // 顶面
    {0, 4}, {1, 5}, {2, 6}, {3, 7}   // 连接底面和顶面的边
};

// 旋转点绕X轴
static void rotate_x(float *point, float angle) {
    float y = point[1];
    float z = point[2];
    point[1] = y * cosf(angle) - z * sinf(angle);
    point[2] = y * sinf(angle) + z * cosf(angle);
}

// 旋转点绕Y轴
static void rotate_y(float *point, float angle) {
    float x = point[0];
    float z = point[2];
    point[0] = x * cosf(angle) + z * sinf(angle);
    point[2] = -x * sinf(angle) + z * cosf(angle);
}

// 旋转点绕Z轴
static void rotate_z(float *point, float angle) {
    float x = point[0];
    float y = point[1];
    point[0] = x * cosf(angle) - y * sinf(angle);
    point[1] = x * sinf(angle) + y * cosf(angle);
}

// 投影3D点到2D屏幕（正交投影）
static Point2D project_point(const float *point, float scale, int16_t center_x, int16_t center_y) {
    Point2D result;
    result.x = center_x + (int16_t)(point[0] * scale);
    result.y = center_y + (int16_t)(point[1] * scale);
    return result;
}

CubeProjection GetCubeLines(float angle_x, float angle_y, float angle_z, 
                           float scale, int16_t center_x, int16_t center_y) {
    CubeProjection result;
    float rotated_vertices[8][3];
    
    // 将角度转换为弧度
    float rad_x = angle_x * M_PI / 180.0f;
    float rad_y = angle_y * M_PI / 180.0f;
    float rad_z = angle_z * M_PI / 180.0f;
    
    // 旋转并投影所有顶点
    for (int i = 0; i < 8; i++) {
        // 复制原始顶点
        rotated_vertices[i][0] = cube_vertices[i][0];
        rotated_vertices[i][1] = cube_vertices[i][1];
        rotated_vertices[i][2] = cube_vertices[i][2];
        
        // 应用旋转
        rotate_x(rotated_vertices[i], rad_x);
        rotate_y(rotated_vertices[i], rad_y);
        rotate_z(rotated_vertices[i], rad_z);
        
        // 投影到2D并保存顶点坐标
        result.vertices[i] = project_point(rotated_vertices[i], scale, center_x, center_y);
    }
    
    // 生成所有边线坐标
    for (int i = 0; i < 12; i++) {
        uint8_t start_idx = cube_edges[i][0];
        uint8_t end_idx = cube_edges[i][1];
        
        result.lines[i].start = result.vertices[start_idx];
        result.lines[i].end = result.vertices[end_idx];
    }
    
    return result;
}

CubeProjection GetCubeLinesDefault(float angle_x, float angle_y, float angle_z, float scale) {
    // 使用默认中心点（OLED 128x64的中心）
    return GetCubeLines(angle_x, angle_y, angle_z, scale, 64, 32);
}