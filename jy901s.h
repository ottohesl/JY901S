#ifndef JY901S_H
#define JY901S_H

#include "main.h"
#include <string.h>

// 传感器数据结构
typedef struct {
    float acc[3];     // 加速度 (m/s²) [X, Y, Z]
    float gyro[3];    // 角速度 (°/s) [X, Y, Z]
    float angle[3];   // 欧拉角 (°) [Roll, Pitch, Yaw]
} JY901S_Data;

// 解析器状态结构
typedef struct {
    uint8_t frame_state;
    uint8_t byte_count;
    uint8_t check_sum;
    uint8_t acc_buffer[8];
    uint8_t gyro_buffer[8];
    uint8_t angle_buffer[8];
} JY901S_Parser;

// 设备句柄结构
typedef struct {
    UART_HandleTypeDef *huart_sensor;
    UART_HandleTypeDef *huart_debug;
    JY901S_Data sensor_data;
    JY901S_Parser parser;
} JY901S_Handle;

// 函数声明
void JY901S_Init(JY901S_Handle *handle, UART_HandleTypeDef *sensor_uart, UART_HandleTypeDef *debug_uart);
void JY901S_ProcessUARTData(JY901S_Handle *handle);
void JY901S_PrintData(JY901S_Handle *handle);
void JY901S_ResetParser(JY901S_Handle *handle);

#endif // JY901S_H