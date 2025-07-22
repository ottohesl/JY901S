#include "jy901s.h"
#include <stdio.h>

// 内部函数声明
static void parse_byte(JY901S_Handle *handle, uint8_t byte);
static void store_data(JY901S_Handle *handle, uint8_t byte);
static void validate_checksum(JY901S_Handle *handle, uint8_t received_sum);
static void update_sensor_data(JY901S_Handle *handle);
static void parse_acceleration(JY901S_Handle *handle);
static void parse_gyroscope(JY901S_Handle *handle);
static void parse_angle(JY901S_Handle *handle);
/**
 * @brief 初始化JY901S传感器
 *
 * 配置传感器句柄并初始化通信接口
 *
 * @param handle 指向JY901S_Handle的指针，包含传感器配置和数据
 * @param sensor_uart 指向传感器UART句柄的指针
 * @param debug_uart 指向调试UART句柄的指针
 *
 * @return 无返回值
 */
void JY901S_Init(JY901S_Handle *handle, UART_HandleTypeDef *sensor_uart, UART_HandleTypeDef *debug_uart) {
    handle->huart_sensor = sensor_uart;
    handle->huart_debug = debug_uart;

    // 重置传感器数据和解析器
    memset(&handle->sensor_data, 0, sizeof(JY901S_Data));
    JY901S_ResetParser(handle);

    // 发送初始化成功消息
    const char *init_msg = "JY901S Init OK\n";
    HAL_UART_Transmit(handle->huart_debug, (uint8_t*)init_msg, strlen(init_msg), 100);
}

/**
 * @brief 处理来自传感器的UART数据
 *
 * 从UART接收缓冲区读取所有可用字节并传递给解析器
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
void JY901S_ProcessUARTData(JY901S_Handle *handle) {
    uint8_t byte;
    // 循环读取所有可用数据
    while (HAL_UART_Receive(handle->huart_sensor, &byte, 1, 60) == HAL_OK) {
        parse_byte(handle, byte);
    }
}

/**
 * @brief 打印传感器数据到调试串口
 *
 * 以固定频率输出传感器角度数据
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
void JY901S_PrintData(JY901S_Handle *handle) {
    // 每500ms打印一次数据
    char buffer[75];
    int len1=sprintf(buffer, " %.2f, %.2f, %.2f\n",
                  handle->sensor_data.angle[0],
                  handle->sensor_data.angle[1],
                  handle->sensor_data.angle[2]);
    HAL_UART_Transmit(handle->huart_debug, (uint8_t*)buffer, len1, 100);
}
/**
 * @brief 重置解析器状态
 *
 * 将解析器状态恢复到初始状态，准备接收新的数据帧
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
void JY901S_ResetParser(JY901S_Handle *handle) {
    handle->parser.frame_state = 0;
    handle->parser.byte_count = 0;
    handle->parser.check_sum = 0;
}

/**
 * @brief 解析单个字节数据
 *
 * 根据当前解析状态处理接收到的字节，实现帧同步和数据提取
 *
 * @param handle 指向JY901S_Handle的指针
 * @param byte 接收到的单个字节数据
 *
 * @return 无返回值
 */
static void parse_byte(JY901S_Handle *handle, uint8_t byte) {
    JY901S_Parser *parser = &handle->parser;

    switch (parser->frame_state) {
        case 0: // 等待帧头
            if (byte == 0x55 && parser->byte_count == 0) {
                parser->check_sum = byte;
                parser->byte_count = 1;
            }
            break;

        case 1: // 接收加速度数据
        case 2: // 接收陀螺仪数据
        case 3: // 接收角度数据
            if (parser->byte_count < 10) {
                store_data(handle, byte);
                parser->check_sum += byte;
                parser->byte_count++;
            } else {
                validate_checksum(handle, byte);
                JY901S_ResetParser(handle);
            }
            break;

        default: // 无效状态
            JY901S_ResetParser(handle);
    }

    // 状态转移
    if (parser->byte_count == 1 && (byte >= 0x51 && byte <= 0x53)) {
        parser->check_sum += byte;
        parser->frame_state = byte - 0x50;
        parser->byte_count = 2;
    }
}

/**
 * @brief 存储接收到的数据
 *
 * 根据当前帧类型将数据存储到对应的缓冲区
 *
 * @param handle 指向JY901S_Handle的指针
 * @param byte 接收到的单个字节数据
 *
 * @return 无返回值
 */
static void store_data(JY901S_Handle *handle, uint8_t byte) {
    JY901S_Parser *parser = &handle->parser;
    uint8_t index = parser->byte_count - 2;

    if (index >= 8) return;  // 防止缓冲区溢出

    switch (parser->frame_state) {
        case 1:
            parser->acc_buffer[index] = byte;
            break;
        case 2:
            parser->gyro_buffer[index] = byte;
            break;
        case 3:
            parser->angle_buffer[index] = byte;
            break;
    }
}

/**
 * @brief 验证校验和
 *
 * 比较计算的校验和与接收到的校验和是否一致
 *
 * @param handle 指向JY901S_Handle的指针
 * @param received_sum 接收到的校验和字节
 *
 * @return 无返回值
 */
static void validate_checksum(JY901S_Handle *handle, uint8_t received_sum) {
    JY901S_Parser *parser = &handle->parser;

    if ((parser->check_sum & 0xFF) == received_sum) {
        update_sensor_data(handle);
    }
}

/**
 * @brief 更新传感器数据
 *
 * 根据帧类型调用相应的解析函数处理数据
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
static void update_sensor_data(JY901S_Handle *handle) {
    switch (handle->parser.frame_state) {
        case 1:
            parse_acceleration(handle);
            break;
        case 2:
            parse_gyroscope(handle);
            break;
        case 3:
            parse_angle(handle);
            break;
    }
}

/**
 * @brief 解析加速度数据
 *
 * 将原始加速度数据转换为实际物理值(m/s²)
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
static void parse_acceleration(JY901S_Handle *handle) {
    JY901S_Parser *parser = &handle->parser;
    JY901S_Data *data = &handle->sensor_data;

    int16_t rawX = (int16_t)((parser->acc_buffer[1] << 8) | parser->acc_buffer[0]);
    int16_t rawY = (int16_t)((parser->acc_buffer[3] << 8) | parser->acc_buffer[2]);
    int16_t rawZ = (int16_t)((parser->acc_buffer[5] << 8) | parser->acc_buffer[4]);

    const float scale = 16.0f * 9.80665f; // 转换为m/s²
    data->acc[0] = (float)rawX / 32768.0f * scale;
    data->acc[1] = (float)rawY / 32768.0f * scale;
    data->acc[2] = (float)rawZ / 32768.0f * scale;
}

/**
 * @brief 解析陀螺仪数据
 *
 * 将原始陀螺仪数据转换为实际物理值(°/s)
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
static void parse_gyroscope(JY901S_Handle *handle) {
    JY901S_Parser *parser = &handle->parser;
    JY901S_Data *data = &handle->sensor_data;

    int16_t rawX = (int16_t)((parser->gyro_buffer[1] << 8) | parser->gyro_buffer[0]);
    int16_t rawY = (int16_t)((parser->gyro_buffer[3] << 8) | parser->gyro_buffer[2]);
    int16_t rawZ = (int16_t)((parser->gyro_buffer[5] << 8) | parser->gyro_buffer[4]);

    const float scale = 2000.0f; // 量程±2000°/s
    data->gyro[0] = (float)rawX / 32768.0f * scale;
    data->gyro[1] = (float)rawY / 32768.0f * scale;
    data->gyro[2] = (float)rawZ / 32768.0f * scale;
}

/**
 * @brief 解析角度数据
 *
 * 将原始角度数据转换为实际角度值(°)，并将Yaw转换为0-360°范围
 *
 * @param handle 指向JY901S_Handle的指针
 *
 * @return 无返回值
 */
static void parse_angle(JY901S_Handle *handle) {
    JY901S_Parser *parser = &handle->parser;
    JY901S_Data *data = &handle->sensor_data;

    int16_t rawRoll = (int16_t)((parser->angle_buffer[1] << 8) | parser->angle_buffer[0]);
    int16_t rawPitch = (int16_t)((parser->angle_buffer[3] << 8) | parser->angle_buffer[2]);
    int16_t rawYaw = (int16_t)((parser->angle_buffer[5] << 8) | parser->angle_buffer[4]);

    const float scale = 180.0f; // 量程±180°
    data->angle[0] = (float)rawRoll / 32768.0f * scale;  // Roll
    data->angle[1] = (float)rawPitch / 32768.0f * scale; // Pitch
    data->angle[2] = (float)rawYaw / 32768.0f * scale;   // Yaw

    // 将Yaw转换为0-360°
    if (data->angle[2] < 0) {
        data->angle[2] += 360.0f;
    }
}