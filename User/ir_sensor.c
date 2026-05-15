#include "ir_sensor.h"

static int last_valid_state = 1; 

void IR_Read(int *left1, int *left2, int *right2, int *right1)
{
    // 物理逻辑：遇到黑线 -> 小灯亮 -> GPIO_PIN_SET ->  1 
    //          遇到白底 -> 小灯灭 -> GPIO_PIN_RESET ->  0 
    *left1  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) ? 1 : 0;  // SW1
    *left2  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ? 1 : 0;  // SW2
    *right2 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) ? 1 : 0;  // SW3
    *right1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) ? 1 : 0;  // SW4
}

int IR_GetCarState(void)
{
    int L1, L2, R2, R1;
    IR_Read(&L1, &L2, &R2, &R1); 

    // 判断十字路口：四个全在黑线上 (全是 1)
    if (L1 == 1 && L2 == 1 && R2 == 1 && R1 == 1) {
        return 0; // 十字路口/停止线
    }

    // 转为 4位二进制对应的数值，1表示压在黑线上，0表示在白底上
    // 位顺序: [L1][L2][R2][R1]
    uint8_t sensor_map = (L1 << 3) | (L2 << 2) | (R2 << 1) | R1;

    // 使用十六进制 (0x) 替代 GCC 特有的二进制 (0b)，兼容 Keil 编译器
    switch(sensor_map)
    {
        case 0x06: // 对应二进制 0110: 中间两个压线
            last_valid_state = 1; 
            return 1; // 直行

        case 0x04: // 对应二进制 0100: 偏右（左内压线）
        case 0x0E: // 对应二进制 1110: 偏右（左侧两探头压线，右侧出线）
            last_valid_state = 2; 
            return 2; // 轻微左转
            
        case 0x02: // 对应二进制 0010: 偏左（右内压线）
        case 0x07: // 对应二进制 0111: 偏左（右侧两探头压线，左侧出线）
            last_valid_state = 3; 
            return 3; // 轻微右转

        case 0x0C: // 对应二进制 1100: 严重偏右（左外+左内）
        case 0x08: // 对应二进制 1000: 极度偏右（仅左外）
            last_valid_state = 4; 
            return 4; // 急左转
            
        case 0x03: // 对应二进制 0011: 严重偏左（右外+右内）
        case 0x01: // 对应二进制 0001: 极度偏左（仅右外）
            last_valid_state = 5; 
            return 5; // 急右转

        case 0x00: // 对应二进制 0000: 全白 (全灭)
            // 根据历史状态决定往哪边找线
            if (last_valid_state == 4 || last_valid_state == 2) {
                return 4; // 之前在左偏，继续急左转找线
            } else if (last_valid_state == 5 || last_valid_state == 3) {
                return 5; // 之前在右偏，继续急右转找线
            } else {
                return 1; // 
            }

        default:
            return last_valid_state; 
    }
}