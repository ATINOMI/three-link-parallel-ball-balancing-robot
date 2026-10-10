/**
  ******************************************************************************
  * @file    motor_command.c
  * @brief   电机步进指令：解析 USB 文本命令并向控制中断传递步进事件。
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------*/
#include "motor_command.h"

#include "main.h"
#include <string.h>

/* 私有定义与变量 ------------------------------------------------*/
#define COMMAND_QUEUE_SIZE 16U  /* 环形队列保留一个空位，最多保存 15 条指令。 */
#define COMMAND_TIMEOUT_MS 250U  /* 接收残留行和排队指令的有效期限，单位：毫秒。 */

typedef struct {
  uint8_t index;  /*!< 电机数组下标，取值为 0～2。 */
  int8_t direction;  /*!< 单次步进方向，+1 增加目标，-1 减少目标。 */
  uint32_t arrival_ms;  /*!< 命令入队时刻，单位：毫秒。 */
} MotorCommand;

static volatile MotorCommand queue[COMMAND_QUEUE_SIZE];
static volatile uint8_t read_index;
static volatile uint8_t write_index;
static char line[16];
static uint8_t used;
static uint8_t discard_line;  /* 丢弃当前行，直到收到下一个 LF。 */
static uint32_t last_byte_ms;
volatile uint32_t motor_command_dropped;

/* 函数实现 ------------------------------------------------*/
/**
  * @brief  校验一条完整命令并写入步进队列。
  * @note   仅接受 STEP <1..3> <+10|-10>，行尾允许 CR；非法命令或队列满时丢弃。
  * @retval 无。
  */
static void ParseLine(void) {
  // 严格匹配 STEP <1..3> <+10|-10>，LF 前允许一个 CR。
  if (used && line[used - 1U] == '\r') --used;
  if (used != 10U || memcmp(line, "STEP ", 5U) != 0 ||
      line[5] < '1' || line[5] > '3' || line[6] != ' ' ||
      (line[7] != '+' && line[7] != '-') ||
      line[8] != '1' || line[9] != '0') {
    ++motor_command_dropped;
    return;
  }
  uint8_t next = (uint8_t)((write_index + 1U) % COMMAND_QUEUE_SIZE);
  if (next == read_index) {
    ++motor_command_dropped;
    return;
  }
  queue[write_index].index = (uint8_t)(line[5] - '1');
  queue[write_index].direction = line[7] == '+' ? 1 : -1;
  queue[write_index].arrival_ms = HAL_GetTick();
  /* 数据填充完毕后再更新写索引，使消费者只读取完整命令。 */
  write_index = next;
}

/**
  * @brief  按字节接收并解析 USB 步进命令。
  * @param  data: 接收数据指针，可包含半条命令或多条命令。
  * @param  length: 本次数据长度。
  * @note   由 USB 接收回调调用；以 LF 结束一行，超长或超时残留行丢弃到下一个 LF。
  * @retval 无。
  */
void Motor_CommandFeed(const uint8_t* data, uint32_t length) {
  if (data == NULL) return;
  uint32_t now = HAL_GetTick();
  if ((used || discard_line) && now - last_byte_ms >= COMMAND_TIMEOUT_MS) {
    // 接收间隔超时后不再拼接旧残留命令，丢弃到下一处换行。
    used = 0;
    discard_line = 1U;
  }
  last_byte_ms = now;
  for (uint32_t i = 0; i < length; ++i) {
    if (data[i] == '\n') {
      if (!discard_line && used) ParseLine();
      used = 0;
      discard_line = 0;
    } else if (!discard_line) {
      if (used < sizeof(line)) line[used++] = (char)data[i];
      else {
        used = 0;
        discard_line = 1U;
        ++motor_command_dropped;
      }
    }
  }
}

/**
  * @brief  清空步进队列与命令行解析状态。
  * @note   在临界区内复位队列索引；保留累计丢弃计数。
  * @retval 无。
  */
void Motor_CommandReset(void) {
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  read_index = write_index = used = discard_line = 0;
  __set_PRIMASK(saved);
}

/**
  * @brief  取出队列中的有效指令并按电机合并步进数。
  * @param  steps: 三元素输出数组；正值为增加步数，负值为减少步数。
  * @note   由控制中断调用；丢弃到达时间达到有效期限的命令。
  * @retval 无。
  */
void Motor_CommandTakeSteps(int16_t steps[3]) {
  steps[0] = steps[1] = steps[2] = 0;
  // 短临界区保护队列读取，避免优先级更高的 USB 中断抢占。
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  uint32_t now = HAL_GetTick();
  while (read_index != write_index) {
    if (now - queue[read_index].arrival_ms < COMMAND_TIMEOUT_MS) {
      steps[queue[read_index].index] += queue[read_index].direction;
    } else {
      ++motor_command_dropped;
    }
    read_index = (uint8_t)((read_index + 1U) % COMMAND_QUEUE_SIZE);
  }
  __set_PRIMASK(saved);
}
