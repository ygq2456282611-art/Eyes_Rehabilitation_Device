#ifndef VOICE_H
#define VOICE_H

#include "main.h"

/* ===== 语音命令（TYPE=0x00, AA 55 00 XX FB）===== */
#define VOICE_CMD_CALIB_STILL   0x01  /* 校准静态 */
#define VOICE_CMD_FIXATION      0x02  /* 注视稳定训练 */
#define VOICE_CMD_SACCADE       0x03  /* 扫视训练 */
#define VOICE_CMD_PURSUIT       0x04  /* 平稳追踪训练 */
#define VOICE_CMD_FOCUS         0x05  /* 视觉聚焦训练 */
#define VOICE_CMD_NEGLECT       0x06  /* 空间忽略训练 */
#define VOICE_CMD_CALIB_MODE    0x07  /* 标定模式 */
#define VOICE_CMD_MODE_SWITCH   0x08  /* 模式切换 */
#define VOICE_CMD_RESTART       0x09  /* 重新开始 */
#define VOICE_CMD_SKIP          0x0A  /* 跳过这个 */

/* ===== TTS播报（TYPE=0xFF, AA 55 FF XX FB）===== */

/* 基础反馈 0x01-0x0B */
#define VOICE_TTS_CALIB_DONE    0x01  /* 校准完成 */
#define VOICE_TTS_KEEP_STILL    0x02  /* 请保持头部稳定 */
#define VOICE_TTS_CORRECT       0x03  /* 答得很棒 */
#define VOICE_TTS_TIMEOUT       0x04  /* 时间到咯，再试一次 */
#define VOICE_TTS_MISS          0x05  /* 错过，请注意该位置 */
#define VOICE_TTS_EYE_ONLY      0x06  /* 请用眼睛跟踪，不要转头 */
#define VOICE_TTS_FIND_LIGHT    0x07  /* 请寻找光点 */
#define VOICE_TTS_NEGLECT_HINT  0x08  /* 超时，请多注意该侧空间 */
#define VOICE_TTS_INIT_OK       0x09  /* 初始化完成 */
#define VOICE_TTS_TRAIN_DONE    0x0A  /* 训练结束，太棒了 */
#define VOICE_TTS_WELCOME       0x0B  /* 欢迎 */

/* 训练结果分级评价 0x0C-0x0E */
#define VOICE_TTS_RESULT_GREAT  0x0C  /* 表现非常出色，继续加油 */
#define VOICE_TTS_RESULT_GOOD   0x0D  /* 表现不错，再接再厉 */
#define VOICE_TTS_RESULT_TRY    0x0E  /* 还需努力，别灰心 */

/* 康复激励 0x0F-0x14 */
#define VOICE_TTS_CONTINUE      0x0F  /* 继续训练 */
#define VOICE_TTS_ENCOURAGE     0x10  /* 加油，您做得很好 */
#define VOICE_TTS_HALFWAY       0x11  /* 已完成一半，坚持住 */
#define VOICE_TTS_STREAK        0x12  /* 连续正确，保持状态 */
#define VOICE_TTS_RELAX         0x13  /* 不要太着急，慢慢来 */
#define VOICE_TTS_BREATHE       0x14  /* 请放松肩膀，自然呼吸 */

/* 过渡衔接 0x15-0x16 */
#define VOICE_TTS_NEXT_PREP     0x15  /* 请稍事休息，准备下一个训练 */
#define VOICE_TTS_ALL_DONE      0x16  /* 全部训练完成，您辛苦了 */

/* 指导纠错 0x17-0x1C */
#define VOICE_TTS_POSTURE       0x17  /* 训练暂停，请调整坐姿 */
#define VOICE_TTS_EYES_NOHEAD   0x18  /* 请用眼睛去寻找，头部不要转动 */
#define VOICE_TTS_LEFT_SIDE     0x19  /* 请注意左侧空间 */
#define VOICE_TTS_RIGHT_SIDE    0x1A  /* 请注意右侧空间 */
#define VOICE_TTS_FOUND_SIDE    0x1B  /* 很好，您发现了患侧的光点 */
#define VOICE_TTS_FOCUS_NOW     0x1C  /* 请集中注意力看光点 */

/* 专项评价 0x1D-0x23 */
#define VOICE_TTS_FIX_GOOD      0x1D  /* 注视训练完成，头部稳定性良好 */
#define VOICE_TTS_FIX_FAIR      0x1E  /* 注视训练完成，头部稳定性一般 */
#define VOICE_TTS_FIX_POOR      0x1F  /* 注视训练完成，稳定性较差需加强 */
#define VOICE_TTS_REACT_FAST    0x20  /* 反应速度很快，非常棒 */
#define VOICE_TTS_REACT_SLOW    0x21  /* 反应速度还需提升，继续练习 */
#define VOICE_TTS_PUR_GOOD      0x22  /* 追踪训练完成，跟踪能力良好 */
#define VOICE_TTS_PUR_HEAD      0x23  /* 追踪训练完成，尝试减少转头 */
#define VOICE_TTS_START_CALIB   0x24  /* 请调整好姿态，按确认键开始校准 */
#define VOICE_TTS_MODE_SELECT   0x25  /* 按一下进入完整训练，按两下进入自选模式 */
#define VOICE_TTS_NEXT_CONFIRM  0x26  /* 按键确认进入下一个模式 */
#define VOICE_TTS_START_FIX     0x27  /* 注视训练开始提示 */
#define VOICE_TTS_START_SAC     0x28  /* 扫视训练开始提示 */
#define VOICE_TTS_START_PUR     0x29  /* 追踪训练开始提示 */
#define VOICE_TTS_START_FOCUS   0x2A  /* 聚焦训练开始提示 */
#define VOICE_TTS_START_NEGLECT 0x2B  /* 空间忽略训练开始提示 */
#define VOICE_TTS_ENTER_FULL_MODE   0x2C  /* 成功进入完整训练模式 */
#define VOICE_TTS_ENTER_CUSTOM_MODE 0x2D  /* 成功进入自选模式 */

/* 函数 */
void Voice_Init(void);
void Voice_Play(uint8_t type, uint8_t id);
void Voice_SendFrame(uint8_t type, uint8_t id);
uint8_t Voice_GetCommand(void);
UART_HandleTypeDef* Voice_GetUART(void);
uint8_t Voice_GetLastTxType(void);
uint8_t Voice_GetLastTxId(void);
uint32_t Voice_GetLastTxTick(void);

/* Voice_GetCommand() 的特殊返回值（唤醒词区命令） */
#define VOICE_CMD_FROM_WAKE(t)  (0x80 | (t))
#define VOICE_CMD_IS_WAKE(cmd)  ((cmd) >= 0x80)
#define VOICE_CMD_WAKE_PAUSE    0x81  /* 暂停训练 (TYPE=0x01) */
#define VOICE_CMD_WAKE_RESUME   0x86  /* 继续训练 (TYPE=0x06) */
#define VOICE_CMD_WAKE_RESTART  0x89  /* 重新开始 (TYPE=0x09) */
#define VOICE_CMD_WAKE_SKIP     0x8A  /* 跳过这个 (TYPE=0x0A) */
#define VOICE_CMD_WAKE_HELLO    0x83  /* 你好小盈 (TYPE=0x03) 统一中断 */

#endif
