#ifndef __MAVLINK_PORT_H
#define __MAVLINK_PORT_H

#include <stdint.h>

/*
 * 极简 MAVLink v1 实现（手写，非官方 C 库）
 * ------------------------------------------------------------------
 * 为什么不用官方 mavlink/c_library_v2:
 *   - STM32F103C8 只有 20KB RAM，map 实测已用 16.5KB，余量 ~3.8KB。
 *     官方库 mavlink_message_t + 收发缓冲 + status 约 0.9KB，且
 *     MAVLINK_HELPER = static inline，头文件里的 static 缓冲会在每个
 *     包含它的 .o 里各复制一份。
 *   - ARMCC 5.06 (AC5) 不支持 __attribute__((packed))，需要改 MAVPACKED。
 * 因此这里只实现用到的 15 条消息，RAM 约 0.5KB，全部集中于本模块。
 *
 * 链路: USART1 (PA9/PA10), 57600 8N1, MAVLink v1
 *       sysid=1, compid=1(MAV_COMP_ID_AUTOPILOT1)
 * ------------------------------------------------------------------
 * CRC_EXTRA 每一格都以 <message> 的"线序"(按字段宽度降序重排) + 基础字段
 * (不含 MAVLink2 extensions) 独立算出并核对过，改动前请重新核对。
 */

#define MAV_STX_V1            0xFE
#define MAV_SYSID             1
#define MAV_COMPID            1      /* MAV_COMP_ID_AUTOPILOT1 */
#define MAV_TARGET_SYS        255

/* 串口参数只在这里改，INIT.c 用 ANO_Uart1_Init(MAV_SERIAL_BAUD)
 * 常见取值：57600 / 115200 / 19200（老匿名模块多为 19200，必须与模块一致）*/
#ifndef MAV_SERIAL_BAUD
#define MAV_SERIAL_BAUD       57600
#endif

/* 把波特率数字变成字符串，用于开机横幅 */
#define MAV_STR_(x)           #x
#define MAV_STR(x)            MAV_STR_(x)
#define MAV_SERIAL_BAUD_STR   MAV_STR(MAV_SERIAL_BAUD)

/* 上电后在 MAVLink 流前面先发一行纯 ASCII 文本。
 * 作用：随便一个串口助手（波特率与上面一致）就能看到它，
 *       用来区分"串口完全没输出" / "波特率不对" / "输出正常但地面站没解析"。
 * 调试完成后置 0 即可（不影响 MAVLink 协议本身）。*/
#ifndef MAVLINK_BOOT_ASCII
#define MAVLINK_BOOT_ASCII    1
#endif

/* 是否允许地面站触发加速度/角速度校准（MAV_CMD_PREFLIGHT_CALIBRATION）。
 * 默认 0：MpuGetOffset() 会与 2ms 控制任务共用软件 I2C 而没有互斥，
 * 并发时可能算出错误零偏。要开启请先给 I2C 加互斥或把它挪进控制任务。*/
#ifndef MAVLINK_ALLOW_CALIBRATION
#define MAVLINK_ALLOW_CALIBRATION 0
#endif

#define MAV_MAX_PAYLOAD       64     /* 本机最长发送载荷 = STATUSTEXT 51 字节 */
#define MAV_MAX_FRAME         (8 + MAV_MAX_PAYLOAD)

/* ---------------- message id ---------------- */
#define MAV_MSG_HEARTBEAT            0
#define MAV_MSG_SYS_STATUS           1
#define MAV_MSG_SET_MODE             11
#define MAV_MSG_PARAM_REQUEST_READ   20
#define MAV_MSG_PARAM_REQUEST_LIST   21
#define MAV_MSG_PARAM_VALUE          22
#define MAV_MSG_PARAM_SET            23
#define MAV_MSG_RAW_IMU              27
#define MAV_MSG_ATTITUDE             30
#define MAV_MSG_RC_CHANNELS          65
#define MAV_MSG_REQUEST_DATA_STREAM  66
#define MAV_MSG_VFR_HUD              74
#define MAV_MSG_COMMAND_LONG         76
#define MAV_MSG_COMMAND_ACK          77
#define MAV_MSG_STATUSTEXT           253

/* ---------------- MAV_RESULT ---------------- */
#define MAV_RESULT_ACCEPTED              0
#define MAV_RESULT_TEMPORARILY_REJECTED  1
#define MAV_RESULT_DENIED                2
#define MAV_RESULT_UNSUPPORTED           3
#define MAV_RESULT_FAILED                4

/* ---------------- MAV_SEVERITY ---------------- */
#define MAV_SEVERITY_EMERGENCY  0
#define MAV_SEVERITY_ALERT      1
#define MAV_SEVERITY_CRITICAL   2
#define MAV_SEVERITY_ERROR      3
#define MAV_SEVERITY_WARNING    4
#define MAV_SEVERITY_NOTICE     5
#define MAV_SEVERITY_INFO       6
#define MAV_SEVERITY_DEBUG      7

/* ---------------- 常用 MAV_CMD ---------------- */
#define MAV_CMD_DO_SET_MODE                     176
#define MAV_CMD_PREFLIGHT_CALIBRATION           241
#define MAV_CMD_COMPONENT_ARM_DISARM            400
#define MAV_CMD_SET_MESSAGE_INTERVAL            511
#define MAV_CMD_REQUEST_MESSAGE                 512

/* ---------------- 参数类型 ---------------- */
#define MAV_PARAM_TYPE_REAL32     9

/* ---------------- MAV_DATA_STREAM ---------------- */
#define MAV_DATA_STREAM_RAW_SENSORS    1
#define MAV_DATA_STREAM_EXTENDED_STATUS 2
#define MAV_DATA_STREAM_RC_CHANNELS    3
#define MAV_DATA_STREAM_EXTRA1         10
#define MAV_DATA_STREAM_EXTRA2         11
#define MAV_DATA_STREAM_EXTRA3         12

/* ================= 载荷小端读写 ================= */
#define MAV_PUT_U8(p, v)   do { (p)[0] = (uint8_t)(v); } while(0)
#define MAV_PUT_I8(p, v)   do { (p)[0] = (uint8_t)(int8_t)(v); } while(0)
#define MAV_PUT_U16(p, v)  do { (p)[0] = (uint8_t)((uint16_t)(v) & 0xFFu); \
                                (p)[1] = (uint8_t)(((uint16_t)(v) >> 8) & 0xFFu); } while(0)
#define MAV_PUT_I16(p, v)  MAV_PUT_U16(p, (uint16_t)(int16_t)(v))
#define MAV_PUT_U32(p, v)  do { (p)[0] = (uint8_t)((uint32_t)(v) & 0xFFu); \
                                (p)[1] = (uint8_t)(((uint32_t)(v) >> 8) & 0xFFu); \
                                (p)[2] = (uint8_t)(((uint32_t)(v) >> 16) & 0xFFu); \
                                (p)[3] = (uint8_t)(((uint32_t)(v) >> 24) & 0xFFu); } while(0)

#define MAV_GET_U8(p)      ((uint8_t)((p)[0]))
#define MAV_GET_U16(p)     ((uint16_t)((uint16_t)((p)[0]) | ((uint16_t)((p)[1]) << 8)))
#define MAV_GET_I16(p)     ((int16_t)MAV_GET_U16(p))
#define MAV_GET_U32(p)     (((uint32_t)((p)[0])) | ((uint32_t)((p)[1]) << 8) | \
                            ((uint32_t)((p)[2]) << 16) | ((uint32_t)((p)[3]) << 24))

void  MAV_PutF32(uint8_t *p, float v);
float MAV_GetF32(const uint8_t *p);

/* ================= 对外接口 ================= */
/* 在 main() 最前面调用：立刻初始化 USART1 并发开机横幅，
 * 不依赖 MPU/NRF 等后续初始化是否成功，便于判断"固件到底跑起来没有" */
void    MAVLink_EarlyInit(void);

void    MAVLink_Init(void);
void    MAVLink_Update(uint32_t now_ms);      /* 放到 4ms 通信任务里调用 */

uint8_t MAVLink_SendMessage(uint8_t msgid, const uint8_t *payload, uint8_t len);
void    MAVLink_SendStatustext(uint8_t severity, const char *text);

/* 消息流速率控制；interval_us < 0 关闭，0 恢复默认；返回 0=成功 1=未知消息 */
uint8_t MAVLink_SetStreamInterval(uint8_t msgid, int32_t interval_us);
/* 立即发一条；返回 0=已发送 1=不支持该消息 */
uint8_t MAVLink_SendMessageOnce(uint8_t msgid);

uint8_t MAVLink_GcsAlive(void);               /* 最近 5s 是否收到地面站心跳 */
uint8_t MAVLink_RxDropCount(void);            /* 校验失败/溢出计数，便于排查 */

/* ================= mavlink_app.c 提供 ================= */
void MAVApp_Init(void);
void MAVApp_ParamListTick(void);
void MAVApp_HandleParamRequestList(const uint8_t *payload);
void MAVApp_HandleParamRequestRead(const uint8_t *payload);
void MAVApp_HandleParamSet(const uint8_t *payload);
void MAVApp_HandleCommandLong(const uint8_t *payload);
uint8_t MAVApp_ParamCount(void);

#endif /* __MAVLINK_PORT_H */
