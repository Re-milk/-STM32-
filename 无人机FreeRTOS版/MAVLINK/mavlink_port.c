//========================================================================
//	MAVLink v1 移植层（USART1 / 57600 8N1）
//	组帧、X25 校验、解析、消息流调度、各条上行消息的填充
//
//	载荷字段顺序 = MAVLink 线序（按字段宽度降序重排，等宽字段保持 XML
//	声明顺序），不是 XML 里的书写顺序。每条消息偏移都按此推导并核对过
//	CRC_EXTRA，改动前请重新核对。
//========================================================================

#include <string.h>
#include "mavlink_port.h"
#include "ALL_DATA.h"
#include "Uart1.h"

/* ---------------- 本模块用到的外部数据 ---------------- */
extern _st_AngE       Angle;
extern _st_Mpu        MPU6050;
extern _st_Mag        AK8975;
extern _st_Remote     Remote;
extern _st_ALL_flag   ALL_flag;
extern _st_FlightData FlightData;
extern st_Command     Command;

extern PidObject pidRateX;
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern s16 voltage;                     /* 单位 mV */
extern u8  NRF_SSI, WIFI_SSI;
extern u8  MPU_Err, SPL_Err, Flow_Err, ult_ok;
extern const float Gyro_Gr;             /* rad/s per LSB */

#define MAV_DEG2RAD   0.01745329252f
#define MAV_GCS_TIMEOUT_MS   5000u
#define MAV_RX_IDLE_TIMEOUT_MS  50u

/************************************************************************
 * 状态
 ************************************************************************/
static uint8_t  mav_tx_seq;

typedef struct
{
	uint8_t  state;
	uint8_t  len;
	uint8_t  idx;
	uint8_t  msgid;
	uint8_t  sysid;
	uint8_t  compid;
	uint8_t  crc_lo;
	uint8_t  crc_hi;
	uint16_t crc;
	uint8_t  payload[MAV_MAX_PAYLOAD];
} mav_rx_t;

static mav_rx_t  mav_rx;
static uint8_t   mav_rx_drop;
static uint32_t  mav_now_ms;
static uint32_t  mav_rx_byte_ms;
static uint32_t  mav_tick_last_ms;
static uint32_t  mav_gcs_last_ms;
static uint8_t   mav_gcs_seen;

typedef struct
{
	uint8_t  msgid;
	uint8_t  enabled;
	uint16_t period_ms;
	uint32_t last_ms;
} mav_stream_t;

/* 默认速率（57600 下总占用约 2KB/s，余量充足）
 * 可由地面站 MAV_CMD_SET_MESSAGE_INTERVAL / REQUEST_DATA_STREAM 改写 */
static mav_stream_t mav_streams[] =
{
	{ MAV_MSG_HEARTBEAT,   1, 1000, 0 },
	{ MAV_MSG_SYS_STATUS,  1, 1000, 0 },
	{ MAV_MSG_ATTITUDE,    1,   50, 0 },
	{ MAV_MSG_VFR_HUD,     1,  200, 0 },
	{ MAV_MSG_RC_CHANNELS, 1,  200, 0 },
	{ MAV_MSG_RAW_IMU,     1,  100, 0 }
};

#define MAV_STREAM_COUNT  (sizeof(mav_streams) / sizeof(mav_streams[0]))

/************************************************************************
 * CRC-16/X25（MAVLink 用的那种）
 ************************************************************************/
static uint16_t mav_crc_accum(uint16_t crc, uint8_t b)
{
	uint8_t tmp;

	tmp = (uint8_t)(b ^ (uint8_t)(crc & 0xFFu));
	tmp = (uint8_t)(tmp ^ (uint8_t)((tmp << 4) & 0xFFu));

	return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)(tmp >> 4)));
}

/* 每格 = 该消息基础字段算出的 CRC_EXTRA */
static uint8_t mav_crc_extra(uint8_t msgid)
{
	switch(msgid)
	{
		case MAV_MSG_HEARTBEAT:           return 50;
		case MAV_MSG_SYS_STATUS:          return 124;
		case MAV_MSG_SET_MODE:            return 89;
		case MAV_MSG_PARAM_REQUEST_READ:  return 214;
		case MAV_MSG_PARAM_REQUEST_LIST:  return 159;
		case MAV_MSG_PARAM_VALUE:         return 220;
		case MAV_MSG_PARAM_SET:           return 168;
		case MAV_MSG_RAW_IMU:             return 144;
		case MAV_MSG_ATTITUDE:            return 39;
		case MAV_MSG_RC_CHANNELS:         return 118;
		case MAV_MSG_REQUEST_DATA_STREAM: return 148;
		case MAV_MSG_VFR_HUD:             return 20;
		case MAV_MSG_COMMAND_LONG:        return 152;
		case MAV_MSG_COMMAND_ACK:         return 143;
		case MAV_MSG_STATUSTEXT:          return 83;
		default:                          return 0;
	}
}

/************************************************************************
 * 浮点载荷存取（避免对齐/aliasing 问题，按字节拷贝）
 ************************************************************************/
void MAV_PutF32(uint8_t *p, float v)
{
	union { float f; uint8_t b[4]; } u;

	u.f = v;
	p[0] = u.b[0];
	p[1] = u.b[1];
	p[2] = u.b[2];
	p[3] = u.b[3];
}

float MAV_GetF32(const uint8_t *p)
{
	union { float f; uint8_t b[4]; } u;

	u.b[0] = p[0];
	u.b[1] = p[1];
	u.b[2] = p[2];
	u.b[3] = p[3];
	return u.f;
}

/************************************************************************
 * 组帧并投递到 USART1 发送缓冲（非阻塞）
 ************************************************************************/
uint8_t MAVLink_SendMessage(uint8_t msgid, const uint8_t *payload, uint8_t len)
{
	uint8_t  frame[MAV_MAX_FRAME];
	uint16_t crc = 0xFFFFu;
	uint8_t  i;

	if(len > MAV_MAX_PAYLOAD)
		return 0;

	frame[0] = MAV_STX_V1;
	frame[1] = len;
	frame[2] = mav_tx_seq++;
	frame[3] = MAV_SYSID;
	frame[4] = MAV_COMPID;
	frame[5] = msgid;

	for(i = 1; i < 6u; i++)
		crc = mav_crc_accum(crc, frame[i]);

	for(i = 0; i < len; i++)
	{
		frame[6u + i] = payload[i];
		crc = mav_crc_accum(crc, payload[i]);
	}

	crc = mav_crc_accum(crc, mav_crc_extra(msgid));

	frame[6u + len] = (uint8_t)(crc & 0xFFu);
	frame[7u + len] = (uint8_t)(crc >> 8);

	return UART1_WriteBuf(frame, (uint16_t)(len + 8u));
}

/************************************************************************
 * 各条上行消息
 ************************************************************************/
static void mav_send_heartbeat(void)
{
	uint8_t p[9];
	uint8_t base_mode = 0x01u;                 /* MAV_MODE_FLAG_CUSTOM_MODE_ENABLED */

	if(ALL_flag.unlock)
		base_mode |= 0x80u;                    /* MAV_MODE_FLAG_SAFETY_ARMED */

	MAV_PUT_U32(&p[0], (uint32_t)Command.FlightMode);   /* custom_mode: LOCK/NORMOL/HEIGHT/Flow */
	p[4] = 2;                                  /* MAV_TYPE_QUADROTOR */
	p[5] = 0;                                  /* MAV_AUTOPILOT_GENERIC */
	p[6] = base_mode;
	p[7] = ALL_flag.unlock ? 4u : 3u;          /* MAV_STATE_ACTIVE : MAV_STATE_STANDBY */
	p[8] = 3;                                  /* mavlink_version */

	(void)MAVLink_SendMessage(MAV_MSG_HEARTBEAT, p, 9);
}

static void mav_send_sys_status(void)
{
	uint8_t  p[31];
	uint32_t present = 0;
	uint32_t health  = 0;
	uint16_t vbat;
	uint8_t  i;

	/* MAV_SYS_STATUS_SENSOR 位 */
	if(!MPU_Err)
	{
		present |= (1u << 0);                  /* 3D_GYRO  */
		present |= (1u << 1);                  /* 3D_ACCEL */
		health  |= (1u << 0);
		health  |= (1u << 1);
	}
	present |= (1u << 2);                      /* 3D_MAG：机上焊了 AK8975 */
	health  |= (1u << 2);
	if(!SPL_Err)
	{
		present |= (1u << 3);                  /* ABSOLUTE_PRESSURE */
		health  |= (1u << 3);
	}
	if(!Flow_Err)
	{
		present |= (1u << 6);                  /* OPTICAL_FLOW */
		health  |= (1u << 6);
	}
	if(ult_ok)
	{
		present |= (1u << 8);                  /* LASER_POSITION */
		health  |= (1u << 8);
	}
	present |= (1u << 15);                     /* MOTOR_OUTPUTS */
	health  |= (1u << 15);
	present |= (1u << 16);                     /* RC_RECEIVER */
	health  |= (1u << 16);

	vbat = (voltage > 0) ? (uint16_t)voltage : 0u;

	MAV_PUT_U32(&p[0],  present);
	MAV_PUT_U32(&p[4],  present);              /* enabled */
	MAV_PUT_U32(&p[8],  health);
	MAV_PUT_U16(&p[12], 0);                    /* load */
	MAV_PUT_U16(&p[14], vbat);                 /* voltage_battery, mV */
	MAV_PUT_I16(&p[16], -1);                   /* current_battery, 未知 */
	MAV_PUT_U16(&p[18], 0);                    /* drop_rate_comm */
	MAV_PUT_U16(&p[20], 0);                    /* errors_comm */
	for(i = 0; i < 4u; i++)
		MAV_PUT_U16(&p[22u + i * 2u], 0);      /* errors_count1..4 */
	p[30] = (uint8_t)0xFF;                     /* battery_remaining = -1 */

	(void)MAVLink_SendMessage(MAV_MSG_SYS_STATUS, p, 31);
}

static void mav_send_attitude(void)
{
	uint8_t p[28];

	/* 与匿名上位机 STATUS 帧保持同一套符号约定：roll / yaw 取反 */
	MAV_PUT_U32(&p[0],  mav_now_ms);                             /* time_boot_ms */
	MAV_PutF32(&p[4],   -Angle.roll  * MAV_DEG2RAD);
	MAV_PutF32(&p[8],    Angle.pitch * MAV_DEG2RAD);
	MAV_PutF32(&p[12],  -Angle.yaw   * MAV_DEG2RAD);
	MAV_PutF32(&p[16],  -pidRateX.measured * MAV_DEG2RAD);        /* rollspeed  */
	MAV_PutF32(&p[20],   pidRateY.measured * MAV_DEG2RAD);        /* pitchspeed */
	MAV_PutF32(&p[24],  -pidRateZ.measured * MAV_DEG2RAD);        /* yawspeed  */

	(void)MAVLink_SendMessage(MAV_MSG_ATTITUDE, p, 28);
}

static void mav_send_vfr_hud(void)
{
	uint8_t  p[20];
	int16_t  heading;
	uint16_t throttle;

	heading = (int16_t)(-Angle.yaw);           /* 与左右方向约定一致 */
	while(heading < 0)      heading = (int16_t)(heading + 360);
	while(heading >= 360)   heading = (int16_t)(heading - 360);

	if(Remote.thr > 1000u)
	{
		throttle = (uint16_t)((Remote.thr - 1000u) / 10u);
		if(throttle > 100u) throttle = 100u;
	}
	else
	{
		throttle = 0;
	}

	MAV_PutF32(&p[0],  0.0f);                                    /* airspeed    */
	MAV_PutF32(&p[4],  0.0f);                                    /* groundspeed */
	MAV_PutF32(&p[8],  FlightData.High.bara_height * 0.01f);     /* alt: cm->m  */
	MAV_PutF32(&p[12], 0.0f);                                    /* climb       */
	MAV_PUT_I16(&p[16], heading);
	MAV_PUT_U16(&p[18], throttle);

	(void)MAVLink_SendMessage(MAV_MSG_VFR_HUD, p, 20);
}

static void mav_send_rc_channels(void)
{
	uint8_t  p[42];
	uint16_t ch[10];
	uint8_t  i;

	ch[0] = Remote.roll;
	ch[1] = Remote.pitch;
	ch[2] = Remote.thr;
	ch[3] = Remote.yaw;
	ch[4] = Remote.AUX1;
	ch[5] = Remote.AUX2;
	ch[6] = Remote.AUX3;
	ch[7] = Remote.AUX4;
	ch[8] = Remote.AUX5;
	ch[9] = Remote.AUX6;

	MAV_PUT_U32(&p[0], mav_now_ms);            /* time_boot_ms */
	for(i = 0; i < 18u; i++)
	{
		uint16_t v = (i < 10u) ? ch[i] : 0u;
		MAV_PUT_U16(&p[4u + i * 2u], v);       /* chan1_raw .. chan18_raw */
	}
	p[40] = 10;                                /* chancount */
	p[41] = (NRF_SSI > WIFI_SSI) ? NRF_SSI : WIFI_SSI;   /* rssi */

	(void)MAVLink_SendMessage(MAV_MSG_RC_CHANNELS, p, 42);
}

static void mav_send_raw_imu(void)
{
	uint8_t  p[26];
	uint64_t us = (uint64_t)mav_now_ms * 1000u;
	uint8_t  i;

	for(i = 0; i < 8u; i++)
		p[i] = (uint8_t)((us >> (8u * i)) & 0xFFu);   /* time_usec */

	/* 加速度: MPU6050 原始 LSB(±2g) -> mG (16384 LSB/g) */
	MAV_PUT_I16(&p[8],  (int16_t)((float)MPU6050.accX * (1000.0f / 16384.0f)));
	MAV_PUT_I16(&p[10], (int16_t)((float)MPU6050.accY * (1000.0f / 16384.0f)));
	MAV_PUT_I16(&p[12], (int16_t)((float)MPU6050.accZ * (1000.0f / 16384.0f)));
	/* 角速度: LSB -> mrad/s */
	MAV_PUT_I16(&p[14], (int16_t)((float)MPU6050.gyroX * Gyro_Gr * 1000.0f));
	MAV_PUT_I16(&p[16], (int16_t)((float)MPU6050.gyroY * Gyro_Gr * 1000.0f));
	MAV_PUT_I16(&p[18], (int16_t)((float)MPU6050.gyroZ * Gyro_Gr * 1000.0f));
	/* 磁力计: 原样上报 */
	MAV_PUT_I16(&p[20], AK8975.magX);
	MAV_PUT_I16(&p[22], AK8975.magY);
	MAV_PUT_I16(&p[24], AK8975.magZ);

	(void)MAVLink_SendMessage(MAV_MSG_RAW_IMU, p, 26);
}

void MAVLink_SendStatustext(uint8_t severity, const char *text)
{
	uint8_t p[51];
	uint8_t i;

	memset(p, 0, sizeof(p));
	p[0] = severity;
	for(i = 0; i < 50u && text[i] != '\0'; i++)
		p[1u + i] = (uint8_t)text[i];

	(void)MAVLink_SendMessage(MAV_MSG_STATUSTEXT, p, 51);
}

#if MAVLINK_BOOT_ASCII
/* 发送一段纯 ASCII（非阻塞，进 TX 环形缓冲） */
static void mav_send_ascii(const char *txt)
{
	uint16_t n = 0;

	while(txt[n] != '\0' && n < 120u)
		n++;

	(void)UART1_WriteBuf((const uint8_t *)txt, n);
}

/* 无符号十进制转字符串（不占栈太多，够打印毫秒数） */
static uint8_t mav_u32_to_str(uint32_t v, char *out)
{
	char tmp[10];
	uint8_t n = 0;
	uint8_t i;

	do
	{
		tmp[n++] = (char)('0' + (v % 10u));
		v /= 10u;
	} while(v != 0u && n < 10u);

	for(i = 0; i < n; i++)
		out[i] = tmp[n - 1u - i];

	out[n] = '\0';
	return n;
}

/* 1s 一次的存活节拍：证明调度器/通信任务/UART 发送都在工作 */
static void mav_send_tick(uint32_t now_ms)
{
	char    txt[32] = "[FC] tick ";
	char    num[12];
	uint8_t len = 10;
	uint8_t i;

	len = (uint8_t)(len + mav_u32_to_str(now_ms, num));
	for(i = 0; num[i] != '\0'; i++)
		txt[len++] = num[i];

	txt[len++] = ' ';
	txt[len++] = 'm';
	txt[len++] = 's';
	txt[len++] = '\r';
	txt[len++] = '\n';
	txt[len]   = '\0';

	mav_send_ascii(txt);
}

/* 上电横幅：串口助手（同波特率）能直接读到，
 * 带编译日期时间，用来确认"芯片里跑的是不是刚编译的这份固件" */
static void mav_send_boot_banner(void)
{
	static const char txt[] =
		"\r\n[FC] BOOT MAVLink v1 " MAV_SERIAL_BAUD_STR " 8N1 build=" __DATE__ " " __TIME__ "\r\n";

	mav_send_ascii(txt);
}
#endif

/************************************************************************
 * 在 main() 最前面调用：先把 USART1 拉起来并打出横幅，
 * 这样即使后面 MPU 校准/NRF 初始化卡住，也能立刻知道固件跑起来了
 ************************************************************************/
void MAVLink_EarlyInit(void)
{
	ANO_Uart1_Init(MAV_SERIAL_BAUD);

#if MAVLINK_BOOT_ASCII
	mav_send_boot_banner();
#endif
}

static void mav_send_by_id(uint8_t msgid)
{
	switch(msgid)
	{
		case MAV_MSG_HEARTBEAT:   mav_send_heartbeat();   break;
		case MAV_MSG_SYS_STATUS:  mav_send_sys_status();  break;
		case MAV_MSG_ATTITUDE:    mav_send_attitude();    break;
		case MAV_MSG_VFR_HUD:     mav_send_vfr_hud();     break;
		case MAV_MSG_RC_CHANNELS: mav_send_rc_channels(); break;
		case MAV_MSG_RAW_IMU:     mav_send_raw_imu();     break;
		default: break;
	}
}

/* 与 mav_send_by_id 对应的"支持性"判断，供 REQUEST_MESSAGE 回 ACK 用 */
static uint8_t mav_msg_supported(uint8_t msgid)
{
	switch(msgid)
	{
		case MAV_MSG_HEARTBEAT:
		case MAV_MSG_SYS_STATUS:
		case MAV_MSG_ATTITUDE:
		case MAV_MSG_VFR_HUD:
		case MAV_MSG_RC_CHANNELS:
		case MAV_MSG_RAW_IMU:
			return 1;
		default:
			return 0;
	}
}

/************************************************************************
 * 消息流速率控制
 ************************************************************************/
uint8_t MAVLink_SetStreamInterval(uint8_t msgid, int32_t interval_us)
{
	uint8_t i;

	for(i = 0; i < (uint8_t)MAV_STREAM_COUNT; i++)
	{
		if(mav_streams[i].msgid != msgid)
			continue;

		if(interval_us < 0)
		{
			mav_streams[i].enabled = 0;
		}
		else if(interval_us == 0)
		{
			mav_streams[i].enabled  = 1;
			mav_streams[i].period_ms = (msgid == MAV_MSG_HEARTBEAT) ? 1000u : 100u;
		}
		else
		{
			uint32_t ms = (uint32_t)(interval_us / 1000);
			if(ms < 20u)     ms = 20u;       /* 上限 50Hz，防止塞爆 57600 */
			if(ms > 10000u)  ms = 10000u;
			mav_streams[i].period_ms = (uint16_t)ms;
			mav_streams[i].enabled   = 1;
		}
		return 0;
	}
	return 1;
}

uint8_t MAVLink_SendMessageOnce(uint8_t msgid)
{
	if(!mav_msg_supported(msgid))
		return 1;

	mav_send_by_id(msgid);
	return 0;
}

uint8_t MAVLink_GcsAlive(void)
{
	if(!mav_gcs_seen)
		return 0;
	return ((uint32_t)(mav_now_ms - mav_gcs_last_ms) < MAV_GCS_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t MAVLink_RxDropCount(void)
{
	return mav_rx_drop;
}

/************************************************************************
 * 接收解析
 ************************************************************************/
static void mav_rx_dispatch(void)
{
	switch(mav_rx.msgid)
	{
		case MAV_MSG_HEARTBEAT:
			mav_gcs_last_ms = mav_now_ms;
			mav_gcs_seen    = 1;
			break;

		case MAV_MSG_PARAM_REQUEST_LIST:
			MAVApp_HandleParamRequestList(mav_rx.payload);
			break;

		case MAV_MSG_PARAM_REQUEST_READ:
			MAVApp_HandleParamRequestRead(mav_rx.payload);
			break;

		case MAV_MSG_PARAM_SET:
			MAVApp_HandleParamSet(mav_rx.payload);
			break;

		case MAV_MSG_COMMAND_LONG:
			MAVApp_HandleCommandLong(mav_rx.payload);
			break;

		case MAV_MSG_REQUEST_DATA_STREAM:
		{
			/* payload: req_message_rate u16, target_system, target_component,
			            req_stream_id, start_stop */
			uint16_t rate    = MAV_GET_U16(&mav_rx.payload[0]);
			uint8_t  tgt_sys = mav_rx.payload[2];
			uint8_t  stream  = mav_rx.payload[4];
			uint8_t  on      = mav_rx.payload[5];

			if(tgt_sys != 0u && tgt_sys != MAV_SYSID)
				break;
			if(!on || rate == 0u)
				break;                     /* 该流不单独处理 */

			switch(stream)
			{
				case MAV_DATA_STREAM_RAW_SENSORS:
					(void)MAVLink_SetStreamInterval(MAV_MSG_RAW_IMU, (int32_t)1000000 / rate);
					break;
				case MAV_DATA_STREAM_RC_CHANNELS:
					(void)MAVLink_SetStreamInterval(MAV_MSG_RC_CHANNELS, (int32_t)1000000 / rate);
					break;
				case MAV_DATA_STREAM_EXTRA1:
					(void)MAVLink_SetStreamInterval(MAV_MSG_ATTITUDE, (int32_t)1000000 / rate);
					break;
				case MAV_DATA_STREAM_EXTRA2:
					(void)MAVLink_SetStreamInterval(MAV_MSG_VFR_HUD, (int32_t)1000000 / rate);
					break;
				case MAV_DATA_STREAM_EXTENDED_STATUS:
				case MAV_DATA_STREAM_EXTRA3:
					(void)MAVLink_SetStreamInterval(MAV_MSG_SYS_STATUS, (int32_t)1000000 / rate);
					break;
				default:
					break;
			}
			break;
		}

		case MAV_MSG_SET_MODE:
			/* 飞行模式由遥控器 AUX1 决定（Mode_Controler 每 4ms 会重写
			   Command.FlightMode），地面站直接改模式会被立刻覆盖，
			   所以这里只提示、不生效，避免给出错误反馈。 */
			MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "Mode is set by RC AUX1");
			break;

		default:
			break;
	}
}

static void mav_rx_byte(uint8_t b)
{
	/* 帧头阶段又见到 STX 就重新同步：噪声里一个假 0xFE 带个大长度会把
	   后面几十字节全当载荷吞掉，导致真帧被漏掉。载荷阶段不能这么做
	   （载荷里本来就可能有 0xFE）。*/
	if(b == MAV_STX_V1 && mav_rx.state >= 2u && mav_rx.state <= 5u)
	{
		mav_rx.state = 1;
		mav_rx.crc   = 0xFFFFu;
		return;
	}

	switch(mav_rx.state)
	{
		case 0:
			if(b == MAV_STX_V1)
			{
				mav_rx.state = 1;
				mav_rx.crc   = 0xFFFFu;
			}
			break;

		case 1:                                   /* len */
			if(b > MAV_MAX_PAYLOAD)
			{
				mav_rx.state = 0;
				mav_rx_drop++;
				break;
			}
			mav_rx.len   = b;
			mav_rx.idx   = 0;
			mav_rx.crc   = mav_crc_accum(mav_rx.crc, b);
			mav_rx.state = 2;
			break;

		case 2:                                   /* seq（不校验） */
			mav_rx.crc   = mav_crc_accum(mav_rx.crc, b);
			mav_rx.state = 3;
			break;

		case 3:                                   /* sysid */
			mav_rx.sysid = b;
			mav_rx.crc   = mav_crc_accum(mav_rx.crc, b);
			mav_rx.state = 4;
			break;

		case 4:                                   /* compid */
			mav_rx.compid = b;
			mav_rx.crc    = mav_crc_accum(mav_rx.crc, b);
			mav_rx.state  = 5;
			break;

		case 5:                                   /* msgid */
			mav_rx.msgid = b;
			mav_rx.crc   = mav_crc_accum(mav_rx.crc, b);
			mav_rx.state = (mav_rx.len == 0u) ? 7u : 6u;
			break;

		case 6:                                   /* payload */
			mav_rx.payload[mav_rx.idx++] = b;
			mav_rx.crc = mav_crc_accum(mav_rx.crc, b);
			if(mav_rx.idx >= mav_rx.len)
				mav_rx.state = 7;
			break;

		case 7:
			mav_rx.crc_lo = b;
			mav_rx.state  = 8;
			break;

		case 8:
			mav_rx.crc_hi = b;
			mav_rx.state  = 0;
			{
				uint16_t rx_crc = (uint16_t)((uint16_t)mav_rx.crc_lo | ((uint16_t)mav_rx.crc_hi << 8));
				uint16_t calc   = mav_crc_accum(mav_rx.crc, mav_crc_extra(mav_rx.msgid));

				if(calc == rx_crc)
					mav_rx_dispatch();
				else
					mav_rx_drop++;
			}
			break;

		default:
			mav_rx.state = 0;
			break;
	}
}

/************************************************************************
 * 周期调用（4ms 通信任务）
 ************************************************************************/
void MAVLink_Update(uint32_t now_ms)
{
	uint8_t  i;
	uint8_t  b;
	uint8_t  budget = 48u;                       /* 单次最多解析 48 字节：4ms 内线上最多来 23 字节，留 2 倍余量 */

	mav_now_ms = now_ms;

	/* 半帧超时：57600 下最长帧(72B)只需 12.5ms，超过 50ms 没有新字节
	   说明这半帧是噪声或上一个残帧，直接丢弃重新同步 */
	if(mav_rx.state != 0u && (uint32_t)(now_ms - mav_rx_byte_ms) > MAV_RX_IDLE_TIMEOUT_MS)
	{
		mav_rx.state = 0;
		mav_rx_drop++;
	}

	while(budget-- > 0u)
	{
		if(!UART1_RxPop(&b))
			break;

		mav_rx_byte_ms = now_ms;
		mav_rx_byte(b);
	}

	MAVApp_ParamListTick();

	for(i = 0; i < (uint8_t)MAV_STREAM_COUNT; i++)
	{
		mav_stream_t *s = &mav_streams[i];

		if(!s->enabled || s->period_ms == 0u)
			continue;
		if((uint32_t)(now_ms - s->last_ms) < (uint32_t)s->period_ms)
			continue;

		s->last_ms = now_ms;
		mav_send_by_id(s->msgid);
	}

#if MAVLINK_BOOT_ASCII
	/* 每秒一行 ASCII 存活节拍（仅调试期，MAVLINK_BOOT_ASCII=0 可关） */
	if((uint32_t)(now_ms - mav_tick_last_ms) >= 1000u)
	{
		mav_tick_last_ms = now_ms;
		mav_send_tick(now_ms);
	}
#endif
}

void MAVLink_Init(void)
{
	memset(&mav_rx, 0, sizeof(mav_rx));
	mav_tx_seq     = 0;
	mav_rx_drop    = 0;
	mav_now_ms     = 0;
	mav_rx_byte_ms = 0;
	mav_gcs_last_ms = 0;
	mav_gcs_seen   = 0;

	{
		uint8_t i;
		for(i = 0; i < (uint8_t)MAV_STREAM_COUNT; i++)
			mav_streams[i].last_ms = 0;
	}

	MAVApp_Init();

	/* 开机横幅已由 MAVLink_EarlyInit() 在 main() 最前面发过，这里不再重复 */
	MAVLink_SendStatustext(MAV_SEVERITY_INFO, "MAVLink v1 ready");
}
