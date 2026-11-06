//========================================================================
//	MAVLink 应用层：参数表(PARAM_*) + 命令处理(COMMAND_LONG)
//
//	PARAM_* 完全等价于原匿名上位机的 PID 读写（0x10~0x15），
//	但改成了名字索引，QGC / Mission Planner 的参数编辑器可以直接用。
//
//	注意：参数只存在 RAM 里，复位后由 pid_param_Init() 恢复默认值，
//	与原匿名上位机的行为一致（工程未编译 eeprom.c，没有持久化）。
//========================================================================

#include <string.h>
#include "mavlink_port.h"
#include "ALL_DATA.h"

/* ---------------- 外部数据 ---------------- */
extern _st_ALL_flag ALL_flag;
extern _st_Remote   Remote;

extern PidObject pidRateX,  pidRateY,  pidRateZ;
extern PidObject pidRoll,   pidPitch,  pidYaw;
extern PidObject pidHeightRate, pidHeightHigh;
extern PidObject Flow_PosPid_x, Flow_PosPid_y;
extern PidObject Flow_SpeedPid_x, Flow_SpeedPid_y;

extern unsigned char baro_start;      /* spl06.c：置 1 请求气压零点 */
extern void MpuGetOffset(void);       /* mpu6050.c：加速度/角速度零偏校准 */

/************************************************************************
 * 参数表（const，占 Flash，每项 12 字节）
 ************************************************************************/
typedef struct
{
	const char     *name;
	volatile float *ptr;
	uint8_t         type;
} mav_param_t;

static const mav_param_t mav_params[] =
{
	{ "RATE_X_KP",     &pidRateX.kp, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_X_KI",     &pidRateX.ki, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_X_KD",     &pidRateX.kd, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Y_KP",     &pidRateY.kp, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Y_KI",     &pidRateY.ki, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Y_KD",     &pidRateY.kd, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Z_KP",     &pidRateZ.kp, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Z_KI",     &pidRateZ.ki, MAV_PARAM_TYPE_REAL32 },
	{ "RATE_Z_KD",     &pidRateZ.kd, MAV_PARAM_TYPE_REAL32 },

	{ "ANGLE_ROLL_KP", &pidRoll.kp,  MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_ROLL_KI", &pidRoll.ki,  MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_ROLL_KD", &pidRoll.kd,  MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_PIT_KP",  &pidPitch.kp, MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_PIT_KI",  &pidPitch.ki, MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_PIT_KD",  &pidPitch.kd, MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_YAW_KP",  &pidYaw.kp,   MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_YAW_KI",  &pidYaw.ki,   MAV_PARAM_TYPE_REAL32 },
	{ "ANGLE_YAW_KD",  &pidYaw.kd,   MAV_PARAM_TYPE_REAL32 },

	{ "ALT_RATE_KP",   &pidHeightRate.kp, MAV_PARAM_TYPE_REAL32 },
	{ "ALT_RATE_KI",   &pidHeightRate.ki, MAV_PARAM_TYPE_REAL32 },
	{ "ALT_RATE_KD",   &pidHeightRate.kd, MAV_PARAM_TYPE_REAL32 },
	{ "ALT_HOLD_KP",   &pidHeightHigh.kp, MAV_PARAM_TYPE_REAL32 },
	{ "ALT_HOLD_KI",   &pidHeightHigh.ki, MAV_PARAM_TYPE_REAL32 },
	{ "ALT_HOLD_KD",   &pidHeightHigh.kd, MAV_PARAM_TYPE_REAL32 },

	{ "FLOW_SPD_X_KP", &Flow_SpeedPid_x.kp, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_SPD_X_KI", &Flow_SpeedPid_x.ki, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_SPD_X_KD", &Flow_SpeedPid_x.kd, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_SPD_Y_KP", &Flow_SpeedPid_y.kp, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_SPD_Y_KI", &Flow_SpeedPid_y.ki, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_SPD_Y_KD", &Flow_SpeedPid_y.kd, MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_X_KP", &Flow_PosPid_x.kp,   MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_X_KI", &Flow_PosPid_x.ki,   MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_X_KD", &Flow_PosPid_x.kd,   MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_Y_KP", &Flow_PosPid_y.kp,   MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_Y_KI", &Flow_PosPid_y.ki,   MAV_PARAM_TYPE_REAL32 },
	{ "FLOW_POS_Y_KD", &Flow_PosPid_y.kd,   MAV_PARAM_TYPE_REAL32 }
};

#define MAV_PARAM_COUNT  ((uint8_t)(sizeof(mav_params) / sizeof(mav_params[0])))

/* 参数列表传输状态：每次 MAVLink_Update() 只发一条，避免刷爆串口 */
static uint8_t mav_list_active = 0;
static uint8_t mav_list_idx    = 0;

/************************************************************************
 * 小工具
 ************************************************************************/
static void mav_send_ack(uint16_t cmd, uint8_t result)
{
	uint8_t p[3];

	MAV_PUT_U16(&p[0], cmd);
	p[2] = result;
	(void)MAVLink_SendMessage(MAV_MSG_COMMAND_ACK, p, 3);
}

static uint8_t mav_param_find(const char *name)
{
	uint8_t i;

	for(i = 0; i < MAV_PARAM_COUNT; i++)
	{
		if(strcmp(name, mav_params[i].name) == 0)
			return i;
	}
	return MAV_PARAM_COUNT;      /* 未找到 */
}

/* PARAM_VALUE 载荷 25 字节：
 * param_value[0..3] param_count[4..5] param_index[6..7] param_id[8..23] param_type[24] */
static uint8_t mav_send_param_value(uint8_t idx)
{
	uint8_t p[25];
	const char *name;
	uint8_t i;

	if(idx >= MAV_PARAM_COUNT)
		return 0;

	name = mav_params[idx].name;
	memset(p, 0, sizeof(p));
	MAV_PutF32(&p[0], *mav_params[idx].ptr);
	MAV_PUT_U16(&p[4], MAV_PARAM_COUNT);
	MAV_PUT_U16(&p[6], (uint16_t)idx);
	for(i = 0; i < 16u && name[i] != '\0'; i++)
		p[8u + i] = (uint8_t)name[i];
	p[24] = mav_params[idx].type;

	return MAVLink_SendMessage(MAV_MSG_PARAM_VALUE, p, 25);
}

uint8_t MAVApp_ParamCount(void)
{
	return MAV_PARAM_COUNT;
}

/************************************************************************
 * PARAM_REQUEST_LIST  ->  逐条回 PARAM_VALUE
 ************************************************************************/
void MAVApp_HandleParamRequestList(const uint8_t *payload)
{
	if(payload[0] != 0u && payload[0] != MAV_SYSID)
		return;

	mav_list_idx    = 0;
	mav_list_active = 1;
}

void MAVApp_ParamListTick(void)
{
	if(!mav_list_active)
		return;

	if(mav_list_idx >= MAV_PARAM_COUNT)
	{
		mav_list_active = 0;
		return;
	}

	if(!mav_send_param_value(mav_list_idx))
		return;                  /* 发送缓冲满，下个周期重试 */

	mav_list_idx++;
	if(mav_list_idx >= MAV_PARAM_COUNT)
		mav_list_active = 0;
}

/************************************************************************
 * PARAM_REQUEST_READ
 * 载荷 20 字节：param_index[0..1] target_system[2] target_component[3] param_id[4..19]
 ************************************************************************/
void MAVApp_HandleParamRequestRead(const uint8_t *payload)
{
	int16_t     index = MAV_GET_I16(&payload[0]);
	char        name[17];
	uint8_t     i;

	if(payload[2] != 0u && payload[2] != MAV_SYSID)
		return;

	memcpy(name, &payload[4], 16);
	name[16] = '\0';

	if(name[0] == '\0')
	{
		if(index >= 0 && index < (int16_t)MAV_PARAM_COUNT)
			(void)mav_send_param_value((uint8_t)index);
		return;
	}

	i = mav_param_find(name);
	if(i < MAV_PARAM_COUNT)
	{
		(void)mav_send_param_value(i);
	}
	else
	{
		MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "PARAM_REQUEST_READ: unknown");
	}
}

/************************************************************************
 * PARAM_SET
 * 载荷 23 字节：param_value[0..3] target_system[4] target_component[5]
 *               param_id[6..21] param_type[22]
 ************************************************************************/
void MAVApp_HandleParamSet(const uint8_t *payload)
{
	float   value;
	char    name[17];
	uint8_t i;

	if(payload[4] != 0u && payload[4] != MAV_SYSID)
		return;
	if(payload[5] != 0u && payload[5] != MAV_COMPID)
		return;

	value = MAV_GetF32(&payload[0]);
	memcpy(name, &payload[6], 16);
	name[16] = '\0';

	i = mav_param_find(name);
	if(i >= MAV_PARAM_COUNT)
	{
		MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "PARAM_SET: unknown name");
		return;
	}

	if(value != value)                                   /* NaN */
	{
		MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "PARAM_SET: NaN rejected");
		return;
	}
	if(value >  10000.0f) value =  10000.0f;
	if(value < -10000.0f) value = -10000.0f;

	*mav_params[i].ptr = value;
	(void)mav_send_param_value(i);                       /* 按 MAVLink 规范回读 */
}

/************************************************************************
 * COMMAND_LONG 处理
 * 载荷 33 字节：param1..7[0..27] command[28..29] target_system[30]
 *               target_component[31] confirmation[32]
 ************************************************************************/
static void mav_cmd_arm_disarm(float param1)
{
	if(param1 != 0.0f)
	{
		if(ALL_flag.unlock)
		{
			mav_send_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
			return;
		}
		if(Remote.thr > 1100u)
		{
			mav_send_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_DENIED);
			MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "ARM denied: throttle not low");
			return;
		}

		baro_start      = 1;        /* 与 remote_unlock() 的解锁流程保持一致 */
		ALL_flag.unlock = 1;
		mav_send_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
		MAVLink_SendStatustext(MAV_SEVERITY_INFO, "ARMED by GCS");
	}
	else
	{
		ALL_flag.unlock = 0;
		mav_send_ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
		MAVLink_SendStatustext(MAV_SEVERITY_INFO, "DISARMED by GCS");
	}
}

static void mav_cmd_calibration(float gyro, float baro, float accel)
{
	if(ALL_flag.unlock)
	{
		mav_send_ack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_DENIED);
		MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "Calibration needs DISARMED");
		return;
	}

	/* 气压清零只是置个标志，由 spl06 自己的任务去做，安全，始终允许 */
	if(baro >= 1.0f && gyro < 1.0f && accel < 1.0f)
	{
		baro_start = 1;
		mav_send_ack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED);
		MAVLink_SendStatustext(MAV_SEVERITY_INFO, "Baro zero requested");
		return;
	}

#if MAVLINK_ALLOW_CALIBRATION
	if(gyro >= 1.0f || accel >= 1.0f)
	{
		mav_send_ack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED);
		MAVLink_SendStatustext(MAV_SEVERITY_INFO, "Calibrating, KEEP STILL");
		/* 注意：MpuGetOffset() 内部自己调 MpuGetData()，而 Task_FlightControl
		   每 2ms 也在调 MpuGetData()，两者共用同一条软件 I2C(PB6/PB7)。
		   这里没有总线互斥，并发会让两边的读数都出错，所以默认关闭。 */
		MpuGetOffset();
		MAVLink_SendStatustext(MAV_SEVERITY_INFO, "Calibration done");
	}
	else
#else
	/* 默认：不接受地面站触发加速度/角速度校准。
	   软件 I2C 总线没有互斥、且 MpuGetOffset() 会与 2ms 控制任务抢总线，
	   并发时可能算出错误的零偏。请沿用原有方式（上电自动校准 / 遥控器流程）。*/
	if(gyro >= 1.0f || accel >= 1.0f)
	{
		mav_send_ack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_UNSUPPORTED);
		MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "Accel/gyro calib via GCS disabled");
	}
	else
#endif
	{
		mav_send_ack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_UNSUPPORTED);
	}
}

void MAVApp_HandleCommandLong(const uint8_t *payload)
{
	float    p1 = MAV_GetF32(&payload[0]);
	float    p2 = MAV_GetF32(&payload[4]);
	float    p3 = MAV_GetF32(&payload[8]);
	float    p5 = MAV_GetF32(&payload[16]);
	uint16_t cmd = MAV_GET_U16(&payload[28]);
	uint8_t  tgt_sys  = payload[30];
	uint8_t  tgt_comp = payload[31];

	if(tgt_sys  != 0u && tgt_sys  != MAV_SYSID)   return;
	if(tgt_comp != 0u && tgt_comp != MAV_COMPID)  return;

	switch(cmd)
	{
		case MAV_CMD_COMPONENT_ARM_DISARM:
			mav_cmd_arm_disarm(p1);
			break;

		case MAV_CMD_PREFLIGHT_CALIBRATION:
			/* param1=gyro param2=mag param3=ground pressure param4=RC param5=accel */
			mav_cmd_calibration(p1, p3, p5);
			break;

		case MAV_CMD_SET_MESSAGE_INTERVAL:
		{
			uint8_t msgid = (uint8_t)p1;
			int32_t us    = (int32_t)p2;

			if(MAVLink_SetStreamInterval(msgid, us) == 0)
				mav_send_ack(MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_ACCEPTED);
			else
				mav_send_ack(MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_UNSUPPORTED);
			break;
		}

		case MAV_CMD_REQUEST_MESSAGE:
			if(MAVLink_SendMessageOnce((uint8_t)p1) == 0)
				mav_send_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
			else
				mav_send_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED);
			break;

		case MAV_CMD_DO_SET_MODE:
			/* 飞行模式由遥控器 AUX1 决定，Mode_Controler() 每 4ms 会重写
			   Command.FlightMode，地面站改模式会被立刻覆盖，因此明确拒绝。 */
			mav_send_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_DENIED);
			MAVLink_SendStatustext(MAV_SEVERITY_WARNING, "Mode is set by RC AUX1");
			break;

		default:
			mav_send_ack(cmd, MAV_RESULT_UNSUPPORTED);
			break;
	}
}

void MAVApp_Init(void)
{
	mav_list_active = 0;
	mav_list_idx    = 0;
}
