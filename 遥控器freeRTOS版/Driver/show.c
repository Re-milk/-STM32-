#include "show.h"
#include "oledfont.h"
#include "oled.h"

//标题栏X轴起始位置
#define Line1_Begin 29
//第二行X轴起始位置
#define Line2_Begin 5
//第三行X轴起始位置(摇杆标签)
#define Line3_Begin 5
//第四行X轴起始位置(进度条)
#define Line4_Begin 30
//第五行X轴起始位置(微调显示)
#define Line5_Begin 2

//姿态角显示X轴起始位置(旧版布局,已弃用)
#define X_Begin 0
#define Y_Begin 51
#define Z_Begin 103

//备用布局参数(未使用)
#define Line1_Begin1 0
#define Line2_Begin1 0
#define Line3_Begin1 40
#define Line4_Begin1 0
#define Line5_Begin1 0

//各行Y轴起始坐标
#define Y0 0      //标题栏行
#define Y1 14     //电压显示行
#define Y2 Y1+12  //摇杆数据第一行(THR/ROL)
#define Y3 Y2+12  //摇杆数据第二行(YAW/PIT)
#define Y4 Y3+12  //底部行(姿态角/微调)
#define Y5 Y4+12  //预留行

struct _Show Show;

unsigned char i;          //通用循环变量
unsigned char Send_Count; //待发送的数据个数
float Vol;

/**************************************************************************
函数功能：OLED主页面显示
入口参数：无
返回  值：无
说明    ：显示标题栏、电压、摇杆数据、姿态角或微调值
**************************************************************************/
void oled_show(void)
{
	u8 temp;
	int temp1;

	static u8 page,page_temp,flash_cnt,show_mode=1;

	//页面切换时清屏
	if(page != page_temp)
	{
		page_temp = page;
		OLED_Clear();
	}
	///////////////////////////////第一行：标题栏///////////////////////////////////
	//显示警告提示
	if(Show.low_power)//遥控器低电量警告
	{
		flash_cnt ++;
		if(flash_cnt>3)
		{
			flash_cnt=0;
			if(show_mode==0)show_mode=1;
			else show_mode = 0;
		}

		for(u8 i=0;i<12;i++) OLED_Show_CH_String(Line1_Begin+i*6,Y0,oled_CH16[i],12,show_mode);
	}
	else if(Show.test_flag&BIT5)//飞机低电量警告
	{
		flash_cnt ++;
		if(flash_cnt>3)
		{
			flash_cnt=0;
			if(show_mode==0)show_mode=1;
			else show_mode = 0;
		}

		for(u8 i=0;i<12;i++) OLED_Show_CH_String(Line1_Begin+i*6,Y0,oled_CH17[i],12,show_mode);
	}
	else
	{
		//正常状态：显示"飞控板主页面"(6个中文字符)
		OLED_Show_CH(Line1_Begin+00,Y0,0,12,1);
		OLED_Show_CH(Line1_Begin+12,Y0,1,12,1);
		OLED_Show_CH(Line1_Begin+24,Y0,2,12,1);
		OLED_Show_CH(Line1_Begin+36,Y0,3,12,1);
		OLED_Show_CH(Line1_Begin+48,Y0,4,12,1);
		OLED_Show_CH(Line1_Begin+60,Y0,5,12,1);
	}
	//////////////////////////////////////////////////////
	//NRF模块异常时显示X，否则显示NRF频道号
	if(Show.NRF_Err)	OLED_Show_CH(2,Y0,6 ,12,1);
	else				OLED_ShowNumber(2,Y0,ANO_Param.NRF_Channel,3,12);

	//显示信号强度图标(根据帧率分6档)
	temp = Show.Rc_num/20;
	switch(temp)
	{
		case 0:OLED_Show_CH(Line1_Begin+85,Y0,6 ,12,1);break;
		case 1:OLED_Show_CH(Line1_Begin+85,Y0,7 ,12,1);break;
		case 2:OLED_Show_CH(Line1_Begin+85,Y0,8 ,12,1);break;
		case 3:OLED_Show_CH(Line1_Begin+85,Y0,9 ,12,1);break;
		case 4:OLED_Show_CH(Line1_Begin+85,Y0,10,12,1);break;
		default:OLED_Show_CH(Line1_Begin+85,Y0,11,12,1);break;
	}
	///////////////////////////////第二行：电压显示///////////////////////////////////
	//显示遥控器电压 (格式: R-V: X.XX)
	OLED_ShowString(Line2_Begin+00,Y1,"R-V:",12,1);
	OLED_ShowString(Line2_Begin+36,Y1,".",12,1);
	OLED_ShowNumber(Line2_Begin+30,Y1,Show.Battery_Rc/100,1,12);
	OLED_ShowNumber(Line2_Begin+42,Y1,Show.Battery_Rc%100,2,12);
	if(Show.Battery_Rc%100<10)	OLED_ShowNumber(Line2_Begin+42,Y1,0,1,12);

	//显示飞机电压 (格式: F-V: XX.XX)
	OLED_ShowString(Line2_Begin+64,Y1,"F-V:",12,1);
	OLED_ShowString(Line2_Begin+106,Y1,".",12,1);
	OLED_ShowNumber(Line2_Begin+94,Y1,Show.Battery_Fly/100,2,12);
	if(Show.Battery_Fly%100>10)
	{OLED_ShowNumber(Line2_Begin+112,Y1,Show.Battery_Fly%100,2,12);}
	else if(Show.Battery_Fly%100<10)
	{OLED_ShowNumber(Line2_Begin+112,Y1,0,1,12);
	OLED_ShowNumber(Line2_Begin+124,Y1,Show.Battery_Fly%100,1,12);}
	else
	{OLED_ShowNumber(Line2_Begin+112,Y1,1,1,12);
	 OLED_ShowNumber(Line2_Begin+124,Y1,0,1,12);
	}

	///////////////////////////////第三、四行：摇杆数据///////////////////////////////////
	//显示摇杆通道数据及进度条
	//THR(油门)
	OLED_ShowString(Line3_Begin+00,Y2,"THR:",12,1);
	temp = (Rc.THR-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y2,12,1);
	temp = (Rc.THR-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y2,12,1);

	//ROL(横滚)
	OLED_ShowString(Line3_Begin+64,Y2,"ROL:",12,1);
	temp = (Rc.ROL-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y2,12,1);
	temp = (Rc.ROL-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y2,12,1);

	//YAW(航向)
	OLED_ShowString(Line3_Begin+00,Y3,"YAW:",12,1);
	temp = (Rc.YAW-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y3,12,1);
	temp = (Rc.YAW-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y3,12,1);

	//PIT(俯仰)
	OLED_ShowString(Line3_Begin+64,Y3,"PIT:",12,1);
	temp = (Rc.PIT-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y3,12,1);
	temp = (Rc.PIT-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y3,12,1);
	///////////////////////////////第五行：姿态角或微调///////////////////////////////////
	if(Show.Rc_num && Show.hardware_type == 1)
	{
		page = 1;//第一页：显示姿态角和高度

		//显示X轴姿态角(单位:度,精度0.1)
		if(Show.X<0) temp1 = -Show.X,OLED_ShowString(0,Y4,"-",12,1);
		else         temp1 = Show.X, OLED_ShowString(0,Y4,"+",12,1);
		OLED_ShowNumber(4,Y4,temp1/10,3,12);
		OLED_ShowString(24,Y4,".",12,1);
		OLED_ShowNumber(28,Y4,temp1%10,1,12);

		//显示Y轴姿态角(单位:度,精度0.1)
		if(Show.Y<0) temp1 = -Show.Y,OLED_ShowString(38,Y4,"-",12,1);
		else    temp1 = Show.Y, OLED_ShowString(38,Y4,"+",12,1);
		OLED_ShowNumber(42,Y4,temp1/10,3,12);
		OLED_ShowString(62,Y4,".",12,1);
		OLED_ShowNumber(66,Y4,temp1%10,1,12);

		//显示高度(单位:米,精度0.01,最大显示999.99m)
		if(Show.H<0) 			temp1 =-Show.H,  OLED_ShowString(74,Y4,"-",12,1);
		else    			temp1 = Show.H,  OLED_ShowString(74,Y4,"+",12,1);
		OLED_ShowNumber(80,Y4,temp1/100,3,12);
		OLED_ShowString(100,Y4,".",12,1);

	  if(temp1%100<10)   {OLED_ShowNumber(104,Y4,0,1,12);OLED_ShowNumber(110,Y4,temp1%100,1,12);	}
		else {OLED_ShowNumber(104,Y4,temp1%100,2,12);	}
		OLED_ShowString(120,Y4,"m",12,1);
	}
	else
	{
		page = 2;//第二页：显示微调按钮数据
		//显示横滚微调值
		OLED_ShowString(Line5_Begin+00,Y4,"R:",12,1);
		temp = (ANO_Param.OffSet_Rol-1000)/41;
		OLED_Show_progress_bar(temp,12,24,Line5_Begin+15 ,Y4,12,1);
		temp = (ANO_Param.OffSet_Rol-1500)/41;
		OLED_Show_progress_bar(temp,12,12,Line5_Begin+27,Y4,12,1);

		//显示航向微调值
		OLED_ShowString(Line5_Begin+44,Y4,"Y:",12,1);
		temp = (ANO_Param.OffSet_Yaw-1000)/41;
		OLED_Show_progress_bar(temp,12,24,Line5_Begin+59 ,Y4,12,1);
		temp = (ANO_Param.OffSet_Yaw-1500)/41;
		OLED_Show_progress_bar(temp,12,12,Line5_Begin+71,Y4,12,1);

		//显示俯仰微调值
		OLED_ShowString(Line5_Begin+88,Y4,"P:",12,1);
		temp = (ANO_Param.OffSet_Pit-1000)/41;
		OLED_Show_progress_bar(temp,12,24,Line5_Begin+103 ,Y4,12,1);
		temp = (ANO_Param.OffSet_Pit-1500)/41;
		OLED_Show_progress_bar(temp,12,12,Line5_Begin+115,Y4,12,1);
	}

	OLED_Refresh_Gram();//刷新OLED显存到屏幕
}

//显示进度条函数
//temp: 当前值(0~12)
//chr_star: 起始字符索引(进度条左半部分)
//chr_default: 默认字符索引(超出范围时显示)
//x,y: 显示坐标
//size: 字符大小
//mode: 显示模式
void OLED_Show_progress_bar(u8 temp,u8 chr_star,u8 chr_default,u8 x,u8 y,u8 size,u8 mode)
{
	switch(temp)
	{
		case  0:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  1:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  2:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  3:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  4:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  5:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  6:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  7:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  8:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  9:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 10:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 11:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 12:OLED_Show_CH(x,y,chr_star+temp,size,size);break;

		default:OLED_Show_CH(x,y,chr_default,size,size);break;
	}
}

//设置页面相关变量
vs16 set_bit=0,bit_max=5;   //当前选中的设置位,最大设置位数
u16 set_temp=0x0000;         //设置参数临时缓存

//设置页面显示函数
void OLED_Show_Seting(void)
{
	u8 cnt=0,bit_cnt=0;
	u8 mode,page;
	static u8 page_temp;

	//显示标题"飞控设置"
	for(u8 i=0;i<8;i++) OLED_Show_CH_String(40+i*6,cnt,oled_CH0[i],12,1);

	//计算当前页码和位偏移
	page = set_bit/4;
	bit_cnt = 4*page;
	if(page_temp!=page)
	{
		page_temp=page;
		OLED_Clear();
	}

	switch(page)
	{
		case 0://第0页：显示前4个设置项
			cnt+=13;
			//设置项1：自稳模式
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH1[i],12,1);
			if(set_bit==bit_cnt) mode=0;	else mode=1;
			if( set_temp&BIT0 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			//设置项2：半自稳模式
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH2[i],12,1);
			if(set_bit==bit_cnt) mode=0;	else mode=1;
			if( set_temp&BIT1 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			//设置项3：无头模式
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH3[i],12,1);
			if(set_bit==bit_cnt) mode=0; else mode=1;
			if( set_temp&BIT2 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			//设置项4：切换箭头
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH4[i],12,1);
			if(set_bit==bit_cnt)	mode=0; else mode=1;
			if( set_temp&BIT3 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;
		break;

		case 1://第1页：显示后2个设置项
			cnt+=13;
			//设置项5：加锁定位
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH14[i],12,1);
			if(set_bit==bit_cnt)	mode=0; else mode=1;
			if( set_temp&BIT4 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			//设置项6：失控保护
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH18[i],12,1);
			if(set_bit==bit_cnt)	mode=0; else mode=1;
			if( set_temp&BIT5 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

		break;

		default:break;
	}

	OLED_Refresh_Gram();//刷新OLED显存到屏幕
}

//测试页面相关变量
vs16 test_bit=0,test_max=5;  //当前选中的测试项,最大测试项数
u16  test_temp=0;             //传感器状态缓存

//测试页面显示函数
void OLED_Show_Test(void)
{
	u8 cnt=0,bit_cnt=0;
	u8 mode,page;
	static u8 page_temp;

	test_temp = Show.test_flag;

	//显示标题"传感器状态"
	for(u8 i=0;i<10;i++) OLED_Show_CH_String(34+i*6,cnt,oled_CH5[i],12,1);

	//计算当前页码和位偏移
	page = test_bit/4;
	bit_cnt = 4*page;
	if(page_temp!=page)
	{
		page_temp=page;
		OLED_Clear();
	}

	switch(page)
	{
		case 0://第0页：显示前4个传感器
			cnt+=13;
			//MPU6050状态
			for(u8 i=0;i<8;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH6[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT0 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);

			cnt+=13;
			//气压计状态
			for(u8 i=0;i<8;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH7[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT1 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);

			cnt+=13;
			//遥控模块状态
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH8[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT2 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);

			cnt+=13;
			//GPS/光流模块状态
			for(u8 i=0;i<12;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH9[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT3 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
		break;

		case 1://第1页：显示后2个传感器
			cnt+=13;
			//加速度计/罗盘状态
			for(u8 i=0;i<14;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH15[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT4 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);

			cnt+=13;
			//超声波模块状态
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH19[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT6 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
		break;

		default:break;
	}

	OLED_Refresh_Gram();//刷新OLED显存到屏幕
}

//遥控器手势检测0：摇杆上下移动选择菜单项
//val: 摇杆值(1000~2000)
//set: 当前选中项指针
//max: 最大选项数
void Gesture_Check0(u16 val,vs16 *set,vs16 max)
{
	static u8 cnt0,cnt1;

	//摇杆向上推(>1800)，计数
	if(val>1800)
	{
		cnt0++;
	}
	else
	{
		if(cnt0>2) (*set)--;  //连续3次检测到才执行，防抖
		cnt0 = 0;
	}

	//摇杆向下推(<1200)，计数
	if(val<1200)
	{
		cnt1++;
	}
	else
	{
		if(cnt1>2) (*set)++;  //连续3次检测到才执行，防抖
		cnt1 = 0;
	}

	//循环选择(到头则跳到尾)
	if((*set)<0) 				(*set) = max;
	else if((*set)>max) (*set) = 0;
}

//遥控器手势检测1：摇杆左右切换设置开关
//val: 摇杆值(1000~2000)
//set: 设置参数指针
//bit: 要切换的位
void Gesture_Check1(u16 val,u16 *set,vs16 bit)
{
	static u8 cnt0;

	//摇杆左推或右推
	if(val>1800 || val<1200)
	{
		cnt0++;
	}
	else
	{
		if(cnt0>2)
			*set = REVERSE(*set,bit);  //翻转指定位
		cnt0 = 0;
	}
}

u8 send_flag=0;  //设置数据发送标志

//手势检测总入口(由调度器调用)
void Gesture_Check(void)
{
	static u8 temp;

	//页面切换时同步设置数据
	if(temp!=Show.windows)
	{
		if(Show.windows==1) set_temp = Show.set_flag;
		if(temp==1 && set_temp!=Show.set_flag) send_flag = 1;  //设置被修改，标记发送
		temp=Show.windows;
	}

	switch(Show.windows)
	{
		case 1://设置页面：PIT选择项,ROL切换开关
			Gesture_Check0(Rc.PIT,&set_bit,bit_max);
			if(Show.Connect_Succeed)
			{
				Gesture_Check1(Rc.ROL,&set_temp,set_bit);
			}
		break;

		case 2://测试页面：PIT选择项
			Gesture_Check0(Rc.PIT,&test_bit,test_max);
		break;

		default:break;
	}
}

//OLED显示总调度函数(由20Hz任务调用)
void Show_Duty(void)
{
	static u8 temp;

	//页面切换时清屏
	if(Show.windows!=temp)
	{
		temp = Show.windows;
		OLED_Clear();
	}

	switch(Show.windows)
	{
		case 0:	oled_show();	break;       //主页面
		case 1:	OLED_Show_Seting();	break;   //设置页面
		case 2:	OLED_Show_Test();	break;     //测试页面
		default:break;
	}
}
