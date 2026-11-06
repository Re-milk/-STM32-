//========================================================================
//	controlé©±åŠ¨
//========================================================================

#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "flow.h"
#include "spl06.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0

#define REMOTE_THR Remote.thr
#define REMOTE_PITCH Remote.pitch
#define REMOTE_ROLL Remote.roll
#define REMOTE_YAW Remote.yaw
//#define measured FeedBack
#define Expect desired 	

 float Throttle_out; //·É¿ØÓÍÃÅÊä³öÖµ //Ò£¿Ø + ¶¨¸ß Êä³öÖµ
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //½á¹¹ÌåÊı×é£¬½«Ã¿Ò»¸öÊı×é·ÅÒ»¸öpid½á¹¹Ìå£¬ÕâÑù¾Í¿ÉÒÔÅúÁ¿²Ù×÷¸÷¸öPIDµÄÊı¾İÁË  ±ÈÈç½âËøÊ±ÅúÁ¿¸´Î»pid¿ØÖÆÊı¾İ£¬ĞÂÊÖÃ÷°×Õâ¾ä»°µÄ×÷ÓÃ¾Í¿ÉÒÔÁË
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y
};
/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

//static uint8_t set_high_desired = 0; //¶¨¸ß¸ß¶ÈÒÑÉè¶¨

int16_t  HIGH_START =180;   //Ò»¼üÆğ·ÉÄ¿±ê¸ß¶È
uint8_t ALL_flag_height_lock;
uint32_t Control_high = 0; //µ±Ç°¸ß¶È
uint8_t flag_bara = 0;
/**************************************************************
 *  //¸ß¶È¿ØÖÆÆ÷     ÆøÑ¹
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)  //¸ß¶È¿ØÖÆÆ÷     ÆøÑ¹
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //µ±Ç°Äã·ÉĞĞÆ÷µÄ¼ÓËÙ¶ÈÖµ
   	int16_t acc_error; //µ±Ç°¼ÓËÙ¶È¼õÈ¥ÖØÁ¦¼ÓËÙ¶ÈÔòÎªÉÏÏÂÒÆ¶¯µÄ¼ÓËÙ¶È
   static int16_t acc_offset;//ÖØÁ¦¼ÓËÙ¶ÈÖµ	 
		static float thr_hold = 0; //½øÈë¸ß¶ÈÊ±¼ÇÂ¼µ±Ç°ÓÍÃÅÖµ
		static uint8_t set_high = 0,High_breaktime;
	
   if(Remote.thr<1200 && Remote.yaw>1800 && Remote.roll<1200 && Remote.pitch>1800  )  //Ë«Ò¡¸ËÓÒÏÂ              Í£ÓÃÆøÑ¹¸ß¶È
	 {flag_bara = 1;}
	 if(Remote.thr<1200 && Remote.yaw>1800 && Remote.roll<1200 && Remote.pitch<1200  )  //×óÒ¡¸ËÓÒÏÂÇÒÓÒÒ¡¸Ë×óÏÂ  ÆôÓÃÆøÑ¹¸ß¶È
	 {flag_bara = 0;}
	 if(flag_bara == 0)
   {
   	Control_high=FlightData.High.bara_height;      //ÆôÓÃÆøÑ¹¸ß¶È
   }
   else
   {
	 	  Control_high=0 ;	 //Í£ÓÃÆøÑ¹¸ß¶È
	 }

	//----------------------------------------------	
	{ //»ñÈ¡´¹Ö±ËÙ¶ÈÊı¾İ

		  acc = (int16_t)GetAccz(); //»ñÈ¡ZÖáACC  
		
			if(!ALL_flag.unlock)      //È¡µÃ¾²Ì¬ACCÖµ 
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//´Ë´¦×öÒ»¸öËÙ¶ÈÓë¸ß¶ÈµÄ»¥²¹ÂË²¨ 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //ËÙ¶È»·ÓÀÔ¶ÊÇÖ÷µ÷£¬ËùÒÔ»¥²¹ÂË²¨¹Ø¼ü¾Í×¥ÔÚËÙ¶È»·ÕâÀï
				last_high =  pidHeightHigh.measured = Control_high;  //ÊµÊ±¸ß¶È·´À¡¸øÍâ»·
			}	
	}
	//----------------------------------------------½ô¼±ÖÕÖ¹·ÉĞĞ
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;
	//----------------------------------------------¿ØÖÆ
	switch(status)
	{
		case WAITING_1: //¼ì²â¶¨¸ß
		  if(ALL_flag.height_lock && ALL_flag.unlock) 
			{
				LED.status = DANGEROURS;
				status = WAITING_2;
				thr_hold=0; 				        //Çå³ı¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ
			}
			break;
		case WAITING_2: //¶¨¸ßÇ°×¼±¸
			thr_hold = Remote.thr -1000;  //¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ
		  set_high = 0;
		  pidHeightHigh.desired = HIGH_START;  //ÆÚÍû¸ß¶È Éè¶¨Öµ
 			status = PROCESS_31;
			break; 
		
		case PROCESS_31://½øÈë¶¨¸ß	

				if(((Remote.thr -1000) > (thr_hold+10)) ||((Remote.thr -1000) < (thr_hold-10))) //²¦¶¯Ò£¸ËÖ»½øĞĞË®Æ½ËÙ¶È¿ØÖÆ
				{			 																			  
						if(High_breaktime++ >20)
						{
							thr_hold = Remote.thr -1000;  //¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ	
							High_breaktime = 0;
							set_high = 0;
						}														
				}
				else
				{}		 
				if(set_high == 0) //Èç¹û¸ÕÍË³öµ÷¸ß
				{
					set_high = 1;
					pidHeightHigh.desired = Control_high;//¼ÇÂ¼¸ß¶Èµ±×öµ±Ç°¶¨¸ß¸ß¶È		  
				}
				pidUpdate(&pidHeightHigh,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	¸©Ñö½ÇPID	
				pidHeightRate.desired = pidHeightHigh.out;  //¸ß¶È»·Êä³ö¸ß¶ÈËÙ¶ÈÉè¶¨Öµ
						 					 										 
				pidUpdate(&pidHeightRate,dt); //ÔÙµ÷ÓÃ¸ß¶ÈËÙ¶ÈÄÚ»·
					 
	//		  pidHeightRate.out += Remote.thr -1000;//¼ÓÈëĞüÍ£Ê±µÄÓÍÃÅ
					 
				if(!ALL_flag.height_lock)     //ÍË³ö¶¨¸ß
				{
					LED.status = AlwaysOn ;
					status = EXIT_255;
				}
			break;
		case EXIT_255: //ÍË³ö¶¨¸ß
			pidRest(&pPidObject[6],1);	//Çå³ıµ±Ç°µÄ¶¨¸ßÊä³öÖµ
			status = WAITING_1;//»Øµ½µÈ´ı½øÈë¶¨¸ß
			break;
		default:
			status = WAITING_1;
			break;	
	}	
				
}

void Mode_Controler(float dt)
{
		const float roll_pitch_ratio = 0.04f;
	
//		if(ALL_flag.unlock == 1)  //ÅĞ¶Ï½âËø
		{
			if(Remote.AUX1 < 1700)   //Èç¹û´óÓÚ1700 Ôò½øÈëÆøÑ¹¶¨¸ßÄ£Ê½¾
			{
				Command.FlightMode = HEIGHT;
				ALL_flag.height_lock = 1;
				Flow_mode_two();       // Ò£¿Ø-¹âÁ÷¿ØÖÆ×Ë
				set_flag=0x21;         // OLED¶¨¸ß¶¨µãÄ£Ê½ÏÔÊ¾
			}
			else                     //×ËÌ¬Ä£Ê½
			{  
				Command.FlightMode = NORMOL;	
				ALL_flag.height_lock = 1;		
				set_flag=0x00;        //OLED×ËÌ¬Ä£Ê½ÏÔÊ¾
				pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;  //Ò¡¸Ë¿ØÖÆ
				pidRoll.desired  = (Remote.roll-1500) *roll_pitch_ratio;  //Ò¡¸Ë¿ØÖÆ
			}
		}					
}

u32 altHold_Pos_Save = 0,Pos_breaktime = 0,Pos_break_save = 0;
float Flow_SpeedPid_x_out,Flow_SpeedPid_y_out;
///////////////////////// Ò£¿Ø-¹âÁ÷¿ØÖÆ×ËÌ¬////////////////////////////////////////////////
void Flow_mode_two(void)                               //¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª4ºÁÃë
{

			pidPitch.desired = -(Remote.pitch-1500)*0.04f; 
			pidRoll.desired  = (Remote.roll-1500)*0.04f;  

}

/**************************************************************
 * //Î»ÖÃ¶¨µã¿ØÖÆÆ÷  ¹âÁ÷
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

/**************************************************************
 * ×ËÌ¬¿ØÖÆ
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //µÈ´ı½âËø
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //×¼±¸½øÈë¿ØÖÆ
			pidRest(pPidObject,8); //ÅúÁ¿¸´Î»PIDÊı¾İ£¬·ÀÖ¹ÉÏ´ÎÒÅÁôµÄÊı¾İÓ°Ïì±¾´Î¿ØÖÆ

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //Ëø¶¨Æ«º½½Ç
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //ÕıÊ½½øÈë¿ØÖÆ
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
		
			pidPitch.measured = Angle.pitch; 		//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
		  pidRoll.measured = Angle.roll;			//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
			pidYaw.measured = Angle.yaw;				//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
		
		 	pidUpdate(&pidRoll,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	ºá¹ö½ÇPID		
			pidRateX.desired = pidRoll.out; //½«Íâ»·µÄPIDÊä³ö×÷ÎªÄÚ»·PIDµÄÆÚÍûÖµ¼´Îª´®¼¶PID
			pidUpdate(&pidRateX,dt);  //ÔÙµ÷ÓÃÄÚ»·

		 	pidUpdate(&pidPitch,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	¸©Ñö½ÇPID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //ÔÙµ÷ÓÃÄÚ»·

			CascadePID(&pidRateZ,&pidYaw,dt);	//Ò²¿ÉÒÔÖ±½Óµ÷ÓÃ´®¼¶PIDº¯ÊıÀ´´¦Àí
			break;
		case EXIT_255:  						//ÍË³ö¿ØÖÆ
			pidRest(pPidObject,8);		//¸´Î»PID²ÎÊı
			status = WAITING_1;				//·µ»ØµÈ´ı½âËø
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void) //µç»ú¿ØÖÆ
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: 	     //µÈ´ı½âËø	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //Èç¹ûËø¶¨£¬Ôòµç»úÊä³ö¶¼Îª0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //½âËøÍê³ÉºóÅĞ¶ÏÊ¹ÓÃÕßÊÇ·ñ¿ªÊ¼²¦¶¯Ò£¸Ë½øĞĞ·ÉĞĞ¿ØÖÆ
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr_temp;
								
				if(ALL_flag.height_lock) //¶¨¸ßÄ£Ê½ÏÂ ÓÍÃÅÒ£¸Ë×÷Îªµ÷Õû¸ß¶ÈÊ¹ÓÃ   
				{	
          pidHeightRate.out = LIMIT(pidHeightRate.out,-100,100); 					
					thr_temp = pidHeightRate.out + (Remote.thr -1000); //Êä³ö¸øµç»úµÄÊÇ¶¨¸ßÊä³öÖµ
				}
				else 										 //Õı³£·ÉĞĞ×´Ì¬£¬ÓÍÃÅÕı³£Ê¹ÓÃ
				{
					thr_temp = Remote.thr -1000; //Êä³ö¸øµç»úµÄÊÇÓÍÃÅÊä³öÖµ
				}
				
				if(Remote.thr<1020)		//ÓÍÃÅÌ«µÍÁË£¬ÔòÏŞÖÆÊä³ö  ²»È»·É»úÂÒ×ª												
				{
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
					break;
				}
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,750); //Áô100¸ø×ËÌ¬¿ØÖÆ

				MOTOR1 +=    + pidRateX.out - pidRateY.out + pidRateZ.out;//; ×ËÌ¬Êä³ö·ÖÅä¸ø¸÷¸öµç»úµÄ¿ØÖÆÁ¿
				MOTOR2 +=    + pidRateX.out + pidRateY.out - pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out + pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out - pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //Èç¹ûËø¶¨£¬Ôòµç»úÊä³ö¶¼Îª0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
#elif (FLY_TYPE >= 3)
	
	PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
	PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
	PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3] ;     
	PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
#else
	#error Please define FLY_TYPE!
		
#endif

} 


/************************************END OF FILE********************************************/ 



