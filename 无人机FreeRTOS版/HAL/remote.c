//========================================================================
//	remote驱动
//========================================================================

#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "WIFI_UFO.h"
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "flow.h"
#include "ELRS.h"
#include "spl06.h"
#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  ͨ�����ݴ���   
 ******************************************************************************************/
u16 test_flag,set_flag;
uint8_t RC_rxData[32];
void remote_unlock(void);	


void Rc_Connect(void)
{
	    if (rcChannels[7] > 0)
	    {CRSF_Connect();}
	    else 
			{  //�����������ȼ�
	      //1��ң����  2��WiFiͼ��ģ�� 
	      if(NRF_Connect()==0)
	      {
	      	if(WIFI_UFO_Connect()==0)
	      	{
	
	      	 }	
				 }	
	     }
}


 
void RC_Analy(void)  
{
		static uint16_t cnt,cnt_tmp;
/*             Receive  and check RC data                               */	
//	static u8 Connect_flag;
	 ///ģʽѡ����г���

		
	//�����������ȼ�
	if(Nrf_Erro==1 || WIFI_UFO_Err==1 || rcChannels[7] > 0) 
	{ 	

				{
//							const float roll_pitch_ratio = 0.04f;
							const float yaw_ratio =  0.0015f;    
					
//							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //��ң��ֵ��Ϊ���нǶȵ�����ֵ
//							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					
//							if((Flow_Err ==0) && (mini.ok == 1) && (FlightData.High.bara_height < 40))//����ģ����λ  ������Ч   //ֱ�ӿ����ٶ� ���ҹص��⻷����
//							{
//								Flow_SpeedPid_y.desired = -(Remote.pitch-1500)*roll_pitch_ratio;   //ҡ�˿����ٶ�����ֵ 
//								Flow_SpeedPid_x.desired = -(Remote.roll-1500) *roll_pitch_ratio;
//								pidPitch.desired = -(Remote.pitch-1500)*0.02f;  //ң����ҡ�˿�����̬�⻷����ֵ
//								pidRoll.desired  = -(Remote.roll-1500) *0.02f;  //ң����ҡ�˿�����̬�⻷����ֵ 								
//							}
//							else
//							{
//								pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;
//								pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  
//							}
//					
					    if(Remote.yaw>1650)
							{
								pidYaw.desired -= 0.80f;	
							}
							else if(Remote.yaw <1350)
							{
								pidYaw.desired += 0.80f;	
							}	
				
				}
				remote_unlock(); // �����ж�
		}		
//���3��û�յ�ң�����ݣ����ж�ң���źŶ�ʧ���ɿ����κ�ʱ��ֹͣ���У��������ˡ�
//���������ʹ���߿ɽ����ر�ң�ص�Դ������������3��������رգ��������ˡ�
//�����ر�ң�أ�����ڷ����л�ֱ�ӵ��䣬���ܻ��𻵷�������
  else
	{					
				cnt++;
				if(cnt>800)						//3��û��ң�����ź� �ж�ң����ʧ�� �źŶ��� �Զ��½�����
				{	
					
					Remote.roll = 1500;  //ͨ��1    ���ݹ���
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = 1500;  //ͨ��2		���ݹ���
					LIMIT(Remote.pitch,1000,2000);
					Remote.yaw =  1500;   //ͨ��4		���ݹ���
					LIMIT(Remote.yaw,1000,2000);		
					if(Remote.thr < 1030)						//�ж�����
					{
							cnt = 0;
							Remote.thr =1000;						//�ر�����
							ALL_flag.unlock = 0; 				//�˳�����
							LED.status = AllFlashLight; //��ʼ����		
							NRF24L01_init();						//��λһ��2.4Gģ��				
					}
					else
					{	
						cnt = 810;
						if(cnt_tmp++>100)                 //�������ż�С��ʱ��
						{
							cnt_tmp=0;
//							printf("Remote.thr: %d  \r\n",Remote.thr);				//����1�Ĵ�ӡ			
							Remote.thr = 	Remote.thr-20;   //ͨ��3 ����ͨ����ԭ���Ļ������Զ�������С  �𵽷ɻ������½�
						}				
					}
					LIMIT(Remote.thr,1000,2000);			
				} 
	}	
}

/*****************************************************************************************
 *  �����ж�    
 ******************************************************************************************/	
void remote_unlock(void)   //�������ݽ���
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1200 &&Remote.yaw>1800)                         //����ң�����½������ɻ�
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://�ȴ�����
			if(Remote.thr<1150)           //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1800)        //��������  
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1150)     //�������Ž���     
			{			 
					 status = WAITING_4;			//������־λ	
					 baro_start=1;            //��ѹ�����־ 
			}			
			break;			
		case WAITING_4:	//�����ɹ�
				ALL_flag.unlock = 1;  
				status = PROCESS_31;
				LED.status = AlwaysOn;  //����									
				 break;		
		case PROCESS_31:	//�������״̬
				if(Remote.thr<1020)
				{
					if(cnt++ > 2000)                                     // ������  ��������ң�˴������6S�Զ�����
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //Other conditions lock 
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //��������
			LED.status = AllFlashLight;	       //ȫ��ͬʱ��˸                          //exit
			cnt = 0;
			LED.FlashTime = 300; //300*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}



/***********************END OF FILE*************************************/








