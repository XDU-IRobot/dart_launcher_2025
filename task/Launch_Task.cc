#include "cmsis_os.h"
#include "remote_control.h"
#include "Launch_Task.h"
#include  "Rc_Task.h"
#include "can.h"
#include "librm.hpp"
#include "tim.h"

using rm::hal::Can;
using namespace rm::device;
using namespace rm::modules::algorithm;

extern const RC_ctrl_t *Remote_control1;
extern double target_angle;

Dart_t Dart;
/********************can1电机***************************** */
M2006 *Load_motor;
PID<PIDType::kPosition> *Load_motor_speed_pid;

GM6020 *Circle_load_motor;
RingPID<PIDType::kPosition> *Circle_load_motor_pid;

M3508 *Bottom_motor0;
PID<PIDType::kPosition> *Bottom_motor0_speed_pid;

M3508 *Bottom_motor1;
PID<PIDType::kPosition> *Bottom_motor1_speed_pid;

/*********************can2电机****************************** */
M2006 *Pitch_motor0;
PID<PIDType::kPosition> *Pitch_motor0_speed_pid;

M2006 *Pitch_motor1;
PID<PIDType::kPosition> *Pitch_motor1_speed_pid;

M2006 *Yaw_motor;
PID<PIDType::kPosition> *Yaw_motor_speed_pid;

void Start_Launch() 
{
  Can can1(hcan1);
  Can can2(hcan2);

  can1.SetFilter(0, 0);
  can2.SetFilter(0, 0);
  /**********创建电机对象及其pid************** */
  Load_motor=new M2006(can1, 1);
  Load_motor_speed_pid = new PID<PIDType::kPosition>(2,0.1,0,LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Circle_load_motor = new GM6020(can1, 2);
  Circle_load_motor_pid = new RingPID<PIDType::kPosition>(5, 0, 0, 30000, 0, 8191);

  Bottom_motor0 = new M3508(can1,3);
  Bottom_motor0_speed_pid = new PID<PIDType::kPosition>(6,0,0,BOTTOM_MOTOR_SPEED_MAX_OUT, BOTTOM_MOTOR_SPEED_MAX_IOUT);

  Bottom_motor1 = new M3508(can1,4);
  Bottom_motor1_speed_pid = new PID<PIDType::kPosition>(6,0,0,BOTTOM_MOTOR_SPEED_MAX_OUT, BOTTOM_MOTOR_SPEED_MAX_IOUT);

  Pitch_motor0 = new M2006(can2,1);
  Pitch_motor0_speed_pid = new PID<PIDType::kPosition>(2,0.1,0,LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Pitch_motor1 = new M2006(can2,2);
  Pitch_motor1_speed_pid = new PID<PIDType::kPosition>(2,0.1,0,LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Yaw_motor = new M2006(can2,3);
  Yaw_motor_speed_pid = new PID<PIDType::kPosition>(2,0.1,0,LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);
  /*************************************** */

  can1.Begin();
  can2.Begin();

  Launch_Init();
	osDelay(2);
  
  for (;;) {
    
		Mode_Judgement(&Dart);
    Hit_target(&Dart); 
    osDelay(1);
    
    DjiMotor<>::SendCommand();                             // 发送控制报文
    osDelay(1);
  }
}
/****************初始化***************** */
void Launch_State_Init(Dart_t *dart)
{
  dart->Mode.Current_work_mode.Pitch_state=PITCH_STOP;
  dart->Mode.Current_work_mode.Yaw_state=YAW_STOP;
	dart->Mode.Current_work_mode.Bottom_state=BOTTOM_STOP;
	dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
  dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_STOP;
  dart->Mode.Current_work_mode.Load_state=LOAD_STOP;
  dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP;
	
	dart->Match_state=STATE_INIT_1;
  target_angle=3000;

}
void Launch_Init()
{
  Launch_State_Init(&Dart);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	
}
/****************模式判断******************* */
void Mode_Judgement(Dart_t *dart)
{
	if(Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_DOWN)		
	{
		dart->Control_mode = WEEK_MODE;
	}
	else if(Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_MID)
	{
		dart->Control_mode = NORMAL_MODE;
	  Normal_Mode_Change(&Dart);
	}
	else if(Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_UP)
	{
		
		if(dart->Control_mode != MATCH_MODE)
		{
		dart->Match_state = STATE_INIT_1;
		}
		dart->Control_mode = MATCH_MODE;
		
	  Match_Mode_Change(&Dart);
	}
}
/**************普通模式************* */
void Normal_Mode_Change(Dart_t *dart)
{
  
	if(Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_UP)	
	{

		//Pitch_Mode_Control  俯仰控制
		if(Remote_control1->rc.ch[3] > -660 && Remote_control1->rc.ch[3] < 0)
		{
			 dart->Mode.Current_work_mode.Pitch_state=PITCH_DOWN;							
		}
		if(Remote_control1->rc.ch[3] < 660 && Remote_control1->rc.ch[3] > 0)
		{
			 dart->Mode.Current_work_mode.Pitch_state=PITCH_UP;
		}
		if(Remote_control1->rc.ch[3] == 0)
		{
			 dart->Mode.Current_work_mode.Pitch_state=PITCH_STOP;
		}
		

		//Yaw_Mode_Control  偏航控制
		if(Remote_control1->rc.ch[2] < 660 && Remote_control1->rc.ch[2] > 0)
		{
			 dart->Mode.Current_work_mode.Yaw_state = YAW_RIGHT;									
		}
		if(Remote_control1->rc.ch[2] > -660 && Remote_control1->rc.ch[2] < 0)
		{
			 dart->Mode.Current_work_mode.Yaw_state = YAW_LEFT;
		}
		if(Remote_control1->rc.ch[2] == 0)
		{
			 dart->Mode.Current_work_mode.Yaw_state=YAW_STOP;
		}			
  }  
		
	else if(Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_MID)	
	{
    //Circle_load_Mode_Control 旋转装填控制
    if(Remote_control1->rc.ch[4]==-660)
		{
			 dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_CLOCKWISE;			
		}
		else if(Remote_control1->rc.ch[4]==660)
		{
			 dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_ANTICLOCKWISE;
		}
    else 
    {
      dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_STOP;
    }

    //Load_PWM_Mode_Control  舵机装填控制
    if(Remote_control1->rc.ch[1]>=-660&&Remote_control1->rc.ch[1]<=-330)
		{
			dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_DOWN; 
      dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;		  			
		}
		if(Remote_control1->rc.ch[1]<=660&&Remote_control1->rc.ch[1]>=330)
		{
						
			dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP;  				
		}
   
		//Load_Mode_Control 装填控制
		if(Remote_control1->rc.ch[3]==-660)
		{
			 dart->Mode.Current_work_mode.Load_state=LOAD_DOWN;								
		}
		if(Remote_control1->rc.ch[3]==660)
		{
			 dart->Mode.Current_work_mode.Load_state=LOAD_UP;
		}
		if(Remote_control1->rc.ch[3]<660&&Remote_control1->rc.ch[3]>-660)
		{
			 dart->Mode.Current_work_mode.Load_state=LOAD_STOP;
		}	
	}
  else if(Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_DOWN)
  {
    //Bottom_Mode_Control  蓄力控制
		if(Remote_control1->rc.ch[1] < 0 && Remote_control1->rc.ch[1] >= -660)
		{
			 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_DOWN;						
		}
		if(Remote_control1->rc.ch[1] > 0 && Remote_control1->rc.ch[1] <= 660)
		{
			 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_UP;
		}
		if(Remote_control1->rc.ch[1] == 0)
		{
			 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_STOP;
		}	
		
		//Launch_Mode_Control  发射控制
		if(Remote_control1->rc.ch[4]==660 && (dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP))
		{
			dart->Mode.Current_work_mode.Launch_state=LAUNCH_OPEN; 						
		}
    else
    {
      dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
    }
  }
}

/******************比赛模式********************* */
void Match_Mode_Change(Dart_t *dart)
{

}

/****************发射初始化*************************** */
void Hit_target(Dart_t *dart)
{
	switch(dart->Control_mode)
	{
		case WEEK_MODE:
			All_States_Init(&Dart);
      Begin_Test_Launch();
		break;
		
		case NORMAL_MODE:
			Begin_Test_Launch();
		break;
		case MATCH_MODE:
	//		Begin_Match_Launch(&Dart);
		break;
		default: break;
	}	
}
/****************无力模式调整********************* */
void All_States_Init(Dart_t *dart)
{
		
	dart->Mode.Current_work_mode.Bottom_3508_state=BOTTOM_STOP;
	dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_STOP;
	dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
  dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP;
	dart->Mode.Current_work_mode.Load_state=LOAD_STOP;
	dart->Mode.Current_work_mode.Pitch_state=PITCH_STOP;
	dart->Mode.Current_work_mode.Yaw_state=YAW_STOP;
	
}

/****************普通模式各个电机舵机控制*********************** */
void Begin_Test_Launch()
{
	Pitch_control(&Dart);			//pitch轴调整
	
	Yaw_control(&Dart);				//Yaw轴调整
	
	Bottom_control(&Dart);		    //3508下拉蓄力
	
	Load_control(&Dart);			//装填电机运动
	
	Circle_load_control(&Dart);	    //旋转装填控制
	
	Launch_control(&Dart);		    //发射扳机状态
	
	Load_PWM_control(&Dart);            //装填舵机状态
	
	
}

/******旋转装填电机控制****** */
void Circle_load_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Circle_load_state)
  {
    case CIRCLE_LOAD_STOP:
    {
     break;
    }
    case CIRCLE_LOAD_CLOCKWISE:
    {
      target_angle+=8191/6.0;
      break;
    }
    case CIRCLE_LOAD_ANTICLOCKWISE:
    {
      target_angle-=8191/6.0;
      break;
    }
    default: break;
  }
  Circle_load_motor_pid->Update(target_angle, Circle_load_motor->encoder());  // 更新PID控制器输出
  Circle_load_motor->SetCurrent(static_cast<int16_t>(Circle_load_motor_pid->value()));  // 设置电机电流
}

/******装填电机控制***** */
void Load_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Load_state)
  {
    case LOAD_STOP:
    {
      Load_motor_speed_pid->Update(0,Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value()));
      break;
    }
    case LOAD_UP:
    {
      Load_motor_speed_pid->Update(4000,Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value())); 
      break;
    }
    case LOAD_DOWN:
    {
      Load_motor_speed_pid->Update(-4000,Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value())); 
      break;
    }
    default: break;
  }
}

/******底部3508电机控制***** */
void Bottom_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Bottom_state)
  {
    case BOTTOM_STOP:
    {
      Bottom_motor0_speed_pid->Update(0,Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value()));
      Bottom_motor1_speed_pid->Update(0,Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value())); 
      break;
    }
    case BOTTOM_DOWN:
    {
      Bottom_motor0_speed_pid->Update(4000,Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value())); 
      Bottom_motor1_speed_pid->Update(-4000,Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value())); 
      break;
    }
    case BOTTOM_UP:
    {
      Bottom_motor0_speed_pid->Update(-4000,Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value())); 
      Bottom_motor1_speed_pid->Update(4000,Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value()));
      break;
    }
    default: break;
  }
}

/******PWM装填舵机控制***** */
void Load_PWM_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Load_PWM_state)
  {
    case LOAD_PWM_UP:
    {
      htim1.Instance->CCR1=3000;
      break;
    }
    case LOAD_PWM_DOWN:
    {
      htim1.Instance->CCR1=800;
      break;
    }
    default: break;
  }
}

/******Pitch轴电机控制***** */
void Pitch_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Pitch_state)
  {
    case PITCH_STOP:
    {
      Pitch_motor0_speed_pid->Update(0,Pitch_motor0->rpm());
      Pitch_motor0->SetCurrent(static_cast<int16_t>(Pitch_motor0_speed_pid->value()));
      Pitch_motor1_speed_pid->Update(0,Pitch_motor1->rpm());
      Pitch_motor1->SetCurrent(static_cast<int16_t>(Pitch_motor1_speed_pid->value()));
      break;
    }
    case PITCH_DOWN:
    {
      Pitch_motor0_speed_pid->Update(-4000,Pitch_motor0->rpm());
      Pitch_motor0->SetCurrent(static_cast<int16_t>(Pitch_motor0_speed_pid->value()));
      Pitch_motor1_speed_pid->Update(-4000,Pitch_motor1->rpm());
      Pitch_motor1->SetCurrent(static_cast<int16_t>(Pitch_motor1_speed_pid->value()));
      break;
    }
    case PITCH_UP:
    {
      Pitch_motor0_speed_pid->Update(4000,Pitch_motor0->rpm());
      Pitch_motor0->SetCurrent(static_cast<int16_t>(Pitch_motor0_speed_pid->value()));
      Pitch_motor1_speed_pid->Update(4000,Pitch_motor1->rpm());
      Pitch_motor1->SetCurrent(static_cast<int16_t>(Pitch_motor1_speed_pid->value()));
      break;
    }
    default: break;
  }
}

/*******Yaw轴电机控制****** */
void Yaw_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Yaw_state)
  {
    case YAW_STOP:
    {
      Yaw_motor_speed_pid->Update(0,Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    case YAW_LEFT:
    {
      Yaw_motor_speed_pid->Update(-4000,Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    case YAW_RIGHT:
    {
      Yaw_motor_speed_pid->Update(4000,Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    default: break;
  }
}

/******发射舵机控制***** */
void Launch_control(Dart_t *dart)
{
  switch(dart->Mode.Current_work_mode.Launch_state)
  {
    case LAUNCH_CLOSE:
    {
      htim8.Instance->CCR3=3000;
      break;
    }
    case LAUNCH_OPEN:
    {
      htim8.Instance->CCR3=1000;
      break;
    }
    default: break;
  }
}


