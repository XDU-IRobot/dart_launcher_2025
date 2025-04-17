#include "cmsis_os.h"
#include "remote_control.h"
#include "Launch_Task.h"
#include "Rc_Task.h"
#include "can.h"
// #include "librm.hpp"
#include "tim.h"

using rm::hal::Can;
using namespace rm::device;
using namespace rm::modules::algorithm;

extern const RC_ctrl_t *Remote_control1;
extern double target_angle;
extern int key1, key2, key3, key4;
double a, b, c, d, e;
uint16_t Circle_sign = 0;
uint16_t Bottom_sign = 0;
uint16_t Launch_sign = 0;
uint16_t Hook_sign = 0;
uint16_t Load_sign = 0;
uint16_t dart_sign = 0;
extern AimbotFrame_SCM_t Aimbot_s;

Dart_t Dart;
/********************can1电机***************************** */
M2006 *Load_motor;
PID<PIDType::kPosition> *Load_motor_speed_pid;

M3508 *Bottom_motor0;
PID<PIDType::kPosition> *Bottom_motor0_speed_pid;

M3508 *Bottom_motor1;
PID<PIDType::kPosition> *Bottom_motor1_speed_pid;

/*********************can2电机****************************** */
M3508 *Pitch_motor;
PID<PIDType::kPosition> *Pitch_motor_speed_pid;
PID<PIDType::kPosition> *Pitch_motor_angle_pid;

M2006 *Yaw_motor;
PID<PIDType::kPosition> *Yaw_motor_speed_pid;
PID<PIDType::kPosition> *Yaw_motor_angle_pid;

GM6020 *Circle_load_motor;
RingPID<PIDType::kPosition> *Circle_load_motor_pid;

M2006 *Hook_motor;
PID<PIDType::kPosition> *Hook_motor_speed_pid;

M2006 *Launch_motor;
PID<PIDType::kPosition> *Launch_motor_speed_pid;

EncoderDevice *YAW_EncodeDevice;
EncoderDevice *PITCH_EncodeDevice;


void Start_Launch() {
  Can can1(hcan1);
  Can can2(hcan2);
  

  can1.SetFilter(0, 0);
  can2.SetFilter(0, 0);
  /**********创建电机对象及其pid************** */
  Load_motor = new M2006(can1, 2);
  Load_motor_speed_pid = new PID<PIDType::kPosition>(3, 0, 1, LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Circle_load_motor = new GM6020(can2, 4);
  Circle_load_motor_pid = new RingPID<PIDType::kPosition>(35, 0, 650, 30000, 0, 8191);

  Bottom_motor0 = new M3508(can1, 5);
  Bottom_motor0_speed_pid = new PID<PIDType::kPosition>(20, 0, 2, 15000, BOTTOM_MOTOR_SPEED_MAX_IOUT);

  Bottom_motor1 = new M3508(can1, 6);
  Bottom_motor1_speed_pid = new PID<PIDType::kPosition>(20, 0, 1, 15000, BOTTOM_MOTOR_SPEED_MAX_IOUT);

  Pitch_motor = new M3508(can2, 1);
  Pitch_motor_speed_pid = new PID<PIDType::kPosition>(10, 0, 0.1, 15000, LOAD_MOTOR_SPEED_MAX_IOUT);

  Yaw_motor = new M2006(can1, 4);
  Yaw_motor_speed_pid = new PID<PIDType::kPosition>(3, 0, 0, LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Hook_motor = new M2006(can1, 3);
  Hook_motor_speed_pid = new PID<PIDType::kPosition>(10, 0, 1, LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  Launch_motor = new M2006(can1, 1);
  Launch_motor_speed_pid = new PID<PIDType::kPosition>(15, 0, 1, LOAD_MOTOR_SPEED_MAX_OUT, LOAD_MOTOR_SPEED_MAX_IOUT);

  PITCH_EncodeDevice = new EncoderDevice(can2, 0x49);
  YAW_EncodeDevice = new EncoderDevice(can2, 0x51);
  /*************************************** */

  can1.Begin();
  can2.Begin();

  Launch_Init();
  // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  //  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  osDelay(2);

  for (;;) {
    Mode_Judgement(&Dart);

    Hit_target(&Dart);
    a = PITCH_EncodeDevice->Angle();
    b = YAW_EncodeDevice->Angle();
    c = Bottom_motor0_speed_pid->value();
    d = Bottom_motor0_speed_pid->d_out();
    e = Bottom_motor0->rpm();


    DjiMotor<>::SendCommand();
    // htim1.Instance->CCR1=target_angle;
    //  htim1.Instance->CCR2=0;                      // 发送控制报文
    osDelay(1);
  }
}
/****************初始化***************** */
void Launch_State_Init(Dart_t *dart) {
  dart->Mode.Current_work_mode.Pitch_state = PITCH_STOP;
  dart->Mode.Current_work_mode.Yaw_state = YAW_STOP;
  dart->Mode.Current_work_mode.Bottom_state = BOTTOM_STOP;
  dart->Mode.Current_work_mode.Launch_state = LAUNCH_CLOSE;
  dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_STOP;
  dart->Mode.Current_work_mode.Load_state = LOAD_STOP;
  dart->Mode.Current_work_mode.Load_PWM_state = LOAD_PWM_UP;
  dart->Mode.Current_work_mode.Hook_state = HOOK_STOP;

  dart->Match_state = STATE_INIT_1;
  target_angle = 96;
}
void Launch_Init() {
  Launch_State_Init(&Dart);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
/****************模式判断******************* */
void Mode_Judgement(Dart_t *dart) {
  if (Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_DOWN) {
    dart->Control_mode = WEEK_MODE;

  } else if (Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_MID) {
    dart->Control_mode = NORMAL_MODE;
    Normal_Mode_Change(&Dart);
    Circle_sign = Remote_control1->rc.s[LEFT_CHANNEL];
  } else if (Remote_control1->rc.s[RIGHT_CHANNEL] == RC_SW_UP) {
    if (dart->Control_mode != MATCH_MODE) {
      dart->Match_state = STATE_INIT_1;
    }
    dart->Control_mode = MATCH_MODE;

    Match_Mode_Change(&Dart);
  }
}
/**************普通模式************* */
void Normal_Mode_Change(Dart_t *dart) {
  if (Remote_control1->rc.ch[4] >= 330 && Remote_control1->rc.ch[4] <= 660) {
    dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN_LITTLE;
  }
  if (Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_UP) {
    // Pitch_Mode_Control  俯仰控制
    if (Remote_control1->rc.ch[3] >= -660 && Remote_control1->rc.ch[3] < 0) {
      dart->Mode.Current_work_mode.Pitch_state = PITCH_DOWN;
    }
    if (Remote_control1->rc.ch[3] <= 660 && Remote_control1->rc.ch[3] > 0) {
      dart->Mode.Current_work_mode.Pitch_state = PITCH_UP;
    }
    if (Remote_control1->rc.ch[3] == 0) {
      dart->Mode.Current_work_mode.Pitch_state = PITCH_STOP;
    }

    // Yaw_Mode_Control  偏航控制
    if (Remote_control1->rc.ch[0] <= 660 && Remote_control1->rc.ch[0] > 0) {
      dart->Mode.Current_work_mode.Yaw_state = YAW_RIGHT;
    }
    if (Remote_control1->rc.ch[0] >= -660 && Remote_control1->rc.ch[0] < 0) {
      dart->Mode.Current_work_mode.Yaw_state = YAW_LEFT;
    }
    if (Remote_control1->rc.ch[0] == 0) {
      dart->Mode.Current_work_mode.Yaw_state = YAW_STOP;
    }
  }

  else if (Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_MID) {
    // Circle_load_Mode_Control 旋转装填控制
    //  if(Remote_control1->rc.ch[1]==660)
    //  {
    //   dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_TEST;
    // }

    // //Load_PWM_Mode_Control  舵机装填控制
    // if(Remote_control1->rc.ch[2]<=660&&Remote_control1->rc.ch[2]>=330)
    // {
    // 	dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_DOWN;
    //   dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
    // }
    // if(Remote_control1->rc.ch[2]>=-660&&Remote_control1->rc.ch[2]<=-330)
    // {
    // 	dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP;
    // }

    // //Load_Mode_Control 装填控制
    // if(Remote_control1->rc.ch[3]==-660)
    // {
    // 	 dart->Mode.Current_work_mode.Load_state=LOAD_DOWN;
    // }
    // if(Remote_control1->rc.ch[3]==660)
    // {
    // 	 dart->Mode.Current_work_mode.Load_state=LOAD_UP;
    // }
    // if(Remote_control1->rc.ch[3]<660&&Remote_control1->rc.ch[3]>-660)
    // {
    // 	 dart->Mode.Current_work_mode.Load_state=LOAD_STOP;
    // }
    // Bottom_Mode_Control  蓄力控制
    //  if(Remote_control1->rc.ch[1] >= -660&&Remote_control1->rc.ch[1]<0)
    //  {
    //  	 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_DOWN;
    //  }
    //  if(Remote_control1->rc.ch[1] <= 660&&Remote_control1->rc.ch[1]>0)
    //  {
    //  	 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_UP;
    //  }
    //  else
    //  {
    //  	 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_STOP;
    //  }

    // 	//Launch_Mode_Control  发射控制
    //  if(Remote_control1->rc.ch[4]<=660&&Remote_control1->rc.ch[4]>=330)
    //  {
    //    dart->Mode.Current_work_mode.Launch_state=LAUNCH_OPEN;
    //  }
    //  else
    //  {
    //    dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
    //  }
    //   //Bottom_Mode_Control  蓄力控制
    // 	if(Remote_control1->rc.ch[1] >= -660&&Remote_control1->rc.ch[1]<0)
    // 	{
    // 		 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_DOWN;
    // 	}
    // 	else if(Remote_control1->rc.ch[1] <= 660&&Remote_control1->rc.ch[1]>0)
    // 	{
    // 		 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_UP;
    // 	}
    // 	else if(Remote_control1->rc.ch[1]==0)
    // 	{
    //    // HAL_Delay(10);
    // 		dart->Mode.Current_work_mode.Bottom_state=BOTTOM_STOP;
    // 	}
  }

  else if (Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_DOWN) {
    // Circle_load_Mode_Control 旋转装填控制
    if (Circle_sign == RC_SW_MID) {
      dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_CLOCKWISE;
    } else if (Circle_sign == RC_SW_DOWN || Circle_sign == RC_SW_UP) {
      dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_STOP;
    }
    // Bottom_Mode_Control  蓄力控制
    if (Remote_control1->rc.ch[1] >= -660 && Remote_control1->rc.ch[1] < 0) {
      dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN;
    } else if (Remote_control1->rc.ch[1] <= 660 && Remote_control1->rc.ch[1] > 0) {
      dart->Mode.Current_work_mode.Bottom_state = BOTTOM_UP;
    }
    // else if(Remote_control1->rc.ch[0]==660)
    // {
    // 	 dart->Mode.Current_work_mode.Bottom_state=BOTTOM_DOWN_LITTLE;
    // }
    else if (Remote_control1->rc.ch[4] != 660 && Remote_control1->rc.ch[1] == 0) {
      dart->Mode.Current_work_mode.Bottom_state = BOTTOM_STOP;
    }

    // Launch_Mode_Control  发射控制
    //  if(Remote_control1->rc.ch[4]==-660)
    //  {
    //   	dart->Mode.Current_work_mode.Launch_state=LAUNCH_OPEN;
    // }
    // else
    // {
    //   dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
    // }

    // Load_PWM_Mode_Control  舵机装填控制
    //  if(Remote_control1->rc.ch[2]==660)
    //  {
    //  	dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_DOWN;
    //   dart->Mode.Current_work_mode.Launch_state=LAUNCH_CLOSE;
    // }
    // if(Remote_control1->rc.ch[2]==-660)
    // {
    // 	dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_UP;
    // }

    // Load_Mode_Control 装填控制
    if (Remote_control1->rc.ch[3] == -660) {
      dart->Mode.Current_work_mode.Load_state = LOAD_DOWN;
    }
    if (Remote_control1->rc.ch[3] == 660) {
      dart->Mode.Current_work_mode.Load_state = LOAD_UP;
    }
    if (Remote_control1->rc.ch[3] < 660 && Remote_control1->rc.ch[3] > -660) {
      dart->Mode.Current_work_mode.Load_state = LOAD_STOP;
    }

    // Hook_Mode_Control  回旋控制
    if (Remote_control1->rc.ch[0] == 660) {
      dart->Mode.Current_work_mode.Hook_state = HOOK_CLOCKWISE;
    } else if (Remote_control1->rc.ch[0] == -660) {
      dart->Mode.Current_work_mode.Hook_state = HOOK_ANTICLOCKWISE;
    } else if (Remote_control1->rc.ch[0] < 660 && Remote_control1->rc.ch[0] > -660) {
      dart->Mode.Current_work_mode.Hook_state = HOOK_STOP;
    }

    if (Remote_control1->rc.ch[2] <= 660 && Remote_control1->rc.ch[2] > 0) {
      dart->Mode.Current_work_mode.Launch_state = LAUNCH_OPEN;
    } else if (Remote_control1->rc.ch[2] >= -660 && Remote_control1->rc.ch[2] < 0) {
      dart->Mode.Current_work_mode.Launch_state = LAUNCH_CLOSE;
    } else if (Remote_control1->rc.ch[2] == 0) {
      dart->Mode.Current_work_mode.Launch_state = LAUNCH_STOP;
    }
  }
}

/******************比赛模式********************* */
void Match_Mode_Change(Dart_t *dart) {
  switch (dart->Match_state) {
    // 发射高度方向初始化状态
    case STATE_INIT_1: {
      dart->Mode.Current_work_mode.Bottom_3508_Peristent = 0;
      dart->Mode.Current_work_mode.Bottom_3508_Time_State = 0;
      dart->Mode.Current_work_mode.Load_Time_State = 0;
      dart->Angle_Target.Pitch_Angle = Initial_Pitch;
      dart->Angle_Target.Yaw_Angle = Initial_Yaw;
      Bottom_sign = 0;
      Circle_sign = 0;
      Launch_sign = 0;
      Hook_sign = 0;
      Load_sign = 0;
      if (Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_MID) {
        dart->Match_state = STATE_INIT_2;
      }
      break;
    }
    // 电机 舵机初始化
    case STATE_INIT_2: {
      dart->Mode.Current_work_mode.Bottom_state = BOTTOM_STOP;
      dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_STOP;
      dart->Mode.Current_work_mode.Load_state = LOAD_STOP;
      // dart->Mode.Current_work_mode.Yaw_state=YAW_STOP;
      // dart->Mode.Current_work_mode.Pitch_state=PITCH_STOP;
      dart->Mode.Current_work_mode.Yaw_state = YAW_MATCH;
      dart->Mode.Current_work_mode.Pitch_state = PITCH_MATCH;

      dart->Mode.Current_work_mode.Hook_state = HOOK_MATCH_UP;
      dart->Mode.Current_work_mode.Launch_state = LAUNCH_OPEN_MATCH;

      dart->Match_state = First_1;
      break;
    }
    // 第一发飞镖上膛，6020旋转，准备发射
    case First_1: {
      if (key1 == 0) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN;
      }
      if (key1 == 1 && dart->Mode.Current_work_mode.Bottom_3508_Time_State <= 10) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN_LITTLE;
        dart->Mode.Current_work_mode.Launch_state = LAUNCH_CLOSE_MATCH;
        dart->Mode.Current_work_mode.Bottom_3508_Time_State++;
      }
      if (key2 == 0 && dart->Mode.Current_work_mode.Bottom_3508_Time_State > 10) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_UP;
        Bottom_sign = 1;
      }
      if (key2 == 1 && Bottom_sign == 1) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_STOP;
        Bottom_sign = 2;
      }

      if (key2 == 1 && Bottom_sign == 2) {
        dart->Mode.Current_work_mode.Bottom_3508_Time_State = 0;
        Bottom_sign = 0;
        dart->Match_state = First_2;
      }
    }

    case First_2: {
      if (key2 == 1 && key1 == 1) {
        dart->Mode.Current_work_mode.Launch_state = LAUNCH_OPEN_MATCH;
      }
      if (key2 == 1 && key1 == 0) {
        dart->Angle_Target.Pitch_Angle += dart_change[2].Pitch_Change;
        dart->Angle_Target.Yaw_Angle += dart_change[2].Yaw_Change;
        dart->Match_state = Second_1;
      }
    }

    case Second_1: {
      if (key1 == 0) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN;
      }
      if (key1 == 1 && dart->Mode.Current_work_mode.Bottom_3508_Time_State <= 10) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN_LITTLE;
        dart->Mode.Current_work_mode.Launch_state = LAUNCH_CLOSE_MATCH;
        dart->Mode.Current_work_mode.Bottom_3508_Time_State++;
      }
      if (key2 == 0 && dart->Mode.Current_work_mode.Bottom_3508_Time_State > 10 && Circle_sign == 0) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_UP;
        dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_CLOCKWISE;
        Circle_sign = 1;
        Bottom_sign = 1;
      }
      if (key2 == 1 && Bottom_sign == 1) {
        dart->Mode.Current_work_mode.Bottom_state = BOTTOM_STOP;
        Bottom_sign = 2;
      }

      if (Circle_sign == 1 && Load_sign == 0) {
        dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_STOP;
        dart->Mode.Current_work_mode.Hook_state = HOOK_MATCH_DOWN;
        if (dart->Mode.Current_work_mode.Load_Time_State < 20) {
          dart->Mode.Current_work_mode.Load_state = LOAD_DOWN;
          dart->Mode.Current_work_mode.Load_Time_State++;
        }
        if (dart->Mode.Current_work_mode.Load_Time_State >= 20 && dart->Mode.Current_work_mode.Load_Time_State < 40) {
          dart->Mode.Current_work_mode.Load_state = LOAD_UP;
          dart->Mode.Current_work_mode.Load_Time_State++;
        }
        if (dart->Mode.Current_work_mode.Load_Time_State == 40) {
          dart->Mode.Current_work_mode.Load_state = LOAD_STOP;
          dart->Mode.Current_work_mode.Hook_state = HOOK_MATCH_UP;
          dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_CLOCKWISE;
          Load_sign = 1;
        }
      }
      if (Circle_sign == 1 && Load_sign == 1) {
        dart->Mode.Current_work_mode.Circle_load_state = CIRCLE_LOAD_STOP;
        Load_sign = 2;
      }
      if (Load_sign == 2 && key2 == 1 && key1 == 1) {
        dart->Mode.Current_work_mode.Launch_state = LAUNCH_OPEN_MATCH;
        dart->Match_state = Second_2;
      }
    }

    case Second_2: {
      if (Load_sign == 2 && key2 == 1 && key1 == 0 && Remote_control1->rc.s[LEFT_CHANNEL] == RC_SW_DOWN) {
        dart->Mode.Current_work_mode.Bottom_3508_Time_State = 0;
        Circle_sign = 0;
        Bottom_sign = 0;
        Load_sign == 0;
        dart->Mode.Current_work_mode.Load_Time_State = 0;
        if (dart_sign <= 1) {
          dart->Angle_Target.Pitch_Angle += dart_change[3 + dart_sign].Pitch_Change;
          dart->Angle_Target.Yaw_Angle += dart_change[3 + dart_sign].Yaw_Change;
          dart->Match_state = Second_1;
        }
      }
    }
  }
}

// if(dart->Mode.Current_work_mode.Bottom_3508_Peristent<35&&dart->Mode.Current_work_mode.Bottom_3508_Time_State==0)
// {
//   dart->Mode.Current_work_mode.Bottom_state = BOTTOM_DOWN;
//   dart->Mode.Current_work_mode.Bottom_3508_Peristent++;
// }

// if(dart->Mode.Current_work_mode.Bottom_3508_Peristent==35&&dart->Mode.Current_work_mode.Bottom_3508_Time_State<=40)
// {
//   dart->Mode.Current_work_mode.Bottom_state=BOTTOM_UP;
//   dart->Mode.Current_work_mode.Bottom_3508_Time_State++;
//   if(Circle_sign==0)
//   {
//     dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_CLOCKWISE;
//     Circle_sign=1;
//   }
//   if(Circle_sign==1)
//   {
//     dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_STOP;
//     dart->Mode.Current_work_mode.Hook_state=HOOK_MATCH_DOWN;
//     if(dart->Mode.Current_work_mode.Load_Time_State<20)
//     {
//       dart->Mode.Current_work_mode.Load_state=LOAD_DOWN;
//       dart->Mode.Current_work_mode.Load_Time_State++;
//     }
//     if(dart->Mode.Current_work_mode.Load_Time_State>=20&&dart->Mode.Current_work_mode.Load_Time_State<40)
//     {
//       dart->Mode.Current_work_mode.Load_state=LOAD_UP;
//       dart->Mode.Current_work_mode.Load_Time_State++;
//     }
//     }
//   }

// dart->Mode.Current_work_mode.Circle_load_state=CIRCLE_LOAD_CLOCKWISE;
// dart->Mode.Current_work_mode.Load_PWM_state=LOAD_PWM_DOWN;
// dart->Mode.Current_work_mode.Load_state=LOAD_DOWN;
// HAL_Delay(100);
// dart->Mode.Current_work_mode.Load_state=LOAD_UP;
// HAL_Delay(100);
// dart->Mode.Current_work_mode.Load_state=LOAD_STOP;

/****************发射初始化*************************** */
void Hit_target(Dart_t *dart) {
  switch (dart->Control_mode) {
    case WEEK_MODE:
      All_States_Init(&Dart);
      break;

    case NORMAL_MODE:
      Begin_Launch();
      break;
    case MATCH_MODE:
      Begin_Launch();
      break;
    default:
      break;
  }
}
/****************无力模式调整********************* */
void All_States_Init(Dart_t *dart) {
  // 3508无力
  // Bottom_motor0_speed_pid->Update(0,Bottom_motor0->rpm());
  Bottom_motor0->SetCurrent(static_cast<int16_t>(0));
  // Bottom_motor1_speed_pid->Update(0,Bottom_motor1->rpm());
  Bottom_motor1->SetCurrent(static_cast<int16_t>(0));
  // 6020无力
  // Circle_load_motor_pid->Update(0, Circle_load_motor->rpm());  // 更新PID控制器输出
  Circle_load_motor->SetCurrent(static_cast<int16_t>(0));  // 设置电机电流

  // 装填2006无力
  // Load_motor_speed_pid->Update(0,Load_motor->rpm());
  Load_motor->SetCurrent(static_cast<int16_t>(0));
  // Pitch2006无力
  // Pitch_motor_speed_pid->Update(0,Pitch_motor->rpm());
  Pitch_motor->SetCurrent(static_cast<int16_t>(0));
  // Yaw2006无力
  // Yaw_motor_speed_pid->Update(0,Yaw_motor->rpm());
  Yaw_motor->SetCurrent(static_cast<int16_t>(0));

  Hook_motor->SetCurrent(static_cast<int16_t>(0));

  Launch_motor->SetCurrent(static_cast<int16_t>(0));
}

/****************普通模式各个电机舵机控制*********************** */
void Begin_Launch() {
  Pitch_control(&Dart);  // pitch轴调整

  Yaw_control(&Dart);  // Yaw轴调整

  Bottom_control(&Dart);  // 3508下拉蓄力

  Load_control(&Dart);  // 装填电机运动

  Circle_load_control(&Dart);  // 旋转装填控制

  Launch_control(&Dart);  // 发射扳机状态

  // Load_PWM_control(&Dart);            //装填舵机状态

  Hook_control(&Dart);  // 回旋电机控制
}

/******旋转装填电机控制****** */
void Circle_load_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Circle_load_state) {
    case CIRCLE_LOAD_STOP: {
      break;
    }
    case CIRCLE_LOAD_CLOCKWISE: {
      target_angle += 8191 / 6.0;

      if (target_angle > 8191) {
        target_angle -= 8191;
      }
      break;
    }
    case CIRCLE_LOAD_ANTICLOCKWISE: {
      target_angle -= 8191 / 6.0;

      if (target_angle < 0) {
        target_angle += 8191;
      }
      break;
    }
    // case CIRCLE_LOAD_TEST:
    // {
    //   target_angle+=8191/6.0;

    //   if(target_angle>8191)
    //   {
    //     target_angle-=8191;
    //   }
    //   HAL_Delay(1000);
    //   break;
    // }
    default:
      break;
  }
  Circle_load_motor_pid->Update(target_angle, Circle_load_motor->encoder());            // 更新PID控制器输出
  Circle_load_motor->SetCurrent(static_cast<int16_t>(Circle_load_motor_pid->value()));  // 设置电机电流
}

// /******装填电机控制***** */
void Load_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Load_state) {
    case LOAD_STOP: {
      Load_motor_speed_pid->Update(0, Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value()));
      break;
    }
    case LOAD_UP: {
      Load_motor_speed_pid->Update(4000, Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value()));
      break;
    }
    case LOAD_DOWN: {
      Load_motor_speed_pid->Update(-4000, Load_motor->rpm());
      Load_motor->SetCurrent(static_cast<int16_t>(Load_motor_speed_pid->value()));
      break;
    }
    default:
      break;
  }
}

/******底部3508电机控制***** */
void Bottom_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Bottom_state) {
    case BOTTOM_STOP: {
      Bottom_motor0_speed_pid->Update(0, Bottom_motor0->rpm());
      Bottom_motor1_speed_pid->Update(0, Bottom_motor1->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value()));
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value()));
      break;
    }
    case BOTTOM_DOWN: {
      Bottom_motor0_speed_pid->Update(Remote_control1->rc.ch[1] * 4.5, Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value()));
      Bottom_motor1_speed_pid->Update(-Remote_control1->rc.ch[1] * 4.5, Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value()));
      break;
    }
    case BOTTOM_UP: {
      Bottom_motor0_speed_pid->Update(Remote_control1->rc.ch[1] * 4.5, Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value()));
      Bottom_motor1_speed_pid->Update(-Remote_control1->rc.ch[1] * 4.5, Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value()));
      break;
    }
    case BOTTOM_DOWN_LITTLE: {
      Bottom_motor0_speed_pid->Update(-250, Bottom_motor0->rpm());
      Bottom_motor0->SetCurrent(static_cast<int16_t>(Bottom_motor0_speed_pid->value()));
      Bottom_motor1_speed_pid->Update(250, Bottom_motor1->rpm());
      Bottom_motor1->SetCurrent(static_cast<int16_t>(Bottom_motor1_speed_pid->value()));
    }
    default:
      break;
  }
}

/******Pitch轴电机控制***** */
void Pitch_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Pitch_state) {
    case PITCH_STOP: {
      Pitch_motor_speed_pid->Update(0, Pitch_motor->rpm());
      Pitch_motor->SetCurrent(static_cast<int16_t>(Pitch_motor_speed_pid->value()));
      break;
    }
    case PITCH_DOWN: {
      Pitch_motor_speed_pid->Update(-1000, Pitch_motor->rpm());
      Pitch_motor->SetCurrent(static_cast<int16_t>(Pitch_motor_speed_pid->value()));
      break;
    }
    case PITCH_UP: {
      Pitch_motor_speed_pid->Update(1000, Pitch_motor->rpm());
      Pitch_motor->SetCurrent(static_cast<int16_t>(Pitch_motor_speed_pid->value()));
      break;
    }
    case PITCH_MATCH: {
      Pitch_motor_angle_pid->Update(dart->Angle_Target.Pitch_Angle, PITCH_EncodeDevice->Angle());
      Pitch_motor_speed_pid->Update(Pitch_motor_angle_pid->value(), Pitch_motor->rpm());
      Pitch_motor->SetCurrent(static_cast<int16_t>(Pitch_motor_speed_pid->value()));
      break;
    }
    default:
      break;
  }
}

/*******Yaw轴电机控制****** */
void Yaw_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Yaw_state) {
    case YAW_STOP: {
      Yaw_motor_speed_pid->Update(0, Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    case YAW_LEFT: {
      Yaw_motor_speed_pid->Update(5000, Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    case YAW_RIGHT: {
      Yaw_motor_speed_pid->Update(-5000, Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    case YAW_MATCH: {
      Yaw_motor_angle_pid->Update(dart->Angle_Target.Yaw_Angle, YAW_EncodeDevice->Angle());
      Yaw_motor_speed_pid->Update(Yaw_motor_angle_pid->value(), Yaw_motor->rpm());
      Yaw_motor->SetCurrent(static_cast<int16_t>(Yaw_motor_speed_pid->value()));
      break;
    }
    default:
      break;
  }
}

/******发射电机控制***** */
void Launch_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Launch_state) {
    case LAUNCH_STOP: {
      Launch_motor_speed_pid->Update(0, Launch_motor->rpm());
      Launch_motor->SetCurrent(static_cast<int16_t>(Launch_motor_speed_pid->value()));
      break;
    }
    case LAUNCH_OPEN: {
      Launch_motor_speed_pid->Update(Remote_control1->rc.ch[2] * 2, Launch_motor->rpm());

      Launch_motor->SetCurrent(static_cast<int16_t>(Launch_motor_speed_pid->value()));
      break;
    }
    case LAUNCH_CLOSE: {
      Launch_motor_speed_pid->Update(Remote_control1->rc.ch[2] * 2, Launch_motor->rpm());

      Launch_motor->SetCurrent(static_cast<int16_t>(Launch_motor_speed_pid->value()));
      break;
    }
    case LAUNCH_OPEN_MATCH: {
      break;
    }
    case LAUNCH_CLOSE_MATCH: {
      break;
    }
    default:
      break;
  }
}

/******PWM装填舵机控制***** */
// void Load_PWM_control(Dart_t *dart)
// {
//   switch(dart->Mode.Current_work_mode.Load_PWM_state)
//   {
//     case LOAD_PWM_UP:
//     {
//       htim1.Instance->CCR1=450;
//       break;
//     }
//     case LOAD_PWM_DOWN:
//     {
//       htim1.Instance->CCR1=1990;
//       break;
//     }
//     default: break;
//   }
// }

/******回旋电机控制***** */
void Hook_control(Dart_t *dart) {
  switch (dart->Mode.Current_work_mode.Hook_state) {
    case HOOK_STOP: {
      Hook_motor_speed_pid->Update(0, Hook_motor->rpm());
      Hook_motor->SetCurrent(static_cast<int16_t>(Hook_motor_speed_pid->value()));
      break;
    }
    case HOOK_CLOCKWISE: {
      Hook_motor_speed_pid->Update(-1000, Hook_motor->rpm());
      Hook_motor->SetCurrent(static_cast<int16_t>(Hook_motor_speed_pid->value()));
      break;
    }
    case HOOK_ANTICLOCKWISE: {
      Hook_motor_speed_pid->Update(1000, Hook_motor->rpm());
      Hook_motor->SetCurrent(static_cast<int16_t>(Hook_motor_speed_pid->value()));
      break;
    }
    case HOOK_MATCH_DOWN: {
      break;
    }
    case HOOK_MATCH_UP: {
      break;
    }
    default:
      break;
  }
}

// void UsbReceive(uint8_t *rx_data, uint8_t len)
// {
//   if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
//     switch (rx_data[1]) {
//       case 0x02:
//         memcpy(&Aimbot_s, rx_data, len);
//         Aimbot_s.YawRelativeAngle = Aimbot_s.Yaw;
//         Aimbot_s.PitchRelativeAngle = Aimbot_s.Pitch;
//         if ((Aimbot_s.AimbotState & 0x02) == 2)
//         {
//           Aimbot_s.Aimbot_Shoot_Flag = 1;
//         }
//         else if (Aimbot_s.Aimbot_Shoot_Flag == 2)
//         {
//           Aimbot_s.Aimbot_Shoot_Flag = 0;
//         }
//         break;
//       default:
//         break;
//     }
//   }
// }
