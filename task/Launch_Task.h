#ifndef LAUNCH_TASH_H
#define LAUNCH_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

extern void Start_Launch();

#ifdef __cplusplus
}

#include "librm.hpp"

#include <iostream>
#include "librm/core/typedefs.h"

using rm::device::CanDevice;
using rm::hal::Can;

#define LEFT_CHANNEL 1
#define RIGHT_CHANNEL 0
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

typedef enum
{
  	WEEK_MODE						= 1,
	NORMAL_MODE						= 2,
	MATCH_MODE						= 3,
}Control_mode;
typedef enum
{
	
  	STATE_INIT_1							    = 1,
	STATE_INIT_2								= 2,
	
	First_1                                     = 11,
	First_2                                     = 12,
	First_3                                     = 13,
	First_4                                     = 14,
	
	Second_1									= 21,
	Second_2									= 22,
	Second_3									= 23,
	Second_4									= 24,
	
	Third_1										= 31,	
	Third_2										= 32,
	Third_3										= 33,
	Third_4										= 34,
	Third_5										= 35,
	Third_6										= 36,

	
	Four_1										= 41,
	Four_2										= 42,
	Four_3										= 43,
	Four_4										= 44,
	Four_5										= 45,
	Four_6										= 46,
	Four_7 										= 47,
	Four_8										= 48,
	
	Five_1										= 51,
	Five_2										= 52,
	  
}Match_state;
typedef enum
{
    BLOCKED=1,
	UNBLOCKED=2
}Bottom_3508_state;

typedef enum
{
	LOAD_UP=1,
	LOAD_DOWN=2,
	LOAD_STOP=3
}Load_state;

typedef enum
{
  CIRCLE_LOAD_STOP=1,
  CIRCLE_LOAD_CLOCKWISE=2,
  CIRCLE_LOAD_ANTICLOCKWISE=3,
  CIRCLE_LOAD_TEST=4
}Circle_load_state;

typedef enum
{
	LOAD_PWM_UP=1,
	LOAD_PWM_DOWN=2
}LOAD_PWM_STATE;

typedef enum
{
	YAW_STOP=1,
	YAW_LEFT=2,
	YAW_RIGHT=3,
	YAW_MATCH = 4
}Yaw_state;

typedef enum
{
    PITCH_STOP=1,
	PITCH_DOWN=2,
	PITCH_UP=3,
	PITCH_MATCH = 4
}Pitch_state;

typedef enum
{
    BOTTOM_STOP=1,
	BOTTOM_DOWN=2,
	BOTTOM_UP=3,
	BOTTOM_DOWN_LITTLE=4
}Bottom_state;


typedef enum
{
	LAUNCH_CLOSE=1,
	LAUNCH_OPEN=2,
	LAUNCH_STOP=3,
	LAUNCH_CLOSE_MATCH=4,
	LAUNCH_OPEN_MATCH=5
	
}Launch_state;

typedef enum
{
	HOOK_STOP=1,
	HOOK_CLOCKWISE=2,
	HOOK_ANTICLOCKWISE=3,
	HOOK_MATCH_UP=4,
	HOOK_MATCH_DOWN=5
}Hook_state;





typedef struct
{
	uint8_t Yaw_state;						
	uint8_t Pitch_state;					
	uint8_t Bottom_state;					
	uint8_t Load_state;						
	uint8_t Circle_load_state;			
	uint8_t Load_PWM_state;				
	uint8_t Launch_state;
	uint8_t Hook_state;
	
   
	uint8_t Bottom_3508_state;    
	uint16_t Bottom_3508_Peristent; 
	 
	uint16_t Bottom_3508_Time_State; 
	
	uint16_t Bottom_3508_block_time; 

	uint16_t Load_Time_State;
}Work_Mode_t;
typedef struct
{
   Work_Mode_t Current_work_mode;
}Work_Mode;
typedef struct
{
	float Bottom_speed;
    float Middle_speed;
	float Circle_load_speed;
	float Load_speed;
	float Yaw_speed;
	float Pitch_speed;
}Target_Speed_t;

typedef struct
{
	float Yaw_Angle;
	float Pitch_Angle;
}Angle_TarGet;

typedef struct
{
	uint8_t					Control_mode; 					
	Work_Mode				Mode;													
	uint8_t                 Match_state;
	Angle_TarGet			Angle_Target;
}Dart_t;

typedef struct
{
	float Yaw_Change;
	float Pitch_Change;
}Dart_Change;

Dart_Change dart_change[4];

typedef struct __attribute__((packed)) {
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 自瞄状态
    uint8_t AimbotState;
    uint8_t AimbotTarget;
    // 自瞄数据
    float Pitch;
    float Yaw;
    // 自瞄目标角速度
    float TargetPitchSpeed;
    float TargetYawSpeed;
    // 时间戳
    float SystemTimer;
    // 包尾
    uint8_t _EOF;
    // 处理后数据
    float PitchRelativeAngle;
    float YawRelativeAngle;
    uint8_t Aimbot_Shoot_Flag;
  } AimbotFrame_SCM_t;



void Launch_State_Init(Dart_t *dart);
void Launch_Init();
void Mode_Judgement(Dart_t *dart);
void Hit_target(Dart_t *dart);
void All_States_Init(Dart_t *dart);
void Begin_Launch();
void Normal_Mode_Change(Dart_t *dart);
void Match_Mode_Change(Dart_t *dart);
void Pitch_control(Dart_t *dart);
void Yaw_control(Dart_t *dart);
void Launch_control(Dart_t *dart);
void Circle_load_control(Dart_t *dart);
void Load_control(Dart_t *dart);
void Bottom_control(Dart_t *dart);
void Load_PWM_control(Dart_t *dart);
void Hook_control(Dart_t *dart);
void UsbReceive(uint8_t *rx_data, uint8_t len);
namespace rm::device{
class EncoderDevice : public CanDevice {
	public:
	  EncoderDevice(rm::hal::CanInterface &can, uint32_t rx_std_id)
		  : CanDevice{can, rx_std_id} {}
		
	  /** 取值函数 **/
  	[[nodiscard]] int16_t angle() const { return this->angle_; }
  	[[nodiscard]] int16_t loops() const { return this->loops_; }
  	[[nodiscard]] int16_t angular_velocity() const { return this->angular_velocity_; }

  	[[nodiscard]] float Angle() const { return (float)this->angle() * 360.f/32768.0f; }
	private:
	  void RxCallback(const rm::hal::CanMsg *msg) override {
		// 在这里处理接收到的报文
		this->angle_ = (int16_t)(msg->data[3] << 8) | msg->data[2];
 		this->loops_  = (int16_t)(msg->data[5] << 8) | msg->data[4];
  		this->angular_velocity_ = (int16_t)(msg->data[7] << 8) | msg->data[6];
		
	  }
  /**   FEEDBACK DATA   **/
  	 int16_t angle_{};    
  	 int16_t loops_{};        
 	 int16_t angular_velocity_{};    
 	
	};
} // namespace rm::device
#endif

#endif /* LAUNCH_TASK_H */