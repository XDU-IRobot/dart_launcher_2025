#include "remote_control.h"
#include "cmsis_os.h"

uint32_t timer;
const RC_ctrl_t *Remote_control1;
double target_angle;

void rc_task(const void *pv_arg)
{
    remote_control_init();

    for(;;)
    {
        Remote_control1 = get_remote_control_point();
        // double dt = (double)(HAL_GetTick() - timer) / 1000;
        // timer = HAL_GetTick();
        // target_angle+=Remote_control1->rc.ch[1]*dt*0.2;

        // if(target_angle>=8191)
        // {
        //     target_angle-=8191;
        // }
        // else if (target_angle<=0)
        // {
        //     target_angle+=8191;
        // }

        osDelay(1);
    }
}