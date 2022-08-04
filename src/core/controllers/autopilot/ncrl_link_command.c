#include <stdint.h>
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "autopilot.h"
#include "fence.h"
#include "sys_time.h"
#include "se3_math.h"
#include "autopilot.h"
#include "takeoff_landing.h"
#include "waypoint_following.h"
#include "ncrl_link.h"

extern autopilot_t autopilot;


void ncrl_link_command_handler(char mode_, char aux_info, float* curr_pos_enu){
    
    send_ncrl_link_fsm_msg();
    char mode = mode_;
    switch (mode){
        case '0' : /* takeoff */
		    autopilot_takeoff_handler();
            //send_ncrl_link_fsm_msg();
            break;

        case '1' : /* goto */
		    autopilot_goto_handler(curr_pos_enu);
            //send_ncrl_link_fsm_msg();
            break;

        case '2' : /* landing */
		    autopilot_landing_handler(curr_pos_enu);
            //send_ncrl_link_fsm_msg();
            break;
    }
}
