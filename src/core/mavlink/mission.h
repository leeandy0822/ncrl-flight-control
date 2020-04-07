#ifndef __MISSION_H__
#define __MISSION_H__

#include "mavlink.h"

void mav_command_long(mavlink_message_t *received_msg);
void mav_mission_request_list(mavlink_message_t *received_msg);
void mav_trajectory_representation_waypoints(mavlink_message_t *received_msg);

#endif
