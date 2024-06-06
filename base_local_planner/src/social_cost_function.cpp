/*
 * social_cost_function.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: Morgan Quigley
 */

#include <base_local_planner/social_cost_function.h>

#include <math.h>

namespace base_local_planner {

void SocialCostFunction::setParams(int head_dir, int speed) {
  head_dir_ = head_dir; // LEFT 0, STRAIGHT 1, RIGHT 2
  speed_ = speed;       // SLOW DOWN 0, SPEED UP 1, MAINTAIN 2, STOP 3
}

void SocialCostFunction::resetParams() {
  head_dir_ = -1; // LEFT 0, STRAIGHT 1, RIGHT 2
  speed_ = -1;       // SLOW DOWN 0, SPEED UP 1, MAINTAIN 2, STOP 3
}

double SocialCostFunction::scoreTrajectory(Trajectory &traj) {
    double lateral_deviation = 0;
    double speed_deviation = 0;
    double preferred_angle = 0;
    double preferred_speed = 0.3;

    //  no social cost recieved
    if (head_dir_ < 0 || speed_ < 0)
        return 0;

    ////////////////////////////////// lateral
    // preference on the left
    if (head_dir_ == 1) 
        preferred_angle = 0.25;
    // preference on the straight
    else if (head_dir_ == 0) 
        preferred_angle = 0;
    // preference on the right
    else if (head_dir_ == 2) 
        preferred_angle = -0.25;

    lateral_deviation = fabs(preferred_angle - traj.thetav_);

    ////////////////////////////////// speed
    // slow down
    if (speed_ == 0) 
        preferred_speed = 0.2;
    // speed up
    else if (speed_ == 1) 
        preferred_speed = 0.4;
    // maintain
    else if (speed_ == 2)
        preferred_speed = 0.3;// do nothing
    // stop
    else if (speed_ == 3) 
        return fabs(traj.xv_)*100;

    speed_deviation = fabs(preferred_speed - traj.xv_);

    return lateral_deviation*50 + speed_deviation*5;
}

} /* namespace base_local_planner */
