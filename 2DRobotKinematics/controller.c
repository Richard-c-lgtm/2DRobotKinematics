# include "controller.h"

int moveTo(Robot *r , Point goal , double *v , double *omega){
    // Returns 1 if arrived, returns 0 if not arrived.
    r -> pose.theta = normalizeAngle(r -> pose.theta);
    

    // Calculate the angle of the target point relative to the robot.
    double tempXForGoal = goal.x - r -> pose.x;
    double tempYForGoal = goal.y - r -> pose.y;
    double targetAngle = myAtan2(tempXForGoal, tempYForGoal);
  
    // Calculate distance and angle difference.
    double dist = distance(r , goal);
    double angleDiff = targetAngle - (r -> pose.theta);
    
    // Multiply the difference by an adjustable parameter to obtain the final angular velocity and linear velocity.
    // 调整参数关键点：
    // 1. 如果大于90度，则说明目标点在机器人后面，必须要原地旋转
    // 2. 如果30 ～ 90之间，说明比较大。大幅度降低线速度的转弯
    // 3. 如果小于30度，边走边修正。
    
    // A 1cm error margin is acceptable; otherwise, achieving perfect precision to the target point would be too demanding, potentially leading to constant fluctuations at the target point.
    if(dist > 0.01){
        if(fabs(angleDiff) > RADIAN_FOR_40){
            // The angle difference is too large(angle > 60), perform a stationary turning.
            *v = 0;
            *omega = CONSTANT_OMEGA * (angleDiff);
        }else if(fabs(angleDiff) > RADIAN_FOR_10){
            // The angle difference is not too large(30 < angle < 60), perform a stationary turning.
            // Because the angle is in (10,90).Therefore using cos function automaticly change the linear velocity makes robot moving more smothly.
            *v = cos(angleDiff) * dist;
            if(*v > MAX_V){
                // Limit the speed to avoid excessive speed over long distances.
                *v = MAX_V;
            }
            *omega = CONSTANT_OMEGA_10 * (angleDiff);
            if (dist > 0.01 && *v < 0.5) {
                // To prevent the robot from running too slowly
                *v = 0.5;
            }
        }else{
            // The angle difference is <= 10.
            *v = CONSTANT_V * dist;
            *omega = CONSTANT_OMEGA_10 * (angleDiff);
        }
        
        return 0;
    }
    return 1;
}


