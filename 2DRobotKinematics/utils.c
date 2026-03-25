# include "utils.h"

double deg2rad(const double deg){
    return deg * (PI / 180);
}
double rad2deg(const double rad){
    return rad * (180 / PI);
}

double distance(Robot *r, Point target){
    // Distance between two points
    return sqrt(pow((r -> pose.x - target.x),2) + pow((r -> pose.y - target.y),2));
}


double myAtan2(const double x , const double y){
    if(x == 0){
        // Since y/x is undefined, manually return a value.
        if(y > 0){
            return (PI / 2);
        }else if(y < 0){
            return (-PI / 2);
        }else{
            //Singular values, although almost impossible, help prevent program errors caused by unforeseen circumstances.
            return 0.0;
        }
    }else if(x > 0){
        // In the first or fouth quadrant.
        return atan(y/x);
    }
    
    // f_x < 0
    if(y < 0){
        // In the thrid quadrant.
        return (atan(y/x) - PI);
    }
    // In the second quadrant which include situation of f_x = 0.
    return (atan(y/x) + PI);
}


double normalizeAngle(const double theta){
    // normalizing the angle into the interval (-pi, pi].
    // Find the x and y coordinates of the robot's facing direction.
    double f_x = cos(theta);
    double f_y = sin(theta);
    
    return myAtan2(f_x, f_y);
}





