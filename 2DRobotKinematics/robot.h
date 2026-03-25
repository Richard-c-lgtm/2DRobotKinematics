// This header only for define the some basic property of the robot

# ifndef ROBOT_H
# define ROBOT_H

# define WHEEL_RADIUS 0.05 // In meters
# define WHEEL_BASE 0.20 // The distance of the wheel(center). In meters
# define DT 0.01 // Time Step. In seconds[10ms]

# define CONSTANT_V 0.5 // The manually adjusted parameters are used to calculate the linear velocity required for distance error calculation.
# define CONSTANT_OMEGA 3.0 // The manually adjusted parameters are used to calculate the required angular velocity for the angle error.
# define CONSTANT_OMEGA_10 1.2 // The manually adjusted parameters are used to calculate the angular velocity when angle diff > 30 degree and < 90 degree.
# define MAX_V 0.8 // In meters

# define EPSILON 0.000001f // The minimum angular velocity; if it is less than this value, will directly using linear motion.
# define RADIAN_FOR_40 0.6981317f // 40 degrees in radians
# define RADIAN_FOR_10 0.17453293f // 10 degrees in radians

typedef struct{
    double x; // x component. In meter.
    double y; // y component. In meter.
    double theta; // The direction that robot facing at. In radian.
} Pose;


typedef struct{
    Pose pose; // Because the lifecycle of a pose is exactly the same as that of a robot, you don't need to malloc a new pose; you can simply nest it inside the robot.
    double v; // linear velocity.
    double omega; // angular velocity.
} Robot;


typedef struct{
    // Used to represent the coordinate position of the target point
    double x;
    double y;
} Point;


# endif
