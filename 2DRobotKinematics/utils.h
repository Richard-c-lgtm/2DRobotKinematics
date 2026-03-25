
# ifndef UTILS_H
# define UTILS_H
# include <math.h>
# include "robot.h"
# define PI 3.1415926535

double deg2rad(const double deg);
double rad2deg(const double rad);
double distance(Robot *r, Point target);
double normalizeAngle(const double theta);
double myAtan2(const double x , const double y);

# endif

