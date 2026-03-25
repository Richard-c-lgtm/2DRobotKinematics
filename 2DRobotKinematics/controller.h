
# ifndef CONTROLLER_H
# define CONTROLLER_H
# include "robot.h"
# include "kinematics.h"
# include "utils.h"

// Trajectory adjustment towards a single target.
int moveTo(Robot *r , Point goal , double *v , double *omega);

# endif
