// Used to store functions related to robot motion.

# ifndef KINEMATICS_H
# define KINEMATICS_H
# include "robot.h"
# include <stdlib.h>
# include <stdio.h>
# include <math.h>


void robotInit(Robot *r , const double x , const double y , const double theta);
void diffDrive(const double v , const double omega , double *v_l , double  *v_r);
void updatePose(Robot *r , const double v , const double omega);
void step(Robot *r , const double v , const double omega);

#endif
