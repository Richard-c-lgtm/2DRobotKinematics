
# ifndef TRAJECTORY_H
# define TRAJECTORY_H
# include "robot.h"
# include <stdlib.h>
#include <stdio.h>

typedef struct{
    int index;
    Pose *pose;
}Trajectory;

Trajectory * trajCreator(void);
void trajRecord(Trajectory *t, const Pose *p);
void trajSave(const Trajectory *t, const char *filepath);
void trajDestroy(Trajectory *t);

# endif
