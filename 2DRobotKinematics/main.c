
#include <stdio.h>
# include "controller.h"
# include "trajectory.h"
# include "kinematics.h"

int main(int argc, const char * argv[]) {
    // Custom test data
    Point case8[] = {{2.5, 5.0}, {4.0, 4.5}, {4.0, 3.5}, {1.0, 1.0}, {4.5, 1.0}}; // number "2"
    
    int numberOfPoints = sizeof(case8) / sizeof(Point);
    
    
    Robot r;
    // Test case 8 need to replace start point to {1.0, 4.0}
    robotInit(&r, 5.0, 0.0, 0.0);
    Trajectory *t = trajCreator();
    
    
    double v , omega;
    
    for(int i = 0 ; i < numberOfPoints ; i++){
        while(!moveTo(&r , case8[i] , &v , &omega)){
            step(&r, v, omega);
            trajRecord(t, &r.pose);
            if(t -> index > 300000){
                trajSave(t, "trajectory.csv");
                trajDestroy(t);
                printf("LOST.");
                exit(-1);
            }
        }
    }
    
    printf("%d\n",t -> index);
    trajSave(t, "trajectory.csv");
    trajDestroy(t);
    
    return 0;
}
