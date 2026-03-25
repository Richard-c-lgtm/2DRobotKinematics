# include "trajectory.h"

Trajectory * trajCreator(void){
    Trajectory *t = malloc(sizeof(Trajectory));
    if(t == NULL){
        return NULL;
    }
    t -> pose = malloc(300000 * sizeof(Pose));
    if(t -> pose == NULL){
        return NULL;
    }
    t -> index = 0;
    return t;
}

void trajRecord(Trajectory *t , const Pose *p){
    t -> pose[t -> index] = *p;
    t -> index++;
}

void trajSave(const Trajectory *t , const char *filePath){
    FILE *f = fopen(filePath, "w");
    
    fprintf(f, "x,y,theta\n");
    for(int i = 0 ; i < t -> index ; i++){
        fprintf(f, "%.17g,%.17g,%.17g\n", t->pose[i].x,t->pose[i].y,t->pose[i].theta);
    }
    fclose(f);
}

void trajDestroy(Trajectory *t){
    free(t -> pose);
    t -> pose = NULL;
    free(t);
}



