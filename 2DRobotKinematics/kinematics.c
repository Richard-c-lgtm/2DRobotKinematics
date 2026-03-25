# include "kinematics.h"


void robotInit(Robot *r , const double x , const double y , const double theta){
    r -> pose.x = x;
    r -> pose.y = y;
    r -> pose.theta = theta;
}

void diffDrive(const double v , const double omega , double *v_l , double  *v_r){
    // The speed of each tire is derived by inversely using the values ​​of angular velocity and linear velocity.
    // NOTE：This step is only used to simulate the tire speed transmitted to the motor under actual conditions and has no practical use in the current program.
    
    if(*v_l == 0){
        // Spin around the left wheel in place.
        *v_r = v + (omega * WHEEL_BASE / 2);
    }
    else if(*v_r == 0){
        // Spin around the right wheel in place.
        *v_l = v - (omega * WHEEL_BASE / 2);
    }
    else{
        *v_r = v + (omega * WHEEL_BASE / 2);
        *v_l = v - (omega * WHEEL_BASE / 2);
    }
}

static void findCoordinationOfICC(Robot *r , const double v , const double omega , double *O_x , double *O_y){
    // Calculate the center of the ICC for the next step of rotation using the rotation matrix.
    double radius = v / omega;
    *O_x = r -> pose.x - radius * sin(r -> pose.theta);
    *O_y = r -> pose.y + radius * cos(r -> pose.theta);
}

static void rotate(Robot *r , const double v , const double omega){
    double O_x;
    double O_y;
    findCoordinationOfICC(r , v , omega , &O_x , &O_y);
    
    // Construct a temporary coordinate system with point O as the center, and calculate the robot's position in this coordinate system.
    double temp_x = r -> pose.x - O_x;
    double temp_y = r -> pose.y - O_y;
    
    // Calculate the position after rotation in the temporary coordinate system, and then adjust it back to the original coordinate system.
    r -> pose.x =
    (cos(omega * DT) * temp_x)+
    (-sin(omega * DT) * temp_y) + O_x;
    
    r -> pose.y =
    (sin(omega * DT) * temp_x)+
    (cos(omega * DT) * temp_y)+ O_y;
    
    r -> pose.theta += omega * DT;
}


void updatePose(Robot *r , const double v , const double omega){
    // Find the values ​​of x, y, and theta using the positive kinematic value integral.
    if(fabs(omega) < EPSILON){
        // Angle error is too small, no needed to do the extra turn. Therefore here will be linear motion.
        r -> pose.x += v * cos(r -> pose.theta) * DT;
        r -> pose.y += v * sin(r -> pose.theta) * DT;
    }else{
        rotate(r , v , omega);
    }
}

void step(Robot * r , const double v , const double omega){
    // Note：This function, like diffDrive, is only used to simulate real embedded systems and is not needed in this project.
    // These two variables are not used in this program, so they are written in this function.
    double v_l;
    double v_r;
    updatePose(r, v, omega);
    diffDrive(v , omega , &v_l , &v_r);
}
