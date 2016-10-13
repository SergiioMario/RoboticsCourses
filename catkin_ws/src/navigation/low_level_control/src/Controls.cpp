#include "Controls.h"

float Controls::robotDiameter = 0.48;

std::vector<float> Controls::CalculateSpeeds(float robotX, float robotY, float robotTheta,
                                              float goalX, float goalY, int option)
{
    //Constants for the proportional control law
    float Ka  = 0.1;
    float Kd  = 0.3;
    float psi = 4.0;
    //Constants for the v_max control law
    float alpha = 0.2;
    float beta  = 0.2;
    float v_max = 0.1;
    float w_max = 1.5;
    //Physical parameters
    float L = 0.48;
    std::vector<float> speeds;
    speeds.push_back(0.0);
    speeds.push_back(0.0);

    float e_a = atan2(goalY - robotY, goalX - robotX) - robotTheta;
    if(e_a > M_PI) e_a -= 2*M_PI;
    if(e_a <= -M_PI) e_a += 2*M_PI;
    float r = sqrt((goalY - robotY)*(goalY - robotY) +
                   (goalX - robotX)*(goalX - robotX));

    if(option == 0)
    {
        speeds[0] = -Ka*e_a + Kd*r*exp(-psi*e_a*e_a);
        speeds[1] = +Ka*e_a + Kd*r*exp(-psi*e_a*e_a);
    }
    else
    {
        float v = v_max * exp(-e_a*e_a/alpha);
        float w = w_max * (2/(1 + exp(-e_a/beta)) - 1);
        speeds[0] = v - L/2*w;
        speeds[1] = v + L/2*w;
    }
    
    return speeds;
}
