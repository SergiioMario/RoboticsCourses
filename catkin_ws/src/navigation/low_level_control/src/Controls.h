#include <iostream>
#include <cmath>
#include <vector>

class Controls
{
public:
    static float robotDiameter;
    static std::vector<float> CalculateSpeeds(float robotX, float robotY, float robotTheta,
                                              float goalX, float goalY, int option = 0);
};
