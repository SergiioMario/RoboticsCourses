#include <iostream>
#include <vector>
#include <cmath>

class Perceptron
{
public:
    Perceptron(int inputs);
    ~Perceptron();

    std::vector<float> w;
    float threshold;
    float eval(unsigned char* data);
};
