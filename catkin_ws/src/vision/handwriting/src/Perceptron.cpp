#include "Perceptron.h"

Perceptron::Perceptron(int inputs)
{
    w.resize(inputs);
    for(int i=0; i < w.size(); i++)
        w[i] = 0;
    threshold = 0;
    std::cout << "Perceptron created with " << w.size() << " weights" << std::endl;
}

Perceptron::~Perceptron()
{
}

float Perceptron::eval(unsigned char* data)
{
    float sum = 0;
    for(int i=0;  i < w.size(); i++)
        sum += w[i] * data[i];
    sum -= threshold;

    return 1.0/(1.0 + exp(-sum));
}
