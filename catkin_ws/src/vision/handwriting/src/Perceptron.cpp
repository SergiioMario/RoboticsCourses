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
    //TODO: Implement sigmoid function (with threshold)

    return 0;
}
