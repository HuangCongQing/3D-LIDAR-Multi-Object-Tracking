//
// Created by kosuke on 11/26/17.
//




#ifndef EXAMPLE_GAUS_BLUR_H
#define EXAMPLE_GAUS_BLUR_H

#include <vector>
#include <array>
#include "ground_removal.h"

double gauss(double sigma, double x);

std::vector<double> gaussKernel(int samples, double sigma);

void gaussSmoothen(std::array<Cell, numBin>& values, double sigma, int samples);

#endif //TEST1_GAUS_BLUR_H
