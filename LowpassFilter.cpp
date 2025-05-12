#include <cmath>
#include <vector>
#include "LowpassFilter.h"

using namespace std;

LowpassFilter::LowpassFilter(unsigned int order) {
    this->order = order;
    this->period = 1.0f;
    this->frequency = 1000.0f;
    x.assign(order, 0.0f); // alle Zustände auf 0 setzen
    alpha = 0.0f;
    computeAlpha();
}

LowpassFilter::~LowpassFilter() {}

void LowpassFilter::reset() {
    fill(x.begin(), x.end(), 0.0f);
}

void LowpassFilter::reset(float value) {
    fill(x.begin(), x.end(), value);
}

void LowpassFilter::setPeriod(float period) {
    this->period = period;
    computeAlpha();
}

void LowpassFilter::setFrequency(float frequency) {
    this->frequency = frequency;
    computeAlpha();
}

float LowpassFilter::getFrequency() {
    return frequency;
}

void LowpassFilter::computeAlpha() {
    float tau = 1.0f / frequency;
    alpha = period / (tau + period);
}

float LowpassFilter::filter(float value) {
    x[0] += alpha * (value - x[0]);
    for (unsigned int i = 1; i < order; ++i) {
        x[i] += alpha * (x[i - 1] - x[i]);
    }
    return x[order - 1];
}
