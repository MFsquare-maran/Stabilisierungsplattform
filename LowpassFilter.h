/*
 * LowpassFilter.h
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include <vector>

/**
 * This class implements a time-discrete lowpass filter of configurable order.
 * The filter is implemented as a cascade of first-order filters and can be used
 * within a periodic task that filters measured values like speed or position.
 */
class LowpassFilter {
    
    public:
        LowpassFilter(unsigned int order = 1);
        virtual ~LowpassFilter();

        void  reset();
        void  reset(float value);
        void  setPeriod(float period);
        void  setFrequency(float frequency);
        float getFrequency();
        float filter(float value);

    private:
        void computeAlpha();

        float period;
        float frequency;
        float alpha;
        unsigned int order;
        std::vector<float> x;
};

#endif /* LOWPASS_FILTER_H_ */
