#pragma once

#ifndef FILTER_H
#define FILTER_H

#define M_PIf       3.14159265358979323846f

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
    float alpha;
} pt1Filter_t;

float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dt);

#endif