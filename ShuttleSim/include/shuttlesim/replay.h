#ifndef SHUTTLESIM_REPLAY_H
#define SHUTTLESIM_REPLAY_H
#include <stdbool.h>
#include <stddef.h>

#define REPLAY_MAX 200000

typedef struct {
    double time_s;
    double aoa_deg;
    double bank_deg;
    bool gear_down;
} ReplayPoint;

typedef struct {
    ReplayPoint *points;
    size_t count;
    size_t cursor;
} AttitudeReplay;

bool replay_load_csv(AttitudeReplay *r, const char *path);
void replay_free(AttitudeReplay *r);
bool replay_sample(AttitudeReplay *r, double time_s, ReplayPoint *out);
#endif
