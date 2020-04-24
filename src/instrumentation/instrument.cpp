#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if OMP
#include <omp.h>
#else
#include "fake_omp.h"
#endif

#include "cycletimer.h"
#include "instrument.h"

#define MAX_THREAD 64

/* Instrument different sections of program */
static const char *activity_name[ACTIVITY_COUNT] = {"overhead", "startup", "render"};

static bool tracking = false;
static double global_start_time = 0.0;

static activity_t current_activity = ACTIVITY_OVERHEAD;
static double current_start_time = 0.0;
static double accum[MAX_THREAD][ACTIVITY_COUNT];
static double global_accum[ACTIVITY_COUNT];

void track_activity(bool enable)
{
    tracking = enable;
    global_start_time = currentSeconds();
    current_start_time = global_start_time;
    memset(accum, 0, ACTIVITY_COUNT * MAX_THREAD * sizeof(double));
    memset(global_accum, 0, ACTIVITY_COUNT * sizeof(double));
    current_activity = ACTIVITY_OVERHEAD;
}

void start_activity(activity_t a)
{
    if (!tracking)
        return;
    current_start_time = currentSeconds();
    current_activity = a;
}

void finish_activity(activity_t a)
{
    if (!tracking)
        return;
    double new_time = currentSeconds();
    if (a != current_activity)
    {
        fprintf(stderr, "Warning.  Started activity %s, but now finishing global activity %s\n",
                activity_name[current_activity], activity_name[a]);
    }
    global_accum[current_activity] += (new_time - current_start_time);
    current_activity = ACTIVITY_OVERHEAD;
}

void show_activity(FILE *f, bool enable)
{
    if (!enable)
        return;
    fprintf(f, "-----------------------------------------------\n");
    double elapsed = currentSeconds() - global_start_time;
    int a;
    double unknown = elapsed;
    for (a = 1; a < (int)ACTIVITY_COUNT; a++)
    {
        if (global_accum[a] == 0.0)
            continue;
        unknown -= global_accum[a];
        double ms = global_accum[a] * 1000.0;
        double pct = global_accum[a] / elapsed * 100.0;
        fprintf(f, "    %8d ms    %5.1f %%    %s\n", (int)ms, pct, activity_name[a]);
    }
    double ums = unknown * 1000.0;
    double upct = unknown / elapsed * 100.0;
    fprintf(f, "    %8d ms    %5.1f %%    unknown\n", (int)ums, upct);
    fprintf(f, "    %8d ms    %5.1f %%    elapsed\n", (int)(elapsed * 1000.0), 100.0);
}
