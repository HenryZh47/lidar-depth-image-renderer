/* Versions of OMP functions that can be used in sequential version */
#include "fake_omp.h"
#include <stdio.h>

#if !OMP

int omp_get_max_threads()
{
    return 1;
}

int omp_get_num_threads()
{
    return 1;
}

int omp_get_thread_num()
{
    return 0;
}

void omp_set_num_threads(int numthreads)
{
    return;
}

#endif