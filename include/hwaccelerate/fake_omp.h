/* Versions of OMP functions that can be used in sequential version */

#ifndef __FAKE_OMP_H__

int omp_get_max_threads();
int omp_get_num_threads();
int omp_get_thread_num();
void omp_set_num_threads();

#define __FAKE_OMP_H__
#endif /* __FAKE_OMP_H__ */