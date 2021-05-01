#ifndef UTILS_H
#define UTILS_H

#define DEG_TO_RAD(d) (((d)/180.0)*M_PI)
#define RAD_TO_DEG(r) (((r)/M_PI)*180.0)

#define NUM_DATA_POINTS 10 // For offline trajectory computation

// Change these to #define for enabling
#undef OFFLINE_TRAJECTORY_SUPPORT
#undef UNIT_TEST_SUPPORT

#endif // UTILS_H
