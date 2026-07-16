#ifndef IMU_SIGNAL_FILTER_H
#define IMU_SIGNAL_FILTER_H

/** Advance one scalar first-order low-pass filter. */
float ImuSignalFilter_LowPass(float previous, float input, float alpha);

/** Wrap an angle to the interval (-180, 180]. */
float ImuSignalFilter_WrapAngleDeg(float angle_deg);

#endif
