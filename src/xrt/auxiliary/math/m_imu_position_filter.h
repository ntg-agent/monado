// Copyright 2026, Monado contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Fuses delayed optical position observations with IMU acceleration.
 * @ingroup aux_math
 *
 * For devices whose position comes from optical tracking (for example LED constellation
 * tracked controllers) and whose orientation comes from an IMU fusion. Optical observations
 * are noisy, arrive tens of milliseconds after the camera frame they were taken from, and stop
 * whenever the device is out of view. This filter tracks the position and velocity of the
 * accelerometer with a Kalman filter driven by the gravity-compensated acceleration, and
 * estimates the acceleration bias (from accelerometer calibration and orientation errors) from the
 * observations:
 *
 * - Between observations the position follows the accelerometer, so short gaps are bridged
 *   and the reported position moves smoothly instead of jumping at camera rate.
 * - Observations are applied at the time their camera frame was taken: the filter keeps a
 *   short history of IMU samples and replays the ones that came after.
 * - Observations far outside the expected error are rejected as bad matches, unless several in
 *   a row agree, in which case the filter restarts from them.
 * - Once observations stop, the velocity and the use of the acceleration decay, so the position
 *   settles instead of drifting.
 *
 * All three axes share the same noise model, so they share one 3x3 covariance of position,
 * velocity and bias.
 *
 * Not thread safe: callers serialise access, typically under the device's data lock.
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Number of IMU samples kept for replaying delayed observations.
#define M_IMU_POSITION_FILTER_HISTORY 256

//! Tuning parameters, see m_imu_position_filter_default_params().
struct m_imu_position_filter_params
{
	//! Standard deviation of an optical position observation, in metres.
	float observation_std_m;
	//! Standard deviation of the acceleration error (orientation error, bias, noise), in m/s^2.
	float accel_std_m_s2;
	//! How fast the acceleration bias may change, in m/s^2 per square root second.
	float bias_walk_m_s2_sqrt_s;
	//! How fast the position may wander while unobserved, as a velocity standard deviation in m/s.
	float unobserved_std_m_s;
	//! Gate on the squared Mahalanobis distance of an observation (3 degrees of freedom).
	float gate_chi2;
	//! Restart from the observations after this many rejected ones in a row that agree with each other.
	uint32_t reset_after_rejections;
	//! Rejected observations agree when their offsets from the filter differ by less than this, in metres.
	float reset_agreement_m;
	//! Restart after this many rejected observations in a row, even if they don't agree.
	uint32_t reset_after_any_rejections;
	//! After this long without an accepted observation, the position is no longer tracked.
	float lost_after_s;
	//! Velocity and acceleration decay time constant once lost, in seconds.
	float lost_velocity_tau_s;
	//! Velocity decay time constant while tracked, in seconds, 0 for none.
	float tracked_velocity_tau_s;
};

struct m_imu_position_filter_entry
{
	int64_t timestamp_ns;
	//! Gravity-compensated acceleration in world space, applied from the previous sample to this one.
	double accel[3];
	//! State after this sample: position, velocity, bias, and covariance (see cov in the filter).
	double pos[3];
	double vel[3];
	double bias[3];
	double cov[6];
};

/*!
 * The filter. Initialise with m_imu_position_filter_init(); all fields are internal, but the
 * counters may be shown in debug UIs.
 */
struct m_imu_position_filter
{
	struct m_imu_position_filter_params params;

	//! Position of the accelerometer in the device frame.
	struct xrt_vec3 accel_offset;

	//! Has an observation been accepted since the last reset.
	bool initialized;

	//! Current state, at timestamp_ns.
	int64_t timestamp_ns;
	double pos[3];
	double vel[3];
	//! Estimated acceleration bias in world space, subtracted from the IMU acceleration.
	double bias[3];
	//! Covariance of position, velocity and bias (same for each axis): pp, pv, pb, vv, vb, bb.
	double cov[6];

	//! Ring buffer of past IMU samples and the states after them.
	struct m_imu_position_filter_entry history[M_IMU_POSITION_FILTER_HISTORY];
	uint32_t history_start;
	uint32_t history_count;

	int64_t last_accepted_ns;
	uint32_t rejections_in_row;
	//! Of those, how many in a row agree with each other, and the offset of the last one.
	uint32_t agreeing_rejections;
	double last_rejected_offset[3];

	//! State right after the last accepted observation, at last_accepted_ns.
	struct m_imu_position_filter_entry checkpoint;

	//! Counters for debugging.
	uint64_t accepted_count;
	uint64_t rejected_count;
	uint64_t stale_count;
	uint64_t reset_count;
};

//! Parameters that work for hand-held controllers tracked by cameras on a headset.
struct m_imu_position_filter_params
m_imu_position_filter_default_params(void);

/*!
 * @param params       Tuning parameters, NULL for m_imu_position_filter_default_params().
 * @param accel_offset Position of the accelerometer in the device frame, NULL for the origin.
 */
void
m_imu_position_filter_init(struct m_imu_position_filter *f,
                           const struct m_imu_position_filter_params *params,
                           const struct xrt_vec3 *accel_offset);

//! Forget the position, for example when the device's clock was reset.
void
m_imu_position_filter_reset(struct m_imu_position_filter *f);

/*!
 * Add an IMU sample. Samples must be in increasing time order.
 *
 * @param timestamp_ns  Time of the sample, in the same clock as the observations.
 * @param orientation   Device orientation in world space at that time (from the IMU fusion).
 * @param accel         Accelerometer reading in the device frame, in m/s^2, including gravity
 *                      (reads +9.8 along world +Y when at rest).
 */
void
m_imu_position_filter_push_imu(struct m_imu_position_filter *f,
                               int64_t timestamp_ns,
                               const struct xrt_quat *orientation,
                               const struct xrt_vec3 *accel);

/*!
 * Add an optical observation of the device pose in world space, taken at @p timestamp_ns.
 * The orientation is only used to find the accelerometer's position.
 *
 * @return true if the observation was used, false if it was rejected or too old.
 */
bool
m_imu_position_filter_push_observation(struct m_imu_position_filter *f,
                                       int64_t timestamp_ns,
                                       const struct xrt_pose *pose);

/*!
 * Get the device position and linear velocity in world space at the filter's current time.
 *
 * @param orientation       Current device orientation in world space.
 * @param angular_velocity  Current angular velocity in the device frame, in rad/s.
 * @param[out] out_timestamp_ns Time the position is for (the last IMU sample).
 * @param[out] out_tracked  Whether an observation was accepted recently.
 *
 * @return false if there is no position yet.
 */
bool
m_imu_position_filter_get(const struct m_imu_position_filter *f,
                          const struct xrt_quat *orientation,
                          const struct xrt_vec3 *angular_velocity,
                          struct xrt_vec3 *out_position,
                          struct xrt_vec3 *out_linear_velocity,
                          int64_t *out_timestamp_ns,
                          bool *out_tracked);

#ifdef __cplusplus
}
#endif
