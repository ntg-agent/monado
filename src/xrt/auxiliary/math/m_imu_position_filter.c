// Copyright 2026, Monado contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Fuses delayed optical position observations with IMU acceleration.
 * @ingroup aux_math
 */

#include "m_imu_position_filter.h"

#include "math/m_api.h"

#include <math.h>
#include <string.h>


/*
 *
 * Helpers.
 *
 */

static struct m_imu_position_filter_entry *
history_at(struct m_imu_position_filter *f, uint32_t i)
{
	return &f->history[(f->history_start + i) % M_IMU_POSITION_FILTER_HISTORY];
}

//! Index of the newest history entry at or before @p timestamp_ns, or -1 if there is none.
static int32_t
history_find(struct m_imu_position_filter *f, int64_t timestamp_ns)
{
	for (int32_t i = (int32_t)f->history_count - 1; i >= 0; i--) {
		if (history_at(f, (uint32_t)i)->timestamp_ns <= timestamp_ns) {
			return i;
		}
	}
	return -1;
}

static void
history_push(struct m_imu_position_filter *f, const struct m_imu_position_filter_entry *e)
{
	if (f->history_count < M_IMU_POSITION_FILTER_HISTORY) {
		f->history_count++;
	} else {
		f->history_start = (f->history_start + 1) % M_IMU_POSITION_FILTER_HISTORY;
	}
	*history_at(f, f->history_count - 1) = *e;
}

//! Unpack the symmetric covariance pp, pv, pb, vv, vb, bb into a full matrix.
static void
cov_to_matrix(const double c[6], double m[3][3])
{
	m[0][0] = c[0];
	m[0][1] = m[1][0] = c[1];
	m[0][2] = m[2][0] = c[2];
	m[1][1] = c[3];
	m[1][2] = m[2][1] = c[4];
	m[2][2] = c[5];
}

static void
matrix_to_cov(double m[3][3], double c[6])
{
	c[0] = m[0][0];
	c[1] = m[0][1];
	c[2] = m[0][2];
	c[3] = m[1][1];
	c[4] = m[1][2];
	c[5] = m[2][2];
}

//! Move the current state forward by @p dt_s seconds with constant acceleration @p accel.
static void
integrate(struct m_imu_position_filter *f, const double accel[3], double dt_s, int64_t at_ns)
{
	if (dt_s <= 0) {
		return;
	}

	const struct m_imu_position_filter_params *p = &f->params;
	double dt2 = dt_s * dt_s;

	/* Once unobserved, phase out the acceleration as well as the velocity: any remaining bias
	 * would otherwise keep the position moving at a constant rate. */
	double lost_s = (double)(at_ns - f->last_accepted_ns) / 1e9 - p->lost_after_s;
	bool lost = lost_s > 0;
	double k = lost && p->lost_velocity_tau_s > 0 ? exp(-lost_s / p->lost_velocity_tau_s) : 1.0;

	for (int i = 0; i < 3; i++) {
		double a = (accel[i] - f->bias[i]) * k;
		f->pos[i] += f->vel[i] * dt_s + 0.5 * a * dt2;
		f->vel[i] += a * dt_s;
	}

	// P = F P F^T + Q, F = [1 dt -k dt^2/2; 0 1 -k dt; 0 0 1] for the state (position, velocity, bias).
	double F[3][3] = {{1, dt_s, -k * dt2 / 2}, {0, 1, -k * dt_s}, {0, 0, 1}};
	double P[3][3], FP[3][3], N[3][3];
	cov_to_matrix(f->cov, P);
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			FP[r][c] = F[r][0] * P[0][c] + F[r][1] * P[1][c] + F[r][2] * P[2][c];
		}
	}
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			N[r][c] = FP[r][0] * F[c][0] + FP[r][1] * F[c][1] + FP[r][2] * F[c][2];
		}
	}

	// White acceleration noise, and a slowly wandering bias.
	double sa = (double)p->accel_std_m_s2 * p->accel_std_m_s2;
	N[0][0] += sa * dt2 * dt2 / 4;
	N[0][1] += sa * dt2 * dt_s / 2;
	N[1][0] += sa * dt2 * dt_s / 2;
	N[1][1] += sa * dt2;
	N[2][2] += (double)p->bias_walk_m_s2_sqrt_s * p->bias_walk_m_s2_sqrt_s * dt_s;

	// Once unobserved, the device may go anywhere: let the position uncertainty grow, and let the
	// velocity decay so the position settles.
	if (lost) {
		N[0][0] += (double)p->unobserved_std_m_s * p->unobserved_std_m_s * dt_s;
	}
	double tau = lost ? p->lost_velocity_tau_s : p->tracked_velocity_tau_s;
	if (tau > 0) {
		double d = exp(-dt_s / tau);
		for (int i = 0; i < 3; i++) {
			f->vel[i] *= d;
		}
		for (int i = 0; i < 3; i++) {
			N[1][i] *= d;
			N[i][1] *= d;
		}
	}

	// Keep the uncertainties bounded, so a long time out of view doesn't make the gate meaningless.
	const double max_var[3] = {1.0, 4.0, 1.0};
	for (int i = 0; i < 3; i++) {
		if (N[i][i] > max_var[i]) {
			double sc = sqrt(max_var[i] / N[i][i]);
			for (int j = 0; j < 3; j++) {
				N[i][j] *= sc;
				N[j][i] *= sc;
			}
		}
	}

	matrix_to_cov(N, f->cov);
}

static void
save_state(const struct m_imu_position_filter *f, struct m_imu_position_filter_entry *e)
{
	memcpy(e->pos, f->pos, sizeof(e->pos));
	memcpy(e->vel, f->vel, sizeof(e->vel));
	memcpy(e->bias, f->bias, sizeof(e->bias));
	memcpy(e->cov, f->cov, sizeof(e->cov));
}

static void
load_state(struct m_imu_position_filter *f, const struct m_imu_position_filter_entry *e)
{
	memcpy(f->pos, e->pos, sizeof(f->pos));
	memcpy(f->vel, e->vel, sizeof(f->vel));
	memcpy(f->bias, e->bias, sizeof(f->bias));
	memcpy(f->cov, e->cov, sizeof(f->cov));
	f->timestamp_ns = e->timestamp_ns;
}

/*!
 * The state is at @p from_ns, which lies between history entries @p index and @p index + 1.
 * Integrate the IMU samples after it, saving the state after each one, up to the newest.
 */
static void
replay_from(struct m_imu_position_filter *f, int32_t index, int64_t from_ns)
{
	int64_t t = from_ns;
	for (uint32_t i = (uint32_t)(index + 1); i < f->history_count; i++) {
		struct m_imu_position_filter_entry *e = history_at(f, i);
		integrate(f, e->accel, (double)(e->timestamp_ns - t) / 1e9, e->timestamp_ns);
		t = e->timestamp_ns;
		save_state(f, e);
	}
	f->timestamp_ns = t;
}

//! Position of the accelerometer in world space, for a device at @p pose.
static void
accel_position(const struct m_imu_position_filter *f, const struct xrt_pose *pose, double out[3])
{
	struct xrt_vec3 offset;
	math_quat_rotate_vec3(&pose->orientation, &f->accel_offset, &offset);
	out[0] = (double)pose->position.x + offset.x;
	out[1] = (double)pose->position.y + offset.y;
	out[2] = (double)pose->position.z + offset.z;
}

//! Start over from an observation of the accelerometer at @p z, taken at @p timestamp_ns.
static void
start_from(struct m_imu_position_filter *f, int64_t timestamp_ns, const double z[3])
{
	const double initial_vel_std = 1.0;
	const double initial_bias_std = 0.5;
	double obs_var = (double)f->params.observation_std_m * f->params.observation_std_m;

	// Keep the bias estimate across restarts, it changes slowly.
	for (int i = 0; i < 3; i++) {
		f->pos[i] = z[i];
		f->vel[i] = 0;
	}
	double bias_var = f->cov[5] > 0 ? f->cov[5] : initial_bias_std * initial_bias_std;
	memset(f->cov, 0, sizeof(f->cov));
	f->cov[0] = obs_var;
	f->cov[3] = initial_vel_std * initial_vel_std;
	f->cov[5] = bias_var;

	f->initialized = true;
	f->last_accepted_ns = timestamp_ns;
	f->rejections_in_row = 0;
	f->agreeing_rejections = 0;
	f->checkpoint.timestamp_ns = timestamp_ns;
	save_state(f, &f->checkpoint);

	replay_from(f, history_find(f, timestamp_ns), timestamp_ns);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct m_imu_position_filter_params
m_imu_position_filter_default_params(void)
{
	struct m_imu_position_filter_params p = {
	    .observation_std_m = 0.005f,
	    .accel_std_m_s2 = 0.7f,
	    .bias_walk_m_s2_sqrt_s = 0.1f,
	    .unobserved_std_m_s = 0.5f,
	    .gate_chi2 = 25.0f, // well beyond noise (99.9% for 3 degrees of freedom is 16.3)
	    .reset_after_rejections = 4,
	    .reset_agreement_m = 0.03f,
	    .reset_after_any_rejections = 10,
	    .lost_after_s = 0.25f,
	    .lost_velocity_tau_s = 0.15f,
	    .tracked_velocity_tau_s = 0.0f,
	};
	return p;
}

void
m_imu_position_filter_init(struct m_imu_position_filter *f,
                           const struct m_imu_position_filter_params *params,
                           const struct xrt_vec3 *accel_offset)
{
	memset(f, 0, sizeof(*f));
	f->params = params != NULL ? *params : m_imu_position_filter_default_params();
	f->accel_offset = accel_offset != NULL ? *accel_offset : (struct xrt_vec3){0, 0, 0};
}

void
m_imu_position_filter_reset(struct m_imu_position_filter *f)
{
	f->initialized = false;
	f->timestamp_ns = 0;
	f->history_start = 0;
	f->history_count = 0;
	f->rejections_in_row = 0;
	f->reset_count++;
}

void
m_imu_position_filter_push_imu(struct m_imu_position_filter *f,
                               int64_t timestamp_ns,
                               const struct xrt_quat *orientation,
                               const struct xrt_vec3 *accel)
{
	if (f->history_count > 0 && timestamp_ns <= history_at(f, f->history_count - 1)->timestamp_ns) {
		return; // Out of order, ignore
	}

	struct xrt_vec3 accel_world;
	math_quat_rotate_vec3(orientation, accel, &accel_world);

	struct m_imu_position_filter_entry e = {.timestamp_ns = timestamp_ns};
	e.accel[0] = accel_world.x;
	e.accel[1] = (double)accel_world.y - MATH_GRAVITY_M_S2;
	e.accel[2] = accel_world.z;

	if (f->initialized) {
		integrate(f, e.accel, (double)(timestamp_ns - f->timestamp_ns) / 1e9, timestamp_ns);
		save_state(f, &e);
	}
	f->timestamp_ns = timestamp_ns;

	history_push(f, &e);
}

bool
m_imu_position_filter_push_observation(struct m_imu_position_filter *f,
                                       int64_t timestamp_ns,
                                       const struct xrt_pose *pose)
{
	double z[3];
	accel_position(f, pose, z);

	// An observation newer than the newest IMU sample is applied at that sample, a few ms early.
	if (f->history_count > 0) {
		int64_t newest_ns = history_at(f, f->history_count - 1)->timestamp_ns;
		if (timestamp_ns > newest_ns) {
			timestamp_ns = newest_ns;
		}
	}

	if (!f->initialized) {
		start_from(f, timestamp_ns, z);
		f->accepted_count++;
		return true;
	}

	// Older than an observation already applied (the replay would undo that one), or older
	// than the history.
	int32_t index = history_find(f, timestamp_ns);
	if (timestamp_ns <= f->last_accepted_ns || index < 0) {
		f->stale_count++;
		return false;
	}

	// Rewind to the time of the observation. If the last accepted observation lies between
	// that history entry and this observation, start from the state right after it instead.
	struct m_imu_position_filter_entry current = {.timestamp_ns = f->timestamp_ns};
	save_state(f, &current);

	const struct m_imu_position_filter_entry *from = history_at(f, (uint32_t)index);
	if (from->timestamp_ns < f->checkpoint.timestamp_ns) {
		from = &f->checkpoint;
	}
	load_state(f, from);
	if ((uint32_t)index + 1 < f->history_count) {
		integrate(f, history_at(f, (uint32_t)index + 1)->accel,
		          (double)(timestamp_ns - from->timestamp_ns) / 1e9, timestamp_ns);
	}

	double obs_var = (double)f->params.observation_std_m * f->params.observation_std_m;
	double s = f->cov[0] + obs_var;
	double y[3];
	double d2 = 0;
	for (int i = 0; i < 3; i++) {
		y[i] = z[i] - f->pos[i];
		d2 += y[i] * y[i] / s;
	}

	if (d2 > f->params.gate_chi2) {
		f->rejected_count++;
		f->rejections_in_row++;

		/* Rejected observations that are all off from the filter by about the same amount mean
		 * the filter is what's wrong. Ones that are off by different amounts are bad matches,
		 * and restarting from the latest of those would pick a wrong position and, from the
		 * next good observation, a wrong velocity. */
		double dy2 = 0;
		for (int i = 0; i < 3; i++) {
			double dy = y[i] - f->last_rejected_offset[i];
			dy2 += dy * dy;
			f->last_rejected_offset[i] = y[i];
		}
		double agreement = f->params.reset_agreement_m;
		if (f->rejections_in_row > 1 && dy2 < agreement * agreement) {
			f->agreeing_rejections++;
		} else {
			f->agreeing_rejections = 1;
		}

		if (f->agreeing_rejections < f->params.reset_after_rejections &&
		    f->rejections_in_row < f->params.reset_after_any_rejections) {
			load_state(f, &current);
			return false;
		}

		// Several in a row disagree with the filter, so the filter is what's wrong.
		f->reset_count++;
		start_from(f, timestamp_ns, z);
		f->accepted_count++;
		return true;
	}

	// Kalman update, H = [1 0 0]: K = P H^T / s, P -= K H P.
	double P[3][3];
	cov_to_matrix(f->cov, P);
	double K[3] = {P[0][0] / s, P[1][0] / s, P[2][0] / s};
	for (int i = 0; i < 3; i++) {
		f->pos[i] += K[0] * y[i];
		f->vel[i] += K[1] * y[i];
		f->bias[i] += K[2] * y[i];
	}
	double row0[3] = {P[0][0], P[0][1], P[0][2]};
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			P[r][c] -= K[r] * row0[c];
		}
	}
	matrix_to_cov(P, f->cov);

	f->last_accepted_ns = timestamp_ns;
	f->rejections_in_row = 0;
	f->accepted_count++;
	f->checkpoint.timestamp_ns = timestamp_ns;
	save_state(f, &f->checkpoint);

	replay_from(f, index, timestamp_ns);
	return true;
}

bool
m_imu_position_filter_get(const struct m_imu_position_filter *f,
                          const struct xrt_quat *orientation,
                          const struct xrt_vec3 *angular_velocity,
                          struct xrt_vec3 *out_position,
                          struct xrt_vec3 *out_linear_velocity,
                          int64_t *out_timestamp_ns,
                          bool *out_tracked)
{
	if (!f->initialized) {
		return false;
	}

	// The filter tracks the accelerometer, move back to the device origin.
	struct xrt_vec3 offset_world;
	math_quat_rotate_vec3(orientation, &f->accel_offset, &offset_world);
	out_position->x = (float)f->pos[0] - offset_world.x;
	out_position->y = (float)f->pos[1] - offset_world.y;
	out_position->z = (float)f->pos[2] - offset_world.z;

	// v_origin = v_accel - w x r, with both in world space.
	struct xrt_vec3 w_world;
	math_quat_rotate_vec3(orientation, angular_velocity, &w_world);
	struct xrt_vec3 w_cross_r;
	math_vec3_cross(&w_world, &offset_world, &w_cross_r);
	out_linear_velocity->x = (float)f->vel[0] - w_cross_r.x;
	out_linear_velocity->y = (float)f->vel[1] - w_cross_r.y;
	out_linear_velocity->z = (float)f->vel[2] - w_cross_r.z;

	*out_timestamp_ns = f->timestamp_ns;
	*out_tracked = (double)(f->timestamp_ns - f->last_accepted_ns) <= (double)f->params.lost_after_s * 1e9;

	return true;
}
