// Copyright 2026, Monado contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Test the IMU + optical position filter with simulated devices.
 */

#include "catch_amalgamated.hpp"
#include "math/m_api.h"
#include "math/m_imu_position_filter.h"

#include <cmath>
#include <deque>
#include <random>

namespace {

constexpr int64_t kImuPeriodNs = 5'000'000;  // 200 Hz
constexpr int64_t kObsPeriodNs = 16'666'667; // ~60 Hz controller camera frames
constexpr int64_t kObsDelayNs = 30'000'000;  // observations arrive 30 ms after their frame
constexpr double kG = MATH_GRAVITY_M_S2;

struct Truth
{
	xrt_vec3 position;     // device origin, world
	xrt_quat orientation;  // device -> world
	xrt_vec3 accel_device; // accelerometer reading, device frame, including gravity
};

using TruthFn = Truth (*)(double t);

struct Run
{
	double max_error = 0;  // largest error of the filter output after warm-up
	double mean_error = 0; // mean error after warm-up
	double max_step = 0;   // largest change of the output between IMU samples after warm-up
	bool tracked_at_end = false;
	m_imu_position_filter f;
};

/*!
 * Feed a simulated device into the filter. Observations of the device origin (with noise @p
 * obs_noise_m) are delivered @p kObsDelayNs late, except between @p gap_start and @p gap_end
 * seconds. Errors are measured from @p warmup_s seconds on.
 */
Run
simulate(TruthFn truth,
         double duration_s,
         double obs_noise_m,
         double gap_start = -1,
         double gap_end = -1,
         const xrt_vec3 *accel_offset = nullptr,
         double warmup_s = 0.5)
{
	Run r;
	m_imu_position_filter_init(&r.f, nullptr, accel_offset);

	std::mt19937 rng(1234);
	std::normal_distribution<double> noise(0, obs_noise_m);

	struct Pending
	{
		int64_t deliver_ns;
		int64_t frame_ns;
		xrt_pose pose;
	};
	std::deque<Pending> pending;

	int64_t next_obs_ns = 0;
	double error_sum = 0;
	int error_count = 0;
	bool have_last = false;
	xrt_vec3 last = {};

	for (int64_t t_ns = kImuPeriodNs; t_ns <= (int64_t)(duration_s * 1e9); t_ns += kImuPeriodNs) {
		double t = t_ns / 1e9;
		Truth tr = truth(t);

		// Camera frames: queue observations for later delivery.
		while (next_obs_ns <= t_ns) {
			double ft = next_obs_ns / 1e9;
			if (!(ft >= gap_start && ft < gap_end)) {
				Truth ftr = truth(ft);
				xrt_pose p = {ftr.orientation, ftr.position};
				p.position.x += (float)noise(rng);
				p.position.y += (float)noise(rng);
				p.position.z += (float)noise(rng);
				pending.push_back({next_obs_ns + kObsDelayNs, next_obs_ns, p});
			}
			next_obs_ns += kObsPeriodNs;
		}

		m_imu_position_filter_push_imu(&r.f, t_ns, &tr.orientation, &tr.accel_device);

		while (!pending.empty() && pending.front().deliver_ns <= t_ns) {
			m_imu_position_filter_push_observation(&r.f, pending.front().frame_ns, &pending.front().pose);
			pending.pop_front();
		}

		xrt_vec3 pos, vel;
		int64_t ts;
		bool tracked;
		xrt_vec3 w = {0, 0, 0};
		if (!m_imu_position_filter_get(&r.f, &tr.orientation, &w, &pos, &vel, &ts, &tracked)) {
			continue;
		}
		r.tracked_at_end = tracked;

		if (t >= warmup_s) {
			double e = std::sqrt(std::pow(pos.x - tr.position.x, 2) + std::pow(pos.y - tr.position.y, 2) +
			                     std::pow(pos.z - tr.position.z, 2));
			r.max_error = std::max(r.max_error, e);
			error_sum += e;
			error_count++;
			if (have_last) {
				double step = std::sqrt(std::pow(pos.x - last.x, 2) + std::pow(pos.y - last.y, 2) +
				                        std::pow(pos.z - last.z, 2));
				r.max_step = std::max(r.max_step, step);
			}
		}
		last = pos;
		have_last = true;
	}
	r.mean_error = error_count ? error_sum / error_count : 0;
	return r;
}

Truth
still(double)
{
	return {{0.2f, 1.2f, -0.5f}, {0, 0, 0, 1}, {0, (float)kG, 0}};
}

// Swings along X: x = 0.3 sin(2 t), the kind of motion a hand makes.
Truth
swinging(double t)
{
	double x = 0.3 * std::sin(2 * t);
	double ax = -0.3 * 4 * std::sin(2 * t);
	return {{(float)x, 1.2f, -0.5f}, {0, 0, 0, 1}, {(float)ax, (float)kG, 0}};
}

// Spins about its own Y axis at 2 turns per second, with the device origin still. The
// accelerometer sits 5 cm along device X, so it reads the centripetal acceleration.
constexpr double kSpinRadS = 2 * 2 * M_PI;
const xrt_vec3 kSpinOffset = {0.05f, 0, 0};

Truth
spinning(double t)
{
	xrt_quat q;
	xrt_vec3 axis = {0, 1, 0};
	math_quat_from_angle_vector((float)(kSpinRadS * t), &axis, &q);
	// Centripetal acceleration points from the accelerometer to the axis: -w^2 r, in the device frame.
	float a = (float)(kSpinRadS * kSpinRadS * kSpinOffset.x);
	return {{0.2f, 1.2f, -0.5f}, q, {-a, (float)kG, 0}};
}

} // namespace

TEST_CASE("Still device: output is smoother than the observations")
{
	// With the noise the filter expects (sigma per axis), the raw observations are off by 1.6
	// sigma on average and jump by about 2.3 sigma from one to the next.
	const double sigma = m_imu_position_filter_default_params().observation_std_m;
	Run r = simulate(still, 3.0, sigma);
	CHECK(r.mean_error < 1.0 * sigma);
	CHECK(r.max_error < 2.5 * sigma);
	CHECK(r.max_step < 1.5 * sigma);
	CHECK(r.tracked_at_end);
	CHECK(r.f.rejected_count == 0);
}

TEST_CASE("Moving device: delayed observations are applied at their frame time")
{
	Run r = simulate(swinging, 4.0, 0.004);
	// Using the observations at arrival time would lag 30 ms behind, up to 1.8 cm at 0.6 m/s.
	CHECK(r.mean_error < 0.005);
	CHECK(r.max_error < 0.012);
	CHECK(r.f.stale_count == 0);
}

TEST_CASE("Moving device: the IMU bridges a gap in the observations")
{
	// No camera observations for 300 ms in the middle of the swing.
	Run r = simulate(swinging, 4.0, 0.004, 2.0, 2.3);
	CHECK(r.max_error < 0.03);
	CHECK(r.tracked_at_end);
	CHECK(r.f.reset_count == 0);
}

TEST_CASE("Lost device: the position settles instead of drifting away")
{
	// A biased accelerometer (0.3 m/s^2, as from a 1.8 degree tilt error) and no observations
	// after 1 s. Without phasing out the acceleration this drifts by metres.
	Run r = simulate(
	    [](double t) {
		    Truth tr = still(t);
		    tr.accel_device.x += 0.3f;
		    return tr;
	    },
	    6.0, 0.004, 1.0, 100.0);
	CHECK_FALSE(r.tracked_at_end);
	CHECK(r.max_error < 0.08);
}

TEST_CASE("Spinning device: the accelerometer offset is accounted for")
{
	Run r = simulate(spinning, 3.0, 0.004, -1, -1, &kSpinOffset);
	CHECK(r.mean_error < 0.005);
	CHECK(r.max_error < 0.012);
}

TEST_CASE("A single bad observation is rejected, repeated ones restart the filter")
{
	m_imu_position_filter f;
	m_imu_position_filter_init(&f, nullptr, nullptr);
	Truth tr = still(0);
	xrt_pose good = {tr.orientation, tr.position};
	xrt_pose bad = good;
	bad.position.x += 0.5f;

	int64_t t = 0;
	auto step = [&](const xrt_pose *obs) {
		for (int i = 0; i < 4; i++) {
			t += kImuPeriodNs;
			m_imu_position_filter_push_imu(&f, t, &tr.orientation, &tr.accel_device);
		}
		return obs ? m_imu_position_filter_push_observation(&f, t - kImuPeriodNs, obs) : true;
	};
	for (int i = 0; i < 30; i++) {
		REQUIRE(step(&good));
	}

	xrt_vec3 pos, vel, w = {0, 0, 0};
	int64_t ts;
	bool tracked;

	CHECK_FALSE(step(&bad));
	REQUIRE(m_imu_position_filter_get(&f, &tr.orientation, &w, &pos, &vel, &ts, &tracked));
	CHECK(std::fabs(pos.x - good.position.x) < 0.005);
	CHECK(f.rejected_count == 1);

	CHECK(step(&good));
	CHECK(f.rejections_in_row == 0);

	// The device really moved: after a few observations agree, the filter follows.
	for (uint32_t i = 0; i < f.params.reset_after_rejections; i++) {
		step(&bad);
	}
	REQUIRE(m_imu_position_filter_get(&f, &tr.orientation, &w, &pos, &vel, &ts, &tracked));
	CHECK(std::fabs(pos.x - bad.position.x) < 0.005);
	CHECK(f.reset_count == 1);
}

TEST_CASE("Bad observations that disagree with each other don't restart the filter")
{
	m_imu_position_filter f;
	m_imu_position_filter_init(&f, nullptr, nullptr);
	Truth tr = still(0);
	xrt_pose good = {tr.orientation, tr.position};

	int64_t t = 0;
	auto step = [&](const xrt_pose *obs) {
		for (int i = 0; i < 4; i++) {
			t += kImuPeriodNs;
			m_imu_position_filter_push_imu(&f, t, &tr.orientation, &tr.accel_device);
		}
		return m_imu_position_filter_push_observation(&f, t - kImuPeriodNs, obs);
	};
	for (int i = 0; i < 30; i++) {
		REQUIRE(step(&good));
	}

	// Wrong matches scattered around the device, as when the LEDs of two controllers mix.
	const float offsets[][3] = {{0.1f, 0, 0}, {0, 0, -0.12f}, {-0.08f, 0.05f, 0}, {0, -0.1f, 0.06f}};
	for (uint32_t i = 0; i + 1 < f.params.reset_after_any_rejections; i++) {
		xrt_pose bad = good;
		bad.position.x += offsets[i % 4][0];
		bad.position.y += offsets[i % 4][1];
		bad.position.z += offsets[i % 4][2];
		CHECK_FALSE(step(&bad));
	}
	CHECK(f.reset_count == 0);

	xrt_vec3 pos, vel, w = {0, 0, 0};
	int64_t ts;
	bool tracked;
	CHECK(step(&good));
	REQUIRE(m_imu_position_filter_get(&f, &tr.orientation, &w, &pos, &vel, &ts, &tracked));
	CHECK(std::fabs(pos.x - good.position.x) < 0.005);
	CHECK(std::fabs(vel.x) < 0.05);
	CHECK(std::fabs(vel.z) < 0.05);
}

TEST_CASE("Two observations between the same IMU samples are both used")
{
	m_imu_position_filter f;
	m_imu_position_filter_init(&f, nullptr, nullptr);
	Truth tr = still(0);
	xrt_pose obs = {tr.orientation, tr.position};

	for (int64_t t = kImuPeriodNs; t <= 100 * kImuPeriodNs; t += kImuPeriodNs) {
		m_imu_position_filter_push_imu(&f, t, &tr.orientation, &tr.accel_device);
	}
	int64_t base = 50 * kImuPeriodNs;
	CHECK(m_imu_position_filter_push_observation(&f, base + 1'000'000, &obs));
	CHECK(m_imu_position_filter_push_observation(&f, base + 2'000'000, &obs));
	CHECK(f.accepted_count == 2);
	// Older than one already used.
	CHECK_FALSE(m_imu_position_filter_push_observation(&f, base + 1'500'000, &obs));
	CHECK(f.stale_count == 1);
}
