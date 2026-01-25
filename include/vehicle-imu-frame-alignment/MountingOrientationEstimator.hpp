#pragma once

/**
 * @file MountingCalibrator.h
 *
 * @brief GNSS-free online calibration of IMU mounting angles
 *
 * Based on:
 * Senofieni, R., Corno, M., Strada, S. C., Savaresi, S. M., & Fredella, S. (2023)
 * "GNSS-free Online Calibration of Inertial Measurement Units in Road Vehicles"
 * IEEE Sensors Journal
 *
 * Author:  Oğuzhan Türk
 * Contact: stkyoht@hotmail.com
 *
 * Notes:
 * - Inputs: accelerometer [g], gyroscope [rad/s]
 * - Designed for ground vehicles
 */

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include <cmath>

// ---------- small helpers ----------
struct Vec3 {
	float x, y, z;
};
struct Mat3 {
	float m[3][3];
	Vec3 mul(const Vec3& v) const
	{
		return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
				m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
	}
};

// ---------- IIR filters from paper ----------
// First-order low-pass: y(k) = alpha * u(k) + (1-alpha) * y(k-1)
struct Lowpass {
	float alpha; // computed from fcut and dt
	float y;
	bool initialized = false;
	Lowpass() : alpha(1.0f), y(0.0f)
	{
	}
	void init(float fcut, float dt)
	{
		float tau = 1.0f / (2.0f * M_PI * fcut);
		// Derived alpha equivalent to the paper formula:
		alpha = (2.0f * M_PI * dt * fcut) / (2.0f * M_PI * dt * fcut + 1.0f);
		// if fcut very small result alpha approximates small
	}
	float apply(float u)
	{
		if (!initialized) {
			y = u;
			initialized = true;
			return y;
		}
		y = alpha * u + (1.0f - alpha) * y;
		return y;
	}
	void reset()
	{
		initialized = false;
		y = 0.0f;
	}
};

// First-order high-pass: y(k) = beta*y(k-1) + beta*(u(k)-u(k-1))
struct Highpass {
	float beta;
	float y;
	float u_prev;
	bool initialized = false;
	Highpass() : beta(1.0f), y(0.0f), u_prev(0.0f)
	{
	}
	void init(float fcut, float dt)
	{
		beta = 1.0f / (2.0f * M_PI * dt * fcut + 1.0f);
	}
	float apply(float u)
	{
		if (!initialized) {
			u_prev = u;
			y = u;
			initialized = true;
			u_prev = u;
			return y;
		}
		y = beta * y + beta * (u - u_prev);
		u_prev = u;
		return y;
	}
	void reset()
	{
		initialized = false;
		y = 0.0f;
		u_prev = 0.0f;
	}
};

// ---------- Simple scalar RLS for model y = m * x (no intercept) ----------
// Recursive Least Squares with forgetting factor lambda (paper uses mu in [0,1) with cost sum mu^(k-i) -> lambda=mu)
struct ScalarRLS {
	float lambda; // forgetting factor in (0,1), smaller -> fast forgetting
	float P;	  // covariance scalar
	float m;	  // slope estimate
	bool initialized = false;
	ScalarRLS(float lambda_ = 0.995f, float P0 = 1e6f) : lambda(lambda_), P(P0), m(0.0f)
	{
	}
	void reset(float P0 = 1e6f)
	{
		P = P0;
		m = 0.0f;
		initialized = true;
	}
	// update with one sample (x,y)
	void update(float x, float y)
	{
		// avoid degenerate x
		float denom = lambda + x * P * x;
		float K = (P * x) / denom; // scalar gain
		float err = y - x * m;
		m = m + K * err;
		P = (1.0f / lambda) * (P - K * x * P);
	}
	float predict(float x) const
	{
		return m * x;
	}
};

// ---------- Calibration class implementing the pipeline ----------
class MountingCalibrator {
public:
	// Parameters (tunable)
	float downsampleHz = 13.0f;		// paper downsamples to 13 Hz
	float fcut_pre_lp = 1.4f;		// low-pass for acc/gyro preprocessing (Hz)
	float fcut_angle_lp = 1.0f;		// LP for roll/pitch angle smoothing
	float fcut_gyro_bias = 0.001f;	// LP for gyro bias estimation (very low)
	float g = 1.0f;					// gravitational magnitude in g units (1g)
	float dg_th = 0.15f;			// tolerance for |a| ~ g to select gravity samples
	float omega_mod_hp_fcut = 0.1f; // HPF for gyro norm to detect stillness (Hz)
	float omega_threshold = 0.02f;	// threshold for stationary detection (rad/s)
	// Yaw selection thresholds (paper tunings)
	float omega_z_th = 0.1f;  // small yaw rate threshold for "not turning" [rad/s]
	float a_xy_th = 0.02f;	  // planar acceleration threshold for selecting straight accel [g]
	float omega_xy_th = 0.2f; // threshold for roll/pitch dynamics limited
	// Travel direction recognition thresholds
	float gamma_lp_fcut = 0.5f;
	float gamma_threshold = 0.6f; // if filtered gamma passes this => direction recognized
	// Convergence HPF thresholds for angles (paper uses HPF then threshold)
	float angle_hpf_fcut = 0.01f;
	float angle_conv_th = 0.2f * (M_PI / 180.0f); // 0.2 deg in rad

	// RLS params
	float rls_lambda = 0.995f; // forgetting factor

	// State
	float input_period = 0.0f; // dt used for incoming samples
	int decimation = 1;
	int decim_count = 0;

	// Filters and helpers
	Lowpass lp_acc_x, lp_acc_y, lp_acc_z;
	Lowpass lp_gyro_x, lp_gyro_y, lp_gyro_z;
	Lowpass lp_angle_roll, lp_angle_pitch;
	Lowpass lp_gamma0, lp_gamma180;
	Lowpass lp_gyro_bias_x, lp_gyro_bias_y, lp_gyro_bias_z;

	Highpass hp_omega_mod;
	Highpass hp_angle_roll, hp_angle_pitch, hp_yaw;
	Highpass hp_gamma0, hp_gamma180;

	ScalarRLS rls_nominal, rls_rotated;

	// outputs
	float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // radians
	bool flagRollPitchConverged = false;
	bool flagYawConverged = false;
	bool flagDirectionConverged = false;
	bool flagCalibrated = false;

	MountingCalibrator(float fs_in = 104.0f) : rls_nominal(rls_lambda), rls_rotated(rls_lambda)
	{
		// initialize filters with dt from input sampling rate (only for alpha calculations)
		input_period = 1.0f / fs_in;
		// compute decimation to reach downsampleHz
		if (fs_in > downsampleHz)
			decimation = std::max(1, int(std::round(fs_in / downsampleHz)));
		else
			decimation = 1;

		// init filters
		lp_acc_x.init(fcut_pre_lp, input_period);
		lp_acc_y.init(fcut_pre_lp, input_period);
		lp_acc_z.init(fcut_pre_lp, input_period);
		lp_gyro_x.init(fcut_pre_lp, input_period);
		lp_gyro_y.init(fcut_pre_lp, input_period);
		lp_gyro_z.init(fcut_pre_lp, input_period);

		lp_angle_roll.init(fcut_angle_lp, input_period);
		lp_angle_pitch.init(fcut_angle_lp, input_period);

		lp_gyro_bias_x.init(fcut_gyro_bias, input_period);
		lp_gyro_bias_y.init(fcut_gyro_bias, input_period);
		lp_gyro_bias_z.init(fcut_gyro_bias, input_period);

		hp_omega_mod.init(omega_mod_hp_fcut, input_period);
		hp_angle_roll.init(angle_hpf_fcut, input_period);
		hp_angle_pitch.init(angle_hpf_fcut, input_period);
		hp_yaw.init(angle_hpf_fcut, input_period);

		lp_gamma0.init(gamma_lp_fcut, input_period);
		lp_gamma180.init(gamma_lp_fcut, input_period);
		hp_gamma0.init(angle_hpf_fcut, input_period);
		hp_gamma180.init(angle_hpf_fcut, input_period);

		rls_nominal = ScalarRLS(rls_lambda, 1e6f);
		rls_rotated = ScalarRLS(rls_lambda, 1e6f);
		rls_nominal.reset();
		rls_rotated.reset();
	}

	// To be called per raw IMU sample. acc in G's, gyro in rad/s.
	void feedSample(const Vec3& acc_raw, const Vec3& gyro_raw)
	{
		decim_count++;
		if (decim_count < decimation)
			return;
		decim_count = 0;

		// Preprocess: low-pass the raw signals
		float ax_lp = lp_acc_x.apply(acc_raw.x);
		float ay_lp = lp_acc_y.apply(acc_raw.y);
		float az_lp = lp_acc_z.apply(acc_raw.z);

		float gx_lp = lp_gyro_x.apply(gyro_raw.x);
		float gy_lp = lp_gyro_y.apply(gyro_raw.y);
		float gz_lp = lp_gyro_z.apply(gyro_raw.z);

		// gyro mod norm and HPF to detect motion / stationary
		float omega_mod = std::sqrt(gx_lp * gx_lp + gy_lp * gy_lp + gz_lp * gz_lp);
		float omega_mod_hpf = hp_omega_mod.apply(omega_mod);

		bool vehicle_standstill = std::fabs(omega_mod_hpf) <= omega_threshold;

		// If standstill -> update gyro bias estimation with a very low pass
		if (vehicle_standstill) {
			lp_gyro_bias_x.apply(gx_lp);
			lp_gyro_bias_y.apply(gy_lp);
			lp_gyro_bias_z.apply(gz_lp);
		}

		// bias subtract (paper estimates biases using LP when standing)
		float gx_unbiased = gx_lp - lp_gyro_bias_x.y;
		float gy_unbiased = gy_lp - lp_gyro_bias_y.y;
		float gz_unbiased = gz_lp - lp_gyro_bias_z.y;

		// ---------- ROLL & PITCH ESTIMATION ----------
		// Select gravitational-like samples: ||a|| close to g
		float a_norm = std::sqrt(ax_lp * ax_lp + ay_lp * ay_lp + az_lp * az_lp);
		bool acc_valid_for_rp = (a_norm >= (g - dg_th) && a_norm <= (g + dg_th));

		if (acc_valid_for_rp) {
			// compute roll/pitch as in paper (Eq. 11)
			float roll_k = std::atan2(ay_lp, az_lp);
			float pitch_k = std::atan2(-ax_lp, std::sin(roll_k) * ay_lp + std::cos(roll_k) * az_lp);

			// lowpass the angles
			float expected_roll = lp_angle_roll.apply(roll_k);
			float expected_pitch = lp_angle_pitch.apply(pitch_k);

			// convergence detection for RP: HPF angles small => converged
			float roll_hpf = hp_angle_roll.apply(expected_roll);
			float pitch_hpf = hp_angle_pitch.apply(expected_pitch);

			assignLatchedValue(latched_roll, roll_hpf, angle_conv_enter, angle_conv_exit);
			assignLatchedValue(latched_pitch, pitch_hpf, angle_conv_enter, angle_conv_exit);

			if (latched_roll && latched_pitch) {
				roll = expected_roll;
				pitch = expected_pitch;
				flagRollPitchConverged = true;
			}
		}

		// ---------- Prepare RP-rotated frame values ----------
		// rotate acc and gyro to roll/pitch rotated frame (remove roll+pitch)
		// Build rotation: R_rp = R_x(roll) * R_y(pitch) (note order to rotate IMU frame to RP frame)
		// We'll approximate by applying inverse rotations: first rotate by -roll about X, then by -pitch about Y.
		Vec3 acc_RP = rotate_remove_roll_pitch({ax_lp, ay_lp, az_lp}, roll, pitch);
		Vec3 gyro_RP = rotate_remove_roll_pitch({gx_unbiased, gy_unbiased, gz_unbiased}, roll, pitch);

		// ---------- YAW ESTIMATION (after RP converged) ----------
		if (flagRollPitchConverged) {
			// selection for yaw: not turning & accelerating on a straight & limited roll/pitch dynamics (Eq.13)
			float wz_rp = gyro_RP.z;
			float wxy_rp_norm = std::sqrt(gyro_RP.x * gyro_RP.x + gyro_RP.y * gyro_RP.y);
			float axy_rp_norm = std::sqrt(acc_RP.x * acc_RP.x + acc_RP.y * acc_RP.y);

			bool cond_not_turning = (std::fabs(wz_rp) < omega_z_th);
			bool cond_accel = (axy_rp_norm > a_xy_th);
			bool cond_rp_limited = (wxy_rp_norm <= omega_xy_th);

			if (cond_not_turning && cond_accel && cond_rp_limited) {
				// feed RLS with the 2D planar acceleration points (ax,ay)
				float axp = acc_RP.x;
				float ayp = acc_RP.y;

				// Nominal: y = m * x  => ay = m * ax
				rls_nominal.update(axp, ayp);

				// Rotated by +90 degrees: (x_r, y_r) = (-ay, ax)
				float xr = -ayp;
				float yr = axp;
				rls_rotated.update(xr, yr);

				// compute non-linear distance errors (Eq.17)
				float m_nom = rls_nominal.m;
				float m_rot = rls_rotated.m;

				float Jnom = std::fabs(ayp - m_nom * axp) / std::sqrt(1.0f + m_nom * m_nom);
				float Jrot = std::fabs(yr - m_rot * xr) / std::sqrt(1.0f + m_rot * m_rot);

				float expectedYaw;

				// choose best RLS
				if (Jnom <= Jrot) {
					float psi = std::atan(m_nom); // yaw estimate
					expectedYaw = psi;
				}
				else {
					float psi = std::atan(m_rot) - M_PI / 2.0f;
					expectedYaw = psi;
				}

				// lowpass/HPF and convergence check for yaw (paper uses HPF small)
				float yaw_hpf = hp_yaw.apply(expectedYaw);
				// For convergence we check HPF magnitude small
				// flagYawConverged = (std::fabs(yaw_hpf) <= angle_conv_th);
				assignLatchedValue(latched_yaw, yaw_hpf, angle_conv_enter, angle_conv_exit);
				if (!latched_yaw)
					return;

				yaw = expectedYaw;
				flagYawConverged = true;

				// ---------- Travel direction recognition ----------
				// Two candidate yaws: yaw and yaw + pi
				float psi0 = yaw;
				float psiPi = yaw + M_PI;

				// select turning instants: |wz_rp| > w_z_tdir and |a_xy_rp| > a_tdir (paper)
				bool sel_turn = (std::fabs(wz_rp) > 0.01f) && (axy_rp_norm > 0.01f);
				if (sel_turn) {
					// rotate the RP planar acceleration into candidate vehicle frames:
					float ax0, ay0, axp0, ayp0;
					rotate2D(acc_RP.x, acc_RP.y, -psi0, ax0, ay0);	  // transform by -psi0
					rotate2D(acc_RP.x, acc_RP.y, -psiPi, axp0, ayp0); // transform by -(psi+pi)

					// Gamma0 = 1 if sign(a_y) == sign(wz)
					int g0 = std::copysign(1.0f, ay0) == std::copysign(1.0f, wz_rp) ? 1 : 0;
					int gpi = std::copysign(1.0f, ayp0) == std::copysign(1.0f, wz_rp) ? 1 : 0;

					float g0f = lp_gamma0.apply(float(g0));
					float gpif = lp_gamma180.apply(float(gpi));
					// After lowpass, check if one exceeds threshold
					if (!latched_gamma) {
						if (g0f >= gamma_conv_enter) {
							latched_gamma = true;
							// candidate 0 (psi0) is correct -> nothing
						}
						else if (gpif >= gamma_conv_enter) {
							latched_gamma = true;
							yaw += M_PI; // invert yaw
						}
					}
					else if (g0f < gamma_conv_exit || gpif < gamma_conv_exit)
						latched_gamma = false;

					flagDirectionConverged = latched_gamma;
				}
			}
		}

		// calibrated flag if all three (RP, Yaw, Direction) converged
		flagCalibrated = flagRollPitchConverged && flagYawConverged && flagDirectionConverged;
	}

	// helper: given roll,pitch angles remove roll & pitch from a body vector
	// apply inverse rotations: first rotate about X by -roll, then about Y by -pitch
	static Vec3 rotate_remove_roll_pitch(const Vec3& v, float roll, float pitch)
	{
		// rotate about X by -roll
		float cr = std::cos(-roll), sr = std::sin(-roll);
		Vec3 v1{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
		// rotate about Y by -pitch
		float cp = std::cos(-pitch), sp = std::sin(-pitch);
		Vec3 v2{cp * v1.x + sp * v1.z, v1.y, -sp * v1.x + cp * v1.z};
		return v2;
	}

	// Return rotation matrix from mounting angles (roll,pitch,yaw) following Z-Y-X (yaw-pitch-roll) order
	Mat3 getRotationMatrix_IMUtoVehicle() const
	{
		float cr = std::cos(roll), sr = std::sin(roll);
		float cp = std::cos(pitch), sp = std::sin(pitch);
		float cy = std::cos(yaw), sy = std::sin(yaw);
		// Compose R = R_z(yaw) * R_y(pitch) * R_x(roll)
		Mat3 R;
		R.m[0][0] = cy * cp;
		R.m[0][1] = cy * sp * sr - sy * cr;
		R.m[0][2] = cy * sp * cr + sy * sr;

		R.m[1][0] = sy * cp;
		R.m[1][1] = sy * sp * sr + cy * cr;
		R.m[1][2] = sy * sp * cr - cy * sr;

		R.m[2][0] = -sp;
		R.m[2][1] = cp * sr;
		R.m[2][2] = cp * cr;
		return R;
	}

	// Print status
	void printStatus() const
	{
		std::cout << "RP conv: " << flagRollPitchConverged << " Yaw conv: " << flagYawConverged << " Dir conv: " << flagDirectionConverged
				  << " Calibrated: " << flagCalibrated << "\n";
		std::cout << "roll (deg): " << roll * 180.0f / M_PI << " pitch (deg): " << pitch * 180.0f / M_PI << " yaw (deg): " << yaw * 180.0f / M_PI << "\n";
	}

	// 2D rotate (x,y) by angle radians: (x',y') = R(angle) * (x,y)
	static void rotate2D(float x, float y, float angle, float& xr, float& yr)
	{
		float c = std::cos(angle), s = std::sin(angle);
		xr = c * x - s * y;
		yr = s * x + c * y;
	}

	static void assignLatchedValue(bool& latch, float errorValue, float enterThreshold, float exitThreshold) noexcept
	{
		if (const float absErrorValue = std::fabs(errorValue); !latch) {
			if (absErrorValue <= enterThreshold)
				latch = true;
		}
		else if (absErrorValue > exitThreshold)
			latch = false;
	}

private:
	float angle_conv_enter = 0.075f * (M_PI / 180.0f); // 0.075° enter (hysteresis)
	float angle_conv_exit = 0.2f * (M_PI / 180.0f);	   // 0.2° exit (hysteresis)

	float gamma_conv_enter = 0.57f;
	float gamma_conv_exit = 0.43f;

	// latched flags
	bool latched_roll = false;
	bool latched_pitch = false;
	bool latched_yaw = false;
	bool latched_gamma = false;
};
