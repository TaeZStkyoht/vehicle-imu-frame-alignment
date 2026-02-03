#pragma once

/**
 * @file MountingOrientationEstimator.hpp
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

#include <cmath>

namespace imu {
	namespace data {
		struct Vec3 final {
			constexpr Vec3 operator-(const Vec3& v) const noexcept
			{
				return Vec3{
					.x = x - v.x,
					.y = y - v.y,
					.z = z - v.z,
				};
			}

			float x;
			float y;
			float z;
		};

		struct Mat3 final {
			constexpr Vec3 Mul(const Vec3& v) const noexcept
			{
				return {
					m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
					m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
					m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
				};
			}

			std::array<std::array<float, 3>, 3> m;
		};

		struct ImuData final {
			Vec3 acc;
			Vec3 gyro;
		};
	}

	// ---------- IIR filters from paper ----------
	namespace filter {
		// First-order low-pass: y(k) = alpha * u(k) + (1-alpha) * y(k-1)
		class LowPass final {
		public:
			constexpr LowPass(float fcut, float dt) noexcept : _alpha(2.0f * M_PIf * dt * fcut / (2.0f * M_PIf * dt * fcut + 1.0f))
			{
			}

			constexpr float Apply(float u) noexcept
			{
				if (!_initialized) [[unlikely]] {
					_initialized = true;
					_y = u;
				}
				else
					_y = _alpha * u + (1.0f - _alpha) * _y;

				return _y;
			}

			constexpr float Output() const noexcept
			{
				return _y;
			}

		private:
			bool _initialized = false;
			float _alpha;

			float _y{};
		};

		class LowPass3d final {
		public:
			constexpr LowPass3d(float fcut, float dt) noexcept : _lp_x(fcut, dt), _lp_y(fcut, dt), _lp_z(fcut, dt)
			{
			}

			constexpr data::Vec3 Apply(const data::Vec3& input) noexcept
			{
				return data::Vec3{
					.x = _lp_x.Apply(input.x),
					.y = _lp_y.Apply(input.y),
					.z = _lp_z.Apply(input.z),
				};
			}

			constexpr data::Vec3 Output() const noexcept
			{
				return data::Vec3{
					.x = _lp_x.Output(),
					.y = _lp_y.Output(),
					.z = _lp_z.Output(),
				};
			}

		private:
			LowPass _lp_x;
			LowPass _lp_y;
			LowPass _lp_z;
		};

		// First-order high-pass: y(k) = beta*y(k-1) + beta*(u(k)-u(k-1))
		class HighPass final {
		public:
			constexpr HighPass(float fcut, float dt) noexcept : _beta(1.0f / (2.0f * M_PIf * dt * fcut + 1.0f))
			{
			}

			constexpr float Apply(float u) noexcept
			{
				if (!_initialized) [[unlikely]] {
					_initialized = true;
					_y = u;
				}
				else
					_y = _beta * _y + _beta * (u - _u_prev);

				_u_prev = u;
				return _y;
			}

		private:
			bool _initialized = false;
			float _beta;

			float _y{};
			float _u_prev{};
		};

		// ---------- Simple scalar RLS for model y = m * x (no intercept) ----------
		// Recursive Least Squares with forgetting factor lambda (paper uses mu in [0,1) with cost sum mu^(k-i) -> lambda=mu)
		class ScalarRLS final {
		public:
			constexpr ScalarRLS(float lambda, float P0) noexcept : _lambda(lambda), _P(P0)
			{
			}

			constexpr void Update(float x, float y) noexcept
			{
				// avoid degenerate x
				const float denom = _lambda + x * _P * x;
				const float K = (_P * x) / denom; // scalar gain
				const float err = y - x * _m;
				_m = _m + K * err;
				_P = (1.0f / _lambda) * (_P - K * x * _P);
			}

			constexpr float Output() const noexcept
			{
				return _m;
			}

		private:
			float _lambda; // forgetting factor in (0,1), smaller -> fast forgetting
			float _P;	   // covariance scalar
			float _m{};	   // slope estimate
		};
	}

	namespace allignment {
		struct Parameters final {
			// Parameters (tunable)
			float fcut_pre_lp = 1.4f;		// low-pass for acc/gyro preprocessing (Hz)
			float fcut_angle_lp = 1.0f;		// LP for roll/pitch angle smoothing
			float fcut_gyro_bias = 0.001f;	// LP for gyro bias estimation (very low)
			float dg_th = 0.15f;			// tolerance for |a| ~ g to select gravity samples
			float omega_mod_hp_fcut = 0.1f; // HPF for gyro norm to detect stillness (Hz)
			float omega_threshold = 0.02f;	// threshold for stationary detection (rad/s)
			// Yaw selection thresholds (paper tunings)
			float omega_z_th = 0.1f;  // small yaw rate threshold for "not turning" [rad/s]
			float a_xy_th = 0.02f;	  // planar acceleration threshold for selecting straight accel [g]
			float omega_xy_th = 0.2f; // threshold for roll/pitch dynamics limited
			// Travel direction recognition thresholds
			float gamma_lp_fcut = 0.5f;
			// Convergence HPF thresholds for angles (paper uses HPF then threshold)
			float angle_hpf_fcut = 0.01f;

			// RLS params
			float rls_lambda = 0.995f; // forgetting factor

			float g = 1.0f; // gravitational magnitude in g units (1g)

			float angle_conv_enter = 0.075f * (M_PIf / 180.0f); // 0.075° enter (hysteresis)
			float angle_conv_exit = 0.2f * (M_PIf / 180.0f);	// 0.2° exit (hysteresis)

			float gamma_conv_enter = 0.57f;
			float gamma_conv_exit = 0.43f;
		};

		// ---------- Calibration class implementing the pipeline ----------
		class MountingOrientationEstimator final {
		public:
			constexpr explicit MountingOrientationEstimator(float input_period, const Parameters& parameters = {}) noexcept
				: _parameters(parameters), _lp_acc(_parameters.fcut_pre_lp, input_period), _lp_gyro(_parameters.fcut_pre_lp, input_period),
				  _lp_gyro_bias(_parameters.fcut_gyro_bias, input_period), _lp_angle_roll(_parameters.fcut_angle_lp, input_period),
				  _lp_angle_pitch(_parameters.fcut_angle_lp, input_period), _lp_gamma0(_parameters.gamma_lp_fcut, input_period),
				  _lp_gamma180(_parameters.gamma_lp_fcut, input_period), _hp_omega_mod(_parameters.omega_mod_hp_fcut, input_period),
				  _hp_angle_roll(_parameters.angle_hpf_fcut, input_period), _hp_angle_pitch(_parameters.angle_hpf_fcut, input_period),
				  _hp_yaw(_parameters.angle_hpf_fcut, input_period), _rls_nominal(_parameters.rls_lambda, 1e6f), _rls_rotated(_parameters.rls_lambda, 1e6f)
			{
			}

			constexpr void FeedSample(const data::ImuData& imu_data) noexcept
			{
				const auto [acc_filtered, gyro_filtered] = PreProcessData(imu_data.acc, imu_data.gyro);

				if (!_convergenceFlags.roll_pitch) {
					EstimateRollPitch(acc_filtered);
					return;
				}

				const auto [acc_RP, gyro_RP] = PrepareRollPitchRotatedImuData(acc_filtered, gyro_filtered);

				const float wz_rp = gyro_RP.z;
				const float axy_rp_norm = Normalize(acc_RP.x, acc_RP.y);

				if (!_convergenceFlags.yaw) {
					EstimateYaw(wz_rp, axy_rp_norm, acc_RP, gyro_RP);
					return;
				}

				if (!_convergenceFlags.direction)
					EstimateDirection(wz_rp, axy_rp_norm, acc_RP);
			}

			// Return rotation matrix from mounting angles (roll,pitch,yaw) following Z-Y-X (yaw-pitch-roll) order
			constexpr data::Mat3 GetRotationMatrixOfImuToVehicle() const noexcept
			{
				const auto [sr, cr] = GetSinCos(_mountingAngles.roll);
				const auto [sp, cp] = GetSinCos(_mountingAngles.pitch);
				const auto [sy, cy] = GetSinCos(_mountingAngles.yaw);

				// Compose R = R_z(yaw) * R_y(pitch) * R_x(roll)
				return data::Mat3{
					std::array{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
					std::array{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
					std::array{-sp, cp * sr, cp * cr},
				};
			}

			constexpr bool IsRollPitchConverged() const noexcept
			{
				return _convergenceFlags.roll_pitch;
			}

			constexpr bool IsYawConverged() const noexcept
			{
				return _convergenceFlags.yaw;
			}

			constexpr bool IsDirectionConverged() const noexcept
			{
				return _convergenceFlags.direction;
			}

			constexpr bool IsCalibrated() const noexcept
			{
				return _convergenceFlags.roll_pitch && _convergenceFlags.yaw && _convergenceFlags.direction;
			}

			constexpr float Roll() const noexcept
			{
				return _mountingAngles.roll;
			}

			constexpr float Pitch() const noexcept
			{
				return _mountingAngles.pitch;
			}

			constexpr float Yaw() const noexcept
			{
				return _mountingAngles.yaw;
			}

			static constexpr float FrequencyToPeriod(float fs_in) noexcept
			{
				return 1.0f / fs_in;
			}

		private:
			struct SinCos final {
				float s;
				float c;
			};

			struct GammaValues final {
				float g0f;
				float gpif;
			};

			struct ConvergenceFlags final {
				bool roll_pitch = false;
				bool yaw = false;
				bool direction = false;
			};

			struct Latches final {
				bool roll = false;
				bool pitch = false;
				bool yaw = false;
				bool gamma = false;
			};

			struct MountingAngles final {
				float roll = 0.0f;
				float pitch = 0.0f;
				float yaw = 0.0f;
			};

			constexpr data::ImuData PreProcessData(const data::Vec3& acc_raw, const data::Vec3& gyro_raw) noexcept
			{
				// Preprocess: low-pass the raw signals
				auto a_lp = _lp_acc.Apply(acc_raw);
				const auto g_lp = _lp_gyro.Apply(gyro_raw);

				// gyro mod norm and HPF to detect motion / stationary
				// If standstill -> update gyro bias estimation with a very low pass
				if (std::fabs(_hp_omega_mod.Apply(Normalize(g_lp.x, g_lp.y, g_lp.z))) <= _parameters.omega_threshold)
					_lp_gyro_bias.Apply(g_lp);

				return {a_lp, g_lp - _lp_gyro_bias.Output()}; // bias subtract (paper estimates biases using LP when standing)
			}

			constexpr void EstimateRollPitch(const data::Vec3& acc_filtered) noexcept
			{
				// Select gravitational-like samples: ||a|| close to g
				if (const float a_norm = Normalize(acc_filtered.x, acc_filtered.y, acc_filtered.z);
					a_norm >= (_parameters.g - _parameters.dg_th) && a_norm <= (_parameters.g + _parameters.dg_th)) {
					// compute roll/pitch as in paper (Eq. 11)
					const float roll_k = std::atan2(acc_filtered.y, acc_filtered.z);
					const auto [sr, cr] = GetSinCos(roll_k);
					const float pitch_k = std::atan2(-acc_filtered.x, sr * acc_filtered.y + cr * acc_filtered.z);

					// lowpass the angles
					const float expected_roll = _lp_angle_roll.Apply(roll_k);
					const float expected_pitch = _lp_angle_pitch.Apply(pitch_k);

					// convergence detection for RP: HPF angles small => converged
					AssignLatchedValue(_latches.roll, _hp_angle_roll.Apply(expected_roll), _parameters.angle_conv_enter, _parameters.angle_conv_exit);
					AssignLatchedValue(_latches.pitch, _hp_angle_pitch.Apply(expected_pitch), _parameters.angle_conv_enter, _parameters.angle_conv_exit);

					if (_latches.roll && _latches.pitch) {
						_mountingAngles.roll = expected_roll;
						_mountingAngles.pitch = expected_pitch;
						_convergenceFlags.roll_pitch = true;
					}
				}
			}

			constexpr data::ImuData PrepareRollPitchRotatedImuData(const data::Vec3& acc_filtered, const data::Vec3& gyro_filtered) const noexcept
			{
				// rotate acc and gyro to roll/pitch rotated frame (remove roll+pitch)
				// Build rotation: R_rp = R_x(roll) * R_y(pitch) (note order to rotate IMU frame to RP frame)
				// We'll approximate by applying inverse rotations: first rotate by -roll about X, then by -pitch about Y.
				return {RotateRemoveRollPitch(acc_filtered, _mountingAngles.roll, _mountingAngles.pitch),
						RotateRemoveRollPitch(gyro_filtered, _mountingAngles.roll, _mountingAngles.pitch)};
			}

			constexpr float CalculateYaw(const data::Vec3& acc_RP) noexcept
			{
				// feed RLS with the 2D planar acceleration points (ax,ay)
				const float axp = acc_RP.x;
				const float ayp = acc_RP.y;

				// Nominal: y = m * x  => ay = m * ax
				_rls_nominal.Update(axp, ayp);

				// Rotated by +90 degrees: (x_r, y_r) = (-ay, ax)
				const float xr = -ayp;
				const float yr = axp;
				_rls_rotated.Update(xr, yr);

				// compute non-linear distance errors (Eq.17)
				const float m_nom = _rls_nominal.Output();
				const float m_rot = _rls_rotated.Output();

				// choose best RLS (below condition: Jnom <= Jrot)
				return std::fabs(ayp - m_nom * axp) / std::sqrt(1.0f + Square(m_nom)) <= std::fabs(yr - m_rot * xr) / std::sqrt(1.0f + Square(m_rot))
						   ? std::atan(m_nom)
						   : (std::atan(m_rot) - M_PIf / 2.0f); // yaw estimate
			}

			constexpr void EstimateYaw(float wz_rp, float axy_rp_norm, const data::Vec3& acc_RP, const data::Vec3& gyro_RP) noexcept
			{
				// Below conditions: not turning && accelerating && limited roll/pitch dynamics (wxy_rp_norm <= omega_xy_th) (Eq.13)
				if (std::fabs(wz_rp) < _parameters.omega_z_th && axy_rp_norm > _parameters.a_xy_th &&
					Normalize(gyro_RP.x, gyro_RP.y) <= _parameters.omega_xy_th) {
					const float expectedYaw = CalculateYaw(acc_RP);

					// convergence detection for yaw: HPF angles small => converged
					AssignLatchedValue(_latches.yaw, _hp_yaw.Apply(expectedYaw), _parameters.angle_conv_enter, _parameters.angle_conv_exit);
					if (_latches.yaw) {
						_mountingAngles.yaw = expectedYaw;
						_convergenceFlags.yaw = true;
					}
				}
			}

			constexpr GammaValues CalculateGamma(float wz_rp, const data::Vec3& acc_RP) noexcept
			{
				// Two candidate yaws: yaw and yaw + pi
				// rotate the RP planar acceleration into candidate vehicle frames:
				float ay0 = Rotate2D(acc_RP.x, acc_RP.y, -_mountingAngles.yaw);			   // transform by -psi0
				float ayp0 = Rotate2D(acc_RP.x, acc_RP.y, -(_mountingAngles.yaw + M_PIf)); // transform by -(psi+pi)

				// Gamma0 = 1 if sign(a_y) == sign(wz)
				int g0 = std::copysign(1.0f, ay0) == std::copysign(1.0f, wz_rp) ? 1 : 0;
				int gpi = std::copysign(1.0f, ayp0) == std::copysign(1.0f, wz_rp) ? 1 : 0;

				return {_lp_gamma0.Apply(float(g0)), _lp_gamma180.Apply(float(gpi))};
			}

			constexpr void EstimateDirection(float wz_rp, float axy_rp_norm, const data::Vec3& acc_RP) noexcept
			{
				// select turning instants: |wz_rp| > w_z_tdir and |a_xy_rp| > a_tdir (paper)
				if (std::fabs(wz_rp) > 0.01f && axy_rp_norm > 0.01f) {
					if (const auto [g0f, gpif] = CalculateGamma(wz_rp, acc_RP); !_latches.gamma) { // After lowpass, check if one exceeds threshold
						if (g0f >= _parameters.gamma_conv_enter) {
							_latches.gamma = true;
							// candidate 0 (psi0) is correct -> nothing
						}
						else if (gpif >= _parameters.gamma_conv_enter) {
							_latches.gamma = true;
							_mountingAngles.yaw += M_PIf; // invert yaw
						}
					}
					else if (g0f < _parameters.gamma_conv_exit || gpif < _parameters.gamma_conv_exit)
						_latches.gamma = false;

					if (_latches.gamma)
						_convergenceFlags.direction = true;
				}
			}

			static constexpr float Square(float value) noexcept
			{
				return value * value;
			}

			template <typename... Args>
			static constexpr float Normalize(Args... args) noexcept
			{
				return std::sqrt((Square(args) + ...));
			}

			static constexpr SinCos GetSinCos(float angle) noexcept
			{
				return {std::sin(angle), std::cos(angle)};
			}

			static constexpr void AssignLatchedValue(bool& latch, float errorValue, float enter_threshold, float exit_threshold) noexcept
			{
				if (const float abs_error_value = std::fabs(errorValue); !latch) {
					if (abs_error_value <= enter_threshold)
						latch = true;
				}
				else if (abs_error_value > exit_threshold)
					latch = false;
			}

			// helper: given roll,pitch angles remove roll & pitch from a body vector
			// apply inverse rotations: first rotate about X by -roll, then about Y by -pitch
			static constexpr data::Vec3 RotateRemoveRollPitch(const data::Vec3& v, float roll, float pitch) noexcept
			{
				// rotate about X by -roll
				const auto [sr, cr] = GetSinCos(-roll);
				const data::Vec3 v1{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
				// rotate about Y by -pitch
				const auto [sp, cp] = GetSinCos(-pitch);
				return {cp * v1.x + sp * v1.z, v1.y, -sp * v1.x + cp * v1.z};
			}

			// 2D rotate (x,y) by angle radians: (x',y') = R(angle) * (x,y) and return only y value
			static constexpr float Rotate2D(float x, float y, float angle) noexcept
			{
				const auto [s, c] = GetSinCos(angle);
				// xr = c * x - s * y; // x value
				return s * x + c * y;
			}

			Parameters _parameters;

			filter::LowPass3d _lp_acc;
			filter::LowPass3d _lp_gyro;
			filter::LowPass3d _lp_gyro_bias;
			filter::LowPass _lp_angle_roll;
			filter::LowPass _lp_angle_pitch;
			filter::LowPass _lp_gamma0;
			filter::LowPass _lp_gamma180;

			filter::HighPass _hp_omega_mod;
			filter::HighPass _hp_angle_roll;
			filter::HighPass _hp_angle_pitch;
			filter::HighPass _hp_yaw;

			filter::ScalarRLS _rls_nominal;
			filter::ScalarRLS _rls_rotated;

			// outputs
			MountingAngles _mountingAngles;

			// latched flags
			Latches _latches;

			ConvergenceFlags _convergenceFlags;
		};
	}
}
