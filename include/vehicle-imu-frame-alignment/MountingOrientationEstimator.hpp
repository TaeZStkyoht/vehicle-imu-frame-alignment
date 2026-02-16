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
 * Co-Author: Adrian Kersten
 *
 * Notes:
 * - Inputs: accelerometer [g], gyroscope [rad/s]
 * - Designed for ground vehicles
 */

#include <cmath>

namespace imu {
	namespace data {
		struct Vec3 final {
			constexpr Vec3 operator-(const Vec3& v) const noexcept
			{
				return Vec3{
					x - v.x,
					y - v.y,
					z - v.z,
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

			float m[3][3];
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
				_y = _alpha * u + (1.0f - _alpha) * _y;
				return _y;
			}

			constexpr float Output() const noexcept
			{
				return _y;
			}

		private:
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
					_lp_x.Apply(input.x),
					_lp_y.Apply(input.y),
					_lp_z.Apply(input.z),
				};
			}

			constexpr data::Vec3 Output() const noexcept
			{
				return data::Vec3{
					_lp_x.Output(),
					_lp_y.Output(),
					_lp_z.Output(),
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
				_y = _beta * _y + _beta * (u - _u_prev);
				_u_prev = u;
				return _y;
			}

		private:
			float _beta;

			float _y{};
			float _u_prev{};
		};

		// ---------- Simple scalar RLS for model y = m * x (no intercept) ----------
		// Recursive Least Squares with forgetting factor lambda (paper uses mu in [0,1) with cost sum mu^(k-i) -> lambda=mu)
		class ScalarRLS final {
		public:
			constexpr ScalarRLS(float lambda, float P0) noexcept : _lambda(lambda), _covariance_P(P0)
			{
			}

			constexpr void Update(float x, float y) noexcept
			{
				// avoid degenerate x
				const float denominator = _lambda + x * _covariance_P * x;
				const float gain_K = (_covariance_P * x) / denominator;
				const float estimation_error = y - x * _slope_m;
				_slope_m = _slope_m + gain_K * estimation_error;
				_covariance_P = (1.0f / _lambda) * (_covariance_P - gain_K * x * _covariance_P);
			}

			constexpr float Output() const noexcept
			{
				return _slope_m;
			}

		private:
			float _lambda; // forgetting factor in (0,1), smaller -> faster forgetting
			float _covariance_P;
			float _slope_m{};
		};
	}

	namespace allignment {
		struct Parameters final {
			static constexpr float deg_to_rad = M_PIf / 180.0f; // conversion factor for degrees to radians
			static constexpr float g = 1.0f;					// gravitational magnitude in g units (1g)

			// Data pre-processing
			float fcut_pre_lp = 1.4f;		// low-pass for acc/gyro preprocessing (Hz)
			float omega_mod_hp_fcut = 0.1f; // HPF for gyro norm to detect stillness (Hz)
			float omega_th = 0.02f;			// threshold for stationary detection (rad/s)
			float fcut_gyro_bias = 0.001f;	// LP for gyro bias estimation (very low)

			// Roll/pitch estimation
			float delta_g_th = 0.01f;					  // tolerance for |a| ~ g to select gravity samples
			float fcut_angle_lp = 1.0f;					  // LP for roll/pitch angle smoothing
			float angle_hpf_fcut = 0.01f;				  // Convergence HPF thresholds for angles (paper uses HPF then threshold)
			float angle_conv_enter = 0.075f * deg_to_rad; // 0.075° enter (hysteresis)
			float angle_conv_exit = 0.2f * deg_to_rad;	  // 0.2° exit (hysteresis)

			// Yaw selection thresholds
			float omega_z_th = 0.05f; // small yaw rate threshold for "not turning" (rad/s)
			float a_xy_th = 0.15f;	  // planar acceleration threshold for selecting straight accel (g)
			float omega_xy_th = 0.1f; // threshold for roll/pitch dynamics limited

			// RLS params
			float rls_lambda = 0.995f; // forgetting factor
			float rls_p0 = 1e4f;	   // initial covariance

			// Travel direction recognition thresholds
			float omega_z_dir_th = 0.05f; // threshold for selecting turning instants (rad/s)
			float a_xy_dir_th = 0.1f;	  // threshold for selecting turning instants with enough planar acceleration (g)
			float gamma_lp_fcut = 0.2f;
			float gamma_conv_enter = 0.57f;
			float gamma_conv_exit = 0.43f;
		};

		// ---------- Calibration class implementing the pipeline ----------
		class MountingOrientationEstimator final {
		public:
#ifdef DEBUG_VIFA
			size_t rollPitchEntrance = 0;
			size_t yawEntrance = 0;
			size_t dirEntrance = 0;

			size_t rollPitchTotalPoint = 0;
			size_t yawTotalPoint = 0;
			size_t dirTotalPoint = 0;
#endif
			constexpr explicit MountingOrientationEstimator(float input_period, const Parameters& parameters = {}) noexcept
				: _parameters(parameters), _lp_acc(_parameters.fcut_pre_lp, input_period), _lp_gyro(_parameters.fcut_pre_lp, input_period),
				  _lp_gyro_bias(_parameters.fcut_gyro_bias, input_period), _lp_angle_roll(_parameters.fcut_angle_lp, input_period),
				  _lp_angle_pitch(_parameters.fcut_angle_lp, input_period), _lp_gamma_0(_parameters.gamma_lp_fcut, input_period),
				  _lp_gamma_pi(_parameters.gamma_lp_fcut, input_period), _hp_omega_mod(_parameters.omega_mod_hp_fcut, input_period),
				  _hp_angle_roll(_parameters.angle_hpf_fcut, input_period), _hp_angle_pitch(_parameters.angle_hpf_fcut, input_period),
				  _hp_yaw(_parameters.angle_hpf_fcut, input_period), _rls_nominal(_parameters.rls_lambda, _parameters.rls_p0),
				  _rls_rotated(_parameters.rls_lambda, _parameters.rls_p0)
			{
			}

			constexpr void FeedSample(const data::ImuData& imu_data) noexcept
			{
				const auto [acc_filtered, gyro_filtered] = PreProcessData(imu_data.acc, imu_data.gyro);

				if (!_convergence_flags.roll_pitch) {
					EstimateRollPitch(acc_filtered);
					return;
				}

				const auto [acc_rp, gyro_rp] = PrepareRollPitchRotatedImuData(acc_filtered, gyro_filtered);

				const float omega_z_rp = gyro_rp.z;
				const float a_xy_rp_norm = Normalize(acc_rp.x, acc_rp.y);

				if (!_convergence_flags.yaw) {
					EstimateYaw(omega_z_rp, a_xy_rp_norm, acc_rp, gyro_rp);
					return;
				}

				if (!_convergence_flags.direction)
					EstimateDirection(omega_z_rp, a_xy_rp_norm, acc_rp);
			}

			// Return rotation matrix from mounting angles (roll,pitch,yaw) following Z-Y-X (yaw-pitch-roll) order
			constexpr data::Mat3 GetRotationMatrixOfImuToVehicle() const noexcept
			{
				const auto [sin_r, cos_r] = GetSinCos(_mounting_angles.roll);
				const auto [sin_p, cos_p] = GetSinCos(_mounting_angles.pitch);
				const auto [sin_y, cos_y] = GetSinCos(_mounting_angles.yaw);

				// Compose R = R_z(yaw) * R_y(pitch) * R_x(roll)
				return data::Mat3{{
					{cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r},
					{sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r},
					{-sin_p, cos_p * sin_r, cos_p * cos_r},
				}};
			}

			constexpr bool IsRollPitchConverged() const noexcept
			{
				return _convergence_flags.roll_pitch;
			}

			constexpr bool IsYawConverged() const noexcept
			{
				return _convergence_flags.yaw;
			}

			constexpr bool IsDirectionConverged() const noexcept
			{
				return _convergence_flags.direction;
			}

			constexpr bool IsCalibrated() const noexcept
			{
				return _convergence_flags.roll_pitch && _convergence_flags.yaw && _convergence_flags.direction;
			}

			constexpr float Roll() const noexcept
			{
				return _mounting_angles.roll;
			}

			constexpr float Pitch() const noexcept
			{
				return _mounting_angles.pitch;
			}

			constexpr float Yaw() const noexcept
			{
				return _mounting_angles.yaw;
			}

			static constexpr float FrequencyToPeriod(float frequency) noexcept
			{
				return 1.0f / frequency;
			}

		private:
			struct SinCos final {
				float sin;
				float cos;
			};

			struct GammaValues final {
				float g_0_f;
				float g_pi_f;
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
				if (std::fabs(_hp_omega_mod.Apply(Normalize(g_lp.x, g_lp.y, g_lp.z))) <= _parameters.omega_th)
					_lp_gyro_bias.Apply(g_lp);

				return {a_lp, g_lp - _lp_gyro_bias.Output()}; // bias subtract (paper estimates biases using LP when standing)
			}

			constexpr void EstimateRollPitch(const data::Vec3& acc_filtered) noexcept
			{
#ifdef DEBUG_VIFA
				++rollPitchTotalPoint;
#endif
				// Select gravitational-like samples: ||a|| close to g
				if (const float a_norm = Normalize(acc_filtered.x, acc_filtered.y, acc_filtered.z);
					a_norm >= (Parameters::g - _parameters.delta_g_th) && a_norm <= (Parameters::g + _parameters.delta_g_th)) {
#ifdef DEBUG_VIFA
					++rollPitchEntrance;
#endif
					// compute roll/pitch as in paper (Eq. 11)
					const float roll_estimate = std::atan2(acc_filtered.y, acc_filtered.z);
					const auto [sin_roll, cos_roll] = GetSinCos(roll_estimate);
					const float pitch_estimate = std::atan2(-acc_filtered.x, sin_roll * acc_filtered.y + cos_roll * acc_filtered.z);

					// lowpass the angles
					const float filtered_roll_estimate = _lp_angle_roll.Apply(roll_estimate);
					const float filtered_pitch_estimate = _lp_angle_pitch.Apply(pitch_estimate);

					// convergence detection for RP: HPF angles small => converged
					AssignLatchedValue(_latches.roll, _hp_angle_roll.Apply(filtered_roll_estimate), _parameters.angle_conv_enter,
									   _parameters.angle_conv_exit);
					AssignLatchedValue(_latches.pitch, _hp_angle_pitch.Apply(filtered_pitch_estimate), _parameters.angle_conv_enter,
									   _parameters.angle_conv_exit);

					if (_latches.roll && _latches.pitch) {
						_mounting_angles.roll = filtered_roll_estimate;
						_mounting_angles.pitch = filtered_pitch_estimate;
						_convergence_flags.roll_pitch = true;
					}
				}
			}

			constexpr data::ImuData PrepareRollPitchRotatedImuData(const data::Vec3& acc_filtered, const data::Vec3& gyro_filtered) const noexcept
			{
				// rotate acc and gyro to roll/pitch rotated frame (remove roll+pitch)
				// Build rotation: R_rp = R_x(roll) * R_y(pitch) (note order to rotate IMU frame to RP frame)
				// We'll approximate by applying inverse rotations: first rotate by -roll about X, then by -pitch about Y.
				return {RotateRemoveRollPitch(acc_filtered, _mounting_angles.roll, _mounting_angles.pitch),
						RotateRemoveRollPitch(gyro_filtered, _mounting_angles.roll, _mounting_angles.pitch)};
			}

			constexpr float CalculateYaw(const data::Vec3& acc_rp) noexcept
			{
				// feed RLS with the 2D planar acceleration points (ax,ay)
				const float a_x_nom = acc_rp.x;
				const float a_y_nom = acc_rp.y;

				// Nominal: y = m * x  => ay = m * ax
				_rls_nominal.Update(a_x_nom, a_y_nom);

				// Rotated by +90 degrees: (a_x_rot, a_y_rot) = (-ay, ax)
				const float a_x_rot = -a_y_nom;
				const float a_y_rot = a_x_nom;
				_rls_rotated.Update(a_x_rot, a_y_rot);

				// compute non-linear distance errors (Eq.17)
				const float m_nom = _rls_nominal.Output();
				const float m_rot = _rls_rotated.Output();

				// choose best RLS
				const float J_nom = std::fabs(a_y_nom - m_nom * a_x_nom) / std::sqrt(1.0f + Square(m_nom));
				const float J_rot = std::fabs(a_y_rot - m_rot * a_x_rot) / std::sqrt(1.0f + Square(m_rot));
				return J_nom <= J_rot ? std::atan(m_nom) : (std::atan(m_rot) - M_PIf / 2.0f); // yaw estimate
			}

			constexpr void EstimateYaw(float omega_z_rp, float a_xy_rp_norm, const data::Vec3& acc_rp, const data::Vec3& gyro_rp) noexcept
			{
#ifdef DEBUG_VIFA
				++yawTotalPoint;
#endif
				// Below conditions: not turning && accelerating && limited roll/pitch dynamics (omega_xy_rp_norm <= omega_xy_th) (Eq.13)
				if (std::fabs(omega_z_rp) < _parameters.omega_z_th && a_xy_rp_norm > _parameters.a_xy_th &&
					Normalize(gyro_rp.x, gyro_rp.y) <= _parameters.omega_xy_th) {
#ifdef DEBUG_VIFA
					++yawEntrance;
#endif
					const float expected_yaw = CalculateYaw(acc_rp);

					// convergence detection for yaw: HPF angles small => converged
					AssignLatchedValue(_latches.yaw, _hp_yaw.Apply(expected_yaw), _parameters.angle_conv_enter, _parameters.angle_conv_exit);
					if (_latches.yaw) {
						_mounting_angles.yaw = expected_yaw;
						_convergence_flags.yaw = true;
					}
				}
			}

			constexpr GammaValues CalculateGamma(float omega_z_rp, const data::Vec3& acc_rp) noexcept
			{
				// Two candidate yaws: yaw and yaw + pi
				// rotate the RP planar acceleration into candidate vehicle frames:
				float a_y_0 = Rotate2D(acc_rp.x, acc_rp.y, -_mounting_angles.yaw);			  // transform by -psi0
				float a_y_pi = Rotate2D(acc_rp.x, acc_rp.y, -(_mounting_angles.yaw + M_PIf)); // transform by -(psi+pi)

				// Gamma0 = 1 if sign(a_y) == sign(omega_z)
				int gamma_0 = std::copysign(1.0f, a_y_0) == std::copysign(1.0f, omega_z_rp) ? 1 : 0;
				int gamma_pi = std::copysign(1.0f, a_y_pi) == std::copysign(1.0f, omega_z_rp) ? 1 : 0;

				return {_lp_gamma_0.Apply(float(gamma_0)), _lp_gamma_pi.Apply(float(gamma_pi))};
			}

			constexpr void EstimateDirection(float omega_z_rp, float axy_rp_norm, const data::Vec3& acc_rp) noexcept
			{
#ifdef DEBUG_VIFA
				++dirTotalPoint;
#endif
				// select turning instants with significant planar acceleration
				if (std::fabs(omega_z_rp) > _parameters.omega_z_dir_th && axy_rp_norm > _parameters.a_xy_dir_th) {
#ifdef DEBUG_VIFA
					++dirEntrance;
#endif
					if (const auto [gamma_0_f, gamma_pi_f] = CalculateGamma(omega_z_rp, acc_rp); !_latches.gamma) {
						if (gamma_0_f >= _parameters.gamma_conv_enter) {
							_latches.gamma = true;
							// candidate 0 (psi0) is correct -> nothing
						}
						else if (gamma_pi_f >= _parameters.gamma_conv_enter) {
							_latches.gamma = true;
							_mounting_angles.yaw += M_PIf; // invert yaw
						}
					}
					else if (gamma_0_f < _parameters.gamma_conv_exit && gamma_pi_f < _parameters.gamma_conv_exit)
						_latches.gamma = false;

					if (_latches.gamma)
						_convergence_flags.direction = true;
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

			static constexpr void AssignLatchedValue(bool& latch, float error_value, float enter_threshold, float exit_threshold) noexcept
			{
				if (const float abs_error_value = std::fabs(error_value); !latch) {
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
				const auto [sin_roll, cos_roll] = GetSinCos(-roll);
				const data::Vec3 v1{v.x, cos_roll * v.y - sin_roll * v.z, sin_roll * v.y + cos_roll * v.z};
				// rotate about Y by -pitch
				const auto [sin_pitch, cos_pitch] = GetSinCos(-pitch);
				return {cos_pitch * v1.x + sin_pitch * v1.z, v1.y, -sin_pitch * v1.x + cos_pitch * v1.z};
			}

			// 2D rotate (x,y) by angle radians: (x',y') = R(angle) * (x,y) and return only y value
			static constexpr float Rotate2D(float x, float y, float angle) noexcept
			{
				const auto [sin_angle, cos_angle] = GetSinCos(angle);
				// xr = cos_angle * x - sin_angle * y; // x value
				return sin_angle * x + cos_angle * y;
			}

			Parameters _parameters;

			filter::LowPass3d _lp_acc;
			filter::LowPass3d _lp_gyro;
			filter::LowPass3d _lp_gyro_bias;
			filter::LowPass _lp_angle_roll;
			filter::LowPass _lp_angle_pitch;
			filter::LowPass _lp_gamma_0;
			filter::LowPass _lp_gamma_pi;

			filter::HighPass _hp_omega_mod;
			filter::HighPass _hp_angle_roll;
			filter::HighPass _hp_angle_pitch;
			filter::HighPass _hp_yaw;

			filter::ScalarRLS _rls_nominal;
			filter::ScalarRLS _rls_rotated;

			// outputs
			MountingAngles _mounting_angles;

			// latched flags
			Latches _latches;

			ConvergenceFlags _convergence_flags;
		};
	}
}
