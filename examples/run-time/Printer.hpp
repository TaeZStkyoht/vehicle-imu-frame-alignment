#pragma once

#define DEBUG_VIFA

#include "../../include/vehicle-imu-frame-alignment/MountingOrientationEstimator.hpp"

#include <iostream>

namespace Printer {
	inline void PrintEntranceAndStatus(const imu::alignment::MountingOrientationEstimator& mountingOrientationEstimator)
	{
		std::cout << "rollPitchEntrance: " << mountingOrientationEstimator.rollPitchEntrance << '\n'
				  << "yawEntrance: " << mountingOrientationEstimator.yawEntrance << '\n'
				  << "dirEntrance: " << mountingOrientationEstimator.dirEntrance << '\n';

		std::cout << "rollPitchTotalPoint: " << mountingOrientationEstimator.rollPitchTotalPoint << '\n'
				  << "yawTotalPoint: " << mountingOrientationEstimator.yawTotalPoint << '\n'
				  << "dirTotalPoint: " << mountingOrientationEstimator.dirTotalPoint << '\n';

		std::cout << "RP conv: " << mountingOrientationEstimator.IsRollPitchConverged() << " Yaw conv: " << mountingOrientationEstimator.IsYawConverged()
				  << " Dir conv: " << mountingOrientationEstimator.IsDirectionConverged() << " Calibrated: " << mountingOrientationEstimator.IsCalibrated()
				  << '\n';
		std::cout << "roll (deg): " << mountingOrientationEstimator.Roll() / imu::alignment::constants::deg_to_rad
				  << " pitch (deg): " << mountingOrientationEstimator.Pitch() / imu::alignment::constants::deg_to_rad
				  << " yaw (deg): " << mountingOrientationEstimator.Yaw() / imu::alignment::constants::deg_to_rad << '\n';
	}

	inline void PrintRotationMatrix(const imu::alignment::MountingOrientationEstimator& mountingOrientationEstimator)
	{
		const auto rotationMatrix = mountingOrientationEstimator.GetRotationMatrixOfImuToVehicle();
		std::cout << "GetRotationMatrixOfImuToVehicle:\n"
				  << rotationMatrix.m[0][0] << ' ' << rotationMatrix.m[0][1] << ' ' << rotationMatrix.m[0][2] << '\n'
				  << rotationMatrix.m[1][0] << ' ' << rotationMatrix.m[1][1] << ' ' << rotationMatrix.m[1][2] << '\n'
				  << rotationMatrix.m[2][0] << ' ' << rotationMatrix.m[2][1] << ' ' << rotationMatrix.m[2][2] << '\n';
	}
}
