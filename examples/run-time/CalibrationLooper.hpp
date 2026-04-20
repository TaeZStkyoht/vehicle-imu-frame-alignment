#pragma once

#include "CsvReader.hpp"
#include "Printer.hpp"

#include <iostream>

namespace CalibrationLooper {
	inline bool Run(CsvReader& csvReader, imu::alignment::MountingOrientationEstimator& mountingOrientationEstimator)
	{
		size_t iteration{};

		for (;;) {
			{
				const auto imuData = csvReader.Read();
				if (!imuData) [[unlikely]] {
					if (imuData.error() == CsvReader::ReadStatus::RS_SKIP)
						continue;
					break;
				}

				mountingOrientationEstimator.FeedSample(*imuData);
			}
			++iteration;
			if (mountingOrientationEstimator.IsCalibrated()) [[unlikely]] {
				std::cout << "Mounting angle calculated after " << iteration << " iteration\n";
				Printer::PrintEntranceAndStatus(mountingOrientationEstimator);
				Printer::PrintRotationMatrix(mountingOrientationEstimator);
				return true;
			}
		}

		std::cerr << "Mounting angle could not be calculated! Iteration count: " << iteration << '\n';
		Printer::PrintEntranceAndStatus(mountingOrientationEstimator);
		return false;
	}
}
