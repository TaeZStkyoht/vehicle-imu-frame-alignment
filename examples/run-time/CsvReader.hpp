#pragma once

#include "../../include/vehicle-imu-frame-alignment/MountingOrientationEstimator.hpp"

#include <expected>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>

class CsvReader final {
public:
	enum class ReadStatus : bool {
		RS_SKIP,
		RS_EXIT,
	};

	static std::optional<CsvReader> Create(const char* path)
	{
		std::optional<CsvReader> csvReader = CsvReader(path);
		if (!csvReader->_inputFile.is_open()) {
			puts("File could not be opened!");
			csvReader.reset();
		}
		return csvReader;
	}

	std::expected<imu::data::ImuData, ReadStatus> Read()
	{
		if (std::string line; getline(_inputFile, line)) {
			if (line.contains("timestamp_ms"))
				return std::unexpected(ReadStatus::RS_SKIP);

			std::stringstream ss(std::move(line));
			long long timestamp;
			char comma;
			imu::data::ImuData imuData;
			ss >> timestamp >> comma >> imuData.acc.x >> comma >> imuData.acc.y >> comma >> imuData.acc.z >> comma >> imuData.gyro.x >> comma >>
				imuData.gyro.y >> comma >> imuData.gyro.z;
			return imuData;
		}

		return std::unexpected(ReadStatus::RS_EXIT);
	}

private:
	explicit CsvReader(const char* path) : _inputFile(path)
	{
	}

	std::ifstream _inputFile;
};
