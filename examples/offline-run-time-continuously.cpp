#define DEBUG_VIFA

#include "../include/vehicle-imu-frame-alignment/MountingOrientationEstimator.hpp"

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

#include <cstdint>

using namespace std;

using namespace imu;
using namespace data;
using namespace allignment;

class CsvReader final {
public:
	static optional<CsvReader> Create(const char* path)
	{
		if (CsvReader csvReader(path); csvReader._inputFile.is_open()) {
			csvReader._inputFile.ignore(numeric_limits<streamsize>::max(), '\n'); // Skip first line of csv
			return csvReader;
		}

		cerr << "File could  not be opened!" << endl;
		return {};
	}

	optional<ImuData> Read()
	{
		optional<ImuData> imuData;

		if (string line; getline(_inputFile, line)) {
			stringstream ss(line);
			long long timestamp;
			char comma;
			imuData.emplace();
			ss >> timestamp >> comma >> imuData->acc.x >> comma >> imuData->acc.y >> comma >> imuData->acc.z >> comma >> imuData->gyro.x >> comma >>
				imuData->gyro.y >> comma >> imuData->gyro.z;

			// // imuData->acc.z *= -1;
			// // imuData->gyro.z *= -1;

			// imuData->acc.y *= -1;
			// imuData->gyro.y *= -1;

			// float tempAcc = imuData->acc.x;
			// imuData->acc.x = -imuData->acc.z;
			// imuData->acc.z = -tempAcc;

			// float tempGyro = imuData->gyro.x;
			// imuData->gyro.x = -imuData->gyro.z;
			// imuData->gyro.z = -tempGyro;
		}

		return imuData;
	}

private:
	explicit CsvReader(const char* path) : _inputFile(path)
	{
	}

	ifstream _inputFile;
};

static optional<Mat3> TryCalibrate(CsvReader& csvReader)
{
	MountingOrientationEstimator mountingOrientationEstimator(MountingOrientationEstimator::FrequencyToPeriod(10.f));

	for (;;) {
		const auto imuData = csvReader.Read();
		if (!imuData)
			break;

		mountingOrientationEstimator.FeedSample(*imuData);
		if (mountingOrientationEstimator.IsCalibrated()) {
			auto rotationMatrix = mountingOrientationEstimator.GetRotationMatrixOfImuToVehicle();
			cout << "GetRotationMatrixOfImuToVehicle:\n"
				 << rotationMatrix.m[0][0] << ' ' << rotationMatrix.m[0][1] << ' ' << rotationMatrix.m[0][2] << "\n"
				 << rotationMatrix.m[1][0] << ' ' << rotationMatrix.m[1][1] << ' ' << rotationMatrix.m[1][2] << "\n"
				 << rotationMatrix.m[2][0] << ' ' << rotationMatrix.m[2][1] << ' ' << rotationMatrix.m[2][2] << "\n";

			cout << "rollPitchEntrance: " << mountingOrientationEstimator.rollPitchEntrance << '\n'
				 << "yawEntrance: " << mountingOrientationEstimator.yawEntrance << '\n'
				 << "dirEntrance: " << mountingOrientationEstimator.dirEntrance << '\n';

			cout << "rollPitchTotalPoint: " << mountingOrientationEstimator.rollPitchTotalPoint << '\n'
				 << "yawTotalPoint: " << mountingOrientationEstimator.yawTotalPoint << '\n'
				 << "dirTotalPoint: " << mountingOrientationEstimator.dirTotalPoint << '\n';

			return rotationMatrix;
		}
	}

	cout << "rollPitchEntrance: " << mountingOrientationEstimator.rollPitchEntrance << '\n'
		 << "yawEntrance: " << mountingOrientationEstimator.yawEntrance << '\n'
		 << "dirEntrance: " << mountingOrientationEstimator.dirEntrance << '\n';

	cout << "rollPitchTotalPoint: " << mountingOrientationEstimator.rollPitchTotalPoint << '\n'
		 << "yawTotalPoint: " << mountingOrientationEstimator.yawTotalPoint << '\n'
		 << "dirTotalPoint: " << mountingOrientationEstimator.dirTotalPoint << '\n';

	return {};
}

int main(int argc, const char* argv[])
{
	if (argc < 2) {
		cerr << "Please put filename to read!" << endl;
		return EXIT_FAILURE;
	}

	auto csvReader = CsvReader::Create(argv[1]);
	if (!csvReader)
		return EXIT_FAILURE;

	vector<Mat3> rotationMatrices;

	for (;;) {
		auto rotationMatrix = TryCalibrate(*csvReader);
		if (!rotationMatrix)
			break;
		rotationMatrices.push_back(move(*rotationMatrix));
	}

	Mat3 averageRotationMatrix{};

	for (const auto& rotationMatrix : rotationMatrices)
		for (uint8_t i = 0; i < 3; ++i)
			for (uint8_t j = 0; j < 3; ++j)
				averageRotationMatrix.m[i][j] += rotationMatrix.m[i][j];

	const auto rotationMatricesSize = static_cast<float>(rotationMatrices.size());
	for (uint8_t i = 0; i < 3; ++i)
		for (uint8_t j = 0; j < 3; ++j)
			averageRotationMatrix.m[i][j] /= rotationMatricesSize;

	cout << "Average Rotation Matrix:\n"
		 << averageRotationMatrix.m[0][0] << ' ' << averageRotationMatrix.m[0][1] << ' ' << averageRotationMatrix.m[0][2] << "\n"
		 << averageRotationMatrix.m[1][0] << ' ' << averageRotationMatrix.m[1][1] << ' ' << averageRotationMatrix.m[1][2] << "\n"
		 << averageRotationMatrix.m[2][0] << ' ' << averageRotationMatrix.m[2][1] << ' ' << averageRotationMatrix.m[2][2] << "\n"
		 << endl;

	return EXIT_FAILURE;
}
