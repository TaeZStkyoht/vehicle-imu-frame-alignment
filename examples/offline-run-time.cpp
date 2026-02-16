#define DEBUG_VIFA

#include "../include/vehicle-imu-frame-alignment/MountingOrientationEstimator.hpp"

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>

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

static void printStatus(const MountingOrientationEstimator& mountingOrientationEstimator)
{
	std::cout << "RP conv: " << mountingOrientationEstimator.IsRollPitchConverged() << " Yaw conv: " << mountingOrientationEstimator.IsYawConverged()
			  << " Dir conv: " << mountingOrientationEstimator.IsDirectionConverged() << " Calibrated: " << mountingOrientationEstimator.IsCalibrated()
			  << "\n";
	std::cout << "roll (deg): " << mountingOrientationEstimator.Roll() * 180.0f / M_PIf
			  << " pitch (deg): " << mountingOrientationEstimator.Pitch() * 180.0f / M_PIf
			  << " yaw (deg): " << mountingOrientationEstimator.Yaw() * 180.0f / M_PIf << "\n";
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

	size_t iteration = 1;

	static constinit MountingOrientationEstimator mountingOrientationEstimator(MountingOrientationEstimator::FrequencyToPeriod(10.f));
	for (;; ++iteration) {
		const auto imuData = csvReader->Read();
		if (!imuData)
			break;

		mountingOrientationEstimator.FeedSample(*imuData);
		if (mountingOrientationEstimator.IsCalibrated()) {
			cout << "Mounting angle calculated after " << iteration << " iteration\n";

			printStatus(mountingOrientationEstimator);

			const auto rotationMatrix = mountingOrientationEstimator.GetRotationMatrixOfImuToVehicle();
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

			return EXIT_SUCCESS;
		}
	}

	cout << "rollPitchEntrance: " << mountingOrientationEstimator.rollPitchEntrance << '\n'
		 << "yawEntrance: " << mountingOrientationEstimator.yawEntrance << '\n'
		 << "dirEntrance: " << mountingOrientationEstimator.dirEntrance << '\n';

	cout << "rollPitchTotalPoint: " << mountingOrientationEstimator.rollPitchTotalPoint << '\n'
		 << "yawTotalPoint: " << mountingOrientationEstimator.yawTotalPoint << '\n'
		 << "dirTotalPoint: " << mountingOrientationEstimator.dirTotalPoint << '\n';

	printStatus(mountingOrientationEstimator);
	cerr << "Mounting angle could not be calculated! Iteration count: " << iteration << endl;
	return EXIT_FAILURE;
}
