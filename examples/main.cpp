#include "../include/vehicle-imu-frame-alignment/MountingOrientationEstimator.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

int main()
{
	ifstream inputFile("../../examples/full_drive_7104822000");
	if (!inputFile.is_open()) {
		cerr << "File could  not be opened!" << endl;
		return EXIT_FAILURE;
	}

	string line;
	getline(inputFile, line);

	MountingCalibrator mountingCalibrator(10.f);

	size_t iteration = 0;

	while (getline(inputFile, line)) {
		++iteration;

		float ax, ay, az, gx, gy, gz;
		{
			stringstream ss(line);
			long long timestamp;
			char comma;
			ss >> timestamp >> comma >> ax >> comma >> ay >> comma >> az >> comma >> gx >> comma >> gy >> comma >> gz;
		}

		mountingCalibrator.feedSample({ax, ay, az}, {gx, gy, gz});
		if (mountingCalibrator.flagCalibrated) {
			const auto rotationMatrix = mountingCalibrator.getRotationMatrix_IMUtoVehicle();

			cout << "Mounting angle calculated after " << iteration << " iteration\n";

			cout << "Roll: " << mountingCalibrator.roll * 180 / M_PI << ", Pitch: " << mountingCalibrator.pitch * 180 / M_PI
				 << ", Yaw: " << mountingCalibrator.yaw * 180 / M_PI << "\n";

			cout << "getRotationMatrix_IMUtoVehicle:\n"
				 << rotationMatrix.m[0][0] << ' ' << rotationMatrix.m[0][1] << ' ' << rotationMatrix.m[0][2] << "\n"
				 << rotationMatrix.m[1][0] << ' ' << rotationMatrix.m[1][1] << ' ' << rotationMatrix.m[1][2] << "\n"
				 << rotationMatrix.m[2][0] << ' ' << rotationMatrix.m[2][1] << ' ' << rotationMatrix.m[2][2] << "\n";

			return EXIT_SUCCESS;
		}
	}

	cerr << "Mounting angle could not be calculated! Iteration count: " << iteration << endl;
	return EXIT_FAILURE;
}
