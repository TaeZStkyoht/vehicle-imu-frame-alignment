#define DEBUG_VIFA

#include "CalibrationLooper.hpp"

using namespace std;

using namespace imu::allignment;

static bool TryCalibrate(CsvReader& csvReader)
{
	MountingOrientationEstimator mountingOrientationEstimator(MountingOrientationEstimator::FrequencyToPeriod(13.f));
	return CalibrationLooper::Run(csvReader, mountingOrientationEstimator);
}

int main(int argc, const char* argv[])
{
	if (argc < 2) {
		cerr << "Please put filename to read!" << endl;
		return EXIT_FAILURE;
	}

	unsigned short calculatedCount{};

	{
		auto csvReader = CsvReader::Create(argv[1]);
		if (!csvReader)
			return EXIT_FAILURE;

		for (;; ++calculatedCount) {
			const bool calibrationResult = TryCalibrate(*csvReader);
			cout << endl;
			if (!calibrationResult)
				break;
		}
	}

	cout << "calculatedCount: " << calculatedCount << endl;
	return calculatedCount > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
