#define DEBUG_VIFA

#include "CalibrationLooper.hpp"

using namespace std;

using namespace imu::allignment;

int main(int argc, const char* argv[])
{
	if (argc < 2) {
		cerr << "Please put filename to read!" << endl;
		return EXIT_FAILURE;
	}

	auto csvReader = CsvReader::Create(argv[1]);
	if (!csvReader)
		return EXIT_FAILURE;

	static constinit MountingOrientationEstimator mountingOrientationEstimator(MountingOrientationEstimator::FrequencyToPeriod(10.f));
	return CalibrationLooper::Run(*csvReader, mountingOrientationEstimator) ? EXIT_SUCCESS : EXIT_FAILURE;
}
