#define DEBUG_VIFA

#include "CalibrationLooper.hpp"

#include <vector>

using namespace std;

using namespace imu::alignment;

struct RollPitchYaw final {
	float roll;
	float pitch;
	float yaw;
};

int main(int argc, const char* argv[])
{
	if (argc < 2) {
		puts("Please put filename to read!");
		return EXIT_FAILURE;
	}

	unsigned short calculatedCount{};

	{
		vector<RollPitchYaw> rollPitchYaws;

		{
			auto csvReader = CsvReader::Create(argv[1]);
			if (!csvReader)
				return EXIT_FAILURE;

			for (;; ++calculatedCount) {
				MountingOrientationEstimator mountingOrientationEstimator(MountingOrientationEstimator::FrequencyToPeriod(13.f));
				if (!CalibrationLooper::Run(*csvReader, mountingOrientationEstimator))
					break;
				puts("");
				rollPitchYaws.emplace_back(mountingOrientationEstimator.Roll() / constants::deg_to_rad,
										   mountingOrientationEstimator.Pitch() / constants::deg_to_rad,
										   mountingOrientationEstimator.Yaw() / constants::deg_to_rad);
			}
		}

		for (const auto& ele : rollPitchYaws)
			cout << '\n' << ele.roll << '\t' << ele.pitch << '\t' << ele.yaw;
	}

	cout << "\n\ncalculatedCount: " << calculatedCount << '\n';

	return calculatedCount > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
