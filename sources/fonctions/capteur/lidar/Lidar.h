#pragma once

#include "../../utility/myUtil.h"

namespace AJT_lidar {

	class rplidar {
		public:
		rplidar();
		~rplidar();
		bool connect(const char* port);
		void disconnect();
		bool isConnected() const;
		bool startScan();
		void stopScan();
		struct ScanData {
			float angle;
			float distance;
			int quality;
		};
		bool getScanData(ScanData* data, size_t& count);
	};
}