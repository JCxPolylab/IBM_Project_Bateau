#pragma once

#include "../../utility/myUtil.h"

namespace AJT_gyroAccelNorth {

	class spatial {
		public:
		spatial();
		~spatial();
		bool connect(const char* port);
		void disconnect();
		bool isConnected() const;
	};
}