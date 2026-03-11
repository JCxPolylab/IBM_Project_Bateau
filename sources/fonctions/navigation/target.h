#pragma once

#include "../utility/myUtil.h"

#define MAX_TARGETS 100

namespace CATJ_target {
	class target {
		public:
		struct position_struct {
			float virtual_map_x; // Position X du target
			float virtual_map_y; // Position Y du target
		};

		target();
		~target();
		bool addTarget(float x, float y);
		bool removeTarget(int index);

		int getNumberOfTargets() const { return numberOfTargets; }
		float getTargetX(int index) const { return (index >= 0 && index < numberOfTargets) ? position[index].virtual_map_x : -1; }
		float getTargetY(int index) const { return (index >= 0 && index < numberOfTargets) ? position[index].virtual_map_y : -1; }

	private:
		int numberOfTargets;
		position_struct position[MAX_TARGETS];
	};
}