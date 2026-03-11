#pragma once

#include "../utility/myUtil.h"
#include "../capteur/camera/camera.h"
#include "../capteur/accelero_gyro_north/accelero_gyro_north.h"
#include "../capteur/lidar/Lidar.h"
#include "../navigation/target.h"

namespace CATJ_navigation {
	class Navigation {
	public:
		Navigation();
		~Navigation();
		bool navigateToTarget(const CATJ_target::target& targ, int targetIdx);
		void updatePosition();
		void avoidObstacles();
	private:
		struct position_struct {
			float Magn_orientation; // Orientation en degr�s par rapport au nord magn�tique
			float targ_orientation; // Orientation en degr�s par rapport � la cible active
			float vitesse; // Vitesse actuelle du robot
		};
		position_struct position;
		// Internal state and helper functions
	};
}