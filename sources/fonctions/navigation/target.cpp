#include "target.h"

CATJ_target::target::target() {
	// Initialisation du target
	this->numberOfTargets = 0; // Aucun target au départ
	std::cout << "Target initialized\n";
}

CATJ_target::target::~target() {
	// Libération des ressources
	std::cout << "Target destroyed\n";
}

bool CATJ_target::target::addTarget(float x, float y) 
{
	if (numberOfTargets < MAX_TARGETS) 
	{
		position[this->numberOfTargets].virtual_map_x = x;
		position[this->numberOfTargets].virtual_map_y = y;
		this->numberOfTargets++;
		std::cout << "Added target at (" << x << ", " << y << ")\n";
		return true;
	} else {
		std::cout << "Cannot add more targets, limit reached\n";
		return false;
	}
}


bool CATJ_target::target::removeTarget(int index) 
{
	if (index >= 0 && index < numberOfTargets) 
	{
		for (int i = index; i < numberOfTargets - 1; i++) 
		{
			position[i] = position[i + 1];
		}
		this->numberOfTargets--;
		std::cout << "Removed target at index " << index << "\n";
		return true;
	} else {
		std::cout << "Invalid index for removing target\n";
		return false;
	}
}
