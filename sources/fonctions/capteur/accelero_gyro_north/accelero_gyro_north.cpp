#include "accelero_gyro_north.h"

namespace {
	using namespace AJT_gyroAccelNorth; // valable seulement dans ce fichier ET seulement ici
}

spatial::spatial() {
	// Constructor
}

spatial::~spatial() {
	// Destructor
	disconnect();
}

bool spatial::connect(const char* port) {
	// Simulate connection logic
	std::cout << "Connecting to RPLIDAR on port: " << port << std::endl;
	return true; // Assume connection is always successful for this mock
}

void spatial::disconnect() {
	// Simulate disconnection logic
	std::cout << "Disconnecting RPLIDAR" << std::endl;
}

bool spatial::isConnected() const {
	// Simulate connection status
	return true; // Assume always connected for this mock
}