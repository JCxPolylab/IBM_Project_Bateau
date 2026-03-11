#include "Lidar.h"

namespace {
	using namespace AJT_lidar; // valable seulement dans ce fichier ET seulement ici
}

rplidar::rplidar() {
	// Constructor
}

rplidar::~rplidar() {
	// Destructor
	disconnect();
}

bool rplidar::connect(const char* port) {
	// Simulate connection logic
	std::cout << "Connecting to RPLIDAR on port: " << port << std::endl;
	return true; // Assume connection is always successful for this mock
}

void rplidar::disconnect() {
	// Simulate disconnection logic
	std::cout << "Disconnecting RPLIDAR" << std::endl;
}

bool rplidar::isConnected() const {
	// Simulate connection status
	return true; // Assume always connected for this mock
}

bool rplidar::startScan() {
	// Simulate starting the scan
	std::cout << "Starting RPLIDAR scan" << std::endl;
	return true; // Assume scan starts successfully
}

void rplidar::stopScan() {
	// Simulate stopping the scan
	std::cout << "Stopping RPLIDAR scan" << std::endl;
}

bool rplidar::getScanData(ScanData* data, size_t& count) {
	// Simulate getting scan data
	if (count < 1) return false; // No space to store data
	data[0].angle = 45.0f; // Example angle
	data[0].distance = 100.0f; // Example distance in cm
	data[0].quality = 15; // Example quality
	count = 1; // We have one scan data point
	return true;
}