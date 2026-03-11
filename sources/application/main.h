/* 
 * Build sans ORT
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_ORT=OFF
cmake --build build -j
*/

/* 
 * Build avec ORT
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_ORT=ON
cmake --build build -j
*/

/* run : ./build/CATJ_ibmRobotProject */
#include "../fonctions/utility/myUtil.h"
#include "../fonctions/utility/errorManagement.h"
#include "../fonctions/capteur/camera/camera.h"
#include "../fonctions/navigation/target.h"
#include <iostream>
#include <ostream>
#include <fstream>