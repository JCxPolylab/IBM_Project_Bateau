#include "errorManagement.h"

CATJ_error::error erreur;

void CATJ_error::error::printErrorLog()
{
	std::fstream f(this->path, std::ios::out | std::ios::trunc);
	f << "test";
}