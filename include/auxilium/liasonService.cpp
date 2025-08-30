//declaring neccessary includes
#include <fstream>

//declaring the header
#include "auxilium/liasonService.hpp"


//constructor that opens the log file and adds the header,
//filename example would be "/usd/logfile.csv" (you could literally copy and paste this in and it would work)
fileLogger::fileLogger(std::string filename, std::string header) {
	logFile.open(filename);
	if (logFile.is_open()) {
		logFile << header << "\n"; //header
	}
}
		
//simple function that logs data to the file
void fileLogger::logData(std::string data) {
	if (logFile.is_open()) {
		logFile << data << "\n"; //logged data
	}
}