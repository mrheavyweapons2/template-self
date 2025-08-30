/**
 * @file liasonService.hpp
 * @author Jeremiah Nairn
 * @ingroup cpp-auxilium-template
 *
 * @brief Contains headers for a variety of classes that are used to
 * the robots brain and other systems that are not directly supposed to 
 * communicate with the v5 brain. 
 */

#ifndef LIASONSERVICE_HPP
#define LIASONSERVICE_HPP

//include neccessary services
#include <string>
#include <fstream>

class fileLogger {
	private:
		std::ofstream logFile;

	public:
		fileLogger(std::string filename, std::string header);
		void logData(std::string data);
};

#endif // LIASONSERVICE_HPP
