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

/**
 * @class fileLogger
 * @brief Class that handles logging data into a a file,
 * allowing for easy tracking of robot performance and behavior.
 * The intended use case is to use a CSV file format for the logs,
 * making it easy to analyze the data with spreadsheet software, and/or
 * placing the data into desmos or any other graph sheet.
 */
class fileLogger {
	private:
        //file stream for logging
		std::ofstream logFile;

	public:
        /**
         * @brief Constructor that initializes the log file and writes the header.
         * @param filename The name of the file to log data to.
         * @param header The header line to write to the file.
         */
		fileLogger(std::string filename, std::string header);

        /**
         * @brief Logs data to the CSV file.
         * @param data The data to log (if using a CSV, commas separate values)
         */
		void logCSVData(std::string data);
};

#endif // LIASONSERVICE_HPP
