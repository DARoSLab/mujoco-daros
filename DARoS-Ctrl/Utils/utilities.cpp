/*!
 * @file Utilities.cpp
 * @brief Common utility functions
 */


#include <ctime>
#include <iomanip>
#include <iostream>

#include <utilities.h>


/*!
 * Write std::string to file with given name
 */
void writeStringToFile(const std::string& fileName,
                       const std::string& fileData) {
  FILE* fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    printf("Failed to fopen %s\n", fileName.c_str());
    throw std::runtime_error("Failed to open file");
  }
  fprintf(fp, "%s", fileData.c_str());
  fclose(fp);
}

/*!
 * Get the current time and date as a string
 */
std::string getCurrentTimeAndDate() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%c");
  return ss.str();
}

/*!
 * Todo: do something better to keep track of where we are relative to the
 * config directory
 */
std::string getConfigDirectoryPath() { return "../config/"; }

