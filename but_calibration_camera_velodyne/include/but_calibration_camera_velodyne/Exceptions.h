/*
 * Exceptions.h
 *
 *  Created on: 10.1.2014
 *      Author: ivelas
 */

#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

#include <stdexcept>
#include <string>

namespace but_calibration_camera_velodyne
{

class NotImplementedException : public std::runtime_error
{
public:
  NotImplementedException(std::string what) :
      std::runtime_error("Not Implemented: " + what + ".")
  {
  }
};

};

#endif /* EXCEPTIONS_H_ */
