#include "diffdrive_arduino/wheel.h" // Includes the class definition

#include <cmath> // For M_PI (π constant)


// Constructor with wheel name and encoder resolution
Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
  // Call the setup function to initialize values
  setup(wheel_name, counts_per_rev);
}


// Initialize wheel properties
void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name; // Assign wheel name
  rads_per_count = (2 * M_PI) / counts_per_rev; 
  // Compute how many radians per encoder tick
}


// Compute the wheel's angular position (in radians)
double Wheel::calcEncAngle()
{
  return enc * rads_per_count; 
  // Total ticks × radians per tick = angle
}
