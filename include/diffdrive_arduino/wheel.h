#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>  // for using std::string

// Wheel class represents a single robot wheel
class Wheel
{
public:
    std::string name = "";     // Name of the wheel (e.g., "left_wheel")
    int enc = 0;               // Raw encoder tick value
    double cmd = 0;            // Commanded velocity (rad/s)
    double pos = 0;            // Position of the wheel (radians)
    double vel = 0;            // Actual velocity of the wheel (rad/s)
    double eff = 0;            // Effort or torque applied (unused or optional)
    double velSetPt = 0;       // Desired velocity setpoint (rad/s)
    double rads_per_count = 0; // Conversion factor from encoder ticks to radians

    // Default constructor
    Wheel() = default;

    // Constructor with parameters
    Wheel(const std::string &wheel_name, int counts_per_rev);

    // Setup function to initialize name and ticks-to-radians conversion
    void setup(const std::string &wheel_name, int counts_per_rev);

    // Calculate angle based on encoder ticks and conversion factor
    double calcEncAngle();
};

#endif // DIFFDRIVE_ARDUINO_WHEEL_H
