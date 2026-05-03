#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Header file containing function declarations, world paths, and sensor declarations.
 */

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>

using namespace std;
using namespace webots;

#define MAX_SPEED          10
#define MEDIUM_SPEED        5
#define SLOW_SPEED          1

#define DEGREE_TOLERANCE    5      // [=] degrees (more intuitive)
#define DISTANCE_TOLERANCE  0.1    // [=] meters

#define WHEELS_DISTANCE     0.3606 // [=] meters
#define WHEEL_RADIUS        0.0825 // [=] meters

#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DISTANCE_SENSOR 16

// World Path Definitions
// Note: Only path1 is actively used. Waypoints flagged with .001 trigger a rescue spin.
// Coordinate axes vary per world — see inline comments for axis orientation.

// World 1: left = y, up = x
const float world1_path1[][2] = {
    {-2, 1.3},
    {-5.5, -3},
    {-2, -3.5},
    {-3.3, -12.5},
    {-5, -12.3},
    {-6, -14.5},        // First rescue point
    {-6.501, -13.501},  // Flagged: triggers rescue spin
    {-11, -13},         // Last rescue point
    {-6.101, -13.501},
    {-5.5, -12},
    {-4, -12},
    {-2.7, -3},
    {-6, -2.5},
    {-2.5, 1.5},
    {-2.5, 3}
};
const float world1_path2[][2] = {
    {-4.0, -15.8},  
};
const float world1_path3[][2] = {
    {-6.5, -7.2},
    {-4, -7.0},
    {-4.5, -4.0},
    {-6, -3.0},
    {-6.8, 1.2},
};

// World 2: down = y, left = x
const float world2_path1[][2] = {
    // {-4,-4.3},
    // {-4, -6},
    {-1, -5},
    {3.2, -7},
    {3.2, -4},
    {8.1, -4},
    {8.1, -7},
    {13.7, -7},
    {13.701, -5.501},       // First rescue point
    {14, -3.7},
    {14.001, -7.101},       // Flagged: triggers rescue spin
    {8, -7},
    {8, -4},
    {3.4, -4},
    {3.4, -7},
    {-1, -5},
    {-3, -5},
};
const float world2_path2[][2] = {};
const float world2_path3[][2] = {};

// World 3: right = x, up = y
const float world3_path1[][2] = {
    {0,-7.53},
    {0,-6.5}, 
    {3.5,-6},
    {3.5, -3.5}, 
    {9.3, -3.5},  // 5
    {9.3, -4.8},
    {14, -4.8},
    {14.001, -4.801},
    {14.5, -10},
    {14.501, -10.001},
    {14,-5.3},
    {9.3, -5.3}, //12
    {9.3, -3.8},
    {3.5, -3.8},
    {3.5, -6.5},
    {0,-6.5}, //16
    {0, -7.53},
    {-2.5,-7.5},
    
};


const float world3_path2[][2] = {};
const float world3_path3[][2] = {};

// World 4: left = x, down = y
const float world4_path1[][2] = {
    {1.8, 1.3}, 
    {-3, 2.6}, 
    {-3, -.3},
    {-12,-1.3},
    {-12.3, .3},
    {-13.5, .3},
    {-13.5, 1.5},       // First rescue point
    {-14.301, 1.501},   // Flagged: triggers rescue spin
    {-14.3, 3.5},       // Last rescue point
    {-13.701, 0.301},
    {-11.4, .3},
    {-11.4, -1},
    {-2, -.2},
    {-2, 3},
    {1.8, 1.3},
    {3.7,1.3}
};
const float world4_path2[][2] = {};
const float world4_path3[][2] = {};

// World 5: down = y
const float world5_path1[][2] = {
    {1.85, 1.33},
    {1.6, 5.7},
    {-2, 5.4},
    {-3.5, 4},
    {-5.8, 4},
    {-6, 7.5},
    {-8, 6},
    {-15, 5.8},         // First rescue point
    {-14.001, 5.501},   // Flagged: triggers rescue spin
    {-13.8, 0},         // Last rescue point
    {-14.001, 5.801},
    {-8, 6},
    {-6, 7.5},
    {-5.8, 4},
    {-3.5, 4},
    {-2, 5.4},
    {2.3, 5.7},
};
const float world5_path2[][2] = {};
const float world5_path3[][2] = {};

// World 6
const float world6_path1[][2] = {
     {.5,0},
    {.5,3},
    {-3, 3},
    {-3, 4},
    {-5.25, 4},
    {-5.25, 6.5},
    {-14.5, 6.0},  // waypoint 7
    {-14.501, 5.801},  // First rescue point
    {-14.5,1.8},
    {-14.001, 1.801},// Last rescue point
    {-14.5, 6},
    {-5.25, 6.5},
    {-5.25, 4},
    {-3, 4},
    {-3, 3},
    {.5,3},
    {.7,0},
    {3.5,0},
};
const float world6_path2[][2] = {};
const float world6_path3[][2] = {};

// World 7
const float world7_path1[][2] = {
    {-1.6,0},
    {-9, -.5},
    {-9, 2},
    {-13.3, 2.3},       // First rescue point
    {-14.501, 6.101},   // Last rescue point
    {-13.101, 2.501},
    {-9, 2},
    {-9, -.5},
    {3.2,.5},
};
const float world7_path2[][2] = {};
const float world7_path3[][2] = {};

// World 8
const float world8_path1[][2] = {
    {-2,1},
    {-6.7, 4},
    {-7, 4.5},
    {-9, 6},
    {-9, 7.5},
    {-17, 7.5},         // First rescue point
    {-17.401, 7.101},   // Flagged: triggers rescue spin
    {-17.601, 7.901},
    {-9.3, 8},
    {-9.3, 6},
    {-7, 5},
    {-7, 4},
    {-1.7,1},
};
const float world8_path2[][2] = {};
const float world8_path3[][2] = {};

// World 9: right = positive x, up = positive y
const float world9_path1[][2] = {
     {-1.5, -5.0},
     {3.5, -8},
     {6, -9},
     {6, -11},
     {6.7, -11.3},
     {15, -11.3},
     {15.001, -12.001},     // First rescue point
     {14.501, -5.001},  // Last rescue point
     {14.5,-11.601},
     {7.8, -11.4},
     {5.5, -13},
     {5.5, -8},
     {3, -8},
     {-1.9, -5.5},
     {-2.7, -5.3},
};
const float world9_path2[][2] = {};
const float world9_path3[][2] = {};

// World 10: x = up
const float world10_path1[][2] = {
    {-2, 1.3},
    {-2, 3},
    {-9, 4},
    {-9.3, 1.7},
    {-8.3, 0},
    {-9, -3},
    {-6.2, -3},
    {-6.6, -5},
    {-9.3, -5},
    {-10.7, -13},
    {-9, -13.2},        // First rescue point
    {-9.101, -14.501},  // Flagged: triggers rescue spin
    {-8, -14.9},        // Last rescue point
    {-11.101, -14.301},
    {-9.3, -5.5},
    {-7, -5.5},
    {-6.7, -4},
    {-9, -4},
    {-8.5, -.5},
    {-9.5, .9},
    {-9.2, 2.7}
};
const float world10_path2[][2] = {
    {-2, .5},
    {-2, 2.5},
    {-9.7, .5},
    {-9.3, -.8},
    {-7.5, -2.5},
    {-7.5, -5},
    {-4.5, -5},
    {-4, -6.6},
    {-6.5, -7.5},
    {-4.8, -15},
    {-2.7, -15},        // First rescue point
    {-2.701, -15.501},  // Flagged: triggers rescue spin
    {-1.4, -15.5},      // Last rescue point
    {-4.801, -15.001},
    {-6.5, -7.5},
    {-4, -6.6},
    {-4.5, -5},
    {-7.5, -5},
    {-7.5, -2.5},
    {-9.3, -.8},
    {-9.7, .5},
    {-9.7, 2}
};
const float world10_path3[][2] = {
    {-2, .5},
    {-2, 2.5},
    {-9.7, .5},
    {-9.3, -.8},
    {-7.5, -2.5},
    {-7.5, -5},
    {-4.5, -5},
    {-4, -6.6},
    {-6.5, -7.5},
    {-4.8, -15},
    {-2.7, -15},        // First rescue point
    {-2.701, -15.501},  // Flagged: triggers rescue spin
    {-1.4, -15.5},      // Last rescue point
    {-4.801, -15.001},
    {-6.5, -7.5},
    {-4, -6.6},
    {-4.5, -5},
    {-7.5, -5},
    {-7.5, -2.5},
    {-9.3, -.8},
    {-9.7, .5},
    {-9.7, 2}
};


// Robot controller class inheriting from the Webots Robot base class
class MyRobot : public Robot {
public:
    /**
     * @brief Constructor: initializes all sensors, motors, and state variables.
     */
    MyRobot();

    /**
     * @brief Destructor: stops motors and disables all sensors cleanly.
     */
    ~MyRobot();

    /**
     * @brief Main control loop: detects the world via camera color, then executes the corresponding path.
     */
    void run();

    /**
     * @brief Prints the current x, y, theta odometry values to standard output.
     */
    void print_odometry();
    
    /**
     * @brief Decides whether to turn toward the goal or drive forward, based on angular error.
     */
    void go_route();

    /**
     * @brief Returns true if the robot is within DISTANCE_TOLERANCE of the current goal.
     */
    bool goal_reached();
    
    /**
     * @brief Rotates the robot to a target angle using wheel odometry.
     * @param target_angle_rad Target orientation in radians.
     */
    void rotate_to_angle(double target_angle_rad);

    /**
     * @brief Drives the robot to the starting corner using wall-following and distance sensors.
     */
    void go_to_start();
    
    /**
     * @brief Physically explores the environment to identify worlds 1, 8, or 10 using distance sensors.
     * @return Identified world number.
     */
    int routineRouge();
    
    /**
     * @brief Physically explores the environment to identify worlds 4, 5, 6, or 7 using distance sensors.
     * @return Identified world number.
     */
    int routineVerte();

    /**
     * @brief Legacy path follower (obsolete, replaced by follow_path with 001 flag support).
     */
    void follow_path_begining(const float path[][2], int length);

    /**
     * @brief Drives the robot through a sequence of waypoints.
     *        Waypoints ending in .001 trigger a rescue spin (turn_full_circle).
     * @param path 2D array of [x, y] waypoints.
     * @param length Number of waypoints.
     */
    void follow_path(const float path[][2], int length);

    /**
     * @brief Counts the percentage of green pixels in the forward camera image.
     * @return Percentage of green pixels (0.0 to 100.0).
     */
    float compute_green_percentage();
    
    /**
     * @brief Rotates the robot to a target angle using the compass sensor.
     * @param target_angle_rad Target orientation in radians.
     */
    void rotate_to_compass_angle(double target_angle_rad);

    /**
     * @brief Performs a full 360-degree spin to signal a rescue location.
     */
    void turn_full_circle();
    
    /**
     * @brief Returns true if the thousandths digit of the absolute value of a float is 1.
     *        Used to detect the .001 rescue spin flag in waypoint coordinates.
     */
    bool ends_in_001(float value);


private:
    int _time_step;     // Simulation step duration in milliseconds
    int world;          // Currently identified world number (1–10)

    double _x_offset;       // Odometry X offset for resetting position
    double _y_offset;       // Odometry Y offset for resetting position
    double _theta_offset;   // Odometry orientation offset for resetting heading

    double _left_speed, _right_speed;  // Current motor velocities
        
    float _x, _y, _x_goal, _y_goal;    // Current position and goal position in meters
    float _theta, _theta_goal;          // Current orientation and goal orientation in radians
    
    float _path_goal[5][2] = {{-5,0},{-5, 4},{-13,4},{-13,0},{-18,0}};  // Legacy path (unused)
    int _active_point, _total_points = 5;  // Legacy path tracking variables (unused)
    
    float _sr, _sl;  // Accumulated distance traveled by right and left wheels in meters

    // Wheel position sensors (encoders)
    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;

    Camera *_forward_camera;  // Forward-facing camera for world color detection

    DistanceSensor* _distance_sensor[NUM_DISTANCE_SENSOR];  // Array of 16 distance sensors
    const char *ds_name[NUM_DISTANCE_SENSOR] = {
        "ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7",
        "ds8", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"
    };
    
    GPS *_gps;              // GPS sensor (available but not used for navigation)
    Compass *_my_compass;   // Compass sensor for absolute heading

    // Motors
    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;

    /**
     * @brief Resets odometry state so the robot treats its current position as the origin.
     * @param use_compass If true, initializes heading from the compass instead of setting to zero.
     */
    void reset_odometry(bool use_compass);

    /**
     * @brief Updates _x, _y, _theta using differential drive odometry from wheel encoders.
     * @param use_compass If true, overrides heading with the compass reading.
     */
    void compute_odometry(bool use_compass=false);
        
    /**
     * @brief Reads the compass and returns the robot's heading in degrees.
     * @return Heading in degrees.
     */
    double convert_bearing_to_degrees();
    
    /**
     * @brief Reads the compass and returns the robot's heading in radians.
     * @return Heading in radians.
     */
    double convert_bearing_to_radians();
    
    /**
     * @brief Converts an angle from radians to degrees.
     */
    double convert_rad_to_deg(double rad);

    /**
     * @brief Converts an angle from degrees to radians.
     */
    double convert_deg_to_rad(double deg);
    
    /**
     * @brief Converts raw encoder ticks to meters using the wheel radius.
     * @param tics Raw encoder value.
     * @return Distance in meters.
     */
    float encoder_tics_to_meters(float tics);
    
    /**
     * @brief Returns the straight-line (Euclidean) distance from the robot to the current goal.
     */
    float compute_distance_goal();

    /**
     * @brief Returns the angular error (in radians) between the robot's heading and the direction to the goal.
     */
    float compute_angle_goal();
    
    // Low-level movement functions
    void head_goal();     // Rotates the robot toward the goal
    void move_forward();  // Drives forward toward the goal
    void stop();          // Halts both wheels
        
};

#endif

