#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A header file containing function declarations, world paths, and sensor declarations
 *
 * @author  Julien Vollet, Devon Salgado
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

#define DEGREE_TOLERANCE    5      //[=] degrees (more intuitive)
#define DISTANCE_TOLERANCE  0.1    //[=] meters

#define WHEELS_DISTANCE     0.3606 //[=] meters
#define WHEEL_RADIUS        0.0825 //[=] meters

#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DISTANCE_SENSOR 16

//  World Path Definitions Note: Only first paths were used. 001 flag used in points after the rescue points

//  left is y 
//  up is x
//done
const float world1_path1[][2] = {
    {-2, 1.3},
    {-5.5, -3},
    {-2, -3.5},
    {-3.3, -12.5},
    {-5, -12.3},
    {-6, -14.5}, //first
    {-6.501, -13.501},
    {-11, -13},//last
    {-6.101, -13.501},
    {-5.5, -12},
    {-4, -12},
    {-2.7, -3},
    {-6, -2.5},
    {-2.5, 1.5},
    {-2.5,3}
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

//down is y
// left is x
//done
const float world2_path1[][2] = {
    {-4,-4.3},
    {-4, -6},
    {-1, -6},
    {3, -8},
    {3, -4.5},
    {8.5, -4.5},
    {8.5, -6},
    {13.7, -6},
    {13.7, -5.5},//first
    {13.701, -6.501},//last
    {13.701, -6.101},
    {8.5, -6},
    {8.5, -4.5},
    {3, -4.5},
    {3, -8},
    {-1, -6},
    {-3, -6},
    
};
const float world2_path2[][2] = {
   
};
const float world2_path3[][2] = {
    
};

//right is x
//up is y
//done
const float world3_path1[][2] = {
    {-4.22,-4.5},
    {-4, -10.2}, 
    {1.7, -10.2}, 
    {1.7, -12}, 
    {3.5, -12}, 
    {3.5, -9},
    {5, -9},
    {5, -13},
    {14.25, -13},
    {14.25, -12}, //first green
    {13.401, -11.001},
    {14.25, -7}, //last green
    {14.201, -10.001},
    {14.2, -13},
    {5.5, -13},
    {5.5, -9},
    {4.3, -9},
    {4.3, -12},
    {2.4, -12},
    {2.4, -10.2},
    {-2.25, -10.2}
};

const float world3_path2[][2] = {
  
};

const float world3_path3[][2] = {
    
};

//left is x
// down is y
//done
const float world4_path1[][2] = {
    {1.8, 1.3}, 
    {-3, 2.6}, 
    {-3, -.3},
    {-12,-1.3},
    {-12.3, .3},
    {-13.5, .3},
    {-13.5, 1.5},// first
    {-14.301, 1.501},
    {-14.3, 3.5}, //last green
    {-13.701, 0.301},
    {-11.4, .3},
    {-11.4, -1},
    {-2, -.2},
    {-2, 3},
    {1.8, 1.3},
    {3.7,1.3}
};

const float world4_path2[][2] = {

};

const float world4_path3[][2] = {
   
};

//down is y
//done
const float world5_path1[][2] = {
    {1.85, 1.33},
    {1.6, 5.7},
    {-2, 5.4},
    {-3.5, 4},
    {-5.8, 4},
    {-6, 7.5},
    {-8, 6},
    {-15, 5.8}, //first
    {-14.001, 5.501},
    {-13.8, 0}, //last
    {-14.001, 5.801},
    {-8, 6},
    {-6, 7.5},
    {-5.8, 4},
    {-3.5, 4},
    {-2, 5.4},
    {2.3, 5.7},
    
};

const float world5_path2[][2] = {
    
};

const float world5_path3[][2] = {
    
};

//done
const float world6_path1[][2] = {
    {-1.5,0},
    {-1.5,1},
    {-3, 1},
    {-3, 0},
    {-6, -.5},
    {-6.5, 3.3},
    {-12, 2.7},
    {-12, 1.5},
    {-15, 1.5}, //first
    {-14.001, -2.101}, //last
    {-14.001, 1.801},
    {-12, 1.8},
    {-12, 3.3},
    {-6.5, 3},
    {-6, 0},
    {-3, 0},
    {-3, 1},
    {2.1,.7},
};

const float world6_path2[][2] = {
  
};

const float world6_path3[][2] = {
    
};

//done
const float world7_path1[][2] = {
    {-1.6,0},
    {-9, -.5},
    {-9, 2},
    {-13.3, 2.3}, //first
    {-14.501, 6.101}, //last
    {-13.101, 2.501},
    {-9, 2},
    {-9, -.5},
    {3.2,.5},
};

const float world7_path2[][2] = {
   
};

const float world7_path3[][2] = {
   
    
};

//done
const float world8_path1[][2] = {
    {-2,1},
    {-6.7, 4},
    {-7, 4.5},
    {-9, 6},
    {-9, 7.5},
    {-17, 7.5},//first
    {-17.401, 7.101}, // last green
    {-17.601, 7.901},
    {-9.3, 8},
    {-9.3, 6},
    {-7, 5},
    {-7, 4},
    {-1.7,1},

};

const float world8_path2[][2] = {
  
};

const float world8_path3[][2] = {
   
};

//right is pos x
//up is pos y
//done
const float world9_path1[][2] = {
     {-4.3, -4.1},
     {-1.9, -5.5},
     {3, -8},
     {4.8, -8},
     {4.8, -13},
     {6.7, -11.3},
     {14.5, -11.3}, //first
     {13.501, -6.101}, //last
     {13.501,-11.601},
     {7.8, -11.4},
     {5.5, -13},
     {5.5, -8},
     {3, -8},
     {-1.9, -5.5},
     {-2.7, -5.3},
};

const float world9_path2[][2] = {
   
};

const float world9_path3[][2] = {

};

//x is up
//done
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
    {-9, -13.2}, // first 
    {-9.101, -14.501},
    {-8, -14.9}, //last
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
    {-2.7, -15}, //first green
    {-2.701, -15.501},
    {-1.4, -15.5}, // last green
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
    {-2.7, -15}, //first green
    {-2.701, -15.501},
    {-1.4, -15.5}, // last green
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


//Declares author generated functions
class MyRobot : public Robot {
public:
    /**
     * @brief Empty constructor of the class.
     */
    MyRobot();

    /**
     * @brief Destructor of the class.
     */
    ~MyRobot();

    /**
     * @brief Function with the logic of the controller.
     * @param
     * @return
     */
    void run();

    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot.
     */
    void print_odometry();
    
     /**
     * @brief Main logic of the route.
     */
    void go_route();

    /**
     * @brief Checks whether the robot has reached the goal established for this controller.
     * @return true if the robot has reached the goal established for this controller; false otherwise.
     */
    bool goal_reached();
    
    void rotate_to_angle(double target_angle_rad);

    void go_to_start();
    
    int routineRouge();
    
    int routineVerte();
    void follow_path_begining(const float path[][2], int length);

    void follow_path(const float path[][2], int length);
    float compute_green_percentage();
    
    void rotate_to_compass_angle(double target_angle_rad);




    void turn_full_circle();
    
    bool ends_in_001(float value);



    

private:
    int _time_step;
    int world;
    double _x_offset;
    double _y_offset;
    double _theta_offset;
    // velocities
    double _left_speed, _right_speed;
        
    float _x, _y, _x_goal, _y_goal;   // [=] meters
    float _theta, _theta_goal;   // [=] rad
    
    float _path_goal[5][2] = {{-5,0},{-5, 4},{-13,4},{-13,0},{-18,0}};
    int _active_point, _total_points = 5;
    
    float _sr, _sl;  // [=] meters

    // Motor Position Sensor
    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;

    Camera *_forward_camera;

    DistanceSensor* _distance_sensor[NUM_DISTANCE_SENSOR]; // Tableau de 16 capteurs de distance
    const char *ds_name[NUM_DISTANCE_SENSOR] = {"ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds8", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};
    
    GPS *_gps;
    // Compass sensor
    Compass * _my_compass;
    void reset_odometry(bool use_compass);
    // Motors
    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;

    /**
     * @brief Updates the odometry of the robot in meters and radians. The atributes _x, _y, _theta are updated.
     * @param whether to use or not the compass to compute theta
     */
    void compute_odometry(bool use_compass=false);
        
    /**
     * @brief Computes orientation of the robot in degrees based on the information from the compass         * 
     * @return orientation of the robot in degrees 
     */       
    double convert_bearing_to_degrees();
    
    /**
     * @brief Computes orientation of the robot in radians based on the information from the compass         * 
     * @return orientation of the robot in radians 
     */ 
    double convert_bearing_to_radians();
    
    /**
     * @brief Utility functions to convert degrees in radians and viceversa         
     * 
     * @param the angle in radians or degrees
     * @return angle in degrees or radians 
     */ 
    double convert_rad_to_deg(double rad);
    double convert_deg_to_rad(double deg);
    
    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot. 
     * This method uses the encoder resolution and the wheel radius defined in the model of the robot.
     * 
     * @param tics raw value read from an encoder
     * @return meters corresponding to the tics value 
     */
    float encoder_tics_to_meters(float tics);
    
    float compute_distance_goal();
    float compute_angle_goal();
    
    // Movement fuctions
    void head_goal();
    void move_forward();
    void stop();
        
};

#endif
