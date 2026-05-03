#include "MyRobot.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()  // Constructor of MyRobot class, inheriting from Robot class
{
    world = 1;  // Initialize the world

    _time_step = 64;  // Time step duration

    _left_speed = 0;  // Left wheel speed
    _right_speed = 0;  // Right wheel speed

    _x = _y = _theta = 0.0;  // Initial position and orientation
    _x_offset = _y_offset = _theta_offset = 0.0;  // Offsets for position and orientation

    _sr = _sl = 0.0;  // Speed variables for left and right wheels

    _x_goal = 0.0;  // X position goal
    _y_goal = 0.0;  // Y position goal
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate goal orientation

    _left_wheel_sensor = getPositionSensor("left wheel sensor");  // Left wheel position sensor
    _right_wheel_sensor = getPositionSensor("right wheel sensor");  // Right wheel position sensor
    _left_wheel_sensor->enable(_time_step);  // Enable left wheel sensor
    _right_wheel_sensor->enable(_time_step);  // Enable right wheel sensor

    _my_compass = getCompass("compass");  // Compass
    _my_compass->enable(_time_step);  // Enable the compass

    _gps = getGPS("gps");  // GPS
    _gps->enable(_time_step);  // Enable the GPS
    
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++) {  // Initialize all distance sensors
        _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);  // Get the distance sensor by name
        _distance_sensor[ind]->enable(_time_step);  // Enable the distance sensor
    }
    
    _forward_camera = getCamera("camera_f");  // Forward camera
    _forward_camera->enable(_time_step);  // Enable the forward camera

    _left_wheel_motor = getMotor("left wheel motor");  // Left wheel motor
    _right_wheel_motor = getMotor("right wheel motor");  // Right wheel motor

    _right_wheel_motor->setPosition(0.0);  // Set the initial position for the right wheel motor
    _left_wheel_motor->setPosition(0.0);  // Set the initial position for the left wheel motor

    _right_wheel_motor->setPosition(INFINITY);  // Infinite position for free-moving wheel
    _left_wheel_motor->setPosition(INFINITY);  // Infinite position for free-moving wheel

    _right_wheel_motor->setVelocity(0.0);  // Set initial velocity for the right wheel motor
    _left_wheel_motor->setVelocity(0.0);  // Set initial velocity for the left wheel motor
}


//////////////////////////////////////////////

MyRobot::~MyRobot()  // Destructor of MyRobot class
{
    _left_wheel_motor->setVelocity(0.0);  // Set velocity of left wheel motor to zero
    _right_wheel_motor->setVelocity(0.0);  // Set velocity of right wheel motor to zero
    _my_compass->disable();  // Disable the compass
    _left_wheel_sensor->disable();  // Disable the left wheel position sensor
    _right_wheel_sensor->disable();  // Disable the right wheel position sensor
    _gps->disable();  // Disable the GPS
    _forward_camera->disable();  // Disable the forward camera
}



//////////////////////////////////////////////
void MyRobot::run(){
    
    this->go_to_start();  // Move to the starting position
    while (step(_time_step) != -1)  // Continue looping until the simulation ends
    {
        const unsigned char* image = _forward_camera->getImage();  // Get image from the forward camera
        int width = _forward_camera->getWidth();  // Get the image width
        int height = _forward_camera->getHeight();  // Get the image height
        
        int center_x = width / 2;  // X-coordinate of the image center
        int center_y = height / 2;  // Y-coordinate of the image center
    
        int r = _forward_camera->imageGetRed(image, width, center_x, center_y);  // Get the red value at the center pixel
        int g = _forward_camera->imageGetGreen(image, width, center_x, center_y);  // Get the green value at the center pixel
        int b = _forward_camera->imageGetBlue(image, width, center_x, center_y);  // Get the blue value at the center pixel
        cout << "CENTER RGB: " << r << " " << g << " " << b << endl;

        // Check for color range corresponding to the "rouge" (red) routine
        if( (40 < r && r < 50) && (106 < g && g < 116 ) && ( 106 < b && b < 116) )
        {
         world = this->routineRouge();  // Call the routine to identify worlds 1, 8, or 10
         cout << " world : " << world << endl;  
        }
        
        // Check for color range corresponding to world 9
        if( (160 < r && r < 170) && (247 < g && g < 257 ) && ( 247 < b && b < 257) )
        {
        world = 9;  // Set world to 9
        cout << " world : " << world << endl;  
        }
        
        // Check for color range corresponding to world 3
        if( (70 < r && r < 75) && (160 < g && g < 180 ) && (160< b && b < 180) )
        {
        world = 3;  // Set world to 3
        cout << " world : " << world << endl; 
        }
        
        // Check for color range corresponding to world 2
        if( (83 < r && r < 89) && (183 < g && g < 193 ) && (183 < b && b < 193) )
        {
        world = 2;  // Set world to 2
        cout << " world : " << world << endl; 
        }
        
        // Check for color range corresponding to the "verte" (green) routine
        if( (88 < r && r < 94) && (187 < g && g < 197 ) && (187 < b && b < 197) )
        {
        world = this->routineVerte();  // Call the routine to identify worlds 4, 5, 6, or 7
        cout << " world : " << world << endl; 
        }
        
        if (world > 0 && world <= 10) {  // Check if world is within valid range
              const float (*path1)[2];
              const float (*path2)[2];
              const float (*path3)[2];

              int length1 = 0;
              int length2 = 0;
              int length3 = 0;
              
              // Select path arrays based on the identified world
              switch ( world) {
                  case 1: path1 = world1_path1; length1 = sizeof(world1_path1)/sizeof(world1_path1[0]);
                          path2 = world1_path2; length2 = sizeof(world1_path2)/sizeof(world1_path2[0]);
                          path3 = world1_path3; length3 = sizeof(world1_path3)/sizeof(world1_path3[0]);
                          break;
                  case 2: path1 = world2_path1; length1 = sizeof(world2_path1)/sizeof(world2_path1[0]);
                          path2 = world2_path2; length2 = sizeof(world2_path2)/sizeof(world2_path2[0]);
                          path3 = world2_path3; length3 = sizeof(world2_path3)/sizeof(world2_path3[0]);
                          break;
                  case 3: path1 = world3_path1; length1 = sizeof(world3_path1)/sizeof(world3_path1[0]);
                          path2 = world3_path2; length2 = sizeof(world3_path2)/sizeof(world3_path2[0]);
                          path3 = world3_path3; length3 = sizeof(world3_path3)/sizeof(world3_path3[0]);
                          break;
                  case 4: path1 = world4_path1; length1 = sizeof(world4_path1)/sizeof(world4_path1[0]);
                          path2 = world4_path2; length2 = sizeof(world4_path2)/sizeof(world4_path2[0]);
                          path3 = world4_path3; length3 = sizeof(world4_path3)/sizeof(world4_path3[0]);
                          break;
                  case 5: path1 = world5_path1; length1 = sizeof(world5_path1)/sizeof(world5_path1[0]);
                          path2 = world5_path2; length2 = sizeof(world5_path2)/sizeof(world5_path2[0]);
                          path3 = world5_path3; length3 = sizeof(world5_path3)/sizeof(world5_path3[0]);
                          break;
                  case 6: path1 = world6_path1; length1 = sizeof(world6_path1)/sizeof(world6_path1[0]);
                          path2 = world6_path2; length2 = sizeof(world6_path2)/sizeof(world6_path2[0]);
                          path3 = world6_path3; length3 = sizeof(world6_path3)/sizeof(world6_path3[0]);
                          break;
                  case 7: path1 = world7_path1; length1 = sizeof(world7_path1)/sizeof(world7_path1[0]);
                          path2 = world7_path2; length2 = sizeof(world7_path2)/sizeof(world7_path2[0]);
                          path3 = world7_path3; length3 = sizeof(world7_path3)/sizeof(world7_path3[0]);
                          break;
                  case 8: path1 = world8_path1; length1 = sizeof(world8_path1)/sizeof(world8_path1[0]);
                          path2 = world8_path2; length2 = sizeof(world8_path2)/sizeof(world8_path2[0]);
                          path3 = world8_path3; length3 = sizeof(world8_path3)/sizeof(world8_path3[0]);
                          break;
                  case 9: path1 = world9_path1; length1 = sizeof(world9_path1)/sizeof(world9_path1[0]);
                          path2 = world9_path2; length2 = sizeof(world9_path2)/sizeof(world9_path2[0]);
                          path3 = world9_path3; length3 = sizeof(world9_path3)/sizeof(world9_path3[0]);
                          break;
                  case 10:path1 = world10_path1; length1 = sizeof(world10_path1)/sizeof(world10_path1[0]);
                          path2 = world10_path2; length2 = sizeof(world10_path2)/sizeof(world10_path2[0]);
                          path3 = world10_path3; length3 = sizeof(world10_path3)/sizeof(world10_path3[0]);
                          break;
                  default: cout << "No path defined for this world." << endl; return;  // Handle invalid world case
              }
          
              follow_path(path1, length1);  // Follow the first path

        break;
        }
     break;
    }
}

//////////////////////////////////////////////////////////////
void MyRobot::follow_path_begining(const float path[][2], int length) {
    // NOTE: this function has been rendered obsolete by the 001 flagging system.
    
    if (world == 2 || world == 3 || world == 9) {  // Reset position for worlds that use a local coordinate frame
        _x = _y = _theta = 0.0;  // Reset robot's position and orientation
    }

    for (int i = 0; i < length; ++i) {  // Loop through each waypoint in the path
        _x_goal = path[i][0];  // Set the X goal from the path
        _y_goal = path[i][1];  // Set the Y goal from the path
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate the goal orientation

        while (step(_time_step) != -1) {  // Loop while the simulation is running
            compute_odometry();  // Compute the robot's odometry
            go_route();  // Move towards the goal

            if (goal_reached()) {  // Check if the goal is reached
                stop();  // Stop the robot
                break;  // Exit the loop once the goal is reached
            }
        }
    }
}

///////////////////////////////////////////////////////////////

void MyRobot::follow_path(const float path[][2], int length) {
    // Same logic as follow_path_begining but with 001 flag support
    for (int i = 0; i < length; ++i) {
        _x_goal = path[i][0];
        _y_goal = path[i][1];
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

        // DEBUG
        const double* gps = _gps->getValues();
        cout << "=== Waypoint " << i+1 << "/" << length 
             << " | Target: (" << _x_goal << ", " << _y_goal << ")"
             << " | GPS: (" << gps[0] << ", " << gps[1] << ")"
             << " | ODO: (" << _x << ", " << _y << ")" << endl;

        // If a waypoint coordinate ends in .001 it is flagged as a rescue spin point
        if (ends_in_001(_x_goal) && ends_in_001(_y_goal)) {
            cout << ">>> RESCUE SPIN at waypoint " << i+1 << endl;
            turn_full_circle();
        }

        while (step(_time_step) != -1) {
            compute_odometry();
            go_route();

            if (goal_reached()) {
                const double* gps_arrived = _gps->getValues();
                cout << ">>> Reached waypoint " << i+1 
                     << " | GPS: (" << gps_arrived[0] << ", " << gps_arrived[1] << ")"
                     << " | ODO: (" << _x << ", " << _y << ")" << endl;
                stop();
                break;
            }
        }
    }
}

bool MyRobot::ends_in_001(float value) {
    value = fabs(value);

    // Step 1: Round to the nearest thousandth
    value = roundf(value * 1000.0f) / 1000.0f;

    // Step 2: Isolate the thousandths digit
    int thousandths_digit = static_cast<int>(value * 1000) % 10;

    bool result = (thousandths_digit == 1);

    return result;
}

////////////////////////////////////////////////////////////////

float MyRobot::compute_green_percentage() {
    const unsigned char* image = _forward_camera->getImage();  // Get the image from the forward camera
    int width = _forward_camera->getWidth();  // Get the width of the image
    int height = _forward_camera->getHeight();  // Get the height of the image

    int green_pixel_count = 0;  // Counter for green pixels
    int total_pixels = width * height;  // Total number of pixels in the image

    for (int y = 0; y < height; ++y) {  // Loop through the image rows
        for (int x = 0; x < width; ++x) {  // Loop through the image columns
            int r = _forward_camera->imageGetRed(image, width, x, y);  // Get the red value of the pixel
            int g = _forward_camera->imageGetGreen(image, width, x, y);  // Get the green value of the pixel
            int b = _forward_camera->imageGetBlue(image, width, x, y);  // Get the blue value of the pixel

            if (g > 100 && g > r + 30 && g > b + 30) {  // Check if the pixel is predominantly green
                ++green_pixel_count;  // Increment the green pixel counter
            }
        }
    }

    float green_percentage = (green_pixel_count * 100.0f) / total_pixels;  // Calculate the percentage of green pixels

    return green_percentage;  
}

///////////////////////////////////////////////

// Drives to key locations and uses distance sensors to identify which world this is (4, 5, 6, or 7)
int MyRobot::routineVerte() {

    double target_angle = convert_deg_to_rad(180);  // Set target angle to 180 degrees
    rotate_to_angle(target_angle);  // Rotate the robot to face the opposite direction

    // Drive to first diagnostic waypoint to check for a specific wall
    _x = _y = _theta = 0.0;  // Reset robot position and orientation
    _x_goal = 2.15;  // Set X goal position
    _y_goal = 0;  
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate the goal orientation
    _theta = convert_deg_to_rad(-180);  // Set robot's starting orientation
    stop();  

    while (step(_time_step) != -1) {  
        this->compute_odometry(); 
        this->go_route();  

        if (this->goal_reached()) {  
            this->stop();  
            double distanceAvant = _distance_sensor[0]->getValue();  // Read front distance sensor
             
            // Wall detected at this position means world is 6 or 7
            if (distanceAvant == 0) {  

                // Drive to second diagnostic waypoint to distinguish 6 from 7
                _x_goal = -1.75;  
                _y_goal = 0.10;  
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x)); 
                _theta = convert_deg_to_rad(-180);  
                stop();  

                while (step(_time_step) != -1) {  
                    this->compute_odometry(); 
                    this->go_route();  

                    if (this->goal_reached()) {  
                        this->stop(); this->stop(); 
                        double front_value = _distance_sensor[0]->getValue(); 
                        
                        // Open space in front means world 6, wall means world 7
                        if (front_value > 10) {  
                            return 6;
                        } else {  
                            return 7;
                        }
                    }
                }
                return -1;  // Error: goal never reached

            } else { 
                // No wall at first point, so world is 4 or 5
                // Drive to third diagnostic waypoint to distinguish 4 from 5
                _x_goal = 1.85;  
                _y_goal = 1.4;  
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  
                _theta = convert_deg_to_rad(-180);  
                stop();  
                
                while (step(_time_step) != -1) {  
                    this->compute_odometry();
                    this->go_route(); 

                    if (this->goal_reached()) {  
                        this->stop(); 
                        double left_side_value = _distance_sensor[3]->getValue();  // Read left distance sensor
                        
                        // Open space on the left means world 5, wall means world 4
                        if (left_side_value > 10) {  
                            return 5;
                        } else {  
                            return 4;
                        }
                    }
                }
                return -1;  // Error: goal never reached
            }
        }
    }
    return -1;  // Error: goal never reached
}

//////////////////////////////////////////////////////////

// Drives to key locations and uses distance sensors to identify which world this is (1, 8, or 10)
int MyRobot::routineRouge() {

    double target_angle = convert_deg_to_rad(-2);
    rotate_to_angle(target_angle);

    compute_odometry(true);
    reset_odometry(true);

    // Drive to first diagnostic waypoint to check for a wall
    _x_goal = -2.0;
    _y_goal = 0;
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

    while (step(_time_step) != -1) {
        compute_odometry();
        go_route();

        if (goal_reached()) {
            stop();
            double distanceAvant = _distance_sensor[0]->getValue();
            
            // Wall present at this position means world 8
            if (distanceAvant == 0) {
                return 8;
            } else {
                // No wall, so world is 1 or 10
                // Drive to second diagnostic waypoint
                _x_goal = -2.0;
                _y_goal = 1.4;
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));
                _theta = convert_deg_to_rad(-180);
                stop();

                while (step(_time_step) != -1) {
                    this->compute_odometry();
                    this->go_route();

                    if (this->goal_reached()) {
                        this->stop(); this->stop();

                        // Drive to third diagnostic waypoint
                        _x_goal = -2.4;
                        _y_goal = 0.5;
                        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));
                        _theta = convert_deg_to_rad(-180);
                        stop();

                        while (step(_time_step) != -1) {
                            this->compute_odometry();
                            this->go_route();

                            if (this->goal_reached()) {
                                this->stop(); this->stop();

                                double front_value        = _distance_sensor[0]->getValue();
                                double front_left_value   = _distance_sensor[1]->getValue();
                                double front_right_value  = _distance_sensor[14]->getValue();

                                // All sensors open (no walls) means world 10, otherwise world 1
                                if (front_value > 50 && front_left_value > 50 && front_right_value > 50) {
                                    return 10;
                                } else {
                                    return 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return -1;  // Error: goal never reached
}


//////////////////////////////////////////////

void MyRobot::go_to_start()
{
    step(_time_step); 
    
    // Rotate to face the wall using the compass
    rotate_to_compass_angle(convert_deg_to_rad(0));  

    // Drive forward until the front sensor detects a wall
    while (step(_time_step) != -1) {  
        double front_value = _distance_sensor[0]->getValue();  // Read front distance sensor

        if (front_value > 10) {  // Stop when a wall is detected
            stop();
            break;
        }

        _left_speed = MAX_SPEED - 2; 
        _right_speed = MAX_SPEED - 2; 
        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed); 
    }

    // Spin in place until the left sensor finds a wall (aligning to a corner)
    while (step(_time_step) != -1) {  
        double left_value = _distance_sensor[3]->getValue();  // Read left distance sensor
        
        if (left_value > 350) {  // Stop spinning once the left wall is found
            stop();          
            break;  
        }
        
        _left_speed = SLOW_SPEED;  
        _right_speed = -SLOW_SPEED; 
        _left_wheel_motor->setVelocity(_left_speed); 
        _right_wheel_motor->setVelocity(_right_speed); 
    }
    
    // Follow the left wall forward until a corner is reached (front sensor detects a wall)
    while (step(_time_step) != -1) {  
        double front_value = _distance_sensor[0]->getValue(); 
        double left_value  = _distance_sensor[3]->getValue();  // Read left distance sensor
    
        if (left_value > 10) {  // Too far from left wall: steer left
            _left_speed = MEDIUM_SPEED - 1;  
            _right_speed = MEDIUM_SPEED - 3; 
        } else if (left_value < 400) {  // Too close to left wall: steer right
            _left_speed = MEDIUM_SPEED - 3;  
            _right_speed = MEDIUM_SPEED - 1; 
        } else {  // Correct distance from left wall: go straight at max speed
            _left_speed = MAX_SPEED - 2;  
            _right_speed = MAX_SPEED - 2; 
        }
        
        if (front_value > 700) {  // Corner reached: stop the robot
            stop();  
            break;  
        }
        
        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed);  
    }
}

///////////////////////////////////////////////////////////////////////////////

void MyRobot::rotate_to_compass_angle(double target_angle_rad) {
    while (step(_time_step) != -1) {  
        double current_angle = convert_bearing_to_radians();  // Get the current heading from the compass
        double angle_diff = target_angle_rad - current_angle;  // Calculate angular error

        // Normalize angle difference to range [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (fabs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE)) {  // Stop when within tolerance
            stop();  
            break;  
        }

        double speed = SLOW_SPEED;
        if (fabs(angle_diff) < convert_deg_to_rad(2 * DEGREE_TOLERANCE)) {  // Slow down when close to target
            speed /= 5.0;
        }

        if (angle_diff > 0) {  // Rotate counterclockwise
            _left_speed = -speed;  
            _right_speed = speed; 
        } else {  // Rotate clockwise
            _left_speed = speed;  
            _right_speed = -speed;  
        }

        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed);  
    }

    stop(); 
}

//////////////////////////////////////////////

void MyRobot::rotate_to_angle(double target_angle_rad)
{
    while (step(_time_step) != -1) {  
        compute_odometry();  
        double angle_diff = target_angle_rad - _theta;  // Calculate angular error from odometry

        // Normalize the angle difference to range [-pi, pi]
        if (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;
        else if (angle_diff > M_PI)  
            angle_diff -= 2 * M_PI;

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE))  // Stop when within tolerance
            break;

        if (angle_diff > 0) {  // Rotate counterclockwise
            _left_speed = -SLOW_SPEED; 
            _right_speed = SLOW_SPEED;  
        }
        else {  // Rotate clockwise
            _left_speed = SLOW_SPEED;
            _right_speed = -SLOW_SPEED;
        }

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE * 2)) {  // Slow down when approaching target
            _left_speed /= 5.0;
            _right_speed /= 5.0;
        }

        _left_wheel_motor->setVelocity(_left_speed); 
        _right_wheel_motor->setVelocity(_right_speed);  
    }
}

//////////////////////////////////////////////

void MyRobot::go_route()
{
    if (abs(compute_angle_goal()) > convert_deg_to_rad(DEGREE_TOLERANCE))  // If not facing the goal, turn toward it
        head_goal();
    else
        move_forward();  // If aligned with the goal, move forward
}

//////////////////////////////////////////////

void MyRobot::reset_odometry(bool use_compass) {
    compute_odometry(use_compass);  // Update _x, _y, and _theta before resetting
    _sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());  // Snapshot the current left encoder value
    _sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());  // Snapshot the current right encoder value

    _x = _y = 0.0;  // Reset position to origin
    _theta = (use_compass ? convert_bearing_to_radians() : 0.0);  // Use compass heading or reset to zero

    _x_offset = 0.0;  // Reset X offset
    _y_offset = 0.0;  // Reset Y offset
    _theta_offset = 0.0;  // Reset orientation offset
}

//////////////////////////////////////////////

void MyRobot::compute_odometry(bool use_compass)
{
    float new_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());  // Read left encoder and convert to meters
    float new_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());  // Read right encoder and convert to meters

    float diff_sl = new_sl - _sl;  // Distance traveled by the left wheel since last update
    float diff_sr = new_sr - _sr;  // Distance traveled by the right wheel since last update

    _sl = new_sl;  // Save current left encoder value
    _sr = new_sr;  // Save current right encoder value

    // Update X and Y position using the differential drive odometry equations
    _x = (_x + ((diff_sr + diff_sl) / 2 * cos(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _x_offset;
    _y = (_y + ((diff_sr + diff_sl) / 2 * sin(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _y_offset;

    if (use_compass == true)  // Override orientation with compass reading if requested
        _theta = convert_bearing_to_radians();
    else
    {
        _theta = _theta + ((diff_sr - diff_sl) / WHEELS_DISTANCE);  // Update orientation from wheel difference

        // Keep theta within [-pi, pi]
        if (_theta <= -M_PI)
            _theta += 2 * M_PI;
        else if (_theta >= M_PI)  
            _theta -= 2 * M_PI;
    }

    _theta -= _theta_offset;  // Apply orientation offset correction
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_radians()
{
    const double *in_vector = _my_compass->getValues();  // Get the raw compass sensor values
    
    double rad = atan2(in_vector[2], in_vector[0]);  // Convert compass vector to heading in radians

    return rad;
}
     
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    double rad = convert_bearing_to_radians();  // Get heading in radians
    double deg = rad * (180.0 / M_PI);  // Convert to degrees

    return deg;
}

//////////////////////////////////////////////

double MyRobot::convert_deg_to_rad(double deg)
{    
    double rad = deg * (M_PI / 180.0);  // Convert degrees to radians
    return rad;
}

//////////////////////////////////////////////

double MyRobot::convert_rad_to_deg(double rad)
{
    double deg = rad * (180.0 / M_PI);  // Convert radians to degrees
    return deg;
}

//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;  // Convert raw encoder ticks to distance in meters
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y
    << " theta:" << _theta
    << " theta degrees:" << _theta * (180.0 / M_PI) << endl;
}

//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
    if (abs(_x_goal - _x) < DISTANCE_TOLERANCE && abs(_y_goal - _y) < DISTANCE_TOLERANCE)  // Check if within tolerance of the goal
        return true;
  
    return false;
}

//////////////////////////////////////////////

float MyRobot::compute_distance_goal()
{
    float x_target, y_target;
    
    x_target = _x_goal - _x;  // Horizontal distance to goal
    y_target = _y_goal - _y;  // Vertical distance to goal
    
    double distance = sqrt(pow(x_target, 2) + pow(y_target, 2));  // Euclidean distance to the goal
    return distance;
}

//////////////////////////////////////////////

float MyRobot::compute_angle_goal()
{
    float x_target, y_target, theta_target;
    
    x_target = _x_goal - _x;  // Horizontal offset to goal
    y_target = _y_goal - _y;  // Vertical offset to goal
    
    theta_target = atan2(y_target, x_target);  // Angle from current position to goal
    
    theta_target -= _theta;  // Make the angle relative to the robot's current orientation
    
    // Normalize to [-pi, pi]
    if (theta_target < -M_PI)
        theta_target += 2 * M_PI;
    else if (theta_target > M_PI)
        theta_target -= 2 * M_PI;
    
    return theta_target;
}

//////////////////////////////////////////////

void MyRobot::head_goal()
{    
    float angle_difference = compute_angle_goal();  // Get angular error to the goal
   
    if (angle_difference < -convert_deg_to_rad(DEGREE_TOLERANCE))  // Goal is to the right: rotate clockwise
    {
        _left_speed = SLOW_SPEED;
        _right_speed = -SLOW_SPEED;
        
        if (angle_difference > -convert_deg_to_rad(DEGREE_TOLERANCE * 2))  // Very close to aligned: slow down
        {
            _left_speed = SLOW_SPEED / 5.0;
            _right_speed = (-SLOW_SPEED) / 5.0;
        }
    }
    else if (angle_difference > convert_deg_to_rad(DEGREE_TOLERANCE))  // Goal is to the left: rotate counterclockwise
    {
        _left_speed = -SLOW_SPEED;
        _right_speed = SLOW_SPEED;
        
        if (angle_difference < convert_deg_to_rad(DEGREE_TOLERANCE * 2))  // Very close to aligned: slow down
        {
            _left_speed = (-SLOW_SPEED) / 5.0;
            _right_speed = SLOW_SPEED / 5.0;
        }
    }
    else  // Within tolerance: stop turning
    {
        _left_speed = 0;
        _right_speed = 0;
    }
            
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

//////////////////////////////////////////////

void MyRobot::move_forward()
{
    float distance = compute_distance_goal();  // Get distance remaining to goal
    if (distance < DISTANCE_TOLERANCE * 10)  // Close to goal: use medium speed
    {
        _left_speed = MEDIUM_SPEED;
        _right_speed = MEDIUM_SPEED;
    }
    else  // Far from goal: use maximum speed
    {
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
    }
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
 
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

/////////////////////////////////////

void MyRobot::turn_full_circle() {
    cout << "you are safe" << endl;  // Indicate a rescue spin is being performed

    _left_speed = -MAX_SPEED;  // Spin left wheel backward
    _right_speed = MAX_SPEED;  // Spin right wheel forward

    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);

    double duration = 3.0;  // Duration of the full circle in seconds
    int steps = (int)(duration * 1000 / _time_step);  // Number of simulation steps needed

    for (int i = 0; i < steps; ++i) {  // Execute the spin for the calculated number of steps
        step(_time_step);
    }

    stop();  // Stop after completing the full circle
}



