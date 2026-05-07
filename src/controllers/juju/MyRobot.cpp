#include "MyRobot.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
/**
 * @file    MyRobot.h
 * @brief   A header file containing function declarations, world paths, and sensor declarations
 *
 * @author  Douagi selma Elizabeth Faulkner
 * @date    2024-06-01
 */


// Robot constructor
MyRobot::MyRobot() : Robot()
{
    world = 1;
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _x = _y = _theta = 0.0;
    _x_offset = _y_offset = _theta_offset = 0.0;

    _sr = _sl = 0.0;

    _x_goal = 0.0;
    _y_goal = 0.0;
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _gps = getGPS("gps");
    _gps->enable(_time_step);
    
    // Initialize distance sensors
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++) {
        _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);
        _distance_sensor[ind]->enable(_time_step);
    }
    
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    _right_wheel_motor->setPosition(0.0);
    _left_wheel_motor->setPosition(0.0);

    _right_wheel_motor->setPosition(INFINITY);
    _left_wheel_motor->setPosition(INFINITY);

    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);
}
// Robot destructor
MyRobot::~MyRobot()
{
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
    _my_compass->disable();
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
    _gps->disable();
    _forward_camera->disable();
}

// Main robot execution function
void MyRobot::run(){
    
    this->go_to_start();
    while (step(_time_step) != -1)
    {
        const unsigned char* image = _forward_camera->getImage();
        int width = _forward_camera->getWidth();
        int height = _forward_camera->getHeight();
        
        int center_x = width / 2;
        int center_y = height / 2;
    
        int r = _forward_camera->imageGetRed(image, width, center_x, center_y);
        int g = _forward_camera->imageGetGreen(image, width, center_x, center_y);
        int b = _forward_camera->imageGetBlue(image, width, center_x, center_y);
    
           double luminosity = 0.299 * r + 0.587 * g + 0.114 * b;
           cout << std::fixed << std::setprecision(2)
               << "Luminosité centre: " << luminosity
               << " | RGB: R=" << r << " G=" << g << " B=" << b << endl;

        
        // Red world detection
        if( (40 < r && r < 50) && (106 < g && g < 116 ) && ( 106 < b && b < 116) )
        {
         world = this->routineRouge();
         cout << "Monde détecté : rouge -> world = " << world << endl;  
        }
        
        // Black/blue world detection
        if( (160 < r && r < 170) && (247 < g && g < 257 ) && ( 247 < b && b < 257) )
        {
        world = 9;
        cout << "Monde détecté : noir/bleu -> world = " << world << endl;  
        }
        
        // Blue world detection
        if( (57 < r && r < 67) && (140 < g && g < 160 ) && (140 < b && b < 160) )
        {
        world = 3;
        cout << "Monde détecté : bleu -> world = " << world << endl; 
        }
        
        // Black world detection
        if( (83 < r && r < 89) && (183 < g && g < 193 ) && (183 < b && b < 193) )
        {
        world = 2;
        cout << "Monde détecté : noir -> world = " << world << endl; 
        }
        
        // Green world detection
        if( (88 < r && r < 94) && (187 < g && g < 197 ) && (187 < b && b < 197) )
        {
        world = this->routineVerte();
        cout << "Monde détecté : vert -> world = " << world << endl; 
        }
        
        // Execute path for detected world
        if (world > 0 && world <= 10) {
              cout << "Monde trouvé, chargement du parcours pour world = " << world << endl;
              const float (*path1)[2];
              const float (*path2)[2];
              const float (*path3)[2];

              int length1 = 0;
              int length2 = 0;
              int length3 = 0;
              
              // Select path based on world value
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
                  default: cout << "No path defined for this world." << endl; return;
              }
          
              follow_path(path1, length1);

        break;
        }
     break;
    }
}

// Follow predefined path (legacy version)
void MyRobot::follow_path_begining(const float path[][2], int length) {
    
    // Reset position for certain worlds
    if (world == 2 || world == 3 || world == 9) {
        _x = _y = _theta = 0.0;
    }

    // Traverse path point by point
    for (int i = 0; i < length; ++i) {
        _x_goal = path[i][0];
        _y_goal = path[i][1];
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

        while (step(_time_step) != -1) {
            compute_odometry();
            go_route();

            if (goal_reached()) {
                stop();
                break;
            }
        }
    }
}

// Main path following function
void MyRobot::follow_path(const float path[][2], int length) {
     // Logique identique à follow_path_begining
    for (int i = 0; i < length; ++i) {
        _x_goal = path[i][0];
        _y_goal = path[i][1];
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

        // Detect flagged 001 points
        if (ends_in_001(_x_goal) && ends_in_001(_y_goal)) {
            turn_full_circle();
        }

        while (step(_time_step) != -1) {
            compute_odometry();
            go_route();

            if (goal_reached()) {
                stop();
                break;
            }
        }
    }
}

// Check if value ends in 001
bool MyRobot::ends_in_001(float value) {
    value = fabs(value);

    // Round to nearest thousandth
    value = roundf(value * 1000.0f) / 1000.0f;

    // Isolate thousandths digit
    int thousandths_digit = static_cast<int>(value * 1000) % 10;

    bool result = (thousandths_digit == 1);

    return result;
}

// Calculate percentage of green pixels in image
float MyRobot::compute_green_percentage() {
    const unsigned char* image = _forward_camera->getImage();
    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();

    int green_pixel_count = 0;
    int total_pixels = width * height;

    // Iterate through each pixel
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int r = _forward_camera->imageGetRed(image, width, x, y);
            int g = _forward_camera->imageGetGreen(image, width, x, y);
            int b = _forward_camera->imageGetBlue(image, width, x, y);

            // Green pixel detection
            if (g > 100 && g > r + 30 && g > b + 30) {
                ++green_pixel_count;
            }
        }
    }

    float green_percentage = (green_pixel_count * 100.0f) / total_pixels;

    return green_percentage;
}
// Identify green world
int MyRobot::routineVerte() {

    double target_angle = convert_deg_to_rad(180);
    rotate_to_angle(target_angle);

    // First point to find a specific wall
    _x = _y = _theta = 0.0;
    _x_goal = 2.15;
    _y_goal = 0;
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));
    _theta = convert_deg_to_rad(-180);
    stop();

    while (step(_time_step) != -1) {
        this->compute_odometry();
        this->go_route();

        if (this->goal_reached()) {
            this->stop();
            double distanceAvant = _distance_sensor[0]->getValue();
             
            // Detect worlds 6 or 7
            if (distanceAvant == 0) {

                // Second important point to find a wall
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

                        // Determine world (6 or 7)
                        if (front_value > 10) {
                            return 6;
                        } else {
                            return 7;
                        }
                    }
                }
                return -1;

            } else {
                // Important point to distinguish 4 or 5
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
                        double left_side_value = _distance_sensor[3]->getValue();

                        // Determine world (4 or 5)
                        if (left_side_value > 10) {
                            return 5;
                        } else {
                            return 4;
                        }
                    }
                }
                return -1;
            }
        }
    }
    return -1;
}

// Identify red world
int MyRobot::routineRouge() {
    // Same logic as routineVerte to identify world

    double target_angle = convert_deg_to_rad(-2);
    rotate_to_angle(target_angle);

    compute_odometry(true);
    reset_odometry(true);

    // Important point to detect a wall
    _x_goal = -2.0;
    _y_goal = 0;
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

    while (step(_time_step) != -1) {
        compute_odometry();
        go_route();

        if (goal_reached()) {
            stop();
            double distanceAvant = _distance_sensor[0]->getValue();

            // Detect world 8
            if (distanceAvant == 0) {
                return 8;
            } else {
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

                                // Distinguish world 10 or 1
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

    return -1;
}





// Initialize: go to starting position
void MyRobot::go_to_start()
{
    step(_time_step);

    // Orient towards wall using compass
    rotate_to_compass_angle(convert_deg_to_rad(0));

    // Find the wall
    while (step(_time_step) != -1) {
        double front_value = _distance_sensor[0]->getValue();

        if (front_value > 10) {
            stop();
            break;
        }

        _left_speed = MAX_SPEED - 2;
        _right_speed = MAX_SPEED - 2;
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }

    // Turn robot towards corner

    while (step(_time_step) != -1) {
        double left_value = _distance_sensor[3]->getValue();

        if (left_value > 500) {
            stop();
            break;
        }

        _left_speed = SLOW_SPEED;
        _right_speed = -SLOW_SPEED;
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }

    // Follow wall to corner
    while (step(_time_step) != -1) {
        double front_value = _distance_sensor[0]->getValue();
        double left_value  = _distance_sensor[3]->getValue();

        if (left_value > 10) {
            _left_speed = MEDIUM_SPEED - 1;
            _right_speed = MEDIUM_SPEED - 3;
        } else if (left_value < 400) {
            _left_speed = MEDIUM_SPEED - 3;
            _right_speed = MEDIUM_SPEED - 1;
        } else {
            _left_speed = MAX_SPEED - 2;
            _right_speed = MAX_SPEED - 2;
        }

        if (front_value > 700) {
            stop();
            break;
        }

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}

// Rotate to angle using compass
void MyRobot::rotate_to_compass_angle(double target_angle_rad) {
    while (step(_time_step) != -1) {
        double current_angle = convert_bearing_to_radians();
        double angle_diff = target_angle_rad - current_angle;

        // Normalize angle difference to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (fabs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE)) {
            stop();
            break;
        }

        double speed = SLOW_SPEED;
        if (fabs(angle_diff) < convert_deg_to_rad(2 * DEGREE_TOLERANCE)) {
            speed /= 5.0;
        }

        if (angle_diff > 0) {
            _left_speed = -speed;
            _right_speed = speed;
        } else {
            _left_speed = speed;
            _right_speed = -speed;
        }

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }

    stop();
}








// Rotate to specific angle
void MyRobot::rotate_to_angle(double target_angle_rad)
{
    while (step(_time_step) != -1) {
        compute_odometry();
        double angle_diff = target_angle_rad - _theta;

        // Normalize to [-pi, pi]
        if (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;
        else if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE))
            break;

        if (angle_diff > 0) {
            _left_speed = -SLOW_SPEED;
            _right_speed = SLOW_SPEED;
        }
        else {
            _left_speed = SLOW_SPEED;
            _right_speed = -SLOW_SPEED;
        }

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE * 2)) {
            _left_speed /= 5.0;
            _right_speed /= 5.0;
        }

        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}



// Execute movement towards goal
void MyRobot::go_route()
{
    if (abs(compute_angle_goal()) > convert_deg_to_rad(DEGREE_TOLERANCE))
        head_goal();
    else
        move_forward();
}

// Reset odometry
void MyRobot::reset_odometry(bool use_compass) {
    compute_odometry(use_compass);
    _sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());
    _sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());

    _x = _y = 0.0;
    _theta = (use_compass ? convert_bearing_to_radians() : 0.0);

    _x_offset = 0.0;
    _y_offset = 0.0;
    _theta_offset = 0.0;
}



// Compute robot odometry
void MyRobot::compute_odometry(bool use_compass)
{
    float new_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());
    float new_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());

    float diff_sl = new_sl - _sl;
    float diff_sr = new_sr - _sr;

    _sl = new_sl;
    _sr = new_sr;

    // Update X and Y position
    _x = (_x + ((diff_sr + diff_sl) / 2 * cos(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _x_offset;
    _y = (_y + ((diff_sr + diff_sl) / 2 * sin(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _y_offset;

    // Update orientation
    if (use_compass == true)
        _theta = convert_bearing_to_radians();
    else
    {
        _theta = _theta + ((diff_sr - diff_sl) / WHEELS_DISTANCE);

        // Normalize to [-pi, pi]
        if (_theta <= -M_PI)
            _theta += 2 * M_PI;
        else if (_theta >= M_PI)
            _theta -= 2 * M_PI;
    }

    _theta -= _theta_offset;
}

// Convert compass bearing to radians
double MyRobot::convert_bearing_to_radians()
{
    const double *in_vector = _my_compass->getValues();

    double rad = atan2(in_vector[2], in_vector[0]);

    return rad;
}

// Convert compass bearing to degrees
double MyRobot::convert_bearing_to_degrees()
{
    double rad = convert_bearing_to_radians();
    double deg = rad * (180.0 / M_PI);

    return deg;
}

// Convert degrees to radians
double MyRobot::convert_deg_to_rad(double deg)
{
    double rad = deg * (M_PI / 180.0);
    return rad;
}

// Convert radians to degrees
double MyRobot::convert_rad_to_deg(double rad)
{
    double deg = rad * (180.0 / M_PI);
    return deg;
}
// Convert encoder tics to meters
float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

// Print odometry information
void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y
    << " theta:" << _theta
    << " theta degrees:" << _theta * (180.0 / M_PI) << endl;
}

// Check if goal is reached
bool MyRobot::goal_reached()
{
    if (abs(_x_goal - _x) < DISTANCE_TOLERANCE && abs(_y_goal - _y) < DISTANCE_TOLERANCE)
        return true;

    return false;
}

// Calculate distance to goal
float MyRobot::compute_distance_goal()
{
    float x_target, y_target;

    x_target = _x_goal - _x;
    y_target = _y_goal - _y;

    double distance = sqrt(pow(x_target, 2) + pow(y_target, 2));
    return distance;
}

// Calculate angle to goal
float MyRobot::compute_angle_goal()
{
    float x_target, y_target, theta_target;

    x_target = _x_goal - _x;
    y_target = _y_goal - _y;

    theta_target = atan2(y_target, x_target);

    theta_target -= _theta;

    // Normalize to [-pi, pi]
    if (theta_target < -M_PI)
        theta_target += 2 * M_PI;
    else if (theta_target > M_PI)
        theta_target -= 2 * M_PI;

    return theta_target;
}


// Move towards goal direction
void MyRobot::head_goal()
{
    float angle_difference = compute_angle_goal();

    if (angle_difference < -convert_deg_to_rad(DEGREE_TOLERANCE))
    {
        _left_speed = SLOW_SPEED;
        _right_speed = -SLOW_SPEED;

        if (angle_difference > -convert_deg_to_rad(DEGREE_TOLERANCE * 2))
        {
            _left_speed = SLOW_SPEED / 5.0;
            _right_speed = (-SLOW_SPEED) / 5.0;
        }
    }
    else if (angle_difference > convert_deg_to_rad(DEGREE_TOLERANCE))
    {
        _left_speed = -SLOW_SPEED;
        _right_speed = SLOW_SPEED;

        if (angle_difference < convert_deg_to_rad(DEGREE_TOLERANCE * 2))
        {
            _left_speed = (-SLOW_SPEED) / 5.0;
            _right_speed = SLOW_SPEED / 5.0;
        }
    }
    else
    {
        _left_speed = 0;
        _right_speed = 0;
    }

    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

// Move forward
void MyRobot::move_forward()
{
    float distance = compute_distance_goal();
    if (distance < DISTANCE_TOLERANCE * 10)
    {
        _left_speed = MEDIUM_SPEED;
        _right_speed = MEDIUM_SPEED;
    }
    else
    {
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
    }
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

// Stop robot
void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;

    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

// Perform full circle rotation
void MyRobot::turn_full_circle() {
    cout << "tu es sauvé // you are safe" << endl;

    _left_speed = -MAX_SPEED;
    _right_speed = MAX_SPEED;

    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);

    double duration = 3.0;
    int steps = (int)(duration * 1000 / _time_step);

    for (int i = 0; i < steps; ++i) {
        step(_time_step);
    }

    stop();
}

