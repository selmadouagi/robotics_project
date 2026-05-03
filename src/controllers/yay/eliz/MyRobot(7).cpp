/**
 * @file    MyRobot_Hybrid.cpp
 * @brief   Working rescue controller - uses proven world ID with your waypoints
 * @author  Hybrid of working ID system + your measured waypoints
 */

#include "MyRobot.h"

MyRobot::MyRobot() : Robot()
{
    _time_step = 64;

    // Initialize all the sensors and motors
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
    if (_left_wheel_motor) {
        _left_wheel_motor->setPosition(INFINITY);
        _left_wheel_motor->setVelocity(0.0);
    }
    if (_right_wheel_motor) {
        _right_wheel_motor->setPosition(INFINITY);
        _right_wheel_motor->setVelocity(0.0);
    }

    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    if (_left_wheel_sensor) _left_wheel_sensor->enable(_time_step);
    if (_right_wheel_sensor) _right_wheel_sensor->enable(_time_step);

    _my_compass = getCompass("compass");
    if (_my_compass) _my_compass->enable(_time_step);

    _my_gps = getGPS("gps");
    if (_my_gps) _my_gps->enable(_time_step);

    _forward_camera = getCamera("camera_f");
    if (_forward_camera) _forward_camera->enable(_time_step);

    for (int i = 0; i < NUM_DS; i++) {
        string name = "ds" + to_string(i);
        _ds[i] = getDistanceSensor(name);
        if (_ds[i]) _ds[i]->enable(_time_step);
    }

    // State initialization - start with simple waypoint navigation
    _state = INITIAL_TURN;
    _victims_found = 0;
    _current_wp = 0;
    _stuck_ticks = 0;
    _spin_ticks = 0;
    _world_id = 1;  // Default to world 1 for now
    
    _left_speed = _right_speed = 0.0;
    _x = _y = _theta = 0.0;
    _prev_left_enc = _prev_right_enc = 0.0;
    
    cout << "MyRobot initialized. Will use World 1 path." << endl;
}

MyRobot::~MyRobot()
{
    if (_left_wheel_motor) _left_wheel_motor->setVelocity(0.0);
    if (_right_wheel_motor) _right_wheel_motor->setVelocity(0.0);
}

void MyRobot::run()
{
    // Initial setup
    step(_time_step);
    step(_time_step);

    // Get initial position
    if (_my_gps) {
        const double* gps = _my_gps->getValues();
        _x = _start_x = gps[2];
        _y = _start_y = gps[0];
    }
    _theta = get_heading_radians();
    
    if (_left_wheel_sensor) _prev_left_enc = _left_wheel_sensor->getValue();
    if (_right_wheel_sensor) _prev_right_enc = _right_wheel_sensor->getValue();

    cout << "Start: x=" << _x << " y=" << _y << " theta=" << _theta << endl;

    // Load waypoints for World 1
    load_path_for_world(1);
    cout << "Loaded " << _path.size() << " waypoints for World 1" << endl;

    // Main loop
    while (step(_time_step) != -1) {
        compute_odometry();

        switch (_state) {
            case INITIAL_TURN:
                step_initial_turn();
                break;
            case FOLLOW_PATH:
                step_follow_path();
                break;
            case SPIN_VICTIM:
                step_spin_victim();
                break;
            case RETURN_PATH:
                step_return_path();
                break;
            case DONE:
                set_speed(0, 0);
                cout << "DONE! Victims found: " << _victims_found << endl;
                return;
            default:
                // Skip any unused states
                break;
        }
    }
}

void MyRobot::step_initial_turn()
{
    if (_path.empty()) {
        cout << "ERROR: No waypoints!" << endl;
        _state = DONE;
        return;
    }

    double target = atan2(_path[0].y - _y, _path[0].x - _x);
    
    if (turn_to_heading(target)) {
        cout << "Heading to waypoint 0 at (" << _path[0].x << ", " << _path[0].y << ")" << endl;
        _state = FOLLOW_PATH;
        _last_dist_to_wp = dist_to(_path[_current_wp].x, _path[_current_wp].y);
        _stuck_ticks = 0;
    }
}

void MyRobot::step_follow_path()
{
    // Check for victims
    double ratio, center_x;
    if (_victims_found < 2 && green_detected(ratio, center_x)) {
        // Debug: print when ANY green is seen
        static int debug_counter = 0;
        if (debug_counter++ % 10 == 0) {
            cout << "[GREEN DEBUG] Ratio=" << ratio << " (need " << GREEN_CONFIRM_RATIO 
                 << "), center_x=" << center_x << endl;
        }
        
        if (ratio >= GREEN_CONFIRM_RATIO) {
            set_speed(0, 0);
            _victims_found++;
            cout << "*** VICTIM " << _victims_found << " found! ***" << endl;
            _spin_ticks = 0;
            _state = SPIN_VICTIM;
            _state_after_spin = FOLLOW_PATH;
            return;
        }
    }

    // Check if done
    if (_current_wp >= (int)_path.size()) {
        cout << "All waypoints visited. Returning home..." << endl;
        _return_wp = _path.size() - 1;
        _state = RETURN_PATH;
        return;
    }

    if (_victims_found >= 2) {
        cout << "Both victims found! Returning..." << endl;
        _return_wp = _current_wp - 1;
        _state = RETURN_PATH;
        return;
    }

    Waypoint wp = _path[_current_wp];
    double dist = dist_to(wp.x, wp.y);
    
    // Reached waypoint?
    if (dist < WAYPOINT_REACHED_M) {
        cout << "Reached waypoint " << _current_wp << endl;
        _current_wp++;
        _stuck_ticks = 0;
        if (_current_wp < (int)_path.size()) {
            _last_dist_to_wp = dist_to(_path[_current_wp].x, _path[_current_wp].y);
        }
        return;
    }

    double target_angle = atan2(wp.y - _y, wp.x - _x);
    double angle_error = normalize_angle(target_angle - _theta);

    // Obstacle avoidance
    double front = front_obstacle();
    if (front > FRONT_BLOCKED_THRESH) {
        _stuck_ticks++;
        if (_stuck_ticks > STUCK_TIMEOUT_TICKS) {
            cout << "Stuck at waypoint " << _current_wp << ", skipping..." << endl;
            _current_wp++;
            _stuck_ticks = 0;
            if (_current_wp < (int)_path.size()) {
                _last_dist_to_wp = dist_to(_path[_current_wp].x, _path[_current_wp].y);
            }
        } else {
            // Arc around obstacle
            set_speed(SPEED_PURSUE, SPEED_PURSUE * 0.3);
        }
    } else {
        _stuck_ticks = 0;
        
        // Navigate to waypoint
        if (fabs(angle_error) > 0.3) {
            turn_to_heading(target_angle);
        } else {
            double turn_adjust = angle_error * 2.0;
            set_speed(SPEED_PURSUE - turn_adjust, SPEED_PURSUE + turn_adjust);
        }
    }
    
    // Update stuck detection
    if (dist < _last_dist_to_wp) {
        _last_dist_to_wp = dist;
    }
}

void MyRobot::step_spin_victim()
{
    _spin_ticks++;
    
    if (_spin_ticks >= 30) {  // ~2 seconds at 64ms per tick
        set_speed(0, 0);
        cout << "Spin complete!" << endl;
        _spin_ticks = 0;
        _state = _state_after_spin;
    } else {
        set_speed(-MAX_SPEED, MAX_SPEED);
    }
}

void MyRobot::step_return_path()
{
    if (_return_wp < 0) {
        double dist = dist_to(_start_x, _start_y);
        if (dist < 1.0) {
            cout << "=== MISSION COMPLETE ===" << endl;
            _state = DONE;
            return;
        }
        
        double target_angle = atan2(_start_y - _y, _start_x - _x);
        double angle_error = normalize_angle(target_angle - _theta);
        
        if (fabs(angle_error) > 0.3) {
            turn_to_heading(target_angle);
        } else {
            double turn_adjust = angle_error * 2.0;
            set_speed(SPEED_PURSUE - turn_adjust, SPEED_PURSUE + turn_adjust);
        }
        return;
    }

    Waypoint wp = _path[_return_wp];
    double dist = dist_to(wp.x, wp.y);
    
    if (dist < WAYPOINT_REACHED_M) {
        cout << "Return: passed waypoint " << _return_wp << endl;
        _return_wp--;
        return;
    }

    double target_angle = atan2(wp.y - _y, wp.x - _x);
    double angle_error = normalize_angle(target_angle - _theta);
    
    double front = front_obstacle();
    if (front > FRONT_BLOCKED_THRESH) {
        set_speed(SPEED_PURSUE, SPEED_PURSUE * 0.3);
    } else {
        if (fabs(angle_error) > 0.3) {
            turn_to_heading(target_angle);
        } else {
            double turn_adjust = angle_error * 2.0;
            set_speed(SPEED_PURSUE - turn_adjust, SPEED_PURSUE + turn_adjust);
        }
    }
}

// ===== SENSOR HELPERS =====

double MyRobot::front_obstacle()
{
    double max_val = 0.0;
    int ids[] = {0, 1, 14, 15};
    for (int i = 0; i < 4; i++) {
        if (_ds[ids[i]]) {
            double v = _ds[ids[i]]->getValue();
            if (v > max_val) max_val = v;
        }
    }
    return max_val;
}

double MyRobot::left_obstacle()
{
    double max_val = 0.0;
    int ids[] = {3, 4, 5};
    for (int i = 0; i < 3; i++) {
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    }
    return max_val;
}

double MyRobot::right_obstacle()
{
    double max_val = 0.0;
    int ids[] = {10, 11, 12};
    for (int i = 0; i < 3; i++) {
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    }
    return max_val;
}

bool MyRobot::green_detected(double& ratio, double& center_x)
{
    ratio = 0.0;
    center_x = 0.0;
    
    if (!_forward_camera) return false;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return false;

    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int green_pixels = 0;
    int total = width * height;
    double green_x_sum = 0.0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = _forward_camera->imageGetRed(img, width, x, y);
            unsigned char g = _forward_camera->imageGetGreen(img, width, x, y);
            unsigned char b = _forward_camera->imageGetBlue(img, width, x, y);

            if (g > GREEN_MIN_G && g > r + GREEN_DOMINANCE && g > b + GREEN_DOMINANCE) {
                green_pixels++;
                green_x_sum += x;
            }
        }
    }

    ratio = (double)green_pixels / total;
    
    if (green_pixels > 0) {
        center_x = (green_x_sum / green_pixels / width) * 2.0 - 1.0;
    }
    
    return (ratio > GREEN_DETECT_RATIO);
}

// ===== NAVIGATION HELPERS =====

double MyRobot::dist_to(double tx, double ty)
{
    double dx = tx - _x;
    double dy = ty - _y;
    return sqrt(dx*dx + dy*dy);
}

double MyRobot::normalize_angle(double a)
{
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

void MyRobot::set_speed(double l, double r)
{
    _left_speed = max(-MAX_SPEED, min(MAX_SPEED, l));
    _right_speed = max(-MAX_SPEED, min(MAX_SPEED, r));
    if (_left_wheel_motor) _left_wheel_motor->setVelocity(_left_speed);
    if (_right_wheel_motor) _right_wheel_motor->setVelocity(_right_speed);
}

bool MyRobot::turn_to_heading(double target)
{
    double current = get_heading_radians();
    double error = normalize_angle(target - current);
    
    if (fabs(error) < ANGLE_TOL) {
        set_speed(0, 0);
        return true;
    }
    
    if (error > 0) {
        set_speed(-SPEED_ROTATE, SPEED_ROTATE);
    } else {
        set_speed(SPEED_ROTATE, -SPEED_ROTATE);
    }
    
    return false;
}

void MyRobot::compute_odometry()
{
    double left_enc = _left_wheel_sensor ? _left_wheel_sensor->getValue() : 0.0;
    double right_enc = _right_wheel_sensor ? _right_wheel_sensor->getValue() : 0.0;
    
    double left_delta = (left_enc - _prev_left_enc) / ENCODER_TICS_PER_RADIAN;
    double right_delta = (right_enc - _prev_right_enc) / ENCODER_TICS_PER_RADIAN;
    
    _prev_left_enc = left_enc;
    _prev_right_enc = right_enc;
    
    double left_dist = left_delta * WHEEL_RADIUS;
    double right_dist = right_delta * WHEEL_RADIUS;
    
    double distance = (left_dist + right_dist) / 2.0;
    
    // Use compass for heading
    _theta = get_heading_radians();
    
    _x += distance * cos(_theta);
    _y += distance * sin(_theta);
}

double MyRobot::get_heading_radians()
{
    if (!_my_compass) return 0.0;
    const double* v = _my_compass->getValues();
    return atan2(-v[2], -v[0]);
}

// ===== PATH LOADING =====

void MyRobot::load_path_for_world(int world_id)
{
    switch (world_id) {
        case 1:  copy_path(world1_path); break;
        case 2:  copy_path(world2_path); break;
        case 3:  copy_path(world3_path); break;
        case 4:  copy_path(world4_path); break;
        case 5:  copy_path(world5_path); break;
        case 6:  copy_path(world6_path); break;
        case 7:  copy_path(world7_path); break;
        case 8:  copy_path(world8_path); break;
        case 9:  copy_path(world9_path); break;
        case 10: copy_path(world10_path); break;
        default:
            cout << "Unknown world " << world_id << endl;
            _path.clear();
    }
}

const char* MyRobot::state_name()
{
    switch (_state) {
        case INITIAL_TURN: return "INITIAL_TURN";
        case FOLLOW_PATH: return "FOLLOW_PATH";
        case SPIN_VICTIM: return "SPIN_VICTIM";
        case RETURN_PATH: return "RETURN_PATH";
        case DONE: return "DONE";
        default: return "UNKNOWN";
    }
}