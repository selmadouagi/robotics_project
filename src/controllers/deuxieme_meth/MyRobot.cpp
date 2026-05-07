/**
 * @file    MyRobot.cpp
 * @brief   5-Phase Rescue Controller: Localize → Identify → Navigate → Detect → Return
 *
 * @author  Elizabeth Faulkner
 * @date    2026-04
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    _time_step = 64;

    _left_speed = _right_speed = 0.0;
    _x = _y = _theta = 0.0;
    _prev_left_enc = _prev_right_enc = 0.0;

    // Start with physical odometry calibration using walls.
    _state          = ID_TURN_TO_BACK_WALL;
    _victims_found  = 0;
    _state_after_spin = FOLLOW_PATH;
    _world_id       = 0;
    _current_wp     = 0;
    _gps_timer      = 0;

    // Phase 2 — world identification fields
    _id_brightness        = 0.0;
    _id_avg_r = _id_avg_g = _id_avg_b = 0.0;
    _id_variance = 0.0;
    _id_saturation_ratio = 0.0;
    _id_right_wall_dist   = 0.0;
    _id_left_wall_dist    = 0.0;
    _id_right_center_front = 0.0;
    _id_left_center_front  = 0.0;
    _id_right_wall_found  = false;
    _id_left_wall_found   = false;
    _id_cube_ahead        = false;
    _id_initial_gps_x     = 0.0;
    _id_initial_gps_y     = 0.0;
    _id_scan_start_x      = 0.0;
    _id_scan_start_y      = 0.0;
    _id_line_anchor_x     = 0.0;
    _id_line_anchor_y     = 0.0;
    _id_forward_heading   = 0.0;
    _id_initialized       = false;
    _id_wall_steps        = 0;
    _id_settle_steps      = 0;
    _id_scan_heading_target = 0.0;
    _id_front_hit_count   = 0;
    _id_right_scan_left_peak = 0.0;
    _id_left_scan_right_peak = 0.0;
    _id_probe_start_x = 0.0;
    _id_probe_start_y = 0.0;
    _id_probe_end_x   = 0.0;
    _id_probe_end_y   = 0.0;
    _id_probe_steps   = 0;
    _id_right_center_hit = false;
    _id_left_center_hit  = false;
    _id_left_object = OBJ_NOTHING;
    _id_middle_object = OBJ_NOTHING;
    _stuck_ticks = 0;
    _last_dist_to_wp = 1e9;
    _id_cube_on_left = false;

    // Wall/corner calibration state.
    _corner_wall_seen = false;
    _corner_push_steps = 0;

    _back_wall_seen = false;
    _back_wall_push_steps = 0;

    _left_wheel_sensor  = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    if (_left_wheel_sensor)  _left_wheel_sensor->enable(_time_step);
    if (_right_wheel_sensor) _right_wheel_sensor->enable(_time_step);

    _my_compass = getCompass("compass");
    if (_my_compass) _my_compass->enable(_time_step);

    _my_gps = getGPS("gps");
    if (_my_gps) _my_gps->enable(_time_step);

    for (int i = 0; i < NUM_DS; i++) {
        string name = "ds" + to_string(i);
        _ds[i] = getDistanceSensor(name);
        if (_ds[i]) _ds[i]->enable(_time_step);
    }

    _left_wheel_motor  = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
    if (_left_wheel_motor)  { _left_wheel_motor->setPosition(INFINITY);  _left_wheel_motor->setVelocity(0.0); }
    if (_right_wheel_motor) { _right_wheel_motor->setPosition(INFINITY); _right_wheel_motor->setVelocity(0.0); }

    _forward_camera = getCamera("camera_f");
    if (_forward_camera) _forward_camera->enable(_time_step);

    _spherical_camera = getCamera("camera_s");
    if (_spherical_camera) _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    if (_left_wheel_motor)  _left_wheel_motor->setVelocity(0.0);
    if (_right_wheel_motor) _right_wheel_motor->setVelocity(0.0);
    if (_my_compass)        _my_compass->disable();
    if (_left_wheel_sensor) _left_wheel_sensor->disable();
    if (_right_wheel_sensor)_right_wheel_sensor->disable();
    if (_my_gps)            _my_gps->disable();
    if (_forward_camera)   _forward_camera->disable();
    if (_spherical_camera) _spherical_camera->disable();
    for (int i = 0; i < NUM_DS; i++)
        if (_ds[i]) _ds[i]->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{

    if (step(_time_step) == -1) return;

    // Seed odometry from GPS + compass on the very first tick.
    // GPS is noisy (3 m) but good enough for initial pose; compass gives accurate heading.
    if (_my_gps) {
        _x = (float)_my_gps->getValues()[2];   // Webots Z → our x (long axis)
        _y = (float)_my_gps->getValues()[0];   // Webots X → our y (lateral)
    }
    _theta = normalize_angle(get_heading_radians());
    _start_x = _x;
    _start_y = _y;
    if (_left_wheel_sensor)  _prev_left_enc  = _left_wheel_sensor->getValue();
    if (_right_wheel_sensor) _prev_right_enc = _right_wheel_sensor->getValue();
    cout << "GPS: x=" << _x << " y=" << _y << " θ=" << _theta << endl;

    while (step(_time_step) != -1)
    {
        compute_odometry();
        _theta = normalize_angle(get_heading_radians());

        bool in_phase2 = (_state >= ID_FACE_FORWARD && _state <= ID_CLASSIFY);

        bool in_wall_calibration =
            (_state == ID_TURN_TO_BACK_WALL ||
            _state == ID_DRIVE_TO_BACK_WALL ||
            _state == ID_TURN_RIGHT_FROM_BACK_WALL ||
            _state == ID_DRIVE_TO_LEFT_CORNER ||
            _state == ID_SET_RIGHT_CORNER_ODOM ||
            _state == ID_MEASURE_LIGHT_CALIB ||
            _state == ID_CALIB_DONE);

        if (++_gps_timer >= 78) {
            if (!in_phase2 && !in_wall_calibration) {
                apply_gps_correction();
            }
            _gps_timer = 0;
        }

        double front = front_obstacle();
        double right = right_obstacle();

        switch (_state)
        {
        // ════════════════════════════════════════════════════════════════════
        // TEMPORARY CALIBRATION SCENARIO
        // 1) Turn toward the back wall using the compass.
        // 2) Drive until the back wall is detected.
        // 3) Turn left and drive along the wall until the right corner is reached.
        // 4) Force odometry to the known corner coordinates.
        // 5) Measure brightness, then stop.
        // ════════════════════════════════════════════════════════════════════

        case ID_TURN_TO_BACK_WALL:
        {
            if (turn_to_heading_forced_left(BACK_WALL_HEADING, CALIB_TURN_SPEED)) {
                cout << "[CALIB] Facing back wall. Start driving." << endl;

                _back_wall_seen = false;
                _back_wall_push_steps = 0;

                _state = ID_DRIVE_TO_BACK_WALL;
            }
            break;
        }

        case ID_DRIVE_TO_BACK_WALL:
        {
            if (drive_to_back_wall()) {
                cout << "[CALIB] Back wall reached. Turning left toward corner." << endl;

                _state = ID_TURN_RIGHT_FROM_BACK_WALL;
            }
            break;
        }

        case ID_TURN_RIGHT_FROM_BACK_WALL:
        {
            if (turn_to_heading(1.49, 0.7)) {
                cout << "[CALIB] right turn done. Drive to left corner." << endl;
                _corner_wall_seen = false;
                _corner_push_steps = 0;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps = 0;
                _id_front_hit_count = 0;
                _id_scan_heading_target = normalize_angle(get_heading_radians());
                _state = ID_DRIVE_TO_LEFT_CORNER;
            }
            break;
        }

        case ID_DRIVE_TO_LEFT_CORNER:
        {
            if (drive_to_right_corner_after_back_wall()) {
                _state = ID_MEASURE_LIGHT_CALIB;
            }
            break;
        }

        case ID_MEASURE_LIGHT_CALIB:
        {


            // Étape 2 : laisser quelques steps à la caméra pour produire
            // une image rendue dans la nouvelle orientation.
            set_speed(0.0, 0.0);
            static int wait_steps = 0;
            if (wait_steps < 10) {
                ++wait_steps;
                break;
            }

            // Étape 3 : mesurer
            measure_camera_brightness();
            _state = ID_SET_RIGHT_CORNER_ODOM;
            break;
        }
        
        case ID_SET_RIGHT_CORNER_ODOM:
        {
            set_speed(0.0, 0.0);

            // Force odometry at the known corner. Do not trust odometry before this point.
            _y = 4.4;
            _x = -9.43;
            _theta = -M_PI / 2.0;  // orientation vers le mur/coin après calibration
            _id_forward_heading = _theta;

            cout << "[ODOM_RESET_RIGHT_CORNER] x=" << _x
                 << " y=" << _y
                 << " theta=" << _theta << endl;

            _state =ID_CALIB_DONE ;
            break;
        }

        

        case ID_CALIB_DONE:
        {
            set_speed(0.0, 0.0);
            cout << "[DONE] Back-wall + right-corner odometry calibration finished." << endl;
            _state =ID_CLASSIFY ;
            break;
        }

        case ID_CLASSIFY:
        {
            int world_id = classify_world_full();
            _world_id = world_id;

            cout << "Classified world as ID " << _world_id << endl;

            load_path_for_world(_world_id);

            _current_wp = 0;
            _return_wp = -1;
            _stuck_ticks = 0;
            _last_dist_to_wp = 1e9;

            if (_path.empty()) {
                set_speed(0.0, 0.0);
                cout << "[ERROR] No path loaded for world " << _world_id << endl;
                _state = DONE;
                break;
            }

            cout << "[PATH] Loaded " << _path.size()
                << " waypoints for world " << _world_id << endl;

            _state = NAVIGATE_WAYPOINT;
            break;
        }

        case NAVIGATE_WAYPOINT:
        {
            if (_current_wp >= (int)_path.size()) {
                set_speed(0.0, 0.0);
                cout << "[NAV] All waypoints reached." << endl;

                _return_wp = (int)_path.size() - 1;
                _state = RETURN_PATH;
                break;
            }

            Waypoint wp = _path[_current_wp];

            cout << "[NAV] Going to wp=" << _current_wp
                << " target=(" << wp.x << "," << wp.y << ")"
                << " pos=(" << _x << "," << _y << ")"
                << " dist=" << dist_to(wp.x, wp.y)
                << endl;

            if (go_to_point(wp.x, wp.y)) {
                cout << "[WAYPOINT] reached " << _current_wp << endl;
                _current_wp++;
            }

            break;
        }

        case RETURN_PATH:
        {
            if (_return_wp >= 0) {
                Waypoint wp = _path[_return_wp];

                cout << "[RETURN] Going to wp=" << _return_wp
                    << " target=(" << wp.x << "," << wp.y << ")"
                    << " pos=(" << _x << "," << _y << ")"
                    << " dist=" << dist_to(wp.x, wp.y)
                    << endl;

                if (go_to_point(wp.x, wp.y)) {
                    cout << "[RETURN] waypoint reached " << _return_wp << endl;
                    _return_wp--;
                }

                break;
            }

            if (go_to_point(_start_x, _start_y)) {
                set_speed(0.0, 0.0);
                cout << "[RETURN] Start reached. Task complete." << endl;
                _state = TASK_COMPLETE;
            }

            break;
        }
    }

        cout << " x=" << _x << " y=" << _y << " θ=" << _theta
             << " front=" << front << " right=" << right
             << " victims=" << _victims_found
             << " current_wp=" << _current_wp
             << " return_wp=" << _return_wp
             << " path_size=" << _path.size()
             << endl;
    }

    set_speed(0.0, 0.0);
    
}





//////////////////////////////////////////////
// Phase 2 helpers: world identification
//////////////////////////////////////////////

void MyRobot::measure_camera_brightness()
{
    _id_brightness = 0.0;
    _id_avg_r = _id_avg_g = _id_avg_b = 0.0;

    cout << "[Brightness DEBUG] forward=" << (_forward_camera ? "OK" : "NULL")
         << " spherical=" << (_spherical_camera ? "OK" : "NULL") << endl;

    Camera* cam = _forward_camera ? _forward_camera : _spherical_camera;
    cam->saveImage("/tmp/camera_capture.png", 100);
    if (!cam) { cout << "[Brightness DEBUG] no cam!" << endl; return; }

    cout << "[Brightness DEBUG] using cam, w=" << cam->getWidth()
         << " h=" << cam->getHeight() << endl;

    const unsigned char* img = cam->getImage();
    cout << "[Brightness DEBUG] img ptr=" << (void*)img << endl;
    if (!img) { cout << "[Brightness DEBUG] img null!" << endl; return; }

    // Échantillonner 3 pixels au hasard pour voir ce qui sort
    int width = cam->getWidth();
    int height = cam->getHeight();
    cout << "[Brightness DEBUG] center px = "
         << (int)Camera::imageGetRed(img, width, width/2, height/2) << "/"
         << (int)Camera::imageGetGreen(img, width, width/2, height/2) << "/"
         << (int)Camera::imageGetBlue(img, width, width/2, height/2)
         << " | corner px = "
         << (int)Camera::imageGetRed(img, width, 0, 0) << "/"
         << (int)Camera::imageGetGreen(img, width, 0, 0) << "/"
         << (int)Camera::imageGetBlue(img, width, 0, 0)
         << endl;
    if (width <= 0 || height <= 0) return;

    int x0 = width / 4;
    int x1 = (3 * width) / 4;
    int y0 = height / 4;
    int y1 = (3 * height) / 4;

    double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
    int count = 0;
    int saturated_pixels = 0;

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            unsigned char r = Camera::imageGetRed(img, width, x, y);
            unsigned char g = Camera::imageGetGreen(img, width, x, y);
            unsigned char b = Camera::imageGetBlue(img, width, x, y);
            sum_r += r; sum_g += g; sum_b += b;
            if (r >= 250 || g >= 250 || b >= 250) ++saturated_pixels;
            ++count;
        }
    }

    if (count == 0) return;

    _id_avg_r = sum_r / count;
    _id_avg_g = sum_g / count;
    _id_avg_b = sum_b / count;
    _id_brightness = (_id_avg_r + _id_avg_g + _id_avg_b) / 3.0;

    double sat_ratio = (double)saturated_pixels / (double)count;

    double var_r = 0.0, var_g = 0.0, var_b = 0.0;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            double dr = (double)Camera::imageGetRed(img, width, x, y)   - _id_avg_r;
            double dg = (double)Camera::imageGetGreen(img, width, x, y) - _id_avg_g;
            double db = (double)Camera::imageGetBlue(img, width, x, y)  - _id_avg_b;
            var_r += dr*dr; var_g += dg*dg; var_b += db*db;
        }
    }
    double variance = (var_r + var_g + var_b) / (3.0 * count);
    _id_variance = variance;
    _id_saturation_ratio = sat_ratio;
    cout << "[Brightness] avg R/G/B = " << _id_avg_r << "/" << _id_avg_g << "/" << _id_avg_b
         << " brightness=" << _id_brightness
         << " variance=" << variance
         << " sat_ratio=" << sat_ratio
         << endl;
}

//////////////////////////////////////////////
int MyRobot::classify_world_full()
{
    cout << "[Classify LIGHT ONLY] "
         << "R=" << _id_avg_r
         << " G=" << _id_avg_g
         << " B=" << _id_avg_b
         << " brightness=" << _id_brightness
         << " variance=" << _id_variance
         << " sat_ratio=" << _id_saturation_ratio
         << endl;

    // M9 : très lumineux, cyan/blanc saturé
    if (_id_brightness >= 210.0 &&
        _id_avg_r >= 150.0 &&
        _id_avg_g >= 230.0 &&
        _id_avg_b >= 230.0) {
        cout << "[Classify LIGHT ONLY] signature => WORLD 9" << endl;
        return 9;
    }

    // M3 : brightness autour de 134
    if (_id_brightness >= 125.0 && _id_brightness < 145.0) {
        cout << "[Classify LIGHT ONLY] signature => WORLD 3" << endl;
        return 3;
    }

    // M2 : brightness autour de 154.667
    if (_id_brightness >= 145.0 && _id_brightness < 156.5) {
        cout << "[Classify LIGHT ONLY] signature => WORLD 2" << endl;
        return 2;
    }

    // M4 / M5 / M6 / M7 : même luminosité autour de 157.667
    if (_id_brightness >= 156.5 && _id_brightness < 170.0) {
        cout << "[Classify LIGHT ONLY] group 4/5/6/7 => idmonde4567()" << endl;
        return idmonde4567();
    }

    // M1 / M8 / M10 : même luminosité autour de 87
    if (_id_brightness >= 80.0 && _id_brightness < 95.0) {
        cout << "[Classify LIGHT ONLY] group 1/8/10 => idmonde1810()" << endl;
        return idmonde1810();
    }

    cout << "[Classify LIGHT ONLY] Unknown world" << endl;
    return -1;
}

//////////////////////////////////////////////
// Core sensors, odometry, movement helpers
//////////////////////////////////////////////

void MyRobot::apply_gps_correction()
{
    if (!_my_gps) return;
    const double* vals = _my_gps->getValues();
    // GPS axes match the original controller mapping
    double gx = vals[2];
    double gy = vals[0];

    double dx = gx - _x;
    double dy = gy - _y;
    // Only blend if the GPS reading is within a plausible range of odometry.
    // Low weight (0.1) because GPS has 3 m resolution.
    if (sqrt(dx*dx + dy*dy) < 4.0) {
        _x += 0.1f * (float)dx;
        _y += 0.1f * (float)dy;
    }
}

//////////////////////////////////////////////

void MyRobot::compute_odometry()
{
    double left_enc  = _left_wheel_sensor  ? _left_wheel_sensor->getValue()  : 0.0;
    double right_enc = _right_wheel_sensor ? _right_wheel_sensor->getValue() : 0.0;

    double dl = encoder_tics_to_meters((float)(left_enc  - _prev_left_enc));
    double dr = encoder_tics_to_meters((float)(right_enc - _prev_right_enc));

    _prev_left_enc  = (float)left_enc;
    _prev_right_enc = (float)right_enc;

    double ds     = (dl + dr) / 2.0;
    double dtheta = (dr - dl) / WHEELS_DISTANCE;

    _x     += (float)(ds * cos(_theta + dtheta / 2.0));
    _y     += (float)(ds * sin(_theta + dtheta / 2.0));
    _theta  = (float)normalize_angle(_theta + dtheta);
}

//////////////////////////////////////////////

double MyRobot::get_heading_radians()
{
    const double* v = _my_compass->getValues();
    return atan2(-v[2], -v[0]);
}

//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

double MyRobot::normalize_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

//////////////////////////////////////////////
bool MyRobot::turn_to_heading(double target)
{
    return turn_to_heading(target, SPEED_ROTATE);
}

bool MyRobot::turn_to_heading(double target, double max_speed)
{
    double current = normalize_angle(get_heading_radians());
    double error = normalize_angle(target - current);

    _theta = current;

    if (fabs(error) < ANGLE_TOL) {
        set_speed(0.0, 0.0);

        cout << "[turn_to_heading] REACHED target=" << target
             << " theta=" << _theta
             << " error=" << error << endl;

        return true;
    }

    double speed = ROT_KP * fabs(error);

    if (speed > max_speed) speed = max_speed;
    if (speed < ROT_MIN_SPEED) speed = ROT_MIN_SPEED;

    if (fabs(error) < 0.20) {
        speed = 0.30;
    }

    if (error < 0.0) {
        // Need to decrease theta.
        set_speed(speed, -speed);
        cout << "[turn_to_heading] TURN NEG target=" << target
            << " theta=" << _theta
            << " error=" << error
            << " L=" << speed
            << " R=" << -speed << endl;
    } else {
        // Need to increase theta.
        set_speed(-speed, speed);
        cout << "[turn_to_heading] TURN POS target=" << target
            << " theta=" << _theta
            << " error=" << error
            << " L=" << -speed
            << " R=" << speed << endl;
    }

    return false;
}

//////////////////////////////////////////////

void MyRobot::set_speed(double left, double right)
{
    if (left  >  MAX_SPEED) left  =  MAX_SPEED;
    if (left  < -MAX_SPEED) left  = -MAX_SPEED;
    if (right >  MAX_SPEED) right =  MAX_SPEED;
    if (right < -MAX_SPEED) right = -MAX_SPEED;

    _left_speed  = left;
    _right_speed = right;
    if (_left_wheel_motor)  _left_wheel_motor->setVelocity(_left_speed);
    if (_right_wheel_motor) _right_wheel_motor->setVelocity(_right_speed);
}

//////////////////////////////////////////////

double MyRobot::dist_to(double tx, double ty)
{
    double dx = tx - _x;
    double dy = ty - _y;
    return sqrt(dx*dx + dy*dy);
}

//////////////////////////////////////////////

double MyRobot::front_obstacle()
{
    double max_val = 0.0;
    int ids[] = {0, 1, 14, 15};
    for (int i = 0; i < 4; i++)
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    return max_val;
}

//////////////////////////////////////////////

double MyRobot::left_obstacle()
{
    int ids[] = {4, 5, 6};
    vector<double> values;
    values.reserve(3);

    for (int i = 0; i < 3; ++i) {
        if (_ds[ids[i]])
            values.push_back(_ds[ids[i]]->getValue());
    }

    if (values.empty())
        return 0.0;

    sort(values.begin(), values.end());
    return values[values.size() / 2];
}

//////////////////////////////////////////////

double MyRobot::right_obstacle()
{
    int ids[] = {9, 10, 11};
    vector<double> values;
    values.reserve(3);

    for (int i = 0; i < 3; ++i) {
        if (_ds[ids[i]])
            values.push_back(_ds[ids[i]]->getValue());
    }

    if (values.empty()) {
        cout << "[right_obstacle] NO SENSORS!" << endl;
        return 0.0;
    }

    sort(values.begin(), values.end());
    double median = values[values.size() / 2];
    if (_state == ID_DRIVE_TO_RIGHT_WALL || _state == ID_TURN_RIGHT) {
        cout << "[right_obstacle] ds[9]=" << (_ds[9] ? _ds[9]->getValue() : -1)
             << " ds[10]=" << (_ds[10] ? _ds[10]->getValue() : -1)
             << " ds[11]=" << (_ds[11] ? _ds[11]->getValue() : -1)
             << " median=" << median << endl;
    }
    return median;
}

//////////////////////////////////////////////
// Debug helpers
//////////////////////////////////////////////


const char* MyRobot::object_name(DetectedObject obj)
{
    switch (obj) {
        case OBJ_WALL:    return "WALL";
        case OBJ_CUBE:    return "CUBE";
        case OBJ_NOTHING: return "NOTHING";
        default:          return "UNKNOWN";
    }
}


////////////////////////////////////////////
// path following 
//////////////////////////////////////////////
// Phase 3/4: path navigation and victim scan
//////////////////////////////////////////////

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


// ===== WALL-BASED ODOMETRY CALIBRATION =====

bool MyRobot::drive_to_back_wall()
{
    double front = front_obstacle();

    cout << "[DRIVE_TO_BACK_WALL] front=" << front
         << " wall_seen=" << _back_wall_seen
         << " push_steps=" << _back_wall_push_steps
         << " odom_x=" << _x
         << " odom_y=" << _y
         << " theta=" << _theta << endl;

    const double BACK_CONTACT_THRESHOLD = 250.0;
    const int    BACK_PUSH_STEPS       = 40;

    // 1) Si le mur arrière est détecté, on passe en mode "push"
    if (front > BACK_CONTACT_THRESHOLD && !_back_wall_seen) {
        _back_wall_seen = true;
        _back_wall_push_steps = 0;
        cout << "[DRIVE_TO_BACK_WALL] Back wall detected. Slow push." << endl;
    }

    // 2) Une fois détecté : pousser doucement, mais avec correction d'angle
    if (_back_wall_seen) {
        _back_wall_push_steps++;

        if (_back_wall_push_steps < BACK_PUSH_STEPS) {
            double target = BACK_WALL_HEADING;
            double err = normalize_angle(target - _theta);

            double corr = 1.5 * err;
            if (corr > 0.25) corr = 0.25;
            if (corr < -0.25) corr = -0.25;

            double push_speed = 0.8;

            set_speed(push_speed - corr, push_speed + corr);
            return false;
        }

        set_speed(0.0, 0.0);
        cout << "[DRIVE_TO_BACK_WALL] Back wall reached after slow push." << endl;
        return true;
    }


    set_speed(5 , 5 );
    return false;
}
bool MyRobot::drive_to_right_corner_after_back_wall()
{
    // ─────────────────────────────────────────────────────────────
    // Roule tout droit en gardant le cap (_id_scan_heading_target)
    // jusqu'à voir un mur en face. Puis approche lentement et
    // s'arrête à ~10-20 cm du mur.
    //
    // Distance sensors (Webots): plus la valeur est GRANDE, plus
    // le mur est PROCHE. Les valeurs sont calibrées empiriquement.
    //
    //   FRONT_FAR_LIMIT (~300)  : mur détecté, on commence à ralentir
    //   FRONT_NEAR_LIMIT (~900) : ~10-20 cm, on s'ARRÊTE
    // ─────────────────────────────────────────────────────────────

    _id_wall_steps++;

    double front = front_obstacle();
    double heading_err = normalize_angle(_id_scan_heading_target - _theta);

    // Seuils — capteur retourne ~800-900 quand le mur est à ~5 cm
    const double FRONT_FAR_LIMIT  = 300.0;   // mur visible → passer en vitesse lente
    const double FRONT_STOP_LIMIT = 800.0;   // ~5 cm du mur → STOP
    const double CRUISE_SPEED     = 3.0;     // vitesse croisière (mur pas encore visible)
    const double APPROACH_SPEED   = 0.8;     // approche lente (mur en vue)

    cout << "[DRIVE_TO_LEFT_CORNER] front=" << front
         << " theta=" << _theta
         << " heading_err=" << heading_err
         << " x=" << _x << " y=" << _y
         << " steps=" << _id_wall_steps << endl;

    // 1) Mur très proche → STOP. Calibration odométrique au coin.
    if (front > FRONT_STOP_LIMIT) {
        set_speed(0.0, 0.0);
        cout << "[DRIVE_TO_LEFT_CORNER] STOP — wall ~10-20 cm ahead. front="
             << front << endl;
        return true;
    }

    // 2) Sécurité : timeout pour ne pas rouler éternellement
    if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
        set_speed(0.0, 0.0);
        cout << "[DRIVE_TO_LEFT_CORNER] TIMEOUT — no wall found in "
             << WALL_DRIVE_TIMEOUT << " steps." << endl;
        return true;
    }

    // 3) Choix de la vitesse : croisière tant que le mur est loin, approche lente quand il est visible
    double base = (front > FRONT_FAR_LIMIT) ? APPROACH_SPEED : CRUISE_SPEED;

    // 4) Correction de cap (P contrôle simple)
    double corr = ID_HEADING_KP * heading_err;
    if (corr >  0.6) corr =  0.6;
    if (corr < -0.6) corr = -0.6;

    set_speed(base - corr, base + corr);
    return false;
}


bool MyRobot::turn_to_heading_forced_left(double target, double speed)
{
    double current = normalize_angle(get_heading_radians());
    _theta = current;

    double error = normalize_angle(target - current);

    if (fabs(error) < ANGLE_TOL) {
        set_speed(0.0, 0.0);
        cout << "[turn_forced_left] REACHED target=" << target
             << " theta=" << _theta
             << " error=" << error << endl;
        return true;
    }

    // Always rotate in the same physical direction.
    set_speed(-speed, speed);

    cout << "[turn_forced_left] TURNING target=" << target
         << " theta=" << _theta
         << " error=" << error
         << " L=" << -speed
         << " R=" << speed << endl;

    return false;
}

bool MyRobot::go_to_point(double gx, double gy)
{
    // ============================================================
    // Persistent obstacle avoidance for go_to_point()
    // 0 = normal navigation
    // 1 = short backup
    // 2 = turn away from obstacle
    // 3 = follow around obstacle
    // ============================================================
    static int avoid_mode = 0;
    static int avoid_steps = 0;
    static double avoid_start_dist = 0.0;
    static int avoid_side = 0; // -1 = bypass right, +1 = bypass left
    static double last_gx = 9999.0;
    static double last_gy = 9999.0;

    // Reset avoidance when target changes
    if (fabs(gx - last_gx) > 0.01 || fabs(gy - last_gy) > 0.01) {
        avoid_mode = 0;
        avoid_steps = 0;
        avoid_start_dist = 0.0;
        avoid_side = 0;
        last_gx = gx;
        last_gy = gy;
    }

    double front = front_obstacle();
    double left  = left_obstacle();
    double right = right_obstacle();

    double dist = dist_to(gx, gy);
    double target_angle = atan2(gy - _y, gx - _x);
    double angle_error = normalize_angle(target_angle - _theta);

    const double TARGET_REACHED = 0.10;

    const double FRONT_DETECT = 700.0;
    const double FRONT_HARD   = 1000.0;
    const double FRONT_CLEAR  = 250.0;

    const double SIDE_CLOSE = 650.0;
    const double SIDE_LOST  = 120.0;

    const double TURN_ONLY_ERR = 0.25;

    const double BASE_SPEED = 2.2;
    const double SLOW_SPEED = 1.4;
    const double TURN_SPEED = 1.6;

    // Goal reached
    if (dist < TARGET_REACHED) {
        set_speed(0.0, 0.0);
        avoid_mode = 0;
        avoid_steps = 0;

        cout << "[GO_TO_POINT] REACHED target=(" << gx << "," << gy
             << ") current=(" << _x << "," << _y << ")" << endl;

        return true;
    }

    // ============================================================
    // 1) Start obstacle avoidance
    // ============================================================
    if (avoid_mode == 0 && front > FRONT_DETECT) {
        avoid_start_dist = dist;
        avoid_steps = 0;

        // Lower sensor value = freer side
        if (right <= left) {
            avoid_side = -1; // go around by the right
            cout << "[GO_TO_POINT_AVOID] START bypass RIGHT front="
                 << front << " left=" << left << " right=" << right << endl;
        } else {
            avoid_side = +1; // go around by the left
            cout << "[GO_TO_POINT_AVOID] START bypass LEFT front="
                 << front << " left=" << left << " right=" << right << endl;
        }

        if (front > FRONT_HARD) {
            avoid_mode = 1; // backup first
        } else {
            avoid_mode = 2; // directly turn away
        }

        return false;
    }

    // ============================================================
    // 2) Backup if too close
    // ============================================================
    if (avoid_mode == 1) {
        avoid_steps++;

        set_speed(-1.8, -1.8);

        if (avoid_steps >= 10) {
            avoid_mode = 2;
            avoid_steps = 0;
        }

        cout << "[GO_TO_POINT_AVOID] BACKUP front=" << front
             << " step=" << avoid_steps << endl;

        return false;
    }

    // ============================================================
    // 3) Turn away from obstacle until front is clear
    // ============================================================
    if (avoid_mode == 2) {
        avoid_steps++;

        if (avoid_side < 0) {
            // bypass right: turn right
            set_speed(TURN_SPEED, -TURN_SPEED);
        } else {
            // bypass left: turn left
            set_speed(-TURN_SPEED, TURN_SPEED);
        }

        cout << "[GO_TO_POINT_AVOID] TURN "
             << (avoid_side < 0 ? "RIGHT" : "LEFT")
             << " front=" << front
             << " step=" << avoid_steps << endl;

        if (front < FRONT_CLEAR || avoid_steps > 45) {
            avoid_mode = 3;
            avoid_steps = 0;
        }

        return false;
    }

    // ============================================================
    // 4) Follow around the obstacle
    // ============================================================
    if (avoid_mode == 3) {
        avoid_steps++;

        double current_dist = dist_to(gx, gy);
        double current_target_angle = atan2(gy - _y, gx - _x);
        double current_angle_error = normalize_angle(current_target_angle - _theta);

        // Leave avoidance only when:
        // - front is clear
        // - we moved enough around the obstacle
        // - target direction is reasonably reachable
        // - we are closer than when we hit the obstacle
        if (avoid_steps > 35 &&
            front < FRONT_CLEAR &&
            fabs(current_angle_error) < 0.45 &&
            current_dist < avoid_start_dist - 0.15)
        {
            cout << "[GO_TO_POINT_AVOID] EXIT avoidance, back to target. dist="
                 << current_dist << " start_dist=" << avoid_start_dist << endl;

            avoid_mode = 0;
            avoid_steps = 0;
            return false;
        }

        // Safety: if avoidance is taking too long, allow retry toward target
        if (avoid_steps > 220 && front < FRONT_DETECT) {
            cout << "[GO_TO_POINT_AVOID] TIMEOUT exit, retry target." << endl;
            avoid_mode = 0;
            avoid_steps = 0;
            return false;
        }

        if (avoid_side < 0) {
            // Bypass RIGHT: obstacle should stay on the LEFT side.
            if (front > FRONT_DETECT) {
                set_speed(TURN_SPEED, -TURN_SPEED);       // turn right
            }
            else if (left > SIDE_CLOSE) {
                set_speed(2.4, 1.0);                      // too close left -> steer right
            }
            else if (left < SIDE_LOST) {
                set_speed(1.0, 2.2);                      // lost obstacle -> curve left
            }
            else {
                set_speed(BASE_SPEED, BASE_SPEED);        // follow side
            }
        } else {
            // Bypass LEFT: obstacle should stay on the RIGHT side.
            if (front > FRONT_DETECT) {
                set_speed(-TURN_SPEED, TURN_SPEED);       // turn left
            }
            else if (right > SIDE_CLOSE) {
                set_speed(1.0, 2.4);                      // too close right -> steer left
            }
            else if (right < SIDE_LOST) {
                set_speed(2.2, 1.0);                      // lost obstacle -> curve right
            }
            else {
                set_speed(BASE_SPEED, BASE_SPEED);        // follow side
            }
        }

        cout << "[GO_TO_POINT_AVOID] FOLLOW side="
             << (avoid_side < 0 ? "RIGHT" : "LEFT")
             << " front=" << front
             << " left=" << left
             << " right=" << right
             << " dist=" << current_dist
             << " err=" << current_angle_error
             << " step=" << avoid_steps << endl;

        return false;
    }

    // ============================================================
    // 5) Normal go-to-point navigation
    // ============================================================
    if (fabs(angle_error) > TURN_ONLY_ERR) {
        if (angle_error > 0.0) {
            set_speed(-TURN_SPEED, TURN_SPEED);
        } else {
            set_speed(TURN_SPEED, -TURN_SPEED);
        }

        cout << "[GO_TO_POINT] TURN target=(" << gx << "," << gy << ")"
             << " dist=" << dist
             << " target_theta=" << target_angle
             << " theta=" << _theta
             << " err=" << angle_error << endl;

        return false;
    }

    double base = (dist < 0.7) ? SLOW_SPEED : BASE_SPEED;
    double corr = 2.0 * angle_error;

    double left_speed  = base - corr;
    double right_speed = base + corr;

    set_speed(left_speed, right_speed);

    cout << "[GO_TO_POINT] DRIVE target=(" << gx << "," << gy << ")"
         << " current=(" << _x << "," << _y << ")"
         << " dist=" << dist
         << " theta=" << _theta
         << " err=" << angle_error << endl;

    return false;
}

// detection spec monde 
double MyRobot::measure_front_avg(int samples)
{
    double sum = 0.0;

    for (int i = 0; i < samples; ++i) {
        step(_time_step);
        compute_odometry();
        set_speed(0.0, 0.0);
        sum += front_obstacle();
    }

    return sum / samples;
}
double MyRobot::go_to_point_and_measure_front(double gy, double gx, double final_theta)
{
    int safety = 0;

    while (step(_time_step) != -1 && safety < 1500) {
        compute_odometry();

        if (go_to_point(gx, gy)) {
            set_speed(0.0, 0.0);
            break;
        }

        safety++;
    }

    set_speed(0.0, 0.0);

    for (int i = 0; i < 10; ++i) {
        step(_time_step);
        compute_odometry();
        set_speed(0.0, 0.0);
    }

    safety = 0;

    while (step(_time_step) != -1 && safety < 500) {
        compute_odometry();

        if (turn_to_heading(final_theta, 1.0)) {
            set_speed(0.0, 0.0);
            break;
        }

        safety++;
    }

    set_speed(0.0, 0.0);

    for (int i = 0; i < 10; ++i) {
        step(_time_step);
        compute_odometry();
        set_speed(0.0, 0.0);
    }

    double front = measure_front_avg(8);

    cout << "[ID_POINT_FRONT] gx=" << gx
         << " gy=" << gy
         << " final_theta=" << final_theta
         << " front=" << front
         << endl;

    return front;
}

int MyRobot::idmonde4567()
{
    const double OPEN_MAX = 80.0;
    const double WALL_MIN = 120.0;
    const double CUBE_MIN = 650.0;

    cout << "[idmonde4567] Start" << endl;

    double front_p1 = go_to_point_and_measure_front(
        4.26,
        -7,
        0.0
    );

    if (front_p1 < OPEN_MAX) {
        cout << "[idmonde4567] P1 open => possible WORLD 6 or 7" << endl;

        double front_p2 = go_to_point_and_measure_front(
            -1,
            -6.4,
            0.0
        );

        if (front_p2 >= WALL_MIN) {
            cout << "[idmonde4567] P2 wall detected => WORLD 7" << endl;
            return 7;
        }

        cout << "[idmonde4567] P2 open => WORLD 6" << endl;
        return 6;
    }

    cout << "[idmonde4567] P1 wall detected => possible WORLD 4 or 5" << endl;

    double front_p3 = go_to_point_and_measure_front(
        2.3,
        -7.7,
        -M_PI / 2.0
    );

    if (front_p3 >= CUBE_MIN) {
        cout << "[idmonde4567] P3 cube detected => WORLD 5" << endl;
        return 5;
    }

    cout << "[idmonde4567] P2 open => WORLD 4" << endl;
    return 4;
}

int MyRobot::idmonde1810()
{
    const double OPEN_MAX = 80.0;
    const double WALL_MIN = 120.0;
    const double CUBE_MIN = 650.0;

    cout << "[idmonde4567] Start" << endl;

    double front_p1 = go_to_point_and_measure_front(
        4.26,
        -7,
        0.0
    );

    if (front_p1 < OPEN_MAX) {
        cout << "[idmonde4567] P1 open => world 8" << endl;
        return 8;
        
    }
    else {
        cout << "[idmonde4567] P1 wall detected => possible WORLD 1 or 10" << endl;

        double front_p2 = go_to_point_and_measure_front(
            2.96707,
            -6.8,
            0.0
        );

        if (front_p2 >= WALL_MIN) {
            cout << "[idmonde4567] P2 wall detected => WORLD 10" << endl;
            return 10;
        }

        cout << "[idmonde4567] P2 open => WORLD 1" << endl;
        return 1;
    }
}