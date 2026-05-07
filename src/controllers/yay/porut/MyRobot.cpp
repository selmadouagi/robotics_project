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
            _state == ID_TURN_LEFT_FROM_BACK_WALL ||
            _state == ID_DRIVE_TO_RIGHT_CORNER ||
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
        double right  = right_obstacle();

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

                _state = ID_TURN_LEFT_FROM_BACK_WALL;
            }
            break;
        }

        case ID_TURN_LEFT_FROM_BACK_WALL:
        {
            if (turn_to_heading(LEFT_FROM_BACK_HEADING, 0.7)) {
                cout << "[CALIB] Left turn done. Drive to right corner." << endl;
                _corner_wall_seen = false;
                _corner_push_steps = 0;
                _state = ID_DRIVE_TO_RIGHT_CORNER;
            }
            break;
        }

        case ID_DRIVE_TO_RIGHT_CORNER:
        {
            if (drive_to_right_corner_after_back_wall()) {
                _state = ID_SET_RIGHT_CORNER_ODOM;
            }
            break;
        }

        case ID_SET_RIGHT_CORNER_ODOM:
        {
            set_speed(0.0, 0.0);

            // Force odometry at the known corner. Do not trust odometry before this point.
            _x = -4.6;
            _y = -9.46;
            _theta = -1.5708;   // orientation vers le mur/coin après calibration
            _id_forward_heading = _theta;

            cout << "[ODOM_RESET_RIGHT_CORNER] x=" << _x
                 << " y=" << _y
                 << " theta=" << _theta << endl;

            _state = ID_MEASURE_LIGHT_CALIB;
            break;
        }

        case ID_MEASURE_LIGHT_CALIB:
        {
            // Étape 1 : tourner de 90° à gauche pour passer de θ=-1.5708
            // (face au mur de calibration) à θ=0 (face au couloir ouvert).
            // Sinon la caméra ne voit que le ciel/skybox.
            if (!turn_to_heading(0.0, ID_ROT_SPEED)) {
                // Encore en train de tourner
                break;
            }

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
            _state = ID_CALIB_DONE;
            break;
        }

        case ID_CALIB_DONE:
        {
            set_speed(0.0, 0.0);
            cout << "[DONE] Back-wall + right-corner odometry calibration finished." << endl;
            return;
        }
}
        cout << "[" << state_name() << "]"
             << " x=" << _x << " y=" << _y << " θ=" << _theta
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
    const unsigned char* image = _forward_camera->getImage();  // Get image from the forward camera / Obtenir l'image de la caméra avant
        int width = _forward_camera->getWidth();  // Get the image width / Obtenir la largeur de l'image
        int height = _forward_camera->getHeight();  // Get the image height / Obtenir la hauteur de l'image
        
        int center_x = width / 2;  // X-coordinate of the image center / Coordonnée X du centre de l'image
        int center_y = height / 2;  // Y-coordinate of the image center / Coordonnée Y du centre de l'image
    
        int r = _forward_camera->imageGetRed(image, width, center_x, center_y);  // Get the red value at the center pixel / Obtenir la valeur rouge du pixel central
        int g = _forward_camera->imageGetGreen(image, width, center_x, center_y);  // Get the green value at the center pixel / Obtenir la valeur verte du pixel central
        int b = _forward_camera->imageGetBlue(image, width, center_x, center_y);  // Get the blue value at the center pixel / Obtenir la valeur bleue du pixel central
    
        //cout << "🎨 RGB at center - R: " << r << " G: " << g << " B: " << b << endl;  
           double luminosity = 0.299 * r + 0.587 * g + 0.114 * b;
           cout << "Luminosité centre: " << luminosity
               << " | RGB: R=" << r << " G=" << g << " B=" << b << endl;

}

//////////////////////////////////////////////

int MyRobot::classify_world_full()
{
    bool L = _id_left_wall_found;
    bool R = _id_right_wall_found;

    bool obstacle_from_right = _id_right_center_hit;
    bool obstacle_from_left  = _id_left_center_hit;

    double side_diff = fabs(_id_left_wall_dist - _id_right_wall_dist);

    bool side_cube_signature =
        (_id_right_scan_left_peak > ID_SIDE_OBS_CUBE_THRESH) ||
        (_id_left_scan_right_peak  > ID_SIDE_OBS_CUBE_THRESH);

    bool cube_detected =
        _id_cube_ahead ||
        (_id_middle_object == OBJ_CUBE) ||
        side_cube_signature;

    double maxRGB = std::max(_id_avg_r, std::max(_id_avg_g, _id_avg_b));
    double minRGB = std::min(_id_avg_r, std::min(_id_avg_g, _id_avg_b));
    double saturation = maxRGB - minRGB;

    bool is_foggy =
        (_id_avg_b > 100.0 &&
         _id_avg_g > 100.0 &&
         _id_avg_r < 95.0 &&
         fabs(_id_avg_g - _id_avg_b) < 25.0);

    cout << "[Classify] "
         << "R_center=" << (obstacle_from_right ? "HIT" : "OPEN")
         << " distR=" << _id_right_center_front
         << " | L_center=" << (obstacle_from_left ? "HIT" : "OPEN")
         << " distL=" << _id_left_center_front
         << " R=" << _id_avg_r
         << " G=" << _id_avg_g
         << " B=" << _id_avg_b
         << " sat=" << saturation
         << " foggy=" << is_foggy
         << " L=" << (L ? "WALL" : "NO")
         << " R=" << (R ? "WALL" : "NO")
         << " dDiff=" << side_diff
         << " sidePeakSig=" << (side_cube_signature ? "YES" : "NO")
         << " cube=" << (cube_detected ? "YES" : "NO")
         << endl;

    if (_id_brightness >= 84.0 && _id_brightness < 88.5 &&
        _id_avg_r >= 40.0 && _id_avg_r <= 55.0 &&
        _id_avg_g >= 95.0 && _id_avg_g <= 115.0 &&
        _id_avg_b >= 95.0 && _id_avg_b <= 115.0 &&
        _id_left_wall_found && _id_left_wall_dist < 1.5) {
        
        cout << "[Classify] Direct light/color signature => WORLD 5" << endl;
        return 5;
    }   


    if (!obstacle_from_right && obstacle_from_left) {
        cout << "[Classify] Signature: right center OPEN, left center HIT." << endl;

        if (_id_brightness <= 80) {
            return 10;
        } else {
            return 4;
        }
    }

    if (obstacle_from_right && !obstacle_from_left) {
        cout << "[Classify] Signature: right center HIT, left center OPEN." << endl;
        return 8;
    }

    if (!obstacle_from_right && !obstacle_from_left) {
        cout << "[Classify] Signature: both center probes OPEN." << endl;

        if (_id_brightness <= 100) {
            return 7;
        }

        return 6;
    }

    if (obstacle_from_right && obstacle_from_left) {
        if (_id_cube_on_left) {
            if (_id_brightness <= 100) {
                return 2;
            } else {
                return 3;
            }
        } else {
            if (_id_brightness <= 60) {
                return 9;
            } else {
                return 1;
            }
        }
    }

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

const char* MyRobot::state_name()
{
    switch (_state) {
        case ID_TURN_TO_BACK_WALL:       return "ID_TURN_TO_BACK_WALL";
        case ID_DRIVE_TO_BACK_WALL:      return "ID_DRIVE_TO_BACK_WALL";
        case ID_TURN_LEFT_FROM_BACK_WALL:return "ID_TURN_LEFT_FROM_BACK_WALL";
        case ID_DRIVE_TO_RIGHT_CORNER:   return "ID_DRIVE_TO_RIGHT_CORNER";
        case ID_SET_RIGHT_CORNER_ODOM:   return "ID_SET_RIGHT_CORNER_ODOM";
        case ID_MEASURE_LIGHT_CALIB:     return "ID_MEASURE_LIGHT_CALIB";
        case ID_CALIB_DONE:              return "ID_CALIB_DONE";
        case ID_FACE_FORWARD:        return "ID_FACE_FWD";
        case ID_MEASURE_LIGHT:       return "ID_LIGHT";
        case ID_TURN_RIGHT:          return "ID_TURN_R";
        case ID_SETTLE_RIGHT:        return "ID_SETTLE_R";
        case ID_DRIVE_TO_RIGHT_WALL: return "ID_SCAN_R";
        case ID_TURN_RIGHT_TO_CENTER:return "ID_TURN_R_C";
        case ID_MEASURE_RIGHT_CENTER:return "ID_MEAS_R_C";
        case ID_PROBE_RIGHT_CENTER:  return "ID_PROBE_R_C";
        case ID_BACKUP_RIGHT_CENTER: return "ID_BACK_R_C";
        case ID_TURN_RIGHT_TO_SCAN:  return "ID_TURN_R_S";
        case ID_RETURN_FROM_RIGHT:   return "ID_BACK_R";
        case ID_TURN_LEFT:           return "ID_TURN_L";
        case ID_SETTLE_LEFT:         return "ID_SETTLE_L";
        case ID_DRIVE_TO_LEFT_WALL:  return "ID_SCAN_L";
        case ID_TURN_LEFT_TO_CENTER: return "ID_TURN_L_C";
        case ID_MEASURE_LEFT_CENTER: return "ID_MEAS_L_C";
        case ID_PROBE_LEFT_CENTER:   return "ID_PROBE_L_C";
        case ID_BACKUP_LEFT_CENTER:  return "ID_BACK_L_C";
        case ID_TURN_LEFT_TO_SCAN:   return "ID_TURN_L_S";
        case ID_RETURN_FROM_LEFT:    return "ID_BACK_L";
        case ID_FACE_FORWARD_AGAIN:  return "ID_FACE_FWD2";
        case ID_CHECK_CUBE_AHEAD:    return "ID_CUBE";
        case ID_CLASSIFY:            return "ID_CLASSIFY";
        case INITIAL_TURN: return "INITIAL_TURN";
        case FOLLOW_PATH: return "FOLLOW_PATH";
        case SPIN_VICTIM: return "SPIN_VICTIM";
        case RETURN_PATH: return "RETURN_PATH";
        case DONE: return "DONE";
        default: return "UNKNOWN";
    }
}

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

    // Réglages locaux pour ne pas toucher au .h
    const double BACK_CONTACT_THRESHOLD = 900.0;
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


    set_speed(4 , 4 );
    return false;
}

bool MyRobot::drive_to_right_corner_after_back_wall()
{
    double front = front_obstacle();
    double right = right_obstacle();

    static int drive_ticks = 0;
    static int front_confirm = 0;

    drive_ticks++;

    cout << "[DRIVE_TO_RIGHT_CORNER] front=" << front
         << " right=" << right
         << " drive_ticks=" << drive_ticks
         << " front_confirm=" << front_confirm
         << " wall_seen=" << _corner_wall_seen
         << " push_steps=" << _corner_push_steps
         << " odom_x=" << _x
         << " odom_y=" << _y
         << " theta=" << _theta << endl;

    const double FRONT_CONTACT = 1000.0;

    /*
       IMPORTANT :
       On interdit la détection du mur devant trop tôt.
       Sinon le robot s'arrête au milieu à cause d'un pic du capteur front.
    */
    const int MIN_DRIVE_TICKS_BEFORE_FRONT = 120;
    const int FRONT_CONFIRM_NEEDED = 6;

    if (!_corner_wall_seen) {
        if (drive_ticks > MIN_DRIVE_TICKS_BEFORE_FRONT && front > FRONT_CONTACT) {
            front_confirm++;
        } else {
            front_confirm = 0;
        }

        if (front_confirm >= FRONT_CONFIRM_NEEDED) {
            _corner_wall_seen = true;
            _corner_push_steps = 0;
            cout << "[DRIVE_TO_RIGHT_CORNER] REAL front wall detected. Final push." << endl;
        }
    }

    if (_corner_wall_seen) {
        _corner_push_steps++;

        if (_corner_push_steps < 20) {
            set_speed(0.25, 0.25);
            return false;
        }

        set_speed(0.0, 0.0);

        drive_ticks = 0;
        front_confirm = 0;

        cout << "[DRIVE_TO_RIGHT_CORNER] Corner reached." << endl;
        return true;
    }

    /*
       Suivi du mur droit :
       - right grand : mur droit proche / bien vu
       - right baisse : il s'éloigne du mur droit
       - right = 0 : il a perdu le mur droit
    */
    double L = 4;
    double R = 4;

    if (right >= 650.0) {
        // Mur droit bien vu : avance vite droit
        L = 5;
        R = 5;
        cout << "[SIDE] ok -> fast straight" << endl;
    }
    else if (right > 250.0 && right < 650.0) {
        // Il commence à s'éloigner du mur droit : resserrer un peu à droite
        L = 0.80;
        R = 0.65;
        cout << "[SIDE] moving away -> tighten right" << endl;
    }
    else if (right > 0.0 && right <= 250.0) {
        // Il est loin du mur droit : resserrer plus fort
        L = 0.75;
        R = 0.50;
        cout << "[SIDE] far from right wall -> stronger right" << endl;
    }
    else {
        // Mur droit perdu : chercher à droite sans faire un gros arc
        L = 0.65;
        R = 0.42;
        cout << "[SIDE] right wall lost -> search right" << endl;
    }

    set_speed(L, R);
    return false;
        
}

bool MyRobot::look_back_right()
{
    double target = normalize_angle(BACK_RIGHT_CORNER_HEADING);

    bool done = turn_to_heading_forced_left(target, 1.2);

    if (done) {
        cout << "[look_back_right] DONE target=" << target
             << " theta=" << _theta << endl;
        return true;
    }

    cout << "[look_back_right] TURNING target=" << target
         << " theta=" << _theta << endl;

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

bool MyRobot::go_to_point(double target_x, double target_y)
{
    double dx = target_x - _x;
    double dy = target_y - _y;

    double distance = sqrt(dx * dx + dy * dy);

    if (distance < 0.08) {
        set_speed(0.0, 0.0);
        std::cout << "[GO_TO_POINT] REACHED target=(" 
                  << target_x << "," << target_y << ") current=("
                  << _x << "," << _y << ")" << std::endl;
        return true;
    }

    double target_theta = atan2(dy, dx);
    double err = normalize_angle(target_theta - _theta);

    // Si l'angle est trop mauvais, on tourne d'abord sur place
    if (fabs(err) > 0.18) {
        double turn = 2.0 * err;

        if (turn > 2.0) turn = 2.0;
        if (turn < -2.0) turn = -2.0;

        set_speed(-turn, turn);

        std::cout << "[GO_TO_POINT] TURN target=(" 
                  << target_x << "," << target_y << ") dist="
                  << distance << " target_theta=" << target_theta
                  << " theta=" << _theta << " err=" << err
                  << std::endl;

        return false;
    }

    // Avance avec correction d'angle
    double speed = 3.0;

    if (distance < 0.60) speed = 2.0;
    if (distance < 0.25) speed = 1.0;

    double corr = 2.5 * err;

    if (corr > 0.8) corr = 0.8;
    if (corr < -0.8) corr = -0.8;

    set_speed(speed - corr, speed + corr);

    std::cout << "[GO_TO_POINT] DRIVE target=(" 
              << target_x << "," << target_y << ") current=("
              << _x << "," << _y << ") dist=" << distance
              << " theta=" << _theta << " err=" << err
              << std::endl;

    return false;
}