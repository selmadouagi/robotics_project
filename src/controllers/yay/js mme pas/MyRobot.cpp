/**
 * @file    MyRobot.cpp
 * @brief   5-Phase Rescue Controller: Localize → Identify → Navigate → Detect → Return
 *
 * @author  Elizabeth Faulkner
 * @date    2026-04
 *
 * FIXES APPLIED:
 *  - Use robot's actual starting heading as "forward" instead of hardcoded ALIGN_HEADING.
 *    This allows the professor to start the robot at any point on the yellow line
 *    facing the maze.
 *  - Lowered ID_ROT_SPEED to 1.5 (was 5.0) to prevent rotation overshoot/oscillation.
 *  - Removed all "+2.0" speed bumps on turn_to_heading calls during identification.
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    _time_step = 64;

    _left_speed = _right_speed = 0.0;
    _x = _y = _theta = 0.0;
    _prev_left_enc = _prev_right_enc = 0.0;

    // Start directly in world identification.
    _state          = ID_FACE_FORWARD;
    _victims_found  = 0;
    _state_after_spin = FOLLOW_PATH;
    _world_id       = 0;
    _current_wp     = 0;
    _gps_timer      = 0;

    // Phase 2 — world identification fields
    _id_brightness        = 0.0;
    _id_avg_r = _id_avg_g = _id_avg_b = 0.0;
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
        if (++_gps_timer >= 78) {
            if (!in_phase2) apply_gps_correction();
            _gps_timer = 0;
        }

        double front = front_obstacle();
        double right  = right_obstacle();

        switch (_state)
        {
        // ════════════════════════════════════════════════════════════════════
        // ──  Phase 2: WORLD IDENTIFICATION (rotate-and-scan, on yellow line) ─
        // ════════════════════════════════════════════════════════════════════

        // 2.1: rotate to forward heading
        // FIXED: Use the robot's CURRENT heading as forward instead of hardcoded
        // ALIGN_HEADING. This allows starting from any point on the yellow line
        // as long as the robot is facing the maze.
        case ID_FACE_FORWARD:
        {
            if (!_id_initialized) {
                _id_forward_heading = _theta;  // Use current heading as forward
                _id_cube_ahead = false;
                if (_my_gps) {
                    _id_initial_gps_x = _my_gps->getValues()[2];
                    _id_initial_gps_y = _my_gps->getValues()[0];
                }
                _id_initialized = true;
                cout << "[Phase2.1] Init. GPS=("
                     << _id_initial_gps_x << ", " << _id_initial_gps_y
                     << ")  forward θ=" << _id_forward_heading << endl;
            }

            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED)) {
                cout << "[Phase2.1] Facing forward (θ=" << _theta << ")." << endl;
                cout << "[Phase2.1] Front sensor at start = "
                     << front_obstacle() << endl;
                _id_line_anchor_x = _x;
                _id_line_anchor_y = _y;
                _state = ID_MEASURE_LIGHT;
            }
            break;
        }

        // 2.2: stationary camera brightness measurement
        case ID_MEASURE_LIGHT:
        {
            set_speed(0.0, 0.0);
            measure_camera_brightness();
            cout << "[Phase2.2] Camera: brightness=" << _id_brightness
                 << "  R=" << _id_avg_r
                 << "  G=" << _id_avg_g
                 << "  B=" << _id_avg_b << endl;
            _id_wall_steps = 0;
            _id_right_scan_left_peak = 0.0;
            _id_left_scan_right_peak = 0.0;
            _id_left_object = OBJ_NOTHING;
            _id_middle_object = OBJ_NOTHING;
            _state = ID_TURN_RIGHT;
            break;
        }

        // 2.3: turn 90° right (target heading = forward - π/2)
        case ID_TURN_RIGHT:
        {
            double target = normalize_angle(_id_forward_heading - M_PI / 2.0);
            double err = normalize_angle(target - _theta);
            cout << "[DEBUG_TURN_R] forward_heading=" << _id_forward_heading
                 << " target=" << target << " current_θ=" << _theta
                 << " err=" << err << " |err|=" << fabs(err) << endl;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.3] Facing right (θ=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_RIGHT;
            }
            break;
        }

        // 2.4a: settle after right turn
        case ID_SETTLE_RIGHT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            cout << "[SETTLE_RIGHT] target=" << _id_scan_heading_target << " theta=" << _theta
                 << " err=" << err << " steps=" << _id_settle_steps << "/" << ID_ROT_SETTLE_STEPS << endl;
            if (fabs(err) > ANGLE_TOL) {
                cout << "  --> returning to TURN_RIGHT (error too large)" << endl;
                _state = ID_TURN_RIGHT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                cout << "  --> moving to DRIVE_TO_RIGHT_WALL" << endl;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_RIGHT_WALL;
            }
            break;
        }

        // 2.4b: drive forward until front sensor sees wall, or timeout
        case ID_DRIVE_TO_RIGHT_WALL:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            double side = right_obstacle();
            _id_right_scan_left_peak = max(_id_right_scan_left_peak, left_obstacle());
            if (_id_wall_steps % 20 == 0) {
                cout << "[DS_RIGHT]";
                for (int i = 0; i < NUM_DS; ++i) {
                    cout << " ds" << i << "=" << (_ds[i] ? _ds[i]->getValue() : -1);
                }
                cout << endl;
            }
            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x)
                                  + (_y - _id_scan_start_y) * (_y - _id_scan_start_y));
            double heading_err = normalize_angle(_id_scan_heading_target - _theta);

            if (fabs(heading_err) > ID_HEADING_DRIFT_ABORT) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.4] Right scan drift (err=" << heading_err
                     << "). Re-aligning." << endl;
                _state = ID_TURN_RIGHT;
                break;
            }

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = true;
                cout << "[Phase2.4] Right scan: WALL at " << travelled << " m"
                    << "  front=" << f << "  side=" << side
                    << "  steps=" << _id_wall_steps << endl;
                _id_settle_steps = 0;
                _state = ID_TURN_RIGHT_TO_CENTER;
            } else if (f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = true;
                cout << "[Phase2.4] Right scan: FRONT wall stop (travelled " << travelled
                     << " m, front=" << f << ", side=" << side << ")" << endl;
                _id_settle_steps = 0;
                _state = ID_TURN_RIGHT_TO_CENTER;
            } else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = false;
                cout << "[Phase2.4] Right scan: NO WALL (travelled " << travelled
                     << " m, timeout)" << endl;
                _state = ID_RETURN_FROM_RIGHT;
            } else {
                double base = (side > OBSTACLE_SIDE_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
                double corr = ID_HEADING_KP * heading_err;
                set_speed(base - corr, base + corr);
            }
            break;
        }

        // 2.4c: turn back toward room center after right wall hit
        case ID_TURN_RIGHT_TO_CENTER:
        {
            double target = _id_forward_heading;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_settle_steps = 0;
                cout << "[Phase2.4b] Facing center from right wall (θ=" << _theta << ")" << endl;
                _state = ID_MEASURE_RIGHT_CENTER;
            }
            break;
        }

        // 2.4d: kick off a forward probe toward room center (right side)
        case ID_MEASURE_RIGHT_CENTER:
        {
            set_speed(0.0, 0.0);
            if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                _id_probe_start_x = _x;
                _id_probe_start_y = _y;
                _id_probe_steps   = 0;
                _id_front_hit_count = 0;
                _id_right_center_hit  = false;
                _id_right_center_front = 0.0;
                cout << "[Phase2.4c] Starting forward probe (right side, max "
                     << PROBE_FORWARD_MAX_M << " m)." << endl;
                _state = ID_PROBE_RIGHT_CENTER;
            }
            break;
        }

        // 2.4e-probe: drive forward up to 2 m, check if there is an obstacle
        case ID_PROBE_RIGHT_CENTER:
        {
            _id_probe_steps++;
            double f = front_obstacle();

            cout << "[PROBE_RIGHT_CENTER_FRONT] "
                << "ds0="  << (_ds[0]  ? _ds[0]->getValue()  : -1)
                << " ds1=" << (_ds[1]  ? _ds[1]->getValue()  : -1)
                << " ds14=" << (_ds[14] ? _ds[14]->getValue() : -1)
                << " ds15=" << (_ds[15] ? _ds[15]->getValue() : -1)
                << " front_max=" << f
                << endl;

            double travelled = sqrt((_x - _id_probe_start_x) * (_x - _id_probe_start_x)
                                + (_y - _id_probe_start_y) * (_y - _id_probe_start_y));

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS || f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);
                _id_right_center_hit = true;
                _id_right_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;
                cout << "[Phase2.4c-probe] Right-center: obstacle detected ahead. "
                    << "distance=" << travelled
                    << " front=" << f << endl;
                _state = ID_BACKUP_RIGHT_CENTER;
            }
            else if (travelled > PROBE_FORWARD_MAX_M || _id_probe_steps > PROBE_FORWARD_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_right_center_hit = false;
                _id_right_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;
                cout << "[Phase2.4c-probe] Right-center: NO obstacle in "
                    << travelled << " m." << endl;
                _state = ID_BACKUP_RIGHT_CENTER;
            }
            else {
                set_speed(WALL_SPEED_SLOW, WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.4f-backup: reverse the SAME distance we just travelled
        case ID_BACKUP_RIGHT_CENTER:
        {
            double dx = _x - _id_probe_end_x;
            double dy = _y - _id_probe_end_y;
            double backed = sqrt(dx*dx + dy*dy);
            double target = _id_right_center_front;

            if (backed >= target - RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.4c-backup] Restored pre-probe position (backed "
                     << backed << " m of " << target << " m)." << endl;
                _state = ID_TURN_RIGHT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.4g: turn back to right scan heading before reversing
        case ID_TURN_RIGHT_TO_SCAN:
        {
            double target = _id_scan_heading_target;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_settle_steps = 0;
                cout << "[Phase2.4d] Facing right scan heading again (θ=" << _theta << ")" << endl;
                _state = ID_RETURN_FROM_RIGHT;
            }
            break;
        }

        // 2.5: reverse until odometry shows we're back on the yellow line
        case ID_RETURN_FROM_RIGHT:
        {
            double dx = _x - _id_line_anchor_x;
            double dy = _y - _id_line_anchor_y;
            double off_axis = sqrt(dx*dx + dy*dy);

            if (off_axis < RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.5] Back on yellow line (off=" << off_axis << " m)." << endl;
                _state = ID_TURN_LEFT;
            } else {
                double heading_err = normalize_angle(_id_scan_heading_target - _theta);
                double corr = ID_HEADING_KP * heading_err;
                set_speed(-ID_FORWARD_SPEED - corr, -ID_FORWARD_SPEED + corr);
            }
            break;
        }

        // 2.6: turn 180° to face the left side (forward + π/2)
        case ID_TURN_LEFT:
        {
            double target = normalize_angle(_id_forward_heading + M_PI / 2.0);
            double err = normalize_angle(target - _theta);
            cout << "[DEBUG_TURN_L] forward_heading=" << _id_forward_heading
                 << " target=" << target << " current_θ=" << _theta
                 << " err=" << err << " |err|=" << fabs(err) << endl;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.6] Facing left (θ=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_LEFT;
            }
            break;
        }

        // 2.6b: settle after left turn
        case ID_SETTLE_LEFT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            cout << "[SETTLE_LEFT] target=" << _id_scan_heading_target << " theta=" << _theta
                 << " err=" << err << " steps=" << _id_settle_steps << "/" << ID_ROT_SETTLE_STEPS << endl;
            if (fabs(err) > ANGLE_TOL) {
                cout << "  --> returning to TURN_LEFT (error too large)" << endl;
                _state = ID_TURN_LEFT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                cout << "  --> moving to DRIVE_TO_LEFT_WALL" << endl;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_LEFT_WALL;
            }
            break;
        }

        // 2.7: drive forward until front sensor sees left wall, or timeout
        case ID_DRIVE_TO_LEFT_WALL:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            double side = left_obstacle();
            double left_scan_side_peak = 0.0;
            if (_ds[13]) {
                left_scan_side_peak = max(left_scan_side_peak, _ds[13]->getValue());
            }
            left_scan_side_peak = max(left_scan_side_peak, right_obstacle());
            _id_left_scan_right_peak = max(_id_left_scan_right_peak, left_scan_side_peak);

            if (_id_left_scan_right_peak > ID_SIDE_OBS_CUBE_THRESH && f < OBSTACLE_FRONT_THRESH) {
                _id_cube_on_left = true;
                _id_left_object = OBJ_CUBE;
            }

            if (_id_wall_steps % 20 == 0) {
                cout << "[DS_LEFT]";
                for (int i = 0; i < NUM_DS; ++i) {
                    cout << " ds" << i << "=" << (_ds[i] ? _ds[i]->getValue() : -1);
                }
                cout << endl;
            }

            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x)
                                + (_y - _id_scan_start_y) * (_y - _id_scan_start_y));
            double heading_err = normalize_angle(_id_scan_heading_target - _theta);

            if (fabs(heading_err) > ID_HEADING_DRIFT_ABORT) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.7] Left scan drift (err=" << heading_err
                    << "). Re-aligning." << endl;
                _state = ID_TURN_LEFT;
                break;
            }

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                set_speed(0.0, 0.0);
                _id_left_wall_dist  = travelled;
                _id_left_wall_found = true;
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_WALL;
                }
                cout << "[Phase2.7] Left scan: WALL at " << travelled << " m"
                    << "  front=" << f << "  side=" << side
                    << "  peak_right=" << _id_left_scan_right_peak
                    << "  object=" << object_name(_id_left_object)
                    << "  steps=" << _id_wall_steps << endl;
                _id_settle_steps = 0;
                _state = ID_TURN_LEFT_TO_CENTER;
            }
            else if (f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);
                _id_left_wall_dist  = travelled;
                _id_left_wall_found = true;
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_WALL;
                }
                cout << "[Phase2.7] Left scan: FRONT wall stop (travelled " << travelled
                    << " m, front=" << f << ", side=" << side
                    << ", peak_right=" << _id_left_scan_right_peak
                    << ", object=" << object_name(_id_left_object) << ")" << endl;
                _id_settle_steps = 0;
                _state = ID_TURN_LEFT_TO_CENTER;
            }
            else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_left_wall_dist  = travelled;
                _id_left_wall_found = false;
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_NOTHING;
                }
                cout << "[Phase2.7] Left scan: NO WALL (travelled " << travelled
                    << " m, timeout)  peak_right=" << _id_left_scan_right_peak
                    << "  object=" << object_name(_id_left_object) << endl;
                _state = ID_RETURN_FROM_LEFT;
            }
            else {
                double base = (side > OBSTACLE_SIDE_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
                double corr = ID_HEADING_KP * heading_err;
                set_speed(base - corr, base + corr);
            }
            break;
        }

        // 2.7b: turn back toward room center after left wall hit
        case ID_TURN_LEFT_TO_CENTER:
        {
            double target = _id_forward_heading;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_settle_steps = 0;
                cout << "[Phase2.7b] Facing center from left wall (θ=" << _theta << ")" << endl;
                _state = ID_MEASURE_LEFT_CENTER;
            }
            break;
        }

        // 2.7c: kick off a forward probe toward room center (left side)
        case ID_MEASURE_LEFT_CENTER:
        {
            set_speed(0.0, 0.0);
            if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                _id_probe_start_x = _x;
                _id_probe_start_y = _y;
                _id_probe_steps   = 0;
                _id_front_hit_count = 0;
                _id_left_center_hit  = false;
                _id_left_center_front = 0.0;
                cout << "[Phase2.7c] Starting forward probe (left side, max "
                     << PROBE_FORWARD_MAX_M << " m)." << endl;
                _state = ID_PROBE_LEFT_CENTER;
            }
            break;
        }

        // 2.7d-probe: drive forward up to 2 m, check if there is an obstacle
        case ID_PROBE_LEFT_CENTER:
        {
            _id_probe_steps++;
            double f = front_obstacle();
            double travelled = sqrt((_x - _id_probe_start_x) * (_x - _id_probe_start_x)
                                  + (_y - _id_probe_start_y) * (_y - _id_probe_start_y));

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS || f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);
                _id_left_center_hit  = true;
                _id_left_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;
                if (travelled < 0.10) {
                    _id_middle_object = OBJ_CUBE;
                    cout << "[Phase2.7c-probe] Left-center: immediate HIT -> treated as CUBE "
                        << "(dist=" << travelled << " m, front=" << f << ")" << endl;
                } else {
                    cout << "[Phase2.7c-probe] Left-center: HIT at "
                        << travelled << " m (front=" << f << ")" << endl;
                }
                _state = ID_BACKUP_LEFT_CENTER;
            } else if (travelled > PROBE_FORWARD_MAX_M || _id_probe_steps > PROBE_FORWARD_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_left_center_hit  = false;
                _id_left_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;
                cout << "[Phase2.7c-probe] Left-center: NO obstacle in "
                     << travelled << " m." << endl;
                _state = ID_BACKUP_LEFT_CENTER;
            } else {
                set_speed(WALL_SPEED_SLOW, WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.7e-backup: reverse the SAME distance we just travelled
        case ID_BACKUP_LEFT_CENTER:
        {
            double dx = _x - _id_probe_end_x;
            double dy = _y - _id_probe_end_y;
            double backed = sqrt(dx*dx + dy*dy);
            double target = _id_left_center_front;

            if (backed >= target - RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.7c-backup] Restored pre-probe position (backed "
                     << backed << " m of " << target << " m)." << endl;
                _state = ID_TURN_LEFT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.7f: turn back to left scan heading before reversing
        case ID_TURN_LEFT_TO_SCAN:
        {
            double target = _id_scan_heading_target;
            if (turn_to_heading(target, ID_ROT_SPEED)) {
                _id_settle_steps = 0;
                cout << "[Phase2.7d] Facing left scan heading again (θ=" << _theta << ")" << endl;
                _state = ID_RETURN_FROM_LEFT;
            }
            break;
        }

        // 2.8: reverse back to yellow line
        case ID_RETURN_FROM_LEFT:
        {
            double off_line = fabs(_y - _id_line_anchor_y);

            if (off_line < RETURN_TO_LINE_TOL_M || _y <= _id_line_anchor_y) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.8] Back from left: back on yellow line"
                    << " y=" << _y
                    << " anchor_y=" << _id_line_anchor_y
                    << " off_line=" << off_line << endl;
                _state = ID_FACE_FORWARD_AGAIN;
            } else {
                double heading_err = normalize_angle(_id_scan_heading_target - _theta);
                double corr = ID_HEADING_KP * heading_err;
                set_speed(-ID_FORWARD_SPEED - corr, -ID_FORWARD_SPEED + corr);
            }
            break;
        }

        // 2.9: rotate back to forward heading
        case ID_FACE_FORWARD_AGAIN:
        {
            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED)) {
                cout << "[Phase2.9] Facing forward again (θ=" << _theta << ")." << endl;
                bool sides_clear = (!_id_right_wall_found && !_id_left_wall_found);
                bool both_hit_asymmetric = (_id_right_wall_found && _id_left_wall_found &&
                                            fabs(_id_left_wall_dist - _id_right_wall_dist) > 1.0);
                bool dark_scene = (_id_brightness < MEDIUM_MIN);

                if (sides_clear || (both_hit_asymmetric && dark_scene)) {
                    _id_scan_start_x = _x;
                    _id_scan_start_y = _y;
                    _id_wall_steps   = 0;
                    _id_cube_ahead   = false;
                    cout << "[Phase2.9] Probing front for cube (disambiguation)." << endl;
                    _state = ID_CHECK_CUBE_AHEAD;
                } else {
                    _state = ID_CLASSIFY;
                }
            }
            break;
        }

        // 2.10: drive forward briefly to detect the world-6 cube
        case ID_CHECK_CUBE_AHEAD:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x)
                                  + (_y - _id_scan_start_y) * (_y - _id_scan_start_y));

            if (f > CUBE_DETECT_THRESH && travelled <= CUBE_MAX_DIST_M) {
                set_speed(0.0, 0.0);
                _id_cube_ahead = true;
                _id_middle_object = OBJ_CUBE;
                cout << "[Phase2.10] CUBE detected ahead at " << travelled
                     << " m (sensor=" << f << ")." << endl;
                _state = ID_CLASSIFY;
            } else if (_id_wall_steps > CUBE_CHECK_STEPS) {
                set_speed(0.0, 0.0);
                _id_cube_ahead = false;
                _id_middle_object = OBJ_NOTHING;
                cout << "[Phase2.10] No cube ahead within " << travelled
                     << " m." << endl;
                _state = ID_CLASSIFY;
            } else {
                set_speed(WALL_SPEED, WALL_SPEED);
            }
            break;
        }

        // 2.11: classify based on saved measurements
        case ID_CLASSIFY:
        {
            bool sidePeakSig = (_id_right_scan_left_peak > 250.0 || _id_left_scan_right_peak > 250.0);
            bool cube_detected =
                _id_cube_ahead ||
                (_id_middle_object == OBJ_CUBE) ||
                sidePeakSig;

            if (cube_detected) {
                _id_cube_ahead = true;
                _id_middle_object = OBJ_CUBE;
            }

            set_speed(0.0, 0.0);

            cout << "════════════════════════════════════════════════════════" << endl;
            cout << "[Phase2.11] WORLD IDENTIFICATION SUMMARY" << endl;
            cout << "  brightness     = " << _id_brightness          << endl;
            cout << "  R/G/B          = " << _id_avg_r << " / " << _id_avg_g << " / " << _id_avg_b << endl;
            cout << "  right wall     = " << (_id_right_wall_found ? "YES" : "NO ")
                << "  (dist=" << _id_right_wall_dist << " m)" << endl;
            cout << "  left  wall     = " << (_id_left_wall_found  ? "YES" : "NO ")
                << "  (dist=" << _id_left_wall_dist  << " m)" << endl;
            cout << "  R-center probe = " << (_id_right_center_hit ? "HIT " : "open")
                << "  (forward dist=" << _id_right_center_front << " m)" << endl;
            cout << "  L-center probe = " << (_id_left_center_hit  ? "HIT " : "open")
                << "  (forward dist=" << _id_left_center_front  << " m)" << endl;
            cout << "  cube detected  = " << (cube_detected ? "YES" : "NO") << endl;
            cout << "  cube ahead raw = " << (_id_cube_ahead ? "YES" : "NO") << endl;
            cout << "  side peaks(Rscan-left / Lscan-right)= "
                << _id_right_scan_left_peak
                << " / " << _id_left_scan_right_peak << endl;
            cout << "════════════════════════════════════════════════════════" << endl;

            _world_id = classify_world_full();
            cout << "[Phase2.11] >>> Identified as WORLD " << _world_id << " <<<" << endl;

            load_path_for_world(_world_id);
            _current_wp = 0;
            _state = INITIAL_TURN;
            break;
        }

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
            break;
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
        is_foggy) {
        cout << "[Classify] Direct light/color signature => WORLD 5" << endl;
        return 5;
    }   

    if (!obstacle_from_right && obstacle_from_left) {
        cout << "[Classify] Signature: right center OPEN, left center HIT." << endl;
        if (_id_brightness >= 160) {
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
            if (_id_brightness >= 160) {
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
    double gx = vals[2];
    double gy = vals[0];

    double dx = gx - _x;
    double dy = gy - _y;
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
    double err = normalize_angle(target - _theta);
    if (fabs(err) < ANGLE_TOL) {
        set_speed(0.0, 0.0);
        cout << "[turn_to_heading DEBUG] REACHED target=" << target << " theta=" << _theta << " err=" << err << endl;
        return true;
    }

    double speed = ROT_KP * fabs(err);
    if (speed < ROT_MIN_SPEED) speed = ROT_MIN_SPEED;
    if (speed > max_speed) speed = max_speed;
    if (err < 0.0) speed = -speed;

    cout << "[turn_to_heading DEBUG] TURNING target=" << target << " theta=" << _theta 
         << " err=" << err << " speed=" << speed << endl;
    set_speed(-speed, speed);
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
        return 0.0;
    }

    sort(values.begin(), values.end());
    return values[values.size() / 2];
}

//////////////////////////////////////////////
// Debug helpers
//////////////////////////////////////////////

const char* MyRobot::state_name()
{
    switch (_state) {
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
    double ratio, center_x;
    if (_victims_found < 2 && green_detected(ratio, center_x)) {
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
            set_speed(SPEED_PURSUE, SPEED_PURSUE * 0.3);
        }
    } else {
        _stuck_ticks = 0;
        
        if (fabs(angle_error) > 0.3) {
            turn_to_heading(target_angle);
        } else {
            double turn_adjust = angle_error * 2.0;
            set_speed(SPEED_PURSUE - turn_adjust, SPEED_PURSUE + turn_adjust);
        }
    }
    
    if (dist < _last_dist_to_wp) {
        _last_dist_to_wp = dist;
    }
}

void MyRobot::step_spin_victim()
{
    _spin_ticks++;
    
    if (_spin_ticks >= 30) {
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