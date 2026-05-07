/**
 * @file    MyRobot.cpp
 * @brief   5-phase rescue controller: Localize, identify, navigate, detect victims, return
 *
 * @author  Elizabeth Faulkner
 * @date    2026-04
 */

#include "MyRobot.h"

namespace {
constexpr bool DEBUG_LOG = true;  //debugging messs
static const bool DEBUG_LOG345 = false;    // phase 345
static const bool INFO_LOG = true;         // Important messages
}

MyRobot::MyRobot() : Robot()
{
    _time_step = 64;

    _left_speed = _right_speed = 0.0;
    _x = _y = _theta = 0.0;
    _prev_left_enc = _prev_right_enc = 0.0;
    _origin_x = _origin_y = 0.0;

    // start facing forward as a first state
    _state          = ID_FACE_FORWARD;
    _corner_turning = true;
    _victims_found  = 0;
    _spin_total     = 0.0;
    _state_after_spin = NAVIGATE_WAYPOINT;
    // Navigation and identification state.
    _ignore_detection_steps = 0;
    _world_id       = 0;
    _current_wp     = 0;
    _gps_timer      = 0;
    _avoid_steps    = 0;

    // Phase 2: world identification data.
    _id_brightness = 0.0;
    _id_avg_r = _id_avg_g = _id_avg_b = 0.0;

    _id_right_wall_dist = 0.0;
    _id_left_wall_dist = 0.0;
    _id_right_wall_found = false;
    _id_left_wall_found = false;

    _id_front_blocked_at_start = false;
    _id_cube_ahead = false;
    _id_front_object_type = OBJ_NOTHING;
    _id_right_object = OBJ_NOTHING;
    _id_left_object = OBJ_NOTHING;
    _id_middle_object = OBJ_NOTHING;

    _id_right_center_front = 0.0;
    _id_left_center_front = 0.0;
    _id_right_center_hit = false;
    _id_left_center_hit = false;

    _id_initial_gps_x = 0.0;
    _id_initial_gps_y = 0.0;
    _id_scan_start_x = 0.0;
    _id_scan_start_y = 0.0;
    _id_line_anchor_x = 0.0;
    _id_line_anchor_y = 0.0;
    _id_forward_heading = 0.0;
    _id_scan_heading_target = 0.0;

    _id_initialized = false;
    _id_wall_steps = 0;
    _id_settle_steps = 0;
    _id_front_hit_count = 0;
    _id_probe_steps = 0;

    _id_right_scan_left_peak = 0.0;
    _id_left_scan_right_peak = 0.0;

    _id_probe_start_x = 0.0;
    _id_probe_start_y = 0.0;
    _id_probe_end_x = 0.0;
    _id_probe_end_y = 0.0;

    _right_wall_inner_sensor = 0.0;
    _left_wall_inner_sensor = 0.0;
    _right_wall_sees_second_yellow_side = false;
    _left_wall_sees_second_yellow_side = false;

    _id_cube_on_right = false;
    _id_cube_on_left = false;

    // Sensors.
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    if (_left_wheel_sensor)
        _left_wheel_sensor->enable(_time_step);

    if (_right_wheel_sensor)
        _right_wheel_sensor->enable(_time_step);

    _my_compass = getCompass("compass");
    if (_my_compass)
        _my_compass->enable(_time_step);

    _my_gps = getGPS("gps");
    if (_my_gps)
        _my_gps->enable(_time_step);

    for (int i = 0; i < NUM_DS; i++) {
        string name = "ds" + to_string(i);
        _ds[i] = getDistanceSensor(name);

        if (_ds[i])
            _ds[i]->enable(_time_step);
    }
    // Motors.
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

    // Cameras.
    _forward_camera = getCamera("camera_f");
    if (_forward_camera)
        _forward_camera->enable(_time_step);

    _spherical_camera = getCamera("camera_s");
    if (_spherical_camera)
        _spherical_camera->enable(_time_step);
    }


// Robot destructorrrr
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

void MyRobot::run()
{
    const double FORWARD_SPEED  = 4.0;
    const double ROT_SPEED      = 2.5;
    const double WALL_SPEED     = 4.5;
    
    // Speeds used during world identification.
    const double ID_ROT_SPEED   = 5.0;    
    const double ID_FORWARD_SPEED = 6.0;  

    if (step(_time_step) == -1) return;

    // Seed odometry from GPS + compass on the very first tick.
    // GPS is noisy (3 m) but good enough for initial pose; compass gives accurate heading at least 
    if (_my_gps) {
        _x = (float)_my_gps->getValues()[2];   
        _y = (float)_my_gps->getValues()[0];   
    }
    _theta = normalize_angle(get_heading_radians());
    _origin_x = _x;
    _origin_y = _y;

    while (step(_time_step) != -1)
    {
        compute_odometry();
        _theta = normalize_angle(get_heading_radians());

        // GPS correction every ~5 s — low weight because resolution is 3 m so not worth a lot :(
        bool in_phase2 = (_state >= ID_FACE_FORWARD && _state <= ID_CLASSIFY);
        if (++_gps_timer >= 78) {
            if (!in_phase2) apply_gps_correction();
            _gps_timer = 0;
        }

        double front = front_obstacle();
        double right  = right_obstacle();

        switch (_state)
        {
        // Phase 1: world identification
        //1.1: rotate to face forward 
        case ID_FACE_FORWARD:
        {
            if (!_id_initialized) {
                _id_forward_heading = ALIGN_HEADING;
                _id_cube_ahead = false;
                if (_my_gps) {
                    _id_initial_gps_x = _my_gps->getValues()[2];
                    _id_initial_gps_y = _my_gps->getValues()[0];
                }
                _id_initialized = true;
            }

            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED+2.0)) {
                if (DEBUG_LOG) cout << "Phase1.1: facing forward (theta=" << _theta << ")." << endl;
                _id_line_anchor_x = _x;
                _id_line_anchor_y = _y;
                _state = ID_MEASURE_LIGHT;
            }
            break;
        }

        // 1.2: stationary camera brightness measurement
        case ID_MEASURE_LIGHT:
        {
            set_speed(0.0, 0.0);
            measure_camera_brightness();
            _id_wall_steps = 0;
            _id_right_scan_left_peak = 0.0;
            _id_left_scan_right_peak = 0.0;
            _id_cube_on_left = false;
            _id_left_object = OBJ_NOTHING;
            _id_right_object = OBJ_NOTHING;
            _id_middle_object = OBJ_NOTHING;
            _state = ID_TURN_RIGHT;
            break;
        }

        // 1.3: turn 90 deg right so target heading = forward - pie/2
        case ID_TURN_RIGHT:
        {
            double target = normalize_angle(_id_forward_heading - M_PI / 2.0);
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                if (DEBUG_LOG) cout << "Phase1.3: facing right (theta=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_RIGHT;
            }
            break;
        }

        // 1.4: Wait briefly after turning right.
        case ID_SETTLE_RIGHT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            if (fabs(err) > ANGLE_TOL) {
                _state = ID_TURN_RIGHT;
                break;
            }
            _id_settle_steps++;
            if (_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_RIGHT_WALL;
            }
            break;
        }

        // 1.5: Drive to the right wall and measure the distance.
        case ID_DRIVE_TO_RIGHT_WALL:
        {
            _id_wall_steps++;
            double front = front_obstacle();
            double side = right_obstacle();
            _id_right_scan_left_peak = max(_id_right_scan_left_peak, left_obstacle());
            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x) + (_y - _id_scan_start_y) * (_y - _id_scan_start_y));
            double heading_err = normalize_angle(_id_scan_heading_target - _theta);
            // Re-align if the robot drifted too much.
            if (fabs(heading_err) > ID_HEADING_DRIFT_ABORT) {
                set_speed(0.0, 0.0);
                _state = ID_TURN_RIGHT;
                break;
            }
            // Confirm front obstacle over several steps.
            if (front > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;
            bool wall_detected =(_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) ||(front > OBSTACLE_FRONT_HARD_STOP);
            if (wall_detected) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist = travelled;
                _id_right_wall_found = true;
                _id_right_object = OBJ_WALL;
                _id_settle_steps = 0;
                _state = ID_TURN_RIGHT_TO_CENTER;
                break;
            }
            // No wall found after timeout.
            if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist = travelled;
                _id_right_wall_found = false;
                _id_right_object = OBJ_NOTHING;

                _state = ID_RETURN_FROM_RIGHT;
                break;
            }
            // Keep driving straight.
            double base = (side > OBSTACLE_SIDE_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
            double corr = ID_HEADING_KP * heading_err;

            set_speed(base - corr, base + corr);
            break;
        }

        // 1.6a: turn back toward room center after right wall hit
        case ID_TURN_RIGHT_TO_CENTER:
        {
            double target = _id_forward_heading;
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                if (DEBUG_LOG) cout << "Phase1.6a facing center from right wall (theta=" << _theta << ")" << endl;
                _state = ID_MEASURE_RIGHT_CENTER;
            }
            break;
        }

        // 1.6b: kick off a forward probe toward room center (right side)
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
                if (DEBUG_LOG) cout << "Phase1.6b starting forward probe right "
                     << PROBE_FORWARD_MAX_M << " m)." << endl;
                _state = ID_PROBE_RIGHT_CENTER;
            }
            break;
        }

            
        /// Probe the right-center path.
        case ID_PROBE_RIGHT_CENTER:
        {
            _id_probe_steps++;

            double front = front_obstacle();
            double travelled = sqrt((_x - _id_probe_start_x) * (_x - _id_probe_start_x) +
                                    (_y - _id_probe_start_y) * (_y - _id_probe_start_y));

            if (front > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            bool obstacle_detected =
                (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) ||
                (front > OBSTACLE_FRONT_HARD_STOP);

            bool probe_finished =
                (travelled > PROBE_FORWARD_MAX_M) ||
                (_id_probe_steps > PROBE_FORWARD_TIMEOUT);

            if (obstacle_detected) {
                set_speed(0.0, 0.0);

                _id_right_center_hit = true;
                _id_right_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;

                bool very_close = (travelled < 0.10 && front > 300.0);
                _id_front_object_type = very_close ? OBJ_CUBE : OBJ_WALL;

                _state = ID_BACKUP_RIGHT_CENTER;
                break;
            }

            if (probe_finished) {
                set_speed(0.0, 0.0);

                _id_right_center_hit = false;
                _id_right_center_front = travelled;
                _id_front_object_type = OBJ_NOTHING;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;

                _state = ID_BACKUP_RIGHT_CENTER;
                break;
            }

            set_speed(WALL_SPEED_SLOW, WALL_SPEED_SLOW);
            break;
        }

        // Back up by the same distance used during the probe.
        case ID_BACKUP_RIGHT_CENTER:
        {
            double dx = _x - _id_probe_end_x;
            double dy = _y - _id_probe_end_y;
            double backed = sqrt(dx*dx + dy*dy);
            double target = _id_right_center_front;  // forward probe distance

            if (backed >= target - RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                if (DEBUG_LOG) cout << "Phase 1.6c going back to the previous pos, went back "
                     << backed << " m of " << target << " m)." << endl;
                _state = ID_TURN_RIGHT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }
            break;
        }

        // 1.6d: turn back to right scan heading before reversing
        case ID_TURN_RIGHT_TO_SCAN:
        {
            double target = _id_scan_heading_target; // right-facing heading
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                if (DEBUG_LOG) cout << "Phase2.4d facing right scan heading again (theta=" << _theta << ")" << endl;
                _state = ID_RETURN_FROM_RIGHT;
            }
            break;
        }

        // 1.7: reverse until odometry shows we're back on the yellow line
        case ID_RETURN_FROM_RIGHT:
        {
            double dx = _x - _id_line_anchor_x;
            double dy = _y - _id_line_anchor_y;
            double off_axis = sqrt(dx*dx + dy*dy);

            if (off_axis < RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                if (DEBUG_LOG) cout << "Phase1.7 Back on yellow line "<< endl;
                _state = ID_TURN_LEFT;
            } else {
                double heading_err = normalize_angle(_id_scan_heading_target - _theta);
                double corr = ID_HEADING_KP * heading_err;
                set_speed(-ID_FORWARD_SPEED - corr, -ID_FORWARD_SPEED + corr);
            }
            break;
        }

        // 1.8: turn 180 deg to face the left side so forward + pi/2
        case ID_TURN_LEFT:
        {
            double target = normalize_angle(_id_forward_heading + M_PI / 2.0);
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                if (DEBUG_LOG) cout << "Phase1.8 Facing left (theta=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_LEFT;
            }
            break;
        }

        // 1.8b: settle after left turn to avoid recording while still moving
        case ID_SETTLE_LEFT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            if (fabs(err) > ANGLE_TOL) {
                if (DEBUG_LOG) cout << " returning to TURN_LEFT error too big" << endl;
                _state = ID_TURN_LEFT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                if (DEBUG_LOG) cout << "  moving to DRIVE_TO_LEFT_WALL" << endl;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_LEFT_WALL;
            }
            break;
        }
        // 1.9: Drive to the left wall and measure the distance.
        case ID_DRIVE_TO_LEFT_WALL:
        {
            _id_wall_steps++;

            double front = front_obstacle();
            double side = left_obstacle();

            double side_peak = right_obstacle();

            if (_ds[13])
                side_peak = max(side_peak, _ds[13]->getValue());

            _id_left_scan_right_peak = max(_id_left_scan_right_peak, side_peak);

            if (_id_left_scan_right_peak > ID_SIDE_OBS_CUBE_THRESH &&
                front < OBSTACLE_FRONT_THRESH) {
                _id_cube_on_left = true;
                _id_left_object = OBJ_CUBE;
            }

            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x) +
                                    (_y - _id_scan_start_y) * (_y - _id_scan_start_y));

            double heading_err = normalize_angle(_id_scan_heading_target - _theta);

            if (fabs(heading_err) > ID_HEADING_DRIFT_ABORT) {
                set_speed(0.0, 0.0);
                _state = ID_TURN_LEFT;
                break;
            }

            if (front > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            bool wall_detected =
                (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) ||
                (front > OBSTACLE_FRONT_HARD_STOP);

            if (wall_detected) {
                set_speed(0.0, 0.0);

                _id_left_wall_dist = travelled;
                _id_left_wall_found = true;

                if (!_id_cube_on_left)
                    _id_left_object = OBJ_WALL;

                _id_settle_steps = 0;
                _state = ID_TURN_LEFT_TO_CENTER;
                break;
            }

            if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);

                _id_left_wall_dist = travelled;
                _id_left_wall_found = false;

                if (!_id_cube_on_left)
                    _id_left_object = OBJ_NOTHING;

                _state = ID_RETURN_FROM_LEFT;
                break;
            }

            double base = (side > OBSTACLE_SIDE_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
            double corr = ID_HEADING_KP * heading_err;

            set_speed(base - corr, base + corr);
            break;
        }

        // 1.9b: turn back toward room center after left wall hit
        case ID_TURN_LEFT_TO_CENTER:
        {
            double target = _id_forward_heading;
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                if (DEBUG_LOG) cout << "Phase1.9b Facing center from left wall (theta=" << _theta << ")" << endl;
                _state = ID_MEASURE_LEFT_CENTER;
            }
            break;
        }

        // 1.9c
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
                if (DEBUG_LOG) cout << "Phase2.7c Starting forward probe ";
                _state = ID_PROBE_LEFT_CENTER;
            }
            break;
        }

        // 1.9d: probe the left-center path.
        case ID_PROBE_LEFT_CENTER:
        {
            _id_probe_steps++;

            double front = front_obstacle();
            double travelled = sqrt((_x - _id_probe_start_x) * (_x - _id_probe_start_x) +
                                    (_y - _id_probe_start_y) * (_y - _id_probe_start_y));

            if (front > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            bool obstacle_detected =
                (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) ||
                (front > OBSTACLE_FRONT_HARD_STOP);

            bool probe_finished =
                (travelled > PROBE_FORWARD_MAX_M) ||
                (_id_probe_steps > PROBE_FORWARD_TIMEOUT);

            if (obstacle_detected) {
                set_speed(0.0, 0.0);

                _id_left_center_hit = true;
                _id_left_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;

                if (travelled < 0.10)
                    _id_middle_object = OBJ_CUBE;

                _state = ID_BACKUP_LEFT_CENTER;
                break;
            }

            if (probe_finished) {
                set_speed(0.0, 0.0);

                _id_left_center_hit = false;
                _id_left_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;

                _state = ID_BACKUP_LEFT_CENTER;
                break;
            }

            set_speed(WALL_SPEED_SLOW, WALL_SPEED_SLOW);
            break;
        }

        // 1.10: back up to the probe start position.
        case ID_BACKUP_LEFT_CENTER:
        {
            double dx = _x - _id_probe_end_x;
            double dy = _y - _id_probe_end_y;
            double backed = sqrt(dx * dx + dy * dy);
            double target = _id_left_center_front;

            if (backed >= target - RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                _state = ID_TURN_LEFT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }

            break;
        }

        // 1.11: face the left-scan heading again.
        case ID_TURN_LEFT_TO_SCAN:
        {
            if (turn_to_heading(_id_scan_heading_target, ID_ROT_SPEED + 2.0)) {
                _id_settle_steps = 0;
                _state = ID_RETURN_FROM_LEFT;
            }

            break;
        }

        // 1.12: return from the left scan to the yellow line.
        case ID_RETURN_FROM_LEFT:
        {
            double off_line = fabs(_y - _id_line_anchor_y);

            if (off_line <= RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                _state = ID_FACE_FORWARD_AGAIN;
                break;
            }

            double heading_err = normalize_angle(_id_scan_heading_target - _theta);
            double corr = ID_HEADING_KP * heading_err;

            set_speed(-ID_FORWARD_SPEED - corr, -ID_FORWARD_SPEED + corr);
            break;
        }

        // 1.13: face forward again.
        case ID_FACE_FORWARD_AGAIN:
        {
            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED + 2.0)) {
                bool sides_clear =
                    !_id_right_wall_found &&
                    !_id_left_wall_found;

                bool both_hit_asymmetric =
                    _id_right_wall_found &&
                    _id_left_wall_found &&
                    fabs(_id_left_wall_dist - _id_right_wall_dist) > 1.0;

                bool dark_scene = (_id_brightness < MEDIUM_MIN);

                if (sides_clear || (both_hit_asymmetric && dark_scene)) {
                    _id_scan_start_x = _x;
                    _id_scan_start_y = _y;
                    _id_wall_steps = 0;
                    _id_cube_ahead = false;

                    _state = ID_CHECK_CUBE_AHEAD;
                } else {
                    _state = ID_CLASSIFY;
                }
            }

            break;
        }

        // 1.14: check if a cube is directly ahead.
        case ID_CHECK_CUBE_AHEAD:
        {
            _id_wall_steps++;

            double front = front_obstacle();
            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x) +
                                    (_y - _id_scan_start_y) * (_y - _id_scan_start_y));

            if (front > CUBE_DETECT_THRESH && travelled <= CUBE_MAX_DIST_M) {
                set_speed(0.0, 0.0);

                _id_cube_ahead = true;
                _id_middle_object = OBJ_CUBE;

                _state = ID_CLASSIFY;
                break;
            }

            if (_id_wall_steps > CUBE_CHECK_STEPS) {
                set_speed(0.0, 0.0);

                _id_cube_ahead = false;
                _id_middle_object = OBJ_NOTHING;

                _state = ID_CLASSIFY;
                break;
            }

            set_speed(WALL_SPEED, WALL_SPEED);
            break;
        }

        // 1.15: classify the world.
        case ID_CLASSIFY:
        {
            bool side_peak_signature =
                (_id_right_scan_left_peak > 250.0) ||
                (_id_left_scan_right_peak > 250.0);

            bool cube_detected =
                _id_cube_ahead ||
                (_id_middle_object == OBJ_CUBE) ||
                side_peak_signature;

            if (cube_detected) {
                _id_cube_ahead = true;
                _id_middle_object = OBJ_CUBE;
            }

            set_speed(0.0, 0.0);

            if (DEBUG_LOG) {
                cout << "--------------------------------------------------" << endl
                    << "[Phase1.15] WORLD IDENTIFICATION SUMMARY" << endl
                    << "  brightness     = " << _id_brightness << endl
                    << "  R/G/B          = " << _id_avg_r << " / "
                                            << _id_avg_g << " / "
                                            << _id_avg_b << endl
                    << "  right wall     = " << (_id_right_wall_found ? "YES" : "NO")
                    << "  (dist=" << _id_right_wall_dist << " m)" << endl
                    << "  left wall      = " << (_id_left_wall_found ? "YES" : "NO")
                    << "  (dist=" << _id_left_wall_dist << " m)" << endl
                    << "  R-center probe = " << (_id_right_center_hit ? "HIT" : "open")
                    << "  (dist=" << _id_right_center_front << " m)" << endl
                    << "  L-center probe = " << (_id_left_center_hit ? "HIT" : "open")
                    << "  (dist=" << _id_left_center_front << " m)" << endl
                    << "  front blocked  = " << (_id_front_blocked_at_start ? "YES" : "NO") << endl
                    << "  cube detected  = " << (cube_detected ? "YES" : "NO") << endl
                    << "  side peaks     = " << _id_right_scan_left_peak
                    << " / " << _id_left_scan_right_peak << endl
                    << "---------------------------------------------" << endl;
            }

            _world_id = classify_world_full();

            if (INFO_LOG) {
                cout << "[World] Identified WORLD " << _world_id << endl;
            }

            load_waypoints(_world_id);
            _current_wp = 0;
            _state = NAVIGATE_WAYPOINT;
            break;
        }


        // Phase 2: navigate waypoints and detect victims
        case NAVIGATE_WAYPOINT:
        {
            if (_ignore_detection_steps > 0)
                _ignore_detection_steps--;

            if (_victims_found < 2 && _ignore_detection_steps == 0 && detect_green_victim()) {
                double ratio = 0.0;
                double center_x = 0.0;
                victim_position_in_image(ratio, center_x);

                if (ratio >= GREEN_CLOSE_THRESH) {
                    set_speed(0.0, 0.0);

                    _victims_found++;

                    if (INFO_LOG) {
                        cout << "[Victim] Victim " << _victims_found
                            << " found at x=" << _x
                            << ", y=" << _y << endl;
                    }

                    _ignore_detection_steps = 80;
                    start_spin(NAVIGATE_WAYPOINT);
                    break;
                }

                if (DEBUG_LOG) {
                    cout << "[Victim] Approaching target"
                        << " | ratio=" << ratio
                        << " | center_x=" << center_x
                        << endl;
                }

                if (center_x < -0.2) {
                    set_speed(1.0, 3.0);
                } else if (center_x > 0.2) {
                    set_speed(3.0, 1.0);
                } else {
                    set_speed(3.0, 3.0);
                }

                break;
            }

            if (_victims_found == 2) {
                if (_current_wp >= (int)_waypoints.size())
                    _current_wp = (int)_waypoints.size() - 1;

                _state = RETURN_TO_START;

                if (INFO_LOG)
                    cout << "[Navigation] Both victims found. Returning through waypoints." << endl;

                break;
            }

            if (_current_wp >= (int)_waypoints.size()) {
                _state = RETURN_TO_START;

                if (INFO_LOG)
                    cout << "[Navigation] All waypoints visited. Returning to start." << endl;

                break;
            }

            Waypoint& wp = _waypoints[_current_wp];
            double dist = dist_to(wp.x, wp.y);

            if (dist < WAYPOINT_DIST_TOL) {
                if (DEBUG_LOG) {
                    cout << "[Navigation] Waypoint " << _current_wp
                        << " reached." << endl;
                }

                _current_wp++;
                break;
            }

            double target_angle = atan2(wp.y - _y, wp.x - _x);
            double angle_err = normalize_angle(target_angle - _theta);

            if (front > OBSTACLE_FRONT_THRESH) {
                _avoid_steps++;

                if (_avoid_steps > 100) {
                    if (INFO_LOG) {
                        cout << "[Navigation] Waypoint " << _current_wp
                            << " blocked. Skipping." << endl;
                    }

                    _current_wp++;
                    _avoid_steps = 0;
                } else {
                    set_speed(WALL_SPEED, WALL_SPEED * 0.25);
                }

                break;
            }

            _avoid_steps = 0;

            if (fabs(angle_err) > ANGLE_TOL) {
                double spd = (angle_err > 0) ? ROT_SPEED : -ROT_SPEED;
                set_speed(-spd, spd);
            } else {
                double corr = 1.5 * angle_err;
                set_speed(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
            }

            break;
        }

        // Phase 3: spin after finding a victim 
        case SPIN_VICTIM:
        {
            _spin_total += 1.0;

            if (_spin_total >= 30.0) {
                set_speed(0.0, 0.0);

                _spin_total = 0.0;
                _state = _state_after_spin;

                if (INFO_LOG)
                    cout << "[Victim] Spin complete." << endl;

                break;
            }

            set_speed(-MAX_SPEED, MAX_SPEED);
            break;
        }

        // Phase 4: return through waypoints, then to departure line
        case RETURN_TO_START:
        {
            double target_x;
            double target_y;

            if (_current_wp >= 0 && _current_wp < (int)_waypoints.size()) {
                target_x = _waypoints[_current_wp].x;
                target_y = _waypoints[_current_wp].y;
            } else {
                target_x = _origin_x;
                target_y = _origin_y;
            }

            double dist = dist_to(target_x, target_y);

            if (dist < WAYPOINT_DIST_TOL) {
                set_speed(0.0, 0.0);

                if (_current_wp >= 0) {
                    _current_wp--;
                    break;
                }

                _state = TASK_COMPLETE;

                if (INFO_LOG)
                    cout << "[Return] Departure line reached. Task complete." << endl;

                break;
            }

            double target_angle = atan2(target_y - _y, target_x - _x);
            double angle_err = normalize_angle(target_angle - _theta);

            if (front > OBSTACLE_FRONT_THRESH) {
                set_speed(WALL_SPEED, WALL_SPEED * 0.25);
            } else if (fabs(angle_err) > ANGLE_TOL) {
                double spd = (angle_err > 0) ? ROT_SPEED : -ROT_SPEED;
                set_speed(-spd, spd);
            } else {
                double corr = 1.5 * angle_err;
                set_speed(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
            }

            break;
        }
        case TASK_COMPLETE:
        {
                set_speed(0.0, 0.0);
                break;
        }
    }
    }   
}


//////////////////////////////
// Victim detection

// Start a timed spin, then return to the given state.
void MyRobot::start_spin(State next_state)
{
    _state_after_spin = next_state;
    _spin_total = 0.0;
    _state = SPIN_VICTIM;
}

// Detect if enough green pixels are visible in the forward camera.
bool MyRobot::detect_green_victim()
{
    if (!_forward_camera)
        return false;

    const unsigned char* img = _forward_camera->getImage();
    if (!img)
        return false;

    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total = width * height;
    int green_count = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = _forward_camera->imageGetRed(img, width, x, y);
            unsigned char g = _forward_camera->imageGetGreen(img, width, x, y);
            unsigned char b = _forward_camera->imageGetBlue(img, width, x, y);

            if (g > GREEN_MIN_G &&
                g > (int)r + GREEN_DOMINANCE &&
                g > (int)b + GREEN_DOMINANCE) {
                green_count++;
            }
        }
    }

    double ratio = (double)green_count / total;
    return ratio >= GREEN_RATIO_THRESH;
}

// Compute how much green is visible and where it is in the image.
void MyRobot::victim_position_in_image(double& ratio, double& center_x)
{
    ratio = 0.0;
    center_x = 0.0;

    if (!_forward_camera)
        return;

    const unsigned char* img = _forward_camera->getImage();
    if (!img)
        return;

    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total = width * height;

    int green_count = 0;
    long sum_x = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = _forward_camera->imageGetRed(img, width, x, y);
            unsigned char g = _forward_camera->imageGetGreen(img, width, x, y);
            unsigned char b = _forward_camera->imageGetBlue(img, width, x, y);

            if (g > GREEN_MIN_G &&
                g > (int)r + GREEN_DOMINANCE &&
                g > (int)b + GREEN_DOMINANCE) {
                green_count++;
                sum_x += x;
            }
        }
    }

    ratio = (double)green_count / total;

    if (green_count > 0) {
        double avg_x = (double)sum_x / green_count;
        center_x = (avg_x / width) * 2.0 - 1.0;
    }
}

// Check if the green victim is close enough.
bool MyRobot::victim_is_close()
{
    double ratio = 0.0;
    double center_x = 0.0;

    victim_position_in_image(ratio, center_x);
    return ratio >= GREEN_CLOSE_THRESH;
}


/////////////////////////////////////////
// Camera and world identification

// Measure average image brightness and RGB values.
void MyRobot::measure_camera_brightness()
{
    _id_brightness = 0.0;
    _id_avg_r = 0.0;
    _id_avg_g = 0.0;
    _id_avg_b = 0.0;

    if (!_forward_camera)
        return;

    const unsigned char* img = _forward_camera->getImage();
    if (!img)
        return;

    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();

    long total = (long)width * (long)height;
    if (total <= 0)
        return;

    long sum_r = 0;
    long sum_g = 0;
    long sum_b = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            sum_r += _forward_camera->imageGetRed(img, width, x, y);
            sum_g += _forward_camera->imageGetGreen(img, width, x, y);
            sum_b += _forward_camera->imageGetBlue(img, width, x, y);
        }
    }

    _id_avg_r = (double)sum_r / total;
    _id_avg_g = (double)sum_g / total;
    _id_avg_b = (double)sum_b / total;

    _id_brightness = (_id_avg_r + _id_avg_g + _id_avg_b) / 3.0;
}

// Check if the front path is blocked.
bool MyRobot::front_blocked()
{
    return front_obstacle() > OBSTACLE_FRONT_THRESH;
}

// Classify the world using saved scan measurements.
int MyRobot::classify_world_full()
{
    bool left_wall = _id_left_wall_found;
    bool right_wall = _id_right_wall_found;

    bool right_center_hit = _id_right_center_hit;
    bool left_center_hit = _id_left_center_hit;

    double side_diff = fabs(_id_left_wall_dist - _id_right_wall_dist);

    bool side_cube_signature =
        (_id_right_scan_left_peak > ID_SIDE_OBS_CUBE_THRESH) ||
        (_id_left_scan_right_peak > ID_SIDE_OBS_CUBE_THRESH);

    bool cube_detected =
        _id_cube_ahead ||
        (_id_middle_object == OBJ_CUBE) ||
        side_cube_signature;

    double max_rgb = std::max(_id_avg_r, std::max(_id_avg_g, _id_avg_b));
    double min_rgb = std::min(_id_avg_r, std::min(_id_avg_g, _id_avg_b));
    double saturation = max_rgb - min_rgb;

    bool is_foggy =
        (_id_avg_b > 100.0 &&
         _id_avg_g > 100.0 &&
         _id_avg_r < 95.0 &&
         fabs(_id_avg_g - _id_avg_b) < 25.0);

    if (DEBUG_LOG) {
        cout << "[Classify]"
             << " R_center=" << (right_center_hit ? "HIT" : "OPEN")
             << " distR=" << _id_right_center_front
             << " | L_center=" << (left_center_hit ? "HIT" : "OPEN")
             << " distL=" << _id_left_center_front
             << " | RGB=" << _id_avg_r << "/" << _id_avg_g << "/" << _id_avg_b
             << " | sat=" << saturation
             << " | foggy=" << is_foggy
             << " | L=" << (left_wall ? "WALL" : "NO")
             << " | R=" << (right_wall ? "WALL" : "NO")
             << " | diff=" << side_diff
             << " | sideCube=" << (side_cube_signature ? "YES" : "NO")
             << " | cube=" << (cube_detected ? "YES" : "NO")
             << endl;
    }

    if (_id_brightness >= 86.0 && _id_brightness < 88.0)
        return 5;

    if (!right_center_hit && left_center_hit) {
        if (_id_brightness <= 80.0)
            return 10;

        return 4;
    }

    if (right_center_hit && !left_center_hit)
        return 8;

    if (!right_center_hit && !left_center_hit) {
        if (_id_brightness <= 100.0)
            return 7;

        return 6;
    }

    if (right_center_hit && left_center_hit) {
        if (_id_cube_on_left) {
            if (_id_brightness >=100.0)
                return 3;

            return 2;
        }

        if (_id_brightness <= 60.0)
            return 9;

        return 1;
    }

    return -1;
}

// Load the path followed after world identification.
void MyRobot::load_waypoints(int world_id)
{
    _waypoints.clear();

    switch (world_id) {
        case 1:
            _waypoints = {
                { -3.05,  -1.33   },
                {  1.879, -0.317  },
                {  5.231,  1.027  },
                {  8.82,   0.4215 },
                {  8.816, -2.018  }
            };
            break;

        case 2:
            _waypoints = {
                { -7.57,   3.3   },
                { -2.35,   1.619 },
                {  1.866,  4.17  },
                {  5.26,   0.487 },
                {  8.83,   0.48  },
                {  8.83,   3.35  }
            };
            break;

        case 3:
            _waypoints = {
                { -5.92,   0.875 },
                { -2.5,    1.369 },
                {  3.47,   2.79  },
                {  8.38,   2.78  },
                {  8.37,  -2.26  }
            };
            break;

        case 4:
            _waypoints = {
                { -3.05,  -1.33   },
                {  1.879, -0.317  },
                {  5.231,  1.027  },
                {  8.82,   0.4215 },
                {  8.816, -2.018  }
            };
            break;

        case 5:
            _waypoints = {
                { -2.86,  -1.809 },
                {  0.1,   -3.1   },
                {  8.66,  -3.19  },
                {  8.67,   3.18  }
            };
            break;

        case 6:
            _waypoints = {
                { -2.9,   -1.32  },
                {  0.215, -3.144 },
                {  3.175, -2.9   },
                {  9.065, -2.918 },
                {  9.07,   0.831 }
            };
            break;

        case 7:
            _waypoints = {
                {  2.05,   2.65  },
                {  7.93,   1.38  },
                {  8.49,  -3.377 }
            };
            break;

        case 8:
            _waypoints = {
                { -3.06,  -1.23  },
                { -3.065, -3.479 },
                {  0.925, -2.755 },
                {  8.49,  -2.76  },
                {  8.49,  -1.51  }
            };
            break;

        case 9:
            _waypoints = {
                { -3.05,  -1.33  },
                {  1.879, -0.317 },
                {  5.26,   0.717 },
                {  8.225,  3.25  },
                {  8.744, -3.668 }
            };
            break;

        case 10:
            _waypoints = {
                { -4.67,  -0.43  },
                { -4.24,   1.222 },
                { -2.62,  -0.18  },
                {  2.011,  1.07  },
                {  5.77,   2.52  },
                {  9.42,   0.71  }
            };
            break;

        default:
            cout << "[Waypoints] Unknown world " << world_id
                 << ". Using world 1 path." << endl;

            _waypoints = {
                { -3.05,  -1.33   },
                {  1.879, -0.317  },
                {  5.231,  1.027  },
                {  8.82,   0.4215 },
                {  8.816, -2.018  }
            };
            break;
    }

    if (INFO_LOG) {
        cout << "[Waypoints] World " << world_id
             << " path loaded: "
             << _waypoints.size()
             << " waypoints." << endl;
    }
}


////////////////////////////////////
// Odometry and movement


// Blend odometry with GPS when the GPS error is plausible.
void MyRobot::apply_gps_correction()
{
    if (!_my_gps)
        return;

    const double* vals = _my_gps->getValues();

    double gps_x = vals[2];
    double gps_y = vals[0];

    double dx = gps_x - _x;
    double dy = gps_y - _y;
    double error = sqrt(dx * dx + dy * dy);

    if (error < 4.0) {
        _x += 0.1f * (float)dx;
        _y += 0.1f * (float)dy;
    }
}

// Update local position using wheel encoders.
void MyRobot::compute_odometry()
{
    double left_enc = _left_wheel_sensor ? _left_wheel_sensor->getValue() : 0.0;
    double right_enc = _right_wheel_sensor ? _right_wheel_sensor->getValue() : 0.0;

    double dl = encoder_tics_to_meters((float)(left_enc - _prev_left_enc));
    double dr = encoder_tics_to_meters((float)(right_enc - _prev_right_enc));

    _prev_left_enc = (float)left_enc;
    _prev_right_enc = (float)right_enc;

    double ds = (dl + dr) / 2.0;
    double dtheta = (dr - dl) / WHEELS_DISTANCE;

    _x += (float)(ds * cos(_theta + dtheta / 2.0));
    _y += (float)(ds * sin(_theta + dtheta / 2.0));
    _theta = (float)normalize_angle(_theta + dtheta);
}

// Read heading from the compass.
double MyRobot::get_heading_radians()
{
    if (!_my_compass)
        return 0.0;

    const double* v = _my_compass->getValues();
    return atan2(-v[2], -v[0]);
}

// Convert wheel encoder radians to meters.
float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

// Rotate toward a target heading.
bool MyRobot::turn_to_heading(double target, double max_speed)
{
    double err = normalize_angle(target - _theta);

    if (fabs(err) < ANGLE_TOL) {
        set_speed(0.0, 0.0);
        return true;
    }

    double speed = ROT_KP * fabs(err);

    if (speed < ROT_MIN_SPEED)
        speed = ROT_MIN_SPEED;

    if (speed > max_speed)
        speed = max_speed;

    if (err < 0.0)
        speed = -speed;

    set_speed(-speed, speed);
    return false;
}

// Clamp and send wheel speeds to the motors.
void MyRobot::set_speed(double left, double right)
{
    if (left > MAX_SPEED)
        left = MAX_SPEED;
    if (left < -MAX_SPEED)
        left = -MAX_SPEED;

    if (right > MAX_SPEED)
        right = MAX_SPEED;
    if (right < -MAX_SPEED)
        right = -MAX_SPEED;

    _left_speed = left;
    _right_speed = right;

    if (_left_wheel_motor)
        _left_wheel_motor->setVelocity(_left_speed);

    if (_right_wheel_motor)
        _right_wheel_motor->setVelocity(_right_speed);
}


//////////////////////////////////
// Geometry and sensors

// Keep an angle inside [-pi, pi].
double MyRobot::normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

// Distance from robot position to a target point.
double MyRobot::dist_to(double tx, double ty)
{
    double dx = tx - _x;
    double dy = ty - _y;

    return sqrt(dx * dx + dy * dy);
}

// Check if the robot is back near its origin.
bool MyRobot::goal_reached()
{
    return dist_to(_origin_x, _origin_y) < WAYPOINT_DIST_TOL;
}

// Maximum value from front sensors.
double MyRobot::front_obstacle()
{
    double max_val = 0.0;
    int ids[] = {0, 1, 14, 15};

    for (int i = 0; i < 4; i++) {
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    }

    return max_val;
}

// Median value from left sensors.
double MyRobot::left_obstacle()
{
    int ids[] = {4, 5, 6};
    vector<double> values;
    values.reserve(3);

    for (int i = 0; i < 3; i++) {
        if (_ds[ids[i]])
            values.push_back(_ds[ids[i]]->getValue());
    }

    if (values.empty())
        return 0.0;

    sort(values.begin(), values.end());
    return values[values.size() / 2];
}

// Median value from right sensors.
double MyRobot::right_obstacle()
{
    int ids[] = {9, 10, 11};
    vector<double> values;
    values.reserve(3);

    for (int i = 0; i < 3; i++) {
        if (_ds[ids[i]])
            values.push_back(_ds[ids[i]]->getValue());
    }

    if (values.empty())
        return 0.0;

    sort(values.begin(), values.end());
    return values[values.size() / 2];
}

// Estimate free space in front of the robot.
double MyRobot::measure_forward_distance()
{
    double max_obstacle = 0.0;
    int ids[] = {0, 1, 14, 15};

    for (int i = 0; i < 4; i++) {
        if (_ds[ids[i]]) {
            double value = _ds[ids[i]]->getValue();

            if (value > max_obstacle)
                max_obstacle = value;
        }
    }

    return 1024.0 - max_obstacle;
}


////////////////////
// Debug and names

// Print odometry only when position debug is enabled.
void MyRobot::print_odometry()
{
    if (DEBUG_LOG345) {
        cout << "[Position]"
             << " x=" << _x
             << " | y=" << _y
             << " | theta=" << _theta
             << endl;
    }
}

// Convert detected object enum to text.
const char* MyRobot::object_name(DetectedObject obj)
{
    switch (obj) {
        case OBJ_WALL:
            return "WALL";

        case OBJ_CUBE:
            return "CUBE";

        case OBJ_NOTHING:
            return "NOTHING";

        default:
            return "UNKNOWN";
    }
}

