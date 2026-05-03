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
    _state = ID_WORLDS;
    _victims_found = 0;
    _current_wp = 0;
    _stuck_ticks = 0;
    _spin_ticks = 0;
    _world_id = 1;  // Default to world 1 for now
    
    _state_id = ID_FACE_FORWARD;  // Initialize world ID state machine
    _id_initialized = false;
    
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
            case ID_WORLDS:
                id_worlds();
                break;
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

void MyRobot::id_worlds()
{
    switch (_state_id)
        {
        case ID_FACE_FORWARD:
        {
            if (!_id_initialized) {
                _id_forward_heading = ALIGN_HEADING;
                _id_cube_ahead = false;  // reset phase-2 cube probe result
                if (_my_gps) {
                    _id_initial_gps_x = _my_gps->getValues()[2];
                    _id_initial_gps_y = _my_gps->getValues()[0];
                }
                _id_initialized = true;
                cout << "[Phase2.1] Init. GPS=("
                     << _id_initial_gps_x << ", " << _id_initial_gps_y
                     << ")  current θ=" << _theta
                     << "  target θ=" << _id_forward_heading << endl;
            }

            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED+2.0)) {
                cout << "[Phase2.1] Facing forward (θ=" << _theta << ")." << endl;
                cout << "[Phase2.1] Front sensor at start = "
                     << front_obstacle() << "  (info only, world 5 ignored)" << endl;
                _id_line_anchor_x = _x;
                _id_line_anchor_y = _y;
_state_id = ID_MEASURE_LIGHT;
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
            _id_cube_on_left = false;
            _id_left_object = OBJ_NOTHING;
            _id_right_object = OBJ_NOTHING;
            _id_middle_object = OBJ_NOTHING;
_state_id = ID_TURN_RIGHT;
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
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.3] Facing right (θ=" << _theta << "), settling." << endl;
_state_id = ID_SETTLE_RIGHT;
            }
            break;
        }

        // 2.4a: settle after right turn to avoid recording while inertia still rotates
        case ID_SETTLE_RIGHT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            cout << "[SETTLE_RIGHT] target=" << _id_scan_heading_target << " theta=" << _theta
                 << " err=" << err << " steps=" << _id_settle_steps << "/" << ID_ROT_SETTLE_STEPS << endl;
            if (fabs(err) > ANGLE_TOL) {
                cout << "  --> returning to TURN_RIGHT (error too large)" << endl;
_state_id = ID_TURN_RIGHT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                cout << "  --> moving to DRIVE_TO_RIGHT_WALL" << endl;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
_state_id = ID_DRIVE_TO_RIGHT_WALL;
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
_state_id = ID_TURN_RIGHT;
                break;
            }

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                // Wall (or some obstacle) detected.
                set_speed(0.0, 0.0);

                _id_right_wall_dist  = travelled;
                _id_right_wall_found = true;
                _id_right_object = OBJ_WALL;
                _id_settle_steps = 0;
_state_id = ID_TURN_RIGHT_TO_CENTER;
              } else if (f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                 _id_right_wall_found = true;
                 cout << "[Phase2.4] Right scan: FRONT wall stop (travelled " << travelled
                     << " m, front=" << f << ", side=" << side << ")" << endl;
                _id_settle_steps = 0;
_state_id = ID_TURN_RIGHT_TO_CENTER;
            } else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                // Timed out without detecting anything → no wall on this side.
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = false;
                _id_right_object = OBJ_NOTHING;
                cout << "[Phase2.4] Right scan: NO WALL (travelled " << travelled
                     << " m, timeout)" << endl;
_state_id = ID_RETURN_FROM_RIGHT;
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
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                cout << "[Phase2.4b] Facing center from right wall (θ=" << _theta << ")" << endl;
_state_id = ID_MEASURE_RIGHT_CENTER;
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
_state_id = ID_PROBE_RIGHT_CENTER;
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

                if (travelled < 0.10 && f > 300) {
                    cout << "[Phase2.4c-probe] Right-center: CUBE / obstacle tres proche detecte. "
                        << "distance=" << travelled
                        << " front=" << f << endl;

                    _id_front_object_type = 2;
                }
                else {
                    cout << "[Phase2.4c-probe] Right-center: WALL detected ahead. "
                        << "distance=" << travelled
                        << " front=" << f << endl;

                    _id_front_object_type = 1;
                }

_state_id = ID_BACKUP_RIGHT_CENTER;
            }
            else if (travelled > PROBE_FORWARD_MAX_M || _id_probe_steps > PROBE_FORWARD_TIMEOUT) {
                set_speed(0.0, 0.0);

                _id_right_center_hit = false;
                _id_right_center_front = travelled;
                _id_front_object_type = 0;

                _id_probe_end_x = _x;
                _id_probe_end_y = _y;

                cout << "[Phase2.4c-probe] Right-center: NO obstacle in "
                    << travelled << " m." << endl;

_state_id = ID_BACKUP_RIGHT_CENTER;
            }
            else {
                set_speed(WALL_SPEED_SLOW, WALL_SPEED_SLOW);
            }

            break;
        }

        // 2.4f-backup: reverse the SAME distance we just travelled during the probe, to return to the original position before the probe
        case ID_BACKUP_RIGHT_CENTER:
        {
            double dx = _x - _id_probe_end_x;
            double dy = _y - _id_probe_end_y;
            double backed = sqrt(dx*dx + dy*dy);
            double target = _id_right_center_front;  // forward probe distance

            if (backed >= target - RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.4c-backup] Restored pre-probe position (backed "
                     << backed << " m of " << target << " m)." << endl;
_state_id = ID_TURN_RIGHT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.4g: turn back to right scan heading before reversing
        case ID_TURN_RIGHT_TO_SCAN:
        {
            double target = _id_scan_heading_target; // right-facing heading
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                cout << "[Phase2.4d] Facing right scan heading again (θ=" << _theta << ")" << endl;
_state_id = ID_RETURN_FROM_RIGHT;
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
_state_id = ID_TURN_LEFT;
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
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.6] Facing left (θ=" << _theta << "), settling." << endl;
_state_id = ID_SETTLE_LEFT;
            }
            break;
        }

        // 2.6b: settle after left turn to avoid recording while inertia still rotates
        case ID_SETTLE_LEFT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            cout << "[SETTLE_LEFT] target=" << _id_scan_heading_target << " theta=" << _theta
                 << " err=" << err << " steps=" << _id_settle_steps << "/" << ID_ROT_SETTLE_STEPS << endl;
            if (fabs(err) > ANGLE_TOL) {
                cout << "  --> returning to TURN_LEFT (error too large)" << endl;
_state_id = ID_TURN_LEFT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                cout << "  --> moving to DRIVE_TO_LEFT_WALL" << endl;
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
_state_id = ID_DRIVE_TO_LEFT_WALL;
            }
            break;
        }

        // 2.7: drive forward until front sensor sees left wall, or timeout
        case ID_DRIVE_TO_LEFT_WALL:
        {
            _id_wall_steps++;

            double f = front_obstacle();
            double side = left_obstacle();

            // During the full drive toward the left wall, track the peak on the right side.
            // While facing left, right-side sensors can still see central/left-side objects.
            double left_scan_side_peak = 0.0;

            // In left scans, the cube can appear on ds13, so include it as well.
            if (_ds[13]) {
                left_scan_side_peak = max(left_scan_side_peak, _ds[13]->getValue());
            }

            // Keep the standard right-side aggregate too, for robustness across worlds.
            left_scan_side_peak = max(left_scan_side_peak, right_obstacle());

            // Keep the highest side peak over the full trajectory.
            _id_left_scan_right_peak = max(_id_left_scan_right_peak, left_scan_side_peak);

            // If side peak is strong while front still does not see a wall,
            // this is likely a side cube.
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
_state_id = ID_TURN_LEFT;
                break;
            }

            if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                // Wall detected.
                set_speed(0.0, 0.0);

                _id_left_wall_dist  = travelled;
                _id_left_wall_found = true;

                // If no cube was seen during the drive, the main left object is a wall.
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_WALL;
                }

                cout << "[Phase2.7] Left scan: WALL at " << travelled << " m"
                    << "  front=" << f
                    << "  side=" << side
                    << "  peak_right=" << _id_left_scan_right_peak
                    << "  object=" << object_name(_id_left_object)
                    << "  steps=" << _id_wall_steps << endl;

                _id_settle_steps = 0;
_state_id = ID_TURN_LEFT_TO_CENTER;
            }
            else if (f > OBSTACLE_FRONT_HARD_STOP) {
                set_speed(0.0, 0.0);

                _id_left_wall_dist  = travelled;
                _id_left_wall_found = true;

                // If no cube was seen during the drive, the main left object is a wall.
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_WALL;
                }

                cout << "[Phase2.7] Left scan: FRONT wall stop (travelled " << travelled
                    << " m, front=" << f
                    << ", side=" << side
                    << ", peak_right=" << _id_left_scan_right_peak
                    << ", object=" << object_name(_id_left_object)
                    << ")" << endl;

                _id_settle_steps = 0;
_state_id = ID_TURN_LEFT_TO_CENTER;
            }
            else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);

                _id_left_wall_dist  = travelled;
                _id_left_wall_found = false;

                // If no cube was seen and no wall was found, mark as empty.
                if (!_id_cube_on_left) {
                    _id_left_object = OBJ_NOTHING;
                }

                cout << "[Phase2.7] Left scan: NO WALL (travelled " << travelled
                    << " m, timeout)"
                    << "  peak_right=" << _id_left_scan_right_peak
                    << "  object=" << object_name(_id_left_object)
                    << endl;

_state_id = ID_RETURN_FROM_LEFT;
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
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                cout << "[Phase2.7b] Facing center from left wall (θ=" << _theta << ")" << endl;
_state_id = ID_MEASURE_LEFT_CENTER;
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
_state_id = ID_PROBE_LEFT_CENTER;
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

                // If we hit almost immediately after turning toward center,
                // this is likely the same central cube/object seen in left scan.
                if (travelled < 0.10) {
                    _id_middle_object = OBJ_CUBE;
                    cout << "[Phase2.7c-probe] Left-center: immediate HIT -> treated as CUBE "
                        << "(dist=" << travelled << " m, front=" << f << ")" << endl;
                } else {
                    cout << "[Phase2.7c-probe] Left-center: HIT at "
                        << travelled << " m (front=" << f << ")" << endl;
                }

_state_id = ID_BACKUP_LEFT_CENTER;
            } else if (travelled > PROBE_FORWARD_MAX_M || _id_probe_steps > PROBE_FORWARD_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_left_center_hit  = false;
                _id_left_center_front = travelled;
                _id_probe_end_x = _x;
                _id_probe_end_y = _y;
                cout << "[Phase2.7c-probe] Left-center: NO obstacle in "
                     << travelled << " m." << endl;
_state_id = ID_BACKUP_LEFT_CENTER;
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
_state_id = ID_TURN_LEFT_TO_SCAN;
            } else {
                set_speed(-WALL_SPEED_SLOW, -WALL_SPEED_SLOW);
            }
            break;
        }

        // 2.7f: turn back to left scan heading before reversing
        case ID_TURN_LEFT_TO_SCAN:
        {
            double target = _id_scan_heading_target; // left-facing heading
            if (turn_to_heading(target, ID_ROT_SPEED+2.0)) {
                _id_settle_steps = 0;
                cout << "[Phase2.7d] Facing left scan heading again (θ=" << _theta << ")" << endl;
_state_id = ID_RETURN_FROM_LEFT;
            }
            break;
        }

        // 2.8: reverse back to yellow line
        case ID_RETURN_FROM_LEFT:
        {
            // On return from the left wall, the main goal is to get back to yellow line.
            // In these worlds, yellow line is close to y ≈ 0.
            double off_line = fabs(_y - _id_line_anchor_y);

            // Safety: stop if we reached or crossed the yellow line.
            // Returning from left wall, y decreases toward 0, then can go negative.
            if (off_line < RETURN_TO_LINE_TOL_M || _y <= _id_line_anchor_y) {
                set_speed(0.0, 0.0);

                cout << "[Phase2.8] Back from left: back on yellow line"
                    << " y=" << _y
                    << " anchor_y=" << _id_line_anchor_y
                    << " off_line=" << off_line
                    << endl;

_state_id = ID_FACE_FORWARD_AGAIN;
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
            if (turn_to_heading(_id_forward_heading, ID_ROT_SPEED+2.0)) {
                cout << "[Phase2.9] Facing forward again (θ=" << _theta << ")." << endl;

                // Cube probe is needed in two cases:
                //  1) both sides clear (world 6 vs 7)
                //  2) both sides hit but strongly asymmetric distances in DARK scenes
                //     (world 2/3 candidates). In bright scenes this probe can hit
                //     a far wall and create false positives (world 1 -> world 2).
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
_state_id = ID_CHECK_CUBE_AHEAD;
                } else {
_state_id = ID_CLASSIFY;
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
_state_id = ID_CLASSIFY;
            } else if (_id_wall_steps > CUBE_CHECK_STEPS) {
                set_speed(0.0, 0.0);
                _id_cube_ahead = false;
                _id_middle_object = OBJ_NOTHING;
                cout << "[Phase2.10] No cube ahead within " << travelled
                     << " m." << endl;
_state_id = ID_CLASSIFY;
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
            cout << "  front blocked  = " << (_id_front_blocked_at_start ? "YES" : "NO") << endl;
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

        case IDENTIFY_WORLD:
        {
            set_speed(0.0, 0.0);
            _world_id = classify_world();
            load_path_for_world(_world_id);
            _current_wp = 0;
            _state = INITIAL_TURN;
            cout << "[Phase2] World " << _world_id
                 << " identified. " << _path.size()
                 << " waypoints loaded." << endl;
            break;
        }
        default:
            break;
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

double MyRobot::measure_camera_brightness()
{
    if (!_forward_camera) return 0.0;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return 0.0;

    int width = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total = width * height;
    long brightness = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = _forward_camera->imageGetRed(img, width, x, y);
            unsigned char g = _forward_camera->imageGetGreen(img, width, x, y);
            unsigned char b = _forward_camera->imageGetBlue(img, width, x, y);
            brightness += (r + g + b) / 3;
            _id_avg_r += r;
            _id_avg_g += g;
            _id_avg_b += b;
        }
    }

    _id_avg_r /= total;
    _id_avg_g /= total;
    _id_avg_b /= total;
    
    return (double)brightness / total;
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

bool MyRobot::turn_to_heading(double target, double speed)
{
    double current = get_heading_radians();
    double error = normalize_angle(target - current);
    
    if (fabs(error) < ANGLE_TOL) {
        set_speed(0, 0);
        return true;
    }
    
    double rot_speed = (speed < 0) ? SPEED_ROTATE : speed;
    
    if (error > 0) {
        set_speed(-rot_speed, rot_speed);
    } else {
        set_speed(rot_speed, -rot_speed);
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


const char* MyRobot::object_name(ObjectType obj)
{
    switch (obj) {
        case OBJ_WALL:    return "WALL";
        case OBJ_CUBE:    return "CUBE";
        case OBJ_NOTHING: return "NOTHING";
        default:          return "UNKNOWN";
    }
}


int MyRobot::classify_world()
{
    if (!_forward_camera) return 0;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return 0;

    int width  = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total  = width * height;
    long brightness = 0;

    unsigned char r, g, b;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            r = _forward_camera->imageGetRed(img,   width, x, y);
            g = _forward_camera->imageGetGreen(img, width, x, y);
            b = _forward_camera->imageGetBlue(img,  width, x, y);
            brightness += (r + g + b) / 3;
        }
    }

    double avg = (double)brightness / total;
    // Fog worlds are significantly darker; threshold may need tuning
    return (avg < 100) ? 1 : 0;
}


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

    if (_id_brightness >= 86 && _id_brightness < 88) {
        return 5;
    }
    if (!obstacle_from_right && obstacle_from_left) {
        cout << "[Classify] Signature: right center OPEN, left center HIT." << endl;
        if (_id_brightness <= 80){
            return 10;
        }
        else { return 4;}

    }

    // Inverse du cas précédent
    if (obstacle_from_right && !obstacle_from_left) {
        cout << "[Classify] Signature: right center HIT, left center OPEN." << endl;
        return 8;
    }

    // Les deux côtés vers le centre sont libres
    if (!obstacle_from_right && !obstacle_from_left) {
        cout << "[Classify] Signature: both center probes OPEN." << endl;

        if (_id_brightness <= 100){ 
            return 7;
        }

        return 6;
    }
    
    if (obstacle_from_right && obstacle_from_left) {
        if (_id_cube_on_left) {
            if (_id_brightness <= 100){ 
            return 2;
            }
            else{
                return 3; }
        } else {
            if (_id_brightness <= 60){ 
            return 9;   
            }
            else{
                return 1;}
        }
    }
    return -1;
}