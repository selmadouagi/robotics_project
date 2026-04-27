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
    _sr = _sl = 0.0;
    _prev_left_enc = _prev_right_enc = 0.0;
    _origin_x = _origin_y = 0.0;

    // Localization states are implemented but skipped; GPS+compass init is more
    // reliable when interior obstacles block access to room corners.
    // Start directly in Phase 2 (world identification) sub-state machine.
    _state          = ID_FACE_FORWARD;
    _corner_turning = true;
    _victims_found  = 0;
    _orient_step          = 0;
    _orient_best_heading  = 0.0;
    _orient_best_distance = 0.0;
    _orient_start_theta   = 0.0;
    _orient_initialized   = false;
    // _spin_last_theta removed — spin now uses a step counter
    _spin_total     = 0.0;
    _state_after_spin = NAVIGATE_WAYPOINT;
    _ignore_detection_steps = 0;
    _world_id       = 0;
    _current_wp     = 0;
    _gps_timer      = 0;
    _avoid_steps    = 0;

    // Phase 2 — world identification fields
    _id_brightness        = 0.0;
    _id_avg_r = _id_avg_g = _id_avg_b = 0.0;
    _id_right_wall_dist   = 0.0;
    _id_left_wall_dist    = 0.0;
    _id_right_wall_found  = false;
    _id_left_wall_found   = false;
    _id_front_blocked_at_start = false;
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
    const double FORWARD_SPEED  = 4.0;
    const double ROT_SPEED      = 2.5;
    const double WALL_SPEED     = 4.5;

    if (step(_time_step) == -1) return;

    // Seed odometry from GPS + compass on the very first tick.
    // GPS is noisy (3 m) but good enough for initial pose; compass gives accurate heading.
    if (_my_gps) {
        _x = (float)_my_gps->getValues()[2];   // Webots Z → our x (long axis)
        _y = (float)_my_gps->getValues()[0];   // Webots X → our y (lateral)
    }
    _theta = normalize_angle(get_heading_radians());
    _origin_x = _x;
    _origin_y = _y;
    if (_left_wheel_sensor)  _prev_left_enc  = _left_wheel_sensor->getValue();
    if (_right_wheel_sensor) _prev_right_enc = _right_wheel_sensor->getValue();
    cout << "[Init] GPS seed: x=" << _x << " y=" << _y << " θ=" << _theta << endl;

    while (step(_time_step) != -1)
    {
        compute_odometry();
        _theta = normalize_angle(get_heading_radians());

        // GPS correction every ~5 s — low weight because resolution is 3 m
        if (++_gps_timer >= 78) {
            apply_gps_correction();
            _gps_timer = 0;
        }

        double front = front_obstacle();
        double right  = right_obstacle();

        switch (_state)
        {
        // ── Phase 0: tourne sur soi-même pour trouver le fond de la salle ──
        case ORIENT_INITIAL:
        {
            run_orient_initial();
            break;
        }

        // ── Phase 1a: drive until a front wall is detected ────────────────
        case LOCALIZE_FIND_WALL:
        {
            if (front > OBSTACLE_FRONT_THRESH) {
                set_speed(0.0, 0.0);
                _corner_turning = true;
                _state = LOCALIZE_FOLLOW_CORNER;
                cout << "[Phase1a] Wall found. Following to corner." << endl;
            } else {
                set_speed(FORWARD_SPEED, FORWARD_SPEED);
            }
            break;
        }

        // ── Phase 1b: follow right wall until a corner is reached ─────────
        case LOCALIZE_FOLLOW_CORNER:
        {
            if (_corner_turning) {
                // Turn left until front is clear AND right wall is in range
                if (front < OBSTACLE_FRONT_THRESH && right > OBSTACLE_SIDE_THRESH) {
                    _corner_turning = false;
                    cout << "[Phase1b] Right wall acquired. Following." << endl;
                } else {
                    set_speed(-ROT_SPEED, ROT_SPEED);
                }
            } else {
                if (front > OBSTACLE_FRONT_THRESH) {
                    // Both front and right walls present → corner
                    set_speed(0.0, 0.0);
                    _state = LOCALIZE_ALIGN;
                    cout << "[Phase1b] Corner reached. Aligning." << endl;
                } else {
                    // P-controller: maintain right wall at target reading
                    // Target reading ~70 keeps a safe side distance; tune if needed
                    double err = right - 70.0;
                    double kp  = 0.03;
                    set_speed(WALL_SPEED + kp * err, WALL_SPEED - kp * err);
                }
            }
            break;
        }

        // ── Phase 1c: rotate to known heading then reset odometry ─────────
        case LOCALIZE_ALIGN:
        {
            double err = normalize_angle(ALIGN_HEADING - _theta);
            if (fabs(err) < ANGLE_TOL) {
                set_speed(0.0, 0.0);
                _x = 0.0;  _y = 0.0;  _theta = ALIGN_HEADING;
                _origin_x = 0.0;  _origin_y = 0.0;
                if (_left_wheel_sensor)  _prev_left_enc  = _left_wheel_sensor->getValue();
                if (_right_wheel_sensor) _prev_right_enc = _right_wheel_sensor->getValue();
                _state = IDENTIFY_WORLD;
                cout << "[Phase1c] Aligned. Odometry reset to (0,0)." << endl;
            } else {
                double speed = (err > 0) ? ROT_SPEED : -ROT_SPEED;
                set_speed(-speed, speed);
            }
            break;
        }

        // ════════════════════════════════════════════════════════════════════
        // ── Phase 2: WORLD IDENTIFICATION (rotate-and-scan, on yellow line) ─
        // Plan:
        //   2.1  Use compass to face forward (toward the far yellow line).
        //        Check front sensor BEFORE moving — if blocked, this is WORLD 5
        //        (obstacle on the yellow line).
        //   2.2  Stationary camera measurement → brightness + R/G/B averages.
        //   2.3  Turn 90° right (heading = forward − π/2).
        //   2.4  Drive forward until front sensor saturates OR timeout.
        //        Distance travelled (via odometry) = right wall distance.
        //   2.5  Reverse along same heading until odometry shows we're back on
        //        the yellow line (within RETURN_TO_LINE_TOL_M).
        //   2.6  Turn left 180° (heading = forward + π/2).
        //   2.7  Drive forward → left wall distance via odometry.
        //   2.8  Reverse back to yellow line.
        //   2.9  Rotate to forward heading.
        //   2.10 If both sides reported "no wall" → drive forward briefly to
        //        check for the world-6 cube. Else skip this step.
        //   2.11 Classify (wall pattern + brightness + cube + start-front-blocked).
        // ════════════════════════════════════════════════════════════════════

        // ── 2.1: rotate to forward heading ──────────────────────────────────
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

            if (turn_to_heading(_id_forward_heading, ROT_SPEED)) {
                cout << "[Phase2.1] Facing forward (θ=" << _theta << ")." << endl;
                cout << "[Phase2.1] Front sensor at start = "
                     << front_obstacle() << "  (info only, world 5 ignored)" << endl;
                _id_line_anchor_x = _x;
                _id_line_anchor_y = _y;
                _state = ID_MEASURE_LIGHT;
            }
            break;
        }

        // ── 2.2: stationary camera brightness measurement ───────────────────
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
            _state = ID_TURN_RIGHT;
            break;
        }

        // ── 2.3: turn 90° right (target heading = forward - π/2) ────────────
        case ID_TURN_RIGHT:
        {
            double target = normalize_angle(_id_forward_heading - M_PI / 2.0);
            if (turn_to_heading(target, ROT_SPEED)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.3] Facing right (θ=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_RIGHT;
            }
            break;
        }

        // ── settle after right turn to avoid recording while inertia still rotates
        case ID_SETTLE_RIGHT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            if (fabs(err) > ANGLE_TOL) {
                _state = ID_TURN_RIGHT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_RIGHT_WALL;
            }
            break;
        }

        // ── 2.4: drive forward until front sensor sees wall, or timeout ─────
        case ID_DRIVE_TO_RIGHT_WALL:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            _id_right_scan_left_peak = max(_id_right_scan_left_peak, left_obstacle());
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

            if (f > OBSTACLE_FRONT_HARD_STOP)
                _id_front_hit_count = ID_FRONT_CONFIRM_STEPS;
            else if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                // Wall (or some obstacle) detected.
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = (travelled < WALL_FOUND_DIST_M)
                                       ? true : true;  // any hit = wall here
                cout << "[Phase2.4] Right scan: WALL at " << travelled << " m"
                     << "  sensor=" << f
                     << "  steps=" << _id_wall_steps << endl;
                _state = ID_RETURN_FROM_RIGHT;
            } else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                // Timed out without detecting anything → no wall on this side.
                set_speed(0.0, 0.0);
                _id_right_wall_dist  = travelled;
                _id_right_wall_found = false;
                cout << "[Phase2.4] Right scan: NO WALL (travelled " << travelled
                     << " m, timeout)" << endl;
                _state = ID_RETURN_FROM_RIGHT;
            } else {
                double base = (f > OBSTACLE_FRONT_SLOW_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
                double corr = ID_HEADING_KP * heading_err;
                set_speed(base - corr, base + corr);
            }
            break;
        }

        // ── 2.5: reverse until odometry shows we're back on the yellow line ─
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
                set_speed(-WALL_SPEED_SLOW - corr, -WALL_SPEED_SLOW + corr);
            }
            break;
        }

        // ── 2.6: turn 180° to face the left side (forward + π/2) ────────────
        case ID_TURN_LEFT:
        {
            double target = normalize_angle(_id_forward_heading + M_PI / 2.0);
            if (turn_to_heading(target, ROT_SPEED)) {
                _id_scan_heading_target = target;
                _id_settle_steps = 0;
                cout << "[Phase2.6] Facing left (θ=" << _theta << "), settling." << endl;
                _state = ID_SETTLE_LEFT;
            }
            break;
        }

        // ── settle after left turn to avoid recording while inertia still rotates
        case ID_SETTLE_LEFT:
        {
            set_speed(0.0, 0.0);
            double err = normalize_angle(_id_scan_heading_target - _theta);
            if (fabs(err) > ANGLE_TOL) {
                _state = ID_TURN_LEFT;
            } else if (++_id_settle_steps >= ID_ROT_SETTLE_STEPS) {
                _id_scan_start_x = _x;
                _id_scan_start_y = _y;
                _id_wall_steps   = 0;
                _id_front_hit_count = 0;
                _state = ID_DRIVE_TO_LEFT_WALL;
            }
            break;
        }

        // ── 2.7: drive forward until front sensor sees left wall, or timeout
        case ID_DRIVE_TO_LEFT_WALL:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            _id_left_scan_right_peak = max(_id_left_scan_right_peak, right_obstacle());
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

            if (f > OBSTACLE_FRONT_HARD_STOP)
                _id_front_hit_count = ID_FRONT_CONFIRM_STEPS;
            else if (f > OBSTACLE_FRONT_THRESH)
                _id_front_hit_count++;
            else if (_id_front_hit_count > 0)
                _id_front_hit_count--;

            if (_id_front_hit_count >= ID_FRONT_CONFIRM_STEPS) {
                set_speed(0.0, 0.0);
                _id_left_wall_dist  = travelled;
                _id_left_wall_found = true;
                cout << "[Phase2.7] Left scan: WALL at " << travelled << " m"
                     << "  sensor=" << f
                     << "  steps=" << _id_wall_steps << endl;
                _state = ID_RETURN_FROM_LEFT;
            } else if (_id_wall_steps > WALL_DRIVE_TIMEOUT) {
                set_speed(0.0, 0.0);
                _id_left_wall_dist  = travelled;
                _id_left_wall_found = false;
                cout << "[Phase2.7] Left scan: NO WALL (travelled " << travelled
                     << " m, timeout)" << endl;
                _state = ID_RETURN_FROM_LEFT;
            } else {
                double base = (f > OBSTACLE_FRONT_SLOW_THRESH) ? WALL_SPEED_SLOW : WALL_SPEED;
                double corr = ID_HEADING_KP * heading_err;
                set_speed(base - corr, base + corr);
            }
            break;
        }

        // ── 2.8: reverse back to yellow line ────────────────────────────────
        case ID_RETURN_FROM_LEFT:
        {
            double dx = _x - _id_line_anchor_x;
            double dy = _y - _id_line_anchor_y;
            double off_axis = sqrt(dx*dx + dy*dy);

            if (off_axis < RETURN_TO_LINE_TOL_M) {
                set_speed(0.0, 0.0);
                cout << "[Phase2.8] Back on yellow line (off=" << off_axis << " m)." << endl;
                _state = ID_FACE_FORWARD_AGAIN;
            } else {
                double heading_err = normalize_angle(_id_scan_heading_target - _theta);
                double corr = ID_HEADING_KP * heading_err;
                set_speed(-WALL_SPEED_SLOW - corr, -WALL_SPEED_SLOW + corr);
            }
            break;
        }

        // ── 2.9: rotate back to forward heading ─────────────────────────────
        case ID_FACE_FORWARD_AGAIN:
        {
            if (turn_to_heading(_id_forward_heading, ROT_SPEED)) {
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
                    _state = ID_CHECK_CUBE_AHEAD;
                } else {
                    _state = ID_CLASSIFY;
                }
            }
            break;
        }

        // ── 2.10: drive forward briefly to detect the world-6 cube ──────────
        case ID_CHECK_CUBE_AHEAD:
        {
            _id_wall_steps++;
            double f = front_obstacle();
            double travelled = sqrt((_x - _id_scan_start_x) * (_x - _id_scan_start_x)
                                  + (_y - _id_scan_start_y) * (_y - _id_scan_start_y));

            if (f > CUBE_DETECT_THRESH) {
                set_speed(0.0, 0.0);
                _id_cube_ahead = true;
                cout << "[Phase2.10] CUBE detected ahead at " << travelled
                     << " m (sensor=" << f << ")." << endl;
                _state = ID_CLASSIFY;
            } else if (_id_wall_steps > CUBE_CHECK_STEPS) {
                set_speed(0.0, 0.0);
                _id_cube_ahead = false;
                cout << "[Phase2.10] No cube ahead within " << travelled
                     << " m." << endl;
                _state = ID_CLASSIFY;
            } else {
                set_speed(WALL_SPEED, WALL_SPEED);
            }
            break;
        }

        // ── 2.11: classify based on saved measurements ──────────────────────
        case ID_CLASSIFY:
        {
            set_speed(0.0, 0.0);

            cout << "════════════════════════════════════════════════════════" << endl;
            cout << "[Phase2.11] WORLD IDENTIFICATION SUMMARY" << endl;
            cout << "  brightness     = " << _id_brightness          << endl;
            cout << "  R/G/B          = " << _id_avg_r << " / " << _id_avg_g << " / " << _id_avg_b << endl;
            cout << "  right wall     = " << (_id_right_wall_found ? "YES" : "NO ")
                 << "  (dist=" << _id_right_wall_dist << " m)" << endl;
            cout << "  left  wall     = " << (_id_left_wall_found  ? "YES" : "NO ")
                 << "  (dist=" << _id_left_wall_dist  << " m)" << endl;
            cout << "  front blocked  = " << (_id_front_blocked_at_start ? "YES" : "NO") << endl;
            cout << "  cube ahead     = " << (_id_cube_ahead ? "YES" : "NO") << endl;
              cout << "  side peaks(L/R)= " << _id_right_scan_left_peak
                  << " / " << _id_left_scan_right_peak << endl;
            cout << "════════════════════════════════════════════════════════" << endl;

            _world_id = classify_world_full();
            cout << "[Phase2.11] >>> Identified as WORLD " << _world_id << " <<<" << endl;

            load_waypoints(_world_id);
            _current_wp = 0;
            _state = NAVIGATE_WAYPOINT;
            break;
        }

        // ════════════════════════════════════════════════════════════════════
        // ── Legacy single-snapshot identifier (kept as fallback) ────────────
        // ════════════════════════════════════════════════════════════════════
        case IDENTIFY_WORLD:
        {
            set_speed(0.0, 0.0);
            _world_id = classify_world();
            load_waypoints(_world_id);
            _current_wp = 0;
            _state = NAVIGATE_WAYPOINT;
            cout << "[Phase2] World " << _world_id
                 << " identified. " << _waypoints.size()
                 << " waypoints loaded." << endl;
            break;
        }

        // ── Phase 3 + 4: navigate waypoints, detect victims inline ────────
        case NAVIGATE_WAYPOINT:
        {
            // Décrémente le compteur d'ignore detection
            if (_ignore_detection_steps > 0) {
                _ignore_detection_steps--;
            }

            // Victim detection runs every step during exploration
            // Mais on l'ignore pendant quelques secondes après avoir trouvé une victime
            if (_victims_found < 2 && _ignore_detection_steps == 0 && detect_green_victim()) {
                double ratio, center_x;
                victim_position_in_image(ratio, center_x);

                if (ratio >= 0.15) {
                    // Assez proche → on stop et on spin
                    set_speed(0.0, 0.0);
                    _victims_found++;
                    cout << "[Phase4] Victim " << _victims_found
                         << " found at (" << _x << ", " << _y << ")!" << endl;
                    // Ignore les détections pendant les ~5 secondes suivantes
                    // (le temps que le robot s'éloigne de cette victime)
                    _ignore_detection_steps = 80;  // 80 * 64ms ≈ 5 secondes
                    start_spin(NAVIGATE_WAYPOINT);
                    break;
                } else {
                    // On voit la victime mais on est encore loin
                    // → tourner VERS le vert + avancer
                    cout << "[Phase4] Approaching victim... (center_x=" << center_x << ")" << endl;

                    if (center_x < -0.2) {
                        // Vert à gauche → tourner à gauche en avançant
                        set_speed(1.0, 3.0);
                    } else if (center_x > 0.2) {
                        // Vert à droite → tourner à droite en avançant
                        set_speed(3.0, 1.0);
                    } else {
                        // Vert centré → foncer
                        set_speed(3.0, 3.0);
                    }
                    break;
                }
            }

            if (_victims_found == 2) {
                _state = RETURN_TO_START;
                cout << "[Phase3] Both victims found. Returning to start." << endl;
                break;
            }

            if (_current_wp >= (int)_waypoints.size()) {
                // Swept all waypoints — return even if not all victims found
                _state = RETURN_TO_START;
                cout << "[Phase3] All waypoints visited." << endl;
                break;
            }

            Waypoint& wp = _waypoints[_current_wp];
            double dist = dist_to(wp.x, wp.y);

            if (dist < WAYPOINT_DIST_TOL) {
                _current_wp++;
                break;
            }

            double target_angle = atan2(wp.y - _y, wp.x - _x);
            double angle_err    = normalize_angle(target_angle - _theta);

            if (front > OBSTACLE_FRONT_THRESH) {
                _avoid_steps++;
                // Skip this waypoint after ~3 s of being unable to pass it.
                // 47 steps * 64 ms ≈ 3 s — keeps the robot from looping forever.
                if (_avoid_steps > 47) {
                    cout << "[Nav] Waypoint " << _current_wp
                         << " blocked, skipping." << endl;
                    _current_wp++;
                    _avoid_steps = 0;
                } else {
                    // Arc forward while turning — physically moves around the obstacle
                    // rather than spinning in place. Left wheel faster → arcs right
                    // (clockwise), which tends to pass obstacles on the left side.
                    set_speed(WALL_SPEED, WALL_SPEED * 0.25);
                }
            } else {
                _avoid_steps = 0;   // reset as soon as path is clear
                if (fabs(angle_err) > ANGLE_TOL) {
                    double spd = (angle_err > 0) ? ROT_SPEED : -ROT_SPEED;
                    set_speed(-spd, spd);
                } else {
                    double corr = 1.5 * angle_err;
                    set_speed(FORWARD_SPEED - corr, FORWARD_SPEED + corr);
                }
            }
            break;
        }

        // ── Phase 4 spin: timed full rotation ─────────────────────────────
        // Step-counter is more reliable than compass-delta under noise.
        // At MAX_SPEED with WHEELS_DISTANCE=0.3606: ω ≈ 4.58 rad/s → 360° in ~1.37 s.
        // 30 steps × 64 ms = 1.92 s comfortably covers one full circle.
        case SPIN_VICTIM:
        {
            _spin_total += 1.0;
            if (_spin_total >= 30.0) {
                set_speed(0.0, 0.0);
                _spin_total = 0.0;
                _state = _state_after_spin;
                cout << "[Phase4] Spin complete." << endl;
            } else {
                set_speed(-MAX_SPEED, MAX_SPEED);
            }
            break;
        }

        // ── Phase 5: navigate back to origin ─────────────────────────────
        case RETURN_TO_START:
        {
            double dist = dist_to(_origin_x, _origin_y);

            if (dist < WAYPOINT_DIST_TOL) {
                set_speed(0.0, 0.0);
                _state = TASK_COMPLETE;
                cout << "[Phase5] Returned to departure line. Task complete!" << endl;
                break;
            }

            double target_angle = atan2(_origin_y - _y, _origin_x - _x);
            double angle_err    = normalize_angle(target_angle - _theta);

            if (front > OBSTACLE_FRONT_THRESH) {
                set_speed(WALL_SPEED, WALL_SPEED * 0.25);   // arc right past obstacle
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
            set_speed(0.0, 0.0);
            break;
        }

        cout << "[" << state_name() << "]"
             << " x=" << _x << " y=" << _y << " θ=" << _theta
             << " front=" << front << " right=" << right
             << " victims=" << _victims_found
             << " wp=" << _current_wp << "/" << _waypoints.size()
             << endl;
    }

    set_speed(0.0, 0.0);
}

//////////////////////////////////////////////

void MyRobot::start_spin(State next_state)
{
    _state_after_spin = next_state;
    _spin_total       = 0.0;
    _state = SPIN_VICTIM;
}

//////////////////////////////////////////////

bool MyRobot::detect_green_victim()
{
    if (!_forward_camera) return false;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return false;

    int width  = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total  = width * height;
    int green_count = 0;

    unsigned char r, g, b;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            r = _forward_camera->imageGetRed(img,   width, x, y);
            g = _forward_camera->imageGetGreen(img, width, x, y);
            b = _forward_camera->imageGetBlue(img,  width, x, y);
            // Relative dominance works under fog: G must beat R and B by a margin,
            // not just undercut an absolute ceiling that fog can push everything below.
            if (g > GREEN_MIN_G && g > (int)r + GREEN_DOMINANCE && g > (int)b + GREEN_DOMINANCE)
                green_count++;
        }
    }

    return (double)green_count / total >= GREEN_RATIO_THRESH;
}

void MyRobot::victim_position_in_image(double& ratio, double& center_x)
{
    ratio = 0.0;
    center_x = 0.0;
    if (!_forward_camera) return;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return;

    int width  = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    int total  = width * height;
    int green_count = 0;
    long sum_x = 0;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            unsigned char r = _forward_camera->imageGetRed(img,   width, x, y);
            unsigned char g = _forward_camera->imageGetGreen(img, width, x, y);
            unsigned char b = _forward_camera->imageGetBlue(img,  width, x, y);
            if (g > GREEN_MIN_G && g > (int)r + GREEN_DOMINANCE && g > (int)b + GREEN_DOMINANCE) {
                green_count++;
                sum_x += x;
            }
        }
    }

    ratio = (double)green_count / total;
    if (green_count > 0) {
        // Position moyenne en x, normalisée entre -1 (gauche) et +1 (droite)
        double avg_x = (double)sum_x / green_count;
        center_x = (avg_x / width) * 2.0 - 1.0;
    }
}

bool MyRobot::victim_is_close()
{
    double ratio, center_x;
    victim_position_in_image(ratio, center_x);
    return ratio >= 0.15;   // 15% : on est à ~1m
}

//////////////////////////////////////////////

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

//////////////////////////////////////////////
// ── Phase 2 helpers ──────────────────────────────────────────────────────

void MyRobot::measure_camera_brightness()
{
    _id_brightness = _id_avg_r = _id_avg_g = _id_avg_b = 0.0;
    if (!_forward_camera) return;
    const unsigned char* img = _forward_camera->getImage();
    if (!img) return;

    int width  = _forward_camera->getWidth();
    int height = _forward_camera->getHeight();
    long total = (long)width * (long)height;
    if (total <= 0) return;

    long sum_r = 0, sum_g = 0, sum_b = 0;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            sum_r += _forward_camera->imageGetRed  (img, width, x, y);
            sum_g += _forward_camera->imageGetGreen(img, width, x, y);
            sum_b += _forward_camera->imageGetBlue (img, width, x, y);
        }
    }

    _id_avg_r = (double)sum_r / total;
    _id_avg_g = (double)sum_g / total;
    _id_avg_b = (double)sum_b / total;
    _id_brightness = (_id_avg_r + _id_avg_g + _id_avg_b) / 3.0;
}

//////////////////////////////////////////////

bool MyRobot::front_blocked()
{
    // True if any front-facing distance sensor reads above the threshold.
    // Used as the World-6 trap (obstacle on the yellow line).
    return front_obstacle() > OBSTACLE_FRONT_THRESH;
}

//////////////////////////////////////////////

int MyRobot::classify_world_full()
{
    // ── Strategy: decision tree on the wall pattern (per Elizabeth's table) ─
    // World 5 is intentionally not handled here (will be detected as a runtime
    // obstacle during navigation, not during identification).
    //
    //   World | Left      | Right    | Tiebreaker
    //     1   | wall      | wall     | bright (no fog)
    //     2   | cubes     | wall     | very dark
    //     3   | cubes     | wall     | dark + fog
    //     4   | wall      | no wall  | dim/foggy
    //     6   | no wall   | no wall  | cube ahead
    //     7   | no wall   | no wall  | no cube
    //     8   | no wall   | wall     | bright
    //     9   | wall      | wall     | medium light + fog
    //    10   | wall      | no wall  | bright
    //
    // First-cut decision: classify each side as wall or no-wall based on
    // whether the robot stopped on a sensor hit before timing out.
    //
    // Cubes (world 2 & 3 left side) read as "wall" (a hit happened) — they are
    // disambiguated from real walls by the very-dark camera signature (world 2)
    // or the fog signature (world 3).
    // ─────────────────────────────────────────────────────────────────────────

    bool L = _id_left_wall_found;
    bool R = _id_right_wall_found;
    double b = _id_brightness;
    double side_diff = fabs(_id_left_wall_dist - _id_right_wall_dist);
    double side_peak_max = max(_id_right_scan_left_peak, _id_left_scan_right_peak);
    bool side_cube_signature = (_id_right_scan_left_peak > ID_SIDE_OBS_CUBE_THRESH) ||
                               (_id_left_scan_right_peak > ID_SIDE_OBS_CUBE_THRESH);

    // Saturation = max channel - min channel. Low saturation + non-trivial
    // brightness → grayscale → fog.
    double mx = std::max(_id_avg_r, std::max(_id_avg_g, _id_avg_b));
    double mn = std::min(_id_avg_r, std::min(_id_avg_g, _id_avg_b));
    double saturation = mx - mn;
    bool is_foggy = (b > 5.0) && (saturation < 12.0);

    cout << "[Classify] b=" << b << " sat=" << saturation
         << " foggy=" << is_foggy
         << " L=" << (L?"WALL":"NO ") << " R=" << (R?"WALL":"NO ")
            << " dL=" << _id_left_wall_dist << " dR=" << _id_right_wall_dist
            << " dDiff=" << side_diff
            << " sidePeakSig=" << (side_cube_signature ? "YES" : "NO")
         << " cube=" << (_id_cube_ahead ? "YES" : "NO") << endl;

    // ── Both sides clear → world 6 or 7, decided by cube check ─────────────
    if (!L && !R) {
        return _id_cube_ahead ? 6 : 7;
    }

    // ── Left wall, right clear → world 4 or 10, decided by brightness ──────
    if (L && !R) {
        if (b >= BRIGHT_MIN) return 10;   // bright = world 10
        return 4;                         // dimmer + foggy = world 4
    }

    // ── Right wall, left clear → world 8 (only candidate) ──────────────────
    if (!L && R) {
        return 8;
    }

    // ── Both sides have something (wall OR cubes) → worlds 1, 2, 3, 9 ──────
    // Worlds 2/3 include a cube ahead (closer than side walls). We rely on the
    // explicit front cube probe instead of brightness-only thresholds.
    // Guard: only trust this branch for dark scenes. In bright scenes, occasional
    // front echoes can falsely report a cube.
    if (L && R && _id_cube_ahead && b < MEDIUM_MIN) {
        return is_foggy ? 3 : 2;
    }

    // If side distances are asymmetric but no front cube was found, this is
    // likely just an off-center start in a wall+wall world (1 or 9).
    if (side_diff > 1.0 && !_id_cube_ahead) {
        // In some maps, world 2/3 may not trigger front-cube probe reliably,
        // but still produce very strong side echoes during scan due to cubes.
        if (side_cube_signature) {
            // Bright runs with very strong side peaks are typically world 3 in
            // this map set. Avoid forcing them to world 2.
            if (b >= BRIGHT_MIN && side_peak_max >= ID_SIDE_OBS_VERY_STRONG) {
                return 3;
            }
            return is_foggy ? 3 : 2;
        }
        if (b >= BRIGHT_MIN) return 1;
        if (b >= MEDIUM_MIN) return 9;
        return 9;
    }

    // Discriminate by brightness:
    //   World 2: very dark         (b < DARK_MIN)
    //   World 1: bright, no fog    (b >= BRIGHT_MIN, low saturation? actually high R/G/B)
    //   World 9: medium + fog      (MEDIUM_MIN <= b < BRIGHT_MIN, foggy)
    //   World 3: dark + fog        (b < MEDIUM_MIN, foggy)
    if (b < DARK_MIN) {
        return 2;                   // very dark → world 2
    }
    if (b >= BRIGHT_MIN) {
        return 1;                   // bright → world 1
    }
    if (b >= MEDIUM_MIN) {
        return 9;                   // medium light → world 9
    }
    return 3;                       // dark + fog → world 3
}

//////////////////////////////////////////////

void MyRobot::load_waypoints(int world_id)
{
    _waypoints.clear();

    // Waypoints are GPS-world coordinates, NOT relative to the origin.
    // Long axis of the room runs along x (Webots Z). Original Bug_02 goal was
    // at x≈9.47, so we sweep from near start toward x≈9.
    // _origin_y is the lateral starting position — sweep ±3 m around it.
    // *** Run the simulation once and check [Init] GPS seed to verify direction.
    //     If victims are behind the robot, negate the x offsets. ***

    double spread = 3.0;   // largeur du zigzag (m)

    _waypoints = {
        { 1.5,        0.0  },   // 1.5m devant
        { 1.5,    +spread  },   // puis 3m à gauche
        { 4.0,    +spread  },   // avance à gauche
        { 4.0,    -spread  },   // traverse à droite
        { 6.5,    -spread  },   // avance à droite
        { 6.5,    +spread  },   // retraverse à gauche
        { 8.5,    +spread  },
        { 8.5,    -spread  },
        { 9.5,        0.0  },   // dernier point au centre, loin devant
    };

    cout << "[Waypoints] Loaded " << _waypoints.size()
         << " waypoints (relative to robot start)" << endl;
}

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

bool MyRobot::turn_to_heading(double target, double max_speed)
{
    double err = normalize_angle(target - _theta);
    if (fabs(err) < ANGLE_TOL) {
        set_speed(0.0, 0.0);
        return true;
    }

    double speed = ROT_KP * fabs(err);
    if (speed < ROT_MIN_SPEED) speed = ROT_MIN_SPEED;
    if (speed > max_speed) speed = max_speed;
    if (err < 0.0) speed = -speed;

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
    double max_val = 0.0;
    int ids[] = {2, 3, 4, 5};
    for (int i = 0; i < 4; i++)
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    return max_val;
}

//////////////////////////////////////////////

double MyRobot::right_obstacle()
{
    double max_val = 0.0;
    int ids[] = {10, 11, 12, 13};
    for (int i = 0; i < 4; i++)
        if (_ds[ids[i]] && _ds[ids[i]]->getValue() > max_val)
            max_val = _ds[ids[i]]->getValue();
    return max_val;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y << " theta:" << _theta << endl;
}

//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
    return dist_to(_origin_x, _origin_y) < WAYPOINT_DIST_TOL;
}

//////////////////////////////////////////////

const char* MyRobot::state_name()
{
    switch (_state) {
        case ORIENT_INITIAL:         return "ORIENT";
        case LOCALIZE_FIND_WALL:     return "FIND_WALL";
        case LOCALIZE_FOLLOW_CORNER: return "FOLLOW_CORNER";
        case LOCALIZE_ALIGN:         return "ALIGN";
        case ID_FACE_FORWARD:        return "ID_FACE_FWD";
        case ID_MEASURE_LIGHT:       return "ID_LIGHT";
        case ID_TURN_RIGHT:          return "ID_TURN_R";
        case ID_SETTLE_RIGHT:        return "ID_SETTLE_R";
        case ID_DRIVE_TO_RIGHT_WALL: return "ID_SCAN_R";
        case ID_RETURN_FROM_RIGHT:   return "ID_BACK_R";
        case ID_TURN_LEFT:           return "ID_TURN_L";
        case ID_SETTLE_LEFT:         return "ID_SETTLE_L";
        case ID_DRIVE_TO_LEFT_WALL:  return "ID_SCAN_L";
        case ID_RETURN_FROM_LEFT:    return "ID_BACK_L";
        case ID_FACE_FORWARD_AGAIN:  return "ID_FACE_FWD2";
        case ID_CHECK_CUBE_AHEAD:    return "ID_CUBE";
        case ID_CLASSIFY:            return "ID_CLASSIFY";
        case IDENTIFY_WORLD:         return "IDENTIFY";
        case NAVIGATE_WAYPOINT:      return "NAVIGATE";
        case SPIN_VICTIM:            return "SPIN";
        case RETURN_TO_START:        return "RETURN";
        case TASK_COMPLETE:          return "DONE";
        default:                     return "?";
    }
}


//////////////////////////////////////////////

double MyRobot::measure_forward_distance()
{
    double max_obstacle = 0.0;
    int ids[] = {0, 1, 14, 15};
    for (int i = 0; i < 4; i++) {
        if (_ds[ids[i]]) {
            double v = _ds[ids[i]]->getValue();
            if (v > max_obstacle) max_obstacle = v;
        }
    }
    return 1024.0 - max_obstacle;
}



void MyRobot::run_orient_initial()
{
    const double SPIN_SPEED  = 2.0;
    const int    NUM_SAMPLES = 16;

    // Initialisation au tout premier appel
    if (!_orient_initialized) {
        _orient_start_theta   = _theta;
        _orient_best_heading  = _theta;
        _orient_best_distance = measure_forward_distance();
        _orient_step          = 0;
        _orient_initialized   = true;
        cout << "[Orient] Starting 360° scan from theta=" << _theta << endl;
    }

    // Étape 1 : on tourne et on prend NUM_SAMPLES mesures
    if (_orient_step < NUM_SAMPLES) {
        double angle_done = fabs(normalize_angle(_theta - _orient_start_theta));
        double target_angle = (_orient_step + 1) * (2.0 * M_PI / NUM_SAMPLES);

        if (angle_done >= target_angle - 0.05) {
            double dist = measure_forward_distance();
            cout << "[Orient] Sample " << _orient_step
                 << " theta=" << _theta
                 << " dist=" << dist << endl;

            if (dist > _orient_best_distance) {
                _orient_best_distance = dist;
                _orient_best_heading  = _theta;
            }
            _orient_step++;
        }
        set_speed(-SPIN_SPEED, SPIN_SPEED);
        return;
    }

    double err = normalize_angle(_orient_best_heading - _theta);
    if (fabs(err) < ANGLE_TOL) {
        // Étape 3 : ALIGNÉ ! On reset le repère
        set_speed(0.0, 0.0);
        _x = 0.0;
        _y = 0.0;
        _theta = 0.0;
        _origin_x = 0.0;
        _origin_y = 0.0;

        cout << "[Orient] Aligned! Best dist was " << _orient_best_distance << endl;
        cout << "[Orient] Frame reset: position=(0,0), heading=0" << endl;

        _state = IDENTIFY_WORLD;
        return;
    }

    double spd = (err > 0) ? SPIN_SPEED : -SPIN_SPEED;
    set_speed(-spd, spd);
}
