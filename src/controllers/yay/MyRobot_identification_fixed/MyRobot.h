#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Rescue controller matching the original spec:
 *            1. Identify world
 *            2. Load that world's preset waypoint path
 *            3. Navigate path; camera detects victims
 *            4. Reverse the path to return home
 *
 * @author  Elizabeth Faulkner
 * @date    2026-04
 */

#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace webots;

// ─────────────────────────────────────────────────────────────
// Robot constants
// ─────────────────────────────────────────────────────────────

#define MAX_SPEED               10.0
#define WHEELS_DISTANCE         0.3606
#define WHEEL_RADIUS            0.0825
#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DS                  16

#define FORWARD_SPEED           4.0
#define ROT_SPEED               2.5
#define WALL_SPEED              4.5

#define SPEED_FORWARD           4.0
#define SPEED_ROTATE            2.0
#define SPEED_PURSUE            3.0

// Fast speed only for identification rotations on the yellow line
#define ID_ROT_SPEED            5.0
#define ID_FORWARD_SPEED        6.0

// Compass heading
#define ALIGN_HEADING           0.0

// Distance sensor trigger levels
#define OBSTACLE_FRONT_THRESH   80.0
#define OBSTACLE_SIDE_THRESH    70.0

// Navigation tolerances
#define ANGLE_TOL               0.08
#define WAYPOINT_DIST_TOL       0.30
#define WAYPOINT_REACHED_M      0.5
#define VICTIM_DIST_TOL         1.0

// Rotation control
#define ROT_KP                  1.8
#define ROT_MIN_SPEED           0.45

// Green cylinder detection
#define GREEN_MIN_G             80
#define GREEN_DOMINANCE         25
#define GREEN_RATIO_THRESH      0.04
#define GREEN_DETECT_RATIO      0.01
#define GREEN_CONFIRM_RATIO     0.05

#define VICTIM_DEDUP_RADIUS_M   1.5
#define DEFAULT_VICTIM_TARGET   2

// Obstacle handling during waypoint nav
#define FRONT_BLOCKED_THRESH    100.0
#define STUCK_TIMEOUT_TICKS     100

// ─────────────────────────────────────────────────────────────
// Phase 2: World identification thresholds
// ─────────────────────────────────────────────────────────────

#define DARK_MAX        65.0
#define MEDIUM_MIN      70.0
#define BRIGHT_MIN      85.0
#define VERY_BRIGHT_MIN 105.0

#define WALL_DRIVE_TIMEOUT       400
#define WALL_FOUND_DIST_M        4.0
#define RETURN_TO_LINE_TOL_M     0.06
#define WALL_SPEED_SLOW          2.0
#define OBSTACLE_FRONT_SLOW_THRESH 60.0
#define OBSTACLE_FRONT_HARD_STOP 110.0
#define ID_FRONT_CONFIRM_STEPS   2
#define ID_ROT_SETTLE_STEPS      5
#define ID_HEADING_KP            1.6
#define ID_HEADING_DRIFT_ABORT   0.25
#define ID_SIDE_OBS_CUBE_THRESH  180.0

#define CUBE_CHECK_STEPS         150
#define CUBE_DETECT_THRESH       80.0
#define CUBE_MAX_DIST_M          1.2

#define PROBE_FORWARD_MAX_M      2.0
#define PROBE_FORWARD_TIMEOUT    120

// ─────────────────────────────────────────────────────────────
// Waypoint structure
// ─────────────────────────────────────────────────────────────

struct Waypoint {
    double x;
    double y;
};

// ─────────────────────────────────────────────────────────────
// Preset waypoint paths
// IMPORTANT: these arrays must stay OUTSIDE the class
// ─────────────────────────────────────────────────────────────

static const float world1_path[][2] = {
    { -3.05,  -1.33   },
    {  1.879, -0.317  },
    {  5.231,  1.027  },
    {  8.82,   0.4215 },
    {  8.816, -2.018  },
};

static const float world2_path[][2] = {
    { -7.57,   3.3   },
    { -2.35,   1.619 },
    {  1.866,  4.17  },
    {  5.26,   0.487 },
    {  8.83,   0.48  },
    {  8.83,   3.35  },
};

static const float world3_path[][2] = {
    { -5.92,   0.875 },
    { -2.5,    1.369 },
    {  3.47,   2.79  },
    {  8.38,   2.78  },
    {  8.37,  -2.26  },
};

static const float world4_path[][2] = {
    { -3.05,  -1.33   },
    {  1.879, -0.317  },
    {  5.231,  1.027  },
    {  8.82,   0.4215 },
    {  8.816, -2.018  },
};

static const float world5_path[][2] = {
    { -2.86,  -1.809  },
    {  0.1,   -3.1    },
    {  8.66,  -3.19   },
    {  8.67,   3.18   },
};

static const float world6_path[][2] = {
    { -2.9,   -1.32   },
    {  0.215, -3.144  },
    {  3.175, -2.9    },
    {  9.065, -2.918  },
    {  9.07,   0.831  },
};

static const float world7_path[][2] = {
    {  2.05,   2.65   },
    {  7.93,   1.38   },
    {  8.49,  -3.377  },
};

static const float world8_path[][2] = {
    { -3.06,  -1.23   },
    { -3.065, -3.479  },
    {  0.925, -2.755  },
    {  8.49,  -2.76   },
    {  8.49,  -1.51   },
};

static const float world9_path[][2] = {
    { -3.05,  -1.33   },
    {  1.879, -0.317  },
    {  5.26,   0.717  },
    {  8.225,  3.25   },
    {  8.744, -3.668  },
};

static const float world10_path[][2] = {
    { -4.67,  -0.43  },
    { -4.24,   1.222 },
    { -2.62,  -0.18  },
    {  2.011,  1.07  },
    {  5.77,   2.52  },
    {  9.42,   0.71  },
};

// ─────────────────────────────────────────────────────────────
// MyRobot class
// ─────────────────────────────────────────────────────────────

class MyRobot : public Robot {
public:
    MyRobot();
    ~MyRobot();
    void run();

    GPS* _my_gps;

private:
    int _time_step;

    double _left_speed;
    double _right_speed;

    float _x;
    float _y;
    float _theta;

    float _sr;
    float _sl;

    float _prev_left_enc;
    float _prev_right_enc;

    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;
    Compass*        _my_compass;
    DistanceSensor* _ds[NUM_DS];

    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;

    Camera* _forward_camera;
    Camera* _spherical_camera;

    enum DetectedObject {
        OBJ_NOTHING,
        OBJ_WALL,
        OBJ_CUBE
    };

    enum State {
        ID_FACE_FORWARD,
        ID_MEASURE_LIGHT,

        ID_TURN_RIGHT,
        ID_SETTLE_RIGHT,
        ID_DRIVE_TO_RIGHT_WALL,
        ID_TURN_RIGHT_TO_CENTER,
        ID_MEASURE_RIGHT_CENTER,
        ID_PROBE_RIGHT_CENTER,
        ID_BACKUP_RIGHT_CENTER,
        ID_TURN_RIGHT_TO_SCAN,
        ID_RETURN_FROM_RIGHT,

        ID_TURN_LEFT,
        ID_SETTLE_LEFT,
        ID_DRIVE_TO_LEFT_WALL,
        ID_TURN_LEFT_TO_CENTER,
        ID_MEASURE_LEFT_CENTER,
        ID_PROBE_LEFT_CENTER,
        ID_BACKUP_LEFT_CENTER,
        ID_TURN_LEFT_TO_SCAN,
        ID_RETURN_FROM_LEFT,

        ID_FACE_FORWARD_AGAIN,
        ID_CHECK_CUBE_AHEAD,
        ID_CLASSIFY,

        IDENTIFY_WORLD,
        NAVIGATE_WAYPOINT,

        INITIAL_TURN,
        FOLLOW_PATH,
        SPIN_VICTIM,
        RETURN_PATH,

        RETURN_TO_START,
        TASK_COMPLETE,
        DONE
    };

    State _state;
    State _state_after_spin;

    // ─────────────────────────────────────────────────────────
    // Victim tracking
    // ─────────────────────────────────────────────────────────

    int    _victims_found;
    int    _victims_target;
    int    _spin_ticks;
    int    _ignore_detection_steps;
    double _spin_total;

    vector<Waypoint> _known_victims;

    // ─────────────────────────────────────────────────────────
    // Path navigation - new system
    // ─────────────────────────────────────────────────────────

    vector<Waypoint> _path;

    int    _current_wp;
    int    _return_wp;
    int    _stuck_ticks;
    double _last_dist_to_wp;

    float _start_x;
    float _start_y;

    // ─────────────────────────────────────────────────────────
    // World identification measurements
    // ─────────────────────────────────────────────────────────

    double _id_brightness;
    double _id_avg_r;
    double _id_avg_g;
    double _id_avg_b;

    double _id_right_wall_dist;
    double _id_left_wall_dist;

    double _id_right_center_front;
    double _id_left_center_front;

    bool _id_right_wall_found;
    bool _id_left_wall_found;
    bool _id_cube_ahead;

    double _id_initial_gps_x;
    double _id_initial_gps_y;

    double _id_scan_start_x;
    double _id_scan_start_y;

    double _id_line_anchor_x;
    double _id_line_anchor_y;

    double _id_forward_heading;

    bool _id_initialized;

    int _id_wall_steps;
    int _id_settle_steps;
    int _id_front_hit_count;

    double _id_scan_heading_target;

    double _id_right_scan_left_peak;
    double _id_left_scan_right_peak;

    DetectedObject _id_left_object;
    DetectedObject _id_middle_object;
    DetectedObject _id_right_object;

    bool _id_cube_on_left;

    double _id_probe_start_x;
    double _id_probe_start_y;
    double _id_probe_end_x;
    double _id_probe_end_y;

    int  _id_probe_steps;
    bool _id_right_center_hit;
    bool _id_left_center_hit;

    int _id_front_object_type;

    // ─────────────────────────────────────────────────────────
    // Old waypoint system kept for compatibility
    // ─────────────────────────────────────────────────────────

    int _world_id;

    vector<pair<double, double> > _waypoints;

    float _origin_x;
    float _origin_y;

    int _gps_timer;
    int _avoid_steps;

    float _victim_distance_target;

    // ─────────────────────────────────────────────────────────
    // Helper methods
    // ─────────────────────────────────────────────────────────

    void   compute_odometry();
    double get_heading_radians();
    float  encoder_tics_to_meters(float tics);

    double normalize_angle(double angle);

    void set_speed(double left, double right);

    double front_obstacle();
    double left_obstacle();
    double right_obstacle();

    double dist_to(double tx, double ty);

    bool detect_green_victim();
    void victim_position_in_image(double& ratio, double& center_x);

    bool green_detected(double& ratio, double& center_x);

    bool too_close_to_known_victim();

    int classify_world();
    int classify_world_full();
    int classify_world_by_light_only();

    void load_waypoints(int world_id);
    void load_path_for_world(int world_id);

    void apply_gps_correction();

    void start_spin(State next_state);

    bool turn_to_heading(double target, double max_speed);
    bool turn_to_heading(double target);

    void measure_camera_brightness();

    const char* state_name();
    const char* object_name(DetectedObject obj);

    // ─────────────────────────────────────────────────────────
    // Phase 3 / path functions
    // ─────────────────────────────────────────────────────────

    void step_initial_turn();
    void step_follow_path();
    void step_spin_victim();
    void step_return_path();

    template<size_t N>
    void copy_path(const float (&table)[N][2]);
};

// ─────────────────────────────────────────────────────────────
// Template implementation
// Must stay in .h
// ─────────────────────────────────────────────────────────────

template<size_t N>
void MyRobot::copy_path(const float (&table)[N][2])
{
    _path.clear();
    _path.reserve(N);

    for (size_t i = 0; i < N; i++) {
        Waypoint w;
        w.x = table[i][0];
        w.y = table[i][1];
        _path.push_back(w);
    }
}

#endif