#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Rescue controller matching the original spec:
 *            1. Identify world (left wall scan, right wall scan, light)
 *            2. Load that world's preset waypoint path
 *            3. Navigate path; camera detects victims along the way
 *            4. Reverse the path to return home
 *
 * USER ACTION REQUIRED:
 *   Fill in the world1_path .. world10_path tables below with GPS
 *   coordinates measured from your .wbt files. Each path should be a
 *   sequence of safe positions the robot can drive between without
 *   hitting obstacles.
 */

#include <iostream>
#include <vector>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>

using namespace std;
using namespace webots;

// ── Robot constants ──────────────────────────────────────────────────────
#define MAX_SPEED              10.0
#define WHEELS_DISTANCE         0.3606
#define WHEEL_RADIUS            0.0825
#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DS                 16

// ── Speeds ───────────────────────────────────────────────────────────────
#define SPEED_FORWARD           4.0
#define SPEED_ROTATE            2.0
#define SPEED_PURSUE            3.0

// ── Tolerances ───────────────────────────────────────────────────────────
#define ANGLE_TOL               0.10
#define WAYPOINT_REACHED_M      0.5

// ── World identification thresholds ──────────────────────────────────────
#define WALL_SCAN_DRIVE_TICKS  300      // max ticks driving sideways scanning
#define WALL_HIT_THRESH        100.0    // front-sensor reading = wall reached
#define WALL_FAR_M             4.5      // beyond this = "no wall"

#define BRIGHT_THRESH          55.0     // light: bright (worlds 1, 8, 10)
#define MEDIUM_THRESH          28.0     // medium (world 9)
#define DARK_THRESH             4.0     // very dark (world 2)

// ── Camera / victim detection ────────────────────────────────────────────
#define GREEN_MIN_G            80      // Lowered from 100 - detect darker green
#define GREEN_DOMINANCE         25     // Lowered from 30 - more lenient
#define GREEN_DETECT_RATIO       0.01  // any green seen (lowered from 0.02)
#define GREEN_CONFIRM_RATIO      0.05  // close enough to confirm (lowered from 0.12 - was too strict!)

#define VICTIM_DEDUP_RADIUS_M    1.5    // ignore re-detections within this
#define DEFAULT_VICTIM_TARGET    2

// ── Obstacle handling during waypoint nav ────────────────────────────────
#define FRONT_BLOCKED_THRESH   100.0
#define STUCK_TIMEOUT_TICKS    100      // skip waypoint after no progress

// ─────────────────────────────────────────────────────────────────────────
// ── WAYPOINT PATHS ── FILL THESE IN PER WORLD ────────────────────────────
// ─────────────────────────────────────────────────────────────────────────
//
// Each path is a list of (x, y) GPS coordinates. The robot will drive to
// each one in order. Coordinates need to form a safe driving path — the
// robot will go in straight lines between them, so don't put two points
// on opposite sides of a wall.
//
// Place enough waypoints to cover the area where victims might be. The
// camera will spot them automatically as the robot passes.
//
// Tip: measure 3-6 points per world. Start point doesn't need to be
// included — the controller automatically remembers it.

const float world1_path[][2] = {
    // Adjusted waypoints - robot starts at x≈-6, not -9.05
    // Moving toward victims at positive x
    { -3.05,  -1.33   },  // First turn area
    {  1.879, -0.317  },  // Middle section
    {  5.231,  1.027  },  // Approaching victim area
    {  8.82,   0.4215 },  // Near victim 1
    {  8.816, -2.018  },  // Near victim 2
};

const float world2_path[][2] = {
    { -7.57,   3.3   },
    { -2.35,   1.619 },
    {  1.866,  4.17  },
    {  5.26,   0.487 },
    {  8.83,   0.48  },  // Near victim 1
    {  8.83,   3.35  },  // Near victim 2
};

const float world3_path[][2] = {
    { -5.92,   0.875 },
    { -2.5,    1.369 },
    {  3.47,   2.79  },
    {  8.38,   2.78  },  // Near victim 1
    {  8.37,  -2.26  },  // Near victim 2
};

const float world4_path[][2] = {
    // World 4 has the same physical layout as world 1, just different lighting (foggy)
    { -3.05,  -1.33   },
    {  1.879, -0.317  },
    {  5.231,  1.027  },
    {  8.82,   0.4215 },
    {  8.816, -2.018  },
};

const float world5_path[][2] = {
    { -2.86,  -1.809  },
    {  0.1,   -3.1    },
    {  8.66,  -3.19   },  // Near victim 1
    {  8.67,   3.18   },  // Near victim 2
};

const float world6_path[][2] = {
    { -2.9,   -1.32   },
    {  0.215, -3.144  },
    {  3.175, -2.9    },
    {  9.065, -2.918  },  // Near victim 1
    {  9.07,   0.831  },  // Near victim 2
};

const float world7_path[][2] = {
    {  2.05,   2.65   },
    {  7.93,   1.38   },  // Near victim 1
    {  8.49,  -3.377  },  // Near victim 2
};

const float world8_path[][2] = {
    { -3.06,  -1.23   },
    { -3.065, -3.479  },
    {  0.925, -2.755  },
    {  8.49,  -2.76   },  // Near victim 1
    {  8.49,  -1.51   },  // Near victim 2
};

const float world9_path[][2] = {
    { -3.05,  -1.33   },
    {  1.879, -0.317  },
    {  5.26,   0.717  },
    {  8.225,  3.25   },  // Near victim 1
    {  8.744, -3.668  },  // Near victim 2
};

const float world10_path[][2] = {
    { -4.67,  -0.43  },
    { -4.24,   1.222 },
    { -2.62,  -0.18  },
    {  2.011,  1.07  },
    {  5.77,   2.52  },
    {  9.42,   0.71  },  // Near both victims
};

// ─────────────────────────────────────────────────────────────────────────

struct Waypoint { double x, y; };

class MyRobot : public Robot {
public:
    MyRobot();
    ~MyRobot();
    void run();

private:
    // ── Phase tracking ──
    enum State {
        // Phase 1: world identification (4 steps)
        ID_SCAN_LEFT,        // turn left, drive forward, measure
        ID_RETURN_FROM_LEFT, // drive backward to start line
        ID_SCAN_RIGHT,       // turn right, drive forward, measure
        ID_RETURN_FROM_RIGHT,
        ID_FACE_FORWARD,     // re-orient to forward
        ID_CLASSIFY,         // compute world ID + load path

        // After path loaded, rotate to face first waypoint before driving.
        // (Robot spawns at fixed position but RANDOM heading on the yellow line,
        // so we must explicitly aim at the path before moving.)
        INITIAL_TURN,

        // Phase 2-3: drive waypoint path with camera detection
        FOLLOW_PATH,
        SPIN_VICTIM,

        // Phase 4: reverse the path to go home
        RETURN_PATH,

        DONE
    };
    State _state;
    State _state_after_spin;

    int _time_step;

    // ── Pose ──
    double _left_speed, _right_speed;
    float  _x, _y, _theta;
    float  _start_x, _start_y;
    float  _id_start_x, _id_start_y;     // anchor for return-to-start during scan
    float  _id_forward_heading;          // heading after initial localization
    float  _scan_start_x, _scan_start_y;

    // ── Scan results ──
    double _left_wall_distance_m;
    double _right_wall_distance_m;
    double _light_brightness;
    bool   _left_wall_found;
    bool   _right_wall_found;
    int    _scan_ticks;
    int    _world_id;

    // ── Path navigation ──
    vector<Waypoint> _path;
    int    _current_wp;        // forward index
    int    _return_wp;         // backward index for return phase
    int    _stuck_ticks;
    double _last_dist_to_wp;

    // ── Victim tracking ──
    int          _victims_found;
    int          _victims_target;
    vector<Waypoint> _known_victims;
    int          _spin_ticks;

    // ── Sensors ──
    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;
    Compass*        _my_compass;
    GPS*            _my_gps;
    Camera*         _forward_camera;
    DistanceSensor* _ds[NUM_DS];

    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;

    float _prev_left_enc, _prev_right_enc;

    // ── Helpers ──
    double front_obstacle();
    double left_obstacle();
    double right_obstacle();
    double dist_to(double tx, double ty);
    double normalize_angle(double a);
    void   set_speed(double l, double r);
    bool   turn_to_heading(double target);
    void   compute_odometry();
    double get_heading_radians();

    bool   green_detected(double& ratio, double& center_x);
    bool   too_close_to_known_victim();
    double measure_camera_brightness();

    // ── Phase steps ──
    void step_id_scan_left();
    void step_id_return_from_left();
    void step_id_scan_right();
    void step_id_return_from_right();
    void step_id_face_forward();
    void step_id_classify();
    void step_initial_turn();
    void step_follow_path();
    void step_spin_victim();
    void step_return_path();

    // ── Path loading ──
    void load_path_for_world(int world_id);
    template<size_t N>
    void copy_path(const float (&table)[N][2]);

    const char* state_name();
};

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