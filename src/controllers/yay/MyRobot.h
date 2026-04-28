#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   5-Phase Rescue Controller: Localize → Identify → Navigate → Detect → Return
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

#define MAX_SPEED               10.0
#define WHEELS_DISTANCE         0.3606  // meters — measured Pioneer II axle width
#define WHEEL_RADIUS            0.0825  // meters
#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DS                  16

// --- Tunable thresholds ---

// Compass heading (radians) the robot should face after corner alignment.
// 0.0 = east in Webots default frame. Adjust to match your world's long axis.
#define ALIGN_HEADING           0.0

// Distance sensor trigger levels (raw sensor units)
#define OBSTACLE_FRONT_THRESH   80.0
#define OBSTACLE_SIDE_THRESH    70.0

// Navigation tolerances
#define ANGLE_TOL               0.08    // radians
#define WAYPOINT_DIST_TOL       0.30    // meters — close enough to claim waypoint reached
#define VICTIM_DIST_TOL         1.0     // meters — must stop within this to score

// Rotation control (proportional, avoids overshoot from fixed-speed turns)
#define ROT_KP                  1.8
#define ROT_MIN_SPEED           0.45

// Green cylinder detection.
// Relative margins handle fog scenarios where absolute brightness drops —
// a pixel is "green" when G clearly dominates R and B, not just when R and B
// are below a fixed ceiling.
#define GREEN_MIN_G             100     // absolute floor to reject near-black pixels
#define GREEN_DOMINANCE         30      // G must exceed both R and B by this margin
#define GREEN_RATIO_THRESH      0.04    // fraction of pixels that must qualify

// ── Phase 2: World identification thresholds ──────────────────────────────
// Camera brightness bands (0..255), calibrated from world 1 reading (~73).
//   World 1, 8, 10  (lights=50)            → BRIGHT     (~70-90)
//   World 9         (lights=10)            → MEDIUM     (~30-50)
//   World 3,4,6,7   (lights=0.1, fog)      → DARK_FOG   (~10-25)
//   World 2         (lights=0.05, no fog)  → VERY_DARK  (~0-5)
// *** TUNE from printed values per world ***
#define DARK_MAX        65.0     // monde 9
#define MEDIUM_MIN      70.0
#define BRIGHT_MIN      85.0
#define VERY_BRIGHT_MIN 105.0    // monde 3 / 6

// Wall-scan parameters
//   Side walls are roughly 4-5 m away. At WALL_SPEED=3.0 m/s on Pioneer wheels
//   (radius 0.0825), one step covers roughly 0.016 m. 400 steps ≈ 6.4 m max travel.
//   If front sensor doesn't fire by then → "no wall" on that side.
#define WALL_DRIVE_TIMEOUT       400      // steps — gives ~6 m of travel
#define WALL_FOUND_DIST_M        4.0      // if travelled less than this before stop → real wall
#define RETURN_TO_LINE_TOL_M     0.06     // tighter tolerance to reduce drift before classify/probe
#define WALL_SPEED_SLOW          2.0       // Slow approach speed for center probes
#define OBSTACLE_FRONT_SLOW_THRESH 60.0    // start slowing before hard stop threshold
#define OBSTACLE_FRONT_HARD_STOP 110.0      // immediate stop if a strong close hit appears
#define ID_FRONT_CONFIRM_STEPS   2         // require consecutive hits to filter sensor spikes
#define ID_ROT_SETTLE_STEPS      5         // wait this many ticks after turn before scan start
#define ID_HEADING_KP            1.6       // heading hold while scanning straight
#define ID_HEADING_DRIFT_ABORT   0.25      // re-align if heading drifts too far during scan
#define ID_SIDE_OBS_CUBE_THRESH  180.0     // strong side echo during side-scan => likely cube lane (w2/w3)

// World-6 cube check: drive forward this many steps after left/right scan (only if
// both sides reported "no wall"), then check front sensor.
#define CUBE_CHECK_STEPS         150      // ~2.4 m of forward travel
#define CUBE_DETECT_THRESH       80.0     // sensor reading that confirms cube ahead
#define CUBE_MAX_DIST_M          1.2      // cube should be close; farther hits are likely room wall

// Forward-probe (after each side scan, while facing room center) ------------
//   IR sensors only see ~0.4 m, so a stationary "is there a wall in front?"
//   reading is useless when the wall sits 2-4 m away. We instead roll forward
//   up to PROBE_FORWARD_MAX_M and use the distance travelled as the answer.
#define PROBE_FORWARD_MAX_M      2.0      // hard cap → if travelled this far without a hit, NO WALL ahead
#define PROBE_FORWARD_TIMEOUT    120      // step-count safety net, ~2.4 m worst case

// --------------------------

struct Waypoint { double x, y; };

class MyRobot : public Robot {
public:
    MyRobot();
    ~MyRobot();
    void run();
    void print_odometry();
    bool goal_reached();

    GPS* _my_gps;

private:
    int _time_step;

    double _left_speed, _right_speed;
    float  _x, _y, _theta;
    float  _sr, _sl;
    float  _prev_left_enc, _prev_right_enc;

    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;
    Compass*        _my_compass;
    DistanceSensor* _ds[NUM_DS];
    Motor*          _left_wheel_motor;
    Motor*          _right_wheel_motor;
    Camera*         _forward_camera;
    Camera*         _spherical_camera;


    enum DetectedObject {
    OBJ_NOTHING,
    OBJ_WALL,
    OBJ_CUBE
    };

    // ── State machine ──────────────────────────────────────────────────────
    enum State {
        ORIENT_INITIAL,         // Phase 0:  tourne sur soi-mêm pour trouver le fond
        LOCALIZE_FIND_WALL,     // Phase 1a: drive until front wall detected
        LOCALIZE_FOLLOW_CORNER, // Phase 1b: follow wall right until corner
        LOCALIZE_ALIGN,         // Phase 1c: rotate to ALIGN_HEADING, reset odometry

        // ── Phase 2: world identification sub-states ────────────────────────
        ID_FACE_FORWARD,        // rotate compass to forward heading + check front (world 5 trap)
        ID_MEASURE_LIGHT,       // sample camera brightness while stationary
        ID_TURN_RIGHT,          // rotate +90° from forward heading
        ID_SETTLE_RIGHT,        // short stabilization pause before right scan
        ID_DRIVE_TO_RIGHT_WALL, // drive until front sensor reading peaks → save right wall dist
        ID_TURN_RIGHT_TO_CENTER,// rotate back 90° to face room center
        ID_MEASURE_RIGHT_CENTER,// (settle step) prepare forward probe at right-center
        ID_PROBE_RIGHT_CENTER,  // drive forward up to 2 m, record travelled = obstacle distance
        ID_BACKUP_RIGHT_CENTER, // reverse exactly the probe distance to recover original spot
        ID_TURN_RIGHT_TO_SCAN,  // rotate back to right scan heading before reverse
        ID_RETURN_FROM_RIGHT,   // drive backward until odometry shows we're back on yellow line
        ID_TURN_LEFT,           // rotate to face left (180° from right scan direction)
        ID_SETTLE_LEFT,         // short stabilization pause before left scan
        ID_DRIVE_TO_LEFT_WALL,  // drive until front sensor reading peaks → save left wall dist
        ID_TURN_LEFT_TO_CENTER, // rotate back 90° to face room center
        ID_MEASURE_LEFT_CENTER, // (settle step) prepare forward probe at left-center
        ID_PROBE_LEFT_CENTER,   // drive forward up to 2 m, record travelled = obstacle distance
        ID_BACKUP_LEFT_CENTER,  // reverse exactly the probe distance to recover original spot
        ID_TURN_LEFT_TO_SCAN,   // rotate back to left scan heading before reverse
        ID_RETURN_FROM_LEFT,    // drive backward until odometry shows we're back on yellow line
        ID_FACE_FORWARD_AGAIN,  // rotate back to forward heading
        ID_CHECK_CUBE_AHEAD,    // if both sides "no wall" — drive forward, check for world-6 cube
        ID_CLASSIFY,            // compare measurements → set _world_id + load waypoints

        IDENTIFY_WORLD,         // (legacy single-snapshot identifier — kept as fallback)
        NAVIGATE_WAYPOINT,      // Phase 3+4: waypoint nav with live victim detection
        SPIN_VICTIM,            // Phase 4:  360° spin to confirm victim
        RETURN_TO_START,        // Phase 5:  navigate back to (0,0)
        TASK_COMPLETE          
    };
    State _state;

    // Phase 0 — orientation initiale (cherche le fond de la salle)
    int    _orient_step;            // étape de la mesure (0 à 16)
    double _orient_best_heading;    // meilleur angle trouvé (= direction la + dégagée)
    double _orient_best_distance;   // distance libre maximale mesurée
    double _orient_start_theta;     // angle au début de la phase
    bool   _orient_initialized;     // flag pour init au 1er appel
    double _right_wall_inner_sensor;
    double _left_wall_inner_sensor;
    bool _right_wall_sees_second_yellow_side;
    bool _left_wall_sees_second_yellow_side;

    // Phase 1b sub-state: turn until wall is on right before following
    bool _corner_turning;

    // Phase 4 — victim tracking
    int    _victims_found;
    double _spin_total;       // step counter during spin
    State  _state_after_spin; // which state to resume when spin completes
    int    _ignore_detection_steps;  // ignore detection X steps after finding victim

    // Phase 2 — world identification measurements
    double _id_brightness;          // average camera brightness at start
    double _id_avg_r, _id_avg_g, _id_avg_b;  // separate channel averages (fog vs dark)
    double _id_right_wall_dist;     // meters traveled before right-side wall hit (or > timeout if none)
    double _id_left_wall_dist;      // meters traveled before left-side wall hit (or > timeout if none)
    double _id_right_center_front;  // front sensor value after turning toward room center (right scan)
    double _id_left_center_front;   // front sensor value after turning toward room center (left scan)
    bool   _id_right_wall_found;    // true if a wall stopped us within timeout
    bool   _id_left_wall_found;
    bool   _id_front_blocked_at_start;  // true → world 5 (obstacle on yellow line)
    bool   _id_cube_ahead;          // true → world 6 (cube in middle when both sides clear)
    double _id_initial_gps_x;       // GPS x at start of phase 2 (for context only — coarse)
    double _id_initial_gps_y;       // GPS y at start of phase 2
    double _id_scan_start_x;        // odometry x at start of a side scan
    double _id_scan_start_y;        // odometry y at start of a side scan
    double _id_line_anchor_x;       // fixed odometry anchor for phase-2 returns/probe
    double _id_line_anchor_y;
    double _id_forward_heading;     // compass heading we treat as "forward" (toward far yellow line)
    bool   _id_initialized;         // first-tick init flag for phase 2
    int    _id_wall_steps;          // step counter for drive-to-wall safety timeout
    int    _id_settle_steps;        // stabilization counter after a turn
    double _id_scan_heading_target; // heading to hold while driving/reversing side scans
    int    _id_front_hit_count;     // consecutive front-hit confirmations
    double _id_right_scan_left_peak; // peak LEFT side sensor while scanning right
    double _id_left_scan_right_peak; // peak RIGHT side sensor while scanning left
    int _id_front_object_type;

    DetectedObject _id_right_object;
    DetectedObject _id_left_object;
    DetectedObject _id_middle_object;

    bool _id_cube_on_right;
    bool _id_cube_on_left;

    // Forward probe (taken while facing the room center after a side scan):
    //   The IR sensors only see ~0.4 m, so a stationary reading at the center
    //   never reports a wall that's 2-4 m away. We therefore drive forward up
    //   to PROBE_FORWARD_MAX_M, recording the distance travelled before a hit.
    //   Travel = PROBE_FORWARD_MAX_M  →  no obstacle ahead (open area).
    //   Travel < ~0.7 m              →  obstacle very close (cube).
    //   Travel ~ 0.7 .. 2.0 m        →  obstacle (wall) ahead.
    double _id_probe_start_x;       // odometry x when the probe begins
    double _id_probe_start_y;       // odometry y when the probe begins
    double _id_probe_end_x;         // odometry x when the probe stops (used for backup)
    double _id_probe_end_y;         // odometry y when the probe stops
    int    _id_probe_steps;         // safety step counter for the probe
    bool   _id_right_center_hit;    // true if probe at right-center hit something
    bool   _id_left_center_hit;     // true if probe at left-center hit something

    // Phase 2+3 — world and waypoints
    int             _world_id;
    vector<Waypoint> _waypoints;
    int             _current_wp;

    // Origin point set after localization reset
    float _origin_x, _origin_y;

    // GPS correction — applied periodically to dampen odometry drift
    int _gps_timer;

    // Obstacle avoidance counter — resets when path clears, skips waypoint on timeout
    int _avoid_steps;

    // Victim approach tracking
    float _victim_distance_target;  // target distance to stop at (1.0 m)

    // ── Helper methods ─────────────────────────────────────────────────────
    void   compute_odometry();
    double get_heading_radians();
    float  encoder_tics_to_meters(float tics);
    double normalize_angle(double angle);
    void   set_speed(double left, double right);

    double front_obstacle();
    double left_obstacle();
    double right_obstacle();
    double dist_to(double tx, double ty);

    bool detect_green_victim();
    bool victim_is_close();
    void victim_position_in_image(double& ratio, double& center_x);
    int  classify_world();
    void load_waypoints(int world_id);
    void apply_gps_correction();
    void start_spin(State next_state);

    void   run_orient_initial();
    double measure_forward_distance();
    bool   turn_to_heading(double target, double max_speed);

    // ── Phase 2 (world identification) helpers ─────────────────────────────
    void   measure_camera_brightness();   // fills _id_brightness, _id_avg_{r,g,b}
    int    classify_world_full();         // uses brightness + wall distances + front check
    bool   front_blocked();               // true if obstacle directly in front (world 6 check)

    const char* state_name();
    const char* object_name(DetectedObject obj);


};

#endif
