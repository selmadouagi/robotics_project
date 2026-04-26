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

// Green cylinder detection.
// Relative margins handle fog scenarios where absolute brightness drops —
// a pixel is "green" when G clearly dominates R and B, not just when R and B
// are below a fixed ceiling.
#define GREEN_MIN_G             100     // absolute floor to reject near-black pixels
#define GREEN_DOMINANCE         30      // G must exceed both R and B by this margin
#define GREEN_RATIO_THRESH      0.04    // fraction of pixels that must qualify

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

    // ── State machine ──────────────────────────────────────────────────────
    enum State {
        ORIENT_INITIAL,         // Phase 0:  tourne sur soi-mêm pour trouver le fond
        LOCALIZE_FIND_WALL,     // Phase 1a: drive until front wall detected
        LOCALIZE_FOLLOW_CORNER, // Phase 1b: follow wall right until corner
        LOCALIZE_ALIGN,         // Phase 1c: rotate to ALIGN_HEADING, reset odometry
        IDENTIFY_WORLD,         // Phase 2:  camera snapshot → world classification
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

    // Phase 1b sub-state: turn until wall is on right before following
    bool _corner_turning;

    // Phase 4 — victim tracking
    int    _victims_found;
    double _spin_total;       // step counter during spin
    State  _state_after_spin; // which state to resume when spin completes
    int    _ignore_detection_steps;  // ignore detection X steps after finding victim

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

    const char* state_name();
};

#endif
