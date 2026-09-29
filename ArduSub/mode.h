#pragma once

#include "Sub.h"
class Parameters;
class ParametersG2;

class GCS_Sub;

// Guided modes
enum GuidedSubMode {
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};

// Auto modes
enum AutoSubMode {
    Auto_WP,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_TerrainRecover
};

// RTL states
enum RTLState {
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};

class Mode
{

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual angle with manual depth/throttle
        ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
        ALT_HOLD =      2,  // manual angle with automatic depth/throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        MANUAL =       19,  // Pass-through input with no stabilization
        MOTOR_DETECT = 20,  // Automatically detect motors orientation
        SURFTRAK =     21,  // Track distance above seafloor (hold range)
        DYNAMIC =      22,  // guided-style external control that holds position between commands, with commanded roll/pitch and optional bottom tracking
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // child classes should override these methods
    virtual bool init(bool ignore_checks) { return true; }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool requires_altitude() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool in_guided_mode() const { return false; }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;
  
    // pilot input processing
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);


protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AP_Motors6DOF &motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    RC_Channel *&channel_forward;
    RC_Channel *&channel_lateral;
    AC_PosControl *position_control;
    AC_AttitudeControl_Sub *attitude_control;
    // TODO: channels
    float &G_Dt;

public:

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Sub &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;
    virtual void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }
    Mode::Number number() const override { return Mode::Number::MANUAL; }
};


class ModeAcro : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Acro"; }
    const char *name4() const override { return "ACRO"; }
    Mode::Number number() const override { return Mode::Number::ACRO; }
};


class ModeStabilize : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }
    Mode::Number number() const override { return Mode::Number::STABILIZE; }
};


class ModeAlthold : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }
    void control_depth();

protected:

    void run_pre();
    void run_post();

    const char *name() const override { return "Depth Hold"; }
    const char *name4() const override { return "ALTH"; }
    Mode::Number number() const override { return Mode::Number::ALT_HOLD; }
};


class ModeSurftrak : public ModeAlthold
{

public:
    // constructor
    ModeSurftrak();

    void run() override;

    bool init(bool ignore_checks) override;

    float get_rangefinder_target_cm() const WARN_IF_UNUSED { return rangefinder_target_cm; }
    bool set_rangefinder_target_cm(float target_cm);

protected:

    const char *name() const override { return "Surftrak"; }
    const char *name4() const override { return "STRK"; }
    Mode::Number number() const override { return Mode::Number::SURFTRAK; }

private:

    void reset();
    void control_range();
    void update_surface_offset();

    float rangefinder_target_cm;

    bool pilot_in_control;            // pilot is moving up/down
    float pilot_control_start_z_cm;   // alt when pilot took control
};

class ModeGuided : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool guided_limit_check();
    void guided_limit_init_time_and_pos();
    void guided_set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void guided_set_angle(const Quaternion&, float);
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location&);
    bool guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_velocity(const Vector3f& velocity);
    void guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    float get_auto_heading();
    void guided_limit_clear();
    void set_auto_yaw_mode(autopilot_yaw_mode yaw_mode);

protected:

    const char *name() const override { return "Guided"; }
    const char *name4() const override { return "GUID"; }
    Mode::Number number() const override { return Mode::Number::GUIDED; }

    autopilot_yaw_mode get_default_auto_yaw_mode(bool rtl) const;

private:
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_takeoff_run();
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
};



class ModeAuto : public ModeGuided
{

public:
    // inherit constructor
    using ModeGuided::ModeGuided;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool auto_loiter_start();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location& dest_loc);
    void auto_circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void auto_circle_start();
    void auto_nav_guided_start();
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    void set_yaw_rate(float turn_rate_dps);
    bool auto_terrain_recover_start();

protected:

    const char *name() const override { return "Auto"; }
    const char *name4() const override { return "AUTO"; }
    Mode::Number number() const override { return Mode::Number::AUTO; }

private:
    void auto_wp_run();
    void auto_circle_run();
    void auto_nav_guided_run();
    void auto_loiter_run();
    void auto_terrain_recover_run();
};


class ModePoshold : public ModeAlthold
{

public:
    // inherit constructor
    using ModeAlthold::ModeAlthold;

    virtual void run() override;

    bool init(bool ignore_checks) override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Position Hold"; }
    const char *name4() const override { return "POSH"; }
    Mode::Number number() const override { return Mode::Number::POSHOLD; }

private:

    void control_horizontal();
};


class ModeCircle : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Circle"; }
    const char *name4() const override { return "CIRC"; }
    Mode::Number number() const override { return Mode::Number::CIRCLE; }
};

class ModeSurface : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "Surface"; }
    const char *name4() const override { return "SURF"; }
    Mode::Number number() const override { return Mode::Number::SURFACE; }
    bool nobaro_mode;
};


class ModeMotordetect : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Motor Detection"; }
    const char *name4() const override { return "DETE"; }
    Mode::Number number() const override { return Mode::Number::MOTOR_DETECT; }
};


// DYNAMIC: accepts the same external commands as GUIDED, but always holds
// position (and depth, or range to the bottom) when no command is active, and
// holds a commanded roll/pitch attitude while doing so.
class ModeDynamic : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }

    // same targets as ModeGuided (NEU frame, cm and cm/s, relative to the EKF origin)
    bool dynamic_set_destination(const Vector3f& destination);
    bool dynamic_set_destination(const Location& dest_loc);
    bool dynamic_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void dynamic_set_velocity(const Vector3f& velocity);
    void dynamic_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    bool dynamic_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    bool dynamic_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void dynamic_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);

    // set roll and pitch targets from an attitude quaternion (yaw is ignored),
    // plus optional climb rate and yaw rate
    void dynamic_set_angle(const Quaternion &q, bool use_attitude, bool use_climb_rate, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_cds);

    // bottom tracking: hold a range above the seafloor using the downward rangefinder
    bool bottom_track_active() const { return bt_active; }
    float get_bottom_track_target_cm() const;
    bool set_bottom_track_target_cm(float range_cm);
    bool dynamic_set_destination_NE_range(const Vector2f& destination_ne_cm, float range_cm);

    // path following: fly the uploaded mission's waypoints with AC_WPNav, then hold
    bool start_path(uint16_t first_index);
    void stop_path(const char *reason);
    bool path_active() const { return path_running; }
    int16_t get_path_index() const { return path_running ? int16_t(path_index) : -1; }

protected:

    const char *name() const override { return "Dynamic"; }
    const char *name4() const override { return "DYNM"; }
    Mode::Number number() const override { return Mode::Number::DYNAMIC; }

private:

    enum class SubMode : uint8_t {
        POSITION,   // move to and hold a position target
        VELOCITY,   // follow a velocity target, holding position once it stops
        POSVEL,     // follow a position target moving at a velocity
    };

    bool check_destination(const Vector3f& destination) const;
    void set_horizontal_target(SubMode submode, const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms);
    void set_vertical_target(SubMode submode, float pos_d_m, float vel_d_ms);
    void init_controllers();
    void update_horizontal(uint32_t now_ms, uint32_t timeout_ms);
    void update_vertical(uint32_t now_ms, uint32_t timeout_ms);
    void update_bottom_track();
    void update_pilot_translation();
    void run_path();
    bool path_next_leg();
    void path_hold_here(bool at_destination);
    void update_yaw(float &target_yaw_rate_cds);
    void output_thrust();
    bool timed_out(uint32_t update_ms, uint32_t now_ms, uint32_t timeout_ms) const;
    bool gcs_velocity_active() const;

    SubMode horiz_submode = SubMode::POSITION;
    SubMode vert_submode = SubMode::POSITION;

    // horizontal targets (NED frame, m and m/s)
    Vector2p pos_target_ne_m;
    Vector2f vel_target_ne_ms;
    uint32_t horiz_update_ms = 0;

    // vertical targets (NED frame, m and m/s). When bottom tracking is active,
    // pos_target_d_m is relative to the tracked seafloor (i.e. minus the range).
    float pos_target_d_m = 0.0f;
    float vel_target_d_ms = 0.0f;
    uint32_t vert_update_ms = 0;
    bool climb_from_thrust = false;

    // roll and pitch targets in centidegrees
    float roll_target_cd = 0.0f;
    float pitch_target_cd = 0.0f;
    uint32_t att_update_ms = 0;

    uint32_t yaw_rate_update_ms = 0;
    bool pilot_yawing = false;
    bool pilot_horizontal = false;  // forward/lateral stick deflected last loop
    bool pilot_vertical = false;    // throttle stick deflected last loop

    // GCS velocity commands take priority over the joystick sticks while
    // they keep arriving (the control app's gamepad streams them)
    static constexpr uint32_t GCS_VELOCITY_PRIORITY_MS = 500;
    uint32_t gcs_velocity_ms = 0;

    bool bt_active = false;

    // path following state
    bool path_running = false;
    uint16_t path_index = 0;        // mission index of the current leg's waypoint
    uint16_t path_hold_s = 0;       // seconds to wait at the current waypoint
    uint32_t path_reached_ms = 0;   // when the current waypoint was reached, 0 if not yet
};
