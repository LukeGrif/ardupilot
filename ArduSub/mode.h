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
        AUTOTUNE =     15,  // automatically tune the roll, pitch and yaw rate and angle gains
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

#if AUTOTUNE_ENABLED
/*
  wrapper class for AC_AutoTune
 */
class AutoTune : public AC_AutoTune_Multi
{
public:
    bool init() override;
    void run() override;

protected:
    bool position_ok() override;
    float get_desired_climb_rate_ms(void) const override;
    void get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads) override;
    void init_z_limits() override;
#if HAL_LOGGING_ENABLED
    void log_pids() override;
#endif
    // report each test's measurement so a GCS can graph the tune
    void do_post_test_gcs_announcements() override;

private:
    uint16_t test_count;
};

/*
  gentle tune for ROVs: each axis is moved through slow, small sine
  waves, a damping/restoring model is fitted to the response and the
  gains are calculated from the model
 */
class SubTune
{
public:
    SubTune();

    // do not allow copying
    CLASS_NO_COPY(SubTune);

    static const struct AP_Param::GroupInfo var_info[];

    // the gentle styles (1 and 2) run here; twitch (0) in AutoTune
    bool use_sine_style() const { return style >= 1; }

    bool init();
    void run();
    void stop();
    void disarmed(bool in_autotune_mode);
    void do_aux_function(RC_Channel::AuxSwitchPos ch_flag);

    // loop-rate diagnostic log for studying the tune (ATUN_DIAG)
    void log_diag() const;

private:
    enum class Axis : uint8_t { ROLL = 0, PITCH = 1, YAW = 2 };
    enum class Step : uint8_t { SETTLE, EXCITE, DONE };

    struct Gains {
        float rate_p, rate_i, rate_d, rate_ff;
        float angle_p;
        float accel_radss;
    };

    AP_Int8 style;
    AP_Int8 axes;
    AP_Float amplitude_deg;
    AP_Float period_s;
    AP_Int8 cycles;
    AP_Float bandwidth;
    AP_Int8 diag;
    AP_Float rate_dps;
    AP_Float inj_max;

    Step step;
    Axis axis;
    bool axis_done[3];
    bool have_tuned[3];
    Gains orig[3];
    Gains tuned[3];
    bool tuned_loaded;
    bool aux_used;

    uint32_t step_start_ms;
    uint32_t still_start_ms;
    float still_roll_rad, still_pitch_rad; // attitude when it last started being still
    float settle_peak_rate_rads;  // largest turn rate while waiting to settle
    uint32_t override_ms;
    uint32_t last_msg_ms;
    bool pilot_override;
    float hold_yaw_rad;

    // Fourier coefficients of the rate controller output and the body
    // rate at the two test frequencies, accumulated over whole periods
    float u_cos[2], u_sin[2];
    float rate_cos[2], rate_sin[2];
    float record_s;

    // model style (ATUN_STYLE 2): the test signal is added to the tested
    // axis's thruster output while the vehicle holds level, and
    //   acc = a*u + e*sign(u) - b*w - b2*|w|*w - c*sin(angle) + d
    // is fitted by least squares at the loop rate, for several thruster lags.
    // e is the thrust step left at zero output by an imperfect dead band
    // compensation (MOT_SPIN_MIN)
    static constexpr uint8_t MODEL_N = 6;       // a, e, b, b2, c, d
    static constexpr uint8_t MODEL_LAGS = 25;   // 0, 5, ... 120 ms
    static constexpr uint8_t MODEL_PACK = MODEL_N * (MODEL_N + 1) / 2;
    static constexpr uint8_t U_HIST = 64;       // filtered output history for the lags
    // upper triangle of the symmetric A'A, row by row
    static constexpr uint8_t pack(uint8_t i, uint8_t j) {
        return i <= j ? i * MODEL_N - i * (i - 1) / 2 + (j - i) : pack(j, i);
    }
    float ls_ata[MODEL_LAGS][MODEL_PACK];
    float ls_atb[MODEL_LAGS][MODEL_N];
    float ls_btb[MODEL_LAGS];
    float ls_sum_y;
    uint32_t ls_count;
    float u_hist[U_HIST];
    uint8_t u_hist_idx;
    uint8_t u_hist_len;
    float sg_hist[U_HIST];
    LowPassFilter2pFloat filt_u, filt_sg, filt_w, filt_q, filt_s;
    float prev_wf;
    float inj_amp;          // size of the injected signal
    bool inj_scaled;        // size set from the probe period
    float probe_w_sq;       // rate^2 summed over the probe period
    uint32_t probe_n;
    float rate_sq;          // rate^2 summed over the recording
    uint32_t rate_n;

    bool axis_weakened = false;   // tested axis's rate gains lowered for the test
    Gains pre_weaken;             // its gains before that
    uint8_t settle_halvings;      // times the rate gains were halved because the vehicle would not settle
    float model_dt;               // measured loop period for the model fit, s
    float ref_lag_sum;            // lags found on pitch and yaw, s: roll uses their mean
    uint8_t ref_lag_n;

    void reset_model();
    void weaken_axis_feedback();
    void halve_rate_gains();
    void restore_untuned();
    void restore_axis_feedback();
    void update_excite_model(uint32_t now_ms);
    void finish_axis_model();
    bool solve_model(uint8_t lag, float theta[MODEL_N], float &sse) const;

    AC_PID &rate_pid(Axis ax);
    AC_P &angle_p(Axis ax);
    const char *axis_name(Axis ax) const;
    Gains get_gains(Axis ax);
    void set_gains(Axis ax, const Gains &g);
    void load_gains(bool use_tuned);
    void save_gains();

    bool next_axis();
    void start_settle(uint32_t now_ms);
    void start_excite(uint32_t now_ms);
    void update_settle(uint32_t now_ms);
    void update_excite(uint32_t now_ms);
    void finish_axis();
    float excitation_rad(float t_s) const;
    void reset_sums();
    bool fit_model(float &a, float &b, float &c, float &fit) const;
};

// AUTOTUNE: tunes the roll, pitch and yaw rate and angle gains while
// holding depth and, when a position estimate is available, position
class ModeAutoTune : public Mode
{

    // ParametersG2 sets a pointer within our autotune objects:
    friend class ParametersG2;

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    void exit();

    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return false; }

    void disarmed(bool in_autotune_mode);
    void do_aux_function(RC_Channel::AuxSwitchPos ch_flag);

    // shared by both tune styles
    void init_position_hold();
    void control_horizontal();

    AutoTune autotune;
    SubTune subtune;

protected:

    const char *name() const override { return "AutoTune"; }
    const char *name4() const override { return "ATUN"; }
    Mode::Number number() const override { return Mode::Number::AUTOTUNE; }

private:
    bool using_sine = false;
};
#endif  // AUTOTUNE_ENABLED
