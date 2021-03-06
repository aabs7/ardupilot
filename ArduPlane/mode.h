#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>

class Mode
{
public:

    /* Do not allow copies */
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
        QAUTOTUNE     = 22,
        QACRO         = 23,
        // added
        PAYLOADRELEASE = 24,
        // add finish
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }
};


class ModeAcro : public Mode
{
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

//added
class ModePayloadRelease : public Mode
{

public:
    ModePayloadRelease()
    {
        // gcs().send_text(MAV_SEVERITY_INFO, "mode Payloadrelease constructor");

        is_payload_released = false;
        g = 9.8;
        m = 1;  //mass
        Area = 0.015;
        rho = 1.2;
        dt = 0.01;
        cd = 1.2;
        z_0 = 0.1;
        time_delay = 2;
        a = 1;  //sign changing variable
        vz = 0;
        az = g;
        z = 0;
        total_height = 0;
        remaining_height = 0;
        vx = 0;
        ax = 0;
        x = 0;
        vrx = 0;
        v = 0;
        fd = 0;
        fdx = 0;
        fdz = 0;
        calculated = false;
        intermediate_distance = 100;
        radius = 50;
        //servo channel to release payload
        channel_payload = 6;
    }
    // This is for payload release state. 
    enum PayloadReleaseState{
        PayloadRelease_NotStarted,
        PayloadRelease_Start,
        PayloadRelease_Intermediate_point_reached,
        PayloadRelease_Loiter,
        PayloadRelease_Loiter_complete,
        PayloadRelease_Release_point_reached,
        PayloadRelease_Finish
    };
    //point where payload should be dropped
    Location drop_point;
    Vector3d drop_point_neu;
    //point where payload should be released to reach drop point
    Location release_point;
    Vector3d release_point_neu;

    // These variables are for intermediate point 
    /*
    before going to release point, plane should go to intermediate point
    so that it travels from intermediate point to release point parallel
    with the path to drop payload
    */
    Location int_point;
    bool is_int_reached;
    int intermediate_distance;
    
    Number mode_number() const override { return Number::PAYLOADRELEASE; }
    const char *name() const override { return "PAYLOADRELEASE"; }
    const char *name4() const override { return "PAYR"; }
    

    // methods that affect movement of the vehicle in this mode
    void update() override;    
    void update_releasepoint();
    void release_payload();
    void set_state(PayloadReleaseState state){_state = state;}
    PayloadReleaseState get_state() {return _state;}
    void set_loiter_parameters();
    

protected:

    bool _enter() override;
    void _exit() override;
private:

    bool is_payload_released;
    PayloadReleaseState _state = PayloadRelease_NotStarted;
    int radius; //radius to loiter around intermediate point

    uint8_t g; //acceleration due to gravity
    uint8_t m; //mass of payload
    float Area;    //Cross sectional area
    float rho;  //air density
    float dt;   //freefall prediction timestamp
    float cd;   //coefficent of drag
    float z_0;  //surface roughness
    float time_delay;   //time to open payload mechanism
    uint8_t a ; //sign changing value

    float vz ;  //velocity in z direction
    float az ;   //acceleration in z direction
    float z ;    //intermediate height calculated
    float total_height ;    //height over target
    float remaining_height;   //remaining height over target
    float vw;    //velocity of wind
    float vx ;   //velocity in x direction
    float ax ;   //acceleration in x direction
    float x ;   //displacement in NE plane in NED frame
    float vrx;  //velocity in x direction with relative to wind

    float v ;   //relative speed vector
    float fd;   //drag force
    float fdx;  //drag force in x direction
    float fdz;  //drag force in z direction

    bool calculated;

    Vector3f wind;
    int wind_speed_north;
    int wind_speed_east;
    float wind_speed_normalized;
    int airspeed_uav;
    
    float theta,phi,C,dirn; //angles
    float relative; //relative direction vector in which payload and parachute will fall 

    uint8_t channel_payload;

    // Functions used

    void llh_to_ecef(Location &current_llh, Vector3d &current_neu);
    void ecef_to_llh(Vector3d &current_neu, Location &current_llh);
    void llh_to_neu(Location &current_llh, Vector3d &current_neu);
    void neu_to_llh(Vector3d &current_neu, Location &current_llh);
    void get_intermediate_point(Vector3d RP);
    void calculate_release_point();
    
    bool verify_loiter_complete_heading(bool init);
    void initialise_initial_condition();
    void calculate_displacement();

};
//add finish

class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:
    bool _enter() override;
    
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

protected:

    bool _enter() override;
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool _enter() override;

protected:

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

protected:

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACO"; }
    const char *name4() const override { return "QACRO"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];
    
protected:
    AP_Int16 target_alt;
    AP_Int16 target_dist;
    AP_Int16 level_alt;
    AP_Int8 level_pitch;

    bool takeoff_started;
    Location start_loc;

    bool _enter() override;
};
