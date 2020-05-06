#include "Copter.h"
#include "mode.h"


bool ModePayloadRelease::init(bool ignore_checks)
{
    //initialise_initial_condition();
    //wind = ahrs.wind_estimate();
    if (!ignore_checks) {   //if not ignore checks
        return true;
    }

    return true;

}

void ModePayloadRelease::run()
{
    //this below 4 lines make sure that at background the copter is moving towards given waypoint
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    //copter.failsafe_terrain_set_status(wp_nav->update_wpnav()); //run way point controller
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);

}

void ModePayloadRelease::initialise_initial_condition() {
    //initialize all the values
    a = 1;  //sign changing variable
    vz = 0;
    az = g;
    z = 0;
    total_height = copter.current_loc.alt / 10; // divided by 10 because current altitude in cms.
    remaining_height = total_height;
    vx = AP::gps().ground_speed();
    ax = 0;
    x = 0;
    vrx = 0;
    v = 0;
    fd = 0;
    fdx = 0;
    fdz = 0;

    airspeed_uav = 5;

    llh_to_local(drop_point,drop_point_neu);
    //intialize wind values
    wind_speed_north = 100;
    wind_speed_east = 100;
    wind_speed_normalized = sqrt((wind_speed_east*wind_speed_east) + (wind_speed_north*wind_speed_north));
    //Perform initial calculation to initialize values
    theta = wrap_2PI((copter.flightmode->wp_bearing() / 100) * DEG_TO_RAD) ;  //calculate heading in radian
    phi = wrap_2PI(atan2f(wind_speed_east,wind_speed_north)); //wind vector direction

    if(phi > theta ) {
        C = phi - theta;
    }
    else if (phi < theta) {
        C = theta - phi;
    }

    if(fabs(theta - phi) <= 0.001) { //if theta == phi but for floating point
        if(C < M_PI ) {
            C = M_PI  - C;
            a = 1;
        }
        else {
            C = C - M_PI ;
            a = -1;
        }
        relative = sqrt((airspeed_uav*airspeed_uav) + (wind_speed_normalized*wind_speed_normalized) - 2 * airspeed_uav * wind_speed_normalized * cos(C));
        dirn = acos(((relative*relative) + (airspeed_uav*airspeed_uav) - (wind_speed_normalized *wind_speed_normalized))/(2 * relative * airspeed_uav));
    }
    else {
        dirn = theta;
    }

}

void ModePayloadRelease::calculate_displacement() {
    while (remaining_height > 0.01) {
        //calculation along z direction in NED frame
        vz = vz + az * dt;
        z = z + vz * dt;

        //calculation of wind velocity according to variation in height
        vw = wind_speed_normalized*(logf(remaining_height/z_0)/logf(total_height/z_0));

        //calculation along displacement vector i.e in NE plane
        vx = vx + ax * dt;
        x = x + vx * dt;
        vrx = vx + vw;

        //calculation of drag force
        v = sqrt(vz * vz + vrx * vrx);
        fd = 0.5 * rho * cd * Area * v * v;
        fdx = fd * vrx /v;
        fdz = fd * vz / v;

        //calculation of acceleration
        az = g - fdz / m;
        ax = -fdx / m;
        remaining_height = remaining_height - z;
    }
}

void ModePayloadRelease::calculate_release_point() {
    //If the relative payload path is same as uav path
    if(fabs(theta - phi) <= 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(dirn);
        release_point_neu.z = drop_point_neu.z;
    }
    //If the relative payload path is above the uav path
    else if(theta - phi > 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(theta + a * dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(theta + a * dirn);
        release_point_neu.z = drop_point_neu.z;
    }
    //If the relative payload path is below the uav path OR no wind is blowing
    else if((phi - theta > 0.001) || phi <= 0.001) {
        release_point_neu.x = drop_point_neu.x - x * cos(theta - a * dirn);
        release_point_neu.y = drop_point_neu.y - x * sin(theta - a * dirn);
        release_point_neu.z = drop_point_neu.z;
    }
}

void ModePayloadRelease::llh_to_local(Location &current_llh, Vector3d &current_neu) {
    Vector3d v_current_llh;
    //convert Location class to Vector3d structure form
    v_current_llh.x = current_llh.lat * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.y = current_llh.lng * 1.0e-7f * DEG_TO_RAD;
    v_current_llh.z = current_llh.alt * 1.0e-2f; //in meters
    //convert to ecef and store in current_neu
    wgsllh2ecef(v_current_llh,current_neu);
}

void ModePayloadRelease::local_to_llh(Vector3d &current_neu, Location &current_llh) {
    Vector3d v_current_llh;
    //convert current ecef to lng,lat,alt.
    wgsecef2llh(current_neu, v_current_llh);
    //store vector structure to location class

    current_llh.lat = v_current_llh.x * RAD_TO_DEG * 1.0e7f;
    current_llh.lng = v_current_llh.y * RAD_TO_DEG * 1.0e7f;
    current_llh.alt = drop_point.alt; //same altitude as drop_point altitude
}


void ModePayloadRelease::update_releasepoint() {
    if(state() == PayloadRelease_Start) {
        if(!calculated && AP::gps().ground_speed() > 3.5) {
            gcs().send_text(MAV_SEVERITY_INFO, "drop lat = %d, drop lon = %d,drop ht =%d",drop_point.lat,drop_point.lng,drop_point.alt);
            //gcs().send_text(MAV_SEVERITY_INFO, "inside here");
            initialise_initial_condition();
            //gcs().send_text(MAV_SEVERITY_INFO, "drop N = %f, drop E = %f",drop_point_neu.x,drop_point_neu.y);
            calculate_displacement();
            gcs().send_text(MAV_SEVERITY_INFO, "displacement = %f",x);
            calculate_release_point();
            local_to_llh(release_point_neu,release_point);
            calculated = true;
            //gcs().send_text(MAV_SEVERITY_INFO, "rel N = %f, rel E = %f",release_point_neu.x,release_point_neu.y);
            gcs().send_text(MAV_SEVERITY_INFO, "rel lat = %d, rel lon = %d,rel ht = %d",release_point.lat,release_point.lng,release_point.alt);
            if (!wp_nav->set_wp_destination(release_point)) {
                // failure to set destination can only be because of missing terrain data
                copter.failsafe_terrain_on_event();
                return ;
            }
        }
    }
}