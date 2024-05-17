/*
  CINS state estimator for AP_AHRS, developed by Mr Patrick Wiltshire and Dr Pieter Van Goor 
 */

#include "AP_CINS.h"
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// gains tested for 5Hz GPS
#define CINS_GAIN_GPSPOS_ATT (1.0E-4)
#define CINS_GAIN_GPSVEL_ATT (1.0E-4)

// Gains for new CINS method
#define CINS_GAIN_GPSPOS_POS (10.0)
#define CINS_GAIN_GPSVEL_VEL (10.0)
#define CINS_GAIN_MAG (5.0)
#define CINS_GAIN_Q11 (0.01)
#define CINS_GAIN_Q22 (0.001)
// Gains for the Bias
#define CINS_GAIN_POS_BIAS (1E-6)
#define CINS_GAIN_VEL_BIAS (1E-6)
#define CINS_GAIN_MAG_BIAS (0.005)
#define CINS_SAT_BIAS (0.0873) // 0.0873 rad/s is 5 deg/s
// Naive GPS Time Delay settings
#define CINS_GPS_DELAY (0.1) // seconds of delay


// table of user settable parameters
const AP_Param::GroupInfo AP_CINS::var_info[] = {

    // @Param: GPATT
    // @DisplayName: CINS GPS position attitude gain
    // @Description: CINS GPS position attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GPATT", 1, AP_CINS, gains.gpspos_att, CINS_GAIN_GPSPOS_ATT),

    // @Param: GVATT
    // @DisplayName: CINS GPS velocity attitude gain
    // @Description: CINS GPS velocity attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GVATT", 2, AP_CINS, gains.gpsvel_att, CINS_GAIN_GPSVEL_ATT),

    // @Param: GPLAG
    // @DisplayName: CINS GPS lag
    // @Description: CINS GPS lag
    // @Range: 0.0 0.2
    // @User: Advanced
    // @Units: s
    AP_GROUPINFO("GPLAG", 3, AP_CINS, gains.gps_lag, CINS_GPS_DELAY),

    // @Param: GPPOS
    // @DisplayName: CINS GPS position gain
    // @Description: CINS GPS position gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPPOS", 4, AP_CINS, gains.gps_pos, CINS_GAIN_GPSPOS_POS),

    // @Param: GPVEL
    // @DisplayName: CINS GPS velocity gain
    // @Description: CINS GPS velocity gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPVEL", 5, AP_CINS, gains.gps_vel, CINS_GAIN_GPSVEL_VEL),

    // @Param: MGATT
    // @DisplayName: CINS magnetometer attitude gain
    // @Description: CINS magnetometer (compass) attitude gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("MGATT", 6, AP_CINS, gains.mag_att, CINS_GAIN_MAG),
    
    // @Param: INTQV
    // @DisplayName: CINS internal velocity Q gain
    // @Description: CINS internal velocity Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQV", 7, AP_CINS, gains.Q11, CINS_GAIN_Q11),

    // @Param: INTQP
    // @DisplayName: CINS internal position Q gain
    // @Description: CINS internal position Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQP", 8, AP_CINS, gains.Q22, CINS_GAIN_Q22),

    // @Param: GPSPB
    // @DisplayName: CINS GPS position - gyro bias gain
    // @Description: CINS GPS position - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPSPB", 9, AP_CINS, gains.gps_pos_bias, CINS_GAIN_POS_BIAS),
    
    // @Param: GPSVB
    // @DisplayName: CINS GPS velocity - gyro bias gain
    // @Description: CINS GPS velocity - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPSVB", 10, AP_CINS, gains.gps_vel_bias, CINS_GAIN_VEL_BIAS),

    // @Param: MBIAS
    // @DisplayName: CINS magnetometer bias gain
    // @Description: CINS magnetometer (compass) bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MBIAS", 11, AP_CINS, gains.mag_bias, CINS_GAIN_MAG_BIAS),

    // @Param: SATBS
    // @DisplayName: CINS gyro bias saturation
    // @Description: CINS gyro bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATBS", 12, AP_CINS, gains.sat_bias, CINS_SAT_BIAS),
    
    AP_GROUPEND
};

Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt);

// constructor
AP_CINS::AP_CINS(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = Gal3F::identity();
    state.ZHat = SIM23::identity();

    state.gyro_bias.zero();
    state.bias_gain_mat.rot.zero();
    state.bias_gain_mat.vel.zero();
    state.bias_gain_mat.pos.zero();
}
/*
  update function, called at loop rate
 */
void AP_CINS::update(void)
{
    auto &dal = AP::dal();

    if (!done_yaw_init) {
        done_yaw_init = init_yaw();
    }

    const auto &ins = dal.ins();

    // Get delta angle and convert to gyro rad/s
    const uint8_t gyro_index = ins.get_primary_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!ins.get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
        return;
    }
    // turn delta angle into a gyro in radians/sec
    const Vector3F gyro = (delta_angle / dangle_dt).toftype();

    // get delta velocity and convert to accel m/s/s
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3F accel = (delta_velocity / dvel_dt).toftype();

    if (done_yaw_init && state.have_origin){
        update_imu(gyro, accel, dangle_dt);
    }

    // see if we have new GPS data
    const auto &gps = dal.gps();
    if (gps.status() >= AP_DAL_GPS::GPS_OK_FIX_3D) {
        const uint32_t last_gps_fix_ms = gps.last_message_time_ms(0);
        if (last_gps_update_ms != last_gps_fix_ms) {
            // don't allow for large gain if we lose and regain GPS
            const float gps_dt = MIN((last_gps_fix_ms - last_gps_update_ms)*0.001, 1);
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;
            }
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);

            update_gps(pos.toftype(), vel.toftype(), gps_dt);
        }
    }

    update_attitude_from_compass();

    // Update GNSS delay buffers
    buffers.internal_time += dangle_dt;
    buffers.stamped_pos.push_front({buffers.internal_time, state.XHat.pos()});
    while (!buffers.stamped_pos.empty() && buffers.internal_time - buffers.stamped_pos.back().first > gains.gps_lag){
        buffers.stamped_pos.pop_back();
    }
    buffers.stamped_vel.push_front({buffers.internal_time, state.XHat.vel()});
    while (!buffers.stamped_pos.empty() && buffers.internal_time - buffers.stamped_vel.back().first > gains.gps_lag){
        buffers.stamped_vel.pop_back();
    }



    // Write logging messages

    // @LoggerMessage: CINS
    // @Description: CINS state
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PN: position north
    // @Field: PE: position east
    // @Field: PD: position down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();

    AP::logger().WriteStreaming("CINS", "TimeUS,I,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Lat,Lon,Alt",
                                "s#dddnnnmmmDUm",
                                "F-000000000GG0",
                                "QBfffffffffLLf",
                                dal.micros64(),
                                DAL_CORE(0),
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                wrap_360(degrees(yaw_rad)),
                                state.XHat.vel().x,
                                state.XHat.vel().y,
                                state.XHat.vel().z,
                                state.XHat.pos().x,
                                state.XHat.pos().y,
                                state.XHat.pos().z,
                                loc.lat,
                                loc.lng,
                                loc.alt*0.01);

    // @LoggerMessage: CIN2
    // @Description: Extra CINS states
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: GX: Gyro Bias X
    // @Field: GY: Gyro Bias Y
    // @Field: GZ: Gyro Bias Z
    // @Field: AVN: auxiliary velocity north
    // @Field: AVE: auxiliary velocity east
    // @Field: AVD: auxiliary velocity down
    // @Field: APN: auxiliary position north
    // @Field: APE: auxiliary position east
    // @Field: APD: auxiliary position down
    AP::logger().WriteStreaming("CIN2", "TimeUS,I,GX,GY,GZ,AVN,AVE,AVD,APN,APE,APD",
                                "s#kkknnnmmm",
                                "F-000000000",
                                "QBfffffffff",
                                dal.micros64(),
                                DAL_CORE(0),
                                degrees(state.gyro_bias.x),
                                degrees(state.gyro_bias.y),
                                degrees(state.gyro_bias.z),
                                state.ZHat.W1().x,
                                state.ZHat.W1().y,
                                state.ZHat.W1().z,
                                state.ZHat.W2().x,
                                state.ZHat.W2().y,
                                state.ZHat.W2().z);
}


/*
  update on new GPS sample
 */
void AP_CINS::update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt)
{
    // Forward the GPS to the current time
    const Vector3F undelayed_pos = buffers.stamped_pos.empty() && gains.gps_lag > 0. ? pos : pos + state.XHat.pos() - buffers.stamped_pos.back().second;
    const Vector3F undelayed_vel = buffers.stamped_vel.empty() && gains.gps_lag > 0. ? vel : vel + state.XHat.vel() - buffers.stamped_vel.back().second;


    //compute correction terms 
    update_states_gps_cts(undelayed_pos, undelayed_vel, gps_dt);
    // update_states_gps(pos, vel, gps_dt);

    // use AHRS3 for debugging
    ftype roll_rad, pitch_rad, yaw_rad;
    const auto rot = state.XHat.rot() * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body().toftype();
    rot.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();
    const mavlink_ahrs3_t pkt {
    roll : float(roll_rad),
    pitch : float(pitch_rad),
    yaw : float(yaw_rad),
    altitude : float(-state.XHat.pos().z),
    lat: loc.lat,
    lng: loc.lng,
    v1 : 0,
    v2 : 0,
    v3 : 0,
    v4 : 0
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_AHRS3, (const char *)&pkt);
}


/*
  update from IMU data
 */
void AP_CINS::update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt)
{
    //Integrate Dynamics using the Matrix exponential 
    //Create Zero vector 
    Vector3F zero_vector;
    zero_vector.zero();
    Vector3F gravity_vector = Vector3F(0,0,GRAVITY_MSS);

    // Update the bias gain matrices
    // In mathematical terms, \dot{M} = B = - Ad_{Z^{-1} \hat{X}} [I_3;0_3;0_3].
    // Here, we split B into its parts and add them to the parts of the bias gain matrix.
    if (done_yaw_init) {
        const SIM23 XInv_Z = SIM23(state.XHat.inverse()) * state.ZHat;
        state.bias_gain_mat.rot += -state.XHat.rot() * dt;
        state.bias_gain_mat.vel += Matrix3F::skew_symmetric(XInv_Z.W1()) * XInv_Z.R().transposed() * dt;
        state.bias_gain_mat.pos += Matrix3F::skew_symmetric(XInv_Z.W2()) * XInv_Z.R().transposed() * dt;
    }

    const Gal3F leftMat = Gal3F::exponential(zero_vector, zero_vector, gravity_vector*dt, -dt);
    const Gal3F rightMat = Gal3F::exponential((gyro_rads-state.gyro_bias)*dt, zero_vector, accel_mss*dt, dt);
    //Update XHat (Observer Dynamics)
    state.XHat = leftMat * state.XHat * rightMat;

    //Update ZHat (Auxilary Dynamics)
    SIM23 Gamma_IMU_inv = SIM23::identity();
    const GL2 S_Gamma = 0.5 * state.ZHat.A().transposed() * GL2(gains.Q11, 0., 0., gains.Q22) * state.ZHat.A();
    Gamma_IMU_inv.A() = GL2::identity() - dt * S_Gamma;

    // Update the bias gain with Gamma
    compute_bias_update_imu(Gamma_IMU_inv.inverse());

    state.ZHat = SIM23(leftMat) * state.ZHat * Gamma_IMU_inv;
}


/*
Compute correction terms when a GPS signal is received. This code generates corrections terms
Delta and Gamma which are then applied to the estimated state XHat and auxiliary state ZHat, respectively.
This allows the update_gps to output corrected estimates for attitude, velocity (NED) and position. 
*/
void AP_CINS::update_states_gps(const Vector3F &pos_tru, const Vector3F &vel_tru, const ftype gps_dt)
{
    const SIM23 ZInv = state.ZHat.inverse();
    const Vector3F& pos_est = state.XHat.pos();
    const Vector3F& pos_ZInv = ZInv.W2();

    // TODO: Current version does NOT use the velocity.

    // Gamma: Correction term to apply to the auxiliary state ZHat
    const Vector2F C = Vector2F(0,1); // (0, 1) for position measurements.
    const GL2 CCT = GL2(0,0,0,1.);
    const Vector2F gains_Gamma = gains.gps_pos * ZInv.A() * C;

    Matrix3F R_gamma;
    R_gamma.identity();
    const GL2 A_Gamma = GL2::identity() + 0.5 * gps_dt * gains.gps_pos * ZInv.A() * CCT * ZInv.A().transposed();
    const Vector3F V_Gamma_1 = (pos_tru + pos_ZInv) * gains_Gamma.x * gps_dt;
    const Vector3F V_Gamma_2 = (pos_tru + pos_ZInv) * gains_Gamma.y * gps_dt;

    const SIM23 Gamma_inv = SIM23(R_gamma, V_Gamma_1, V_Gamma_2, A_Gamma);

    // Compute the bias correction
    const Vector3F bias_correction = compute_bias_update_gps(Gamma_inv.inverse(), pos_tru, vel_tru, gps_dt);
    state.gyro_bias += bias_correction * gps_dt;
    const Gal3F Delta_bias(
        Matrix3F::from_angular_velocity(-state.bias_gain_mat.rot * bias_correction * gps_dt),
        -state.bias_gain_mat.pos * bias_correction * gps_dt,
        -state.bias_gain_mat.vel * bias_correction * gps_dt,
        0.
    );


    // Delta: Correction term to apply to the estimated state XHat
    Matrix3F R_delta;
    const Vector3F maucross_mauhat = Matrix3F::skew_symmetric(pos_tru + pos_ZInv) * (pos_est + pos_ZInv);
    const ftype mautrans_mauhat = (pos_tru + pos_ZInv) * (pos_est + pos_ZInv); //Dotproduct 
    const ftype norm_maucross_mauhat = maucross_mauhat.length();    
    if (fabsF(norm_maucross_mauhat) > 0.00001f){
        ftype psi = atan2F(norm_maucross_mauhat, mautrans_mauhat);
        R_delta = Matrix3F::from_angular_velocity(maucross_mauhat * ((-gains.gpspos_att*psi*gps_dt )/norm_maucross_mauhat));
    } else {
        R_delta.identity();
    }

    const Vector3F pre_V_Delta = (pos_tru + pos_ZInv) - R_delta * (pos_est + pos_ZInv);
    const Vector2F gains_Delta = A_Gamma.inverse().transposed() * gains_Gamma;
    const Vector3F V_Delta_1 = pre_V_Delta * gains_Delta.x * gps_dt;
    const Vector3F V_Delta_2 = pre_V_Delta * gains_Delta.y * gps_dt;

    SIM23 Delta_SIM23 = Gal3F(R_delta, V_Delta_2, V_Delta_1, 0.0f);
    Delta_SIM23 = state.ZHat * Delta_bias * Delta_SIM23 * ZInv;
    const Gal3F Delta = Gal3F(Delta_SIM23.R(), Delta_SIM23.W2(), Delta_SIM23.W1(), 0.);

    // Update the states using the correction terms
    state.XHat = Delta * state.XHat;
    state.ZHat = state.ZHat * Gamma_inv;
}


/*
Compute correction terms when a GPS signal is received. This code generates corrections terms
Delta and Gamma which are then applied to the estimated state XHat and auxiliary state ZHat, respectively.
This allows the update_gps to output corrected estimates for attitude, velocity (NED) and position. 

This is the continuous-time version, discretised through Euler integration.
*/
void AP_CINS::update_states_gps_cts(const Vector3F &pos_tru, const Vector3F &vel_tru, const ftype gps_dt)
{
    const SIM23 ZInv = state.ZHat.inverse();
    const Vector3F& pos_est = state.XHat.pos();
    const Vector3F& pos_ZInv = ZInv.W2();
    const Vector3F& vel_est = state.XHat.vel();
    const Vector3F& vel_ZInv = ZInv.W1();

    // Gamma: Correction term to apply to the auxiliary state ZHat
    const Vector2F C_pos = Vector2F(0,1); // e2 for position measurements.
    const GL2 CCT_pos = GL2(0,0,0,1.);
    const Vector2F gains_AC_pos = ZInv.A() * C_pos;
    const Vector2F C_vel = Vector2F(1,0); // e1 for velocity measurements.
    const GL2 CCT_vel = GL2(1.,0,0,0);
    const Vector2F gains_AC_vel = ZInv.A() * C_vel;

    Matrix3F eye3;
    eye3.identity();
    const GL2 S_Gamma = -0.5 * (gains.gps_pos+gains.gpspos_att) * ZInv.A() * CCT_pos * ZInv.A().transposed() 
                        -0.5 * (gains.gps_vel+gains.gpsvel_att) * ZInv.A() * CCT_vel * ZInv.A().transposed();
    const Vector3F W_Gamma_1 = - (pos_tru + pos_ZInv) * gains_AC_pos.x * (gains.gps_pos+gains.gpspos_att)
                               - (vel_tru + vel_ZInv) * gains_AC_vel.x * (gains.gps_vel+gains.gpsvel_att);
    const Vector3F W_Gamma_2 = - (pos_tru + pos_ZInv) * gains_AC_pos.y * (gains.gps_pos+gains.gpspos_att)
                               - (vel_tru + vel_ZInv) * gains_AC_vel.y * (gains.gps_vel+gains.gpsvel_att);

    const SIM23 Gamma_inv = SIM23(eye3, -W_Gamma_1*gps_dt, -W_Gamma_2*gps_dt, GL2::identity() - gps_dt*S_Gamma);

    // const SIM23 Gamma_inv = SIM23(eye3, -W_Gamma_1*gps_dt, -W_Gamma_2*gps_dt, GL2::exponential(- gps_dt*S_Gamma));

    // State Estimate Correction
    // Vector3F Omega_Delta = (Matrix3F::skew_symmetric(pos_est + pos_ZInv) * (pos_tru + pos_ZInv)) * 4.*gains.gpspos_att
                        //  + (Matrix3F::skew_symmetric(vel_est + vel_ZInv) * (vel_tru + vel_ZInv)) * 4.*gains.gpsvel_att;
    Vector3F Omega_Delta = computeRotationCorrection((pos_est + pos_ZInv), -(pos_tru + pos_ZInv), 4.*gains.gpspos_att, gps_dt)
                         + computeRotationCorrection((vel_est + vel_ZInv), -(vel_tru + vel_ZInv), 4.*gains.gpsvel_att, gps_dt);

    Vector3F W_Delta1 = (pos_tru - pos_est) * gains_AC_pos.x * (gains.gps_pos+gains.gpspos_att)
                      + (vel_tru - vel_est) * gains_AC_pos.x * (gains.gps_vel+gains.gpsvel_att);
    Vector3F W_Delta2 = (pos_tru - pos_est) * gains_AC_pos.y * (gains.gps_pos+gains.gpspos_att)
                      + (vel_tru - vel_est) * gains_AC_pos.y * (gains.gps_vel+gains.gpsvel_att);

    // Add the bias correction
    const Vector3F bias_correction = compute_bias_update_gps(Gamma_inv.inverse(), pos_tru, vel_tru, gps_dt);
    state.gyro_bias += bias_correction * gps_dt;
    Omega_Delta += state.bias_gain_mat.rot * bias_correction;
    W_Delta1 += state.bias_gain_mat.pos * bias_correction;
    W_Delta2 += state.bias_gain_mat.vel * bias_correction;

    // Construct the correction term Delta
    SIM23 Delta_SIM23 = SIM23(Matrix3F::from_angular_velocity(Omega_Delta*gps_dt), W_Delta1*gps_dt, W_Delta2*gps_dt, GL2::identity());
    Delta_SIM23 = state.ZHat * Delta_SIM23 * ZInv;
    const Gal3F Delta = Gal3F(Delta_SIM23.R(), Delta_SIM23.W2(), Delta_SIM23.W1(), 0.);

    // Update the states using the correction terms
    state.XHat = Delta * state.XHat;
    state.ZHat = state.ZHat * Gamma_inv;
}

/*
  initialise yaw from compass, if available
 */
bool AP_CINS::init_yaw(void)
{
    ftype mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return false;
    }
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    state.XHat.rot().from_euler(roll_rad, pitch_rad, mag_yaw);
    
    return true;
}

/*
  get yaw from compass
 */
bool AP_CINS::get_compass_yaw(ftype &yaw_rad, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    const auto &field = compass.get_field(mag_idx);
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const float declination_rad = dal.compass().get_declination();
    if (is_zero(declination_rad)) {
        // wait for declination
        return false;
    }

    // const float cos_pitch_sq = 1.0f-(state.XHat.rot().c.x*state.XHat.rot().c.x);
    // const float headY = field.y * state.XHat.rot().c.z - field.z * state.XHat.rot().c.y;

    // // Tilt compensated magnetic field X component:
    // const float headX = field.x * cos_pitch_sq - state.XHat.rot().c.x * (field.y * state.XHat.rot().c.y + field.z * state.XHat.rot().c.z);

    // return magnetic yaw

    yaw_rad = wrap_PI(-atan2f(field.y,field.x) + declination_rad);

    return true;
}


bool AP_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    mag_vec = compass.get_field(mag_idx).toftype();
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const Location loc = get_location();
    mag_ref = AP_Declination::get_earth_field_ga(loc).toftype();
    if (mag_ref.is_zero()) {
        // wait for declination
        return false;
    }

    return true;
}

void AP_CINS::update_attitude_from_compass() {
    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    // Convert mag measurement from milliGauss to Gauss
    mag_vec *= 1.e-3;
    // Vector3F Omega_Delta = - (Matrix3F::skew_symmetric(mag_ref) * state.XHat.rot() * mag_vec) * gains.mag_att;
    Vector3F Omega_Delta = computeRotationCorrection(mag_ref, state.XHat.rot()*mag_vec, gains.mag_att, dt);

    // Add the bias correction
    const Vector3F bias_correction = compute_bias_update_compass(mag_vec, mag_ref, dt);
    state.gyro_bias += bias_correction * dt;
    Omega_Delta += state.bias_gain_mat.rot * bias_correction;
    const Vector3F W_Delta1 = state.bias_gain_mat.pos * bias_correction;
    const Vector3F W_Delta2 = state.bias_gain_mat.vel * bias_correction;

    Matrix3F R_Delta = Matrix3F::from_angular_velocity(Omega_Delta*dt);

    SIM23 Delta = Gal3F(R_Delta, W_Delta2 * dt, W_Delta1 * dt, 0.0f);
    Delta = state.ZHat * Delta * state.ZHat.inverse();

    state.XHat = Gal3(Delta.R(), Delta.W2(), Delta.W1(), 0.) * state.XHat;
}

Vector3F AP_CINS::compute_bias_update_gps(const SIM23& Gamma, const Vector3F& pos_tru, const Vector3F& vel_tru,  const ftype& dt) {
    // Compute the bias update for GPS position

    // First compute the bias update delta_b and then update the bias gain matrices M

    // Construct the components of the 9x9 Adjoint matrix \Ad_Z
    Matrix3F I3; I3.identity();
    const SIM23 ZInv = state.ZHat.inverse();
    const Matrix3F AdZ_11 = state.ZHat.R();
    const Matrix3F AdZ_12{}; // Zero matrix
    const Matrix3F AdZ_13{}; // Zero matrix
    const Matrix3F AdZ_21 = - Matrix3F::skew_symmetric(ZInv.W1());
    const Matrix3F AdZ_22 = state.ZHat.R() * ZInv.A().a11();
    const Matrix3F AdZ_23 = state.ZHat.R() * ZInv.A().a21();
    const Matrix3F AdZ_31 = - Matrix3F::skew_symmetric(ZInv.W2());
    const Matrix3F AdZ_32 = state.ZHat.R() * ZInv.A().a12();
    const Matrix3F AdZ_33 = state.ZHat.R() * ZInv.A().a22();

    // The position and velocity measurements have both a measurement and gain matrix associated with them.
    // These matrices are labelled C for the measurement and K for the gain.

    // Position measurement matrix C1x
    const Matrix3F pre_C11 = - Matrix3F::skew_symmetric(state.XHat.pos());
    const Matrix3F pre_C12{};
    const Matrix3F pre_C13 = I3;

    const Matrix3F C11 = pre_C11 * AdZ_11 + pre_C12 * AdZ_21 + pre_C13 * AdZ_31;
    const Matrix3F C12 = pre_C11 * AdZ_12 + pre_C12 * AdZ_22 + pre_C13 * AdZ_32;
    const Matrix3F C13 = pre_C11 * AdZ_13 + pre_C12 * AdZ_23 + pre_C13 * AdZ_33;

    const Matrix3F K11 = Matrix3F::skew_symmetric(state.XHat.pos() + ZInv.W2()) * 4. * gains.gpspos_att * dt;
    const Matrix3F K21 = I3 * ZInv.A().a12() * (gains.gps_pos+gains.gpspos_att) * dt;
    const Matrix3F K31 = I3 * ZInv.A().a22() * (gains.gps_pos+gains.gpspos_att) * dt;

    // Velocity measurement matrix C2x
    const Matrix3F pre_C21 = - Matrix3F::skew_symmetric(state.XHat.vel());
    const Matrix3F pre_C22 = I3;
    const Matrix3F pre_C23{};

    const Matrix3F C21 = pre_C21 * AdZ_11 + pre_C22 * AdZ_21 + pre_C23 * AdZ_31;
    const Matrix3F C22 = pre_C21 * AdZ_12 + pre_C22 * AdZ_22 + pre_C23 * AdZ_32;
    const Matrix3F C23 = pre_C21 * AdZ_13 + pre_C22 * AdZ_23 + pre_C23 * AdZ_33;

    const Matrix3F K12 = Matrix3F::skew_symmetric(state.XHat.vel() + ZInv.W1()) * 4. * gains.gpsvel_att * dt;
    const Matrix3F K22 = I3 * ZInv.A().a11() * (gains.gps_vel+gains.gpsvel_att) * dt;
    const Matrix3F K32 = I3 * ZInv.A().a21() * (gains.gps_vel+gains.gpsvel_att) * dt;

    // Compute the bias correction using the C matrices.
    // The correction to bias is given by the formula delta_b = k_b (MC)^\top (y-\hat{y})
    const Matrix3F M1 = state.bias_gain_mat.rot;
    const Matrix3F M2 = state.bias_gain_mat.vel;
    const Matrix3F M3 = state.bias_gain_mat.pos;

    Vector3F bias_correction = (C11*M1 + C12*M2 + C13*M3).mul_transpose(pos_tru - state.XHat.pos()) * gains.gps_pos_bias
                                   + (C21*M1 + C22*M2 + C23*M3).mul_transpose(vel_tru - state.XHat.vel()) * gains.gps_vel_bias;
    saturate_bias(bias_correction, dt);

    // Compute the bias gain matrix updates
    // This implements the formula d/dt M = (A-KC)M.
    // The discretisation of this formula is simply thanks to Gamma already being in discrete-time form.

    const SIM23 GammaInv = Gamma.inverse();
    const Matrix3F A11 = Gamma.R();
    const Matrix3F A12{}; // Zero matrix
    const Matrix3F A13{}; // Zero matrix
    const Matrix3F A21 = - Matrix3F::skew_symmetric(GammaInv.W1());
    const Matrix3F A22 = Gamma.R() * GammaInv.A().a11();
    const Matrix3F A23 = Gamma.R() * GammaInv.A().a21();
    const Matrix3F A31 = - Matrix3F::skew_symmetric(GammaInv.W2());
    const Matrix3F A32 = Gamma.R() * GammaInv.A().a12();
    const Matrix3F A33 = Gamma.R() * GammaInv.A().a22();

    state.bias_gain_mat.rot = (A11 - K11 * C11 - K12 * C21) * M1 + (A12 - K11 * C12 - K12 * C22) * M2 + (A13 - K11 * C13 - K12 * C23) * M3;
    state.bias_gain_mat.vel = (A21 - K21 * C11 - K22 * C21) * M1 + (A22 - K21 * C12 - K22 * C22) * M2 + (A23 - K21 * C13 - K22 * C23) * M3;
    state.bias_gain_mat.pos = (A31 - K31 * C11 - K32 * C21) * M1 + (A32 - K31 * C12 - K32 * C22) * M2 + (A33 - K31 * C13 - K32 * C23) * M3;

    return bias_correction;
}


void AP_CINS::compute_bias_update_imu(const SIM23& Gamma) {
    // Compute the bias update for IMU inputs

    const SIM23 GammaInv = Gamma.inverse();
    const Matrix3F Ad_Gamma_11 = Gamma.R();
    const Matrix3F Ad_Gamma_12{}; // Zero matrix
    const Matrix3F Ad_Gamma_13{}; // Zero matrix
    const Matrix3F Ad_Gamma_21 = - Matrix3F::skew_symmetric(GammaInv.W1()) * Gamma.R();
    const Matrix3F Ad_Gamma_22 = Gamma.R() * GammaInv.A().a11();
    const Matrix3F Ad_Gamma_23 = Gamma.R() * GammaInv.A().a21();
    const Matrix3F Ad_Gamma_31 = - Matrix3F::skew_symmetric(GammaInv.W2()) * Gamma.R();
    const Matrix3F Ad_Gamma_32 = Gamma.R() * GammaInv.A().a12();
    const Matrix3F Ad_Gamma_33 = Gamma.R() * GammaInv.A().a22();

    const Matrix3F M1 = state.bias_gain_mat.rot;
    const Matrix3F M2 = state.bias_gain_mat.vel;
    const Matrix3F M3 = state.bias_gain_mat.pos;

    // Implement M(t+) = A M(t)
    state.bias_gain_mat.rot = Ad_Gamma_11 * M1 + Ad_Gamma_12 * M2 + Ad_Gamma_13 * M3;
    state.bias_gain_mat.vel = Ad_Gamma_21 * M1 + Ad_Gamma_22 * M2 + Ad_Gamma_23 * M3;
    state.bias_gain_mat.pos = Ad_Gamma_31 * M1 + Ad_Gamma_32 * M2 + Ad_Gamma_33 * M3;
}


Vector3F AP_CINS::compute_bias_update_compass(const Vector3F& mag_tru, const Vector3F& mag_ref, const ftype& dt) {
    // Compute the bias update for the compass

    const Vector3F yTilde_m = mag_tru - state.XHat.rot().mul_transpose(mag_ref);

    // First compute the bias update delta_b and then update the bias gain matrices M
    const Matrix3F AdZ_11 = state.ZHat.R();
    const Matrix3F pre_C11 = state.XHat.rot().transposed() * Matrix3F::skew_symmetric(mag_ref);
    const Matrix3F C11 = pre_C11 * AdZ_11;
    // The remaining C components are all zero in this case.


    // Implement delta_b = k_b M^T C^T (y-\hat{y})

    const Matrix3F M1 = state.bias_gain_mat.rot;
    Vector3F bias_correction = (C11*M1).mul_transpose(yTilde_m) * gains.mag_bias;
    saturate_bias(bias_correction, dt);


    // Compute the bias gain matrix updates

    const Matrix3F K11 = -Matrix3F::skew_symmetric(mag_ref) * state.XHat.rot() * gains.mag_att;

    // const Matrix3F minus_KC = - K11 * C11;
    // Matrix3F I3; I3.identity();
    // const Matrix3F exp_minus_KC = I3 + minus_KC * dt + minus_KC*minus_KC *(0.5*dt*dt) + minus_KC*minus_KC*minus_KC *(1./6.*dt*dt*dt);

    // Implement M(t+) = exp(-dt * KC)M(t). K21, K31 are zero matrices
    // state.bias_gain_mat.rot = exp_minus_KC * M1;
    state.bias_gain_mat.rot += (- K11 * C11) * M1;
    // state.bias_gain_mat.rot = (- K11 * C11) * M1;

    return bias_correction;
}

void AP_CINS::saturate_bias(Vector3F& bias_correction, const ftype& dt) const {
    // Ensure that no part of the bias exceeds the saturation limit
    if (abs(bias_correction.x) > 1E-8)
        bias_correction.x *= MIN(1., (gains.sat_bias - abs(state.gyro_bias.x)) / (dt * abs(bias_correction.x)));
    if (abs(bias_correction.y) > 1E-8)
        bias_correction.y *= MIN(1., (gains.sat_bias - abs(state.gyro_bias.y)) / (dt * abs(bias_correction.y)));
    if (abs(bias_correction.z) > 1E-8)
        bias_correction.z *= MIN(1., (gains.sat_bias - abs(state.gyro_bias.z)) / (dt * abs(bias_correction.z)));
}


Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt) {
    // The correction is given by Omega = - skew(v1) * v2 * gain, where v2 = R v0 for some const. v0.
    // It is applied as dR/dt = skew(Omega) R.
    // The key point is that Omega changes R which changes Omega.
    // We use a first order expansion to capture this effect here.

    const Vector3F omega1 = - Matrix3F::skew_symmetric(v1) * v2 * gain;
    const Vector3F omega2 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    const Vector3F omega3 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega2) * v2 * gain
        - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    return omega1 + omega2*dt + omega3*0.5*dt*dt;
    // return omega1;

}
