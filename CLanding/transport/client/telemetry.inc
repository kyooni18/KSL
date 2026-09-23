static bool refresh_medium(KrpcCNanoClient *client,unsigned airbrake_group,char *error,size_t error_size){
    float speed_of_sound=(float)client->cached_speed_of_sound;
    float static_pressure=(float)client->cached_static_pressure;
    float g_force=(float)client->cached_g_force;
    float dry_mass=(float)client->cached_dry_mass;
    float available_thrust=(float)client->cached_available_thrust;
    float current_thrust=(float)client->cached_current_thrust;
    krpc_tuple_tuple_double_double_double_tuple_double_double_double_t torque={0};
    krpc_tuple_double_double_double_t inertia={0},lift={0},drag={0},com={0},com_root={0};
    bool gear=client->cached_gear,brakes=client->cached_brakes,airbrakes=client->cached_airbrakes;
    bool rcs=client->cached_rcs,sas=client->cached_sas,autopilot=client->cached_autopilot_engaged;
    krpc_enum_t speed_mode=(krpc_enum_t)KRPC_SPACECENTER_SPEEDMODE_SURFACE;
    krpc_enum_t situation=(krpc_enum_t)KRPC_SPACECENTER_VESSELSITUATION_FLYING;
    if(airbrake_group<1)airbrake_group=1;if(airbrake_group>10)airbrake_group=10;

    ClientBatch batch={0};
#define ADD(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct medium C-Nano telemetry batch");return false;}}while(0)
    ADD(batch_add_object(&batch,PROC_FLIGHT_SPEED_OF_SOUND,client->flight_surface,BATCH_FLOAT,&speed_of_sound,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_STATIC_PRESSURE,client->flight_surface,BATCH_FLOAT,&static_pressure,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_G_FORCE,client->flight_surface,BATCH_FLOAT,&g_force,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_DRY_MASS,client->vessel,BATCH_FLOAT,&dry_mass,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_AVAILABLE_THRUST,client->vessel,BATCH_FLOAT,&available_thrust,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_THRUST,client->vessel,BATCH_FLOAT,&current_thrust,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_AVAILABLE_TORQUE,client->vessel,BATCH_TORQUE,&torque,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_MOMENT_OF_INERTIA,client->vessel,BATCH_TUPLE3,&inertia,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_LIFT,client->flight_body,BATCH_TUPLE3,&lift,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_DRAG,client->flight_body,BATCH_TUPLE3,&drag,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_CENTER_OF_MASS,client->flight_body,BATCH_TUPLE3,&com,true));
    ADD(batch_add_object(&batch,PROC_FLIGHT_CENTER_OF_MASS,client->flight_vessel,BATCH_TUPLE3,&com_root,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_GEAR,client->control,BATCH_BOOL,&gear,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_BRAKES,client->control,BATCH_BOOL,&brakes,true));
    ADD(batch_add_object_uint(&batch,PROC_CONTROL_GET_ACTION_GROUP,client->control,airbrake_group,BATCH_BOOL,&airbrakes,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_RCS,client->control,BATCH_BOOL,&rcs,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_SAS,client->control,BATCH_BOOL,&sas,true));
    ADD(batch_add_object(&batch,PROC_AUTOPILOT_ENGAGED,client->autopilot,BATCH_BOOL,&autopilot,true));
    ADD(batch_add_object(&batch,PROC_CONTROL_SPEED_MODE,client->control,BATCH_ENUM,&speed_mode,true));
    ADD(batch_add_object(&batch,PROC_VESSEL_SITUATION,client->vessel,BATCH_ENUM,&situation,true));
#undef ADD
    if(!batch_run(client,&batch,"medium telemetry",error,error_size))return false;

    client->cached_speed_of_sound=speed_of_sound;
    client->cached_static_pressure=static_pressure;
    client->cached_g_force=g_force;
    client->cached_dry_mass=dry_mass;
    client->cached_available_thrust=available_thrust;
    client->cached_current_thrust=current_thrust;
    client->cached_torque[0]=fmax(fabs(torque.e0.e0),fabs(torque.e1.e0));
    client->cached_torque[1]=fmax(fabs(torque.e0.e1),fabs(torque.e1.e1));
    client->cached_torque[2]=fmax(fabs(torque.e0.e2),fabs(torque.e1.e2));
    client->cached_inertia[0]=fabs(inertia.e0);client->cached_inertia[1]=fabs(inertia.e1);client->cached_inertia[2]=fabs(inertia.e2);
    client->cached_lift_vector=tuple_to_canonical(lift);client->cached_drag_vector=tuple_to_canonical(drag);client->cached_force_vectors_valid=true;
    client->cached_center_of_mass=tuple_to_canonical(com);client->cached_center_of_mass_valid=true;
    client->cached_center_of_mass_root=tuple_to_canonical(com_root);client->cached_center_of_mass_root_valid=true;
    client->cached_gear=gear;client->has_gear=true;
    client->cached_brakes=brakes;client->has_brakes=true;
    client->cached_airbrakes=airbrakes;client->has_airbrakes=true;
    client->cached_rcs=rcs;client->has_rcs=true;
    client->cached_sas=sas;client->has_sas=true;
    client->cached_autopilot_engaged=autopilot;client->has_autopilot_engaged=true;
    client->cached_speed_mode=speed_mode==(krpc_enum_t)KRPC_SPACECENTER_SPEEDMODE_ORBIT?SPEED_ORBIT:SPEED_SURFACE;client->has_speed_mode=true;
    snprintf(client->cached_situation,sizeof(client->cached_situation),"%s",situation_name((krpc_SpaceCenter_VesselSituation_t)situation));
    client->medium_valid=true;
    return true;
}

static bool refresh_slow(KrpcCNanoClient *client,char *error,size_t error_size){
    double apoapsis=client->cached_apoapsis,periapsis=client->cached_periapsis,period=client->cached_period;
    ClientBatch batch={0};
    if(!batch_add_object(&batch,PROC_ORBIT_APOAPSIS_ALTITUDE,client->orbit,BATCH_DOUBLE,&apoapsis,true)||
       !batch_add_object(&batch,PROC_ORBIT_PERIAPSIS_ALTITUDE,client->orbit,BATCH_DOUBLE,&periapsis,true)||
       !batch_add_object(&batch,PROC_ORBIT_PERIOD,client->orbit,BATCH_DOUBLE,&period,true)){
        set_error(error,error_size,"Could not construct slow C-Nano telemetry batch");return false;
    }
    if(!batch_run(client,&batch,"slow telemetry",error,error_size))return false;
    client->cached_apoapsis=apoapsis;client->cached_periapsis=periapsis;client->cached_period=period;client->slow_valid=true;
    return true;
}

bool krpc_cnano_client_read(KrpcCNanoClient *client,
                            const LandingConfiguration *configuration,
                            Telemetry *t, VehicleState *state,
                            char *error, size_t error_size) {
    if (!client || !client->connected || !configuration || !t || !state) {
        set_error(error, error_size, "C-Nano telemetry read has invalid state");
        return false;
    }
    double started = monotonic_seconds();
    unsigned before=client->total_rpc_calls;
    unsigned wire_before=client->total_wire_requests;
    telemetry_init(t);
    snprintf(t->vessel_name, sizeof(t->vessel_name), "%s", client->vessel_name);

    double ut=0.0,radar=0.0;
    float true_air_speed=0.0f,heading=0.0f,pitch=0.0f,roll=0.0f,aoa=0.0f,beta=0.0f,q=0.0f;
    /* AutoPilot.Error throws on the real kRPC server while disengaged. Keep a
       deliberately unsafe default so guidance can never mistake unavailable
       alignment feedback for a perfect zero-degree error. */
    float mass=0.0f,cp=0.0f,cr=0.0f,cy=0.0f,throttle=0.0f,autopilot_error=180.0f;
    krpc_tuple_double_double_double_t position={0},velocity={0},angular={0};
    krpc_tuple_double_double_double_double_t rotation={0};
    ClientBatch fast={0};
#define FAST(expr) do{if(!(expr)){set_error(error,error_size,"Could not construct fast C-Nano telemetry batch");return false;}}while(0)
    FAST(batch_add(&fast,PROC_UT,BATCH_DOUBLE,&ut,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_SURFACE_ALTITUDE,client->flight_surface,BATCH_DOUBLE,&radar,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_TRUE_AIR_SPEED,client->flight_surface,BATCH_FLOAT,&true_air_speed,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_HEADING,client->flight_surface,BATCH_FLOAT,&heading,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_PITCH,client->flight_surface,BATCH_FLOAT,&pitch,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_ROLL,client->flight_surface,BATCH_FLOAT,&roll,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_AOA,client->flight_surface,BATCH_FLOAT,&aoa,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_SIDESLIP,client->flight_surface,BATCH_FLOAT,&beta,true));
    FAST(batch_add_object(&fast,PROC_FLIGHT_DYNAMIC_PRESSURE,client->flight_surface,BATCH_FLOAT,&q,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_POSITION,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&position,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_VELOCITY,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&velocity,true));
    /* Angular velocity in Vessel.ReferenceFrame is identically zero because
       that frame rotates with the craft. Read it in the non-rotating body
       frame and rotate the vector back into vessel axes below. */
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_ANGULAR_VELOCITY,client->vessel,client->body_nonrotating,BATCH_TUPLE3,&angular,true));
    FAST(batch_add_object(&fast,PROC_VESSEL_MASS,client->vessel,BATCH_FLOAT,&mass,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_PITCH,client->control,BATCH_FLOAT,&cp,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_ROLL,client->control,BATCH_FLOAT,&cr,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_YAW,client->control,BATCH_FLOAT,&cy,true));
    FAST(batch_add_object(&fast,PROC_CONTROL_THROTTLE,client->control,BATCH_FLOAT,&throttle,true));
    if (client->has_autopilot_engaged && client->cached_autopilot_engaged)
        FAST(batch_add_object(&fast,PROC_AUTOPILOT_ERROR,client->autopilot,BATCH_FLOAT,&autopilot_error,true));
    FAST(batch_add_two_objects(&fast,PROC_VESSEL_ROTATION,client->vessel,client->body_nonrotating,BATCH_TUPLE4,&rotation,true));
#undef FAST
    if(!batch_run(client,&fast,"fast telemetry",error,error_size))return false;

    t->ut=ut;t->radar_altitude=radar;t->true_air_speed=true_air_speed;t->heading=heading;t->pitch=pitch;t->roll=roll;
    t->angle_of_attack=aoa;t->sideslip=beta;t->dynamic_pressure=q;t->mass=mass;
    t->has_controls=true;t->control_pitch=cp;t->control_roll=cr;t->control_yaw=cy;t->throttle=throttle;
    t->autopilot_error=fabs((double)autopilot_error);
    Vector3 pos = tuple_to_canonical(position);
    Vector3 vel = tuple_to_canonical(velocity);
    GeoPoint geo = predictor_geo_point(pos, &client->planet, ut);
    t->latitude = geo.latitude;
    t->longitude = geo.longitude;
    t->mean_altitude = geo.altitude;

    Vector3 up = vnorm(pos, v3(0, 1, 0));
    Vector3 surface_velocity = vsub(vel, vcross(planet_rotation_vector(&client->planet), pos));
    t->vertical_speed = vdot(surface_velocity, up);
    t->horizontal_speed = vmag(vproject_plane(surface_velocity, up));
    t->surface_speed = vmag(surface_velocity);
    t->flight_path_angle = atan2(t->vertical_speed, fmax(.1, t->horizontal_speed)) * RAD2DEG;

    if (t->true_air_speed > 1.0 && t->dynamic_pressure >= 0.0)
        t->atmospheric_density = 2.0 * t->dynamic_pressure / (t->true_air_speed * t->true_air_speed);
    else
        t->atmospheric_density = planet_atmospheric_density(&client->planet, t->mean_altitude);

    /* `rotation` is kRPC's (x,y,z,w) quaternion mapping vessel-frame vectors
       into body-non-rotating coordinates. Apply its transpose to the inertial
       angular-velocity vector to recover true body-axis rates. */
    double qx=rotation.e0,qy=rotation.e1,qz=rotation.e2,qw=rotation.e3;
    double r00=1.0-2.0*(qy*qy+qz*qz),r01=2.0*(qx*qy-qz*qw),r02=2.0*(qx*qz+qy*qw);
    double r10=2.0*(qx*qy+qz*qw),r11=1.0-2.0*(qx*qx+qz*qz),r12=2.0*(qy*qz-qx*qw);
    double r20=2.0*(qx*qz-qy*qw),r21=2.0*(qy*qz+qx*qw),r22=1.0-2.0*(qx*qx+qy*qy);
    double body_x=r00*angular.e0+r10*angular.e1+r20*angular.e2;
    double body_y=r01*angular.e0+r11*angular.e1+r21*angular.e2;
    double body_z=r02*angular.e0+r12*angular.e1+r22*angular.e2;
    /* Keep the established bridge/control sign convention. */
    t->has_body_pitch_rate = t->has_body_roll_rate = t->has_body_yaw_rate = true;
    t->body_pitch_rate = -body_x * RAD2DEG;
    t->body_roll_rate = -body_y * RAD2DEG;
    t->body_yaw_rate = -body_z * RAD2DEG;

    t->has_attitude_quaternion=true;
    t->attitude_quaternion[0]=rotation.e0;t->attitude_quaternion[1]=rotation.e1;t->attitude_quaternion[2]=rotation.e2;t->attitude_quaternion[3]=rotation.e3;
    snprintf(t->attitude_reference_frame,sizeof(t->attitude_reference_frame),"body-non-rotating");

    if (client->has_previous_attitude) {
        double dt = clampd(ut - client->previous_ut, .01, 1.0);
        t->pitch_rate = lowpass_update(&client->pitch_rate_filter, norm_signed_deg(t->pitch - client->previous_pitch) / dt, dt);
        t->roll_rate = lowpass_update(&client->roll_rate_filter, norm_signed_deg(t->roll - client->previous_roll) / dt, dt);
        t->heading_rate = lowpass_update(&client->heading_rate_filter, norm_signed_deg(t->heading - client->previous_heading) / dt, dt);
        t->angle_of_attack_rate=lowpass_update(&client->angle_of_attack_rate_filter,
            norm_signed_deg(t->angle_of_attack-client->previous_angle_of_attack)/dt,dt);
        t->has_angle_of_attack_rate=true;
    }
    client->has_previous_attitude = true;
    client->previous_ut = ut; client->previous_pitch = t->pitch; client->previous_roll = t->roll; client->previous_heading = t->heading;
    client->previous_angle_of_attack=t->angle_of_attack;

    t->ground_track_heading = surface_course(pos, vel, planet_rotation_vector(&client->planet), client->planet.north_axis, t->heading);
    if (client->has_previous_course) {
        double dt = ut - client->previous_course_ut;
        if (dt >= .01 && dt <= 1.0) {
            t->course_rate = lowpass_update(&client->course_rate_filter,
                norm_signed_deg(t->ground_track_heading - client->previous_course) / dt, dt);
            t->has_course_rate = true;
        } else lowpass_reset(&client->course_rate_filter);
    }
    client->has_previous_course = true;
    client->previous_course_ut = ut; client->previous_course = t->ground_track_heading;

    if (!client->medium_valid || ut - client->last_medium_ut >= CNANO_MEDIUM_PERIOD) {
        if(!refresh_medium(client,configuration->vehicle.airbrake_action_group,error,error_size))return false;
        client->last_medium_ut=ut;
    }
    if (!client->slow_valid || ut - client->last_slow_ut >= CNANO_SLOW_PERIOD) {
        if(!refresh_slow(client,error,error_size))return false;
        client->last_slow_ut=ut;
    }

    t->speed_of_sound = client->cached_speed_of_sound > 1.0
        ? client->cached_speed_of_sound : planet_atmospheric_speed_of_sound(&client->planet, t->mean_altitude);
    t->mach = t->true_air_speed / fmax(1.0, t->speed_of_sound);
    t->static_pressure = client->cached_static_pressure;
    t->g_force = client->cached_g_force;
    t->dry_mass = client->cached_dry_mass;
    t->available_thrust = fmax(client->cached_available_thrust, client->commandable_thrust);
    t->current_thrust = client->cached_current_thrust;
    t->has_torque = client->cached_torque[0] > 0 || client->cached_torque[1] > 0 || client->cached_torque[2] > 0;
    t->available_pitch_torque = client->cached_torque[0];
    t->available_roll_torque = client->cached_torque[1];
    t->available_yaw_torque = client->cached_torque[2];
    t->has_inertia = client->cached_inertia[0] > 0 && client->cached_inertia[1] > 0 && client->cached_inertia[2] > 0;
    t->pitch_moment_of_inertia = client->cached_inertia[0];
    t->roll_moment_of_inertia = client->cached_inertia[1];
    t->yaw_moment_of_inertia = client->cached_inertia[2];
    t->apoapsis_altitude = client->cached_apoapsis;
    t->periapsis_altitude = client->cached_periapsis;
    t->orbit_period = client->cached_period;
    t->gear = client->cached_gear;
    t->brakes = client->cached_brakes;
    t->has_airbrakes = client->has_airbrakes;
    t->airbrakes = client->cached_airbrakes;
    t->navball_speed_mode = client->has_speed_mode ? client->cached_speed_mode : SPEED_UNCHANGED;
    if (client->cached_situation[0]) snprintf(t->vessel_situation, sizeof(t->vessel_situation), "%s", client->cached_situation);
    t->has_force_vectors = client->cached_force_vectors_valid;
    t->lift_vector = client->cached_lift_vector; t->drag_vector = client->cached_drag_vector;
    t->lift_force = vmag(t->lift_vector); t->drag_force = vmag(t->drag_vector);
    t->has_center_of_mass = client->cached_center_of_mass_valid; t->center_of_mass = client->cached_center_of_mass;
    t->has_center_of_mass_root=client->cached_center_of_mass_root_valid;t->center_of_mass_root=client->cached_center_of_mass_root;

    /* C-Nano exposes no stream-side stall metric. Share the same conservative
       q/speed/AoA fallback proxy with predictor shadow telemetry so safety decisions
       see identical inputs without letting nominal low-q protective AoA classify
       itself as a measured stall emergency. */
    t->stall_fraction = vessel_physics_conservative_stall_fraction(
        t->true_air_speed, t->angle_of_attack, t->dynamic_pressure, &configuration->vehicle);
    t->stall_fraction_is_measured = false;
    t->physics_sample_valid = t->has_force_vectors && t->dynamic_pressure > 1.0 && t->mass > 1.0;

    GeoPoint current = {t->latitude, t->longitude, t->mean_altitude};
    GeoPoint site = {configuration->site.latitude, configuration->site.longitude, configuration->site.altitude};
    t->range_to_site = great_circle_distance(current, site, client->planet.radius);
    t->bearing_to_site = initial_bearing(current, site);
    t->heading_error = norm_signed_deg(t->bearing_to_site - t->heading);
    t->course_to_site_error = norm_signed_deg(t->bearing_to_site - t->ground_track_heading);
    runway_coordinates(current, site, configuration->site.runway_heading, client->planet.radius,
                       &t->runway_along_track, &t->runway_cross_track);

    state->ut = ut; state->position = pos; state->velocity = vel; state->mass = t->mass;
    client->last_read_calls=client->total_rpc_calls-before;
    client->last_read_wire_requests=client->total_wire_requests-wire_before;
    client->last_read_seconds=monotonic_seconds()-started;
    return true;
}

