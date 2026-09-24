/* Shared terminal/HAC aerodynamic targets.
   Keep the target independent of acquisition strategy: MM305 path planning may
   use it, but it must never create a separate energy-dissipation phase. */

double terminal_hac_energy_aoa(const Telemetry*t,const VehicleProfile*v){
    (void)t;
    if(!v)return 0.0;
    return clampd(v->entry_angle_of_attack-8.0,8.0,10.5);
}
