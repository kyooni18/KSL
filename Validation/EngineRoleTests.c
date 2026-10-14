#include "engine_role.h"
#include <assert.h>
int main(void){
 assert(engine_role_identify("SSME","Vector","")==ENGINE_ROLE_MAIN);
 assert(engine_role_identify("benjee10.AJ10.v2","KJ-10b Viking Orbital Manoeuvring Engine","")==ENGINE_ROLE_OMS);
 assert(engine_role_identify("liquidEngine3","RE-I5 'Terrier' Liquid Fuel Engine","")==ENGINE_ROLE_OMS);
 assert(engine_role_identify("unknown","unknown","OMS")==ENGINE_ROLE_OMS);
 assert(!engine_role_selected(ENGINE_ROLE_MAIN,true));
 assert(!engine_role_selected(ENGINE_ROLE_OTHER,true));
 assert(engine_role_selected(ENGINE_ROLE_OMS,true));
 assert(!engine_role_selected(ENGINE_ROLE_MAIN,false));
 assert(engine_role_selected(ENGINE_ROLE_OTHER,false));
 assert(engine_propellant_is_intake_air("IntakeAir"));
 assert(!engine_propellant_is_intake_air("LiquidFuel"));
 assert(!engine_propellant_is_intake_air(NULL));
}
