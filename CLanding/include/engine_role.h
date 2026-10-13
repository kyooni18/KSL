#ifndef ENGINE_ROLE_H
#define ENGINE_ROLE_H
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
typedef enum { ENGINE_ROLE_OTHER, ENGINE_ROLE_OMS, ENGINE_ROLE_MAIN } EngineRole;
static inline bool engine_role_contains(const char *text,const char *word){
    if(!text)return false;
    for(;*text;text++){
        size_t i=0;
        while(word[i]&&text[i]&&tolower((unsigned char)text[i])==tolower((unsigned char)word[i]))i++;
        if(!word[i])return true;
    }
    return false;
}
static inline EngineRole engine_role_identify(const char *name,const char *title,const char *tag){
    /* Explicit role tags take precedence over descriptive part metadata. */
    if(tag&&(!strcmp(tag,"OMS")||!strcmp(tag,"oms")))return ENGINE_ROLE_OMS;
    if(tag&&(!strcmp(tag,"SSME")||!strcmp(tag,"ssme")))return ENGINE_ROLE_MAIN;
    if(engine_role_contains(name,"SSME")||engine_role_contains(title,"space shuttle main engine"))return ENGINE_ROLE_MAIN;
    if(engine_role_contains(title,"orbital manoeuvr")||engine_role_contains(title,"orbital maneuver")||
       engine_role_contains(title,"terrier")||engine_role_contains(name,"terrier")||
       engine_role_contains(name,"OMS"))return ENGINE_ROLE_OMS;
    return ENGINE_ROLE_OTHER;
}
static inline bool engine_role_selected(EngineRole role,bool has_oms){
    return has_oms?role==ENGINE_ROLE_OMS:role!=ENGINE_ROLE_MAIN;
}
#endif
