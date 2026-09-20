#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "shuttlesim/protocol.h"

static bool json_number(const char *s,const char *key,double *out){
    char pattern[96]; snprintf(pattern,sizeof(pattern),"\"%s\"",key);
    const char *p=strstr(s,pattern); if(!p)return false; p=strchr(p,':'); if(!p)return false; p++;
    *out=strtod(p,NULL); return true;
}
static bool json_bool(const char *s,const char *key,bool *out){
    char pattern[96]; snprintf(pattern,sizeof(pattern),"\"%s\"",key);
    const char *p=strstr(s,pattern); if(!p)return false; p=strchr(p,':'); if(!p)return false; p++;
    while(*p==' '||*p=='\t') p++;
    if(!strncmp(p,"true",4)){*out=true;return true;}
    if(!strncmp(p,"false",5)){*out=false;return true;}
    return false;
}
bool protocol_parse_command(const char *json,SimCommand *cmd){
    memset(cmd,0,sizeof(*cmd));
    double aoa=0,bank=0; bool ha=json_number(json,"aoa_deg",&aoa), hb=json_number(json,"bank_deg",&bank);
    if(!ha) ha=json_number(json,"aoa",&aoa);
    if(!hb) hb=json_number(json,"bank",&bank);
    if(ha&&hb){ cmd->has_attitude=true; cmd->aoa_deg=aoa; cmd->bank_deg=bank; }
    bool gear=false; if(json_bool(json,"gear_down",&gear)){cmd->has_gear=true;cmd->gear_down=gear;}
    bool brakes=false; if(json_bool(json,"brakes",&brakes)){cmd->has_brakes=true;cmd->brakes=brakes;}
    if(strstr(json,"\"step\":true"))cmd->step=true;
    if(strstr(json,"\"pause\":true"))cmd->pause=true;
    if(strstr(json,"\"resume\":true"))cmd->resume=true;
    /* type matching without regex */
    if(strstr(json,"\"type\":\"pause\"")||strstr(json,"\"type\": \"pause\""))cmd->pause=true;
    if(strstr(json,"\"type\":\"resume\"")||strstr(json,"\"type\": \"resume\""))cmd->resume=true;
    return cmd->has_attitude||cmd->has_gear||cmd->has_brakes||cmd->pause||cmd->resume||cmd->step;
}
bool protocol_open(Protocol *p,int command_port,const char *host,int telemetry_port,int web_telemetry_port){
    memset(p,0,sizeof(*p)); p->command_fd=-1;p->telemetry_fd=-1;
    if(command_port<=0&&telemetry_port<=0&&web_telemetry_port<=0)return true;
    if(command_port>0){
        p->command_fd=socket(AF_INET,SOCK_DGRAM,0); if(p->command_fd<0)return false;
        int one=1; setsockopt(p->command_fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
        struct sockaddr_in a; memset(&a,0,sizeof(a)); a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons((unsigned short)command_port);
        if(bind(p->command_fd,(struct sockaddr*)&a,sizeof(a))<0){protocol_close(p);return false;}
        fcntl(p->command_fd,F_SETFL,fcntl(p->command_fd,F_GETFL,0)|O_NONBLOCK);
    }
    if(telemetry_port>0){
        p->telemetry_fd=socket(AF_INET,SOCK_DGRAM,0); if(p->telemetry_fd<0){protocol_close(p);return false;}
        snprintf(p->telemetry_host,sizeof(p->telemetry_host),"%s",host?host:"127.0.0.1"); p->telemetry_port=telemetry_port;
    } else if(web_telemetry_port>0) {
        p->telemetry_fd=socket(AF_INET,SOCK_DGRAM,0); if(p->telemetry_fd<0){protocol_close(p);return false;}
        snprintf(p->telemetry_host,sizeof(p->telemetry_host),"%s",host?host:"127.0.0.1");
    }
    p->web_telemetry_port=web_telemetry_port;
    p->udp_enabled=true; return true;
}
void protocol_close(Protocol *p){ if(p->command_fd>=0)close(p->command_fd);if(p->telemetry_fd>=0)close(p->telemetry_fd);p->command_fd=p->telemetry_fd=-1; }
bool protocol_poll_command(Protocol *p,SimCommand *cmd){
    if(p->command_fd<0) return false;
    char buf[2048];
    ssize_t n=recvfrom(p->command_fd,buf,sizeof(buf)-1,0,NULL,NULL);
    if(n<=0) return false;
    buf[n]=0;
    return protocol_parse_command(buf,cmd);
}
static void send_port(Protocol *p,const char *json,size_t len,int port){
    if(p->telemetry_fd<0||port<=0) return;
    struct sockaddr_in a;
    memset(&a,0,sizeof(a));
    a.sin_family=AF_INET;
    a.sin_port=htons((unsigned short)port);
    if(inet_pton(AF_INET,p->telemetry_host,&a.sin_addr)!=1) return;
    (void)sendto(p->telemetry_fd,json,len,0,(struct sockaddr*)&a,sizeof(a));
}
void protocol_send_telemetry(Protocol *p,const char *json,size_t len){
    send_port(p,json,len,p->telemetry_port);
    if(p->web_telemetry_port!=p->telemetry_port) send_port(p,json,len,p->web_telemetry_port);
}
