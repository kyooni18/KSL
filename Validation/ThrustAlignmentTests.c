#include "thrust_alignment.h"
#include <assert.h>
#include <math.h>
int main(void){
 Vector3 axis=vnorm(v3(.1,.93,.34),v3(0,1,0)),d=vnorm(v3(-.8,.5,.2),v3(0,1,0)),u=vnorm(vproject_plane(v3(0,0,1),d),v3(0,0,1)),nose,roof;
 thrust_alignment_attitude(axis,d,u,&nose,&roof);
 Vector3 right=vnorm(vcross(nose,roof),v3(1,0,0));
 Vector3 actual=vadd(vscale(right,axis.x),vadd(vscale(nose,axis.y),vscale(roof,-axis.z)));
 assert(vmag(vsub(actual,d))<1e-9);
 assert(fabs(vdot(nose,roof))<1e-9);
 thrust_alignment_attitude(v3(0,1,0),d,u,&nose,&roof);
 assert(vmag(vsub(nose,d))<1e-9);
}
