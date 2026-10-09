#include "adaptive_trim.h"
#include <assert.h>
#include <stdio.h>

static AdaptiveTrim train(double intercept,double slope) {
    AdaptiveTrim m={0};
    for (int i=0;i<100;++i) {
        double angle=5+(i%30)/3.0;
        adaptive_trim_observe(&m,0.1*i,2.0,5000,angle,
            intercept+slope*angle,true);
    }
    return m;
}
int main(void) {
    double a,b,noise,input;
    AdaptiveTrim m=train(0.05,0.02);
    assert(adaptive_trim_fit(&m,&a,&b,&noise));
    assert(fabs(a-.05)<1e-8&&fabs(b-.02)<1e-8);
    double first=adaptive_trim_ceiling(&m,15,28);
    assert(first>15&&first<=28);
    m=train(.1,-.04); /* Opposite control sign, different vehicle response. */
    assert(adaptive_trim_fit(&m,&a,&b,&noise));
    assert(fabs(a-.1)<1e-8&&fabs(b+.04)<1e-8);
    double second=adaptive_trim_ceiling(&m,15,28);
    assert(second<first&&second>0);
    assert(adaptive_trim_input(&m,12,&input)&&fabs(input+.38)<1e-8);

    AdaptiveTrim flat={0};
    for(int i=0;i<100;++i)
        adaptive_trim_observe(&flat,.1*i,2,5000,10,.4,true);
    assert(!adaptive_trim_fit(&flat,&a,&b,&noise));
    assert(adaptive_trim_ceiling(&flat,15,28)==15);
    assert(!adaptive_trim_input(&flat,20,&input));
    assert(adaptive_trim_input(&flat,10,&input)&&fabs(input-.4)<1e-8);
    AdaptiveTrim empty={0};
    for(int i=0;i<100;++i)
        adaptive_trim_observe(&empty,.1*i,2,5000,5+i*.1,.99,true);
    assert(!adaptive_trim_input(&empty,10,&input));
    adaptive_trim_observe(&flat,10,2,5000,NAN,.3,true);
    assert(isfinite(flat.weight));

    m=train(.05,.02);
    adaptive_trim_observe(&m,10,3,5000,12,.3,true);
    assert(!adaptive_trim_fit(&m,&a,&b,&noise));
    assert(adaptive_trim_ceiling(&m,15,28)==15);
    m=train(.05,.02);
    adaptive_trim_observe(&m,100,2,5000,12,.3,true);
    assert(!adaptive_trim_input(&m,12,&input));
    puts("Adaptive trim identification, reserve, excitation and stale-regime checks passed.");
}
