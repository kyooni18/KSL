/* White-box, offline MM305 mode-selection regression.
 * A polar inlet that is abeam the runway is not a Final approach just
 * because the along-track station lies behind its current x coordinate.
 */
#include "../CLanding/guidance/guidance_internal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../CLanding/guidance/taem/energy_law.inc"

int main(void){
    /* Whole-include white-box test: the full static law is linked for ABI
       fidelity, although this fixture exercises its pure geometry gates. */
    (void)taem_energy_law_step;
    TelContext c={.d_station=8000.0};
    TaemEnergyLawState law={.initialized=true,.leg=TEL_LEG_OUT,
        .loop=false,.x_turn=-8000.0,.radius=5000.0};
    TelPoint polar={.x=973.0,.y=67957.0,.psi=95.0*DEG2RAD};
    assert(tel_plan_open(&law,&c,&polar));
    assert(!tel_final_line_ready(&c,&polar));
    polar.x=-1552.0;polar.y=38725.0;polar.psi=94.0*DEG2RAD;
    assert(tel_plan_open(&law,&c,&polar));
    assert(!tel_final_line_ready(&c,&polar));
    /* Even an in-range abeam approach does not shortcut the bounded turn. */
    polar.x=-10000.0;polar.y=3000.0;polar.psi=5.0*DEG2RAD;
    assert(!tel_final_line_ready(&c,&polar));
    polar.y=1000.0;polar.psi=30.0*DEG2RAD;
    assert(!tel_final_line_ready(&c,&polar));
    polar.psi=5.0*DEG2RAD;
    assert(tel_final_line_ready(&c,&polar));
    polar.x=973.0;
    assert(tel_final_line_ready(&c,&polar));
    polar.y=NAN;
    assert(!tel_final_line_ready(&c,&polar));
    puts("MM305 off-axis and Final-mode geometry tests passed.");
    return 0;
}
