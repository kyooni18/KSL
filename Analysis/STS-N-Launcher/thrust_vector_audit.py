#!/usr/bin/env python3
import math

G0 = 9.80665
# Launch-prepared STS-N: saved craft with all onboard Oxidizer removed.
ORBITER_M = 40.155
ORBITER_Z = 4.55
ORBITER_Y = -8.8267
ORBITER_THRUST_Z = ORBITER_Z + 0.111568
ORBITER_THRUST_Y = -7.1558
ORBITER_T_VAC = 820.0
ORBITER_T_SL = ORBITER_T_VAC * 500.0 / 580.0
# ET/core
ET_DRY = 62.7003701868
ET_Y = 2.5
CORE_M = 8.5
CORE_Z = 0.0
CORE_Y = -12.64
CORE_THRUST_Y = -15.437
CORE_T_VAC = 2150.0
CORE_T_SL = CORE_T_VAC * 295.0 / 345.0
CORE_GIMBAL = 6.0
ADAPTER_M = 0.3
ADAPTER_Y = -11.247
DECOUPLER_M = 0.25
DECOUPLER_Z = ORBITER_Z / 2.0
# ET loading and full-throttle flow
VOLUME = 67901.305380
LF_M = VOLUME * 0.45 * 0.005
CORE_MDOT = CORE_T_VAC / (345.0 * G0)
CORE_LF_MDOT = CORE_MDOT * 0.45
CORE_OX_MDOT = CORE_MDOT * 0.55
ORBITER_MDOT = ORBITER_T_VAC / (580.0 * G0)
OX_M = LF_M * CORE_OX_MDOT / (CORE_LF_MDOT + ORBITER_MDOT)
ET_PROP0 = LF_M + OX_M
LIQ_MDOT = CORE_MDOT + ORBITER_MDOT
# PhotonCorp RSRB-5
SRB_R = 4.45
SRB_DRY = 7.0 + 0.075 + 3.0 * 0.0075 + 0.25
SRB_FUEL0 = 16390.0 * 0.0075
SRB_Y = 0.0
SRB_THRUST_Y = -13.768
SRB_T_VAC = 3632.0
SRB_T_SL = SRB_T_VAC * 241.0 / 267.0
SRB_GIMBAL = 7.5
SRB_KEYS = [
    (0.0,0.15,0.0,0.0),
    (0.01252535,0.1839709,1.820318,4.336654),
    (0.02268416,0.2996643,3.372431,3.372431),
    (0.09468817,0.5038492,1.704485,1.41069),
    (0.3896041,0.7964334,0.8484721,0.6002516),
    (0.4083008,0.7989929,-0.08580669,-0.4211084),
    (0.65,0.65,-0.4383497,0.6146616),
    (0.95,1.0,0.3562177,0.0),
    (1.0,0.95,0.0,0.0),
]
SRB_MDOT_BASE = SRB_T_VAC / (267.0 * G0)

def hermite_curve(x):
    if x <= SRB_KEYS[0][0]: return SRB_KEYS[0][1]
    if x >= SRB_KEYS[-1][0]: return SRB_KEYS[-1][1]
    for a,b in zip(SRB_KEYS,SRB_KEYS[1:]):
        if a[0] <= x <= b[0]:
            x0,y0,_,m0 = a; x1,y1,m1,_ = b
            h=x1-x0; u=(x-x0)/h
            return ((2*u**3-3*u**2+1)*y0 + (u**3-2*u**2+u)*h*m0
                    +(-2*u**3+3*u**2)*y1 + (u**3-u**2)*h*m1)
    raise AssertionError

def integrate_srb(initial_fuel=SRB_FUEL0, dt=0.002):
    t=0.0; fuel=initial_fuel; out=[]
    while fuel > 0.0:
        frac=fuel/SRB_FUEL0
        c=hermite_curve(max(0.0,min(1.0,frac)))
        out.append((t,fuel,c))
        fuel=max(0.0,fuel-SRB_MDOT_BASE*c*dt)
        t += dt
    return out,t

NOM_SRB, SRB_BURN = integrate_srb()

def srb_state(t, hist=NOM_SRB):
    if t >= hist[-1][0]+0.002: return 0.0,0.0
    i=min(int(t/0.002),len(hist)-1)
    return hist[i][1],hist[i][2]

def thrust_at_pressure(vac, sl, p):
    # Robust interpolation for sweep. Endpoint ratios are authoritative; all relevant curves are monotonic 0..1 atm.
    return vac + (sl-vac)*p

def nominal_mass_com(t, et_prop=None, srb_fuel=None):
    if et_prop is None: et_prop=max(0.0,ET_PROP0-LIQ_MDOT*t)
    if srb_fuel is None:
        srb_fuel,_=srb_state(t)
    masses=[
        (ORBITER_M,0.0,ORBITER_Z,ORBITER_Y),
        (ET_DRY+et_prop,0.0,0.0,ET_Y),
        (CORE_M,0.0,CORE_Z,CORE_Y),
        (ADAPTER_M,0.0,0.0,ADAPTER_Y),
        (DECOUPLER_M,0.0,DECOUPLER_Z,0.0),
    ]
    if srb_fuel > 0.0:
        masses += [(SRB_DRY+srb_fuel,SRB_R,0.0,SRB_Y),(SRB_DRY+srb_fuel,-SRB_R,0.0,SRB_Y)]
    M=sum(m for m,_,_,_ in masses)
    x=sum(m*x for m,x,_,_ in masses)/M
    z=sum(m*z for m,_,z,_ in masses)/M
    y=sum(m*y for m,_,_,y in masses)/M
    return M,x,z,y

def pitch_moment(u,t,p,core_cant_deg=0.0,srb_cant_deg=0.0,srb_limiter=0.60,et_prop=None,srb_fuel=None):
    M,xcm,zcm,ycm=nominal_mass_com(t,et_prop,srb_fuel)
    To=thrust_at_pressure(ORBITER_T_VAC,ORBITER_T_SL,p)
    Tc=thrust_at_pressure(CORE_T_VAC,CORE_T_SL,p)
    sf,curve=srb_state(t) if srb_fuel is None else (srb_fuel,hermite_curve(srb_fuel/SRB_FUEL0) if srb_fuel>0 else 0)
    Ts=thrust_at_pressure(SRB_T_VAC,SRB_T_SL,p)*curve if sf>0 else 0.0
    dc=math.radians(core_cant_deg+CORE_GIMBAL*u)
    db=math.radians(srb_cant_deg+SRB_GIMBAL*srb_limiter*u)
    # Mx = ry*Fz - rz*Fy. Sign is arbitrary but internally consistent.
    mom=(ORBITER_THRUST_Y-ycm)*0.0 - (ORBITER_THRUST_Z-zcm)*To
    mom+=(CORE_THRUST_Y-ycm)*Tc*math.sin(dc) - (CORE_Z-zcm)*Tc*math.cos(dc)
    if Ts:
        # both boosters lie at z=0, pitch forces add; x offsets only affect roll/yaw.
        mom+=2*((SRB_THRUST_Y-ycm)*Ts*math.sin(db) - (0.0-zcm)*Ts*math.cos(db))
    return mom

def solve_pitch(t,p,core_cant_deg=0.0,srb_cant_deg=0.0,srb_limiter=0.60,et_prop=None,srb_fuel=None):
    f=lambda u:pitch_moment(u,t,p,core_cant_deg,srb_cant_deg,srb_limiter,et_prop,srb_fuel)
    lo,hi=-1.0,1.0; flo,fhi=f(lo),f(hi)
    if flo*fhi>0:return None
    for _ in range(80):
        mid=(lo+hi)/2; fm=f(mid)
        if flo*fm<=0:hi=mid;fhi=fm
        else:lo=mid;flo=fm
    u=(lo+hi)/2
    sf,curve=srb_state(t) if srb_fuel is None else (srb_fuel,hermite_curve(srb_fuel/SRB_FUEL0) if srb_fuel>0 else 0)
    To=thrust_at_pressure(ORBITER_T_VAC,ORBITER_T_SL,p)
    Tc=thrust_at_pressure(CORE_T_VAC,CORE_T_SL,p)
    Ts=thrust_at_pressure(SRB_T_VAC,SRB_T_SL,p)*curve if sf>0 else 0
    dc=math.radians(core_cant_deg+CORE_GIMBAL*u)
    db=math.radians(srb_cant_deg+SRB_GIMBAL*srb_limiter*u)
    Fy=To+Tc*math.cos(dc)+2*Ts*math.cos(db)
    Fz=Tc*math.sin(dc)+2*Ts*math.sin(db)
    angle=math.degrees(math.atan2(Fz,Fy))
    ideal=To+Tc+2*Ts
    loss=1.0-Fy/ideal
    return u,math.degrees(dc),math.degrees(db),angle,loss

def prop_from_actual_time(t):
    # Full throttle until vacuum-equivalent liquid TWR reaches 2g, then common throttle caps acceleration at 2g.
    # Compute threshold on post-SRB central mass model, then integrate exponential mass decay analytically.
    final_attached=ORBITER_M+ET_DRY+CORE_M+ADAPTER_M+DECOUPLER_M
    Tv=ORBITER_T_VAC+CORE_T_VAC
    M2g=Tv/(2.0*G0)
    prop_at_2g=M2g-final_attached
    t2g=(ET_PROP0-prop_at_2g)/LIQ_MDOT
    if t<=t2g:return max(0.0,ET_PROP0-LIQ_MDOT*t),1.0,t2g
    M=M2g*math.exp(-LIQ_MDOT*(t-t2g)/M2g)
    if M<=final_attached:return 0.0,final_attached/M2g,t2g
    return M-final_attached,M/M2g,t2g

def actual_et_depletion_time():
    final_attached=ORBITER_M+ET_DRY+CORE_M+ADAPTER_M+DECOUPLER_M
    Tv=ORBITER_T_VAC+CORE_T_VAC
    M2g=Tv/(2.0*G0)
    prop_at_2g=M2g-final_attached
    t2g=(ET_PROP0-prop_at_2g)/LIQ_MDOT
    dt=M2g/LIQ_MDOT*math.log(M2g/final_attached)
    return t2g+dt,t2g

def yaw_solve(t,p,fuel_plus,fuel_minus,srb_limiter=0.60,pitch_u=0.0):
    # 3D-small-angle conservative yaw solution with current pitch deflection using a shared yaw command.
    # SRBs remain attached while fuel >0. Central masses use nominal ET propellant at this time.
    et=max(0.0,ET_PROP0-LIQ_MDOT*t)
    elems=[(ORBITER_M,0.0,ORBITER_Y),(ET_DRY+et,0.0,ET_Y),(CORE_M,0.0,CORE_Y),(ADAPTER_M,0.0,ADAPTER_Y),(DECOUPLER_M,0.0,0.0)]
    if fuel_plus>0: elems.append((SRB_DRY+fuel_plus, SRB_R,SRB_Y))
    if fuel_minus>0: elems.append((SRB_DRY+fuel_minus,-SRB_R,SRB_Y))
    M=sum(m for m,_,_ in elems); xcm=sum(m*x for m,x,_ in elems)/M; ycm=sum(m*y for m,_,y in elems)/M
    To=thrust_at_pressure(ORBITER_T_VAC,ORBITER_T_SL,p)
    Tc=thrust_at_pressure(CORE_T_VAC,CORE_T_SL,p)
    cp=CORE_GIMBAL*pitch_u
    bp=SRB_GIMBAL*srb_limiter*pitch_u
    cplus=hermite_curve(fuel_plus/SRB_FUEL0) if fuel_plus>0 else 0
    cminus=hermite_curve(fuel_minus/SRB_FUEL0) if fuel_minus>0 else 0
    Tbase=thrust_at_pressure(SRB_T_VAC,SRB_T_SL,p)
    Tp=Tbase*cplus; Tm=Tbase*cminus
    # Conservative spherical gimbal cone: yaw range reduced by existing pitch deflection.
    cyawmax=math.sqrt(max(0.0,CORE_GIMBAL**2-cp**2))
    byawlim=SRB_GIMBAL*srb_limiter
    byawmax=math.sqrt(max(0.0,byawlim**2-bp**2))
    def mz(u):
        dc=math.radians(cyawmax*u); db=math.radians(byawmax*u)
        out=(0-xcm)*To
        out+=(0-xcm)*Tc*math.cos(dc)-(CORE_THRUST_Y-ycm)*Tc*math.sin(dc)
        if fuel_plus>0: out+=(SRB_R-xcm)*Tp*math.cos(db)-(SRB_THRUST_Y-ycm)*Tp*math.sin(db)
        if fuel_minus>0: out+=(-SRB_R-xcm)*Tm*math.cos(db)-(SRB_THRUST_Y-ycm)*Tm*math.sin(db)
        return out
    lo,hi=-1.0,1.0; flo,fhi=mz(lo),mz(hi)
    if flo*fhi>0:return None,(mz(0),mz(lo),mz(hi),xcm,Tp,Tm,cyawmax,byawmax)
    for _ in range(80):
        mid=(lo+hi)/2; fm=mz(mid)
        if flo*fm<=0:hi=mid;fhi=fm
        else:lo=mid;flo=fm
    u=(lo+hi)/2
    return u,(mz(0),mz(lo),mz(hi),xcm,Tp,Tm,cyawmax,byawmax)

def asymmetric_hist(frac_delta):
    # one booster starts +delta/2 and the other -delta/2 relative to nominal total fill.
    hp,bp=integrate_srb(SRB_FUEL0*(1+frac_delta/2))
    hm,bm=integrate_srb(SRB_FUEL0*(1-frac_delta/2))
    return hp,bp,hm,bm

def hist_state(hist,t):
    if t>=hist[-1][0]+0.002:return 0.0
    return hist[min(int(t/0.002),len(hist)-1)][1]

def run():
    print('=== NOMINAL PITCH / RESULTANT VECTOR ===')
    depletion,t2g=actual_et_depletion_time()
    print(f'SRB burnout: {SRB_BURN:.3f} s')
    print(f'2g cap starts (vacuum-equivalent): {t2g:.3f} s')
    print(f'ET depletion with 2g cap: {depletion:.3f} s (not 355.190 s full-throttle equivalent)')
    # optimize a small fixed Cougar cant for maximum symmetric gimbal reserve over nominal ascent.
    best=None
    for j in range(-100,101,5):
        cant=j/100.0
        worst=0.0; fail=False; where=None
        # actual-time samples; pressure sweep makes it robust to atmosphere/time correlation.
        for k in range(0,121):
            t=depletion*k/120
            et,thr,_=prop_from_actual_time(t)
            sf,_=srb_state(t)
            for p in (0.0,1.0):
                sol=solve_pitch(t,p,cant,0.0,0.60,et,sf)
                if sol is None: fail=True; worst=2; break
                if abs(sol[0])>worst:worst=abs(sol[0]);where=(t,p,sol)
            if fail:break
        if best is None or worst<best[0]:best=(worst,cant,where)
    print(f'Best small fixed Cougar cant with 60% SRB gimbal limiter: {best[1]:+.2f} deg; worst shared pitch input {best[0]*100:.1f}%')
    print('Representative states using zero fixed cant (simpler build):')
    for t in [0,40,80,120,130,135,SRB_BURN,160,200,240,280,300,t2g,320,340,360,depletion]:
        et,throttle,_=prop_from_actual_time(t)
        sf,_=srb_state(t)
        vals=[]
        for p in (0,1):
            s=solve_pitch(t,p,0,0,0.60,et,sf)
            vals.append(s)
        # actual ascent near SL initially and vacuum late; report robust endpoint range.
        umin=min(v[0] for v in vals);umax=max(v[0] for v in vals)
        tilt=min(v[3] for v in vals),max(v[3] for v in vals)
        loss=max(v[4] for v in vals)
        print(f't={t:7.3f}s ET={et:7.2f}t thr={throttle:5.3f}  pitch input={umin:+.3f}..{umax:+.3f}  core={vals[0][1]:+.2f}/{vals[1][1]:+.2f}deg  SRB={vals[0][2]:+.2f}/{vals[1][2]:+.2f}deg  resultant tilt={tilt[0]:+.3f}..{tilt[1]:+.3f}deg  axial loss<={100*loss:.4f}%')

    print('\n=== BOOSTER FUEL/THRUST ASYMMETRY ===')
    for delta in [0.001,0.005,0.01,0.02,0.05]:
        hp,bp,hm,bm=asymmetric_hist(delta)
        end=max(bp,bm); worst=(0,None); untrim=[]
        t=0.0
        while t<=end+0.001:
            fp=hist_state(hp,t); fm=hist_state(hm,t)
            # use sea-level and vacuum; nominal pitch trim from symmetric state for conservative gimbal cone reservation.
            et=max(0,ET_PROP0-LIQ_MDOT*t)
            sf_nom,_=srb_state(min(t,SRB_BURN))
            pitch=solve_pitch(min(t,SRB_BURN),0.0,0,0,0.60,et,sf_nom)
            pu=pitch[0] if pitch else 0
            for p in (0.0,1.0):
                u,meta=yaw_solve(t,p,fp,fm,0.60,pu)
                if u is None:untrim.append((t,p,meta));metric=2
                else:metric=abs(u)
                if metric>worst[0]:worst=(metric,(t,p,u,fp,fm,meta))
            t+=0.25
        label='UNTRIMMABLE' if untrim else f'{worst[0]*100:.1f}% yaw input'
        print(f'initial pair fill mismatch {100*delta:.2f}%: burn +={bp:.3f}s / -={bm:.3f}s, worst={label} at t={worst[1][0]:.1f}s')

    print('\n=== COMMAND/LIMITER MISMATCH SENSITIVITY ===')
    # Equal fuel, direct thrust mismatch at several thrust-curve states. Treat mismatch as +/-eps/2 around nominal.
    for t in [0,40,80,120,130,135]:
        sf,curve=srb_state(t)
        et=max(0,ET_PROP0-LIQ_MDOT*t)
        ps=solve_pitch(t,0,0,0,0.60,et,sf)
        pu=ps[0] if ps else 0
        base=SRB_T_VAC*curve
        # equivalent fuel values but override via direct moment calculation analytically using yaw_solve isn't convenient.
        # Linear sensitivity: axial mismatch torque ~= R * deltaT. Available small-angle yaw stiffness below.
        M,x,z,y=nominal_mass_com(t,et,sf)
        Tc=CORE_T_VAC; To=ORBITER_T_VAC
        cp=math.radians(CORE_GIMBAL*pu); bpitch=math.radians(SRB_GIMBAL*.60*pu)
        cy=math.radians(math.sqrt(max(0,CORE_GIMBAL**2-(CORE_GIMBAL*pu)**2)))
        by=math.radians(math.sqrt(max(0,(SRB_GIMBAL*.60)**2-(SRB_GIMBAL*.60*pu)**2)))
        # dMz/du at zero yaw (m*kN) for all gimballed engines; position x terms have zero derivative first order.
        stiff=abs((CORE_THRUST_Y-y)*Tc*cy + 2*(SRB_THRUST_Y-y)*base*by)
        for eps in [0.01,0.05,0.10]:
            dm=SRB_R*base*eps
            u=dm/stiff if stiff else math.inf
            print(f't={t:5.1f}s curve={curve:.3f}, pair thrust mismatch={100*eps:4.1f}% -> approx yaw input {100*u:5.1f}% (moment {dm/1000:.3f} MNm)')
        # placement radius mismatch producing same moment as DeltaR*T
        dr50=0.5*stiff/base if base else math.inf
        print(f'  radial placement difference for 50% yaw input: about {dr50:.3f} m')

    print('\n=== LIQUID-ENGINE THRUST-RATIO FAULTS, POST-SRB ===')
    # Solve core absolute gimbal if J-N500 thrust differs while Cougar stays nominal; vacuum worst/simpler.
    for t in [SRB_BURN,200,280,320,depletion]:
        et,_,_=prop_from_actual_time(t)
        M,x,z,y=nominal_mass_com(t,et,0)
        print(f't={t:7.2f}s ET={et:7.2f}t CoM-z={z:.3f}m: ',end='')
        vals=[]
        for factor in [0.0,0.5,0.8,0.9,1.0,1.1,1.2]:
            To=ORBITER_T_VAC*factor; Tc=CORE_T_VAC
            def mom(a):
                return -(ORBITER_THRUST_Z-z)*To + (CORE_THRUST_Y-y)*Tc*math.sin(a) - (CORE_Z-z)*Tc*math.cos(a)
            lo,hi=-math.radians(12),math.radians(12);flo=mom(lo);fhi=mom(hi)
            if flo*fhi>0:deg=None
            else:
                for _ in range(80):
                    mid=(lo+hi)/2;fm=mom(mid)
                    if flo*fm<=0:hi=mid
                    else:lo=mid;flo=fm
                deg=math.degrees((lo+hi)/2)
            vals.append((factor,deg))
        print('  '.join(f'J={f:3.1f}x:{"FAIL" if d is None or abs(d)>6 else f"{d:+.2f}deg"}' for f,d in vals))

if __name__=='__main__':run()
