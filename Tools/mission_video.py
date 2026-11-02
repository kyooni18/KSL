#!/usr/bin/env python3
"""Offline launch/landing HDR HUD. Reads recorded artifacts; never connects to KSP.

Launch has no position log: its 2-D flown profile integrates measured horizontal
speed / measured radius over simulation UT. It is explicitly labelled as such,
and is never extended with a forecast. Native landing uses recorded positions.
Missing intervals remain gaps. The clock interpolates wall-time/UT observations
(including rails warp), marking intervals without dense observations with ~.
"""
from __future__ import annotations

import argparse
import base64
import bisect
import csv
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

try:
    from . import landing_video as old
except ImportError:
    import landing_video as old


def increasing(rows, key):
    """Discard repeated stale snapshots, preserving the first arrival time."""
    out = []
    for row in rows:
        if key in row and (not out or row[key] > out[-1][key]):
            out.append(row)
    return out


class Timeline:
    def __init__(self, launch: Path, landing: Path, native: Path):
        self.launch = launch
        self.ascent = increasing(list(old.read_jsonl(launch / 'telemetry.jsonl')), 'ut')
        self.landing = increasing(list(old.read_jsonl(landing / 'telemetry.jsonl')), 'ut')
        config = json.loads((native.parent / 'configuration.json').read_text())
        self.planet = config['planet']
        self.runway = config['runway']
        self.radius = self.planet['radius']
        self.site = dict(latitude=math.degrees(self.runway['latitude']),
                         longitude=math.degrees(self.runway['longitude']),
                         runwayHeading=math.degrees(self.runway['heading']))
        events = list(old.read_jsonl(launch / 'events.jsonl'))
        self.liftoff_wall = next(e['time'] for e in events if e['message'] == 'ascent: liftoff')
        a, b = self.ascent[:2]
        self.liftoff_ut = a['ut'] + (self.liftoff_wall-a['time']) * (b['ut']-a['ut'])/(b['time']-a['time'])
        # Native records carry simulation UT; mission telemetry pairs that UT
        # with epoch time. Never use computation/wallInterval as a flight clock.
        self.lt = np.array([r['time'] for r in self.landing])
        self.lu = np.array([r['ut'] for r in self.landing])
        landing_g = np.array([r['gForce'] for r in self.landing])
        self.at = np.array([r['time'] for r in self.ascent])
        self.au = np.array([r['ut'] for r in self.ascent])
        rows = []
        # Full ascent CSV is denser than UI telemetry, but UT remains authoritative.
        with (launch / 'ascent.csv').open(newline='') as f:
            for r in csv.DictReader(f):
                u = float(r['ut'])
                if not self.au[0] <= u <= self.au[-1]:
                    continue
                v, h = float(r['vertical_speed_mps']), float(r['horizontal_speed_mps'])
                rows.append(dict(time=float(np.interp(u, self.au, self.at)), ut=u,
                    altitude=float(r['altitude_m']), speed=math.hypot(v,h), horizontalSpeed=h,
                    verticalSpeed=v, dynamicPressure=float(r['dynamic_pressure_pa']),
                    gForce=float(r['actual_g']), pitch=float(r['actual_pitch_deg']),
                    heading=float(r['actual_heading_deg']), roll=float(r['actual_roll_deg']),
                    vel_fpa=math.degrees(math.atan2(v,h)), mode='launch',
                    angleOfAttack=0.0, mach=float('nan')))
        self.ascent_rows = rows[:]
        # A measured orbital snapshot precedes the rails-warp interval.
        first = next(old.read_jsonl(native))
        source = json.loads(base64.b64decode(first['initialDeorbitPlan']))['source']
        review = json.loads((landing / 'plan-review.json').read_text())
        ref = review['reference']
        source_wall = ref['observedAt'] + source['time']-ref['ut']
        source_row = self.native_row(dict(state=source, guidance={}), source_wall)
        source_row['gForce'] = float('nan')
        source_row['mode'] = 'launch'
        rows.append(source_row)
        # Native state has true pitch from its FRD quaternion, including during
        # orbital rotations where FPA+AoA is not an attitude measurement.
        for r in old.read_jsonl(native):
            u = r['state']['time']
            if not self.lu[0] <= u <= self.lu[-1]:
                continue
            wall = float(np.interp(u, self.lu, self.lt))
            row = self.native_row(r, wall)
            row['gForce'] = float(np.interp(u, self.lu, landing_g))
            rows.append(row)
        self.rows = increasing(sorted(rows, key=lambda r:r['time']), 'ut')
        self.times = np.array([r['time'] for r in self.rows])
        self.uts = np.array([r['ut'] for r in self.rows])
        self.values = {k: np.array([r.get(k,float('nan')) for r in self.rows]) for k in
                       ('altitude','speed','gForce','pitch','heading','roll','vel_fpa',
                        'vel_heading','dynamicPressure','mach','rangeToSite',
                        'runwayAlongTrack','runwayCrossTrack','latitude','longitude')}
        self.clock_t = np.concatenate(([self.liftoff_wall], self.at, [source_wall], self.lt))
        self.clock_u = np.concatenate(([self.liftoff_ut], self.au, [source['time']], self.lu))
        self.clock_gaps=[(a,b) for a,b in zip(self.clock_t[:-1],self.clock_t[1:]) if b-a>15]
        self.warp_clock=[]
        # The native backend logs accepted rails-warp slots and UT milestones,
        # but not their epoch timestamps. Reconstruct their relative durations
        # between the two measured clock anchors; retain ~ for this entire gap.
        stderr=landing/'backend.stderr.log'
        if stderr.exists():
            log=stderr.read_text()
            begin=re.search(r'rails warp: start UT=(\d+) stopUT=(\d+)',log)
            accepted=re.search(r'requestedIndex=\d+ acceptedIndex=(\d+)',log)
            end=re.search(r'rails warp: complete at UT=(\d+)',log)
            if begin and accepted and end:
                start_ut,stop_ut=map(float,begin.groups())
                rates=[1,5,10,50,100,1000,10000,100000]  # OrbitalRailsWarpSchedule.stockRates
                knots=[source['time'],start_ut];segment_rates=[1.0]
                rate=rates[int(accepted[1])]
                for rem,idx in re.findall(r'stepped down: remaining=(\d+)s accepted index=(\d+)',log):
                    knots.append(stop_ut-float(rem));segment_rates.append(rate)
                    rate=rates[int(idx)]
                terminal=re.search(r'1x terminal alignment reached: remaining=(\d+)s',log)
                if terminal:
                    knots.append(stop_ut-float(terminal[1]));segment_rates.append(rate);rate=1
                knots.extend([float(end[1]),self.lu[0]]);segment_rates.extend([rate,1])
                if all(b>a for a,b in zip(knots[:-1],knots[1:])):
                    relative=np.concatenate(([0.],np.cumsum(np.diff(knots)/np.array(segment_rates))))
                    epochs=source_wall+relative/relative[-1]*(self.lt[0]-source_wall)
                    self.warp_clock=[dict(time=float(t),ut=float(u)) for t,u in zip(epochs,knots)]
                    paired=sorted(zip(np.concatenate((self.clock_t,epochs[1:-1])),
                                      np.concatenate((self.clock_u,np.array(knots)[1:-1]))))
                    self.clock_t,self.clock_u=(np.array(x) for x in zip(*paired))
        self.ascent_end = self.at[-1]
        self.entry_wall = next((r['time'] for r in self.rows if r['mode']=='landing'), self.lt[0])
        # Only integrate adjacent measured ascent states. No line crosses missing
        # telemetry or orbit-logger handoff. This is an observed 2-D profile,
        # not a future conic or a nose-heading-derived ground track.
        ar = self.ascent_rows
        u = np.array([s['ut'] for s in ar]); alt=np.array([s['altitude'] for s in ar])
        h=np.array([s['horizontalSpeed'] for s in ar])
        du=np.diff(u); omega=h/(self.radius+alt)
        self.arc=np.concatenate(([0.0], np.cumsum(du*(omega[1:]+omega[:-1])/2)))
        self.downrange = self.arc*self.radius/1000
        self.launch_times = np.array([s['time'] for s in ar])
        self.launch_ut = u
        self.launch_alt = alt/1000
        self.launch_q = np.array([s['dynamicPressure'] for s in ar])/1000
        self.launch_xy = np.column_stack(((1+alt/self.radius)*np.sin(self.arc),
                                         -(1+alt/self.radius)*np.cos(self.arc)))
        # Native actual positions, in an equatorial projection of this near-
        # equatorial mission. Blue body circle stays at the panel's center.
        self.native_xy = []
        for r in self.rows:
            if 'position' in r:
                p=r['position']; self.native_xy.append((r['time'],p[0]/self.radius,-p[1]/self.radius))

    def native_row(self, record, wall):
        s=record['state']
        pos=np.array([s['position'][k] for k in ('x','y','z')])
        velocity=np.array([s['velocity'][k] for k in ('x','y','z')])
        radius=float(np.linalg.norm(pos));up=pos/radius
        lon=math.atan2(pos[1],pos[0]);lat=math.asin(float(up[2]))
        east=np.array([-math.sin(lon),math.cos(lon),0]);north=np.cross(up,east)
        q=s['attitude'];qv=np.array([q[k] for k in ('x','y','z')])
        def rotate(v):
            v=np.array(v);cross=2*np.cross(qv,v)
            return v+q['w']*cross+np.cross(qv,cross)
        forward,right,roof=rotate([1,0,0]),rotate([0,1,0]),rotate([0,0,-1])
        heading=math.degrees(math.atan2(float(forward@east),float(forward@north)))%360
        pitch=math.degrees(math.asin(float(np.clip(forward@up,-1,1))))
        roll=-math.degrees(math.atan2(float(right@up),float(roof@up)))
        vertical=float(velocity@up);horizontal=float(np.linalg.norm(velocity-vertical*up))
        fpa=math.degrees(math.atan2(vertical,horizontal))
        vel_heading=math.degrees(math.atan2(float(velocity@east),float(velocity@north)))%360
        lat0=math.radians(self.site['latitude']);dlon=lon-math.radians(self.site['longitude'])
        hav=math.sin((lat-lat0)/2)**2+math.cos(lat)*math.cos(lat0)*math.sin(dlon/2)**2
        distance=2*self.radius*math.asin(math.sqrt(max(0,min(1,hav))))
        bearing=math.atan2(math.sin(dlon)*math.cos(lat),
            math.cos(lat0)*math.sin(lat)-math.sin(lat0)*math.cos(lat)*math.cos(dlon))
        delta=bearing-self.runway['heading']
        phase=record.get('guidance',{}).get('phase','orbitalCoast')
        return dict(time=wall,ut=s['time'],altitude=radius-self.radius,
                    latitude=math.degrees(lat),longitude=math.degrees(lon),
                    speed=float(np.linalg.norm(velocity)),horizontalSpeed=horizontal,
                    verticalSpeed=vertical,pitch=pitch,heading=heading,roll=roll,
                    rangeToSite=distance,runwayAlongTrack=distance*math.cos(delta),
                    runwayCrossTrack=distance*math.sin(delta),gForce=float('nan'),
                    dynamicPressure=s['aerodynamics']['dynamicPressure'],
                    mach=s['aerodynamics']['mach'],
                    vel_heading=vel_heading,vel_fpa=fpa,
                    mode='launch' if phase in ('orbitalCoast','burnSetup','deorbitBurn','entryCoast') else 'landing',
                    position=pos.tolist(),velocity=velocity.tolist())

    def clock(self,t):
        if t < self.clock_t[0]:
            rate=(self.au[1]-self.au[0])/(self.at[1]-self.at[0])
            return (t-self.liftoff_wall)*rate, True
        i=min(max(0,bisect.bisect_right(self.clock_t,t)-1),len(self.clock_t)-2)
        u=float(np.interp(t,self.clock_t,self.clock_u))
        # A logger handoff interval has no continuously observed warp schedule.
        approximate=any(a<t<b for a,b in getattr(self,'clock_gaps',
            [(self.clock_t[j],self.clock_t[j+1]) for j in range(len(self.clock_t)-1)
             if self.clock_t[j+1]-self.clock_t[j]>15]))
        return u-self.liftoff_ut, approximate

    def sample(self,t):
        i=min(max(0,bisect.bisect_right(self.times,t)-1),len(self.times)-2)
        a,b=self.rows[i:i+2]; gap=b['time']-a['time']
        if t<self.times[0] or t>self.times[-1]+2 or gap>15 and t>a['time']+2:
            return None
        f=max(0,min(1,(t-a['time'])/max(1e-9,gap)))
        d={}
        for k,v in self.values.items():
            x,y=v[i:i+2]
            if k in ('heading','roll','vel_heading','longitude'):
                d[k]=x+old.wrap180(y-x)*f
            else:d[k]=x+(y-x)*f
        d['mode']=a['mode'];d['i']=i
        d['roll']=old.wrap180(d['roll'])
        if not math.isfinite(d['vel_heading']): d['vel_heading']=d['heading']
        return d


class MissionHUD:
    def __init__(self,W,H,timeline):
        self.W,self.H,self.data=W,H,timeline
        self.k=H/1200
        k=self.k
        self.f_s,self.f_m,self.f_l=[old.font(old.MONO,round(n*k)) for n in (15,19,27)]
        self.sans=old.font(old.SANS,round(16*k))
        self.width=round(356*k);self.margin=round(24*k)
        # Reuse the existing attitude instrument, with measured native attitude.
        self.navball=old.Hud.navball.__get__(self)
        m=self.margin;R=round(102*k);cy=H-m-round(197*k)
        # Transfer just the HUD panels, not a full mostly-transparent 16-bit
        # video frame. Packing changes no pixels or instrument placement.
        bounds=[(m,m,m+260*k,m+56*k),
                (m,cy-R-38*k,m+280*k,H-m),
                (m+298*k,cy-56*k,m+488*k,H-m),
                (W-m-self.width,m,W-m,m+644*k)]
        self.regions=[(math.floor(a)-2,math.floor(b)-2,math.ceil(c)+3,math.ceil(d)+3)
                      for a,b,c,d in bounds]
        self.packed=[];yy=0
        # Simple vertical atlas; narrow enough to avoid full-screen conversion.
        for a,b,c,d in self.regions:
            self.packed.append((0,yy));yy+=d-b
        self.atlas_size=(max(c-a for a,b,c,d in self.regions),yy)

    def atlas_frame(self,t):
        full=self.frame(t)
        atlas=Image.new('RGBA',self.atlas_size)
        for box,xy in zip(self.regions,self.packed):atlas.paste(full.crop(box),xy)
        return atlas

    def panel(self,dr,box):
        dr.rounded_rectangle(tuple(round(v) for v in box),round(12*self.k),fill=old.PANEL)

    def text(self,dr,xy,text,font=None,fill=old.WHITE,anchor=None):
        dr.text(tuple(round(v) for v in xy),text,font=font or self.f_s,fill=fill,anchor=anchor)

    def frame(self,t):
        k,m=self.k,self.margin
        img=Image.new('RGBA',(self.W,self.H));dr=ImageDraw.Draw(img)
        met,approx=self.data.clock(t)
        sign='−' if met<0 else '+';sec=int(abs(met))
        clock=f'T{sign}{sec//3600:02}:{sec//60%60:02}:{sec%60:02}'
        if approx:clock+=' ~'
        self.panel(dr,(m,m,m+260*k,m+56*k))
        self.text(dr,(m+16*k,m+28*k),clock,self.f_l,anchor='lm')
        d=self.data.sample(t)
        R=round(102*k);cx=m+round(140*k);cy=self.H-m-round(197*k)
        self.panel(dr,(m,cy-R-38*k,m+280*k,self.H-m))
        if d is not None:
            self.navball(img,cx,cy,R,d)
        else:
            dr.ellipse((cx-R,cy-R,cx+R,cy+R),outline=old.DIM,width=2)
            self.text(dr,(cx,cy),'—',self.f_l,anchor='mm')
        dr=ImageDraw.Draw(img)
        y=cy+R+24*k
        self.text(dr,(m+20*k,y),'SPEED · SFC',self.sans,old.DIM)
        self.text(dr,(m+199*k,y),'G',self.sans,old.DIM)
        speed=d['speed'] if d else float('nan');g=d['gForce'] if d else float('nan')
        self.text(dr,(m+20*k,y+25*k),f'{speed:,.0f} m/s' if math.isfinite(speed) else '—',self.f_l)
        self.text(dr,(m+199*k,y+25*k),f'{g:.2f}' if math.isfinite(g) else '—',self.f_l)
        if d and d['mode']=='landing':
            self.landing_map(img,d)
            # Preserve useful landing readouts, adjacent to the navball.
            x=m+298*k
            self.panel(dr,(x,cy-56*k,x+190*k,self.H-m))
            for j,(label,value) in enumerate((('ALT · ASL',f"{d['altitude']:,.0f} m"),
                    ('MACH',f"{d['mach']:.2f}"),('SITE RANGE',f"{d['rangeToSite']/1000:.1f} km"))):
                self.text(dr,(x+15*k,cy-40*k+j*68*k),label,self.sans,old.DIM)
                self.text(dr,(x+15*k,cy-16*k+j*68*k),value,self.f_l)
        else:
            self.launch_panels(img,t)
        return img

    def launch_panels(self,img,t):
        k,m=self.k,self.margin;w=self.width;x=self.W-m-w;y=m
        dr=ImageDraw.Draw(img);h=300*k
        self.panel(dr,(x,y,x+w,y+h))
        self.text(dr,(x+16*k,y+16*k),'FLOWN TRAJECTORY',self.sans,old.DIM)
        cx=x+w/2;cy=y+163*k;R=98*k
        dr.ellipse((cx-R,cy-R,cx+R,cy+R),fill=(22,91,169,235),outline=(99,179,250,255),width=2)
        self.text(dr,(cx,cy),'KERBIN',self.sans,(167,210,249,255),'mm')
        n=bisect.bisect_right(self.data.launch_times,t)
        pts=[(cx+p[0]*R,cy+p[1]*R) for p in self.data.launch_xy[:n]]
        # Positions in the native segment are measured, never a propagated conic.
        native=[r for r in self.data.native_xy if r[0]<=t]
        if native:
            # Switch from launch's speed-integrated profile to the genuinely
            # observed planet-fixed positions. Do not invent a geographical
            # alignment for ascent, whose logger did not record positions.
            nn=[(cx+p[1]*R,cy+p[2]*R) for p in native]
            for j in range(1,len(nn)):
                if native[j][0]-native[j-1][0]<=15:
                    dr.line((nn[j-1],nn[j]),fill=old.GREEN,width=max(2,round(3*k)))
            px,py=nn[-1];dr.ellipse((px-4*k,py-4*k,px+4*k,py+4*k),fill=old.GREEN)
        else:
            if len(pts)>1:dr.line(pts,fill=old.GREEN,width=max(2,round(3*k)))
            if pts:
                px,py=pts[-1];dr.ellipse((px-3*k,py-3*k,px+3*k,py+3*k),fill=old.GREEN)
        self.text(dr,(cx,y+h-20*k),'2D · RECORDED POSITIONS' if native else '2D · SPEED-INTEGRATED',self.f_s,old.DIM,'mm')
        y+=h+12*k
        n=max(0,n)
        self.chart(dr,(x,y,w,155*k),'DYNAMIC PRESSURE',self.data.launch_ut[:n]-self.data.liftoff_ut,
                   self.data.launch_q[:n],'s MET','kPa',old.ACC)
        y+=167*k
        self.chart(dr,(x,y,w,165*k),'ALTITUDE / DOWNRANGE',self.data.downrange[:n],
                   self.data.launch_alt[:n],'km downrange','km ASL',old.GREEN)

    def chart(self,dr,box,title,xs,ys,xunit,yunit,color):
        x,y,w,h=box;k=self.k
        self.panel(dr,(x,y,x+w,y+h));self.text(dr,(x+16*k,y+14*k),title,self.sans,old.DIM)
        left=x+49*k;right=x+w-18*k;top=y+47*k;bottom=y+h-31*k
        xmax=max(1,float(xs[-1]) if len(xs) else 1)
        ymax=max(1,math.ceil(float(np.max(ys))/10)*10 if len(ys) else 1)
        for fraction in (0,.5,1):
            yy=bottom-fraction*(bottom-top)
            dr.line((left,yy,right,yy),fill=(170,190,210,42),width=1)
        dr.line((left,top,left,bottom,right,bottom),fill=(190,200,215,140),width=1)
        self.text(dr,(left-7*k,top),f'{ymax:g}',self.f_s,old.DIM,'rm')
        self.text(dr,(left-7*k,bottom),'0',self.f_s,old.DIM,'rm')
        self.text(dr,(left,y+h-13*k),yunit,self.f_s,old.DIM,'lm')
        self.text(dr,(right,y+h-13*k),f'{xmax:.0f} {xunit}',self.f_s,old.DIM,'rm')
        if len(xs)>1:
            # Pixel-column extrema preserve Q peaks without drawing thousands
            # of overlapping points on every video frame.
            stride=max(1,len(xs)//round(w*2))
            idx=list(range(0,len(xs),stride))
            if idx[-1]!=len(xs)-1:idx.append(len(xs)-1)
            pts=[(left+float(xs[i])/xmax*(right-left),bottom-float(ys[i])/ymax*(bottom-top)) for i in idx]
            dr.line(pts,fill=color,width=max(2,round(2*k)))
            px,py=pts[-1];dr.ellipse((px-3*k,py-3*k,px+3*k,py+3*k),fill=color)

    def landing_map(self,img,d):
        k,m=self.k,self.margin;w=self.width;h=222*k;x=self.W-m-w;y=m
        dr=ImageDraw.Draw(img);self.panel(dr,(x,y,x+w,y+h))
        data=self.data
        if d['rangeToSite']>45000:
            # Geographic track with equal north/east distance scale.
            rows=[r for r in data.rows[:d['i']+1] if r.get('mode')=='landing' and 'latitude' in r]
            lat0=self.data.site['latitude'];lon0=self.data.site['longitude']
            pts=[(math.radians(old.wrap180(r['longitude']-lon0))*data.radius*math.cos(math.radians(lat0)),
                  math.radians(r['latitude']-lat0)*data.radius) for r in rows]
            span=max([45000]+[abs(v)*2.4 for p in pts for v in p]);scale=min(w-35*k,h-65*k)/span
            ox=x+w*.5;oy=y+h*.55
            track=[(ox+p[0]*scale,oy-p[1]*scale) for p in pts]
            label='GROUND TRACK · N ↑'
            dr.rectangle((ox-4*k,oy-4*k,ox+4*k,oy+4*k),outline=old.WARN,width=2)
        else:
            span=75000 if d['rangeToSite']>15000 else 18000 if d['rangeToSite']>4000 else 6000
            scale=(w-35*k)/span;ox=x+w*.78;oy=y+h*.55
            rows=[r for r in data.rows[:d['i']+1] if r.get('mode')=='landing' and r.get('rangeToSite',1e10)<span*1.3]
            track=[(ox+r['runwayAlongTrack']*scale,oy+r['runwayCrossTrack']*scale) for r in rows]
            length=data.runway['length'];width=max(3*k,data.runway['width']*scale)
            # The native runway datum is its center, not its threshold.
            dr.rectangle((ox-length/2*scale,oy-width/2,ox+length/2*scale,oy+width/2),fill=old.WARN)
            label=f"{data.runway['name'].upper()} · {span/1000:g} km"
        self.text(dr,(x+16*k,y+16*k),label,self.sans,old.DIM)
        # Clip trajectory to the map panel before compositing.
        layer=Image.new('RGBA',img.size);ld=ImageDraw.Draw(layer)
        if len(track)>1:ld.line(track,fill=old.ACC,width=max(2,round(3*k)))
        if track:
            px,py=track[-1];ld.ellipse((px-4*k,py-4*k,px+4*k,py+4*k),fill=old.WHITE)
        crop=layer.crop((round(x+8*k),round(y+40*k),round(x+w-8*k),round(y+h-8*k)))
        img.alpha_composite(crop,(round(x+8*k),round(y+40*k)))


def sdr_frame(path,t,crop,W,H):
    """Preview only: PQ/BT.2020 -> 100-nit sRGB, without relying on zscale."""
    cw,ch,cx,cy=crop
    raw=subprocess.check_output(['ffmpeg','-v','error','-ss',str(t),'-i',str(path),'-vf',
        f'crop={cw}:{ch}:{cx}:{cy},scale={W}:{H}:in_color_matrix=bt2020:out_range=full',
        '-pix_fmt','rgb48le','-frames:v','1','-f','rawvideo','pipe:1'])
    a=np.frombuffer(raw,np.uint16).reshape(H,W,3).astype(np.float32)/65535
    p=a**(1/78.84375)
    lin=(np.maximum(p-.8359375,0)/np.maximum(18.8515625-18.6875*p,1e-9))**(1/.1593017578125)*100
    lin=np.clip(lin@np.linalg.inv(old.M709_2020).T,0,1)
    rgb=np.where(lin<=.0031308,12.92*lin,1.055*lin**(1/2.4)-.055)
    return Image.fromarray((rgb*255+.5).astype(np.uint8)).convert('RGBA')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recording',type=Path)
    ap.add_argument('--launch',type=Path,required=True)
    ap.add_argument('--landing',type=Path,required=True)
    ap.add_argument('--native',type=Path,required=True)
    ap.add_argument('--crop',required=True,help='W:H:X:Y interior of recording window')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--width',type=int,default=1920)
    ap.add_argument('--fps',type=int,default=60)
    ap.add_argument('--hud-fps',type=int,default=24)
    ap.add_argument('--bitrate',default='35M')
    ap.add_argument('--preview',type=float,nargs='+')
    ap.add_argument('--seek',type=float,default=0)
    ap.add_argument('--duration',type=float)
    ap.add_argument('--no-audio',action='store_true',help='Video-only segment for lossless assembly')
    args=ap.parse_args()
    rec=args.recording.resolve();out=args.output.resolve()
    meta=old.ffprobe(rec)
    if meta['transfer']!='smpte2084':raise SystemExit('This export requires an HDR10/PQ source.')
    start=old.recording_start_epoch(rec)
    crop=tuple(map(int,args.crop.split(':')));cw,ch,cx,cy=crop
    W=args.width;H=round(W*ch/cw/2)*2
    timeline=Timeline(args.launch,args.landing,args.native)
    hud=MissionHUD(W,H,timeline)
    out.parent.mkdir(parents=True,exist_ok=True)
    if args.preview:
        for t in args.preview:
            im=sdr_frame(rec,t,crop,W,H)
            im.alpha_composite(hud.frame(start+t))
            path=out.with_name(out.stem+f'-preview-{t:g}.jpg')
            im.convert('RGB').save(path,quality=94)
            print(path,flush=True)
        return
    duration=min(meta['duration']-args.seek,args.duration or meta['duration'])
    if out.exists():raise SystemExit(f'Refusing to overwrite {out}')
    white=meta['max_cll'] or 203.0
    graph=(f'[0:v]fps={args.fps},crop={cw}:{ch}:{cx}:{cy},'
           f'scale={W}:{H}:flags=lanczos:in_color_matrix=bt2020:out_color_matrix=bt2020,'
           'format=yuv444p10le[v];'
           '[1:v]scale=in_range=full:out_color_matrix=bt2020:out_range=tv,format=yuva444p10le[h];'
           '[h]split=4[h0][h1][h2][h3];')
    previous='v'
    for i,(box,xy) in enumerate(zip(hud.regions,hud.packed)):
        x0,y0,x1,y1=box;ax,ay=xy
        graph+=f'[h{i}]crop={x1-x0}:{y1-y0}:{ax}:{ay}[p{i}];'
        graph+=f'[{previous}][p{i}]overlay=x={x0}:y={y0}:format=yuv444p10:eof_action=pass[v{i}];'
        previous=f'v{i}'
    graph+=f'[{previous}]format=p010le[o]'
    aw,ah=hud.atlas_size
    cmd=['ffmpeg','-hide_banner','-v','warning','-stats_period','15','-stats',
         '-ss',str(args.seek),'-i',str(rec),'-f','rawvideo','-pixel_format','rgba64le',
         '-video_size',f'{aw}x{ah}','-framerate',str(args.hud_fps),'-i','pipe:0',
         '-filter_complex_threads','2','-filter_complex',graph,'-map','[o]',
         *([] if args.no_audio else ['-map','0:a?','-c:a','copy']),
         '-c:v','hevc_videotoolbox','-profile:v','main10',
         '-b:v',args.bitrate,'-tag:v','hvc1','-color_primaries','bt2020',
         '-color_trc','smpte2084','-colorspace','bt2020nc','-color_range','tv',
         '-map_metadata','0','-movflags','+faststart','-t',str(duration),str(out)]
    manifest=dict(recording=str(rec),sourceMetadata=meta,crop=crop,width=W,height=H,
        fps=args.fps,hudFPS=args.hud_fps,recordingStart=start,liftoffWall=timeline.liftoff_wall,
        reconstructedWarpClock=timeline.warp_clock,
        liftoffUT=timeline.liftoff_ut,launch=str(args.launch),landing=str(args.landing),
        native=str(args.native),command=cmd,sourceRevision=subprocess.check_output(
            ['git','rev-parse','HEAD'],text=True).strip(),
        notes=['Launch trajectory is a 2D integration of measured surface horizontal speed/radius over UT.',
               'No trajectory segments bridge telemetry gaps; no future orbit is displayed.',
               'Clock uses paired epoch/UT samples; ~ marks sparse intervals, including rails warp handoff.',
               'Native attitude comes from measured FRD quaternion, not FPA+AoA.',
               'HDR remains 10-bit BT.2020/PQ; HUD white follows source MaxCLL.',
               'No landing-success claim is implied by the recording or HUD.'])
    out.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Export {duration:.2f}s, {W}x{H}, HDR10, HUD white {white:g} nits -> {out}',flush=True)
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    begun=time.monotonic()
    try:
        total=math.ceil(duration*args.hud_fps)
        for f in range(total):
            proc.stdin.write(old.to_pq_rgba64(hud.atlas_frame(start+args.seek+f/args.hud_fps),white))
            if f and f%(args.hud_fps*30)==0:
                elapsed=time.monotonic()-begun
                print(f'HUD {f/args.hud_fps:.0f}/{duration:.0f}s; elapsed {elapsed:.0f}s; '
                      f'ETA {elapsed*(total-f)/f:.0f}s',flush=True)
    except BrokenPipeError:
        pass
    finally:
        proc.stdin.close()
    code=proc.wait()
    if code:raise SystemExit(code)
    manifest['outputMetadata']=old.ffprobe(out)
    manifest['elapsedSeconds']=time.monotonic()-begun
    out.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Finished in {manifest["elapsedSeconds"]:.0f}s: {out}',flush=True)


if __name__=='__main__':main()
