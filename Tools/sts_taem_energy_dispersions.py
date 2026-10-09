import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sts_taem_energy_proto import run
from concurrent.futures import ProcessPoolExecutor
T=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'ShuttleSim/reference-model/sts_aero_live20261010.csv')
cases={
 'nom':dict(x=-59000,y=2000,h=26900,v=878,gamma=-7.4,course_deg=112),
 'lowE':dict(x=-59000,y=2000,h=24500,v=810,gamma=-8,course_deg=112),
 'hiE':dict(x=-59000,y=2000,h=29000,v=950,gamma=-7,course_deg=112),
 'close':dict(x=-45000,y=-1000,h=24000,v=800,gamma=-9,course_deg=100),
 'far':dict(x=-72000,y=3000,h=27500,v=900,gamma=-7,course_deg=105),
 'xN':dict(x=-59000,y=6000,h=26900,v=878,gamma=-7.4,course_deg=118),
 'xS':dict(x=-59000,y=-4000,h=26900,v=878,gamma=-7.4,course_deg=95),
 'cl-15':dict(x=-59000,y=2000,h=26900,v=878,gamma=-7.4,course_deg=112,cl=0.85),
 'cd+15':dict(x=-59000,y=2000,h=26900,v=878,gamma=-7.4,course_deg=112,cd=1.15),
 'cd-15':dict(x=-59000,y=2000,h=26900,v=878,gamma=-7.4,course_deg=112,cd=0.85),
}
def go(k):
    return k, run(table=T, **cases[k])
if __name__=='__main__':
  with ProcessPoolExecutor(8) as ex:
    for k,r in ex.map(go, cases):
        print(f"{k:7s}", r, flush=True)
