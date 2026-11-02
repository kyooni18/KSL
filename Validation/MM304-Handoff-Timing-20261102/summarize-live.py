"""Offline evidence extraction; never connects to KSP or commands controls."""
import json,math,hashlib,collections
from pathlib import Path
root=Path(__file__).resolve().parents[2]
run=root/'runs/20261102T140105Z-dessert-recovery-fix-live'
config=json.loads((run/'live.jsonl.configuration.json').read_text());rw=config['runway'];radius=config['planet']['radius'];lat,lon,heading=(rw[k] for k in ['latitude','longitude','heading'])
up=(math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat));north=(-math.sin(lat)*math.cos(lon),-math.sin(lat)*math.sin(lon),math.cos(lat));east=(-math.sin(lon),math.cos(lon),0)
along=tuple(n*math.cos(heading)+e*math.sin(heading) for n,e in zip(north,east))
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def norm(a):return math.sqrt(dot(a,a))
def cross(a,b):return(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
right=cross(along,up)
def sample(r):
 s=r['state'];p=tuple(s['position'][k] for k in 'xyz');u=tuple(x/norm(p) for x in p);d=dot(up,u);t=tuple(x-y*d for x,y in zip(u,up));angle=math.atan2(norm(t),d);scale=radius*angle/norm(t) if norm(t)>1e-12 else 0
 return {'UT':s['time'],'phase':r['guidance']['phase'],'ASL':norm(p)-radius,'speed':s['aerodynamics']['airspeed'],'along':dot(t,along)*scale,'cross':dot(t,right)*scale,'thresholdRange':radius*angle,'parts':s['vesselConfiguration']['partCount'],'contact':s['contact'],'throttle':s['systems']['throttle']}
phases=collections.Counter();first={};closest=None;ground=None;loss=None;last=None;unbracketed=0;taem=collections.Counter();previousParts=None
for line in (run/'live.jsonl').open():
 r=json.loads(line);s=sample(r);g=r['guidance'];phases[s['phase']]+=1;first.setdefault(s['phase'],s)
 if closest is None or s['thresholdRange']<closest['thresholdRange']:closest=s
 if ground is None and s['contact']!='airborne':ground=s
 if loss is None and previousParts is not None and s['parts']<previousParts:loss=s
 previousParts=s['parts'];last=s
 if g.get('entry',{}).get('rangeReferenceBracketed') is False:unbracketed+=1
 if s['phase'] in ['acquisition','headingAlignment']:taem[s['phase']]+=1
plan=json.loads((run/'live.jsonl.deorbit-plan.json').read_text())
files=['live.jsonl','live.jsonl.configuration.json','live.jsonl.deorbit-plan.json','ksl-landing']
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
out={'run':str(run.relative_to(root)),'flownRevision':json.loads((run/'provenance.json').read_text())['revision'],'SHA256':{f:sha(run/f) for f in files},'selectedRunway':rw['name'],'window':plan['window'],'entryScreen':plan.get('entryScreen'),'nominalCase':plan['cases'][0],'phases':dict(phases),'firstPhaseStates':first,'closestThreshold':closest,'firstGround':ground,'firstPartLoss':loss,'lastNative':last,'unbracketedEntryRecords':unbracketed,'taemPhases':dict(taem),'handoffTiming':json.loads((run/'actual-handoff-timing.json').read_text()),'postflight':json.loads((run/'postflight-inspect.json').read_text()),'exit':json.loads((run/'exit.json').read_text())}
(root/'Validation/MM304-Handoff-Timing-20261102/live-results.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ['phases','closestThreshold','firstGround','firstPartLoss','lastNative','handoffTiming']},indent=2))
