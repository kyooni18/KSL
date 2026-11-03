import hashlib,json,re,socket,time
from pathlib import Path
import krpc
socket.setdefaulttimeout(10)
def descending_crossing(previous, current):
 return (previous is not None and current['ut'] > previous['ut']
         and previous['altitudeASL'] > 72000 >= current['altitudeASL'])
run=Path(Path('/tmp/ksl-authorized-live-run').read_text().strip())
name='ksl-'+run.name.replace('-deorbit-coverage-reflight','')+'-descent-72km'
savedir=Path('/Users/kyooni18/Library/Application Support/Steam/steamapps/common/Kerbal Space Program/saves/shuttletester')
save=savedir/(name+'.sfs')
assert not save.exists(), 'Refusing to overwrite checkpoint'
end=time.monotonic()+7200;c=None;previous=None;armed=False
print(json.dumps({'status':'watching','name':name,'triggerASL':72000,'controls':'read-only; save once on descending crossing'}),flush=True)
try:
 while time.monotonic()<end:
  try:
   if c is None:
    c=krpc.connect(name='KSL authorized 72km checkpoint',address='127.0.0.1',rpc_port=50000)
   sc=c.space_center;v=sc.active_vessel
   if v.name!='STS' or v.orbit.body.name!='Kerbin' or len(v.parts.all)!=26:
    raise RuntimeError('Checkpoint vessel identity or integrity changed')
   f=v.flight(v.orbit.body.reference_frame);ut=sc.ut;h=f.mean_altitude;vs=f.vertical_speed
   if h>72000:armed=True
   if descending_crossing(previous, {'ut':ut,'altitudeASL':h}):
    before={'ut':ut,'altitudeASL':h,'verticalSpeed':vs,'mass':v.mass,'partCount':len(v.parts.all),'paused':c.krpc.paused,'railsWarp':sc.rails_warp_factor}
    save_error = None
    try:sc.save(name)
    except Exception as e:save_error = str(e)
    deadline=time.monotonic()+15
    while not save.exists() and time.monotonic()<deadline:time.sleep(.1)
    if not save.exists():raise RuntimeError('Save RPC returned but checkpoint file missing')
    raw=save.read_bytes();match=re.search(rb'\bUT = ([^\r\n]+)',raw)
    headers=re.findall(rb'\bVESSEL\s*\{([^{}]*)',raw)
    header=next(hd for hd in headers if re.search(rb'(?m)^\s*name = STS\s*$',hd))
    altitude_match=re.search(rb'(?m)^\s*alt = ([^\r\n]+)',header)
    saved_altitude=float(altitude_match.group(1))
    # KSP's saved vessel header is sampled separately from Flight. Retain its
    # exact altitude and epoch; saveUT must bracket the measured crossing.
    assert previous['ut'] <= float(match.group(1)), 'Save predates the observed crossing interval'
    result={'status':'saved','name':name,'path':str(save),'sha256':hashlib.sha256(raw).hexdigest(),'saveUT':float(match.group(1)) if match else None,'before':before,'previous':previous,'after':{'ut':sc.ut,'altitudeASL':v.flight().mean_altitude},'sourceRevision':json.loads((run/'provenance.json').read_text())['revision'],'controlsChanged':False,'savedVesselAltitudeASL':saved_altitude,'saveRPCError':save_error,'verticalSpeedReferenceFrame':'body-fixed; crossing also checked by decreasing measured ASL altitude'}
    (run/'checkpoint-72km.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True);break
   previous={'ut':ut,'altitudeASL':h,'verticalSpeed':vs}
   if armed and h<70000:raise RuntimeError('Descending 72km checkpoint missed')
   if (run/'exit.json').exists():raise RuntimeError('Flight controller ended before checkpoint')
   time.sleep(.05 if 70000<h<74000 else .5)
  except (ConnectionError,OSError,EOFError) as e:
   print(json.dumps({'status':'reconnecting','error':str(e)}),flush=True)
   if c:
    try:c.close()
    except Exception:pass
   c=None;time.sleep(.5)
 else:raise TimeoutError('72km watcher deadline')
except Exception as e:
 (run/'checkpoint-72km-error.json').write_text(json.dumps({'error':str(e),'previous':previous})+'\n')
 raise
finally:
 if c:c.close()
