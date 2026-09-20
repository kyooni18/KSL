#!/usr/bin/env python3
import json,os,subprocess
root=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
cmd=[os.path.join(root,'build','shuttlesim'),'--scenario',os.path.join(root,'tests','orbit86km_vacuum.ini'),'--rate','max','--telemetry-hz','1','--max-sim-time','1900','--command-port','0','--telemetry-port','0','--web-telemetry-port','0']
p=subprocess.run(cmd,capture_output=True,text=True,check=True)
rows=[json.loads(x) for x in p.stdout.splitlines() if x.startswith('{')]
alts=[r['position']['altitude_m'] for r in rows]
span=max(alts)-min(alts)
if span>0.5:raise SystemExit(f'orbit altitude drift too large: {span:.6f} m')
print(f'orbit_test: PASS altitude span={span:.6f} m')
