#!/usr/bin/env python3
import json,math,os,socket,subprocess,sys,threading

root=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
bin=os.path.join(root,'build','shuttlesim')
received={18796:[],18797:[]}
first_primary=threading.Event()
second_primary=threading.Event()

def recv(port):
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.bind(('127.0.0.1',port))
    s.settimeout(2)
    try:
        while len(received[port])<2:
            data,_=s.recvfrom(8192)
            received[port].append(json.loads(data))
            if port==18796 and len(received[port])==1:
                first_primary.set()
            if port==18796 and len(received[port])==2:
                second_primary.set()
    except socket.timeout:
        pass
    finally:
        s.close()

threads=[threading.Thread(target=recv,args=(p,)) for p in received]
for t in threads:t.start()

# Lockstep turns command reception into a protocol event rather than a race
# against process startup or host scheduling.  Two published frames are enough:
# one proves the telemetry socket is live, the next must contain the command.
# Choose the simulation horizon from the discretization: the first publication
# occurs after one physics step, and the second is the first step at/after one
# telemetry period.  One further step lets the loop exit before a third
# lockstep publication can block.
physics_dt=0.02
telemetry_hz=20.0
second_publish_step=math.ceil((1.0/telemetry_hz)/physics_dt)
max_sim_time=(second_publish_step+1)*physics_dt
cmd=[bin,'--scenario',os.path.join(root,'scenarios','orbit86km.ini'),
     '--dt',str(physics_dt),'--rate','max','--telemetry-hz',str(telemetry_hz),
     '--max-sim-time',str(max_sim_time),'--lockstep',
     '--command-port','18795','--telemetry-port','18796','--web-telemetry-port','18797']
p=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try:
    if not first_primary.wait(timeout=2):
        raise SystemExit('no initial telemetry handshake on primary UDP sink')
    s.sendto(b'{"type":"attitude_command","aoa_deg":23.5,"bank_deg":-41.0,"step":true}',
             ('127.0.0.1',18795))
    if not second_primary.wait(timeout=2):
        raise SystemExit('no post-command telemetry frame on primary UDP sink')
    # The second telemetry publication is itself a lockstep boundary.  Release
    # one final physics step so max-sim-time can terminate the process cleanly.
    s.sendto(b'{"type":"resume"}',('127.0.0.1',18795))
    out,err=p.communicate(timeout=3)
finally:
    s.close()
    if p.poll() is None:
        p.terminate()
        try:p.wait(timeout=1)
        except subprocess.TimeoutExpired:p.kill()

for t in threads:t.join(timeout=3)
rows=[json.loads(x) for x in out.splitlines() if x.startswith('{')]
if not rows:raise SystemExit('no stdout telemetry')
if not any(abs(r['attitude'].get('requested_aoa_deg', float('nan'))-23.5)<0.01 and
           abs(r['attitude'].get('requested_bank_deg', float('nan'))+41.0)<0.01 for r in rows):
    print(out);print(err,file=sys.stderr);raise SystemExit('UDP attitude command not observed')
for previous, current in zip(rows, rows[1:]):
    dt=current['sim_time']-previous['sim_time']
    if dt <= 0:
        continue
    pa=previous['attitude']['cmd_aoa_deg']
    ca=current['attitude']['cmd_aoa_deg']
    pb=previous['attitude']['cmd_bank_deg']
    cb=current['attitude']['cmd_bank_deg']
    if abs(ca-pa)>previous['attitude']['max_pitch_rate_deg_s']*dt+1e-6:
        raise SystemExit('simulator AoA target exceeded its rate limit')
    if abs(cb-pb)>previous['attitude']['max_roll_rate_deg_s']*dt+1e-6:
        raise SystemExit('simulator bank target exceeded its rate limit')
response_fields=(
    'aoa_rate_deg_s','bank_rate_deg_s',
    'pitch_wn_s_inv','pitch_zeta','roll_wn_s_inv','roll_zeta',
    'max_pitch_rate_deg_s','max_roll_rate_deg_s',
    'max_pitch_accel_deg_s2','max_roll_accel_deg_s2',
)
for port,items in received.items():
    if len(items)<2:raise SystemExit(f'insufficient telemetry on port {port}: {len(items)} frames')
    if items[0].get('source')!='sim' or items[0].get('schema')!=1:
        raise SystemExit(f'bad telemetry envelope on {port}')
    attitude=items[0].get('attitude') or {}
    missing=[name for name in response_fields
             if name not in attitude or not isinstance(attitude[name],(int,float))
             or not math.isfinite(attitude[name])]
    if missing:raise SystemExit(f'missing/non-finite attitude response fields on {port}: {missing}')
    for name in response_fields[2:]:
        if attitude[name]<=0:
            raise SystemExit(f'non-positive attitude response parameter {name} on {port}')
print('protocol_test: PASS dual telemetry sinks + response model + lockstep command input')
