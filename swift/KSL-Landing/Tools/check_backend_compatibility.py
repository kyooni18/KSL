#!/usr/bin/env python3
"""Exercise the unchanged KSL BackendClient against the Swift wrapper."""
from __future__ import annotations
import argparse,json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from ksl.backend import BackendClient,BackendError

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connect-read-only',action='store_true')
    p.add_argument('--output',type=pathlib.Path,required=True)
    args=p.parse_args()
    steps=[]
    client=BackendClient(ROOT/'swift/KSL-Landing/bin/landing_backend')
    try:
        ready=client._read_message(20)
        assert ready['type']=='ready' and ready['protocolVersion']==1
        steps.append('ready/protocol-v1')
        result=client.send('updateConfiguration',configuration={})
        assert 'snapshot' in result
        steps.append('updateConfiguration/legacy-response')
        if args.connect_read_only:
            response=client.send('connect',timeout=60)
            assert response['snapshot']['connectionStatus']=='connected'
            steps.append('read-only-connect')
        snap=client.send('getSnapshot')['snapshot']
        assert all(k in snap for k in ['phase','automationEngaged','connectionStatus','configuration','telemetry','command'])
        steps.append('snapshot-contract')
        try:client.send('deliberatelyUnknownCompatibilityProbe')
        except BackendError:steps.append('unknown-command-rejected')
        else:raise AssertionError('Unknown command accepted')
        disconnected=client.send('disconnect')['snapshot']
        assert disconnected['connectionStatus']=='disconnected' and not disconnected['automationEngaged']
        steps.append('disconnect/legacy-response')
    finally:client.shutdown()
    assert client.process.returncode==0
    steps.append('safe-shutdown')
    result={'passed':True,'client':'unmodified ksl.backend.BackendClient','checks':steps,
            'flightArmed':False,'processExitCode':client.process.returncode}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
if __name__=='__main__':main()
