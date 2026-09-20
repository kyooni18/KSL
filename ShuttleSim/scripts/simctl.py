#!/usr/bin/env python3
import argparse,json,socket

def send(payload, host, port):
    data=json.dumps(payload,separators=(",",":")).encode()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.sendto(data,(host,port))
    print(data.decode())

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--host",default="127.0.0.1")
    ap.add_argument("--port",type=int,default=8795)
    sub=ap.add_subparsers(dest="cmd",required=True)
    sub.add_parser("pause");sub.add_parser("resume")
    a=sub.add_parser("attitude");a.add_argument("--aoa",type=float,required=True);a.add_argument("--bank",type=float,required=True)
    g=sub.add_parser("gear");g.add_argument("state",choices=("up","down"))
    args=ap.parse_args()
    if args.cmd in ("pause","resume"): payload={"type":args.cmd}
    elif args.cmd=="attitude": payload={"type":"attitude_command","aoa_deg":args.aoa,"bank_deg":args.bank}
    else: payload={"type":"gear","gear_down":args.state=="down"}
    send(payload,args.host,args.port)
if __name__=="__main__":main()
