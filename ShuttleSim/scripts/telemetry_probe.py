#!/usr/bin/env python3
import argparse,json,socket
ap=argparse.ArgumentParser()
ap.add_argument("--port",type=int,default=8797)
ap.add_argument("--count",type=int,default=1)
ap.add_argument("--timeout",type=float,default=5.0)
a=ap.parse_args()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(("127.0.0.1",a.port));s.settimeout(a.timeout)
for _ in range(a.count):
    data,_=s.recvfrom(65535)
    obj=json.loads(data)
    print(json.dumps(obj,separators=(",",":")))
