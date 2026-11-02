#!/usr/bin/env python3
"""Normalize recorded KSP slowdown to 1x UT, preserving logged rails timewarp.

Retimestamps compressed video packets without changing their image data. The
already-burned HUD follows the same recording timeline, so it stays synchronized.
Rubber Band follows the identical sample-frame map for pitch-preserved audio.
Requires the video extra and the official rubberband CLI; never connects to KSP.
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import time

import av
import numpy as np

try:
    from .mission_video import Timeline
except ImportError:
    from mission_video import Timeline


class PlaybackMap:
    def __init__(self, seconds, game_seconds, warp_intervals=()):
        self.source=np.asarray(seconds,dtype=float)
        game=np.asarray(game_seconds,dtype=float)
        if len(self.source)<2 or len(game)!=len(self.source):
            raise ValueError('At least two paired clock samples are required')
        if np.any(np.diff(self.source)<=0) or np.any(np.diff(game)<=0):
            raise ValueError('Clock observations must increase monotonically')
        self.warp_intervals=list(warp_intervals)
        warp=np.zeros(len(game)-1,dtype=bool)
        for start,end in self.warp_intervals:
            # Callers include the warp boundaries as clock knots.
            warp|=(self.source[:-1]>=start)&(self.source[1:]<=end)
        steps=np.where(warp,np.diff(self.source),np.diff(game))
        self.target=np.concatenate(([0.],np.cumsum(steps)))

    def forward(self,seconds):
        return np.interp(seconds,self.source,self.target)

    def inverse(self,seconds):
        return np.interp(seconds,self.target,self.source)

    @classmethod
    def mission(cls,timeline,start,duration):
        end=start+duration
        walls=np.concatenate(([start],timeline.clock_t[
            (timeline.clock_t>start)&(timeline.clock_t<end)],[end]))
        game=np.array([timeline.clock(t)[0] for t in walls])
        intervals=[]
        if timeline.warp_clock:
            # First/last anchors bracket normal 1x control. The logged warp
            # itself starts at knot 1 and ends at the penultimate knot.
            intervals=[(timeline.warp_clock[1]['time']-start,
                        timeline.warp_clock[-2]['time']-start)]
        return cls(walls-start,game,intervals)

    def write_audio_map(self,path,sample_rate):
        pairs=np.rint(np.column_stack((self.source,self.target))*sample_rate).astype(np.int64)
        if np.any(np.diff(pairs,axis=0)<=0):
            raise ValueError('Audio clock map contains collapsed sample intervals')
        path.write_text(''.join(f'{a} {b}\n' for a,b in pairs))


def probe(path):
    return json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams',
        '-show_format','-of','json',str(path)]))


def copy_video(source,output,clock):
    """Copy every HEVC payload and HDR SEI; only presentation times change."""
    ih=hashlib.sha256();oh=hashlib.sha256();count=0
    with av.open(str(source)) as incoming, av.open(str(output),'w') as outgoing:
        original=incoming.streams.video[0]
        stream=outgoing.add_stream_from_template(original)
        stream.metadata.update(original.metadata)
        outgoing.metadata.update(incoming.metadata)
        tb=original.time_base
        stream.time_base=tb
        previous=None
        for packet in incoming.demux(original):
            if packet.dts is None:continue
            payload=bytes(packet);ih.update(payload)
            pts=float(packet.pts*packet.time_base)
            dts=float(packet.dts*packet.time_base)
            end=pts+float(packet.duration*packet.time_base)
            new_pts=round(float(clock.forward(pts))/tb)
            new_dts=round(float(clock.forward(dts))/tb)
            if previous is not None and new_dts<=previous:
                raise ValueError('Retime collapsed video decode timestamps')
            packet.pts=new_pts;packet.dts=new_dts
            packet.duration=max(1,round((float(clock.forward(end))-float(clock.forward(pts)))/tb))
            packet.time_base=tb;packet.stream=stream
            oh.update(bytes(packet));outgoing.mux(packet)
            previous=new_dts;count+=1
    return dict(frames=count,inputPayloadSHA256=ih.hexdigest(),outputPayloadSHA256=oh.hexdigest())


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('video',type=Path)
    ap.add_argument('--manifest',type=Path,help='Source HUD export sidecar; default beside video')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--prepare-only',action='store_true')
    ap.add_argument('--assemble-only',action='store_true')
    args=ap.parse_args()
    source=args.video.resolve();out=args.output.resolve()
    meta=probe(source)
    source_video=next(s for s in meta['streams'] if s['codec_type']=='video')
    manifest=json.loads((args.manifest or source.with_suffix('.json')).read_text())
    # Decode audio from the capture itself: a prior MP4 remux may shift its
    # initial AAC edit offset even while preserving compressed packet bytes.
    audio_source=Path(manifest['recording'])
    part=json.loads(Path(manifest['segments'][0]['path']).with_suffix('.json').read_text())
    timeline=Timeline(Path(part['launch']),Path(part['landing']),Path(part['native']))
    clock=PlaybackMap.mission(timeline,part['recordingStart'],float(meta['format']['duration']))
    work=out.parent/(out.stem+'-work');work.mkdir(parents=True,exist_ok=True)
    duration=float(clock.target[-1]);sample_rate=48000
    mapping=work/'audio-map.txt';clock.write_audio_map(mapping,sample_rate)
    plan=dict(source=str(source),audioSource=str(audio_source),output=str(out),sourceMetadata=meta,
        sourceSeconds=clock.source.tolist(),outputSeconds=clock.target.tolist(),
        preservedTimewarpIntervals=clock.warp_intervals,duration=duration,
        policy='1x simulation UT outside timewarp; keep logged warp at original video speed',
        notes=['No video payload is re-encoded. Output is intentionally variable frame rate.',
               'Existing HUD is retimed with its corresponding footage.',
               'Logging-gap clock estimates retain their visible ~ uncertainty marker.',
               'Audio follows the same time map with pitch preserved by Rubber Band.'],
        revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
    plan_path=work/'retime-plan.json'
    if plan_path.exists():
        previous=json.loads(plan_path.read_text())
        for key in ('videoCopy','elapsedVideoSeconds','audioCommand','assemblyCommand'):
            if key in previous:plan[key]=previous[key]
    plan_path.write_text(json.dumps(plan,indent=2)+'\n')
    print(f'{clock.source[-1]:.3f}s -> {duration:.3f}s; warp preserved {clock.warp_intervals}',flush=True)
    if not args.assemble_only:
        begun=time.monotonic()
        video=work/'video-retimed.mp4'
        if video.exists():raise SystemExit(f'Refusing to overwrite {video}')
        plan['videoCopy']=copy_video(source,video,clock)
        plan['elapsedVideoSeconds']=time.monotonic()-begun
        assert plan['videoCopy']['inputPayloadSHA256']==plan['videoCopy']['outputPayloadSHA256']
        plan_path.write_text(json.dumps(plan,indent=2)+'\n')
        print(f'Copied {plan["videoCopy"]["frames"]} unchanged video packets in {plan["elapsedVideoSeconds"]:.1f}s',flush=True)
        # Fill the source AAC's initial ~77ms edit offset with silence before
        # passing absolute sample-frame coordinates to the audio time stretcher.
        subprocess.run(['ffmpeg','-v','error','-n','-i',str(audio_source),'-vn','-af',
            'aresample=async=1:first_pts=0','-ar',str(sample_rate),'-c:a','pcm_f32le',
            str(work/'audio-original.wav')],check=True)
        plan['audioCommand']=['rubberband','--fast','--centre-focus','--duration',str(duration),
            '--timemap',str(mapping),str(work/'audio-original.wav'),str(work/'audio-retimed.wav')]
        plan_path.write_text(json.dumps(plan,indent=2)+'\n')
        print('Audio ready for pitch-preserved retiming',flush=True)
        if args.prepare_only:return
        subprocess.run(plan['audioCommand'],check=True)
    if args.prepare_only:return
    if out.exists():raise SystemExit(f'Refusing to overwrite {out}')
    wav=work/'audio-retimed.wav'
    audio_duration=float(probe(wav)['format']['duration'])
    if abs(audio_duration-duration)>.05:
        raise SystemExit(f'Audio time map length mismatch: {audio_duration} vs {duration}')
    plan['assemblyCommand']=['ffmpeg','-v','warning','-stats_period','10','-stats','-n',
        '-i',str(work/'video-retimed.mp4'),'-i',str(wav),'-map','0:v:0','-map','1:a:0',
        '-c:v','copy','-c:a','aac','-b:a','192k','-map_metadata','0','-tag:v','hvc1',
        '-movflags','+faststart',str(out)]
    subprocess.run(plan['assemblyCommand'],check=True)
    result=probe(out);v=next(s for s in result['streams'] if s['codec_type']=='video')
    assert (v['profile'],v['pix_fmt'],v['color_transfer'],v['color_primaries'],v['color_space'])==(
        'Main 10','yuv420p10le','smpte2084','bt2020','bt2020nc')
    assert int(v['nb_frames'])==int(source_video['nb_frames'])
    assert abs(float(result['format']['duration'])-duration)<.1
    sides=v.get('side_data_list',[])
    assert any(s['side_data_type']=='Mastering display metadata' for s in sides)
    assert any(s['side_data_type']=='Content light level metadata' and s['max_content']==100 for s in sides)
    plan['outputMetadata']=result
    out.with_suffix('.json').write_text(json.dumps(plan,indent=2)+'\n')
    plan_path.write_text(json.dumps(plan,indent=2)+'\n')
    print('Finished HDR retime:',out,flush=True)


if __name__=='__main__':main()
