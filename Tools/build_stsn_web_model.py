#!/usr/bin/env python3
"""Assemble the telemetry STS-N mesh from its real KSP craft and .mu parts."""
from __future__ import annotations
import argparse, hashlib, json, math, os, re, shutil, struct, subprocess, sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Tools'/'vendor'))
from ksp_mu import Mu  # type: ignore

KSP_ROOT=Path(os.environ.get('KSP_ROOT',str(Path.home()/'Library/Application Support/Steam/steamapps/common/Kerbal Space Program'))).expanduser()
GAME_DATA=KSP_ROOT/'GameData'
DEFAULT_CRAFT=KSP_ROOT/'saves/JUSTLANDTHEFREAKINGSHUTTLE/Ships/SPH/STS-N.craft'
OUT_DIR=ROOT/'Runtime'/'WebTelemetry'
PART_CFG={
'benjee10.shuttle.forwardFuselage':'Benjee10_shuttleOrbiter/Parts/OV_forwardFuselage.cfg',
'benjee10.shuttle.midFuselage':'Benjee10_shuttleOrbiter/Parts/OV_midFuselage.cfg',
'benjee10.shuttle.deltaWing':'Benjee10_shuttleOrbiter/Parts/OV_deltaWing.cfg',
'benjee10.shuttle.aftFuselage':'Benjee10_shuttleOrbiter/Parts/OV_aftFuselage.cfg',
'benjee10.shuttle.bodyFlap':'Benjee10_shuttleOrbiter/Parts/OV_bodyFlap.cfg',
'benjee10.shuttle.elevon2':'Benjee10_shuttleOrbiter/Parts/OV_elevon2.cfg',
'benjee10.shuttle.elevon1':'Benjee10_shuttleOrbiter/Parts/OV_elevon1.cfg',
'benjee10.shuttle.rudder':'Benjee10_shuttleOrbiter/Parts/OV_rudder.cfg',
'benjee10.shuttle.noseGear':'Benjee10_shuttleOrbiter/Parts/OV_noseGear.cfg',
'benjee10.shuttle.mainGear':'Benjee10_shuttleOrbiter/Parts/OV_mainGear.cfg',
'Rockomax8BW':'Squad/Parts/FuelTank/RockomaxTanks/Rockomax8.cfg',
'nfa-atomic-multimode-25-1':'NearFutureAeronautics/Parts/Engine/Atomic/nfa-atomic-multimode-25-1.cfg',
'IntakeRadialLong':'Squad/Parts/Aero/intakeRadialLong/intakeRadialLong.cfg',
}

@dataclass
class Node:
    name:str
    values:dict[str,list[str]]=field(default_factory=dict)
    children:list['Node']=field(default_factory=list)
    def first(self,key,default=None):
        v=self.values.get(key); return v[0] if v else default
    def all(self,key): return self.values.get(key,[])
    def kids(self,name):
        base=name.split(':',1)[0]
        return [c for c in self.children if c.name.split(':',1)[0]==base]

def parse_cfg(path:Path)->Node:
    root=Node('ROOT'); stack=[root]; pending=None
    for raw in path.read_text(encoding='utf-8-sig',errors='replace').splitlines():
        line=raw.split('//',1)[0].strip()
        if not line: continue
        if line=='{':
            if pending is not None:
                child=Node(pending); stack[-1].children.append(child); stack.append(child); pending=None
            continue
        if line=='}':
            if len(stack)>1: stack.pop()
            pending=None; continue
        if '=' in line:
            key,value=(x.strip() for x in line.split('=',1)); stack[-1].values.setdefault(key,[]).append(value)
        else: pending=line
    return root

def numbers(text,count,default):
    if text is None:return default
    try:v=tuple(float(x.strip()) for x in text.split(','))
    except ValueError:return default
    return v if len(v)==count else default

def mmul(a,b):
    return tuple(sum(a[r*4+k]*b[k*4+c] for k in range(4)) for r in range(4) for c in range(4))

def qmul(a,b):
    aw,ax,ay,az=a; bw,bx,by,bz=b
    return (aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw)

def qaxis(axis,a):
    s,c=math.sin(a/2),math.cos(a/2)
    return (c,s,0,0) if axis=='x' else ((c,0,s,0) if axis=='y' else (c,0,0,s))

def qeuler_xyz_deg(v):
    x,y,z=(math.radians(a) for a in v)
    return qmul(qaxis('z',z),qmul(qaxis('y',y),qaxis('x',x)))

def qmatrix(q):
    w,x,y,z=q; length=math.sqrt(w*w+x*x+y*y+z*z) or 1.0
    w,x,y,z=w/length,x/length,y/length,z/length
    return (1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0,
            2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0,
            2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0,
            0,0,0,1)

def trs(pos,q,scale):
    m=list(qmatrix(q))
    for r in range(3):
        m[r*4]*=scale[0]; m[r*4+1]*=scale[1]; m[r*4+2]*=scale[2]
    m[3],m[7],m[11]=pos
    return tuple(m)

def point(m,p):
    x,y,z=p
    return (m[0]*x+m[1]*y+m[2]*z+m[3],m[4]*x+m[5]*y+m[6]*z+m[7],m[8]*x+m[9]*y+m[10]*z+m[11])

def direction(m,p):
    x,y,z=p; v=(m[0]*x+m[1]*y+m[2]*z,m[4]*x+m[5]*y+m[6]*z,m[8]*x+m[9]*y+m[10]*z)
    l=math.sqrt(sum(c*c for c in v)) or 1.0
    return tuple(c/l for c in v)

def unity_vec_to_blender(v): return (v[0],v[2],v[1])
def unity_scale_to_blender(v): return (v[0],v[2],v[1])
def unity_quat_to_blender(q):
    x,y,z,w=q
    return (w,-x,-z,-y)

def unity_euler_to_blender(v):
    # Current STS-N MODEL rotations are zero, but preserve KSP/Unity Euler support.
    x,y,z=(math.radians(a) for a in v)
    raw=qmul(qaxis('y',y),qmul(qaxis('x',x),qaxis('z',z)))
    w,qx,qy,qz=raw
    return unity_quat_to_blender((qx,qy,qz,w))

def scene_point(v): return (v[0],v[1],-v[2])
def part_base_name(saved): return re.sub(r'_\d+$','',saved)

def b9_visibility(cfg,craft_part):
    selected_state={}
    for module in craft_part.kids('MODULE'):
        if module.first('name')=='ModuleB9PartSwitch':
            module_id=module.first('moduleID'); subtype=module.first('currentSubtype')
            if module_id and subtype is not None: selected_state[module_id]=subtype
    controlled=set(); selected=set()
    for module in cfg.kids('MODULE'):
        if module.first('name')!='ModuleB9PartSwitch': continue
        subtypes=module.kids('SUBTYPE')
        for subtype in subtypes: controlled.update(subtype.all('transform'))
        wanted=selected_state.get(module.first('moduleID') or '')
        if wanted is None: continue
        for subtype in subtypes:
            if (subtype.first('name') or '').lower()==wanted.lower():
                selected.update(subtype.all('transform')); break
    return controlled,selected


def sample_curve(curve,time_value):
    keys=getattr(curve,'keys',[]) or []
    if not keys: return None
    if time_value<=keys[0].time: return float(keys[0].value)
    if time_value>=keys[-1].time: return float(keys[-1].value)
    for left,right in zip(keys,keys[1:]):
        if left.time<=time_value<=right.time:
            dt=right.time-left.time
            if dt<=0: return float(right.value)
            t=(time_value-left.time)/dt
            # Unity animation curves use cubic Hermite interpolation between keys.
            t2=t*t; t3=t2*t
            h00=2*t3-3*t2+1; h10=t3-2*t2+t
            h01=-2*t3+3*t2; h11=t3-t2
            return (h00*left.value + h10*left.tangent[1]*dt +
                    h01*right.value + h11*right.tangent[0]*dt)
    return float(keys[-1].value)


def resolve_object_path(owner,path):
    current=owner
    for name in (segment for segment in path.split('/') if segment):
        current=next((child for child in getattr(current,'children',[]) if child.transform.name==name),None)
        if current is None: return None
    return current


def animation_overrides(mu_root,cfg,craft_part):
    cfg_animations=[m for m in cfg.kids('MODULE') if m.first('name')=='ModuleAnimateGeneric']
    saved_animations=[m for m in craft_part.kids('MODULE') if m.first('name')=='ModuleAnimateGeneric']
    times={}
    for index,module in enumerate(cfg_animations):
        name=module.first('animationName')
        if not name or index>=len(saved_animations): continue
        try: normalized=float(saved_animations[index].first('animTime','0') or 0)
        except ValueError: normalized=0.0
        times[name]=max(0.0,min(1.0,normalized))
    if not times: return {}

    overrides={}
    def visit(obj):
        animation=getattr(obj,'animation',None)
        if animation is not None:
            for clip in getattr(animation,'clips',[]) or []:
                if clip.name not in times: continue
                duration=max((key.time for curve in clip.curves for key in curve.keys),default=0.0)
                sample_time=times[clip.name]*duration
                for curve in clip.curves:
                    if getattr(curve,'type',0)!=0: continue
                    target=resolve_object_path(obj,curve.path)
                    if target is None: continue
                    value=sample_curve(curve,sample_time)
                    if value is not None: overrides.setdefault(id(target),{})[curve.property]=value
        for child in getattr(obj,'children',[]): visit(child)
    visit(mu_root)
    return overrides


def animated_transform(transform,values):
    if not values:
        return transform.localPosition,transform.localRotation,transform.localScale
    bp=transform.localPosition; bq=transform.localRotation; bs=transform.localScale
    pos=[bp[0],bp[2],bp[1]]
    rot=[-bq[1],-bq[3],-bq[2],bq[0]]
    scale=[bs[0],bs[2],bs[1]]
    component={'x':0,'y':1,'z':2,'w':3}
    for prop,value in values.items():
        stem,_,axis=prop.rpartition('.')
        index=component.get(axis)
        if index is None: continue
        if stem.endswith('LocalPosition') and index<3: pos[index]=value
        elif stem.endswith('LocalRotation'): rot[index]=value
        elif stem.endswith('LocalScale') and index<3: scale[index]=value
    return unity_vec_to_blender(tuple(pos)),unity_quat_to_blender(tuple(rot)),unity_scale_to_blender(tuple(scale))


def model_specs(cfg_path,cfg):
    rescale=float(cfg.first('rescaleFactor','1') or 1)
    specs=[]
    for model in cfg.kids('MODEL'):
        name=model.first('model')
        if not name: continue
        path=GAME_DATA/(name if name.lower().endswith('.mu') else name+'.mu')
        pos=unity_vec_to_blender(numbers(model.first('position'),3,(0,0,0)))
        scale=unity_scale_to_blender(numbers(model.first('scale'),3,(1,1,1)))
        scale=tuple(v*rescale for v in scale)
        rot=unity_euler_to_blender(numbers(model.first('rotation'),3,(0,0,0)))
        specs.append((path,trs(pos,rot,scale)))
    if not specs:
        mesh=cfg.first('mesh')
        if mesh:
            path=cfg_path.parent/mesh
            if not path.is_file():
                candidates=sorted(cfg_path.parent.glob('*.mu'))
                if len(candidates)==1: path=candidates[0]
            specs.append((path,trs((0,0,0),(1,0,0,0),(rescale,rescale,rescale))))
    return specs


def resolve_texture(model_path,texture_name):
    if not texture_name: return None
    relative=Path(texture_name)
    bases=[model_path.parent]
    if relative.parent!=Path('.'):
        bases.append(GAME_DATA/relative.parent)
    stem=relative.stem.lower()
    for base in bases:
        direct=base/relative.name
        if direct.is_file(): return direct
        for ext in ('.dds','.png','.jpg','.jpeg','.tga','.mbm'):
            candidate=base/(relative.stem+ext)
            if candidate.is_file(): return candidate
        try:
            for candidate in base.iterdir():
                if candidate.is_file() and candidate.stem.lower()==stem and candidate.suffix.lower() in {'.dds','.png','.jpg','.jpeg','.tga','.mbm'}:
                    return candidate
        except OSError:
            pass
    return None


def web_texture(source,out_dir):
    if source is None: return None
    texture_dir=out_dir/'sts-n-textures'; texture_dir.mkdir(parents=True,exist_ok=True)
    # Preserve the installed KSP source resolution.  DDS/TGA/MBM are converted
    # losslessly to browser-readable PNG, but are no longer downscaled.
    digest=hashlib.sha1((str(source)+'|web-v4-full').encode()).hexdigest()[:10]
    safe=re.sub(r'[^A-Za-z0-9_.-]+','-',source.stem)
    dest=texture_dir/f'{digest}-{safe}.png'
    try:
        fresh=dest.is_file() and dest.stat().st_mtime>=source.stat().st_mtime
    except OSError:
        fresh=False
    if not fresh:
        if source.suffix.lower()=='.png':
            shutil.copy2(source,dest)
        else:
            result=subprocess.run(['sips','-s','format','png',str(source),'--out',str(dest)],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            if result.returncode!=0:
                subprocess.run(['ffmpeg','-v','error','-y','-i',str(source),str(dest)],check=True)
    return f'/asset/sts-n-textures/{dest.name}'


def material_spec(mu,material_index,model_path,out_dir,renderer_path):
    if material_index is None or material_index<0 or material_index>=len(mu.materials):
        meta={'name':'untextured','texture':None,'normalTexture':None,'emissiveTexture':None,
              'color':[0.82,0.85,0.88,1.0],'uvScale':[1,1],'uvOffset':[0,0]}
    else:
        material=mu.materials[material_index]
        props=getattr(material,'textureProperties',{}) or {}
        def texture_property(name):
            prop=props.get(name)
            if prop is None: return None,None,(1.0,1.0),(0.0,0.0)
            index=getattr(prop,'index',-1)
            texture_name=mu.textures[index].name if 0<=index<len(mu.textures) else None
            source=resolve_texture(model_path,texture_name)
            scale=tuple(float(v) for v in getattr(prop,'scale',(1.0,1.0)))
            offset=tuple(float(v) for v in getattr(prop,'offset',(0.0,0.0)))
            return web_texture(source,out_dir),str(source) if source else None,scale,offset
        texture,source_texture,uv_scale,uv_offset=texture_property('_MainTex')
        normal_texture,source_normal,_,_=texture_property('_BumpMap')
        emissive_texture,source_emissive,_,_=texture_property('_Emissive')
        color=(getattr(material,'colorProperties',{}) or {}).get('_Color',(1.0,1.0,1.0,1.0))
        shader=str(getattr(material,'shaderName',getattr(material,'shader','')) or '')
        translucent='Alpha/' in shader or 'Transparent' in shader or 'Translucent' in shader
        meta={'name':getattr(material,'name','material'),'texture':texture,
              'normalTexture':normal_texture,'emissiveTexture':emissive_texture,
              'sourceTexture':source_texture,'sourceNormalTexture':source_normal,'sourceEmissiveTexture':source_emissive,
              'sources':[{'model':str(model_path),'renderer':renderer_path,'materialIndex':int(material_index)}],
              'color':[float(v) for v in color],'uvScale':list(uv_scale),'uvOffset':list(uv_offset),
              'shader':shader,'translucent':translucent,'skip':bool(translucent and texture is None)}
    key_meta={key:value for key,value in meta.items() if key!='sources'}
    key=json.dumps(key_meta,sort_keys=True,separators=(',',':'))
    return key,meta


def append_mesh(output_vertices,groups,mesh,matrix,material_indices,mu,model_path,out_dir,renderer_path):
    # KSP .mu files also contain collision meshes as ordinary mesh objects but
    # without renderer materials. They are physics geometry, never visible.
    if mesh is None or not getattr(mesh,'verts',None) or not material_indices: return
    base=len(output_vertices)//8
    normals=getattr(mesh,'normals',[]) or []
    uvs=getattr(mesh,'uvs',[]) or []
    for index,vertex in enumerate(mesh.verts):
        p=scene_point(point(matrix,vertex))
        source_normal=normals[index] if index<len(normals) else (0.0,0.0,1.0)
        n=scene_point(direction(matrix,source_normal))
        uv=uvs[index] if index<len(uvs) else (0.0,0.0)
        # DDS->PNG conversion already preserves the orientation expected by these
        # .mu UVs. Flipping V here moves every island to the wrong atlas region.
        output_vertices.extend((p[0],p[1],p[2],n[0],n[1],n[2],float(uv[0]),float(uv[1])))
    for submesh_index,submesh in enumerate(getattr(mesh,'submeshes',[]) or []):
        material_index=material_indices[submesh_index] if submesh_index<len(material_indices) else (material_indices[0] if material_indices else -1)
        key,meta=material_spec(mu,material_index,model_path,out_dir,renderer_path)
        if meta.get('skip'): continue
        entry=groups.setdefault(key,{'meta':meta,'indices':[]})
        if entry['meta'] is not meta:
            known=entry['meta'].setdefault('sources',[])
            for source in meta.get('sources',[]):
                if source not in known: known.append(source)
        bucket=entry['indices']
        for a,b,c in submesh: bucket.extend((base+a,base+c,base+b))


def walk_mu(obj,parent,visible,controlled,selected,vertices,groups,mu,model_path,out_dir,overrides,apply_local=True,object_path=''):
    transform=obj.transform
    pos,rot,scale=animated_transform(transform,overrides.get(id(obj)))
    local=trs(pos,rot,scale)
    # KSP instantiates the .mu root at the MODEL transform. Some stock/NFA
    # exports retain editor-scene offsets on that root, so do not apply it.
    matrix=mmul(parent,local) if apply_local else parent
    renderer_path=f'{object_path}/{transform.name}' if object_path else transform.name
    node_visible=visible and (transform.name not in controlled or transform.name in selected)
    if node_visible:
        renderer=getattr(obj,'renderer',None)
        append_mesh(vertices,groups,getattr(obj,'shared_mesh',None),matrix,list(getattr(renderer,'materials',[]) or []),mu,model_path,out_dir,renderer_path)
        skinned=getattr(obj,'skinned_mesh_renderer',None)
        if skinned is not None:
            append_mesh(vertices,groups,getattr(skinned,'mesh',None),matrix,list(getattr(skinned,'materials',[]) or []),mu,model_path,out_dir,renderer_path+'#skinned')
    for child in getattr(obj,'children',[]):
        walk_mu(child,matrix,node_visible,controlled,selected,vertices,groups,mu,model_path,out_dir,overrides,True,renderer_path)

def load_part_cfg(name):
    relative=PART_CFG.get(name)
    if relative is None: raise RuntimeError(f'No config mapping for STS-N part {name}')
    path=GAME_DATA/relative
    root=parse_cfg(path)
    cfg=next((child for child in root.children if child.name.split(':',1)[0]=='PART'),None)
    if cfg is None: raise RuntimeError(f'No PART node in {path}')
    return path,cfg

def build(craft_path,out_dir):
    craft=parse_cfg(craft_path)
    parts=craft.kids('PART')
    if not parts: raise RuntimeError(f'No PART records in {craft_path}')
    out_dir.mkdir(parents=True,exist_ok=True)
    vertices=[]; groups={}; provenance=[]
    for ordinal,part in enumerate(parts):
        saved=part.first('part') or ''
        name=part_base_name(saved)
        cfg_path,cfg=load_part_cfg(name)
        controlled,selected=b9_visibility(cfg,part)
        raw_pos=numbers(part.first('pos'),3,(0.0,0.0,0.0))
        raw_rot=numbers(part.first('rot'),4,(0.0,0.0,0.0,1.0))
        raw_scale=numbers(part.first('mir'),3,(1.0,1.0,1.0))
        part_matrix=trs(unity_vec_to_blender(raw_pos),unity_quat_to_blender(raw_rot),unity_scale_to_blender(raw_scale))
        model_paths=[]
        for model_path,model_matrix in model_specs(cfg_path,cfg):
            if not model_path.is_file(): raise RuntimeError(f'Missing model for {name}: {model_path}')
            mu=Mu().read(str(model_path))
            if mu is None: raise RuntimeError(f'Could not parse {model_path}')
            overrides=animation_overrides(mu.obj,cfg,part)
            walk_mu(mu.obj,mmul(part_matrix,model_matrix),True,controlled,selected,vertices,groups,mu,model_path,out_dir,overrides,False)
            model_paths.append(str(model_path))
        provenance.append({'ordinal':ordinal,'part':name,'saved':saved,'models':model_paths})

    if not vertices or not groups: raise RuntimeError('STS-N assembly produced no visible triangles')
    points=[vertices[offset:offset+3] for offset in range(0,len(vertices),8)]
    low=[min(p[axis] for p in points) for axis in range(3)]
    high=[max(p[axis] for p in points) for axis in range(3)]
    center=[(low[axis]+high[axis])*0.5 for axis in range(3)]
    for offset in range(0,len(vertices),8):
        vertices[offset]-=center[0]; vertices[offset+1]-=center[1]; vertices[offset+2]-=center[2]
    bounds={'span':high[0]-low[0],'length':high[1]-low[1],'height':high[2]-low[2]}
    craft_size=numbers(craft.first('size'),3,(0.0,0.0,0.0))
    expected={'span':craft_size[0],'height':craft_size[1],'length':craft_size[2]}

    indices=[]; draw_groups=[]
    for group in groups.values():
        first_index=len(indices)
        group_indices=group['indices']
        # Some installed .mu assets contain coincident same-facing triangles.
        # With different vertex normals they visibly z-fight in WebGPU. Remove
        # only exact same-facing duplicates for opaque materials; preserve
        # reversed winding and translucent decal layers.
        if not group['meta'].get('translucent'):
            deduped=[]; seen=set()
            for offset in range(0,len(group_indices),3):
                tri=group_indices[offset:offset+3]
                pts=[tuple(round(vertices[index*8+axis],5) for axis in range(3)) for index in tri]
                rotations=[tuple(pts[i:]+pts[:i]) for i in range(3)]
                key=min(rotations)
                if key in seen: continue
                seen.add(key); deduped.extend(tri)
            group_indices=deduped
        indices.extend(group_indices)
        draw_groups.append({**group['meta'],'firstIndex':first_index,'indexCount':len(group_indices)})
    vertex_bytes=struct.pack('<'+'f'*len(vertices),*vertices)
    index_bytes=struct.pack('<'+'I'*len(indices),*indices)
    binary=vertex_bytes+index_bytes
    binary_hash=hashlib.sha1(binary).hexdigest()[:12]
    (out_dir/'sts-n-model.bin').write_bytes(binary)
    metadata={
        'format':'stsn-webgpu-5','ship':craft.first('ship','STS-N'),'craft':str(craft_path),
        'partCount':len(parts),'parts':provenance,'vertexCount':len(vertices)//8,'indexCount':len(indices),
        'vertexByteOffset':0,'vertexByteLength':len(vertex_bytes),'indexByteOffset':len(vertex_bytes),
        'indexByteLength':len(index_bytes),'groups':draw_groups,'bounds':bounds,'craftSize':expected,'binary':f'/asset/sts-n-model.bin?v={binary_hash}',
        'generatedFrom':'STS-N.craft part list + saved B9/animation state + installed .mu renderer meshes + exact renderer/material texture provenance + full-resolution KSP textures',
    }
    (out_dir/'sts-n-model.json').write_text(json.dumps(metadata,separators=(',',':')),encoding='utf-8')
    return metadata

def main():
    parser=argparse.ArgumentParser(description='Build telemetry STS-N WebGPU mesh from real KSP parts')
    parser.add_argument('--craft',type=Path,default=DEFAULT_CRAFT)
    parser.add_argument('--out',type=Path,default=OUT_DIR)
    parser.add_argument('--quiet',action='store_true')
    args=parser.parse_args()
    result=build(args.craft.expanduser(),args.out.expanduser())
    if not args.quiet:
        print(f"STS-N {result['partCount']} parts, {result['vertexCount']} vertices, {result['indexCount']//3} triangles")
        print('assembled bounds',json.dumps(result['bounds'],separators=(',',':')))
        print('craft size',json.dumps(result['craftSize'],separators=(',',':')))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
