import bpy, math, os
from mathutils import Vector

OUTDIR = os.path.abspath('Analysis/STS-N-Launcher')
BLEND = os.path.join(OUTDIR, 'STS-N_Launch_Stack.blend')
GLB = os.path.join(OUTDIR, 'STS-N_Launch_Stack.glb')
PREVIEW = os.path.join(OUTDIR, 'STS-N_Launch_Stack_preview.png')
SIDE = os.path.join(OUTDIR, 'STS-N_Launch_Stack_side.png')
FRONT = os.path.join(OUTDIR, 'STS-N_Launch_Stack_front.png')

# ---------------- scene ----------------
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.length_unit = 'METERS'
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = 1100
scene.render.resolution_y = 1100
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.film_transparent = False

if scene.world is None:
    scene.world = bpy.data.worlds.new('World')
scene.world.color = (0.012, 0.016, 0.025)

# ---------------- helpers ----------------
def mat(name, rgba, metallic=0.0, rough=0.45, emission=None):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = rgba
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = rough
    if emission:
        bsdf.inputs['Emission Color'].default_value = emission
        bsdf.inputs['Emission Strength'].default_value = 3.0
    return m

def assign(obj, material):
    if obj.data and hasattr(obj.data, 'materials'):
        obj.data.materials.append(material)

def cyl(name, radius, depth, loc, material, vertices=64):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=loc)
    o=bpy.context.object; o.name=name; assign(o,material); return o

def cone(name, r1, r2, depth, loc, material, vertices=64):
    bpy.ops.mesh.primitive_cone_add(vertices=vertices, radius1=r1, radius2=r2, depth=depth, location=loc)
    o=bpy.context.object; o.name=name; assign(o,material); return o

def uv_sphere(name, loc, scale, material):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=64, ring_count=32, location=loc)
    o=bpy.context.object; o.name=name; o.scale=scale; bpy.ops.object.transform_apply(location=False, rotation=False, scale=True); assign(o,material); return o

def box(name, dims, loc, material, rot=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(location=loc, rotation=rot)
    o=bpy.context.object; o.name=name; o.dimensions=dims; bpy.ops.object.transform_apply(location=False, rotation=False, scale=True); assign(o,material); return o

def prism(name, verts2d, thickness, axis, material):
    # Build a thin prism from a polygon in two axes.
    verts=[]
    faces=[]
    h=thickness/2
    n=len(verts2d)
    if axis=='x':
        for s in (-h,h):
            for y,z in verts2d: verts.append((s,y,z))
    elif axis=='y':
        for s in (-h,h):
            for x,z in verts2d: verts.append((x,s,z))
    else:
        raise ValueError(axis)
    faces.append(tuple(range(n)))
    faces.append(tuple(range(n,2*n)))
    for i in range(n): faces.append((i,(i+1)%n,(i+1)%n+n,i+n))
    mesh=bpy.data.meshes.new(name+'Mesh'); mesh.from_pydata(verts,[],faces); mesh.update()
    o=bpy.data.objects.new(name,mesh); bpy.context.collection.objects.link(o); assign(o,material); return o

def arrow(name, start, direction, length, shaft_r, material, head_scale=4.0):
    d=Vector(direction).normalized(); s=Vector(start); end=s+d*length
    shaft_len=length*0.78; head_len=length-shaft_len
    shaft_mid=s+d*(shaft_len/2)
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=shaft_r, depth=shaft_len, location=shaft_mid)
    sh=bpy.context.object; sh.name=name+'_shaft'; assign(sh,material)
    sh.rotation_mode='QUATERNION'; sh.rotation_quaternion=Vector((0,0,1)).rotation_difference(d)
    head_mid=s+d*(shaft_len+head_len/2)
    bpy.ops.mesh.primitive_cone_add(vertices=32, radius1=shaft_r*head_scale, radius2=0, depth=head_len, location=head_mid)
    hd=bpy.context.object; hd.name=name+'_head'; assign(hd,material)
    hd.rotation_mode='QUATERNION'; hd.rotation_quaternion=Vector((0,0,1)).rotation_difference(d)
    return sh,hd

def text_obj(name, body, loc, size, material, rot=(math.radians(90),0,0), extrude=0.008):
    bpy.ops.object.text_add(location=loc, rotation=rot)
    o=bpy.context.object; o.name=name; o.data.body=body; o.data.align_x='CENTER'; o.data.size=size; o.data.extrude=extrude; assign(o,material); return o

def look_at(obj, target):
    direction=Vector(target)-obj.location
    obj.rotation_euler=direction.to_track_quat('-Z','Y').to_euler()

# ---------------- materials ----------------
M_ET=mat('ET orange',(0.78,0.26,0.055,1),metallic=0.1,rough=0.38)
M_WHITE=mat('Booster white',(0.78,0.80,0.84,1),metallic=0.05,rough=0.3)
M_BLACK=mat('Orbiter black',(0.025,0.03,0.04,1),metallic=0.12,rough=0.3)
M_TILE=mat('Orbiter tile gray',(0.14,0.16,0.18,1),metallic=0.08,rough=0.55)
M_METAL=mat('Engine metal',(0.16,0.19,0.23,1),metallic=0.72,rough=0.25)
M_DARK=mat('Structure',(0.08,0.10,0.13,1),metallic=0.55,rough=0.30)
M_PAD=mat('Pad',(0.12,0.13,0.15,1),metallic=0.25,rough=0.55)
M_VEC=mat('Nominal thrust',(0.18,0.85,1.0,1),emission=(0.18,0.85,1.0,1))
M_COM=mat('CoM',(1.0,0.25,0.08,1),emission=(1.0,0.11,0.02,1))
M_DIM=mat('Dimension',(1.0,0.83,0.10,1),emission=(1.0,0.55,0.02,1))
M_RED=mat('Failure vector',(1.0,0.08,0.08,1),emission=(1.0,0.02,0.02,1))
M_LABEL=mat('Labels',(0.92,0.95,1.0,1),emission=(0.45,0.5,0.7,1))

# ---------------- dimensions / coordinates ----------------
ET_D=5.245; ET_R=ET_D/2; ET_L=27.55
ET_CENTER_Z=0.0
BOOSTER_D=3.220; BOOSTER_R=BOOSTER_D/2; BOOSTER_L=26.704; BOOSTER_Y=4.45
ORBITER_X=4.55
COUGAR_X=-0.141

# ---------------- ET ----------------
# Engineering envelope: cylindrical barrel + rounded ends, overall ~27.55 m.
barrel=22.5
cyl('ET_barrel',ET_R,barrel,(0,0,0),M_ET)
uv_sphere('ET_forward_dome',(0,0,barrel/2),(ET_R,ET_R,2.55),M_ET)
uv_sphere('ET_aft_dome',(0,0,-barrel/2),(ET_R,ET_R,2.50),M_ET)
# subtle axial centerline reference
cyl('ET_reference_axis',0.018,ET_L+7,(0,0,-1.0),M_DIM,vertices=16)

# ---------------- SRBs ----------------
for side, y in [('L',BOOSTER_Y),('R',-BOOSTER_Y)]:
    cyl(f'RSRB_{side}_case',BOOSTER_R,22.2,(0,y,0.0),M_WHITE)
    cone(f'RSRB_{side}_nose',BOOSTER_R,0.15,4.5,(0,y,13.35),M_WHITE)
    cone(f'RSRB_{side}_aft_skirt',1.50,1.08,1.7,(0,y,-11.95),M_WHITE)
    cone(f'RSRB_{side}_nozzle',0.95,0.42,2.0,(0,y,-13.75),M_METAL)
    # radial separator block
    box(f'RSRB_{side}_separator',(0.50,0.30,3.2),(0, y-math.copysign(1.75,y),0),M_DARK)

# ---------------- STS-N proxy orbiter ----------------
# The model intentionally uses an engineering envelope rather than pretending to reproduce .mu surfaces.
# Longitudinal axis is Z; ET lies toward -X from orbiter.
body_len=21.2; body_r_y=1.75; body_r_x=1.50
uv_sphere('STS-N_fuselage',(ORBITER_X,0,2.2),(body_r_x,body_r_y,body_len/2),M_TILE)
# flatten belly slightly with a dark heat-shield slab
box('STS-N_belly_shield',(0.32,3.15,15.0),(ORBITER_X-1.22,0,0.6),M_BLACK)
cone('STS-N_nose',1.25,0.02,3.0,(ORBITER_X,0,14.1),M_TILE)
# delta wings, thin in X; coordinates are y,z, local x shifted afterwards
wing_poly=[(-1.2,-5.6),(-8.2,-5.2),(-6.0,2.8),(-1.15,5.0)]
w1=prism('STS-N_left_wing',wing_poly,0.34,'x',M_TILE); w1.location.x=ORBITER_X
wing_poly2=[(1.2,-5.6),(8.2,-5.2),(6.0,2.8),(1.15,5.0)]
w2=prism('STS-N_right_wing',wing_poly2,0.34,'x',M_TILE); w2.location.x=ORBITER_X
# dorsal vertical tail: polygon in x,z, thin in y
fin_poly=[(ORBITER_X+0.7,-6.0),(ORBITER_X+4.4,-4.8),(ORBITER_X+2.2,2.8),(ORBITER_X+0.8,2.1)]
fin=prism('STS-N_vertical_tail',fin_poly,0.42,'y',M_TILE)
# J-N500 proxy nozzle; fixed thrust axis slightly dorsal (away from ET) relative orbiter line
JN_X=ORBITER_X+0.1116
cone('J-N500_nozzle',1.05,0.48,2.2,(JN_X,0,-10.1),M_METAL)

# ---------------- ET/orbiter attach structure ----------------
for z in (-3.2,4.4):
    box('ET_OV_attach_'+str(z),(1.8,0.72,0.70),(2.90,0,z),M_DARK)
# central decoupler proxy
box('STS_ET_decoupler',(0.55,2.0,2.6),(2.68,0,0.4),M_METAL)

# ---------------- aft adapter + Cougar ----------------
cone('NR-AD-CAP',2.46,1.88,1.55,(0,0,-14.15),M_DARK)
# short mount visibly offset 0.141 m away from orbiter
cyl('Cougar_mount',1.15,0.65,(COUGAR_X,0,-15.20),M_METAL)
cone('Cougar_nozzle',1.12,0.47,2.7,(COUGAR_X,0,-16.75),M_METAL)

# ---------------- launch stand / clamps ----------------
box('MLP_deck',(18.0,16.0,0.85),(0,0,-18.65),M_PAD)
# flame trench opening approximation
box('MLP_trench',(5.6,8.2,1.1),(0,0,-18.55),M_BLACK)
for i,(x,y) in enumerate([(-5.2,-5.2),(-5.2,5.2),(4.0,-6.0),(4.0,6.0)]):
    box(f'TT18_clamp_tower_{i}',(0.55,0.55,4.8),(x,y,-16.0),M_DARK)
    box(f'TT18_clamp_arm_{i}',(2.2,0.28,0.28),(x + (0.9 if x<0 else -0.9),y,-13.8),M_DARK)

# ---------------- thrust vectors ----------------
# Individual engine arrows at representative post-SRB trim.
post_core_deg=-2.72
arrow('J-N500_fixed_vector',(JN_X,0,-11.0),(0,0,1),9.0,0.095,M_VEC)
arrow('Cougar_postSRB_vector',(COUGAR_X,0,-17.8),(math.sin(math.radians(post_core_deg)),0,math.cos(math.radians(post_core_deg))),11.0,0.12,M_VEC)
# RSRB nominal arrows near liftoff pitch trim (~-0.28 deg in the orbiter/ET plane)
for side,y in [('L',BOOSTER_Y),('R',-BOOSTER_Y)]:
    deg=-0.28
    arrow(f'RSRB_{side}_liftoff_vector',(0,y,-14.65),(math.sin(math.radians(deg)),0,math.cos(math.radians(deg))),8.5,0.09,M_VEC)

# ---------------- CoM and resultant vector snapshots ----------------
# Values are from the thrust-vector audit, mapped into the X/Z engineering model.
snapshots=[
    ('Liftoff',0.28,0.4,-0.27),
    ('Post-SRB',0.643,0.424,-1.98),
    ('ET-empty',1.64,-3.2,1.99),
]
for name,x,z,tilt in snapshots:
    uv_sphere('CoM_'+name,(x,0,z),(0.23,0.23,0.23),M_COM)
    arrow('Resultant_'+name,(x,0,z),(math.sin(math.radians(tilt)),0,math.cos(math.radians(tilt))),7.0,0.07,M_COM,head_scale=3.4)

# ---------------- dimension annotation ----------------
# Actual scale dimension from ET axis to Cougar axis; lifted beside engine so it is visible.
zdim=-14.85
cyl('Cougar_offset_dimension',0.025,abs(COUGAR_X),(COUGAR_X/2,-1.65,zdim),M_DIM,vertices=16)
# rotate cylinder to X axis
bpy.context.object.rotation_euler[1]=math.radians(90)
# markers at axes
box('ET_axis_tick',(0.035,0.20,0.60),(0,-1.65,zdim),M_DIM)
box('Cougar_axis_tick',(0.035,0.20,0.60),(COUGAR_X,-1.65,zdim),M_DIM)
text_obj('Offset_label','0.141 m  AWAY FROM STS-N',(-0.65,-1.66,-14.25),0.42,M_DIM,rot=(math.radians(90),0,0),extrude=0.005)
text_obj('Model_title','STS-N POWERED-ET LAUNCH STACK',(0,-10.2,18.0),0.85,M_LABEL,rot=(math.radians(90),0,0),extrude=0.012)
text_obj('ET_label','5 m POWERED ET',(-0.1,-2.9,6.0),0.55,M_LABEL,rot=(math.radians(90),0,0))
text_obj('Orbiter_label','STS-N',(ORBITER_X,-2.8,10.5),0.65,M_LABEL,rot=(math.radians(90),0,0))

# ---------------- organize collections by name prefix ----------------
# Keep object names semantically useful in Blender/GLB outliner.
for o in bpy.context.scene.objects:
    o.select_set(False)

# ---------------- ground ----------------
box('Ground',(80,80,0.3),(0,0,-19.25),mat('GroundMat',(0.025,0.03,0.035,1),metallic=0.0,rough=0.9))

# ---------------- lighting ----------------
def area(name,loc,energy,size,color):
    bpy.ops.object.light_add(type='AREA',location=loc)
    l=bpy.context.object; l.name=name; l.data.energy=energy; l.data.shape='DISK'; l.data.size=size; l.data.color=color; look_at(l,(0,0,0)); return l
area('Key',(22,-26,28),2400,16,(1.0,0.88,0.74))
area('Fill',(-25,-15,15),1500,14,(0.55,0.70,1.0))
area('Rim',(4,20,30),1800,12,(0.65,0.78,1.0))

# ---------------- camera ----------------
bpy.ops.object.camera_add(location=(36,-48,23))
cam=bpy.context.object; cam.name='Engineering_Perspective'; cam.data.lens=52; look_at(cam,(1.2,0,-0.5)); scene.camera=cam
scene.render.filepath=PREVIEW
bpy.ops.render.render(write_still=True)

# Side orthographic: shows ET/orbiter pitch plane.
bpy.ops.object.camera_add(location=(0,-68,1.0))
side=bpy.context.object; side.name='Side_Orthographic'; side.data.type='ORTHO'; side.data.ortho_scale=45; look_at(side,(1.4,0,-0.5)); scene.camera=side; scene.render.filepath=SIDE
bpy.ops.render.render(write_still=True)

# Front orthographic: shows symmetric SRBs.
bpy.ops.object.camera_add(location=(-68,0,1.0))
front=bpy.context.object; front.name='Front_Orthographic'; front.data.type='ORTHO'; front.data.ortho_scale=45; look_at(front,(0,0,-0.5)); scene.camera=front; scene.render.filepath=FRONT
bpy.ops.render.render(write_still=True)

# Reset perspective as default scene camera.
scene.camera=cam

# Save native model and GLB.
bpy.ops.wm.save_as_mainfile(filepath=BLEND)
# Cameras/lights/text are okay in blend; glTF will export supported objects.
bpy.ops.export_scene.gltf(filepath=GLB, export_format='GLB', export_cameras=True, export_lights=True)

print('WROTE',BLEND)
print('WROTE',GLB)
print('WROTE',PREVIEW)
print('WROTE',SIDE)
print('WROTE',FRONT)
