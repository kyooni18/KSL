const KERBIN_RADIUS = 600000;
const DEG = Math.PI / 180;

const vAdd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const vSub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vScale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const vDot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vCross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const vLength = (a) => Math.hypot(a[0], a[1], a[2]);
const vUnit = (a) => { const l = vLength(a) || 1; return vScale(a, 1 / l); };
const finite = (v) => v !== null && v !== undefined && v !== "" && Number.isFinite(Number(v));
const num = (v, fallback = 0) => finite(v) ? Number(v) : fallback;
const isTerminalPhase = (value) => /TAEM|HAC|FINAL|PREFLARE|FLARE|TOUCHDOWN|ROLLOUT/.test(String(value || "").toUpperCase());
const entryPlanDisplayable = (source) => { const g = source?.guidanceState || {}; return g.entryPlanValid === true && g.entryPlanTerminalReady === true && !isTerminalPhase(source?.phase); };
const terminalReferenceDisplayable = (source) => { const g = source?.guidanceState || {}; return g.terminalPathCommitted === true || g.terminalPathCaptured === true || g.terminalPathComplete === true; };
const displayablePlannedTrajectory = (source) => { const allowed = isTerminalPhase(source?.phase) ? terminalReferenceDisplayable(source) : entryPlanDisplayable(source); return allowed && Array.isArray(source?.plannedTrajectory) ? source.plannedTrajectory : []; };
const displayableReferenceTrajectory = (source) => terminalReferenceDisplayable(source) && Array.isArray(source?.referenceTrajectory) ? source.referenceTrajectory : [];
const displayableProjectedTAEMTrajectory = (source) => entryPlanDisplayable(source) && Array.isArray(source?.projectedTAEMTrajectory) ? source.projectedTAEMTrajectory : [];

const futurePredictionTrajectory = (source) => {
  const trajectory=Array.isArray(source?.predictedTrajectory)?source.predictedTrajectory:[];
  const currentUt=source?.telemetry?.ut;
  if(!trajectory.length||!finite(currentUt))return trajectory;
  const now=num(currentUt);
  let firstFuture=-1,timed=0;
  for(let i=0;i<trajectory.length;i++){
    const point=trajectory[i];
    if(!point||!finite(point.ut))continue;
    timed+=1;
    if(firstFuture<0&&num(point.ut)>=now-.25)firstFuture=i;
  }
  if(!timed)return trajectory;
  if(firstFuture<0)return [];
  return trajectory.slice(Math.max(0,firstFuture-1));
};
const trajectoryFingerprint = (points) => {
  if(!Array.isArray(points)||!points.length)return "0";
  const indices=[0,Math.floor((points.length-1)/2),points.length-1];
  const sample=indices.map((index)=>{
    const p=points[index]||{};
    return [
      finite(p.ut)?num(p.ut).toFixed(2):"",
      finite(p.latitude)?num(p.latitude).toFixed(5):"",
      finite(p.longitude)?num(p.longitude).toFixed(5):"",
      finite(p.altitude)?num(p.altitude).toFixed(0):"",
    ].join(",");
  }).join("|");
  return `${points.length}:${sample}`;
};
const isEntryFlightPhase = (value) => /ENTRY|MM304|TAEM|HAC|FINAL|PREFLARE|FLARE|TOUCHDOWN|ROLLOUT|ATTITUDE RECOVERY/.test(String(value || "").toUpperCase());
const isOrbitMode = (source) => Boolean(source) && !isEntryFlightPhase(source?.phase) && (Array.isArray(source?.orbitalTrajectory) && source.orbitalTrajectory.length >= 2 || String(source?.command?.controlProfile || "").toLowerCase() === "orbital");
const durationText = (value) => {
  if(!finite(value))return "—";
  const total=Math.max(0,Math.round(num(value))),h=Math.floor(total/3600),m=Math.floor((total%3600)/60),s=total%60;
  return h?`${h}:${String(m).padStart(2,"0")}:${String(s).padStart(2,"0")}`:`${m}:${String(s).padStart(2,"0")}`;
};
const countdownText = (value) => !finite(value)?"—":num(value)>=0?`T−${durationText(value)}`:`T+${durationText(-num(value))}`;
function interpolateOrbitPoint(points,targetUT){
  if(!Array.isArray(points)||points.length<2||!finite(targetUT))return null;
  const signed=(x)=>{let v=num(x)%360;if(v>180)v-=360;if(v<-180)v+=360;return v;};
  for(let i=1;i<points.length;i++){
    const a=points[i-1],b=points[i]; if(!finite(a?.ut)||!finite(b?.ut))continue;
    const au=num(a.ut),bu=num(b.ut); if(targetUT<Math.min(au,bu)-1e-6||targetUT>Math.max(au,bu)+1e-6)continue;
    const f=Math.abs(bu-au)<1e-9?0:(num(targetUT)-au)/(bu-au),lerp=(x,y)=>finite(x)&&finite(y)?num(x)+(num(y)-num(x))*f:null;
    const lon=(x,y)=>{if(!finite(x)||!finite(y))return null;return signed(num(x)+signed(num(y)-num(x))*f);};
    return {ut:num(targetUT),latitude:lerp(a.latitude,b.latitude),longitude:lon(a.longitude,b.longitude),altitude:lerp(a.altitude,b.altitude),inertialLatitude:lerp(a.inertialLatitude,b.inertialLatitude),inertialLongitude:lon(a.inertialLongitude,b.inertialLongitude)};
  }
  return null;
}
function orbitPOI(source,kind){
  const t=source?.telemetry||{},dt=kind==="ap"?t.timeToApoapsis:t.timeToPeriapsis;
  return finite(t.ut)&&finite(dt)?interpolateOrbitPoint(source?.orbitalTrajectory,num(t.ut)+Math.max(0,num(dt))):null;
}
function burnWindow3D(source){
  const plan=source?.deorbitPlan,points=source?.orbitalTrajectory||[],t=source?.telemetry||{};
  if(!plan||!finite(plan.burnUT)||points.length<2)return {points:[],start:null,mid:null,end:null};
  const half=Math.max(1,finite(plan.estimatedBurnDuration)?Math.max(0,num(plan.estimatedBurnDuration))/2:1),midUT=num(plan.burnUT),startUT=midUT-half,endUT=midUT+half,currentUT=finite(t.ut)?num(t.ut):null;
  let start=interpolateOrbitPoint(points,startUT);
  if(!start&&currentUT!==null&&currentUT>=startUT&&currentUT<=endUT&&finite(t.latitude)&&finite(t.longitude))start={ut:currentUT,latitude:num(t.latitude),longitude:num(t.longitude),altitude:num(t.meanAltitude)};
  const mid=interpolateOrbitPoint(points,midUT),end=interpolateOrbitPoint(points,endUT);
  return {points:[start,mid,end].filter(Boolean),start,mid,end};
}
function poiSpike(point,size){
  if(!point||!finite(point.altitude))return [];
  const a={...point,altitude:num(point.altitude)-size},b={...point,altitude:num(point.altitude)+size};
  return [a,b];
}

function matIdentity() {
  return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
}

function matMul(a, b) {
  const out = new Array(16).fill(0);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
  }
  return out;
}

function matScale(v) {
  return [v[0],0,0,0, 0,v[1],0,0, 0,0,v[2],0, 0,0,0,1];
}

function matPerspective(fov, aspect, near, far) {
  const f = 1 / Math.tan(fov / 2);
  const nf = 1 / (near - far);
  // WebGPU/D3D clip depth is 0..1, not OpenGL's -1..1.
  return [f/aspect,0,0,0, 0,f,0,0, 0,0,far*nf,-1, 0,0,near*far*nf,0];
}

function matLookAt(eye, center, up) {
  const z = vUnit(vSub(eye, center));
  const x = vUnit(vCross(up, z));
  const y = vCross(z, x);
  return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0, -vDot(x,eye),-vDot(y,eye),-vDot(z,eye),1];
}

function planetBasis(latitude, longitude) {
  // The stock scaled-space bitmap is vertically inverted relative to the scene
  // sphere. Geographic north therefore maps toward scene -Y.
  const lat = -latitude * DEG;
  const lon = longitude * DEG;
  const up = [Math.cos(lat)*Math.cos(lon), Math.sin(lat), Math.cos(lat)*Math.sin(lon)];
  // This is increasing *geographic* latitude (north), not increasing scene latitude.
  const north = [Math.sin(lat)*Math.cos(lon), -Math.cos(lat), Math.sin(lat)*Math.sin(lon)];
  const east = [-Math.sin(lon), 0, Math.cos(lon)];
  return { up, north, east };
}

function planetPoint(point, extraAltitude = 0) {
  const lat = -num(point?.latitude) * DEG;
  const lon = num(point?.longitude) * DEG;
  const altitude = num(point?.altitude, num(point?.meanAltitude));
  const radius = KERBIN_RADIUS + Math.max(-10000, altitude) + extraAltitude;
  const c = Math.cos(lat);
  return [radius*c*Math.cos(lon), radius*Math.sin(lat), radius*c*Math.sin(lon)];
}


function greatCircleMeters(a, b) {
  if (![a?.latitude, a?.longitude, b?.latitude, b?.longitude].every(finite)) return null;
  const lat1 = num(a.latitude) * DEG, lat2 = num(b.latitude) * DEG;
  const dLat = lat2 - lat1, dLon = (num(b.longitude) - num(a.longitude)) * DEG;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
  return KERBIN_RADIUS * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(Math.max(0, 1 - h)));
}

function destinationPoint(origin, bearingDegrees, meters, altitude = null) {
  if (!origin || !finite(origin.latitude) || !finite(origin.longitude)) return null;
  const angular = num(meters) / KERBIN_RADIUS;
  const bearing = num(bearingDegrees) * DEG;
  const lat1 = num(origin.latitude) * DEG, lon1 = num(origin.longitude) * DEG;
  const sinLat1 = Math.sin(lat1), cosLat1 = Math.cos(lat1);
  const sinAngular = Math.sin(angular), cosAngular = Math.cos(angular);
  const lat2 = Math.asin(sinLat1 * cosAngular + cosLat1 * sinAngular * Math.cos(bearing));
  const lon2 = lon1 + Math.atan2(Math.sin(bearing) * sinAngular * cosLat1, cosAngular - sinLat1 * Math.sin(lat2));
  return {
    latitude: lat2 / DEG,
    longitude: ((lon2 / DEG + 540) % 360) - 180,
    altitude: finite(altitude) ? num(altitude) : num(origin.altitude, num(origin.meanAltitude)),
  };
}

function formatDistance(value) {
  if (!finite(value)) return "—";
  const meters = Math.abs(num(value));
  if (meters >= 100000) return `${(meters / 1000).toFixed(0)} km`;
  if (meters >= 1000) return `${(meters / 1000).toFixed(1)} km`;
  return `${meters.toFixed(0)} m`;
}

function bodyMatrix(telemetry, visualLift = 0) {
  const lat = num(telemetry.latitude);
  const lon = num(telemetry.longitude);
  const heading = num(telemetry.heading) * DEG;
  const pitch = num(telemetry.pitch) * DEG;
  const roll = num(telemetry.roll) * DEG;
  const basis = planetBasis(lat, lon);
  const forward = vUnit(vAdd(vScale(basis.up, Math.sin(pitch)), vAdd(vScale(basis.north, Math.cos(pitch)*Math.cos(heading)), vScale(basis.east, Math.cos(pitch)*Math.sin(heading)))));
  const unrolledRight = vUnit(vAdd(vScale(basis.north, -Math.sin(heading)), vScale(basis.east, Math.cos(heading))));
  // Latitude reflection makes {up,north,east} left-handed, so reconstruct local
  // up with right x forward (not forward x right) to keep the vessel upright.
  const unrolledUp = vUnit(vCross(unrolledRight, forward));
  const right = vUnit(vAdd(vScale(unrolledRight, Math.cos(roll)), vScale(unrolledUp, -Math.sin(roll))));
  const craftUp = vUnit(vAdd(vScale(unrolledUp, Math.cos(roll)), vScale(unrolledRight, Math.sin(roll))));
  const down = vScale(craftUp, -1);
  const p = planetPoint({ latitude: lat, longitude: lon, altitude: num(telemetry.meanAltitude) }, visualLift);
  // Visual STS-N handedness needs one local-X reflection: flip left/right only.
  const modelLeft = vScale(right, -1);
  return [modelLeft[0],modelLeft[1],modelLeft[2],0, forward[0],forward[1],forward[2],0, down[0],down[1],down[2],0, p[0],p[1],p[2],1];
}

function builder() { return { vertices: [], indices: [] }; }

function sphereMesh(segments=96,rings=48,radius=KERBIN_RADIUS) {
  const mesh=builder();
  for(let y=0;y<=rings;y++){
    const v=y/rings, lat=(.5-v)*Math.PI, cl=Math.cos(lat), sl=Math.sin(lat);
    for(let x=0;x<=segments;x++){
      const u=x/segments, lon=(u*2-1)*Math.PI, nx=cl*Math.cos(lon), ny=sl, nz=cl*Math.sin(lon);
      // Geometry runs north-to-south as v goes 0 -> 1. The stock scaled-space
      // bitmap needs its V axis inverted at material sampling time.
      mesh.vertices.push(nx*radius,ny*radius,nz*radius,nx,ny,nz,u,v);
    }
  }
  for(let y=0;y<rings;y++)for(let x=0;x<segments;x++){
    const a=y*(segments+1)+x,b=a+segments+1;
    mesh.indices.push(a,a+1,b,a+1,b+1,b);
  }
  return mesh;
}

function createGpuMesh(device, mesh, label) {
  const vertices=new Float32Array(mesh.vertices), indices=new Uint32Array(mesh.indices);
  const vertex=device.createBuffer({label:`${label}-vertices`,size:Math.max(4,vertices.byteLength),usage:GPUBufferUsage.VERTEX|GPUBufferUsage.COPY_DST});
  const index=device.createBuffer({label:`${label}-indices`,size:Math.max(4,indices.byteLength),usage:GPUBufferUsage.INDEX|GPUBufferUsage.COPY_DST});
  if(vertices.byteLength)device.queue.writeBuffer(vertex,0,vertices);
  if(indices.byteLength)device.queue.writeBuffer(index,0,indices);
  return {vertex,index,indexCount:indices.length};
}

async function loadSTSNModel(device) {
  const metadataResponse=await fetch("/asset/sts-n-model.json",{cache:"no-store"});
  if(!metadataResponse.ok)throw new Error(`STS-N model metadata ${metadataResponse.status}`);
  const metadata=await metadataResponse.json();
  if(metadata?.format!=="stsn-webgpu-5")throw new Error("Unsupported STS-N model format");
  const binaryResponse=await fetch(metadata.binary||"/asset/sts-n-model.bin",{cache:"no-store"});
  if(!binaryResponse.ok)throw new Error(`STS-N model data ${binaryResponse.status}`);
  const binary=await binaryResponse.arrayBuffer();
  const vertexOffset=num(metadata.vertexByteOffset),vertexLength=num(metadata.vertexByteLength);
  const indexOffset=num(metadata.indexByteOffset),indexLength=num(metadata.indexByteLength);
  if(vertexLength<=0||indexLength<=0||indexOffset+indexLength>binary.byteLength)throw new Error("Invalid STS-N model buffer");
  const vertices=new Float32Array(binary.slice(vertexOffset,vertexOffset+vertexLength));
  const indices=new Uint32Array(binary.slice(indexOffset,indexOffset+indexLength));
  return {metadata,gpu:createGpuMesh(device,{vertices,indices},"sts-n-craft")};
}

function createTextureFromRGBA(device, rgba) {
  const texture=device.createTexture({size:[1,1,1],format:"rgba8unorm-srgb",usage:GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST});
  const row=new Uint8Array(256); row.set(rgba);
  device.queue.writeTexture({texture},row,{bytesPerRow:256,rowsPerImage:1},{width:1,height:1,depthOrArrayLayers:1});
  return texture;
}

async function loadImageTexture(device,url,fallbackRGBA=[255,255,255,255],maxDimension=0) {
  try {
    const response=await fetch(url,{cache:"force-cache"});
    if(!response.ok)throw new Error(`${url} ${response.status}`);
    const source=await createImageBitmap(await response.blob());
    let bitmap=source;
    if(maxDimension>0&&Math.max(source.width,source.height)>maxDimension){
      const scale=maxDimension/Math.max(source.width,source.height);
      const width=Math.max(1,Math.round(source.width*scale)),height=Math.max(1,Math.round(source.height*scale));
      const offscreen=typeof OffscreenCanvas!=="undefined";
      const canvas=offscreen?new OffscreenCanvas(width,height):document.createElement("canvas");
      if(!offscreen){canvas.width=width;canvas.height=height;}
      const ctx=canvas.getContext("2d",{alpha:true});
      if(!ctx)throw new Error(`2D canvas unavailable for ${url}`);
      ctx.imageSmoothingEnabled=true;
      if("imageSmoothingQuality" in ctx)ctx.imageSmoothingQuality="high";
      ctx.drawImage(source,0,0,width,height);
      bitmap=canvas.transferToImageBitmap?canvas.transferToImageBitmap():await createImageBitmap(canvas);
      source.close?.();
    }
    const mipLevelCount=1+Math.floor(Math.log2(Math.max(bitmap.width,bitmap.height)));
    const texture=device.createTexture({
      size:[bitmap.width,bitmap.height,1],mipLevelCount,format:"rgba8unorm-srgb",
      usage:GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST|GPUTextureUsage.RENDER_ATTACHMENT
    });
    let width=bitmap.width,height=bitmap.height;
    for(let level=0;level<mipLevelCount;level++){
      const offscreen=typeof OffscreenCanvas!=="undefined";
      const canvas=offscreen?new OffscreenCanvas(width,height):document.createElement("canvas");
      if(!offscreen){canvas.width=width;canvas.height=height;}
      const ctx=canvas.getContext("2d",{alpha:true});
      if(!ctx)throw new Error(`2D canvas unavailable for ${url}`);
      ctx.imageSmoothingEnabled=true;
      if("imageSmoothingQuality" in ctx)ctx.imageSmoothingQuality="high";
      ctx.clearRect(0,0,width,height);
      ctx.drawImage(bitmap,0,0,width,height);
      const mipBitmap=canvas.transferToImageBitmap?canvas.transferToImageBitmap():await createImageBitmap(canvas);
      device.queue.copyExternalImageToTexture({source:mipBitmap},{texture,mipLevel:level},{width,height,depthOrArrayLayers:1});
      mipBitmap.close?.();
      width=Math.max(1,width>>1);height=Math.max(1,height>>1);
    }
    bitmap.close?.();
    return {texture,real:true};
  } catch (_) {
    return {texture:createTextureFromRGBA(device,fallbackRGBA),real:false};
  }
}

async function loadKerbinTexture(device) {
  // Stock KSP NewKerbinScaledSpace._MainTex (KerbinScaledSpace300), compressed from 8192x4096.
  return loadImageTexture(device,"/asset/kerbin-map?v=stock-kerbin-scaledspace300-1",[50,67,49,255],2048);
}

const SOLID_WGSL=`
struct Uniforms { mvp:mat4x4<f32>, model:mat4x4<f32>, color:vec4<f32>, light:vec4<f32>, uvTransform:vec4<f32> };
@group(0) @binding(0) var<uniform> u:Uniforms;
@group(0) @binding(1) var smp:sampler;
@group(0) @binding(2) var tex:texture_2d<f32>;
struct In { @location(0) p:vec3<f32>, @location(1) n:vec3<f32>, @location(2) uv:vec2<f32> };
struct Out { @builtin(position) position:vec4<f32>, @location(0) n:vec3<f32>, @location(1) uv:vec2<f32> };
@vertex fn vs(i:In)->Out { var o:Out; o.position=u.mvp*vec4<f32>(i.p,1.0); o.n=normalize((u.model*vec4<f32>(i.n,0.0)).xyz); o.uv=i.uv*u.uvTransform.xy+u.uvTransform.zw; return o; }
@fragment fn fs(i:Out)->@location(0) vec4<f32> {
  let mode=u.light.w;
  let lambert=max(dot(normalize(i.n),normalize(u.light.xyz)),0.0);
  let isPlanet=mode>2.5;
  let baseLight=0.055+0.945*lambert;
  let planetLight=0.24+0.86*lambert;
  let illumination=select(select(baseLight,planetLight,isPlanet),1.0,mode>1.5&&mode<2.5);
  let sampled=textureSample(tex,smp,i.uv);
  let liftedRgb=pow(max(sampled.rgb,vec3<f32>(0.0)),vec3<f32>(0.72))*1.15;
  let rgb=select(sampled.rgb,liftedRgb,isPlanet);
  let alpha=select(1.0,sampled.a*u.color.a,mode>0.5&&mode<1.5);
  return vec4<f32>(rgb*u.color.rgb*illumination,alpha);
}
`;

const LINE_WGSL=`
struct Uniforms { vp:mat4x4<f32>, color:vec4<f32> };
@group(0) @binding(0) var<uniform> u:Uniforms;
@vertex fn vs(@location(0) p:vec3<f32>)->@builtin(position) vec4<f32> { return u.vp*vec4<f32>(p,1.0); }
@fragment fn fs()->@location(0) vec4<f32> { return u.color; }
`;

export async function createTelemetry3D({canvas,statusEl,metaEl}) {
  if(!navigator.gpu)throw new Error("WebGPU is unavailable in this browser");
  statusEl.textContent="WEBGPU · ADAPTER";
  const adapter=await navigator.gpu.requestAdapter({powerPreference:"high-performance"});
  if(!adapter)throw new Error("No WebGPU adapter available");
  const device=await adapter.requestDevice();
  const context=canvas.getContext("webgpu");
  if(!context)throw new Error("WebGPU canvas context unavailable");
  const format=navigator.gpu.getPreferredCanvasFormat();
  context.configure({device,format,alphaMode:"opaque"});

  const MSAA_SAMPLES=4;
  let visible=false,snapshot=null,frame=0,yaw=.78,pitch=.36,distance=110,autoCamera=true,lastCameraTarget=null,depthTexture=null,msaaTexture=null,depthWidth=0,depthHeight=0;
  const pointers=new Map(); let lastPointer=null,pinchDistance=0;
  const wrapAngle=(value)=>{let angle=num(value)%360;if(angle>180)angle-=360;if(angle<-180)angle+=360;return angle;};
  const clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
  device.lost.then(info=>{visible=false;statusEl.textContent="WEBGPU · LOST";metaEl.textContent=info.message||info.reason||"device lost";statusEl.parentElement?.classList.add("alert");});

  const solidModule=device.createShaderModule({label:"sts-n-solid",code:SOLID_WGSL});
  const lineModule=device.createShaderModule({label:"sts-n-lines",code:LINE_WGSL});
  const solidBindGroupLayout=device.createBindGroupLayout({
    label:"sts-n-material-layout",
    entries:[
      {binding:0,visibility:GPUShaderStage.VERTEX|GPUShaderStage.FRAGMENT,buffer:{type:"uniform"}},
      {binding:1,visibility:GPUShaderStage.FRAGMENT,sampler:{type:"filtering"}},
      {binding:2,visibility:GPUShaderStage.FRAGMENT,texture:{sampleType:"float",viewDimension:"2d"}},
    ],
  });
  const solidPipelineLayout=device.createPipelineLayout({label:"sts-n-solid-layout",bindGroupLayouts:[solidBindGroupLayout]});
  const solidPipeline=device.createRenderPipeline({
    label:"sts-n-solid-pipeline",layout:solidPipelineLayout,
    vertex:{module:solidModule,entryPoint:"vs",buffers:[{arrayStride:32,attributes:[{shaderLocation:0,offset:0,format:"float32x3"},{shaderLocation:1,offset:12,format:"float32x3"},{shaderLocation:2,offset:24,format:"float32x2"}]}]},
    fragment:{module:solidModule,entryPoint:"fs",targets:[{format}]},
    primitive:{topology:"triangle-list",frontFace:"ccw",cullMode:"none"},depthStencil:{format:"depth32float",depthWriteEnabled:true,depthCompare:"less"},multisample:{count:MSAA_SAMPLES}
  });
  const alphaPipeline=device.createRenderPipeline({
    label:"sts-n-alpha-pipeline",layout:solidPipelineLayout,
    vertex:{module:solidModule,entryPoint:"vs",buffers:[{arrayStride:32,attributes:[{shaderLocation:0,offset:0,format:"float32x3"},{shaderLocation:1,offset:12,format:"float32x3"},{shaderLocation:2,offset:24,format:"float32x2"}]}]},
    fragment:{module:solidModule,entryPoint:"fs",targets:[{format,blend:{color:{srcFactor:"src-alpha",dstFactor:"one-minus-src-alpha",operation:"add"},alpha:{srcFactor:"one",dstFactor:"one-minus-src-alpha",operation:"add"}}}]},
    primitive:{topology:"triangle-list",frontFace:"ccw",cullMode:"none"},
    depthStencil:{format:"depth32float",depthWriteEnabled:false,depthCompare:"less-equal",depthBias:-2,depthBiasSlopeScale:-1},multisample:{count:MSAA_SAMPLES}
  });
  const lineBindGroupLayout=device.createBindGroupLayout({
    label:"telemetry-line-bind-layout",
    entries:[{binding:0,visibility:GPUShaderStage.VERTEX|GPUShaderStage.FRAGMENT,buffer:{type:"uniform"}}],
  });
  const linePipelineLayout=device.createPipelineLayout({label:"telemetry-line-layout",bindGroupLayouts:[lineBindGroupLayout]});
  const linePipeline=device.createRenderPipeline({
    label:"trajectory-line-pipeline",layout:linePipelineLayout,
    vertex:{module:lineModule,entryPoint:"vs",buffers:[{arrayStride:12,attributes:[{shaderLocation:0,offset:0,format:"float32x3"}]}]},
    fragment:{module:lineModule,entryPoint:"fs",targets:[{format}]},
    primitive:{topology:"line-strip"},depthStencil:{format:"depth32float",depthWriteEnabled:false,depthCompare:"less-equal"},multisample:{count:MSAA_SAMPLES}
  });
  const plumePipeline=device.createRenderPipeline({
    label:"orbital-engine-plume-pipeline",layout:linePipelineLayout,
    vertex:{module:lineModule,entryPoint:"vs",buffers:[{arrayStride:12,attributes:[{shaderLocation:0,offset:0,format:"float32x3"}]}]},
    fragment:{module:lineModule,entryPoint:"fs",targets:[{format,blend:{color:{srcFactor:"src-alpha",dstFactor:"one",operation:"add"},alpha:{srcFactor:"one",dstFactor:"one",operation:"add"}}}]},
    primitive:{topology:"line-list"},depthStencil:{format:"depth32float",depthWriteEnabled:false,depthCompare:"less-equal"},multisample:{count:MSAA_SAMPLES}
  });

  const sampler=device.createSampler({magFilter:"linear",minFilter:"linear",mipmapFilter:"linear",addressModeU:"repeat",addressModeV:"clamp-to-edge",maxAnisotropy:16});
  const whiteTexture=createTextureFromRGBA(device,[255,255,255,255]);
  const kerbin=await loadKerbinTexture(device);
  const planetMesh=createGpuMesh(device,sphereMesh(),"kerbin");
  const sunMesh=createGpuMesh(device,sphereMesh(32,16,1),"kerbol");
  const craft=await loadSTSNModel(device);
  const craftLength=num(craft.metadata?.craftSize?.length,num(craft.metadata?.bounds?.length,1)),craftSpan=num(craft.metadata?.craftSize?.span,num(craft.metadata?.bounds?.span,1));

  function solidState(texture) {
    const uniform=device.createBuffer({size:176,usage:GPUBufferUsage.UNIFORM|GPUBufferUsage.COPY_DST});
    const bind=device.createBindGroup({layout:solidBindGroupLayout,entries:[{binding:0,resource:{buffer:uniform}},{binding:1,resource:sampler},{binding:2,resource:texture.createView()}]});
    return {uniform,bind};
  }
  const planetState=solidState(kerbin.texture);
  const sunState=solidState(whiteTexture);
  const textureCache=new Map();
  async function materialTexture(url){
    if(!url)return whiteTexture;
    if(!textureCache.has(url))textureCache.set(url,loadImageTexture(device,url,[255,255,255,255]));
    return (await textureCache.get(url)).texture;
  }
  const craftGroups=[];
  for(const group of craft.metadata?.groups||[]){
    const texture=await materialTexture(group.texture);
    craftGroups.push({...group,state:solidState(texture)});
  }

  function lineState(color) {
    const uniform=device.createBuffer({size:80,usage:GPUBufferUsage.UNIFORM|GPUBufferUsage.COPY_DST});
    const bind=device.createBindGroup({layout:lineBindGroupLayout,entries:[{binding:0,resource:{buffer:uniform}}]});
    return {color,uniform,bind,buffer:null,capacity:0,count:0,geometryKey:null};
  }
  const lines={
    actual:lineState([.40,.84,.94,1]),
    orbit:lineState([.40,.84,.94,.72]),
    predicted:lineState([.45,.95,.60,1]),
    planned:lineState([1,.61,.32,1]),
    reference:lineState([.92,.95,1,1]),
    projected:lineState([.46,.78,1,1]),
    burn:lineState([1,.42,.12,1]),
    apoapsis:lineState([.40,.84,.94,1]),
    periapsis:lineState([1,.61,.32,1]),
    plume:lineState([1,.42,.08,.72]),
    runway:lineState([1,.72,.30,1]),
    corridor:lineState([1,.52,.22,1]),
  };

  function resize() {
    const rect=canvas.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2);
    const width=Math.max(1,Math.round(rect.width*dpr)),height=Math.max(1,Math.round(rect.height*dpr));
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}
    if(width!==depthWidth||height!==depthHeight){
      depthTexture?.destroy();msaaTexture?.destroy();
      depthTexture=device.createTexture({size:[width,height],sampleCount:MSAA_SAMPLES,format:"depth32float",usage:GPUTextureUsage.RENDER_ATTACHMENT});
      msaaTexture=device.createTexture({size:[width,height],sampleCount:MSAA_SAMPLES,format,usage:GPUTextureUsage.RENDER_ATTACHMENT});
      depthWidth=width;depthHeight=height;
    }
    return {width,height};
  }


  function cameraFrame(t, width, height, visualLift=0) {
    if (!finite(t?.latitude) || !finite(t?.longitude)) return null;
    const orbital=snapshot?.orbitalTrajectory||[];
    if(Array.isArray(orbital)&&orbital.length>=8){const ow=orbital.filter((q)=>finite(q?.inertialLatitude)&&finite(q?.inertialLongitude)).map((q)=>planetPoint({latitude:num(q.inertialLatitude),longitude:num(q.inertialLongitude),altitude:q.altitude},0));if(ow.length>=8){const vf=47*DEG,hf=2*Math.atan(Math.tan(vf/2)*(width/Math.max(1,height))),lf=Math.max(8*DEG,Math.min(vf,hf)/2),extent=ow.reduce((m,q)=>Math.max(m,vLength(q)),KERBIN_RADIUS*1.03),fit=extent/Math.max(.1,Math.sin(lf))*1.08;return{target:[0,0,0],distance:Math.max(900000,Math.min(2400000,fit)),horizon:null,focusCount:ow.length,mode:"ORBIT"};}}
    const phase=String(snapshot?.phase||"").toUpperCase();
    const finalApproach=/PREFLARE|FINAL|FLARE|TOUCHDOWN|ROLLOUT/.test(phase);
    const terminalApproach=finalApproach||/TAEM|HAC/.test(phase);
    const altitude=Math.max(0,num(t?.meanAltitude,num(t?.altitude)));
    const horizon=finalApproach
      ? Math.max(10000,Math.min(28000,altitude*3+7000))
      : terminalApproach
        ? Math.max(22000,Math.min(65000,altitude*3.5+12000))
        : Math.max(35000,Math.min(90000,altitude*2+30000));
    const focus=[];
    const append=(point)=>{if(point&&finite(point.latitude)&&finite(point.longitude))focus.push(point);};
    const appendTrajectory=(trajectory)=>{
      if(!Array.isArray(trajectory))return;
      let kept=0;
      for(const point of trajectory){
        if(!point||!finite(point.latitude)||!finite(point.longitude))continue;
        const range=greatCircleMeters(t,point);
        if(range===null||range<=horizon||kept<2){append(point);kept+=1;}
      }
    };

    append(t);
    if(terminalApproach){
      appendTrajectory(displayableReferenceTrajectory(snapshot));
      appendTrajectory(futurePredictionTrajectory(snapshot));
      appendTrajectory(displayablePlannedTrajectory(snapshot));
    }else{
      appendTrajectory(futurePredictionTrajectory(snapshot));
      appendTrajectory(displayablePlannedTrajectory(snapshot));
      appendTrajectory(displayableProjectedTAEMTrajectory(snapshot));
      appendTrajectory(displayableReferenceTrajectory(snapshot));
    }
    const site=snapshot?.site;
    const siteRange=greatCircleMeters(t,site);
    if(terminalApproach||(finite(siteRange)&&siteRange<=horizon*1.08))append(site);

    const world=focus.map((point)=>planetPoint(point,point===t?visualLift:0));
    const craftPoint=planetPoint({latitude:num(t.latitude),longitude:num(t.longitude),altitude:num(t.meanAltitude)},visualLift);
    const minimum=finalApproach?Math.max(65,craftLength*2.6):terminalApproach?Math.max(150,craftLength*3.5):Math.max(95,craftLength*3.0);
    if(world.length<2)return{target:craftPoint,distance:minimum,horizon,focusCount:world.length};

    const direction=vUnit(world.reduce((sum,point)=>vAdd(sum,vUnit(point)),[0,0,0]));
    const averageRadius=world.reduce((sum,point)=>sum+vLength(point),0)/world.length;
    const target=vScale(direction,averageRadius);
    const extent=Math.max(...world.map((point)=>vLength(vSub(point,target))),craftLength*1.5);
    const verticalFov=47*DEG;
    const horizontalFov=2*Math.atan(Math.tan(verticalFov/2)*(width/Math.max(1,height)));
    const limitingHalfFov=Math.max(8*DEG,Math.min(verticalFov,horizontalFov)/2);
    const fitted=extent/Math.max(.1,Math.sin(limitingHalfFov))*1.06;
    return{target,distance:Math.max(minimum,Math.min(260000,fitted)),horizon,focusCount:world.length};
  }

  function runwayGeometry(site){
    if(!site||!finite(site.latitude)||!finite(site.longitude)||!finite(site.runwayHeading))return{axis:[],corridor:[]};
    const heading=num(site.runwayHeading),altitude=num(site.altitude,70)+8;
    const center={latitude:num(site.latitude),longitude:num(site.longitude),altitude};
    const back=destinationPoint(center,heading+180,32000,altitude);
    const forward=destinationPoint(center,heading,8000,altitude);
    const farCenter=destinationPoint(center,heading+180,30000,altitude);
    const nearCenter=destinationPoint(center,heading+180,700,altitude);
    const offset=(base,lateral)=>destinationPoint(base,heading+(lateral>=0?90:-90),Math.abs(lateral),altitude);
    return{
      axis:[back,center,forward],
      corridor:[offset(farCenter,-4500),offset(nearCenter,-350),offset(nearCenter,350),offset(farCenter,4500),offset(farCenter,-4500)],
    };
  }


  function camera(t, targetOverride=null, cameraDistance=distance, visualLift=0) {
    const lat=num(t?.latitude),lon=num(t?.longitude),alt=num(t?.meanAltitude,80000);
    const target=targetOverride||planetPoint({latitude:lat,longitude:lon,altitude:alt},visualLift);
    const {up,north,east}=planetBasis(lat,lon),cp=Math.cos(pitch);
    // Mirror only the camera's east/west orbit position. Do not mirror the view
    // matrix, because that would reflect Kerbin and every other world-space object.
    const offset=vAdd(vScale(east,-Math.sin(yaw)*cp*cameraDistance),vAdd(vScale(north,Math.cos(yaw)*cp*cameraDistance),vScale(up,Math.sin(pitch)*cameraDistance)));
    return {target,eye:vAdd(target,offset),up};
  }

  function sunDirection(t){
    if(finite(t?.sunLatitude)&&finite(t?.sunLongitude))return planetBasis(num(t.sunLatitude),num(t.sunLongitude)).up;
    // Fallback while an older telemetry observer is still running. Kerbin's
    // solar day is six hours; exact kRPC subsolar coordinates replace this.
    const fallbackLongitude=15.4423-(num(t?.ut)%21600)*360/21600;
    return planetBasis(0,fallbackLongitude).up;
  }

  function sunModel(lightDir){
    const radius=12000,distanceFromKerbin=2600000,p=vScale(lightDir,distanceFromKerbin);
    return [radius,0,0,0, 0,radius,0,0, 0,0,radius,0, p[0],p[1],p[2],1];
  }

  function writeSolid(state,mvp,model,color,uvScale=[1,1],uvOffset=[0,0],translucent=false,lightDir=[.30,.72,-.62],emissive=false,planetLit=false){
    const data=new Float32Array(44);data.set(mvp,0);data.set(model,16);data.set(color,32);data.set([lightDir[0],lightDir[1],lightDir[2],planetLit?3:(emissive?2:(translucent?1:0))],36);data.set([uvScale[0],uvScale[1],uvOffset[0],uvOffset[1]],40);device.queue.writeBuffer(state.uniform,0,data);
  }

  function slerpDirection(a,b,t,arc){
    if(arc<1e-8)return vUnit(vAdd(vScale(a,1-t),vScale(b,t)));
    const s=Math.sin(arc);
    if(Math.abs(s)<1e-8)return vUnit(vAdd(vScale(a,1-t),vScale(b,t)));
    return vUnit(vAdd(vScale(a,Math.sin((1-t)*arc)/s),vScale(b,Math.sin(t*arc)/s)));
  }

  function trajectoryPositions(points,smooth=false,useInertial=false){
    const raw=[];
    if(Array.isArray(points))for(const p of points)if(finite(p?.latitude)&&finite(p?.longitude))raw.push(planetPoint(useInertial&&finite(p?.inertialLatitude)&&finite(p?.inertialLongitude)?{latitude:num(p.inertialLatitude),longitude:num(p.inertialLongitude),altitude:p.altitude}:p,0));
    if(!smooth||raw.length<2)return raw.flat();
    const dirs=raw.map(vUnit),radii=raw.map(vLength),positions=[];
    for(let i=0;i<raw.length-1;i++){
      const a=dirs[i],b=dirs[i+1];
      const arc=Math.acos(Math.max(-1,Math.min(1,vDot(a,b))));
      // Densify the authoritative sample-to-sample geodesic instead of fitting a
      // spline that can invent lateral curvature between sparse planner samples.
      const steps=Math.max(2,Math.min(12,Math.ceil(arc/(.2*DEG))));
      for(let step=0;step<steps;step++){
        const t=step/steps,dir=slerpDirection(a,b,t,arc);
        const radius=radii[i]+(radii[i+1]-radii[i])*t;
        positions.push(dir[0]*radius,dir[1]*radius,dir[2]*radius);
      }
    }
    positions.push(...raw[raw.length-1]);
    return positions;
  }

  function updateLine(state,points,vp,smooth=false,geometryKey=null,useInertial=false){
    // PLAN/PRED geometry changes only on authoritative planner trajectory records;
    // telemetry and camera updates are much more frequent. Keep uniforms live but
    // avoid rebuilding/smoothing/uploading identical trajectory vertex buffers.
    if(geometryKey===null||state.geometryKey!==geometryKey){
      const positions=trajectoryPositions(points,smooth,useInertial);
      state.count=positions.length/3;
      const bytes=Math.max(12,positions.length*4);
      if(bytes>state.capacity){state.buffer?.destroy();state.capacity=2**Math.ceil(Math.log2(bytes));state.buffer=device.createBuffer({size:state.capacity,usage:GPUBufferUsage.VERTEX|GPUBufferUsage.COPY_DST});}
      if(positions.length)device.queue.writeBuffer(state.buffer,0,new Float32Array(positions));
      state.geometryKey=geometryKey;
    }
    const u=new Float32Array(20);u.set(vp,0);u.set(state.color,16);device.queue.writeBuffer(state.uniform,0,u);
  }



  function updateRawLine(state,positions,vp){
    state.count=positions.length/3;
    const bytes=Math.max(12,positions.length*4);
    if(bytes>state.capacity){state.buffer?.destroy();state.capacity=2**Math.ceil(Math.log2(bytes));state.buffer=device.createBuffer({size:state.capacity,usage:GPUBufferUsage.VERTEX|GPUBufferUsage.COPY_DST});}
    if(positions.length)device.queue.writeBuffer(state.buffer,0,new Float32Array(positions));
    const u=new Float32Array(20);u.set(vp,0);u.set(state.color,16);device.queue.writeBuffer(state.uniform,0,u);
  }

  function orbitalPlumePositions(craftModel,thrustRatio,cameraDistance,timeSeconds){
    if(thrustRatio<=.002)return [];
    const left=vUnit([craftModel[0],craftModel[1],craftModel[2]]);
    const forward=vUnit([craftModel[4],craftModel[5],craftModel[6]]);
    const down=vUnit([craftModel[8],craftModel[9],craftModel[10]]);
    const center=[craftModel[12],craftModel[13],craftModel[14]];
    const exhaust=vScale(forward,-1);
    const tail=vAdd(center,vScale(exhaust,craftLength*.47));
    // Symbolic telemetry plume: roots match the aft engine cluster while streak
    // length is camera-scaled so powered flight remains readable in full-orbit view.
    const plumeLength=Math.max(craftLength*1.6,Math.min(26000,cameraDistance*.018))*(.35+.85*Math.min(1,thrustRatio));
    const engineRoots=[-.085,0,.085].map((fraction,index)=>vAdd(tail,vAdd(vScale(left,craftSpan*fraction),vScale(down,index===1?craftLength*.018:0))));
    const positions=[];
    for(let e=0;e<engineRoots.length;e++)for(let i=0;i<9;i++){
      const phase=(i/9+timeSeconds*(.7+.08*e+.03*i))%1;
      const startD=plumeLength*(.03+.60*phase),endD=Math.min(plumeLength,startD+plumeLength*(.10+.06*((i+e)%3)));
      const jitterScale=plumeLength*(.004+.009*phase);
      const jitter=vAdd(vScale(left,Math.sin(i*2.17+timeSeconds*7.1+e)*jitterScale),vScale(down,Math.cos(i*1.71+timeSeconds*5.8-e)*jitterScale*.65));
      const a=vAdd(vAdd(engineRoots[e],vScale(exhaust,startD)),jitter);
      const b=vAdd(vAdd(engineRoots[e],vScale(exhaust,endD)),vScale(jitter,.45));
      positions.push(...a,...b);
    }
    return positions;
  }

  function requestRender(){if(!visible||frame)return;frame=requestAnimationFrame(()=>{frame=0;render();});}

  function render(){
    if(!visible)return;
    const {width,height}=resize();
    const t=snapshot?.telemetry||{};
    const hasPose=finite(t.latitude)&&finite(t.longitude);
    const visualLift=0;
    const frameState=hasPose?cameraFrame(t,width,height,visualLift):null;
    const craftTarget=hasPose?planetPoint({latitude:num(t.latitude),longitude:num(t.longitude),altitude:num(t.meanAltitude)},visualLift):null;
    if(autoCamera&&frameState)distance=frameState.distance;
    // Manual interaction changes only orbit angle/distance. Keep the live shuttle as
    // the pivot so dragging the view never leaves a moving orbiter behind in world space.
    let target=autoCamera?(frameState?.target||craftTarget):(craftTarget||lastCameraTarget);
    if(target)lastCameraTarget=target;
    const cam=camera(t,target,distance,visualLift),near=Math.max(.05,Math.min(1000,distance*.008)),projection=matPerspective(47*DEG,width/Math.max(1,height),near,4000000),view=matLookAt(cam.eye,cam.target,cam.up),vp=matMul(projection,view);
    const lightDir=sunDirection(t);
    // KSP's stock KerbinScaledSpace300 is mirrored in U with its geographic seam
    // rotated 90 degrees, and its sampled V axis is inverted in WebGPU:
    // u_stock = 0.75 - u_geo, v_stock = 1.0 - v_geo.
    const planetModel=matIdentity(); writeSolid(planetState,matMul(vp,planetModel),planetModel,[1,1,1,1],[-1,-1],[.75,1],false,lightDir,false,true);
    const sunWorld=sunModel(lightDir); writeSolid(sunState,matMul(vp,sunWorld),sunWorld,[1,.91,.63,1],[1,1],[0,0],false,lightDir,true);
    const craftModel=hasPose?bodyMatrix(t,visualLift):matIdentity();
    if(hasPose)for(const group of craftGroups)writeSolid(group.state,matMul(vp,craftModel),craftModel,group.color||[1,1,1,1],group.uvScale||[1,1],group.uvOffset||[0,0],Boolean(group.translucent),lightDir);

    const planned=displayablePlannedTrajectory(snapshot),predicted=futurePredictionTrajectory(snapshot),reference=displayableReferenceTrajectory(snapshot),projected=displayableProjectedTAEMTrajectory(snapshot),actual=snapshot?.actualTrajectory||[],orbital=snapshot?.orbitalTrajectory||[];
    const orbitMode=isOrbitMode(snapshot),burn=burnWindow3D(snapshot),apoapsis=orbitPOI(snapshot,"ap"),periapsis=orbitPOI(snapshot,"pe");
    const orbitEpoch=num(snapshot?.orbitalTrajectoryMeta?.generatedAtUT,-1).toFixed(2);
    updateLine(lines.orbit,orbital,vp,true,`orbit:${orbitEpoch}:${orbital.length}`,true);
    updateLine(lines.planned,planned,vp,true,`plan:${trajectoryFingerprint(planned)}`);
    updateLine(lines.predicted,predicted,vp,true,`pred:${trajectoryFingerprint(predicted)}`);
    updateLine(lines.reference,reference,vp,true,`ref:${trajectoryFingerprint(reference)}`);
    updateLine(lines.projected,projected,vp,true,`proj:${trajectoryFingerprint(projected)}`);
    updateLine(lines.actual,actual,vp,false);
    updateLine(lines.burn,orbitMode?burn.points:[],vp,true,`burn:${orbitEpoch}:${num(snapshot?.deorbitPlan?.burnUT,-1).toFixed(2)}:${burn.points.length}`,true);
    const poiSize=Math.max(3500,Math.min(30000,distance*.012));
    updateLine(lines.apoapsis,orbitMode?poiSpike(apoapsis,poiSize):[],vp,false,`ap:${orbitEpoch}:${num(apoapsis?.ut,-1).toFixed(2)}:${poiSize.toFixed(0)}`,true);
    updateLine(lines.periapsis,orbitMode?poiSpike(periapsis,poiSize):[],vp,false,`pe:${orbitEpoch}:${num(periapsis?.ut,-1).toFixed(2)}:${poiSize.toFixed(0)}`,true);

    const runway=runwayGeometry(snapshot?.site);
    const runwayKey=`runway:${num(snapshot?.site?.latitude,-99).toFixed(5)}:${num(snapshot?.site?.longitude,-99).toFixed(5)}:${num(snapshot?.site?.runwayHeading,-1).toFixed(1)}`;
    updateLine(lines.runway,orbitMode?[]:runway.axis,vp,true,`${runwayKey}:${orbitMode?"hidden":"live"}`);
    updateLine(lines.corridor,orbitMode?[]:runway.corridor,vp,true,`${runwayKey}:corridor:${orbitMode?"hidden":"live"}`);

    const currentThrust=finite(t.currentThrust)?Math.max(0,num(t.currentThrust)):finite(t.thrust)?Math.max(0,num(t.thrust)):0;
    const availableThrust=finite(t.availableThrust)?Math.max(0,num(t.availableThrust)):0,throttle=finite(t.throttle)?Math.max(0,Math.min(1,num(t.throttle))):0;
    const thrustRatio=Math.max(throttle,availableThrust>1?currentThrust/availableThrust:0);
    const plumePositions=orbitMode&&hasPose&&thrustRatio>.002?orbitalPlumePositions(craftModel,thrustRatio,distance,performance.now()/1000):[];
    updateRawLine(lines.plume,plumePositions,vp);

    const encoder=device.createCommandEncoder({label:"telemetry-3d-frame"});
    const pass=encoder.beginRenderPass({
      colorAttachments:[{view:msaaTexture.createView(),resolveTarget:context.getCurrentTexture().createView(),clearValue:{r:.012,g:.016,b:.023,a:1},loadOp:"clear",storeOp:"discard"}],
      depthStencilAttachment:{view:depthTexture.createView(),depthClearValue:1,depthLoadOp:"clear",depthStoreOp:"store"}
    });
    pass.setPipeline(solidPipeline);
    pass.setVertexBuffer(0,planetMesh.vertex);pass.setIndexBuffer(planetMesh.index,"uint32");pass.setBindGroup(0,planetState.bind);pass.drawIndexed(planetMesh.indexCount);
    pass.setVertexBuffer(0,sunMesh.vertex);pass.setIndexBuffer(sunMesh.index,"uint32");pass.setBindGroup(0,sunState.bind);pass.drawIndexed(sunMesh.indexCount);
    if(hasPose){
      pass.setVertexBuffer(0,craft.gpu.vertex);pass.setIndexBuffer(craft.gpu.index,"uint32");
      for(const group of craftGroups){if(group.translucent||num(group.indexCount)<=0)continue;pass.setBindGroup(0,group.state.bind);pass.drawIndexed(num(group.indexCount),1,num(group.firstIndex));}
      pass.setPipeline(alphaPipeline);
      for(const group of craftGroups){if(!group.translucent||num(group.indexCount)<=0)continue;pass.setBindGroup(0,group.state.bind);pass.drawIndexed(num(group.indexCount),1,num(group.firstIndex));}
    }
    pass.setPipeline(linePipeline);
    for(const state of [lines.corridor,lines.runway,lines.orbit,lines.burn,lines.apoapsis,lines.periapsis,lines.planned,lines.reference,lines.projected,lines.predicted,lines.actual])if(state.count>1){pass.setVertexBuffer(0,state.buffer);pass.setBindGroup(0,state.bind);pass.draw(state.count);}
    if(lines.plume.count>1){pass.setPipeline(plumePipeline);pass.setVertexBuffer(0,lines.plume.buffer);pass.setBindGroup(0,lines.plume.bind);pass.draw(lines.plume.count);}
    pass.end();device.queue.submit([encoder.finish()]);

    const altitude=finite(t.meanAltitude)?`${(num(t.meanAltitude)/1000).toFixed(1)} km`:"—";
    const phase=String(snapshot?.phase||"FLIGHT").toUpperCase();
    const cameraMode=autoCamera?(frameState?.mode==="ORBIT"?"AUTO ORBIT":"AUTO FLIGHT"):"MANUAL";
    if(orbitMode){
      const plan=snapshot?.deorbitPlan,burnDuration=plan&&finite(plan.estimatedBurnDuration)?Math.max(0,num(plan.estimatedBurnDuration)):0,burnStart=plan&&finite(plan.burnUT)?num(plan.burnUT)-burnDuration*.5:null,toBurn=finite(burnStart)&&finite(t.ut)?num(burnStart)-num(t.ut):null;
      const powered=thrustRatio>.002;
      statusEl.textContent=`${powered?"DEORBIT BURN":plan?"ORBIT / DEORBIT":"ORBIT"} · ${cameraMode}`;
      const warpRate=finite(t.timeWarpRate)?Math.max(1,num(t.timeWarpRate)):1,decel=finite(t.deceleration)?Math.max(0,num(t.deceleration)):null;
      metaEl.textContent=`V ${finite(t.orbitalSpeed)?Math.round(num(t.orbitalSpeed))+" M/S":"—"} · AP ${formatDistance(t.apoapsisAltitude).toUpperCase()} ${countdownText(t.timeToApoapsis)} · PE ${formatDistance(t.periapsisAltitude).toUpperCase()} ${countdownText(t.timeToPeriapsis)}${plan?` · BURN ${countdownText(toBurn)}`:""} · W ${warpRate.toFixed(warpRate<10?1:0)}×${decel!==null?` · DECEL ${decel.toFixed(2)} M/S²`:""}`;
      metaEl.title=`Instantaneous osculating orbit${snapshot?.orbitalTrajectoryMeta?.underThrust?" under current thrust":""} · ${powered?"animated engine plume active":"engines off"} · AP/PE markers use live orbital timing · ${num(craft.metadata?.partCount)} real parts · ${craftLength.toFixed(1)}m × ${craftSpan.toFixed(1)}m`;
    }else{
      const orbitLabel=orbital.length>=2?(snapshot?.orbitalTrajectoryMeta?.endReason==="atmosphere-interface"?" · VAC ARC":" · OSC ORBIT"):"";
      statusEl.textContent=`${phase} · ${cameraMode}`;
      metaEl.textContent=`ALT ${altitude.toUpperCase()} · VIEW ${formatDistance(distance).toUpperCase()}${orbitLabel}`;
      metaEl.title=`${autoCamera?"Auto-framing flight corridor":"Manual orbit locked to live shuttle"} · predicted ${predicted.length} · plan ${planned.length} · reference ${reference.length} · projected TAEM ${projected.length} · actual trail ${actual.length} · ${num(craft.metadata?.partCount)} real parts · ${craftLength.toFixed(1)}m × ${craftSpan.toFixed(1)}m · model scale 1:1${kerbin.real?"":" · map fallback"}`;
    }
    statusEl.parentElement?.classList.remove("alert");
    if(orbitMode&&lines.plume.count>1)requestRender();
  }

  function point(event){const r=canvas.getBoundingClientRect();return{x:event.clientX-r.left,y:event.clientY-r.top};}
  function enterManualCamera(){autoCamera=false;}
  canvas.title="3D flight corridor · drag to orbit around shuttle · wheel/pinch to zoom · double-click to restore auto-frame";
  canvas.addEventListener("wheel",event=>{if(!visible)return;event.preventDefault();enterManualCamera();distance=Math.max(8,Math.min(2400000,distance*Math.exp(event.deltaY*.0022)));requestRender();},{passive:false});
  canvas.addEventListener("dblclick",()=>{autoCamera=true;yaw=.78;pitch=.36;distance=110;requestRender();});
  canvas.addEventListener("pointerdown",event=>{if(event.pointerType==="mouse"&&event.button!==0)return;event.preventDefault();enterManualCamera();canvas.setPointerCapture(event.pointerId);const p=point(event);pointers.set(event.pointerId,p);lastPointer=p;canvas.style.cursor="grabbing";if(pointers.size===2){const[a,b]=[...pointers.values()];pinchDistance=Math.hypot(a.x-b.x,a.y-b.y);}});
  canvas.addEventListener("pointermove",event=>{if(!pointers.has(event.pointerId))return;event.preventDefault();const p=point(event);pointers.set(event.pointerId,p);if(pointers.size>=2){const[a,b]=[...pointers.values()];const d=Math.hypot(a.x-b.x,a.y-b.y);if(pinchDistance>0)distance=Math.max(8,Math.min(2400000,distance*pinchDistance/d));pinchDistance=d;}else if(lastPointer){yaw-=(p.x-lastPointer.x)*.006;pitch=Math.max(-1.15,Math.min(1.15,pitch+(p.y-lastPointer.y)*.005));lastPointer=p;}requestRender();});
  function release(event){pointers.delete(event.pointerId);if(pointers.size===1)lastPointer=[...pointers.values()][0];else if(!pointers.size){lastPointer=null;pinchDistance=0;canvas.style.cursor="grab";}requestRender();}
  canvas.addEventListener("pointerup",release);canvas.addEventListener("pointercancel",release);
  new ResizeObserver(()=>requestRender()).observe(canvas);

  statusEl.textContent="WEBGPU · READY";
  metaEl.textContent="";
  statusEl.parentElement?.classList.remove("alert");
  return {
    update(value){snapshot=value;if(visible)requestRender();},
    setVisible(value){visible=Boolean(value);if(visible){statusEl.textContent="WEBGPU · LIVE";requestRender();}else{if(frame){cancelAnimationFrame(frame);frame=0;}statusEl.textContent="WEBGPU · PAUSED";}},
  };
}
