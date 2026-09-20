(() => {
  const $ = (id) => document.getElementById(id);
  const nav = $("navball");
  const map = $("trajectory-map");
  const navCtx = nav.getContext("2d");
  const mapCtx = map.getContext("2d");
  const kerbin = new Image();
  kerbin.decoding = "async";
  kerbin.src = "/asset/kerbin-map?v=stock-geographic-1";

  let snapshot = null;
  let latestLiveSnapshot = null;
  const archiveReplay = {
    active: false, frames: [], run: null, index: 0, playing: false, speed: 20,
    raf: 0, anchorWall: 0, anchorUt: 0,
  };
  let eventSource = null;
  let reconnectTimer = null;
  let reconnectDelay = 350;
  let lastSnapshotAt = 0;
  let rlPollInFlight = false;
  const mapView = { zoom: 1, offsetX: 0, offsetY: 0, minZoom: 1, maxZoom: 360, autoFocus: true };

  // Exact outer silhouette traced from the user's supplied pure-white marker.
  // Coordinates are centered pixels in the canonical 202x256 image; nose = up.
  const SHUTTLE_MARKER_OUTLINE = [
    [12,-128],[19,-125],[27,-106],[32,-81],[33,-56],[48,16],[89,58],[91,65],
    [91,85],[37,91],[37,105],[32,105],[32,110],[23,110],[21,118],[8,119],
    [-1,117],[-1,107],[-2,111],[-10,111],[-11,105],[-16,105],[-17,93],[-70,88],
    [-70,68],[-67,61],[-25,19],[-17,-8],[-8,-51],[-5,-88],[-2,-101],[6,-123],
  ];
  const mapPointers = new Map();
  let mapDragPoint = null;
  let pinchDistance = 0;
  let pinchCenter = null;
  const scene3dPanel = $("scene3d-panel");
  const scene3dToggle = $("toggle-3d");
  let scene3dViewer = null;
  let scene3dLoading = null;

  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const C = () => ({
    text: css("--text"), muted: css("--muted"), green: css("--green"),
    cyan: css("--cyan"), orange: css("--orange"), line: css("--line")
  });

  const n = (value, fallback = 0) => {
    const out = Number(value);
    return Number.isFinite(out) ? out : fallback;
  };
  const finite = (value) => value !== null && value !== undefined && value !== "" && Number.isFinite(Number(value));
  const signed = (value) => {
    let x = n(value) % 360;
    if (x > 180) x -= 360;
    if (x < -180) x += 360;
    return x;
  };
  const fmt = (value, digits = 1, unit = "") => finite(value) ? `${n(value).toFixed(digits)}${unit}` : "—";
  const fmtSigned = (value, digits = 1, unit = "") => finite(value) ? `${n(value) >= 0 ? "+" : ""}${n(value).toFixed(digits)}${unit}` : "—";
  const distance = (value) => !finite(value) ? "—" : Math.abs(n(value)) >= 1000 ? `${(n(value) / 1000).toFixed(1)} km` : `${n(value).toFixed(0)} m`;
  const speed = (value) => !finite(value) ? "—" : `${n(value).toFixed(0)} m/s`;
  const pressure = (value) => !finite(value) ? "—" : Math.abs(n(value)) >= 1000 ? `${(n(value) / 1000).toFixed(1)} kPa` : `${n(value).toFixed(0)} Pa`;
  const signedDistance = (value) => {
    if (!finite(value)) return "—";
    const x = n(value), magnitude = Math.abs(x), sign = x > 0 ? "+" : "";
    return magnitude >= 1000 ? `${sign}${(x / 1000).toFixed(1)} km` : `${sign}${x.toFixed(0)} m`;
  };
  const specificEnergy = (value) => {
    if (!finite(value)) return "—";
    const x = n(value), sign = x > 0 ? "+" : "";
    return Math.abs(x) >= 1000 ? `${sign}${(x / 1000).toFixed(1)} kJ/kg` : `${sign}${x.toFixed(0)} J/kg`;
  };
  const trajectoryCount = (value) => Array.isArray(value)
    ? value.reduce((count, point) => count + (point && finite(point.latitude) && finite(point.longitude) ? 1 : 0), 0)
    : 0;
  const trajectoryLabel = (label, count) => count >= 2 ? `${label} ${count}` : count === 1 ? `${label} 1PT` : `${label} —`;
  const isTerminalPhase = (value) => /TAEM|HAC|FINAL|PREFLARE|FLARE|TOUCHDOWN|ROLLOUT/.test(String(value || "").toUpperCase());
  const entryPlanDisplayable = (source) => {
    const g = source?.guidanceState || {};
    return g.entryPlanValid === true && g.entryPlanTerminalReady === true && !isTerminalPhase(source?.phase);
  };
  const terminalReferenceDisplayable = (source) => {
    const g = source?.guidanceState || {};
    return g.terminalPathCommitted === true || g.terminalPathCaptured === true || g.terminalPathComplete === true;
  };
  const displayablePlannedTrajectory = (source) => {
    const allowed = isTerminalPhase(source?.phase) ? terminalReferenceDisplayable(source) : entryPlanDisplayable(source);
    return allowed && Array.isArray(source?.plannedTrajectory) ? source.plannedTrajectory : [];
  };
  const displayableReferenceTrajectory = (source) => terminalReferenceDisplayable(source) && Array.isArray(source?.referenceTrajectory)
    ? source.referenceTrajectory : [];
  const displayableProjectedTAEMTrajectory = (source) => entryPlanDisplayable(source) && Array.isArray(source?.projectedTAEMTrajectory)
    ? source.projectedTAEMTrajectory : [];

  const predictionHasFuture = (trajectory, currentUt) => {
    if (!Array.isArray(trajectory) || !trajectory.length) return false;
    if (!finite(currentUt)) return true;
    let lastTimed = null;
    for (const point of trajectory) if (point && finite(point.ut)) lastTimed = n(point.ut);
    return lastTimed === null ? true : lastTimed >= n(currentUt) - .25;
  };
  const futurePredictionTrajectory = (source) => {
    const trajectory = Array.isArray(source?.predictedTrajectory) ? source.predictedTrajectory : [];
    const currentUt = source?.telemetry?.ut;
    if (!trajectory.length || !finite(currentUt)) return trajectory;
    const now = n(currentUt);
    let firstFuture = -1, timed = 0;
    for (let i = 0; i < trajectory.length; i++) {
      const point = trajectory[i];
      if (!point || !finite(point.ut)) continue;
      timed += 1;
      if (firstFuture < 0 && n(point.ut) >= now - .25) firstFuture = i;
    }
    if (!timed) return trajectory;
    if (firstFuture < 0) return [];
    return trajectory.slice(Math.max(0, firstFuture - 1));
  };

  const predictionFreshness = (source) => {
    const trajectory = Array.isArray(source?.predictedTrajectory) ? source.predictedTrajectory : [];
    const currentUt = source?.telemetry?.ut;
    if (!trajectory.length || !finite(currentUt)) return {age:0,horizon:0,stale:false};
    let firstUt = null, lastUt = null;
    for (const point of trajectory) {
      if (!point || !finite(point.ut)) continue;
      const ut = n(point.ut);
      if (firstUt === null) firstUt = ut;
      lastUt = ut;
    }
    if (firstUt === null || lastUt === null) return {age:0,horizon:0,stale:false};
    const age = Math.max(0, n(currentUt) - firstUt);
    const horizon = Math.max(0, lastUt - firstUt);
    const staleAfter = Math.max(20, horizon * .15);
    return {age,horizon,stale:age > staleAfter};
  };
  const isEntryFlightPhase = (value) => /ENTRY|MM304|TAEM|HAC|FINAL|PREFLARE|FLARE|TOUCHDOWN|ROLLOUT|ATTITUDE RECOVERY/.test(String(value || "").toUpperCase());
  const isOrbitMode = (source) => {
    if (!source || isEntryFlightPhase(source.phase)) return false;
    const t = source.telemetry || {};
    const atmosphere = source.orbitalTrajectoryMeta?.atmosphereDepth;
    if (trajectoryCount(source.orbitalTrajectory) >= 2) return true;
    if (finite(t.meanAltitude) && finite(atmosphere) && n(t.meanAltitude) > n(atmosphere) + 500) return true;
    return String(source.command?.controlProfile || "").toLowerCase() === "orbital";
  };
  const duration = (value) => {
    if (!finite(value)) return "—";
    const total = Math.max(0, Math.round(n(value)));
    const h = Math.floor(total / 3600), m = Math.floor((total % 3600) / 60), s = total % 60;
    if (h) return `${h}:${String(m).padStart(2,"0")}:${String(s).padStart(2,"0")}`;
    return `${m}:${String(s).padStart(2,"0")}`;
  };
  const countdown = (value) => !finite(value) ? "—" : n(value) >= 0 ? `T−${duration(value)}` : `T+${duration(-n(value))}`;
  const acceleration = (value) => !finite(value) ? "—" : `${Math.max(0,n(value)).toFixed(2)} m/s²`;
  const thrust = (value) => !finite(value) ? "—" : Math.abs(n(value)) >= 1000 ? `${(n(value)/1000).toFixed(0)} kN` : `${n(value).toFixed(0)} N`;
  const warp = (t) => {
    const rate = finite(t?.timeWarpRate) ? Math.max(1, n(t.timeWarpRate)) : 1;
    const mode = String(t?.timeWarpMode || "none").toUpperCase();
    return rate > 1 ? `${rate.toFixed(rate < 10 ? 1 : 0)}× ${mode}` : "1×";
  };
  const dir3 = (heading, elevation) => {
    const h=n(heading)*Math.PI/180, p=n(elevation)*Math.PI/180, cp=Math.cos(p);
    return [cp*Math.sin(h), cp*Math.cos(h), Math.sin(p)]; // east, north, up
  };
  const dot3 = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  const cross3 = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const unit3 = (a) => { const l=Math.hypot(a[0],a[1],a[2])||1; return [a[0]/l,a[1]/l,a[2]/l]; };
  const add3 = (a,b) => [a[0]+b[0],a[1]+b[1],a[2]+b[2]];
  const scale3 = (a,k) => [a[0]*k,a[1]*k,a[2]*k];
  const progradeAttitude = (t) => ({
    heading: finite(t?.orbitalHeading) ? n(t.orbitalHeading) : n(t?.groundTrackHeading, n(t?.heading)),
    pitch: finite(t?.orbitalFlightPathAngle) ? n(t.orbitalFlightPathAngle) : n(t?.flightPathAngle),
  });
  const retrogradeAttitude = (t) => { const p=progradeAttitude(t); return {heading:(p.heading+180)%360,pitch:-p.pitch}; };
  const angularAttitudeError = (t,target) => {
    if (![t?.heading,t?.pitch,target?.heading,target?.pitch].every(finite)) return null;
    const a=dir3(t.heading,t.pitch), b=dir3(target.heading,target.pitch);
    return Math.acos(Math.max(-1,Math.min(1,dot3(a,b))))*180/Math.PI;
  };
  const retrogradeError = (t) => angularAttitudeError(t, retrogradeAttitude(t));
  function interpolateTrajectoryAtUT(points, targetUT) {
    if (!Array.isArray(points) || points.length < 2 || !finite(targetUT)) return null;
    for (let i=1;i<points.length;i++) {
      const a=points[i-1], b=points[i];
      if (!finite(a?.ut) || !finite(b?.ut)) continue;
      const au=n(a.ut), bu=n(b.ut);
      if (targetUT < Math.min(au,bu)-1e-6 || targetUT > Math.max(au,bu)+1e-6) continue;
      const f=Math.abs(bu-au)<1e-9?0:(n(targetUT)-au)/(bu-au);
      const lerp=(x,y)=>finite(x)&&finite(y)?n(x)+(n(y)-n(x))*f:null;
      const lerpLon=(x,y)=>{ if(!finite(x)||!finite(y))return null; const d=signed(n(y)-n(x)); return signed(n(x)+d*f); };
      return {
        ut:n(targetUT), latitude:lerp(a.latitude,b.latitude), longitude:lerpLon(a.longitude,b.longitude), altitude:lerp(a.altitude,b.altitude),
        inertialLatitude:lerp(a.inertialLatitude,b.inertialLatitude), inertialLongitude:lerpLon(a.inertialLongitude,b.inertialLongitude), kind:"orbit-poi"
      };
    }
    return null;
  }
  function orbitPointOfInterest(source, kind) {
    const t=source?.telemetry||{}, dt=kind==="apoapsis"?t.timeToApoapsis:t.timeToPeriapsis;
    if (!finite(t.ut) || !finite(dt)) return null;
    return interpolateTrajectoryAtUT(source?.orbitalTrajectory, n(t.ut)+Math.max(0,n(dt)));
  }
  function burnWindow(source) {
    const plan=source?.deorbitPlan, orbit=source?.orbitalTrajectory||[], t=source?.telemetry||{};
    if (!plan || !finite(plan.burnUT) || !Array.isArray(orbit) || orbit.length<2) return {points:[],start:null,mid:null,end:null};
    const half=Math.max(1, finite(plan.estimatedBurnDuration)?Math.max(0,n(plan.estimatedBurnDuration))/2:1);
    const midUT=n(plan.burnUT), startUT=midUT-half, endUT=midUT+half, currentUT=finite(t.ut)?n(t.ut):null;
    let start=interpolateTrajectoryAtUT(orbit,startUT);
    let startIsLive=false;
    if(!start && currentUT!==null && currentUT>=startUT && currentUT<=endUT && finite(t.latitude) && finite(t.longitude)){
      start={ut:currentUT,latitude:n(t.latitude),longitude:n(t.longitude),altitude:n(t.meanAltitude)}; startIsLive=true;
    }
    const mid=interpolateTrajectoryAtUT(orbit,midUT), end=interpolateTrajectoryAtUT(orbit,endUT);
    return {points:[start,mid,end].filter(Boolean),start,mid,end,startUT,endUT,startIsLive};
  }
  const setTone = (target, tone = "") => {
    const element = typeof target === "string" ? $(target) : target;
    if (!element) return;
    element.classList.remove("tone-good", "tone-warn", "tone-bad");
    if (tone) element.classList.add(`tone-${tone}`);
  };
  const errorTone = (error, goodLimit, warnLimit) => !finite(error) ? "" : Math.abs(n(error)) <= goodLimit ? "good" : Math.abs(n(error)) <= warnLimit ? "warn" : "bad";
  const setTitle = (id, value) => { const element = $(id); if (element) element.title = value || ""; };

  function resizeCanvas(canvas, ctx) {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { width: rect.width, height: rect.height };
  }

  function normalizeMapOffsetX(width = map.clientWidth) {
    const span = Math.max(1, width * mapView.zoom);
    mapView.offsetX = ((mapView.offsetX + span / 2) % span + span) % span - span / 2;
  }

  function clampMapView(width = map.clientWidth, height = map.clientHeight) {
    mapView.zoom = Math.max(mapView.minZoom, Math.min(mapView.maxZoom, mapView.zoom));
    normalizeMapOffsetX(width);
    const extraY = height * (mapView.zoom - 1) / 2;
    mapView.offsetY = Math.max(-extraY, Math.min(extraY, mapView.offsetY));
  }

  function mapFocusPoints() {
    const points = [];
    const craft = snapshot?.telemetry;
    const phase = String(snapshot?.phase || "").toUpperCase();
    const altitude = finite(craft?.meanAltitude) ? Math.max(0, n(craft.meanAltitude)) : 0;
    const finalApproach = /PREFLARE|FINAL|FLARE|TOUCHDOWN|ROLLOUT/.test(phase);
    const terminalApproach = finalApproach || /TAEM|HAC/.test(phase);
    const horizon = finalApproach
      ? Math.max(12000, Math.min(35000, altitude * 3 + 8000))
      : terminalApproach
        ? Math.max(35000, Math.min(90000, altitude * 4 + 15000))
        : Math.max(60000, Math.min(160000, altitude * 3 + 45000));
    const append = (point) => {
      if (point && finite(point.latitude) && finite(point.longitude)) points.push(point);
    };
    const appendTrajectory = (trajectory) => {
      if (!Array.isArray(trajectory)) return;
      let kept = 0;
      for (const point of trajectory) {
        if (!point || !finite(point.latitude) || !finite(point.longitude)) continue;
        const range = greatCircleMeters(craft, point);
        if (!finite(range) || range <= horizon || kept < 2) {
          append(point);
          kept += 1;
        }
      }
    };

    append(craft);
    const orbital = snapshot?.orbitalTrajectory;
    if (Array.isArray(orbital)) for (const point of orbital) append(point);
    appendTrajectory(futurePredictionTrajectory(snapshot));
    appendTrajectory(displayablePlannedTrajectory(snapshot));
    appendTrajectory(displayableReferenceTrajectory(snapshot));
    appendTrajectory(displayableProjectedTAEMTrajectory(snapshot));

    const site = snapshot?.site;
    const siteRange = greatCircleMeters(craft, site);
    if (terminalApproach || (finite(siteRange) && siteRange <= horizon * 1.08)) append(site);
    return points;
  }

  function visibleGroundWidthMeters(latitude = 0) {
    const lat = finite(latitude) ? n(latitude) : 0;
    const longitudeRadians = (360 / Math.max(1, mapView.zoom)) * Math.PI / 180;
    return 600000 * longitudeRadians * Math.max(.12, Math.cos(lat * Math.PI / 180));
  }

  function niceMapDistance(value) {
    const safe = Math.max(1, n(value, 1));
    const magnitude = 10 ** Math.floor(Math.log10(safe));
    const normalized = safe / magnitude;
    const nice = normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10;
    return nice * magnitude;
  }

  function focusMapOnFlight(width = map.clientWidth, height = map.clientHeight) {
    if (!mapView.autoFocus || !snapshot || !width || !height) return false;
    const points = mapFocusPoints();
    if (!points.length) return false;

    const anchorLongitude = finite(snapshot?.telemetry?.longitude)
      ? n(snapshot.telemetry.longitude)
      : n(points[0].longitude);
    const longitudes = points.map((point) => {
      let longitude = n(point.longitude);
      while (longitude - anchorLongitude > 180) longitude -= 360;
      while (longitude - anchorLongitude < -180) longitude += 360;
      return longitude;
    });
    const latitudes = points.map((point) => Math.max(-90, Math.min(90, n(point.latitude))));
    const minLongitude = Math.min(...longitudes), maxLongitude = Math.max(...longitudes);
    const minLatitude = Math.min(...latitudes), maxLatitude = Math.max(...latitudes);

    const phase = String(snapshot?.phase || "").toUpperCase();
    const finalApproach = /PREFLARE|FINAL|FLARE|TOUCHDOWN|ROLLOUT/.test(phase);
    const terminalApproach = finalApproach || /TAEM|HAC/.test(phase);
    const longitudeFloor = finalApproach ? .18 : terminalApproach ? .35 : .65;
    const latitudeFloor = finalApproach ? .12 : terminalApproach ? .25 : .42;
    const padding = finalApproach ? 1.055 : terminalApproach ? 1.07 : 1.085;
    const longitudeSpan = Math.max(longitudeFloor, maxLongitude - minLongitude);
    const latitudeSpan = Math.max(latitudeFloor, maxLatitude - minLatitude);

    // Tactical framing: fill almost the entire map with the live flight corridor.
    // The previous 12x cap left most of the canvas as irrelevant Kerbin surface.
    mapView.zoom = Math.max(mapView.minZoom, Math.min(
      mapView.maxZoom,
      360 / (longitudeSpan * padding),
      180 / (latitudeSpan * padding),
    ));

    const centerLongitude = (minLongitude + maxLongitude) / 2;
    const wrappedLongitude = ((centerLongitude + 180) % 360 + 360) % 360 - 180;
    const centerLatitude = (minLatitude + maxLatitude) / 2;
    const baseX = (wrappedLongitude + 180) / 360 * width;
    const baseY = (90 - centerLatitude) / 180 * height;
    mapView.offsetX = -(baseX - width / 2) * mapView.zoom;
    mapView.offsetY = -(baseY - height / 2) * mapView.zoom;
    clampMapView(width, height);
    return true;
  }

  function zoomMapAt(x, y, nextZoom) {
    const width = map.clientWidth;
    const height = map.clientHeight;
    if (!width || !height) return;
    const oldZoom = mapView.zoom;
    const newZoom = Math.max(mapView.minZoom, Math.min(mapView.maxZoom, nextZoom));
    if (Math.abs(newZoom - oldZoom) < 0.0001) return;
    const worldX = (x - width / 2 - mapView.offsetX) / oldZoom;
    const worldY = (y - height / 2 - mapView.offsetY) / oldZoom;
    mapView.zoom = newZoom;
    mapView.offsetX = x - width / 2 - worldX * newZoom;
    mapView.offsetY = y - height / 2 - worldY * newZoom;
    clampMapView(width, height);
    drawMap();
  }

  function resetMapView() {
    mapView.zoom = 1;
    mapView.offsetX = 0;
    mapView.offsetY = 0;
    mapView.autoFocus = true;
    drawMap();
  }

  function mapEventPoint(event) {
    const rect = map.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function projectDirectionToNavball(t, target, radius) {
    if (![t?.heading,t?.pitch,target?.heading,target?.pitch].every(finite)) return null;
    const forward = dir3(t.heading, t.pitch);
    const h = n(t.heading) * Math.PI / 180;
    const unrolledRight = [Math.cos(h), -Math.sin(h), 0];
    const unrolledUp = unit3(cross3(unrolledRight, forward));
    const rr = n(t.roll) * Math.PI / 180;
    const right = unit3(add3(scale3(unrolledRight, Math.cos(rr)), scale3(unrolledUp, -Math.sin(rr))));
    const craftUp = unit3(add3(scale3(unrolledUp, Math.cos(rr)), scale3(unrolledRight, Math.sin(rr))));
    const direction = dir3(target.heading, target.pitch);
    const x = dot3(direction, right), y = dot3(direction, craftUp), z = dot3(direction, forward);
    const angle = Math.acos(Math.max(-1, Math.min(1, z)));
    const behind = z < 0;
    const radial = Math.min(radius * .76, angle / (Math.PI / 2) * radius * .76);
    const tangent = Math.hypot(x, y);
    if (tangent < 1e-7) {
      if (!behind) return {x:0,y:0,behind,angle:angle*180/Math.PI};
      // At the exact antipode the projected tangent is mathematically undefined.
      // Never collapse a behind-velocity cue onto the boresight: choose a stable
      // rim direction from the attitude deltas so prograde stays off-center while
      // the vehicle is genuinely aligned retrograde.
      const headingDelta = signed(n(target.heading) - n(t.heading));
      const pitchDelta = n(target.pitch) - n(t.pitch);
      let ex = Math.abs(headingDelta) > .01 ? Math.sign(headingDelta) : 0;
      let ey = Math.abs(pitchDelta) > .01 ? -Math.sign(pitchDelta) : 0;
      if (!ex && !ey) ex = 1;
      const el = Math.hypot(ex, ey) || 1;
      return {x:ex/el*radius*.76,y:ey/el*radius*.76,behind:true,angle:angle*180/Math.PI};
    }
    return {x:(x/tangent)*radial,y:-(y/tangent)*radial,behind,angle:angle*180/Math.PI};
  }

  function drawVelocityNavCue(ctx, cue, color, retrograde = false, target = false) {
    if (!cue) return;
    ctx.save();
    ctx.translate(cue.x, cue.y);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = target ? 2.2 : 1.9;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.globalAlpha = cue.behind ? .48 : .98;
    if (cue.behind) ctx.setLineDash([3, 3]);

    // Keep the maneuver/target frame separate from the stock velocity glyph.
    // KSP's navball uses a ring-and-dot for prograde and a ring-and-X for
    // retrograde; unlike the old corner ticks, both remain readable at a glance.
    if (target) {
      ctx.rotate(Math.PI / 4);
      ctx.strokeRect(-13, -13, 26, 26);
      ctx.rotate(-Math.PI / 4);
    }

    ctx.beginPath();
    ctx.arc(0, 0, 7, 0, Math.PI * 2);
    ctx.stroke();
    if (retrograde) {
      ctx.beginPath();
      ctx.moveTo(-7.5, -7.5); ctx.lineTo(7.5, 7.5);
      ctx.moveTo(7.5, -7.5); ctx.lineTo(-7.5, 7.5);
      ctx.stroke();
    } else {
      ctx.beginPath();
      ctx.arc(0, 0, 2.2, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(0, -13); ctx.lineTo(0, -9);
      ctx.moveTo(0, 13); ctx.lineTo(0, 9);
      ctx.moveTo(-13, 0); ctx.lineTo(-9, 0);
      ctx.moveTo(13, 0); ctx.lineTo(9, 0);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawNavball() {
    const { width, height } = resizeCanvas(nav, navCtx);
    const c = C();
    navCtx.clearRect(0, 0, width, height);
    const t = snapshot?.telemetry || {};
    const cmd = snapshot?.command || {};
    const cx = width / 2;
    const cy = height / 2;
    const r = Math.max(72, Math.min(width, height) * 0.43);
    const pitch = n(t.pitch);
    const roll = n(t.roll);
    const heading = n(t.heading);
    const fpa = n(t.flightPathAngle);
    const ppd = r / 33;
    const orbitMode = isOrbitMode(snapshot);

    navCtx.save();
    navCtx.beginPath(); navCtx.arc(cx, cy, r, 0, Math.PI * 2); navCtx.clip();
    navCtx.fillStyle = "#10202a"; navCtx.fillRect(cx-r, cy-r, 2*r, 2*r);
    navCtx.save();
    navCtx.translate(cx, cy);
    navCtx.rotate(-roll * Math.PI / 180);
    navCtx.translate(0, pitch * ppd);
    navCtx.fillStyle = "#292019"; navCtx.fillRect(-r*1.6, 0, r*3.2, r*2.2);
    navCtx.strokeStyle = c.text; navCtx.globalAlpha = .82; navCtx.lineWidth = 1.5;
    navCtx.beginPath(); navCtx.moveTo(-r*1.5, 0); navCtx.lineTo(r*1.5, 0); navCtx.stroke();
    navCtx.globalAlpha = .56;
    navCtx.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    navCtx.textBaseline = "middle";
    for (let deg = -80; deg <= 80; deg += 10) {
      if (deg === 0) continue;
      const y = -deg * ppd;
      const major = deg % 20 === 0;
      const half = major ? r * .24 : r * .14;
      navCtx.beginPath(); navCtx.moveTo(-half, y); navCtx.lineTo(half, y); navCtx.stroke();
      if (major) {
        navCtx.fillStyle = c.text;
        navCtx.textAlign = "right"; navCtx.fillText(String(Math.abs(deg)), -half - 7, y);
        navCtx.textAlign = "left"; navCtx.fillText(String(Math.abs(deg)), half + 7, y);
      }
    }
    navCtx.restore();
    navCtx.restore();

    navCtx.strokeStyle = c.line; navCtx.lineWidth = 2;
    navCtx.beginPath(); navCtx.arc(cx, cy, r, 0, Math.PI * 2); navCtx.stroke();

    navCtx.save(); navCtx.translate(cx, cy);
    navCtx.strokeStyle = c.text; navCtx.lineWidth = 2.2;
    navCtx.beginPath(); navCtx.moveTo(-34, 0); navCtx.lineTo(-10, 0); navCtx.lineTo(0, 7); navCtx.lineTo(10, 0); navCtx.lineTo(34, 0); navCtx.stroke();

    if (orbitMode) {
      const prograde = projectDirectionToNavball(t, progradeAttitude(t), r);
      const retrograde = projectDirectionToNavball(t, retrogradeAttitude(t), r);
      drawVelocityNavCue(navCtx, prograde, c.cyan, false, false);
      drawVelocityNavCue(navCtx, retrograde, c.orange, true, false);
      const burnTarget = Boolean(snapshot?.deorbitPlan) || cmd.useInertialDirection === true || /COAST TO BURN|BURN SETUP|DEORBIT BURN/.test(String(snapshot?.phase || "").toUpperCase());
      if (burnTarget) drawVelocityNavCue(navCtx, retrograde, c.orange, true, true);
    } else {
      const fpaOffset = Math.max(-r*.66, Math.min(r*.66, (pitch - fpa) * ppd));
      navCtx.strokeStyle = c.cyan; navCtx.lineWidth = 1.8;
      navCtx.beginPath(); navCtx.arc(0, fpaOffset, 7, 0, Math.PI * 2); navCtx.moveTo(-15, fpaOffset); navCtx.lineTo(-7, fpaOffset); navCtx.moveTo(7, fpaOffset); navCtx.lineTo(15, fpaOffset); navCtx.moveTo(0, fpaOffset-14); navCtx.lineTo(0, fpaOffset-7); navCtx.stroke();
    }

    const aerodynamicDirector = !orbitMode && finite(cmd.targetAoA) && finite(t.angleOfAttack);
    if (!orbitMode && (aerodynamicDirector || (finite(cmd.targetPitch) && finite(cmd.targetHeading)))) {
      // Aerodynamic guidance does not command a surface-Euler pose. AoA is the
      // pitch-axis control objective, bank steers the trajectory, and targetHeading
      // is a course reference rather than a direct yaw attitude. Mirror the native
      // controller errors here, matching the in-game AR HUD semantics.
      const pitchError = aerodynamicDirector
        ? (finite(t.commandPitchError) ? n(t.commandPitchError) : n(cmd.targetAoA) - n(t.angleOfAttack))
        : n(cmd.targetPitch) - pitch;
      const rollError = finite(t.commandRollError)
        ? n(t.commandRollError)
        : (finite(cmd.targetRoll) ? signed(n(cmd.targetRoll) - roll) : 0);
      const headingError = aerodynamicDirector ? 0 : signed(n(cmd.targetHeading) - heading);
      const hx = Math.max(-r*.68, Math.min(r*.68, headingError * r / 45));
      const py = Math.max(-r*.68, Math.min(r*.68, -pitchError * ppd));

      navCtx.save();
      navCtx.translate(hx, py);
      navCtx.rotate(rollError * Math.PI / 180);
      navCtx.strokeStyle = c.green;
      navCtx.lineWidth = 2.4;
      navCtx.beginPath();
      navCtx.moveTo(-24, 0); navCtx.lineTo(-8, 0); navCtx.lineTo(-4, 4);
      navCtx.moveTo(4, 4); navCtx.lineTo(8, 0); navCtx.lineTo(24, 0);
      navCtx.moveTo(0, -8); navCtx.lineTo(0, 8);
      navCtx.stroke();
      navCtx.restore();
    }

    if (!orbitMode && finite(cmd.targetRoll)) {
      const a = (n(cmd.targetRoll)-90) * Math.PI / 180;
      const x = Math.cos(a) * (r + 8), y = Math.sin(a) * (r + 8);
      navCtx.fillStyle = c.orange;
      navCtx.beginPath(); navCtx.arc(x, y, 3.2, 0, Math.PI*2); navCtx.fill();
    }
    navCtx.restore();

    navCtx.fillStyle = c.text; navCtx.font = "600 12px ui-monospace, SFMono-Regular, Menlo, monospace"; navCtx.textAlign = "center";
    const headingText = orbitMode && finite(t.orbitalHeading)
      ? `${Math.round(heading).toString().padStart(3,"0")}° · ORB ${Math.round(n(t.orbitalHeading)).toString().padStart(3,"0")}°`
      : `${Math.round(heading).toString().padStart(3,"0")}°`;
    navCtx.fillText(headingText, cx, Math.max(15, cy-r-10));
  }

  function mapPoint(point, width, height) {
    const lon = ((n(point.longitude) + 180) % 360 + 360) % 360 - 180;
    const lat = Math.max(-90, Math.min(90, n(point.latitude)));
    const baseX = (lon + 180) / 360 * width;
    const baseY = (90 - lat) / 180 * height;
    return {
      x: width / 2 + mapView.offsetX + (baseX - width / 2) * mapView.zoom,
      y: height / 2 + mapView.offsetY + (baseY - height / 2) * mapView.zoom,
    };
  }

  function trajectorySegments(points, width, height) {
    if (!Array.isArray(points) || points.length < 2) return [];
    const span = width * mapView.zoom;
    if (!finite(span) || span <= 0) return [];
    const sourceSegments = [];
    let source = [];
    let previousX = null;
    const flush = () => {
      if (source.length > 1) sourceSegments.push(source);
      source = [];
      previousX = null;
    };
    for (const point of points) {
      if (!point || !finite(point.latitude) || !finite(point.longitude)) {
        flush();
        continue;
      }
      const p = mapPoint(point, width, height);
      if (previousX !== null) {
        while (p.x - previousX > span / 2) p.x -= span;
        while (p.x - previousX < -span / 2) p.x += span;
      }
      source.push(p);
      previousX = p.x;
    }
    flush();

    const visible = [];
    const margin = 120;
    for (const segment of sourceSegments) {
      const minX = Math.min(...segment.map((p) => p.x));
      const maxX = Math.max(...segment.map((p) => p.x));
      const firstCopy = Math.floor((-margin - maxX) / span);
      const lastCopy = Math.ceil((width + margin - minX) / span);
      for (let copy = firstCopy; copy <= lastCopy; copy++) {
        const dx = copy * span;
        const shiftedMin = minX + dx, shiftedMax = maxX + dx;
        if (shiftedMax < -margin || shiftedMin > width + margin) continue;
        visible.push(segment.map((p) => ({ x: p.x + dx, y: p.y })));
      }
    }
    return visible;
  }

  function smoothTrajectoryPath(segments) {
    const path = new Path2D();
    if (!segments.length) return path;
    const tangentStrength = .72 / 6;
    for (const segment of segments) {
      path.moveTo(segment[0].x, segment[0].y);
      if (segment.length === 2) {
        path.lineTo(segment[1].x, segment[1].y);
        continue;
      }
      for (let i = 0; i < segment.length - 1; i++) {
        const p0 = segment[Math.max(0, i - 1)];
        const p1 = segment[i];
        const p2 = segment[i + 1];
        const p3 = segment[Math.min(segment.length - 1, i + 2)];
        const c1x = p1.x + (p2.x - p0.x) * tangentStrength;
        const c1y = p1.y + (p2.y - p0.y) * tangentStrength;
        const c2x = p2.x - (p3.x - p1.x) * tangentStrength;
        const c2y = p2.y - (p3.y - p1.y) * tangentStrength;
        path.bezierCurveTo(c1x, c1y, c2x, c2y, p2.x, p2.y);
      }
    }
    return path;
  }

  function drawPath(points, width, height, color, lineWidth, dash = [], glow = 0, alpha = 1) {
    const segments = trajectorySegments(points, width, height);
    if (!segments.length) return segments;
    const path = smoothTrajectoryPath(segments);
    mapCtx.save();
    mapCtx.strokeStyle = color;
    mapCtx.lineJoin = "round";
    mapCtx.lineCap = "round";
    mapCtx.setLineDash(dash);
    if (glow > 0) {
      mapCtx.globalAlpha = alpha * .16;
      mapCtx.lineWidth = lineWidth + glow * 2;
      mapCtx.stroke(path);
      mapCtx.globalAlpha = alpha * .42;
      mapCtx.lineWidth = lineWidth + glow;
      mapCtx.stroke(path);
    }
    mapCtx.globalAlpha = alpha;
    mapCtx.lineWidth = lineWidth;
    mapCtx.stroke(path);
    mapCtx.restore();
    return segments;
  }

  function destinationPoint(origin, bearingDegrees, meters) {
    if (!origin || !finite(origin.latitude) || !finite(origin.longitude)) return null;
    const radius = 600000;
    const angular = meters / radius;
    const bearing = bearingDegrees * Math.PI / 180;
    const lat1 = n(origin.latitude) * Math.PI / 180;
    const lon1 = n(origin.longitude) * Math.PI / 180;
    const sinLat1 = Math.sin(lat1), cosLat1 = Math.cos(lat1);
    const sinAngular = Math.sin(angular), cosAngular = Math.cos(angular);
    const lat2 = Math.asin(sinLat1 * cosAngular + cosLat1 * sinAngular * Math.cos(bearing));
    const lon2 = lon1 + Math.atan2(Math.sin(bearing) * sinAngular * cosLat1, cosAngular - sinLat1 * Math.sin(lat2));
    return { latitude: lat2 * 180 / Math.PI, longitude: lon2 * 180 / Math.PI };
  }

  function drawChevrons(segments, color, spacing = 82, alpha = .75) {
    mapCtx.save();
    mapCtx.strokeStyle = color;
    mapCtx.lineWidth = 1.2;
    mapCtx.globalAlpha = alpha;
    for (const segment of segments) {
      let next = spacing;
      let travelled = 0;
      for (let i = 1; i < segment.length; i++) {
        const a = segment[i - 1], b = segment[i];
        const length = Math.hypot(b.x - a.x, b.y - a.y);
        if (length < .5) continue;
        while (travelled + length >= next) {
          const ratio = (next - travelled) / length;
          const x = a.x + (b.x - a.x) * ratio;
          const y = a.y + (b.y - a.y) * ratio;
          const angle = Math.atan2(b.y - a.y, b.x - a.x);
          mapCtx.save();
          mapCtx.translate(x, y);
          mapCtx.rotate(angle);
          mapCtx.beginPath();
          mapCtx.moveTo(-6, -3.5); mapCtx.lineTo(0, 0); mapCtx.lineTo(-6, 3.5);
          mapCtx.stroke();
          mapCtx.restore();
          next += spacing;
        }
        travelled += length;
      }
    }
    mapCtx.restore();
  }

  function trajectoryDetail(point, currentUt) {
    const detail = [];
    const altitude = finite(point?.altitude) ? point.altitude : point?.meanAltitude;
    const velocity = finite(point?.speed) ? point.speed : point?.trueAirSpeed;
    if (finite(altitude)) detail.push(`${(n(altitude) / 1000).toFixed(1)}KM`);
    if (finite(velocity)) detail.push(`${Math.round(n(velocity))}M/S`);
    if (finite(point?.ut) && finite(currentUt)) {
      const dt = n(point.ut) - n(currentUt);
      if (dt > .5) detail.push(`T+${Math.round(dt)}S`);
    }
    return detail.join(" · ");
  }

  function drawTrajectoryTag(point, width, height, color, label, currentUt, dx = 10, dy = -10) {
    if (!point || !finite(point.latitude) || !finite(point.longitude)) return;
    const p = mapPoint(point, width, height);
    if (p.y < -40 || p.y > height + 40) return;
    const detail = trajectoryDetail(point, currentUt);
    for (const wrappedX of wrappedMapXs(p.x, width, 90)) {
      mapCtx.save();
      mapCtx.font = "700 9px ui-monospace, SFMono-Regular, Menlo, monospace";
      const mainWidth = mapCtx.measureText(label).width;
      mapCtx.font = "600 8px ui-monospace, SFMono-Regular, Menlo, monospace";
      const detailWidth = detail ? mapCtx.measureText(detail).width : 0;
      const boxWidth = Math.max(mainWidth, detailWidth) + 12;
      const boxHeight = detail ? 28 : 17;
      let x = wrappedX + dx, y = p.y + dy - boxHeight;
      if (x + boxWidth > width - 5) x = wrappedX - boxWidth - Math.abs(dx);
      if (y < 5) y = p.y + Math.abs(dy);
      mapCtx.fillStyle = "rgba(7,10,12,.86)";
      mapCtx.strokeStyle = color;
      mapCtx.globalAlpha = .96;
      mapCtx.lineWidth = 1;
      mapCtx.fillRect(x, y, boxWidth, boxHeight);
      mapCtx.strokeRect(x + .5, y + .5, boxWidth - 1, boxHeight - 1);
      mapCtx.fillStyle = color;
      mapCtx.font = "700 9px ui-monospace, SFMono-Regular, Menlo, monospace";
      mapCtx.fillText(label, x + 6, y + 10);
      if (detail) {
        mapCtx.fillStyle = "rgba(238,242,244,.78)";
        mapCtx.font = "600 8px ui-monospace, SFMono-Regular, Menlo, monospace";
        mapCtx.fillText(detail, x + 6, y + 22);
      }
      mapCtx.restore();
    }
  }

  function drawEndpointMarker(point, width, height, color, label, currentUt) {
    if (!point || !finite(point.latitude) || !finite(point.longitude)) return;
    const p = mapPoint(point, width, height);
    for (const x of wrappedMapXs(p.x, width, 16)) {
      mapCtx.save();
      mapCtx.translate(x, p.y);
      mapCtx.rotate(Math.PI / 4);
      mapCtx.strokeStyle = color;
      mapCtx.lineWidth = 1.4;
      mapCtx.strokeRect(-4.5, -4.5, 9, 9);
      mapCtx.restore();
    }
    drawTrajectoryTag(point, width, height, color, label, currentUt, 10, -8);
  }

  function drawRangeRings(site, width, height, color) {
    if (!site || !finite(site.latitude) || !finite(site.longitude)) return;
    const visibleWidth = visibleGroundWidthMeters(site.latitude);
    const baseRadius = Math.max(1000, Math.min(50000, niceMapDistance(visibleWidth / 6)));
    const radii = [baseRadius, baseRadius * 2, baseRadius * 3].filter((radius) => radius <= 150000);
    for (const radius of radii) {
      const ring = [];
      for (let bearing = 0; bearing <= 360; bearing += 10) ring.push(destinationPoint(site, bearing, radius));
      drawPath(ring, width, height, color, .8, [2, 5], 0, .30);
      const labelPoint = destinationPoint(site, 62, radius);
      const p = mapPoint(labelPoint, width, height);
      if (p.y >= 0 && p.y <= height) {
        mapCtx.save();
        mapCtx.fillStyle = color;
        mapCtx.globalAlpha = .52;
        mapCtx.font = "700 8px ui-monospace, SFMono-Regular, Menlo, monospace";
        const text = `${radius >= 1000 ? `${(radius / 1000).toFixed(radius < 10000 ? 1 : 0)}KM` : `${Math.round(radius)}M`}`;
        for (const x of wrappedMapXs(p.x, width, 30)) mapCtx.fillText(text, x + 3, p.y - 3);
        mapCtx.restore();
      }
    }
  }

  function drawRunwayAxis(site, width, height, color) {
    if (!site || !finite(site.latitude) || !finite(site.longitude) || !finite(site.runwayHeading)) return;
    const heading = n(site.runwayHeading);
    const back = destinationPoint(site, heading + 180, 18000);
    const forward = destinationPoint(site, heading, 32000);
    drawPath([back, site, forward], width, height, color, 1, [7, 6], 1, .54);
    const center = mapPoint(site, width, height);
    const angle = (heading - 90) * Math.PI / 180;
    for (const x of wrappedMapXs(center.x, width, 24)) {
      mapCtx.save();
      mapCtx.translate(x, center.y);
      mapCtx.rotate(angle);
      mapCtx.fillStyle = "rgba(7,10,12,.88)";
      mapCtx.strokeStyle = color;
      mapCtx.lineWidth = 1;
      mapCtx.fillRect(-10, -2.5, 20, 5);
      mapCtx.strokeRect(-10, -2.5, 20, 5);
      mapCtx.restore();
    }
  }


  function drawRunwayCorridor(site, width, height, color) {
    if (!site || !finite(site.latitude) || !finite(site.longitude) || !finite(site.runwayHeading)) return;
    const heading = n(site.runwayHeading);
    const offsetPoint = (base, lateralMeters) => destinationPoint(base, heading + (lateralMeters >= 0 ? 90 : -90), Math.abs(lateralMeters));

    // Match the real guidance geometry instead of drawing a generic 36 km
    // airport funnel. STS-N owns an 8 km final line; the MM305 -> FINAL gate
    // requires roughly 200 m cross-track at that station. Narrow the envelope
    // toward the actual runway width as the orbiter descends the 20 deg final.
    const finalDistance = 8000;
    const finalGlideSlope = 20;
    const runwayHalfWidth = Math.max(35, n(site.runwayWidth, 70) / 2);
    const gateHalfWidth = 200;
    const nearHalfWidth = Math.max(runwayHalfWidth + 45, 90);
    const finalGate = destinationPoint(site, heading + 180, finalDistance);
    const nearCenter = destinationPoint(site, heading + 180, 350);
    const corridorPoints = [
      offsetPoint(finalGate, -gateHalfWidth),
      offsetPoint(finalGate, gateHalfWidth),
      offsetPoint(nearCenter, nearHalfWidth),
      offsetPoint(nearCenter, -nearHalfWidth),
    ];
    const polygonCopies = trajectorySegments([...corridorPoints, corridorPoints[0]], width, height);

    mapCtx.save();
    mapCtx.fillStyle = color;
    mapCtx.strokeStyle = color;
    for (const polygon of polygonCopies) {
      if (polygon.length < 4) continue;
      mapCtx.globalAlpha = .045;
      mapCtx.beginPath();
      mapCtx.moveTo(polygon[0].x, polygon[0].y);
      for (let i = 1; i < polygon.length; i++) mapCtx.lineTo(polygon[i].x, polygon[i].y);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.globalAlpha = .42;
      mapCtx.lineWidth = 1.1;
      mapCtx.setLineDash([5, 5]);
      mapCtx.stroke();
    }
    mapCtx.setLineDash([]);

    // Draw the actual final centerline and controller-relevant stations.
    drawPath([finalGate, site], width, height, color, 1.35, [9, 5], 1.0, .74);
    for (const distanceMeters of [8000, 4000, 2000]) {
      const center = destinationPoint(site, heading + 180, distanceMeters);
      const frac = distanceMeters / finalDistance;
      const halfWidth = nearHalfWidth + (gateHalfWidth - nearHalfWidth) * frac;
      const leftPoint = offsetPoint(center, -halfWidth);
      const rightPoint = offsetPoint(center, halfWidth);
      drawPath([leftPoint, rightPoint], width, height, color, distanceMeters === 8000 ? 1.15 : .85, [], 0, distanceMeters === 8000 ? .62 : .34);
      const right = mapPoint(rightPoint, width, height);
      mapCtx.globalAlpha = distanceMeters === 8000 ? .78 : .54;
      mapCtx.font = "700 8px ui-monospace, SFMono-Regular, Menlo, monospace";
      mapCtx.fillStyle = color;
      const label = distanceMeters === 8000 ? `FINAL 8K / ${finalGlideSlope}°` : `${distanceMeters / 1000}K`;
      for (const x of wrappedMapXs(right.x, width, 90)) mapCtx.fillText(label, x + 5, right.y - 4);
    }
    mapCtx.restore();
  }

  function drawTrajectoryLadder(points, width, height, color, currentUt) {
    if (!Array.isArray(points) || points.length < 2 || !finite(currentUt)) return;
    const usable = points.filter((point) => point && finite(point.ut) && finite(point.latitude) && finite(point.longitude));
    if (usable.length < 2) return;
    const compact = width < 600;
    const markStep = compact ? 30 : 10;
    const maxMark = compact ? 120 : 180;
    const span = width * mapView.zoom;
    let nextMark = Math.max(markStep, Math.ceil(Math.max(1, n(usable[0].ut) - n(currentUt)) / markStep) * markStep);
    mapCtx.save();
    mapCtx.strokeStyle = color;
    mapCtx.fillStyle = color;
    mapCtx.font = "700 8px ui-monospace, SFMono-Regular, Menlo, monospace";
    for (let i = 1; i < usable.length && nextMark <= maxMark; i++) {
      const a = usable[i - 1], b = usable[i];
      const da = n(a.ut) - n(currentUt), db = n(b.ut) - n(currentUt);
      if (db < nextMark || db <= da) continue;
      const pa = mapPoint(a, width, height), pb = mapPoint(b, width, height);
      while (pb.x - pa.x > span / 2) pb.x -= span;
      while (pb.x - pa.x < -span / 2) pb.x += span;
      while (nextMark >= da && nextMark <= db && nextMark <= maxMark) {
        const ratio = Math.max(0, Math.min(1, (nextMark - da) / (db - da)));
        const baseX = pa.x + (pb.x - pa.x) * ratio;
        const y = pa.y + (pb.y - pa.y) * ratio;
        const angle = Math.atan2(pb.y - pa.y, pb.x - pa.x) + Math.PI / 2;
        const labeled = compact ? nextMark % 60 === 0 : nextMark % 20 === 0;
        const arm = labeled ? 6 : 4;
        const label = `T+${nextMark}`;
        for (const x of wrappedMapXs(baseX, width, 60)) {
          mapCtx.globalAlpha = labeled ? .62 : .30;
          mapCtx.lineWidth = labeled ? 1.1 : .7;
          mapCtx.beginPath();
          mapCtx.moveTo(x - Math.cos(angle) * arm, y - Math.sin(angle) * arm);
          mapCtx.lineTo(x + Math.cos(angle) * arm, y + Math.sin(angle) * arm);
          mapCtx.stroke();
          if (labeled) {
            mapCtx.globalAlpha = .58;
            mapCtx.fillText(label, x + 6, y - 5);
          }
        }
        nextMark += markStep;
      }
    }
    mapCtx.restore();
  }

  function shortPhaseName(value) {
    const phase = String(value || "").trim().toUpperCase();
    if (!phase) return "";
    if (phase.includes("MM304")) return "MM304";
    if (phase.includes("S-TURN") || phase.includes("STURN")) return "S-TURN";
    if (phase.includes("TAEM")) return phase.includes("PROJ") ? "TAEM PROJ" : "TAEM";
    if (phase.includes("HAC")) return "HAC";
    if (phase.includes("PREFLARE")) return "PREFLARE";
    if (phase.includes("FINAL")) return "FINAL";
    return phase.length > 16 ? phase.slice(0, 16) : phase;
  }

  function drawPhaseGates(points, width, height, color, includeFirst = false) {
    if (!Array.isArray(points) || !points.length) return;
    let previous = "";
    for (let i = 0; i < points.length; i++) {
      const point = points[i];
      if (!point || !finite(point.latitude) || !finite(point.longitude)) continue;
      const phase = shortPhaseName(point.phase);
      if (!phase || phase === previous) continue;
      previous = phase;
      if (i === 0 && !includeFirst) continue;
      const p = mapPoint(point, width, height);
      if (p.y < -20 || p.y > height + 20) continue;
      for (const x of wrappedMapXs(p.x, width, 60)) {
        mapCtx.save();
        mapCtx.translate(x, p.y);
        mapCtx.rotate(Math.PI / 4);
        mapCtx.strokeStyle = color;
        mapCtx.globalAlpha = .88;
        mapCtx.lineWidth = 1.4;
        mapCtx.strokeRect(-5, -5, 10, 10);
        mapCtx.restore();
        mapCtx.save();
        mapCtx.fillStyle = color;
        mapCtx.globalAlpha = .86;
        mapCtx.font = "800 8px ui-monospace, SFMono-Regular, Menlo, monospace";
        mapCtx.fillText(phase, x + 9, p.y + 3);
        mapCtx.restore();
      }
    }
  }

  function drawTerminalCue(points, width, height, color, currentUt) {
    if (!Array.isArray(points) || points.length < 3) return;
    const g = snapshot?.guidanceState || {};
    const radius = finite(g.hacRadius) && n(g.hacRadius) > 0 ? n(g.hacRadius) : n(g.terminalCandidateRadius);
    const state = g.terminalPathCaptured || g.hacCaptured ? "HAC CAPTURE" : g.terminalPathCommitted ? "HAC COMMIT" : g.terminalPathSelected ? "HAC SELECT" : "HAC CAND";
    const point = points[Math.max(1, Math.min(points.length - 2, Math.floor(points.length * .52)))];
    const label = `${state}${radius > 0 ? ` · R${(radius / 1000).toFixed(1)}K` : ""}`;
    drawTrajectoryTag(point, width, height, color, label, currentUt, 12, 18);
  }

  function drawMapChrome(width, height, colors) {
    const pad = 9, arm = 14;
    const t = snapshot?.telemetry || {};
    const g = snapshot?.guidanceState || {};
    const predicted = futurePredictionTrajectory(snapshot);
    const planned = displayablePlannedTrajectory(snapshot);
    const orbital = snapshot?.orbitalTrajectory || [];
    const orbitMeta = snapshot?.orbitalTrajectoryMeta || {};
    const orbitCount = trajectoryCount(orbital);
    const energy = entryPlanDisplayable(snapshot) && finite(g.entryPlanTAEMEnergyError) ? g.entryPlanTAEMEnergyError : t.predictedTAEMEnergyError;
    const range = entryPlanDisplayable(snapshot) && finite(g.entryPlanTAEMRangeError) ? g.entryPlanTAEMRangeError : t.predictedTAEMRangeError;
    const uncertainty = finite(t.predictedPhysicsRelativeUncertainty) ? Math.max(0, n(t.predictedPhysicsRelativeUncertainty)) : null;
    const viewWidth = visibleGroundWidthMeters(t.latitude);
    mapCtx.save();
    mapCtx.strokeStyle = colors.cyan;
    mapCtx.globalAlpha = .38;
    mapCtx.lineWidth = 1;
    mapCtx.beginPath();
    mapCtx.moveTo(pad, pad + arm); mapCtx.lineTo(pad, pad); mapCtx.lineTo(pad + arm, pad);
    mapCtx.moveTo(width - pad - arm, pad); mapCtx.lineTo(width - pad, pad); mapCtx.lineTo(width - pad, pad + arm);
    mapCtx.moveTo(pad, height - pad - arm); mapCtx.lineTo(pad, height - pad); mapCtx.lineTo(pad + arm, height - pad);
    mapCtx.moveTo(width - pad - arm, height - pad); mapCtx.lineTo(width - pad, height - pad); mapCtx.lineTo(width - pad, height - pad - arm);
    mapCtx.stroke();
    mapCtx.globalAlpha = .72;
    mapCtx.fillStyle = colors.text;
    mapCtx.font = "700 8px ui-monospace, SFMono-Regular, Menlo, monospace";
    const revision = finite(snapshot?.trajectoryRevision) ? Math.round(n(snapshot.trajectoryRevision)) : 0;
    mapCtx.fillText(`TRAJ / REV ${String(revision).padStart(4, "0")} · ${String(snapshot?.phase || "FLIGHT").toUpperCase()}`, 17, 22);
    mapCtx.textAlign = "right";
    const zoomText = mapView.zoom >= 20 ? mapView.zoom.toFixed(0) : mapView.zoom.toFixed(1);
    const viewText = width >= 480 ? ` · VIEW ${distance(viewWidth).toUpperCase()}` : "";
    mapCtx.fillText(`Z ${zoomText}X${viewText} · ${mapView.autoFocus ? "AUTO" : "MANUAL"}`, width - 17, 22);
    if (width > 440) {
      mapCtx.textAlign = "left";
      const orbitMode = isOrbitMode(snapshot);
      if (orbitMode && orbitCount >= 2) {
        const plan = snapshot?.deorbitPlan;
        const burnDuration = plan && finite(plan.estimatedBurnDuration) ? Math.max(0,n(plan.estimatedBurnDuration)) : 0;
        const burnStart = plan && finite(plan.burnUT) ? n(plan.burnUT) - burnDuration * .5 : null;
        const toBurn = finite(burnStart) && finite(t.ut) ? n(burnStart)-n(t.ut) : null;
        const orbitLabel = orbitMeta.underThrust ? "OSC ORBIT · POWERED" : "OSC ORBIT";
        mapCtx.fillText(`${orbitLabel} · AP ${distance(t.apoapsisAltitude).toUpperCase()} · PE ${distance(t.periapsisAltitude).toUpperCase()}`, 17, height - 15);
        mapCtx.textAlign = "right";
        mapCtx.fillText(plan
          ? `BURN ${countdown(toBurn)} · ΔV ${finite(plan.deltaV) ? n(plan.deltaV).toFixed(1) : "—"} M/S · ${warp(t)}`
          : `T AP ${countdown(t.timeToApoapsis)} · T PE ${countdown(t.timeToPeriapsis)} · ${warp(t)}`, width - 17, height - 15);
      } else {
        const energyText = finite(energy) ? specificEnergy(energy).toUpperCase() : "—";
        const rangeText = finite(range) ? signedDistance(range).toUpperCase() : "—";
        mapCtx.fillText(`TAEM ΔE ${energyText} · ΔR ${rangeText}`, 17, height - 15);
        mapCtx.textAlign = "right";
        const physicsText = finite(uncertainty) ? ` · MODEL ±${Math.round(uncertainty * 100)}%` : "";
        const predFreshness = predictionFreshness(snapshot);
        const predStatus = predFreshness.stale ? ` · STALE ${Math.round(predFreshness.age)}S` : "";
        mapCtx.fillText(`PRED ${trajectoryCount(predicted)}${predStatus} · PLAN ${trajectoryCount(planned)}${physicsText}`, width - 17, height - 15);
      }
    }
    mapCtx.restore();
  }

  function drawAdaptiveGrid(width, height) {
    const visibleLongitude = 360 / Math.max(1, mapView.zoom);
    const step = visibleLongitude <= 1.5 ? .1
      : visibleLongitude <= 3 ? .25
      : visibleLongitude <= 6 ? .5
      : visibleLongitude <= 12 ? 1
      : visibleLongitude <= 24 ? 2
      : visibleLongitude <= 60 ? 5
      : visibleLongitude <= 120 ? 10
      : visibleLongitude <= 240 ? 30 : 60;

    const leftWorldX = width / 2 + (0 - width / 2 - mapView.offsetX) / mapView.zoom;
    const rightWorldX = width / 2 + (width - width / 2 - mapView.offsetX) / mapView.zoom;
    const topWorldY = height / 2 + (0 - height / 2 - mapView.offsetY) / mapView.zoom;
    const bottomWorldY = height / 2 + (height - height / 2 - mapView.offsetY) / mapView.zoom;
    const minLongitude = leftWorldX / width * 360 - 180;
    const maxLongitude = rightWorldX / width * 360 - 180;
    const maxLatitude = Math.min(90, 90 - topWorldY / height * 180);
    const minLatitude = Math.max(-90, 90 - bottomWorldY / height * 180);

    mapCtx.save();
    mapCtx.strokeStyle = "rgba(102,216,239,.12)";
    mapCtx.lineWidth = .8 / mapView.zoom;
    const drawGridLine = (value, vertical) => {
      const major = Math.abs(Math.round(value / step)) % 5 === 0;
      mapCtx.globalAlpha = major ? .95 : .48;
      mapCtx.beginPath();
      if (vertical) {
        const x = (value + 180) / 360 * width;
        mapCtx.moveTo(x, 0); mapCtx.lineTo(x, height);
      } else {
        const y = (90 - value) / 180 * height;
        mapCtx.moveTo(leftWorldX, y); mapCtx.lineTo(rightWorldX, y);
      }
      mapCtx.stroke();
    };
    for (let lon = Math.ceil(minLongitude / step) * step; lon <= maxLongitude + step * .1; lon += step) drawGridLine(lon, true);
    for (let lat = Math.ceil(minLatitude / step) * step; lat <= maxLatitude + step * .1; lat += step) drawGridLine(lat, false);
    mapCtx.restore();
  }

  function drawKerbinGeographic(width, height) {
    // Stock KerbinScaledSpace300 uses u_stock = 0.75 - u_geo. Reproject the
    // raw stock pixels into the same west-to-east longitude basis as telemetry.
    const sourceWidth = kerbin.naturalWidth;
    const sourceHeight = kerbin.naturalHeight;
    const split = 0.75;
    mapCtx.save();
    mapCtx.translate(width * split, 0);
    mapCtx.scale(-1, 1);
    mapCtx.drawImage(kerbin, 0, 0, sourceWidth * split, sourceHeight, 0, 0, width * split, height);
    mapCtx.restore();
    mapCtx.save();
    mapCtx.translate(width, 0);
    mapCtx.scale(-1, 1);
    mapCtx.drawImage(kerbin, sourceWidth * split, 0, sourceWidth * (1 - split), sourceHeight, 0, 0, width * (1 - split), height);
    mapCtx.restore();
  }


  function drawWrappedKerbin(width, height) {
    if (!kerbin.complete || kerbin.naturalWidth <= 0) return;
    const leftWorldX = width / 2 + (0 - width / 2 - mapView.offsetX) / mapView.zoom;
    const rightWorldX = width / 2 + (width - width / 2 - mapView.offsetX) / mapView.zoom;
    const firstTile = Math.floor(leftWorldX / width) - 1;
    const lastTile = Math.floor(rightWorldX / width) + 1;
    for (let tile = firstTile; tile <= lastTile; tile++) {
      mapCtx.save();
      mapCtx.translate(tile * width, 0);
      drawKerbinGeographic(width, height);
      mapCtx.restore();
    }
  }

  function wrappedMapXs(x, width, margin = 20) {
    const span = width * mapView.zoom;
    if (!finite(span) || span <= 0) return [x];
    const firstCopy = Math.floor((-margin - x) / span);
    const lastCopy = Math.ceil((width + margin - x) / span);
    const values = [];
    for (let copy = firstCopy; copy <= lastCopy; copy++) {
      const wrappedX = x + copy * span;
      if (wrappedX >= -margin && wrappedX <= width + margin) values.push(wrappedX);
    }
    return values;
  }


  function drawShuttleMapIndicator(t, width, height, colors) {
    if (!finite(t?.latitude) || !finite(t?.longitude)) return;
    const p = mapPoint(t, width, height);
    if (p.y < -40 || p.y > height + 40) return;

    // Keep exactly one marker on an infinitely wrapped longitude map.
    const span = width * mapView.zoom;
    const x = finite(span) && span > 0
      ? p.x + Math.round((width / 2 - p.x) / span) * span
      : p.x;
    if (x < -40 || x > width + 40) return;

    const heading = finite(t.heading) ? n(t.heading) : n(t.groundTrackHeading);
    const angle = heading * Math.PI / 180;
    // Scale with map zoom, but ease the shrink so the orbiter stays legible.
    // Square-root scaling avoids the previous collapse to a tiny dot when zooming out.
    const zoomRatio = Math.max(0, Math.min(1, mapView.zoom / mapView.maxZoom));
    const markerPixels = Math.max(20, Math.min(54, 54 * Math.sqrt(zoomRatio)));
    const scale = markerPixels / 256;

    mapCtx.save();
    mapCtx.translate(x, p.y);
    mapCtx.rotate(angle);
    mapCtx.scale(scale, scale);
    mapCtx.fillStyle = "#ffffff";
    mapCtx.beginPath();
    mapCtx.moveTo(SHUTTLE_MARKER_OUTLINE[0][0], SHUTTLE_MARKER_OUTLINE[0][1]);
    for (let i = 1; i < SHUTTLE_MARKER_OUTLINE.length; i++) {
      mapCtx.lineTo(SHUTTLE_MARKER_OUTLINE[i][0], SHUTTLE_MARKER_OUTLINE[i][1]);
    }
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.restore();
  }

  function drawMap() {
    const { width, height } = resizeCanvas(map, mapCtx);
    if (!focusMapOnFlight(width, height)) clampMapView(width, height);
    const c = C();
    mapCtx.clearRect(0, 0, width, height);
    mapCtx.fillStyle = "#0d1012"; mapCtx.fillRect(0, 0, width, height);

    mapCtx.save();
    mapCtx.translate(width / 2 + mapView.offsetX, height / 2 + mapView.offsetY);
    mapCtx.scale(mapView.zoom, mapView.zoom);
    mapCtx.translate(-width / 2, -height / 2);
    if (kerbin.complete && kerbin.naturalWidth > 0) {
      mapCtx.globalAlpha = .72;
      drawWrappedKerbin(width, height);
      mapCtx.globalAlpha = 1;
    }
    drawAdaptiveGrid(width, height);
    mapCtx.restore();

    const actual = snapshot?.actualTrajectory || [];
    const orbital = snapshot?.orbitalTrajectory || [];
    const orbitMeta = snapshot?.orbitalTrajectoryMeta || {};
    const predicted = futurePredictionTrajectory(snapshot);
    const projectedTaem = displayableProjectedTAEMTrajectory(snapshot);
    const planned = displayablePlannedTrajectory(snapshot);
    const site = snapshot?.site || {};
    const t = snapshot?.telemetry || {};
    const orbitMode = isOrbitMode(snapshot);
    const burn = burnWindow(snapshot);
    const apoapsis = orbitPointOfInterest(snapshot, "apoapsis");
    const periapsis = orbitPointOfInterest(snapshot, "periapsis");

    if (!orbitMode) {
      drawRangeRings(site, width, height, c.orange);
      drawRunwayCorridor(site, width, height, c.orange);
      drawRunwayAxis(site, width, height, c.orange);
      if (finite(t.latitude) && finite(t.longitude) && finite(site.latitude) && finite(site.longitude)) {
        drawPath([t, site], width, height, c.cyan, .8, [3, 6], 0, .24);
      }
    }

    const predFreshness = predictionFreshness(snapshot);
    const predAlpha = predFreshness.stale ? .38 : 1;
    const predDash = predFreshness.stale ? [7, 6] : [];
    const orbitalSegments = drawPath(orbital, width, height, c.cyan, 1.45, [8,6], 1.2, .58);
    const plannedSegments = drawPath(planned, width, height, c.orange, 1.8, [5,5], 1.5, .94);
    const predictedSegments = drawPath(predicted, width, height, c.green, predFreshness.stale ? 1.45 : 2.2, predDash, predFreshness.stale ? .8 : 2.4, predAlpha);
    const projectedSegments = drawPath(projectedTaem, width, height, c.green, 1.6, [7,5], 1.4, .72);
    const burnSegments = orbitMode && burn.points.length >= 2 ? drawPath(burn.points, width, height, c.orange, 3.0, [], 3.2, 1) : [];
    drawPath(actual, width, height, c.cyan, 2.0, [], 1.8, .90);

    drawChevrons(orbitalSegments, c.cyan, 120, .42);
    if (burnSegments.length) drawChevrons(burnSegments, c.orange, 46, .92);
    drawChevrons(plannedSegments, c.orange, 96, .50);
    drawChevrons(predictedSegments, c.green, 76, predFreshness.stale ? .24 : .78);
    drawChevrons(projectedSegments, c.green, 88, .45);


    drawTrajectoryLadder(predicted, width, height, c.green, t.ut);
    if (!orbitMode) {
      if (entryPlanDisplayable(snapshot)) drawPhaseGates(predicted, width, height, c.green, false);
      drawPhaseGates(projectedTaem, width, height, c.green, true);
      drawTerminalCue(projectedTaem, width, height, c.green, t.ut);
    }

    if (predicted.length >= 5 && width >= 600) {
      drawTrajectoryTag(
        predicted[Math.floor((predicted.length - 1) * .55)],
        width, height, c.green,
        predFreshness.stale ? "STALE FORECAST" : "PRED",
        t.ut, 9, -7
      );
    }
    if (orbital.length >= 2) drawEndpointMarker(orbital[orbital.length - 1], width, height, c.cyan, orbitMeta.endReason === "atmosphere-interface" ? "ENTRY INTERFACE" : "1 ORBIT", t.ut);
    if (orbitMode && apoapsis) drawEndpointMarker(apoapsis, width, height, c.cyan, "AP", t.ut);
    if (orbitMode && periapsis) drawEndpointMarker(periapsis, width, height, c.orange, "PE", t.ut);
    if (orbitMode && burn.start) drawTrajectoryTag(burn.start, width, height, c.orange, burn.startIsLive ? "BURN NOW" : "DEORBIT START", t.ut, 10, -12);
    if (orbitMode && burn.mid) drawEndpointMarker(burn.mid, width, height, c.orange, "BURN CENTER", t.ut);
    if (orbitMode && burn.end) drawTrajectoryTag(burn.end, width, height, c.orange, "CUTOFF", t.ut, 10, 18);
    if (planned.length >= 2) drawEndpointMarker(planned[planned.length - 1], width, height, c.orange, "PLAN END", t.ut);
    if (predicted.length >= 2) drawEndpointMarker(
      predicted[predicted.length - 1], width, height, c.green,
      predFreshness.stale ? "FORECAST END" : "PRED END", t.ut
    );
    if (projectedTaem.length >= 2) drawEndpointMarker(projectedTaem[projectedTaem.length - 1], width, height, c.green, "TAEM / FINAL", t.ut);

    if (finite(site.latitude) && finite(site.longitude)) {
      const p = mapPoint(site, width, height);
      mapCtx.strokeStyle = c.orange; mapCtx.lineWidth = orbitMode ? 1.0 : 1.5;
      mapCtx.globalAlpha = orbitMode ? .48 : 1;
      mapCtx.fillStyle = c.text; mapCtx.font = "600 9px ui-monospace, SFMono-Regular, Menlo, monospace";
      for (const x of wrappedMapXs(p.x, width)) {
        mapCtx.beginPath(); mapCtx.arc(x, p.y, orbitMode ? 4 : 8, 0, Math.PI * 2); mapCtx.stroke();
        if (!orbitMode) { mapCtx.beginPath(); mapCtx.moveTo(x-11,p.y); mapCtx.lineTo(x+11,p.y); mapCtx.moveTo(x,p.y-11); mapCtx.lineTo(x,p.y+11); mapCtx.stroke(); }
        mapCtx.fillText(orbitMode ? "KSC" : "KSC / RWY", x+(orbitMode?7:13), p.y-8);
      }
      mapCtx.globalAlpha = 1;
    }
    drawShuttleMapIndicator(t, width, height, c);
    drawMapChrome(width, height, c);
  }

  function greatCircleMeters(a, b) {
    if (![a?.latitude,a?.longitude,b?.latitude,b?.longitude].every(finite)) return null;
    const R = 600000;
    const lat1 = n(a.latitude)*Math.PI/180, lat2=n(b.latitude)*Math.PI/180;
    const dlat=lat2-lat1, dlon=(n(b.longitude)-n(a.longitude))*Math.PI/180;
    const h=Math.sin(dlat/2)**2+Math.cos(lat1)*Math.cos(lat2)*Math.sin(dlon/2)**2;
    return R*2*Math.atan2(Math.sqrt(h),Math.sqrt(Math.max(0,1-h)));
  }

  function setBar(id, markerId, target, current, min, max, centered=false) {
    const span = $(id), marker = $(markerId);
    const clamp = (v) => Math.max(0, Math.min(1, (v-min)/(max-min)));
    if (finite(current)) marker.style.left = `${clamp(n(current))*100}%`;
    else marker.style.left = "50%";
    if (!finite(target)) {
      span.style.left = centered ? "50%" : "0";
      span.style.width = "0";
      return;
    }
    const t = clamp(n(target));
    if (centered) {
      const mid = clamp(0);
      span.style.left = `${Math.min(mid,t)*100}%`;
      span.style.width = `${Math.abs(t-mid)*100}%`;
    } else {
      span.style.left = "0"; span.style.width = `${t*100}%`;
    }
  }

  function updateRLMonitor() {
    const rl = snapshot?.rlTraining && typeof snapshot.rlTraining === "object" ? snapshot.rlTraining : {};
    const queue = rl.queue && typeof rl.queue === "object" ? rl.queue : {};
    const queueAvailable = queue.available === true;
    const available = rl.available === true || queueAvailable;
    const state = String(rl.state || queue.state || "idle").toLowerCase();
    const condition = rl.latestCondition && typeof rl.latestCondition === "object" ? rl.latestCondition : {};
    const screen = condition.mm304_taem_screen && typeof condition.mm304_taem_screen === "object"
      ? condition.mm304_taem_screen : {};
    const prettyStatus = String(rl.status || (available ? "LAST RUN" : "NO RUN"))
      .replaceAll("_", " ").toUpperCase();
    const queueLeft = finite(queue.remainingSeconds) ? duration(queue.remainingSeconds) : "—";
    const queuePrefix = queueAvailable ? `6H ${String(queue.state || state).toUpperCase()} · LEFT ${queueLeft}` : null;
    const statusText = available
      ? `${queuePrefix ? queuePrefix + " · " : ""}${prettyStatus} · ${n(rl.episodes, 0).toFixed(0)} EP · ${n(rl.activeSteps, 0).toFixed(0)} ACTIVE`
      : "NO OFFLINE TRAINING RUN DETECTED";
    $("rl-status").textContent = statusText;
    $("rl-run").textContent = available ? String(queue.currentRunId || queue.sessionName || rl.runId || "—") : "—";
    $("rl-state").textContent = available ? (queueAvailable ? `Q ${String(queue.state || state).toUpperCase()}` : state.toUpperCase()) : "—";
    $("rl-policy").textContent = available ? String(rl.curriculum || rl.latestPolicy || "—").slice(0, 12) : "—";
    $("rl-seed").textContent = finite(rl.latestSeed) ? String(Math.trunc(n(rl.latestSeed))) : "—";
    const curriculumCase = condition.stage || rl.curriculum || "—";
    const hasLocalCase = finite(condition.altitude_m) || finite(condition.runway_along_m) || finite(condition.speed_mps);
    if (hasLocalCase) {
      $("rl-ap").textContent = String(curriculumCase).toUpperCase().replaceAll("-", " ");
      $("rl-pe").textContent = distance(condition.altitude_m);
      $("rl-inc").textContent = speed(condition.speed_mps);
      $("rl-post-pe").textContent = fmtSigned(condition.flight_path_angle_deg, 1, "°");
      $("rl-fpa").textContent = signedDistance(condition.runway_along_m);
      $("rl-entry-speed").textContent = signedDistance(condition.runway_cross_m);
    } else {
      $("rl-ap").textContent = distance(condition.apoapsis_altitude_m);
      $("rl-pe").textContent = distance(condition.periapsis_altitude_m);
      $("rl-inc").textContent = fmt(condition.inclination_deg, 1, "°");
      $("rl-post-pe").textContent = distance(condition.target_post_deorbit_periapsis_altitude_m);
      $("rl-fpa").textContent = fmtSigned(screen.entry_fpa_deg, 2, "°");
      $("rl-entry-speed").textContent = speed(screen.entry_speed_mps);
    }
    $("rl-upstream").textContent = queueAvailable ? queueLeft : (finite(screen.entry_along_m) ? distance(-n(screen.entry_along_m)) : "—");
    $("rl-crossrange").textContent = queueAvailable && finite(queue.runsCompleted)
      ? `${Math.trunc(n(queue.runsCompleted))}/${Math.trunc(n(queue.runsDiscovered, 0))}`
      : signedDistance(screen.entry_cross_m);
    $("rl-episodes").textContent = finite(rl.episodes) ? String(Math.trunc(n(rl.episodes))) : "—";
    $("rl-eligible").textContent = finite(rl.eligibleSteps) ? String(Math.trunc(n(rl.eligibleSteps))) : "—";
    $("rl-active").textContent = finite(rl.activeSteps) ? String(Math.trunc(n(rl.activeSteps))) : "—";
    $("rl-fallback").textContent = finite(rl.fallbacks) ? String(Math.trunc(n(rl.fallbacks))) : "—";
    const handoffs = finite(rl.handoffs) ? Math.trunc(n(rl.handoffs)) : null;
    const touchdowns = finite(rl.touchdowns) ? Math.trunc(n(rl.touchdowns)) : null;
    $("rl-touchdowns").textContent = handoffs !== null ? `${handoffs}/${touchdowns ?? 0}` : touchdowns !== null ? String(touchdowns) : "—";
    $("rl-success").textContent = finite(rl.successes) ? String(Math.trunc(n(rl.successes))) : "—";
    $("rl-valid").textContent = finite(rl.validRollouts) ? String(Math.trunc(n(rl.validRollouts))) : "—";
    $("rl-return").textContent = finite(rl.meanReturn) ? n(rl.meanReturn).toFixed(1) : "—";
    const age = finite(rl.fileAgeSeconds) ? ` · artifact ${n(rl.fileAgeSeconds).toFixed(0)}s old` : "";
    const queuePath = queueAvailable ? ` · queue ${queue.session || queue.sessionName || "active"}` : "";
    const currentLog = queue.currentRunLog || queue.masterLog || null;
    const blockers = Array.isArray(rl.blockers) && rl.blockers.length ? ` · blockers ${rl.blockers.join(",")}` : "";
    const unsafe = rl.unsafe_reasons && typeof rl.unsafe_reasons === "object"
      ? Object.entries(rl.unsafe_reasons).map(([k,v]) => `${k}:${v}`).join(",") : "";
    $("rl-note").textContent = available
      ? `Offline simulator artifact: ${rl.output || rl.runId || queue.currentRun || "pending"}${queuePath}${age}${blockers}${unsafe ? " · unsafe " + unsafe : ""} · ${currentLog ? "log " + currentLog + " · " : ""}read-only; cannot command this vessel.`
      : "Reads ShuttleSim/rl/results and Runtime/SlowRuns queues when a training run exists; this monitor cannot command the live vessel.";
    document.body.classList.remove("rl-running", "rl-finished", "rl-stale");
    if (available && ["running", "finished", "stale"].includes(state)) document.body.classList.add(`rl-${state}`);
    setTone("rl-status", !available ? "" : state === "running" ? "good" : state === "finished" ? "warn" : "bad");
    setTone("rl-state", state === "running" ? "good" : state === "finished" ? "warn" : state === "stale" ? "bad" : "");
    setTitle("rl-run", available ? `${rl.output || rl.runId || queue.currentRun || queue.session || "—"}${age}` : "No RL output selected or discovered");
    setTitle("rl-policy", available ? `Curriculum ${rl.curriculum || "unknown"}; latest policy ${rl.latestPolicy || "—"}` : "No policy observed");
    setTitle("rl-active", available ? `${rl.activeSteps || 0} action-dependent residual steps` : "No training data");
    setTitle("rl-touchdowns", available ? `${rl.handoffs || 0} curriculum handoffs / ${rl.touchdowns || 0} touchdowns` : "No handoff data");
    setTitle("rl-note", available ? "RL is simulator-only. The live telemetry server only reads JSONL/report artifacts and queue pid/log metadata." : "No simulator-only RL output is currently available.");
  }

  function updateText() {
    if (!snapshot) return;
    const t = snapshot.telemetry || {}, cmd = snapshot.command || {}, g = snapshot.guidanceState || {};
    const state = String(snapshot.connectionStatus || "offline").toUpperCase();
    const phase = String(snapshot.phase || "OBSERVE").toUpperCase();
    const terminalPhase = isTerminalPhase(phase);
    const orbitMode = isOrbitMode(snapshot);
    const plan = snapshot.deorbitPlan && typeof snapshot.deorbitPlan === "object" ? snapshot.deorbitPlan : null;
    const predictedCount = trajectoryCount(futurePredictionTrajectory(snapshot));
    const plannedCount = trajectoryCount(displayablePlannedTrajectory(snapshot));
    const referenceCount = trajectoryCount(displayableReferenceTrajectory(snapshot));
    const projectedCount = trajectoryCount(displayableProjectedTAEMTrajectory(snapshot));
    const actualCount = trajectoryCount(snapshot.actualTrajectory);
    const orbitalCount = trajectoryCount(snapshot.orbitalTrajectory);
    const orbitMeta = snapshot.orbitalTrajectoryMeta || {};
    const source = String(snapshot.server?.source || "unknown");
    const tick = finite(snapshot.tickSequence) ? Math.trunc(n(snapshot.tickSequence)) : null;
    const entryPlanProven = g.entryPlanValid === true && g.entryPlanTerminalReady === true;
    const terminalPlanProven = g.terminalPathCommitted === true || g.terminalPathCaptured === true || g.terminalPathComplete === true;

    document.body.classList.toggle("orbit-mode", orbitMode);
    $("connection").textContent = state;
    $("connection-dot").className = `dot ${state === "CONNECTED" ? "live" : state === "RECONNECTING" ? "bad" : ""}`;
    $("phase").textContent = orbitMode && phase === "IDLE" ? "ORBIT" : phase;
    const vesselName = t.vesselName || "—";
    $("vessel").textContent = t.vesselName ? `${vesselName} · ${snapshot.automationEngaged ? "AUTO" : "MAN"}` : vesselName;
    setTitle("connection", `${source}${tick !== null ? ` · tick ${tick}` : ""}`);
    setTitle("vessel", `${vesselName}${finite(t.ut) ? ` · UT ${n(t.ut).toFixed(1)} s` : ""}${tick !== null ? ` · controller tick ${tick}` : ""}`);

    const sim = snapshot.simulation && typeof snapshot.simulation === "object" ? snapshot.simulation : null;
    const simStatus = $("simulation-status");
    // Keep completed simulator/replay frames identifiable while they are inspected.
    if (simStatus) {
      const active = !!(sim && (sim.active || sim.sourceMode === "simulator" || sim.sourceMode === "replay"));
      simStatus.hidden = !active;
      document.body.classList.toggle("simulation-mode", active);
      document.body.classList.toggle("replay-mode", active && String(sim && sim.sourceMode || "").toLowerCase() === "replay");
      if (active) {
        const simState = String(sim.state || "idle").toUpperCase();
        const simMode = String(sim.mode || sim.sourceMode || "simulator").toUpperCase();
        const runId = sim.runId || "UNNAMED RUN";
        const elapsed = finite(sim.simElapsedSeconds) ? "T+ " + n(sim.simElapsedSeconds).toFixed(1) + " s" :
          (finite(t.ut) ? "UT " + n(t.ut).toFixed(1) + " s" : "T+ —");
        const effective = finite(sim.effectiveRate) ?
          n(sim.effectiveRate).toFixed(n(sim.effectiveRate) >= 100 ? 0 : 1) + "×" :
          (sim.rateMode ? String(sim.rateMode).toUpperCase() : "MAX");
        $("sim-state").textContent = "SIM " + simState;
        $("sim-state").className = "sim-state " +
          (simState.includes("FINISH") ? "finished" :
           (simState.includes("ERROR") || simState.includes("ABORT") ? "bad" :
            (simState.includes("PAUSE") ? "paused" : "running")));
        $("sim-run").textContent = runId;
        $("sim-mode").textContent = simMode;
        $("sim-time").textContent = elapsed;
        $("sim-rate").textContent = effective;
        $("sim-lockstep").textContent = sim.lockstep ? "LOCKSTEP" : "FREE-RUN";
        const replay = $("sim-replay");
        if (replay) {
          const replaying = String(sim.sourceMode || "").toLowerCase() === "replay";
          replay.hidden = !replaying;
          if (replaying) {
            const ri = finite(sim.replayIndex) ? Math.trunc(n(sim.replayIndex)) + 1 : null;
            const rc = finite(sim.replayCount) ? Math.trunc(n(sim.replayCount)) : null;
            const rs = finite(sim.replaySpeed) ? n(sim.replaySpeed).toFixed(1) + "×" : "";
            replay.textContent = ri !== null && rc ? "REPLAY " + ri + "/" + rc + (rs ? " · " + rs : "") : "REPLAY" + (rs ? " · " + rs : "");
          }
        }
        setTitle("sim-state", "source " + (sim.sourceMode || "simulator") + " · phase " + phase + " · state " + simState);
        setTitle("sim-run", (sim.scenario || "scenario unavailable") + (sim.runDirectory ? " · " + sim.runDirectory : ""));
        setTitle("sim-rate", "effective " + effective +
          (finite(sim.wallSeconds) ? " · wall " + n(sim.wallSeconds).toFixed(2) + " s" : "") +
          (finite(sim.physicsDt) ? " · dt " + n(sim.physicsDt).toFixed(3) + " s" : ""));
      }
    }
    updateRLMonitor();

    const retroErr = retrogradeError(t);
    $("pitch-label").textContent = orbitMode ? "ATT PITCH" : "PITCH";
    $("roll-label").textContent = orbitMode ? "ATT ROLL" : "ROLL";
    $("heading-label").textContent = orbitMode ? "ATT HDG" : "HDG";
    $("aoa-label").textContent = orbitMode ? "RETRO ERR" : "AOA";
    $("pitch").textContent = fmtSigned(t.pitch,1,"°");
    $("roll").textContent = fmtSigned(t.roll,1,"°");
    $("heading").textContent = fmt(t.heading,1,"°");
    $("aoa").textContent = orbitMode ? fmt(retroErr,1,"°") : fmt(t.angleOfAttack,1,"°");

    // Entry/atmospheric metrics remain live but are hidden while the page is in orbital mode.
    $("altitude").textContent = distance(t.meanAltitude);
    $("radar-altitude").textContent = distance(t.radarAltitude);
    $("tas").textContent = speed(t.trueAirSpeed);
    $("vertical-speed").textContent = speed(t.verticalSpeed);
    $("mach").textContent = fmt(t.mach,2,"");
    $("dynamic-pressure").textContent = pressure(t.dynamicPressure);
    $("g-force").textContent = fmt(t.gForce,2," g");
    $("fpa").textContent = fmtSigned(t.flightPathAngle,1,"°");
    setTitle("altitude", `MSL ${distance(t.meanAltitude)} · radar ${distance(t.radarAltitude)} · V/S ${speed(t.verticalSpeed)}`);
    setTitle("radar-altitude", `Terrain clearance ${distance(t.radarAltitude)} · MSL ${distance(t.meanAltitude)}`);
    setTitle("tas", `TAS ${speed(t.trueAirSpeed)} · surface ${speed(t.surfaceSpeed)} · horizontal ${speed(t.horizontalSpeed)}${finite(t.predictedTAEMSpeed) ? ` · predicted TAEM ${speed(t.predictedTAEMSpeed)}` : ""}`);
    setTitle("vertical-speed", `Vertical ${speed(t.verticalSpeed)} · FPA ${fmtSigned(t.flightPathAngle,1,"°")}`);
    setTitle("mach", `Mach ${fmt(t.mach,2)} · speed of sound ${speed(t.speedOfSound)}${finite(t.atmosphericDensity) ? ` · density ${n(t.atmosphericDensity).toFixed(4)} kg/m³` : ""}`);
    setTitle("dynamic-pressure", `Current ${pressure(t.dynamicPressure)} · static ${pressure(t.staticPressure)}${finite(t.predictedPeakDynamicPressure) ? ` · predicted peak ${pressure(t.predictedPeakDynamicPressure)}` : ""}`);
    setTitle("g-force", `Current ${fmt(t.gForce,2," g")}${finite(t.predictedPeakGLoad) ? ` · predicted peak ${fmt(t.predictedPeakGLoad,2," g")}` : ""}`);
    setTitle("fpa", `Current ${fmtSigned(t.flightPathAngle,1,"°")}${finite(t.predictedEntryFlightPathAngle) ? ` · predicted entry ${fmtSigned(t.predictedEntryFlightPathAngle,1,"°")}` : ""}${finite(g.terminalReferenceFPA) ? ` · terminal ref ${fmtSigned(g.terminalReferenceFPA,1,"°")}` : ""}`);

    // Orbital metrics deliberately replace the atmospheric panel in orbit.
    $("orbit-speed").textContent = speed(t.orbitalSpeed);
    $("orbit-ap").textContent = distance(t.apoapsisAltitude);
    $("orbit-pe").textContent = distance(t.periapsisAltitude);
    $("orbit-warp").textContent = warp(t);
    $("orbit-t-ap").textContent = countdown(t.timeToApoapsis);
    $("orbit-t-pe").textContent = countdown(t.timeToPeriapsis);
    $("orbit-decel").textContent = acceleration(t.deceleration);
    $("orbit-shape").textContent = finite(t.eccentricity) || finite(t.inclination)
      ? `e ${finite(t.eccentricity) ? n(t.eccentricity).toFixed(5) : "—"} · i ${fmt(t.inclination,2,"°")}` : "—";
    setTitle("orbit-speed", `Inertial orbital speed ${speed(t.orbitalSpeed)} · surface-relative ${speed(t.trueAirSpeed)}`);
    setTitle("orbit-warp", `Rails factor ${finite(t.railsWarpFactor) ? n(t.railsWarpFactor).toFixed(0) : "—"} · physics factor ${finite(t.physicsWarpFactor) ? n(t.physicsWarpFactor).toFixed(0) : "—"}`);
    setTitle("orbit-decel", `Orbital speed rate ${finite(t.orbitalSpeedRate) ? fmtSigned(t.orbitalSpeedRate,3," m/s²") : "—"}`);

    const burnDuration = plan && finite(plan.estimatedBurnDuration) ? Math.max(0,n(plan.estimatedBurnDuration)) : null;
    const burnStartUT = plan && finite(plan.burnUT) ? n(plan.burnUT) - (burnDuration || 0) * .5 : null;
    const burnEndUT = plan && finite(plan.burnUT) ? n(plan.burnUT) + (burnDuration || 0) * .5 : null;
    const toBurn = finite(burnStartUT) && finite(t.ut) ? n(burnStartUT) - n(t.ut) : null;
    const currentThrust = finite(t.currentThrust) ? n(t.currentThrust) : finite(t.thrust) ? n(t.thrust) : 0;
    const availableThrust = finite(t.availableThrust) ? Math.max(0,n(t.availableThrust)) : 0;
    const throttle = finite(t.throttle) ? Math.max(0,Math.min(1,n(t.throttle))) : 0;
    const engineOn = currentThrust > Math.max(500, availableThrust * .005) || throttle > .02;
    const deliveredDV = finite(g.deliveredDeltaV) ? Math.max(0,n(g.deliveredDeltaV)) : 0;
    const plannedDV = plan && finite(plan.deltaV) ? Math.max(0,n(plan.deltaV)) : null;
    const remainingDV = finite(plannedDV) ? Math.max(0,n(plannedDV)-deliveredDV) : null;
    const burnComplete = g.burnCompleted === true;
    const controllerBurnStarted = g.burnStarted === true;
    const burnWindowOpen = Boolean(plan) && finite(burnStartUT) && finite(burnEndUT) && finite(t.ut) && n(t.ut) >= n(burnStartUT) && n(t.ut) <= n(burnEndUT);
    const burnWindowMissed = Boolean(plan) && !burnComplete && !controllerBurnStarted && finite(burnEndUT) && finite(t.ut) && n(t.ut) > n(burnEndUT) + 1;
    const engineState = engineOn ? "BURNING" : burnComplete ? "COMPLETE" : controllerBurnStarted ? "CUTOFF / COAST" : burnWindowMissed ? "WINDOW MISSED" : burnWindowOpen ? "WINDOW OPEN" : plan ? "ARMED" : "OFF";

    $("orbit-engine").textContent = engineState;
    $("orbit-t-burn").textContent = plan ? countdown(toBurn) : "—";
    $("orbit-burn-dv").textContent = finite(plannedDV) ? `${n(plannedDV).toFixed(1)} m/s` : "—";
    $("orbit-dv-rem").textContent = finite(remainingDV) ? `${n(remainingDV).toFixed(1)} m/s` : "—";
    $("orbit-throttle").textContent = finite(t.throttle) ? `${Math.round(throttle*100)}%` : "—";
    $("orbit-thrust").textContent = thrust(currentThrust);
    $("orbit-retro-error").textContent = fmt(retroErr,1,"°");
    $("orbit-postburn-pe").textContent = plan ? distance(plan.predictedPostBurnPeriapsisAltitude) : "—";
    setTone("orbit-engine", engineOn || burnComplete ? "good" : plan ? "warn" : "");
    setTone("orbit-retro-error", errorTone(retroErr, 3, 10));
    setTitle("orbit-t-burn", plan ? `Burn center UT ${fmt(plan.burnUT,1," s")} · duration ${duration(burnDuration)} · cutoff ${finite(burnEndUT) ? fmt(burnEndUT,1," s") : "—"}` : "No current deorbit burn plan");
    setTitle("orbit-postburn-pe", plan ? `Predicted post-burn periapsis ${distance(plan.predictedPostBurnPeriapsisAltitude)}` : "No current deorbit burn plan");

    $("coordinates").textContent = finite(t.latitude) ? `${fmtSigned(t.latitude,3,"°")}  ${fmtSigned(t.longitude,3,"°")}` : "—";
    const siteRange = greatCircleMeters(t, snapshot.site || {});
    const crossTrack = finite(t.runwayCrossTrack) ? ` · XTK ${signedDistance(t.runwayCrossTrack)}` : "";
    if (orbitMode) {
      $("range-site").textContent = `AP ${countdown(t.timeToApoapsis)} · PE ${countdown(t.timeToPeriapsis)}`;
      setTitle("range-site", `Apoapsis ${distance(t.apoapsisAltitude)} in ${duration(t.timeToApoapsis)} · periapsis ${distance(t.periapsisAltitude)} in ${duration(t.timeToPeriapsis)}`);
      $("planner-state").textContent = plan
        ? `${engineState} · ${countdown(toBurn)} · ΔV ${finite(plannedDV) ? n(plannedDV).toFixed(1) : "—"} m/s`
        : `OSC ORBIT · ${warp(t)} · ORB ${orbitalCount}`;
      setTone("planner-state", engineOn || burnComplete ? "good" : plan ? "warn" : "");
    } else {
      $("range-site").textContent = `KSC ${distance(siteRange)}${crossTrack}`;
      setTitle("range-site", `Range ${distance(siteRange)}${finite(t.bearingToSite) ? ` · bearing ${fmt(t.bearingToSite,1,"°")}` : ""}${finite(t.runwayAlongTrack) ? ` · along-track ${signedDistance(t.runwayAlongTrack)}` : ""}${finite(t.runwayCrossTrack) ? ` · cross-track ${signedDistance(t.runwayCrossTrack)}` : ""}`);
      $("planner-state").textContent = orbitalCount >= 2
        ? `${trajectoryLabel("ORB", orbitalCount)} · ${trajectoryLabel("PRED", predictedCount)} · ${trajectoryLabel("PLAN", plannedCount)}`
        : `${trajectoryLabel("PRED", predictedCount)} · ${trajectoryLabel("PLAN", plannedCount)} · ${trajectoryLabel("REF", referenceCount)}`;
      const activePathReady = terminalPhase
        ? terminalPlanProven && (referenceCount >= 2 || plannedCount >= 2)
        : entryPlanProven && plannedCount >= 2;
      const activePathPartial = terminalPhase
        ? (!terminalPlanProven && (referenceCount > 0 || plannedCount > 0)) || (terminalPlanProven && (referenceCount === 1 || plannedCount === 1))
        : (!entryPlanProven && (predictedCount > 0 || plannedCount > 0)) || (entryPlanProven && plannedCount === 1);
      setTone("planner-state", activePathReady ? "good" : activePathPartial ? "warn" : "");
    }
    setTitle("planner-state", orbitMode
      ? `Instantaneous osculating orbit ${orbitalCount} pts${orbitMeta.underThrust ? " · recomputed under thrust" : ""}${plan ? ` · deorbit burn ${countdown(toBurn)}` : ""}`
      : `Geometry source ${source} · actual ${actualCount} pts · orbit ${orbitalCount} pts · predicted ${predictedCount} pts · proven plan ${plannedCount} pts · committed reference ${referenceCount} pts · TAEM projection ${projectedCount} pts${snapshot.plannerLog ? ` · ${snapshot.plannerLog}` : ""}`);

    $("guidance-phase").textContent = orbitMode ? (/DEORBIT BURN/.test(phase) ? "DEORBIT BURN" : plan ? "ORBIT / DEORBIT" : "ORBIT") : phase;
    const terminalState = g.terminalPathComplete ? "COMPLETE" : g.terminalPathCaptured ? "CAPTURED" : g.terminalPathCommitted ? "COMMITTED" : g.terminalCandidateValid ? "CANDIDATE" : g.terminalPredictionValid ? "PREDICT" : "WAIT";
    let smartStatus;
    if (orbitMode) {
      const orbitalStatus = [];
      if (engineOn) orbitalStatus.push(`ENGINE ${thrust(currentThrust)} · ${Math.round(throttle*100)}%`);
      else orbitalStatus.push(plan ? `DEORBIT ${countdown(toBurn)}` : "ORBIT COAST");
      if (plan && finite(plannedDV)) orbitalStatus.push(`ΔV ${n(plannedDV).toFixed(1)} m/s`);
      orbitalStatus.push(`AP ${distance(t.apoapsisAltitude)} ${countdown(t.timeToApoapsis)}`);
      orbitalStatus.push(`PE ${distance(t.periapsisAltitude)} ${countdown(t.timeToPeriapsis)}`);
      if (finite(t.timeWarpRate) && n(t.timeWarpRate)>1) orbitalStatus.push(warp(t));
      smartStatus = orbitalStatus.join(" · ");
    } else {
      const statusParts = [];
      if (terminalPhase) {
        statusParts.push(terminalState);
        if (finite(t.runwayCrossTrack)) statusParts.push(`XTK ${signedDistance(t.runwayCrossTrack)}`);
        if (finite(g.terminalReferenceFPA) && finite(t.flightPathAngle)) statusParts.push(`FPA ${fmtSigned(t.flightPathAngle,1,"°")}/${fmtSigned(g.terminalReferenceFPA,1,"°")}`);
        statusParts.push(trajectoryLabel("REF", referenceCount));
      } else if (/ENTRY|MM304/.test(phase)) {
        statusParts.push(trajectoryLabel("PRED", predictedCount));
        if (typeof t.predictedTerminalPolicyFeasible === "boolean") statusParts.push(t.predictedTerminalPolicyFeasible ? "TERM OK" : "TERM UNPROVEN");
        if (finite(g.entryPlanTAEMRangeError)) statusParts.push(`ΔR ${signedDistance(g.entryPlanTAEMRangeError)}`);
        if (finite(g.entryPlanTAEMEnergyError)) statusParts.push(`ΔE ${specificEnergy(g.entryPlanTAEMEnergyError)}`);
        if (finite(t.predictedPhysicsRelativeUncertainty)) statusParts.push(`PHYS ±${(Math.abs(n(t.predictedPhysicsRelativeUncertainty)) * 100).toFixed(0)}%`);
      }
      smartStatus = statusParts.length ? statusParts.join(" · ") : (snapshot.statusMessage || "—");
    }
    $("guidance-status").textContent = smartStatus;
    setTone("guidance-status", snapshot.warningMessage ? "warn" : state === "CONNECTED" ? "good" : state === "RECONNECTING" ? "bad" : "");
    setTitle("guidance-status", [snapshot.statusMessage, snapshot.warningMessage ? `WARNING: ${snapshot.warningMessage}` : ""].filter(Boolean).join(" · "));

    let planChip = "NO PLAN", planTone = "";
    if (orbitMode) {
      if (engineOn) { planChip = "DEORBIT BURN"; planTone = "good"; }
      else if (burnComplete) { planChip = "BURN COMPLETE"; planTone = "good"; }
      else if (burnWindowMissed) { planChip = "BURN WINDOW MISSED"; planTone = "warn"; }
      else if (plan) {
        const qualified = plan.executionQualified !== false && plan.robustnessQualified !== false;
        planChip = qualified ? "DEORBIT READY" : "DEORBIT REVIEW";
        planTone = qualified ? "good" : "warn";
      } else { planChip = "ORBIT HOLD"; }
      setTitle("plan-valid", plan
        ? `Burn ${countdown(toBurn)} · ΔV ${finite(plannedDV) ? n(plannedDV).toFixed(1) : "—"} m/s · post-burn PE ${distance(plan.predictedPostBurnPeriapsisAltitude)} · retrograde error ${fmt(retroErr,1,"°")}`
        : `No current deorbit plan · AP ${distance(t.apoapsisAltitude)} · PE ${distance(t.periapsisAltitude)}`);
    } else if (terminalPhase) {
      if (terminalPlanProven && referenceCount >= 2) { planChip = "REF COMMITTED"; planTone = "good"; }
      else if (terminalPlanProven && plannedCount >= 2) { planChip = "PLAN COMMITTED"; planTone = "good"; }
      else if (g.terminalCandidateValid || g.terminalPathSelected) { planChip = "CANDIDATE · UNCOMMITTED"; planTone = "warn"; }
      else if (referenceCount === 1 || plannedCount === 1) { planChip = "PATH 1PT"; planTone = "warn"; }
      else { planChip = "REF WAIT"; planTone = "warn"; }
    } else if (entryPlanProven) {
      if (plannedCount >= 2 && predictedCount >= 2) { planChip = "PLAN PROVEN + PRED"; planTone = "good"; }
      else if (plannedCount >= 2) { planChip = "PLAN PROVEN"; planTone = "good"; }
      else if (predictedCount >= 2) { planChip = "PROVEN · PRED ONLY"; planTone = "warn"; }
      else { planChip = "PLAN PROVEN · WAIT"; planTone = "warn"; }
    } else if (g.entryPlanValid) {
      planChip = predictedCount >= 2 ? "PRED · TERMINAL UNPROVEN" : "GUIDANCE · TERMINAL UNPROVEN";
      planTone = "warn";
    } else if (predictedCount >= 2) {
      planChip = "PRED ONLY"; planTone = "warn";
    } else if (orbitalCount >= 2) {
      planChip = orbitMeta.endReason === "atmosphere-interface" ? "VAC ARC" : "OSC ORBIT"; planTone = "warn";
    }
    $("plan-valid").textContent = planChip;
    $("plan-valid").className = `chip${planTone ? ` ${planTone}` : ""}`;
    if (!orbitMode) setTitle("plan-valid", `Entry command ${g.entryPlanValid ? "valid" : "not valid"} · terminal handoff ${g.entryPlanTerminalReady ? "proven" : "unproven"} · terminal path ${terminalPlanProven ? "committed/captured" : "not committed"} · ${orbitalCount} orbital · ${predictedCount} predicted · ${plannedCount} proven plan · ${referenceCount} committed reference points${orbitalCount >= 2 ? " · orbital geometry is observer-only osculating propagation" : ""}`);

    // Entry command bars, retained for atmospheric mode.
    $("target-aoa").textContent = fmt(cmd.targetAoA,1,"°");
    $("aoa-current").textContent = fmt(t.angleOfAttack,1,"°");
    $("target-bank").textContent = fmtSigned(cmd.targetRoll,1,"°");
    $("bank-current").textContent = fmtSigned(t.roll,1,"°");
    $("target-heading").textContent = fmt(cmd.targetHeading,1,"°");
    $("heading-current").textContent = fmt(t.heading,1,"°");
    setBar("aoa-bar","aoa-now",cmd.targetAoA,t.angleOfAttack,0,30,false);
    setBar("bank-bar","bank-now",cmd.targetRoll,t.roll,-80,80,true);
    const aoaErr = finite(cmd.targetAoA) && finite(t.angleOfAttack) ? n(t.angleOfAttack) - n(cmd.targetAoA) : null;
    const bankErr = finite(cmd.targetRoll) && finite(t.roll) ? signed(n(t.roll) - n(cmd.targetRoll)) : null;
    const hdgErr = finite(cmd.targetHeading) && finite(t.heading) ? signed(n(cmd.targetHeading) - n(t.heading)) : null;
    setBar("heading-bar","heading-now",hdgErr,0,-90,90,true);
    setTone("aoa-current", errorTone(aoaErr, 1.5, 4));
    setTone("bank-current", errorTone(bankErr, 4, 10));
    setTone("heading-current", errorTone(hdgErr, 4, 12));
    setTitle("aoa-current", `Target ${fmt(cmd.targetAoA,1,"°")} · current ${fmt(t.angleOfAttack,1,"°")} · error ${fmtSigned(aoaErr,1,"°")}`);
    setTitle("bank-current", `Target ${fmtSigned(cmd.targetRoll,1,"°")} · current ${fmtSigned(t.roll,1,"°")} · error ${fmtSigned(bankErr,1,"°")}`);
    setTitle("heading-current", `Target ${fmt(cmd.targetHeading,1,"°")} · current ${fmt(t.heading,1,"°")} · error ${fmtSigned(hdgErr,1,"°")}`);

    $("segment-remaining").textContent = entryPlanProven && finite(g.entryPlanSegmentRemaining) ? `${n(g.entryPlanSegmentRemaining).toFixed(0)} s` : "—";
    if (entryPlanProven && g.entryReversalScheduled) {
      const rem = finite(g.entryReversalTimeRemaining) ? `${Math.max(0,n(g.entryReversalTimeRemaining)).toFixed(0)} s` : "QUEUED";
      $("reversal").textContent = `${g.entryReversalIsFinal ? "FINAL " : ""}${rem}`;
    } else $("reversal").textContent = entryPlanProven ? "NONE" : "—";
    setTitle("reversal", g.entryReversalScheduled ? `${g.entryReversalIsFinal ? "Final" : "Planned"} reversal${finite(g.entryReversalRange) ? ` at ${distance(g.entryReversalRange)}` : ""}${finite(g.entryReversalSign) ? ` · sign ${fmtSigned(g.entryReversalSign,0)}` : ""}` : "No reversal currently scheduled");

    const planRangeError = entryPlanProven && finite(g.entryPlanTAEMRangeError) ? g.entryPlanTAEMRangeError : t.predictedTAEMRangeError;
    const planEnergyError = entryPlanProven && finite(g.entryPlanTAEMEnergyError) ? g.entryPlanTAEMEnergyError : t.predictedTAEMEnergyError;
    $("taem-range").textContent = signedDistance(planRangeError);
    $("taem-energy").textContent = specificEnergy(planEnergyError);
    setTitle("taem-range", entryPlanProven ? `Proven plan TAEM ΔR ${signedDistance(g.entryPlanTAEMRangeError)}${finite(t.predictedTAEMRangeError) ? ` · current forecast ${signedDistance(t.predictedTAEMRangeError)}` : ""}` : `Forecast TAEM ΔR ${signedDistance(t.predictedTAEMRangeError)} · no proven entry plan`);
    setTitle("taem-energy", entryPlanProven ? `Proven plan TAEM ΔE ${specificEnergy(g.entryPlanTAEMEnergyError)}${finite(t.predictedTAEMEnergyError) ? ` · current forecast ${specificEnergy(t.predictedTAEMEnergyError)}` : ""}` : `Forecast TAEM ΔE ${specificEnergy(t.predictedTAEMEnergyError)} · no proven entry plan`);

    const hacRadius = terminalPlanProven ? (finite(g.hacRadius) && n(g.hacRadius) > 0 ? g.hacRadius : g.terminalCandidateRadius) : null;
    const terminalActive = terminalPlanProven;
    $("hac-radius").textContent = distance(hacRadius);
    $("hac-remaining").textContent = terminalActive && finite(g.terminalPathRemaining) ? distance(g.terminalPathRemaining) : "—";
    $("terminal-state").textContent = terminalState;
    $("terminal-blend").textContent = terminalPlanProven && finite(g.terminalBlend) ? `${(n(g.terminalBlend)*100).toFixed(0)}%` : "—";
    setTitle("hac-radius", `Active HAC ${distance(g.hacRadius)} · candidate ${distance(g.terminalCandidateRadius)}${finite(g.minimumTurnRadius) ? ` · minimum turn ${distance(g.minimumTurnRadius)}` : ""}`);
    setTitle("hac-remaining", terminalActive ? `Terminal path remaining ${distance(g.terminalPathRemaining)}` : "No committed/captured terminal path yet");
    setTitle("terminal-state", `${terminalState}${finite(g.terminalCandidateAltitude) ? ` · candidate altitude ${distance(g.terminalCandidateAltitude)}` : ""}${finite(g.terminalCandidateSpeed) ? ` · speed ${speed(g.terminalCandidateSpeed)}` : ""}`);
    setTitle("terminal-blend", `Terminal blend ${finite(g.terminalBlend) ? `${(n(g.terminalBlend)*100).toFixed(1)}%` : "unavailable"}${g.hacTransitionActive ? " · HAC transition active" : ""}`);
  }

  async function setScene3DVisible(visible) {
    scene3dPanel.hidden = !visible;
    scene3dToggle.classList.toggle("active", visible);
    scene3dToggle.setAttribute("aria-expanded", visible ? "true" : "false");
    if (!visible) {
      scene3dViewer?.setVisible(false);
      return;
    }
    if (!scene3dViewer) {
      if (!scene3dLoading) {
        const overlay = $("scene3d-status").parentElement;
        overlay?.classList.remove("alert");
        $("scene3d-status").textContent = "3D LOADING";
        scene3dLoading = import("/scene3d.js?v=20260915-orbit-3")
          .then((module) => module.createTelemetry3D({
            canvas: $("scene3d-canvas"),
            statusEl: $("scene3d-status"),
            metaEl: $("scene3d-meta"),
          }))
          .then((viewer) => { scene3dViewer = viewer; overlay?.classList.remove("alert"); return viewer; })
          .catch((error) => {
            $("scene3d-status").textContent = "3D UNAVAILABLE";
            $("scene3d-meta").textContent = String(error?.message || error);
            overlay?.classList.add("alert");
            scene3dLoading = null;
            throw error;
          });
      }
      try { await scene3dLoading; } catch (_) { return; }
    }
    scene3dViewer.setVisible(true);
    if (snapshot) scene3dViewer.update(snapshot);
  }

  scene3dToggle.addEventListener("click", () => setScene3DVisible(scene3dPanel.hidden));

  function render() { updateText(); drawNavball(); drawMap(); }

  function accept(data, source = "live") {
    if (source === "live") {
      latestLiveSnapshot = data;
      lastSnapshotAt = performance.now();
      reconnectDelay = 350;
      if (archiveReplay.active) return;
    }
    snapshot = data;
    render();
    if (scene3dViewer && !scene3dPanel.hidden) scene3dViewer.update(data);
  }

  async function pollRLTraining() {
    if (rlPollInFlight || archiveReplay.active) return;
    rlPollInFlight = true;
    try {
      const response = await fetch("/api/rl-training", { cache: "no-store" });
      if (!response.ok) throw new Error(`RL status ${response.status}`);
      const rl = await response.json();
      if (!snapshot) snapshot = {
        connectionStatus: "waiting", phase: "Offline", statusMessage: "Waiting for telemetry",
        telemetry: {}, command: {}, guidanceState: {}, actualTrajectory: [], predictedTrajectory: [],
        projectedTAEMTrajectory: [], plannedTrajectory: [], referenceTrajectory: [], orbitalTrajectory: [],
      };
      snapshot = { ...snapshot, rlTraining: rl };
      render();
    } catch (_) {
      // Leave the last good RL panel visible; the SSE connection handles global status.
    } finally {
      rlPollInFlight = false;
    }
  }

  const replayTime = (frame, fallback = null) => {
    const sim = frame && frame.simulation || {};
    // Archive runs can contain startup placeholder snapshots with telemetry.ut=0
    // followed by the real absolute KSP UT (~tens of thousands of seconds).
    // Prefer monotonic elapsed simulation time so playback never stalls on that jump.
    if (finite(sim.simElapsedSeconds)) return n(sim.simElapsedSeconds);
    if (finite(sim.simTime)) return n(sim.simTime);
    if (finite(frame?.sim_time)) return n(frame.sim_time);
    const t = frame && frame.telemetry || {};
    if (finite(t.ut)) return n(t.ut);
    if (finite(sim.simUT)) return n(sim.simUT);
    return fallback;
  };


  function hydrateArchiveReplayFrames(guidanceFrames, simulatorFrames) {
    if (!Array.isArray(simulatorFrames) || !simulatorFrames.length) return guidanceFrames;
    const guidance = Array.isArray(guidanceFrames) ? guidanceFrames : [];
    const timedGuidance = guidance.filter((frame) => finite(frame?.telemetry?.ut) && n(frame.telemetry.ut) > 1);
    let current = guidance[0] || {};
    let cursor = -1;
    let heldPrediction = [];
    let heldPredictionRevision = null;
    let heldPredictionKey = "";

    const predictionKey = (trajectory) => {
      if (!Array.isArray(trajectory) || !trajectory.length) return "";
      const first = trajectory[0] || {}, last = trajectory[trajectory.length - 1] || {};
      return [
        trajectory.length,
        finite(first.ut) ? n(first.ut).toFixed(3) : "",
        finite(last.ut) ? n(last.ut).toFixed(3) : "",
        finite(first.latitude) ? n(first.latitude).toFixed(5) : "",
        finite(first.longitude) ? n(first.longitude).toFixed(5) : "",
        finite(last.latitude) ? n(last.latitude).toFixed(5) : "",
        finite(last.longitude) ? n(last.longitude).toFixed(5) : "",
      ].join(":");
    };

    const result = [];
    for (const simFrame of simulatorFrames) {
      const simUt = finite(simFrame?.ut) ? n(simFrame.ut) : n(simFrame?.telemetry?.ut, NaN);
      while (
        cursor + 1 < timedGuidance.length
        && finite(simUt)
        && n(timedGuidance[cursor + 1]?.telemetry?.ut) <= simUt + .05
      ) {
        current = timedGuidance[++cursor];
        const candidate = Array.isArray(current?.predictedTrajectory) ? current.predictedTrajectory : [];
        const candidateKey = predictionKey(candidate);
        const g = current?.guidanceState || {};
        if (candidate.length && predictionHasFuture(candidate, simUt)) {
          if (candidateKey !== heldPredictionKey) {
            heldPrediction = candidate;
            heldPredictionKey = candidateKey;
            heldPredictionRevision = finite(current?.trajectoryRevision)
              ? Math.trunc(n(current.trajectoryRevision))
              : cursor;
          }
        } else if (g.terminalPredictionValid === false || !predictionHasFuture(heldPrediction, simUt)) {
          heldPrediction = [];
          heldPredictionKey = "";
          heldPredictionRevision = null;
        }
      }

      const base = current || {};
      const basePrediction = Array.isArray(base.predictedTrajectory) ? base.predictedTrajectory : [];
      const prediction = predictionHasFuture(basePrediction, simUt)
        ? basePrediction
        : predictionHasFuture(heldPrediction, simUt) ? heldPrediction : [];
      const baseCommand = base.command && typeof base.command === "object" ? base.command : {};
      const simCommand = simFrame?.command && typeof simFrame.command === "object" ? simFrame.command : {};
      const simulation = base.simulation && typeof base.simulation === "object" ? base.simulation : {};
      result.push({
        ...base,
        telemetry: {
          ...(base.telemetry && typeof base.telemetry === "object" ? base.telemetry : {}),
          ...(simFrame?.telemetry && typeof simFrame.telemetry === "object" ? simFrame.telemetry : {}),
        },
        command: Object.keys(baseCommand).length ? baseCommand : simCommand,
        predictedTrajectory: prediction,
        trajectoryRevision: prediction.length && heldPredictionRevision !== null
          ? heldPredictionRevision
          : base.trajectoryRevision,
        simulation: {
          ...simulation,
          active: true,
          sourceMode: "replay",
          simUT: finite(simUt) ? simUt : simulation.simUT,
          simTime: finite(simFrame?.simElapsedSeconds) ? n(simFrame.simElapsedSeconds) : simulation.simTime,
          simElapsedSeconds: finite(simFrame?.simElapsedSeconds) ? n(simFrame.simElapsedSeconds) : simulation.simElapsedSeconds,
          effectiveRate: finite(simFrame?.simRate) ? n(simFrame.simRate) : simulation.effectiveRate,
        },
      });
    }
    return result.length ? result : guidance;
  }
  function setPlaybackExpanded(expanded) {
    const dock = $("playback-dock");
    const open = !!expanded;
    dock.classList.toggle("collapsed", !open);
    document.body.classList.toggle("playback-expanded", open);
    const toggle = $("playback-toggle");
    toggle.setAttribute("aria-expanded", open ? "true" : "false");
    toggle.setAttribute("aria-label", open ? "Collapse playback controls" : "Expand playback controls");
  }

  function setReplayControlsActive(active) {
    const dock = $("playback-dock");
    dock.classList.toggle("replay-active", !!active);
    dock.classList.toggle("live-active", !active);
    const live = $("sim-replay-live");
    live.classList.toggle("active", !active);
    live.setAttribute("aria-pressed", active ? "false" : "true");
    for (const id of ["sim-replay-back", "sim-replay-play", "sim-replay-forward", "sim-replay-speed", "sim-replay-seek"]) {
      $(id).disabled = !active;
    }
    if (active) {
      $("playback-run-name").textContent = archiveReplay.run?.runId || "REPLAY";
    } else {
      $("playback-run-name").textContent = "LIVE TELEMETRY";
      $("sim-replay-play").textContent = "PLAY";
      $("sim-replay-seek").min = "0";
      $("sim-replay-seek").max = "0";
      $("sim-replay-seek").value = "0";
      $("sim-replay-position").textContent = "LIVE";
    }
  }

  function replayTrail(index) {
    const trail = [];
    if (!archiveReplay.frames.length) return trail;
    const stride = Math.max(1, Math.floor((index + 1) / 500));
    for (let i = 0; i <= index; i += stride) {
      const frame = archiveReplay.frames[i] || {};
      const t = frame.telemetry || {};
      if (!finite(t.latitude) || !finite(t.longitude)) continue;
      trail.push({
        ut: finite(t.ut) ? n(t.ut) : null,
        latitude: n(t.latitude), longitude: n(t.longitude),
        altitude: finite(t.meanAltitude) ? n(t.meanAltitude) : 0,
        phase: frame.phase || "REPLAY",
      });
    }
    return trail;
  }

  function replayFrameTime(index) {
    return replayTime(archiveReplay.frames[index] || {}, null);
  }

  function replayTimelineLabel(index) {
    const current = replayFrameTime(index);
    const first = replayFrameTime(0);
    const last = replayFrameTime(Math.max(0, archiveReplay.frames.length - 1));
    if (current !== null && first !== null && last !== null) {
      return `${duration(current - first)} / ${duration(last - first)}`;
    }
    return `${index + 1} / ${archiveReplay.frames.length}`;
  }

  function seekReplayBy(seconds) {
    if (!archiveReplay.active || !archiveReplay.frames.length) return;
    const current = replayFrameTime(archiveReplay.index);
    let next = archiveReplay.index + (seconds < 0 ? -1 : 1);
    if (current !== null) {
      const target = current + seconds;
      let bestDistance = Infinity;
      archiveReplay.frames.forEach((_, index) => {
        const ut = replayFrameTime(index);
        if (ut === null) return;
        const distance = Math.abs(ut - target);
        if (distance < bestDistance) { bestDistance = distance; next = index; }
      });
    } else {
      next = archiveReplay.index + Math.round(seconds);
    }
    setReplayPlaying(false);
    showReplayFrame(Math.max(0, Math.min(archiveReplay.frames.length - 1, next)));
  }

  function showReplayFrame(index) {
    if (!archiveReplay.active || !archiveReplay.frames.length) return;
    archiveReplay.index = Math.max(0, Math.min(archiveReplay.frames.length - 1, Math.trunc(index)));
    const raw = archiveReplay.frames[archiveReplay.index];
    const frame = { ...raw, actualTrajectory: replayTrail(archiveReplay.index) };
    frame.simulation = {
      ...(raw.simulation || {}),
      active: true, sourceMode: "replay",
      state: archiveReplay.playing ? "replay" : "paused",
      replayIndex: archiveReplay.index, replayCount: archiveReplay.frames.length,
      replaySpeed: archiveReplay.speed,
      runId: archiveReplay.run && archiveReplay.run.runId,
    };
    $("sim-replay-seek").max = String(Math.max(0, archiveReplay.frames.length - 1));
    $("sim-replay-seek").value = String(archiveReplay.index);
    $("sim-replay-position").textContent = replayTimelineLabel(archiveReplay.index);
    $("playback-run-name").textContent = archiveReplay.run?.runId || "REPLAY";
    $("sim-replay-play").textContent = archiveReplay.playing ? "PAUSE" : "PLAY";
    accept(frame, "replay");
  }

  function stopReplayClock() {
    if (archiveReplay.raf) cancelAnimationFrame(archiveReplay.raf);
    archiveReplay.raf = 0;
  }

  function replayTick(now) {
    if (!archiveReplay.active || !archiveReplay.playing || !archiveReplay.frames.length) return;
    const targetTime = archiveReplay.anchorUt + (now - archiveReplay.anchorWall) * 0.001 * archiveReplay.speed;
    let next = archiveReplay.index;
    while (next + 1 < archiveReplay.frames.length && replayTime(archiveReplay.frames[next + 1], targetTime) <= targetTime) next++;
    if (next !== archiveReplay.index) showReplayFrame(next);
    if (archiveReplay.index >= archiveReplay.frames.length - 1) {
      archiveReplay.playing = false;
      showReplayFrame(archiveReplay.index);
      return;
    }
    archiveReplay.raf = requestAnimationFrame(replayTick);
  }

  function setReplayPlaying(playing) {
    const requested = !!playing && archiveReplay.active && archiveReplay.frames.length > 0;
    stopReplayClock();
    if (requested && archiveReplay.index >= archiveReplay.frames.length - 1) {
      archiveReplay.index = 0;
    }
    archiveReplay.playing = requested;
    if (archiveReplay.playing) {
      archiveReplay.anchorWall = performance.now();
      archiveReplay.anchorUt = replayTime(archiveReplay.frames[archiveReplay.index], 0);
      archiveReplay.raf = requestAnimationFrame(replayTick);
    }
    if (archiveReplay.active) showReplayFrame(archiveReplay.index);
  }

  function leaveArchiveReplay() {
    setReplayPlaying(false);
    archiveReplay.active = false;
    archiveReplay.frames = [];
    archiveReplay.run = null;
    archiveReplay.index = 0;
    closeRunHistory();
    setReplayControlsActive(false);
    setPlaybackExpanded(false);
    document.querySelectorAll(".sim-run-row.active").forEach((el) => el.classList.remove("active"));
    if (latestLiveSnapshot) accept(latestLiveSnapshot, "live");
    else render();
  }

  async function startArchiveReplay(runId, row) {
    setPlaybackExpanded(true);
    $("playback-run-name").textContent = runId || "LOADING REPLAY";
    $("sim-replay-position").textContent = "LOADING…";
    const response = await fetch("/api/sim-replay?id=" + encodeURIComponent(runId) + "&maxFrames=1600", {cache:"no-store"});
    if (!response.ok) throw new Error("Replay HTTP " + String(response.status));
    const data = await response.json();
    const guidanceFrames = Array.isArray(data.frames) ? data.frames : [];
    const simulatorFrames = Array.isArray(data.simulatorFrames) ? data.simulatorFrames : [];
    const frames = hydrateArchiveReplayFrames(guidanceFrames, simulatorFrames);
    if (!frames.length) throw new Error("Run has no replay frames");
    archiveReplay.active = true;
    archiveReplay.frames = frames;
    archiveReplay.run = data.run || {runId: runId};
    archiveReplay.index = 0;
    archiveReplay.speed = Math.max(.1, n($("sim-replay-speed").value, 20));
    setReplayControlsActive(true);
    document.querySelectorAll(".sim-run-row.active").forEach((el) => el.classList.remove("active"));
    if (row) row.classList.add("active");
    closeRunHistory();
    showReplayFrame(0);
    setReplayPlaying(true);
  }

  const runDistance = (value) => {
    if (!finite(value)) return "—";
    return Math.abs(n(value)) >= 1000 ? (n(value) / 1000).toFixed(1) + "k" : n(value).toFixed(0) + "m";
  };

  function renderRunHistory(runs) {
    const list = $("sim-history-list");
    $("sim-history-count").textContent = String(runs.length) + " RUNS";
    list.replaceChildren();
    if (!runs.length) {
      const empty = document.createElement("div");
      empty.className = "sim-history-empty";
      empty.textContent = "No archived simulator campaigns yet.";
      list.appendChild(empty);
      return;
    }
    for (const run of runs) {
      const row = document.createElement("button");
      row.type = "button";
      row.className = "sim-run-row";
      if (archiveReplay.active && archiveReplay.run?.runId === run.runId) row.classList.add("active");
      row.disabled = !run.hasReplay;
      const dot = document.createElement("span");
      dot.className = "sim-run-dot " + (run.success ? "success" : run.state === "failed" ? "failed" : "");
      const main = document.createElement("span");
      main.className = "sim-run-main";
      const name = document.createElement("span");
      name.className = "sim-run-name";
      name.textContent = run.runId || "unnamed";
      const meta = document.createElement("span");
      meta.className = "sim-run-meta";
      const scenario = String(run.scenario || "").split("/").pop();
      const parts = [run.terminalPhase || run.state || "unknown", scenario];
      if (finite(run.wallSeconds)) parts.push(n(run.wallSeconds).toFixed(1) + "s wall");
      meta.textContent = parts.filter(Boolean).join(" · ");
      main.append(name, meta);
      const final = document.createElement("span");
      final.className = "sim-run-final";
      const f = run.final || {};
      final.textContent = "ALT " + runDistance(f.altitude) + " · A " + runDistance(f.runwayAlongTrack) + " · X " + runDistance(f.runwayCrossTrack);
      row.append(dot, main, final);
      if (run.hasReplay) {
        row.addEventListener("click", () => startArchiveReplay(run.runId, row).catch((error) => {
          $("sim-replay-position").textContent = "Unable to load run";
          $("playback-run-name").textContent = archiveReplay.active ? (archiveReplay.run?.runId || "REPLAY") : "LIVE TELEMETRY";
          if (!archiveReplay.active) setReplayControlsActive(false);
          setReplayPlaying(false);
        }));
      }
      list.appendChild(row);
    }
  }

  async function refreshRunHistory() {
    const response = await fetch("/api/sim-runs", {cache:"no-store"});
    if (!response.ok) throw new Error("History HTTP " + String(response.status));
    const data = await response.json();
    renderRunHistory(Array.isArray(data.runs) ? data.runs : []);
  }

  function openRunHistory() {
    setPlaybackExpanded(true);
    $("sim-history-panel").hidden = false;
    $("sim-history-open").classList.add("active");
    refreshRunHistory().catch((error) => {
      const list = $("sim-history-list");
      list.replaceChildren();
      const empty = document.createElement("div");
      empty.className = "sim-history-empty";
      empty.textContent = String(error.message || error);
      list.appendChild(empty);
    });
  }

  function closeRunHistory() {
    $("sim-history-panel").hidden = true;
    $("sim-history-open").classList.remove("active");
  }

  function connect() {
    clearTimeout(reconnectTimer);
    if (eventSource) eventSource.close();
    eventSource = new EventSource("/events");
    eventSource.addEventListener("snapshot", (event) => {
      try { accept(JSON.parse(event.data), "live"); } catch (_) {}
    });
    eventSource.onopen = () => { reconnectDelay = 350; };
    eventSource.onerror = () => {
      eventSource.close(); eventSource = null;
      if (snapshot) {
        snapshot = { ...snapshot, connectionStatus: snapshot.connectionStatus === "connected" ? "reconnecting" : snapshot.connectionStatus };
        render();
      }
      reconnectTimer = setTimeout(connect, reconnectDelay);
      reconnectDelay = Math.min(5000, reconnectDelay * 1.6);
    };
  }

  map.addEventListener("wheel", (event) => {
    event.preventDefault();
    mapView.autoFocus = false;
    const point = mapEventPoint(event);
    zoomMapAt(point.x, point.y, mapView.zoom * Math.exp(-event.deltaY * 0.0022));
  }, { passive: false });

  map.addEventListener("dblclick", (event) => {
    event.preventDefault();
    resetMapView();
  });

  map.addEventListener("pointerdown", (event) => {
    if (event.pointerType === "mouse" && event.button !== 0) return;
    event.preventDefault();
    mapView.autoFocus = false;
    map.setPointerCapture(event.pointerId);
    const point = mapEventPoint(event);
    mapPointers.set(event.pointerId, point);
    map.style.cursor = "grabbing";
    if (mapPointers.size === 1) {
      mapDragPoint = point;
      pinchDistance = 0;
      pinchCenter = null;
    } else if (mapPointers.size === 2) {
      const [a, b] = [...mapPointers.values()];
      pinchDistance = Math.hypot(b.x - a.x, b.y - a.y);
      pinchCenter = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      mapDragPoint = null;
    }
  });

  map.addEventListener("pointermove", (event) => {
    if (!mapPointers.has(event.pointerId)) return;
    event.preventDefault();
    const point = mapEventPoint(event);
    mapPointers.set(event.pointerId, point);
    if (mapPointers.size >= 2) {
      const [a, b] = [...mapPointers.values()];
      const distance = Math.hypot(b.x - a.x, b.y - a.y);
      const center = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      if (pinchCenter) {
        mapView.offsetX += center.x - pinchCenter.x;
        mapView.offsetY += center.y - pinchCenter.y;
        clampMapView();
      }
      if (pinchDistance > 0) zoomMapAt(center.x, center.y, mapView.zoom * distance / pinchDistance);
      else drawMap();
      pinchDistance = distance;
      pinchCenter = center;
      return;
    }
    if (mapDragPoint) {
      mapView.offsetX += point.x - mapDragPoint.x;
      mapView.offsetY += point.y - mapDragPoint.y;
      clampMapView();
      mapDragPoint = point;
      drawMap();
    }
  });

  function releaseMapPointer(event) {
    mapPointers.delete(event.pointerId);
    if (mapPointers.size === 1) {
      mapDragPoint = [...mapPointers.values()][0];
      pinchDistance = 0;
      pinchCenter = null;
    } else if (mapPointers.size === 0) {
      mapDragPoint = null;
      pinchDistance = 0;
      pinchCenter = null;
      map.style.cursor = "grab";
    }
  }
  map.addEventListener("pointerup", releaseMapPointer);
  map.addEventListener("pointercancel", releaseMapPointer);

  setPlaybackExpanded(false);
  setReplayControlsActive(false);
  $("sim-history-open").addEventListener("click", () => $("sim-history-panel").hidden ? openRunHistory() : closeRunHistory());
  $("sim-history-close").addEventListener("click", closeRunHistory);
  $("playback-toggle").addEventListener("click", () => {
    const expand = $("playback-dock").classList.contains("collapsed");
    if (!expand) closeRunHistory();
    setPlaybackExpanded(expand);
  });
  $("sim-replay-back").addEventListener("click", () => seekReplayBy(-10));
  $("sim-replay-play").addEventListener("click", () => setReplayPlaying(!archiveReplay.playing));
  $("sim-replay-forward").addEventListener("click", () => seekReplayBy(10));
  $("sim-replay-live").addEventListener("click", leaveArchiveReplay);
  $("sim-replay-seek").addEventListener("input", (event) => {
    if (!archiveReplay.active) return;
    const targetIndex = Math.max(0, Math.min(
      archiveReplay.frames.length - 1,
      Math.trunc(n(event.target.value))
    ));
    setReplayPlaying(false);
    showReplayFrame(targetIndex);
  });
  document.addEventListener("keydown", (event) => {
    if (!archiveReplay.active || event.defaultPrevented) return;
    const tag = document.activeElement?.tagName;
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA" || tag === "BUTTON") return;
    if (event.code === "Space") {
      event.preventDefault();
      setReplayPlaying(!archiveReplay.playing);
    } else if (event.key === "ArrowLeft") {
      event.preventDefault();
      seekReplayBy(-10);
    } else if (event.key === "ArrowRight") {
      event.preventDefault();
      seekReplayBy(10);
    }
  });

  $("sim-replay-speed").addEventListener("change", (event) => {
    archiveReplay.speed = Math.max(.1, n(event.target.value, 20));
    if (archiveReplay.playing) setReplayPlaying(true);
    else if (archiveReplay.active) showReplayFrame(archiveReplay.index);
  });
  const resizeObserver = new ResizeObserver(render);
  resizeObserver.observe(nav); resizeObserver.observe(map);
  window.addEventListener("resize", () => { clampMapView(); drawMap(); });
  kerbin.addEventListener("load", drawMap);
  window.addEventListener("visibilitychange", () => {
    if (!document.hidden && !eventSource) connect();
    if (scene3dViewer) scene3dViewer.setVisible(!document.hidden && !scene3dPanel.hidden);
  });
  setInterval(() => {
    if (lastSnapshotAt && performance.now() - lastSnapshotAt > 4000 && eventSource) {
      eventSource.close(); eventSource = null; connect();
    }
  }, 2000);
  connect();
  pollRLTraining();
  setInterval(pollRLTraining, 5000);
  render();
})();
